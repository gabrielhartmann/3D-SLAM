// A program to display a surface of revolution
//
// Use 'l' to render the mesh
// Use 's' to render the surface
// Use 'e' to enable backface culling
// Use 'd' to disable backface culling

#include <stdio.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <vector>
#include "Trackball.h"
#include "Lighting.h"
#include "Geometry.h"
#include "Color.hpp"
#include "simScene.hpp"
#include "Device.hpp"
#include "ukf.hpp"
#include "Utilities.h"

string fileName;

std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > deviceStates;
std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > filterStates;

const int windowWidth=800;
const int windowHeight=800;

typedef enum {LINE_MODE, SURFACE_MODE} RENDER_OPTION;
RENDER_OPTION current_render_option=SURFACE_MODE;

double eyeZ = 600.0;
int stepIndex = 0;

CTrackball trackball;
CLighting lighting;

Color color;

SimScene simScene;
Device simCamera(simScene);
UKF filter(simCamera, simScene);

bool play = false;

double defaultTimeStep = 0.05;

void handleWheel(double zoom);

void handleMouseMotion(int x, int y)
{	
	trackball.tbMotion(x, y);
}

void handleMouseClick(int button, int state, int x, int y)
{
    
    
    if (button == 3) // Wheel up
    {
        handleWheel(-2.0);
    }
    if (button == 4) // Wheel down
    {
        handleWheel(2.0);
    }

       trackball.tbMouse(button, state, x, y); 
       
}

void handleWheel(double zoom)
{
    //scene.zoom(zoom);
    eyeZ += zoom;
     glutPostRedisplay();
}

void handleKeyboardEvent(unsigned char key, int x, int y)
{
    Eigen::Vector3d tmpControl;
    
    switch (key)
    {
        case 'l': 
           current_render_option=LINE_MODE; 
           break;
        case 's':
            current_render_option=SURFACE_MODE; 
            break;
        case 'e':
            glCullFace(GL_BACK);
            glEnable(GL_CULL_FACE);            
            break;
        case 'd':
            glDisable(GL_CULL_FACE);
            break;
        case ' ':
            if (play)
            {
                play = false;
            }
            else{
                simCamera.timeStep();
                filter.step(simCamera.defaultTimeStep, simCamera.control(), simCamera.measure());
            }
            break;
        case 'p':
        case 'P':
            play = !play;
            break;
        default: trackball.tbKeyboard(key);
    }
    glutPostRedisplay();
}

void handleKeyboardReplay(unsigned char key, int x, int y)
{     
    switch (key)
    {
        case 'l': 
           current_render_option=LINE_MODE; 
           break;
        case 's':
            current_render_option=SURFACE_MODE; 
            break;
        case 'e':
            glCullFace(GL_BACK);
            glEnable(GL_CULL_FACE);            
            break;
        case 'd':
            glDisable(GL_CULL_FACE);
            break;
        case ' ':
            if (stepIndex +1 < deviceStates.size()-2)
            {
                stepIndex++;
            }
            break;
        case '=':
            if (stepIndex +1 < deviceStates.size()-2)
            {
                stepIndex++;
            }
            break;
        case '-':
            if (stepIndex - 1 >= 0)
            {
                stepIndex--;
            }
            break;
        default:
            trackball.tbKeyboard(key);
    }
    glutPostRedisplay();
}

void display(void)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
//    gluPerspective(33,1,2, scene.zFar);
    gluPerspective(33,1,2, 1000 + eyeZ);

    glMatrixMode( GL_MODELVIEW );	// Set the view matrix ...
    glLoadIdentity();			//... to identity.
    gluLookAt(0,0,eyeZ, 0,0,0, 0,1,0);                   // camera is on the z-axis
    trackball.tbMatrix();			// rotate scene using the trackball

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    //simScene.draw();
    simCamera.draw();
    filter.draw();
    
    glFlush ();
    glutSwapBuffers();
}

void displayReplay(void)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(33,1,2, 1000 + eyeZ);

    glMatrixMode( GL_MODELVIEW );	// Set the view matrix ...
    glLoadIdentity();			//... to identity.
    gluLookAt(0,0,eyeZ, 0,0,0, 0,1,0);                   // camera is on the z-axis
    trackball.tbMatrix();			// rotate scene using the trackball

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Draw axes
    simScene.drawAxes();
    
    Eigen::VectorXd state;
    Eigen::Vector3d imuPos;
    Eigen::Vector3d camPos;
    double focalLength = 4.0;
    
    // Draw filter state
    state = filterStates[stepIndex];
    imuPos[0] = state[0];
    imuPos[1] = state[1];
    imuPos[2] = state[2];
    Eigen::Quaterniond filterImuDir(state[3], state[4], state[5], state[6]);
    camPos[0] = state[7];
    camPos[1] = state[8];
    camPos[2] = state[9];
    Eigen::Quaterniond filterCamDir(state[10], state[11], state[12], state[13]);
    drawCamera(camPos, filterCamDir, focalLength, 0.0, 0.0, 0.8);
    drawImu(imuPos, filterImuDir, 4.0, 0.0, 0.0, 0.8);
    Color::setColor(0.0, 0.0, 0.8); // Blue
    for (int lmIndex = 14; lmIndex < state.rows(); lmIndex += 3)
    {
        glPushMatrix();
        glTranslated(state[lmIndex], state[lmIndex+1], state[lmIndex+2]);
        glutSolidCube(4.0);
        glPopMatrix();
    }
    
    //Draw filter history line
    Eigen::Vector3d camPos2;
    Color::setColor(0.0, 0.8, 0.8); // cyan
    glLineWidth(4.0);
    for (int i=0; i<stepIndex-1; i++)
    {
        state = filterStates[i];
        camPos[0] = state[7];
        camPos[1] = state[8];
        camPos[2] = state[9];
        
        state = filterStates[i+1];
        camPos2[0] = state[7];
        camPos2[1] = state[8];
        camPos2[2] = state[9];  
        
        glBegin(GL_LINES);
        glVertex3d(camPos2.x(), camPos2.y(), camPos2.z());
        glVertex3d(camPos.x(), camPos.y(), camPos.z());
        glEnd();
    }
    
    // Draw device state which includes landmarks
    state = deviceStates[stepIndex];
    imuPos[0] = state[0];
    imuPos[1] = state[1];
    imuPos[2] = state[2];
    Eigen::Quaterniond deviceImuDir(state[3], state[4], state[5], state[6]);
    camPos[0] = state[7];
    camPos[1] = state[8];
    camPos[2] = state[9];
    Eigen::Quaterniond deviceCamDir(state[10], state[11], state[12], state[13]);
    drawCamera(camPos, deviceCamDir, focalLength, 0.8, 0.0, 0.0);
    drawImu(imuPos, deviceImuDir, 4.0, 0.8, 0.0, 0.0);
    
    for (int lmIndex = 14; lmIndex < state.rows(); lmIndex += 3)
    {
        Eigen::Vector3d lm;
        lm[0] = state[lmIndex];
        lm[1] = state[lmIndex+1];
        lm[2] = state[lmIndex+2];
        if (visible(camPos, deviceCamDir, simCamera.fov, lm))
        {
            Color::setColor(0.0, 0.8, 0.0); // Green
        }
        else{
            Color::setColor(0.8, 0.0, 0.0); // Red
        }
        glPushMatrix();
        glTranslated(lm.x(), lm.y(), lm.z());
        glutSolidCube(4.0);
        glPopMatrix();
    }
    
     //Draw device history line
    Color::setColor(0.8, 0.0, 0.8); // magenta
    glLineWidth(4.0);
    for (int i=0; i<stepIndex-1; i++)
    {
        state = deviceStates[i];
        camPos[0] = state[7];
        camPos[1] = state[8];
        camPos[2] = state[9];
        
        state = deviceStates[i+1];
        camPos2[0] = state[7];
        camPos2[1] = state[8];
        camPos2[2] = state[9];  
        
        glBegin(GL_LINES);
        glVertex3d(camPos2.x(), camPos2.y(), camPos2.z());
        glVertex3d(camPos.x(), camPos.y(), camPos.z());
        glEnd();
    }
    
    glFlush ();
    glutSwapBuffers();
}

void init(void) 
{
    // select clearing color (for glClear)
    glClearColor (0.4 , 0.4, 0.45, 0);	// RGB-value for grey-blue
    // enable depth buffering
    glEnable(GL_DEPTH_TEST);
    // initialize view (simple orthographic projection)
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    trackball.tbInit(GLUT_LEFT_BUTTON);
   
//    scene.init();

    // enable shading and lighting
    lighting.enable();
    glShadeModel(GL_SMOOTH);
}

void reshape(int width, int height ) {
    // Called at start, and whenever user resizes component
    int size = min(width, height);
    glViewport(0, 0, size, size);  // Largest possible square
    trackball.tbReshape(width, height);
}

string vector2string(Eigen::VectorXd vec)
{
    std::ostringstream s;
    
    for (int i=0; i<vec.rows(); i++)
    {
        if (i != vec.rows()-1){
            s << vec[i] << " ";
        }
        else{
            s << vec[i] <<"\n";
        }
    }
    
    return s.str();
}

void write(Eigen::VectorXd state)
{
    std::ofstream outFile;
    outFile.open(fileName.c_str(), std::ios_base::app);
    outFile << vector2string(state);
    outFile.close();
}

void idle()
{
    if (play)
    {
        simCamera.timeStep();
        filter.step(simCamera.defaultTimeStep, simCamera.control(), simCamera.measure());
        write(simCamera.getState());
        write(filter.getState());
        glutPostRedisplay();
    }
}
Eigen::VectorXd line2Vector(string line)
{
    istringstream iss(line);
    string word;
    vector<string> words;
    while (std::getline(iss, word, ' '))
    {
        words.push_back(word);
    }
    
    Eigen::VectorXd v;
    v.resize(words.size());
    for (int i=0; i<words.size(); i++)
    {
        v[i] = atof(words[i].c_str());
    }
    
    return v;
}

void readReplay(string fName)
{
    string line;
    ifstream replayFile(fName.c_str());
    if (replayFile.is_open())
    {
        while (replayFile.good())
        {
            getline(replayFile, line);
            deviceStates.push_back(line2Vector(line));
            getline(replayFile, line);
            filterStates.push_back(line2Vector(line));
        }
    }
}

// create a double buffered colour window
int main(int argc, char** argv)
{
    if(argc == 1) // Record
    {
        time_t rawtime;
        time(&rawtime);
        string time = ctime(&rawtime);
        fileName = "Recording " + time;
        write(simCamera.getState());
        write(filter.getState());

        glutInit(&argc, argv);		
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
        glutInitWindowSize(windowWidth, windowHeight); 
        glutInitWindowPosition(1000, 100);
        glutCreateWindow("UKF SLAM");
        init ();								// initialise view
        glutMouseFunc(handleMouseClick);		// Set function to handle mouse clicks
        glutMotionFunc(handleMouseMotion);		// Set function to handle mouse motion
        glutKeyboardFunc(handleKeyboardEvent);	// Set function to handle keyboard input
        glutDisplayFunc(display);		// Set function to draw scene
        glutReshapeFunc(reshape);		// Set function called if window gets resized
        glutIdleFunc(idle);
        glutMainLoop();
        return 0;
    }else if(argc == 2) // Replay
    {
        string fName = argv[1];
        printf("FileName = %s\n", fName.c_str());
        readReplay(fName);
        glutInit(&argc, argv);		
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
        glutInitWindowSize(windowWidth, windowHeight); 
        glutInitWindowPosition(1000, 100);
        glutCreateWindow("UKF SLAM");
        init ();								// initialise view
        glutMouseFunc(handleMouseClick);		// Set function to handle mouse clicks
        glutMotionFunc(handleMouseMotion);		// Set function to handle mouse motion
        glutKeyboardFunc(handleKeyboardReplay);	// Set function to handle keyboard input
        glutDisplayFunc(displayReplay);		// Set function to draw scene
        glutReshapeFunc(reshape);		// Set function called if window gets resized
        glutMainLoop();
        return 0;
    }
}