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

string fileNameRecording;

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
    printf("Step %d\n", stepIndex);
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
    static int historyLength = 500;
    
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
    for (int lmIndex = 14; lmIndex < state.rows(); lmIndex += 4)
    {
        glPushMatrix();
        glTranslated(state[lmIndex+1], state[lmIndex+2], state[lmIndex+3]);
        glutSolidCube(cubeWidth);
        glPopMatrix();
    }
    
    //Draw filter history line
    Eigen::Vector3d camPos2;
    Color::setColor(0.0, 0.8, 0.8); // cyan
    glLineWidth(4.0);
    int i;
    if (stepIndex < historyLength)
    {
        i=0;
    }
    else{
        i = stepIndex - historyLength;
    }
    for (; i<stepIndex-1; i++)
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
        glutSolidCube(cubeWidth);
        glPopMatrix();
    }
    
     //Draw device history line
    Color::setColor(0.8, 0.0, 0.8); // magenta
    glLineWidth(4.0);
    if (stepIndex < historyLength)
    {
        i=0;
    }
    else{
        i = stepIndex - historyLength;
    }
    for (; i<stepIndex-1; i++)
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

void write(Eigen::VectorXd state, string fileName)
{
    std::ofstream outFile;
    outFile.open(fileName.c_str(), std::ios_base::app);
    outFile << vector2string(state);
    outFile.close();
}

double getError(Eigen::VectorXd filter, Eigen::VectorXd world)
{
    Eigen::Vector3d f_imuPos;
    f_imuPos[0] = filter[0];
    f_imuPos[1] = filter[1];
    f_imuPos[2] = filter[2];
    Eigen::Vector3d f_camPos;
    f_camPos[0] = filter[7];
    f_camPos[1] = filter[8];
    f_camPos[2] = filter[9];
    
    //printf("Filter imu = (%f, %f, %f)\n Filter cam = (%f, %f, %f)\n", f_imuPos[0], f_imuPos[1], f_imuPos[2], f_camPos[0], f_camPos[1], f_camPos[2]);
    
    Eigen::Vector3d w_imuPos;
    w_imuPos[0] = world[0];
    w_imuPos[1] = world[1];
    w_imuPos[2] = filter[2];
    Eigen::Vector3d w_camPos;
    w_camPos[0] = world[7];
    w_camPos[1] = world[8];
    w_camPos[2] = world[9];
    
    //printf("World imu = (%f, %f, %f)\n World cam = (%f, %f, %f)\n", w_imuPos[0], w_imuPos[1], w_imuPos[2], w_camPos[0], w_camPos[1], w_camPos[2]);
    
    Eigen::Vector3d imuDiff;
    imuDiff = f_imuPos - w_imuPos;
    double imuError = imuDiff.adjoint() * imuDiff;
    imuError = sqrt(imuError);
    
    Eigen::Vector3d camDiff;
    camDiff = f_camPos - w_camPos;
    double camError = camDiff.adjoint() * camDiff;
    camError =sqrt(camError);
    double lmError = 0;
    
    // Construct world map of landmarks
    std::map<int, std::vector<double> >lms;
    std::vector<double> lm;
    int numLandmarksInWorld = (world.rows() - 14) / 3;
    for (int i=0; i<numLandmarksInWorld; i++)
    {
        int index = 14 + i*3;
        lm.clear();
        lm.push_back(world[index]);
        lm.push_back(world[index + 1]);
        lm.push_back(world[index + 2]);
        lms[i] = lm;
    }
    
    
    int numLandmarksInFilter = (filter.rows() - 14) / 4;
    for (int i=0; i<numLandmarksInFilter; i++)
    {
        int index = 14 + i*4;
        
        int tag = (int)filter[index];
        
        Eigen::Vector3d f_LM;
        f_LM[0] = filter[index+1];
        f_LM[1] = filter[index+2];
        f_LM[2] = filter[index+3];
        
        Eigen::Vector3d w_LM;
        w_LM[0] = lms[tag][0];
        w_LM[1] = lms[tag][1];
        w_LM[2] = lms[tag][2];
        
        Eigen::Vector3d lmDiff;
        lmDiff = f_LM - w_LM;
        double e = lmDiff.adjoint() * lmDiff;
        lmError += sqrt(e);
    }
    
    return imuError + camError + lmError;
}



void idle()
{
    if (play)
    {
        simCamera.timeStep();
        filter.step(simCamera.defaultTimeStep, simCamera.control(), simCamera.measure());
        write(simCamera.getState(), fileNameRecording);
        write(filter.getState(), fileNameRecording);
        glutPostRedisplay();
    }
}

void computeError(string fName)
{
    string fileNameError = fName + " Error";
    Eigen::VectorXd e;
    e.resize(1);
    for (int i=0; i<deviceStates.size(); i++)
    {
        double error = getError(filterStates[i], deviceStates[i]);
        e[0] = error;
        write(e, fileNameError);
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
    
    deviceStates.pop_back();
    filterStates.pop_back();
    
    //computeError(fName);
}

// create a double buffered colour window
int main(int argc, char** argv)
{
    if(argc == 1) // Record
    {
        time_t rawtime;
        time(&rawtime);
        string time = ctime(&rawtime);
        fileNameRecording = time + "Recording ";
        write(simCamera.getState(), fileNameRecording);
        write(filter.getState(), fileNameRecording);

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