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
#include "Trackball.h"
#include "Lighting.h"
#include "Geometry.h"
#include "Color.hpp"
#include "simScene.hpp"
#include "simCamera.hpp"
#include "ukf.hpp"


const int windowWidth=800;
const int windowHeight=800;

typedef enum {LINE_MODE, SURFACE_MODE} RENDER_OPTION;
RENDER_OPTION current_render_option=SURFACE_MODE;

double eyeZ = 600.0;

CTrackball trackball;
CLighting lighting;

Color color;

SimScene simScene;
SimCamera simCamera(simScene);
ukf filter(simCamera);

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
            play = !play;
            break;
        case 'r':
        case'R':
            simCamera.reset();
            filter.reset(simCamera);
            break;
        default: trackball.tbKeyboard(key);
    }
    glutPostRedisplay();
}

void display(void)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
//    gluPerspective(33,1,2, scene.zFar);
    gluPerspective(33,1,2, 1000);

    glMatrixMode( GL_MODELVIEW );	// Set the view matrix ...
    glLoadIdentity();			//... to identity.
    gluLookAt(0,0,eyeZ, 0,0,0, 0,1,0);                   // camera is on the z-axis
    trackball.tbMatrix();			// rotate scene using the trackball

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    simScene.draw();
    simCamera.draw();
    filter.draw();
    
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

void idle()
{
    if (play)
    {
        simCamera.timeStep();
        filter.step(simCamera.defaultTimeStep, simCamera.measure(simScene));
        glutPostRedisplay();
    }
}

// create a double buffered colour window
int main(int argc, char** argv)
{
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
}