/* 
 * File:   main.cpp
 * Author: gabe
 *
 * Created on 26 March 2012, 4:16 PM
 */
/*
#include <stdio.h>
#include "io.hpp"
#include "grid_features.hpp"
#include "ukf.hpp"

int main (int argc, const char * argv[])
{
    ukf* filter = new ukf();
    
    std::vector<std::string> test;
    
    ImageReader* imgReader = new ImageReader("/home/gabe/sequences/grid sequence/frames/");
    std::string imgWindowName = "Frame";

    GridFeatures* featureFinder = new GridFeatures(8,6);

    for (int i=0; i<imgReader->numImages; i++)
    {
            cv::Mat img = imgReader->getNextImage();
            std::vector<cv::Point2f> corners = featureFinder->getFeatures(img);

            cv::imshow(imgWindowName, img);
            cv::waitKey(0);
    }

} 
 */

//#include "ukf.hpp"
#include "GL/freeglut.h"
#include "GL/gl.h"
#include "simCamera.hpp"
#include "simScene.hpp"
#include "ukf.hpp"
#include <stdio.h>

void drawCamera(Eigen::Vector3d position, double r, double g, double b);
void drawAxes();
void drawLandmarks();
void drawLandmark(double x, double y, double sideLength, double r, double g, double b);

namespace
{
    const float DEG2RAD = 3.14159/180;
    double areaWidth = 200;
    double areaHeight = 100;
    SimScene simScene;
    SimCamera simCamera(simScene);
    ukf filter(simCamera);
}

void drawEllipse(float xradius, float yradius)
{
   glBegin(GL_LINE_LOOP);
 
   for (int i=0; i<360; i++)
   {
      //convert degrees into radians
      float degInRad = i*DEG2RAD;
      glVertex2f(cos(degInRad)*xradius,sin(degInRad)*yradius);
   }
 
   glEnd();
}

void renderFunction()
{
    float grey = 241.0;
    glClearColor(grey/255.0, grey/255.0, grey/255.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    glOrtho(-20.0, areaWidth+25.0, -20.0, areaHeight+25.0, -1.0, 1.0);
    
    drawAxes();
    drawCamera(simCamera.position(), 1.0, 0.0, 0.0);
    drawCamera(filter.position(), 0.0, 0.0, 1.0);
    printf("\n");
    drawLandmarks();

    glutSwapBuffers();
    glFlush();
}

void drawLandmarks()
{
    for (int i=0; i<simScene.landmarks.size(); i++)
    {
        Eigen::Vector3d landmark = simScene.landmarks.at(i);
        drawLandmark(landmark(0,0), landmark(1,0), 2.0, 0.0, 1.0, 0.0);
    }
    
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > lms = filter.landmarks();
    for (int i=0; i<lms.size(); i++)
    {
        drawLandmark(lms.at(i)(0,0), lms.at(i)(1,0), 2.0, 0.0, 0.0, 1.0);
    }
}

void drawCamera(Eigen::Vector3d position, double r, double g, double b)
{
    //Eigen::Vector3d position = simCamera.position();
    
    glColor3f(r, g, b);
    glPushMatrix();
    glBegin(GL_POLYGON);
    glVertex2d(position(0,0), position(1,0));
    glVertex2d(position(0,0)+1.0, position(1,0)+1.0);
    glVertex2d(position(0,0)+1.0, position(1,0)-1.0);
    glEnd();
    glPopMatrix();
}

void drawAxes()
{
    glColor3f(0.0, 0.0, 0.0);
    glBegin(GL_LINES); //x
    glVertex2d(0.0, 0.0);
    glVertex2d(areaWidth, 0.0);
    glEnd();
    
    glBegin(GL_LINES); //y
    glVertex2d(0.0, 0.0);
    glVertex2d(0.0, areaHeight);
    glEnd();
    
    double hashSpace = 10.0;
    double hashLength = 3.0;
    
    for (int i=0; i<areaWidth; i+=hashSpace)
    {
        glBegin(GL_LINES); // x axis hashes
        glVertex2d(i, -1.0 * hashLength / 2.0);
        glVertex2d(i,        hashLength / 2.0);
        glEnd();
    }
    
    for (int i=0; i<areaHeight; i+=hashSpace)
    {
        glBegin(GL_LINES); // y axis hashes
        glVertex2d(-1.0 * hashLength / 2.0, i);
        glVertex2d(       hashLength / 2.0, i);
        glEnd();
    }
}

void drawLandmark(double x, double y, double sideLength, double r, double g, double b)
{
    glColor3f(r, g, b);
    
    glPushMatrix();
    
    glTranslated(x, y, 0.0);
    glRotated(45.0, 0.0, 0.0, 1.0);
       
    glBegin(GL_QUADS);
    glVertex2d(-1.0 * sideLength/2.0, -1.0 * sideLength/2.0);
    glVertex2d(-1.0 * sideLength/2.0,        sideLength/2.0);
    glVertex2d(       sideLength/2.0,        sideLength/2.0);
    glVertex2d(       sideLength/2.0, -1.0 * sideLength/2.0);
    glEnd();
    
    glPopMatrix();
}

void keyboardListener(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 27: // Esc
            exit(0);
            break;
        case 32: // Space
            simCamera.timeStep();
            filter.step(simCamera.defaultTimeStep, simCamera.measure(simScene));
            break;
        case 82: //R
        case 114://r
            simCamera.reset();
            filter.reset(simCamera);
            break;
    }
    glutPostRedisplay();
}

int main(int argc, char** argv)
{    
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE);
    glutInitWindowSize(1000,500);
    glutInitWindowPosition(100,100);
    glutCreateWindow("OpenGL - First window demo");
    glutDisplayFunc(renderFunction);
    // glutIdleFunc(renderFunction);
    glutKeyboardFunc(keyboardListener);
    glutMainLoop();    
    return 0;
}