#include "simScene.hpp"
#include "Utilities.h"

SimScene::SimScene()
{
    
    //landmarks.push_back(Eigen::Vector3d(100.0, 0.0, 60.0));

    // CUBE
    int width, height, depth;
    double space = 70;
    width = 3;
    height = 3;
    depth = 3;

    Eigen::Vector3d origin;
    origin << 200.0, -space * (height - 1) / 2.0 + 10.0, -space * (depth - 1) / 2.0;
    
    for (int x=0; x<width; x++)
    {
        for (int y=0; y<height; y++)
        {
            for (int z=0; z<depth; z++)
            {
                landmarks.push_back(Eigen::Vector3d(origin.x() + x * space, origin.y() + y * space, origin.z() + z * space));
            }
        }
    }    
}

void SimScene::draw()
{
    drawAxes();
    drawLandmarks();
}

void SimScene::drawAxes()
{
    //Draw origin
    Color::setColor(0.8, 0.8, 0.8); //white
    glutSolidCube(axesCubeWidth);
    
    //Draw x axis
    for (int i=1; i<axesLength; i++)
    {
        if (i%2 != 0){
            Color::setColor(0.0, 0.0, 0.8); //blue
        }
        else{
            Color::setColor(0.8, 0.8, 0.8); //white
        }
        glPushMatrix();
        glTranslated(i * axesCubeWidth, 0.0, 0.0);
        glutSolidCube(axesCubeWidth);
        glPopMatrix();
    }
    
    //Draw y axis
    for (int i=1; i<axesLength; i++)
    {
        if (i%2 != 0){
            Color::setColor(0.0, 0.0, 0.8); //blue
        }
        else{
            Color::setColor(0.8, 0.8, 0.8); //white
        }
        glPushMatrix();
        glTranslated(0.0, i * axesCubeWidth, 0.0);
        glutSolidCube(axesCubeWidth);
        glPopMatrix();
    }
    
    //Draw z axis
    for (int i=1; i<axesLength; i++)
    {
        if (i%2 != 0){
            Color::setColor(0.0, 0.0, 0.8); //blue
        }
        else{
            Color::setColor(0.8, 0.8, 0.8); //white
        }
        glPushMatrix();
        glTranslated(0.0, 0.0,  i * axesCubeWidth);
        glutSolidCube(axesCubeWidth);
        glPopMatrix();
    }
}

void SimScene::drawLandmarks()
{
    Color::setColor(0.0, 0.8, 0.0);
    for (int i=0; i < landmarks.size(); i++)
    {
        glPushMatrix();
        glTranslated(landmarks[i].x(), landmarks[i].y(), landmarks[i].z());
        glutSolidCube(cubeWidth);
        glPopMatrix();
    }
}