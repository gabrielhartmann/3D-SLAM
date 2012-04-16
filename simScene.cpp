#include "simScene.hpp"

SimScene::SimScene()
{
//    landmarks.push_back(Eigen::Vector3d (90.0, 100.0, 0.0));
//    
//    landmarks.push_back(Eigen::Vector3d (80.0, 100.0, 0.0));
//    landmarks.push_back(Eigen::Vector3d (70.0, 100.0, 0.0));
//    landmarks.push_back(Eigen::Vector3d (60.0, 100.0, 0.0));
//    landmarks.push_back(Eigen::Vector3d (50.0, 100.0, 0.0));
//    
//    landmarks.push_back(Eigen::Vector3d (100.0, 100.0, 0.0));
//    landmarks.push_back(Eigen::Vector3d (100.0, 90.0, 0.0));
//    landmarks.push_back(Eigen::Vector3d (100.0, 80.0, 0.0));
//    landmarks.push_back(Eigen::Vector3d (100.0, 70.0, 0.0));
//    landmarks.push_back(Eigen::Vector3d (100.0, 50.0, 0.0));
//    landmarks.push_back(Eigen::Vector3d (100.0, 60.0, 0.0));
//    landmarks.push_back(Eigen::Vector3d (100.0, 40.0, 0.0));
//    landmarks.push_back(Eigen::Vector3d (100.0, 30.0, 0.0));
//    landmarks.push_back(Eigen::Vector3d (100.0, 20.0, 0.0));
//    
//    landmarks.push_back(Eigen::Vector3d (100.0, 10.0, 0.0));
//    landmarks.push_back(Eigen::Vector3d (90.0, 10.0, 0.0));
//    landmarks.push_back(Eigen::Vector3d (80.0, 10.0, 0.0));
//    landmarks.push_back(Eigen::Vector3d (70.0, 10.0, 0.0));
//    landmarks.push_back(Eigen::Vector3d (60.0, 10.0, 0.0));
//    landmarks.push_back(Eigen::Vector3d (50.0, 10.0, 0.0));
    
    landmarks.push_back(Eigen::Vector3d (100.0, 0.0, -100.0));
    landmarks.push_back(Eigen::Vector3d (100.0, 0.0, 10.0));
    landmarks.push_back(Eigen::Vector3d (100.0, 0.0, 100.0));
    
    landmarks.push_back(Eigen::Vector3d (100.0, 100.0, -100.0));
    landmarks.push_back(Eigen::Vector3d (100.0, 100.0, 10.0));
    landmarks.push_back(Eigen::Vector3d (100.0, 100.0, 100.0));
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
    glutSolidCube(cubeWidth);
    
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
        glTranslated(i * cubeWidth, 0.0, 0.0);
        glutSolidCube(cubeWidth);
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
        glTranslated(0.0, i * cubeWidth, 0.0);
        glutSolidCube(cubeWidth);
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
        glTranslated(0.0, 0.0,  i * cubeWidth);
        glutSolidCube(cubeWidth);
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