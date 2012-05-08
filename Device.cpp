
#include "Device.hpp"


Device::Device(){}

Device::Device(SimScene simScene)
{    
    measurementNoiseMean << 0.0, 0.0, 0.0;
    measurementNoiseVariance << 0.0001, 0.0001, 0.0001;
    
    accelerationNoiseMean << 0.0, 0.0, 0.0;
    accelerationNoiseVariance << 0.001, 0.001, 0.001;
    
    
    Eigen::AngleAxisd aa(pi / 2.0, Eigen::Vector3d::UnitY());
    direction = aa;
    defaultFocalLength = 1.0;
    intrinsicCalibrationMatrix << defaultFocalLength, 0.0, 0.0,
                                                     0.0, defaultFocalLength, 0.0,
                                                     0.0, 0.0, 1.0;
     
    defaultTimeStep = 0.033;
    sizeScale = 100;
    
    reset();
}

void Device::reset()
{
    currTime = 0.0;
    origin << 0.0, 0.0, 0.0; 
}

void Device::timeStep()
{
    currTime += defaultTimeStep;
}

 void Device::addNoise(Eigen::Vector3d& vec, Eigen::Vector3d noiseMean, Eigen::Vector3d noiseVariance)
{
    double noiseX = generateNoise(noiseMean(0,0), noiseVariance(0,0));
    double noiseY = generateNoise(noiseMean(1,0), noiseVariance(1,0));
    double noiseZ = generateNoise(noiseMean(2,0), noiseVariance(2,0));
    
    vec[0] = vec[0] + noiseX;
    vec[1] = vec[1] + noiseY;
    vec[2] = vec[2] + noiseZ;
}

 Eigen::Vector3d Device::control()
 {
//     Eigen::Vector3d acceleration = getAcceleration();
//     addNoise(acceleration, accelerationNoiseMean, accelerationNoiseVariance);
//     return acceleration;
     return getAcceleration();
 }
 
Eigen::VectorXd Device::measure(SimScene simScene)
{
    //Assume all landmarks observed and 2D observations
    Eigen::VectorXd observations(simScene.landmarks.size() * 2);
    
    for (int i=0, j=0; i<simScene.landmarks.size(); i++, j+=2)
    {
        Eigen::Vector3d landmark;
        landmark << simScene.landmarks[i].x(), simScene.landmarks[i].y(), simScene.landmarks[i].z();
        
        Eigen::Matrix3d rotMat;
        rotMat = direction;
        
        Eigen::Vector3d pixel;
        pixel = rotMat.transpose() * (landmark - getPosition()); // Landmark in camera coordinates
        pixel[0] = pixel.x() / pixel.z();
        pixel[1] = pixel.y() / pixel.z();
        pixel[2] = 1.0;
        pixel = intrinsicCalibrationMatrix * pixel; //Projected landmark
        
        addNoise(pixel, measurementNoiseMean, measurementNoiseVariance);
        //Only returning y value for 2D
        observations[j] = pixel.x();
        observations[j+1] = pixel.y();
    }
    //printf("Observations:\n");
    //std::cout << observations << std::endl;
    return observations;
}

Eigen::Vector3d Device::getPosition()
{

    Eigen::Vector3d position;
    position << 0.0, std::sin(currTime) * sizeScale,  std::cos(currTime) * sizeScale;
    
    return position;
}

Eigen::Vector3d Device::getVelocity()
{
    Eigen::Vector3d velocity;
    velocity << 0.0, std::cos(currTime) * sizeScale, -1.0 * std::sin(currTime) * sizeScale;
    return velocity;
}

Eigen::Vector3d Device::getAcceleration()
{
    Eigen::Vector3d acceleration;
    acceleration << 0.0, -1.0 * std::sin(currTime) * sizeScale, -1.0 * std::cos(currTime) * sizeScale;
    return acceleration;
}

Eigen::Quaterniond Device::getDirection()
{
    
}

void Device::draw()
{
     glPushMatrix();
    
    glTranslated(getPosition()[0], getPosition()[1], getPosition()[2]);
    Eigen::AngleAxisd aa;
    aa = direction;
    
    glRotated(aa.angle() * 180.0 / pi, aa.axis().x(), aa.axis().y(), aa.axis().z());
    
    glBegin(GL_TRIANGLE_FAN);
    
    Color::setColor(0.8, 0.8, 0.8); //white
    //glColor3d(0.8, 0.8, 0.8);
    //glNormal3d(0.0, 0.0, 1.0);
    glVertex3d(0.0, 0.0, 0.0);
    
    Color::setColor(0.8, 0.0, 0.0); //red
    //glNormal3d(-3.0, 3.0, 0.0);
    glVertex3d(-3.0, 3.0, defaultFocalLengthDrawn);
    
    //glNormal3d(3.0, 3.0, 0.0);
    glVertex3d(3.0, 3.0, defaultFocalLengthDrawn);
    
     //glNormal3d(3.0, -3.0, 0.0);
    glVertex3d(3.0, -3.0, defaultFocalLengthDrawn);
      
    //glNormal3d(-3.0, -3.0, 0.0);
    glVertex3d(-3.0, -3.0, defaultFocalLengthDrawn);
            
    //glNormal3d(3.0, 3.0, 0.0);
    glVertex3d(-3.0, 3.0, defaultFocalLengthDrawn);
    glEnd();
    
    glPopMatrix();
    
    //drawVelocity();
    drawAcceleration();
}

void Device::drawVelocity()
{
    glPushMatrix();
    glTranslated(getPosition().x(), getPosition().y(), getPosition().z());
    
    Color::setColor(0.8, 0.0, 0.0); //red
    glLineWidth(4.0);
    glBegin(GL_LINES);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(getVelocity().x(), getVelocity().y(), getVelocity().z());
    glEnd();
    glPopMatrix();
}

void Device::drawAcceleration()
{
    glPushMatrix();
    glTranslated(getPosition().x(), getPosition().y(), getPosition().z());
    
    Color::setColor(0.0, 0.8, 0.0); // green
    glLineWidth(4.0);
    glBegin(GL_LINES);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(getAcceleration().x(), getAcceleration().y(), getAcceleration().z());
    glEnd();
    glPopMatrix();
}