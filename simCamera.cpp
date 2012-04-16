
#include "simCamera.hpp"


SimCamera::SimCamera(){}

SimCamera::SimCamera(SimScene simScene)
{
    pi = 3.1415926535897932384626433832795028841971693993751058;
    
    positionNoiseMean(0,0) = 0.0;
    positionNoiseMean(1,0) = 0.0;
    positionNoiseMean(2,0) = 0.0;
    
    positionNoiseVariance(0,0) = 0.001;
    positionNoiseVariance(1,0) = 0.001;
    positionNoiseVariance(2,0) = 0.001;
    
    velocityNoiseMean(0,0) = 0.0;
    velocityNoiseMean(1,0) = 0.0;
    velocityNoiseMean(2,0) = 0.0;
    
    velocityNoiseVariance(0,0) = 0.001;
    velocityNoiseVariance(1,0) = 0.001;
    velocityNoiseVariance(2,0) = 0.001;
    
    accelerationNoiseMean(0,0) = 0.0;
    accelerationNoiseMean(1,0) = 0.0;
    accelerationNoiseMean(2,0) = 0.0;
    
    accelerationNoiseVariance(0,0) = 0.1;
    accelerationNoiseVariance(1,0) = 0.1;
    accelerationNoiseVariance(2,0) = 0.1;
    
    measurementNoiseMean(0,0) = 0.0;
    measurementNoiseMean(1,0) = 0.0;
    measurementNoiseMean(2,0) = 0.0;
    
    measurementNoiseVariance(0,0) = 0.00001;
    measurementNoiseVariance(1,0) = 0.00001;
    measurementNoiseVariance(2,0) = 0.00001;
    
    Eigen::AngleAxisd aa(pi / 2.0, Eigen::Vector3d::UnitY());
    direction = aa;
    
    focalLength = 1.0;
    intrinsicCalibrationMatrix << focalLength,                0.0, 0.0,
                                                                    0.0, focalLength, 0.0,
                                                                    0.0,                0.0, 1.0;
    
    double depth = 110.0;
    defaultInverseDepth = 1.0/depth;
    defaultTimeStep = 0.05;
    
    reset();
    initializeMap(simScene);
}

void SimCamera::initializeMap(SimScene simScene)
{
    for (int i=0; i<simScene.landmarks.size(); i++)
    {
        Eigen::Vector3d origin;
        origin = getPosition();
        
        Landmark landmark(origin, 
                          simScene.landmarks.at(i) - origin, 
                          defaultInverseDepth);
        map.push_back(landmark);
    }
}

void SimCamera::reset()
{
    currTime = 0.0;
    
    initialVelocity(0,0) = 0.0;
    initialVelocity(1,0) = 0.0;
    initialVelocity(2,0) = 0.0;
    
    initialPosition(0,0) = -1.0;
    initialPosition(1,0) = 200.0;
    initialPosition(2,0) = 0.0;
    
    position= initialPosition;
    velocity = initialVelocity;
    
    acceleration(0,0) = 0.0;
    acceleration(1,0) = -9.81;
    acceleration(2,0) = 0.0;
    
}

void SimCamera::timeStep()
{
//    currTime += defaultTimeStep;
//    addNoise2Acceleration();
//
//    velocity = (acceleration * currTime) + initialVelocity;
//    
//    position = 
//            (0.5 * acceleration * (currTime * currTime)) +
//            (initialVelocity * currTime) +
//            initialPosition;
    
    addNoise2Acceleration();
    position = position + 
            (0.5 * acceleration * (defaultTimeStep * defaultTimeStep)) +
            (velocity * defaultTimeStep);
    
    velocity = velocity + acceleration * defaultTimeStep;
    
    //Reset acceleration -- doesn't appear to matter
    acceleration(0,0) = 0.0;
    acceleration(1,0) = -9.81;
    acceleration(2,0) = 0.0;
    
}

void SimCamera::addNoise2Acceleration()
{
    double noiseX = generateNoise(accelerationNoiseMean(0,0), accelerationNoiseVariance(0,0));
    double noiseY = generateNoise(accelerationNoiseMean(1,0), accelerationNoiseVariance(1,0));
    double noiseZ = generateNoise(accelerationNoiseMean(2,0), accelerationNoiseVariance(2,0));
    
    //Only add noise to Y value for 2d simulation
    //acceleration(1,0) = acceleration(1,0) + noiseY;
    
    //Add noise to all 3 dimensions for 3D simulation
    acceleration[0] = acceleration[0] + noiseX;
    acceleration[1] = acceleration[1] + noiseY;
    acceleration[2] = acceleration[2] + noiseZ;
}

void SimCamera::addNoise2Position()
{
    double noiseX = generateNoise(positionNoiseMean(0,0), positionNoiseVariance(0,0));
    double noiseY = generateNoise(positionNoiseMean(1,0), positionNoiseVariance(1,0));
    double noiseZ = generateNoise(positionNoiseMean(2,0), positionNoiseVariance(2,0));
    
    //printf("Perfect  Position = %f\n", camPosition(1,0));
    
    //Only put noise on Y value for 2d simulation
    position(1,0) = position(1,0) + noiseY;
    
    //printf("Noisy    Position = %f\n", camPosition(1,0));
}

Eigen::VectorXd SimCamera::measure(SimScene simScene)
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
        pixel = rotMat.transpose() * (landmark - position); // Landmark in camera coordinates
        pixel[0] = pixel.x() / pixel.z();
        pixel[1] = pixel.y() / pixel.z();
        pixel[2] = 1.0;
        pixel = intrinsicCalibrationMatrix * pixel; //Projected landmark
        
        addNoise2Measurement(pixel);
        //Only returning y value for 2D
        observations[j] = pixel.x();
        observations[j+1] = pixel.y();
    }
    //printf("Observations:\n");
    //std::cout << observations << std::endl;
    return observations;
}

void SimCamera::addNoise2Measurement(Eigen::Vector3d& imagePoint)
{
    double noiseX = generateNoise(measurementNoiseMean(0,0), measurementNoiseVariance(0,0));
    double noiseY = generateNoise(measurementNoiseMean(1,0), measurementNoiseVariance(1,0));
    double noiseZ = generateNoise(measurementNoiseMean(2,0), measurementNoiseVariance(2,0));    
    
    //Only add noise to y dimension for 2d example
    //imagePoint(1,0) = imagePoint(1,0) + noiseY;
    
    //Add noise to both image plane dimensions for 3d
    imagePoint[0] = imagePoint[0] + noiseX;
    imagePoint[1] = imagePoint[1] + noiseY;
}

Eigen::Vector3d SimCamera::getPosition()
{
    //printf("Simulated Camera Position: (%f, %f)\n", camPosition(0,0), camPosition(1,0));
    return position;
}

void SimCamera::draw()
{
     glPushMatrix();
    
    glTranslated(position[0], position[1], position[2]);
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
}
