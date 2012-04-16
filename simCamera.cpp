
#include "simCamera.hpp"


SimCamera::SimCamera(){}

SimCamera::SimCamera(SimScene simScene)
{
    camPositionNoiseMean(0,0) = 0.0;
    camPositionNoiseMean(1,0) = 0.0;
    camPositionNoiseMean(2,0) = 0.0;
    
    camPositionNoiseVariance(0,0) = 0.001;
    camPositionNoiseVariance(1,0) = 0.001;
    camPositionNoiseVariance(2,0) = 0.001;
    
    camVelocityNoiseMean(0,0) = 0.0;
    camVelocityNoiseMean(1,0) = 0.0;
    camVelocityNoiseMean(2,0) = 0.0;
    
    camVelocityNoiseVariance(0,0) = 0.001;
    camVelocityNoiseVariance(1,0) = 0.001;
    camVelocityNoiseVariance(2,0) = 0.001;
    
    camAccelerationNoiseMean(0,0) = 0.0;
    camAccelerationNoiseMean(1,0) = 0.0;
    camAccelerationNoiseMean(2,0) = 0.0;
    
    camAccelerationNoiseVariance(0,0) = 0.001;
    camAccelerationNoiseVariance(1,0) = 0.001;
    camAccelerationNoiseVariance(2,0) = 0.001;
    
    camMeasurementNoiseMean(0,0) = 0.0;
    camMeasurementNoiseMean(1,0) = 0.0;
    camMeasurementNoiseMean(2,0) = 0.0;
    
    camMeasurementNoiseVariance(0,0) = 0.00001;
    camMeasurementNoiseVariance(1,0) = 0.00001;
    camMeasurementNoiseVariance(2,0) = 0.00001;
    
    camDirection(0,0) = 1.0;
    camDirection(1,0) = 0.0;
    camDirection(2,0) = 0.0;
    
    double depth = 110.0;
    defaultInverseDepth = 1.0/depth;
    defaultTimeStep = 0.33;
    
    reset();
    initializeMap(simScene);
}

void SimCamera::initializeMap(SimScene simScene)
{
    for (int i=0; i<simScene.landmarks.size(); i++)
    {
        Eigen::Vector3d origin;
        origin = camPosition;
        
        Landmark landmark(origin, 
                          simScene.landmarks.at(i) - origin, 
                          defaultInverseDepth);
        map.push_back(landmark);
    }
}

double SimCamera::angleV2V(Eigen::Vector3d v1, Eigen::Vector3d v2)
{
    v1.normalize();
    v2.normalize();
    double angle = acos(v1.dot(v2));
    if (v2(1,0) < v1(1,0))
    {
        angle *= -1.0;
    }
    /*
    printf("angle = %f in radians\n", angle);
    printf("angle = %f in degrees\n", angle * 180.0/3.1415927);
    */
    
    return angle;
}

void SimCamera::reset()
{
    currTime = 0.0;
    
    camInitialVelocity(0,0) = 0.0;
    camInitialVelocity(1,0) = -1.0;
    camInitialVelocity(2,0) = 0.0;
    
    camInitialPosition(0,0) = -1.0;
    camInitialPosition(1,0) = 100.0;
    camInitialPosition(2,0) = 0.0;
    
    camPosition = camInitialPosition;
    camVelocity = camInitialVelocity;
    
    camAcceleration(0,0) = 0.0;
    camAcceleration(1,0) = -9.81;
    camAcceleration(2,0) = 0.0;
    
}

void SimCamera::timeStep()
{
    currTime += defaultTimeStep;
    //camPosition = camPosition + defaultTimeStep * camVelocity ;       
    //addNoise2Position();

    addNoise2Acceleration();

    camVelocity = (camAcceleration * currTime) + camInitialVelocity;
    
    camPosition = 
            (0.5 * camAcceleration * (currTime * currTime)) +
            (camInitialVelocity * currTime) +
            camInitialPosition;
    
    //std::cout << "Noisy camera position:" << std::endl;
    //std::cout << camPosition << std::endl;
    
    //Reset acceleration
    camAcceleration(0,0) = 0.0;
    camAcceleration(1,0) = -9.81;
    camAcceleration(2,0) = 0.0;
    
}

void SimCamera::addNoise2Acceleration()
{
    double noiseX = generateNoise(camAccelerationNoiseMean(0,0), camAccelerationNoiseVariance(0,0));
    double noiseY = generateNoise(camAccelerationNoiseMean(1,0), camAccelerationNoiseVariance(1,0));
    double noiseZ = generateNoise(camAccelerationNoiseMean(2,0), camAccelerationNoiseVariance(2,0));
    
    //Only add noise to Y value for 2d simulation
    camAcceleration(1,0) = camAcceleration(1,0) + noiseY;
    printf("ACTUAL ACCELERATION = %.10f\n", camAcceleration(1,0));
}

void SimCamera::addNoise2Position()
{
    double noiseX = generateNoise(camPositionNoiseMean(0,0), camPositionNoiseVariance(0,0));
    double noiseY = generateNoise(camPositionNoiseMean(1,0), camPositionNoiseVariance(1,0));
    double noiseZ = generateNoise(camPositionNoiseMean(2,0), camPositionNoiseVariance(2,0));
    
    //printf("Perfect  Position = %f\n", camPosition(1,0));
    
    //Only put noise on Y value for 2d simulation
    camPosition(1,0) = camPosition(1,0) + noiseY;
    
    //printf("Noisy    Position = %f\n", camPosition(1,0));
}

Eigen::VectorXd SimCamera::measure(SimScene simScene)
{
    //Assume all landmarks observed and 1D observations
    Eigen::VectorXd observations(simScene.landmarks.size());
    
    for (int i=0; i<simScene.landmarks.size(); i++)
    {
        double slope = (simScene.landmarks.at(i)(1,0) - camPosition(1,0)) /
                       (simScene.landmarks.at(i)(0,0) - camPosition(0,0));
        double yIntercept = camPosition(1,0) - (slope * camPosition(0,0));
        
        Eigen::Vector3d imagePoint(0.0, yIntercept, 0.0);
        imagePoint = imagePoint - camPosition;
        
        /*
        printf("Imagepoint %d\n", i);
        std::cout << imagePoint << std::endl;
        */
        
        addNoise2Measurement(imagePoint);
        
        /*
        printf("Imagepoint after noise\n");
        std::cout << imagePoint << std::endl;
        */
        
        observations(i,0) = imagePoint(1,0); //Observation = y value of projection
    }
    //printf("Observations:\n");
    //std::cout << observations << std::endl;
    return observations;
}

void SimCamera::addNoise2Measurement(Eigen::Vector3d& imagePoint)
{
    double noiseX = generateNoise(camMeasurementNoiseMean(0,0), camMeasurementNoiseVariance(0,0));
    double noiseY = generateNoise(camMeasurementNoiseMean(1,0), camMeasurementNoiseVariance(1,0));
    double noiseZ = generateNoise(camMeasurementNoiseMean(2,0), camMeasurementNoiseVariance(2,0));    
    
    //Only add noise to y dimension for 2d example
    imagePoint(1,0) = imagePoint(1,0) + noiseY;
}

Eigen::Vector3d SimCamera::position()
{
    //printf("Simulated Camera Position: (%f, %f)\n", camPosition(0,0), camPosition(1,0));
    return camPosition;
}

void SimCamera::draw()
{
     glPushMatrix();
    
    glTranslated(camPosition[0], camPosition[1], camPosition[2]);
    Eigen::AngleAxisd aa(90.0, Eigen::Vector3d::UnitY());
    glRotated(aa.angle(), aa.axis().x(), aa.axis().y(), aa.axis().z());
    
    glBegin(GL_TRIANGLE_FAN);
    
    Color::setColor(0.8, 0.8, 0.8); //white
    //glColor3d(0.8, 0.8, 0.8);
    //glNormal3d(0.0, 0.0, 1.0);
    glVertex3d(0.0, 0.0, 0.0);
    
    Color::setColor(0.8, 0.0, 0.0); //red
    //glNormal3d(-3.0, 3.0, 0.0);
    glVertex3d(-3.0, 3.0, defaultFocalLength);
    
    //glNormal3d(3.0, 3.0, 0.0);
    glVertex3d(3.0, 3.0, defaultFocalLength);
    
     //glNormal3d(3.0, -3.0, 0.0);
    glVertex3d(3.0, -3.0, defaultFocalLength);
      
    //glNormal3d(-3.0, -3.0, 0.0);
    glVertex3d(-3.0, -3.0, defaultFocalLength);
            
    //glNormal3d(3.0, 3.0, 0.0);
    glVertex3d(-3.0, 3.0, defaultFocalLength);
    glEnd();
    
    glPopMatrix();
}
