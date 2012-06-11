
#include "Device.hpp"


Device::Device(){}

Device::Device(SimScene simScene)
{    
    measurementNoiseMean << 0.0, 0.0, 0.0;
    measurementNoiseVariance << 0.3, 0.3, 0.3;
    
    accelerationNoiseMean << 0.0, 0.0, 0.0;
    //accelerationNoiseMean << 0.5, 0.5, 0.5;
    accelerationNoiseVariance << 0.0001, 0.0001, 0.0001;
    
    angVelocityNoiseMean << 0.0, 0.0, 0.0;
    //angVelocityNoiseMean << 0.5, 0.5, 0.5;
    angVelocityNoiseVariance << 0.0001, 0.0001, 0.0001;
    
    //Eigen::AngleAxisd aa(pi / 2.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd aa(0.0, Eigen::Vector3d::UnitY());
    initialImuDirection = aa;
//    defaultFocalLength = 1.0;
//    intrinsicCalibrationMatrix << defaultFocalLength, 0.0, 0.0,
//                                                     0.0, defaultFocalLength, 0.0,
//                                                     0.0, 0.0, 1.0;
    defaultFocalLength = 500;
    intrinsicCalibrationMatrix << defaultFocalLength, 0.0, 500.0,
                                                     0.0, defaultFocalLength, 500.0,
                                                     0.0, 0.0, 1.0;
    
    inverseK = intrinsicCalibrationMatrix;
    inverseK(0,0) = 1.0 / inverseK(0,0);
    inverseK(1,1) = 1.0 / inverseK(1,1);
    inverseK(0,2) = (-1.0 * inverseK(0,2)) / intrinsicCalibrationMatrix(0,0);
    inverseK(1,2) = (-1.0 * inverseK(1,2)) / intrinsicCalibrationMatrix(1,1);
    
    imu2CameraTranslation << -20.0, 0.0, 0.0;
    //Eigen::AngleAxisd aa2(0.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd aa2(pi, Eigen::Vector3d::UnitY());
    //Eigen::AngleAxisd aa2(pi/2.0, Eigen::Vector3d::UnitY());
    imu2CameraDirection = aa2;
    //printf("IMU - Camera direction: (%f, %f, %f, %f,)\n", imu2CameraDirection.w(), imu2CameraDirection.x(), imu2CameraDirection.y(), imu2CameraDirection.z());
     
    //defaultTimeStep = 0.033;
    defaultTimeStep = 0.033;
    sizeScale = 150;
    fov = pi / 2.0;
    //fov = 2.0 * std::atan((intrinsicCalibrationMatrix(0,2) / 2.0) / defaultFocalLength);
    
    this->simScene = simScene;
    
    reset();
}

Eigen::VectorXd Device::getState()
{
    Eigen::VectorXd state;
    state.resize(14 + 3 * simScene.landmarks.size());
    
    state.segment(0,3) = getImuPosition();
    Eigen::Quaterniond imuDir;
    imuDir = getImuDirection();
    state[3] = imuDir.w();
    state[4] = imuDir.x();
    state[5] = imuDir.y();
    state[6] = imuDir.z();
    state.segment(7, 3) = getCameraPosition();
    Eigen::Quaterniond camDir;
    camDir = getCameraDirection();
    state[10] = camDir.w();
    state[11] = camDir.x();
    state[12] = camDir.y();
    state[13] = camDir.z();
    
    for (int i=0; i<simScene.landmarks.size(); i++)
    {
        state.segment(14 + i * 3, 3) = simScene.landmarks[i];
    }
    
    return state;
}

void Device::reset()
{
    currTime = 0.0;
    origin << 0.0, 0.0, 0.0; 
}

void Device::timeStep()
{
    currTime += defaultTimeStep;
    if (currTime >= 2.0 * pi)
    {
        currTime -= 2.0 * pi;
    }
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

 Eigen::VectorXd Device::control()
 {
     Eigen::Vector3d acceleration = getAccelerationDev();
     addNoise(acceleration, accelerationNoiseMean, accelerationNoiseVariance);
     
     Eigen::Vector3d angVelocity = getAngularVelocityDev();
     addNoise(angVelocity, angVelocityNoiseMean, angVelocityNoiseVariance);
     
     Eigen::VectorXd control;
     control.resize(acceleration.rows() + angVelocity.rows());
     
     control.segment(0, acceleration.rows()) = acceleration;
     control.segment(3, angVelocity.rows()) = angVelocity;
     
     return control;
 }
 
Measurement Device::measure()
{        
    Measurement m;
    
    for (int i=0, j=0; i<simScene.landmarks.size(); i++, j+=2)
    {
        
        Eigen::Vector3d landmark;
        landmark << simScene.landmarks[i].x(), simScene.landmarks[i].y(), simScene.landmarks[i].z();
        
        if(visible(getCameraPosition(), getCameraDirection(), fov, landmark))
        {
            Eigen::Matrix3d rotMat;
            rotMat = getCameraDirection();

            Eigen::Vector3d pixel;
            pixel = rotMat.transpose() * (landmark - getCameraPosition()); // Landmark in camera coordinates
            pixel[0] = pixel.x() / pixel.z();
            pixel[1] = pixel.y() / pixel.z();
            pixel[2] = 1.0;
            pixel = intrinsicCalibrationMatrix * pixel; //Projected landmark

            // Truncate
            int u = (int)pixel[0];
            int v = (int)pixel[1];
            pixel[0] = (double)u;
            pixel[1] = (double)v;
            
            Eigen::Vector3d p;
            p << pixel[0], pixel[1], 1.0;       
            p = inverseK * p;
            
            addNoise(p, measurementNoiseMean, measurementNoiseVariance);
            m.add(i, p.x(), p.y());
        }
    }
    
    return m;
}

Eigen::Vector3d Device::getImuPosition()
{

    Eigen::Vector3d position;
    position << 0.0, std::sin(currTime) * sizeScale,  std::cos(currTime) * sizeScale;
    
    return position;
}

Eigen::Vector3d Device::getCameraPosition()
{
    Eigen::Vector3d position;
    position = imu2CameraTranslation;
    
    Eigen::Matrix3d rotMat;
    rotMat = getImuDirection();
    
    Eigen::Vector3d translation;
    translation = getImuPosition();
    
    position = (rotMat * position) + translation;
    
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

Eigen::Vector3d Device::getAccelerationDev()
{
    Eigen::Vector3d acceleration;
    acceleration = getAcceleration();
    
    Eigen::Matrix3d rotMat;
    rotMat = getImuDirection();
    
    Eigen::Vector3d accInImuCoordinates;
    accInImuCoordinates = rotMat * acceleration;
    return accInImuCoordinates;
}

Eigen::Quaterniond Device::getImuDirection()
{
    Eigen::Vector3d angVelocity = getAngularVelocity();
    double angMag = std::sqrt(angVelocity.x() * angVelocity.x() + angVelocity.y() * angVelocity.y() + angVelocity.z() * angVelocity.z());
    double theta = 0.5 * angMag * currTime;
    
    double w,x,y,z;
    w = std::cos(theta);
    double s = sinc(theta)*0.5*currTime;
    x = angVelocity.x() * s;
    y = angVelocity.y() * s;
    z = angVelocity.z() * s;
    Eigen::Quaterniond angQuat(w, x, y, z);
    
    return angQuat * initialImuDirection;
}

Eigen::Quaterniond Device::getCameraDirection()
{
    Eigen::Quaterniond camDirection;
    
    //camDirection = imu2CameraDirection * getImuDirection();
    camDirection = getImuDirection() * imu2CameraDirection;
    
    return camDirection;
}

Eigen::Vector3d Device::getAngularVelocity()
{
    Eigen::Vector3d angVelocity;
    //angVelocity[0] = -2.0 * pi;
    angVelocity[0] = -1.0;
    angVelocity[1] = 0.0;
    angVelocity[2] = 0.0;
        
    return angVelocity;
}

Eigen::Vector3d Device::getAngularVelocityDev()
{
    Eigen::Matrix3d rotMat;
    rotMat = getImuDirection();
    
    Eigen::Vector3d angVelocity;
    angVelocity = getAngularVelocity();
    
    Eigen::Vector3d angVelocityInImuCoords;
    angVelocityInImuCoords = rotMat * angVelocity;
    
    return angVelocityInImuCoords;
}

void Device::draw()
{
    // IMU
    glPushMatrix();
        glTranslated(getImuPosition()[0], getImuPosition()[1], getImuPosition()[2]);
        Eigen::AngleAxisd aa;
        aa = getImuDirection();

        glRotated(aa.angle() * 180.0 / pi, aa.axis().x(), aa.axis().y(), aa.axis().z());

        Color::setColor(0.8, 0.0, 0.0); //red
        glutSolidCube(imuCubeWidth); // IMU
        
        // Axes
//        glBegin(GL_LINES);
//        glVertex3d(0.0, 0.0, 0.0);
//        glVertex3d(10.0, 0.0, 0.0);
//        glEnd();
//        
//        Color::setColor(0.0, 0.8, 0.0); //green
//        glBegin(GL_LINES);
//        glVertex3d(0.0, 0.0, 0.0);
//        glVertex3d(0.0, 10.0, 0.0);
//        glEnd();
//        
//        Color::setColor(0.0, 0.0, 0.8); //blue
//        glBegin(GL_LINES);
//        glVertex3d(0.0, 0.0, 0.0);
//        glVertex3d(0.0, 0.0, 10.0);
//        glEnd();

    glPopMatrix();

    // CAMERA
    glPushMatrix();
        glTranslated(getCameraPosition().x(), getCameraPosition().y(), getCameraPosition().z());
        aa = getCameraDirection();
        glRotated(aa.angle() * 180.0 / pi, aa.axis().x(), aa.axis().y(), aa.axis().z());

        glBegin(GL_TRIANGLE_FAN);
            Color::setColor(0.8, 0.8, 0.8); //white
            glVertex3d(0.0, 0.0, 0.0);

            Color::setColor(0.8, 0.0, 0.0); //red
            glVertex3d(-3.0, 3.0, defaultFocalLengthDrawn);
            glVertex3d(3.0, 3.0, defaultFocalLengthDrawn);
            glVertex3d(3.0, -3.0, defaultFocalLengthDrawn);
            glVertex3d(-3.0, -3.0, defaultFocalLengthDrawn);
            glVertex3d(-3.0, 3.0, defaultFocalLengthDrawn);
        glEnd();
    glPopMatrix();

       
    for (int i=0; i < simScene.landmarks.size(); i++)
    {
        Eigen::Vector3d lm;
        lm << simScene.landmarks[i].x(), simScene.landmarks[i].y(), simScene.landmarks[i].z();
        
        if (visible(getCameraPosition(), getCameraDirection(), fov, lm))
        {
            Color::setColor(0.0, 0.8, 0.0); // Green    
        }
        else{
            Color::setColor(0.8, 0.0, 0.0); // Red
        }
        
        glPushMatrix();
        glTranslated(simScene.landmarks[i].x(), simScene.landmarks[i].y(), simScene.landmarks[i].z());
        glutSolidCube(cubeWidth);
        glPopMatrix();
    }
    
    simScene.drawAxes();
    
    //drawVelocity();
    drawAcceleration();
}

void Device::drawVelocity()
{
    glPushMatrix();
    glTranslated(getImuPosition().x(), getImuPosition().y(), getImuPosition().z());
    
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
    glTranslated(getImuPosition().x(), getImuPosition().y(), getImuPosition().z());
    
    Color::setColor(0.1, 0.1, 0.1); // black
    glLineWidth(4.0);
    glBegin(GL_LINES);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(getAcceleration().x(), getAcceleration().y(), getAcceleration().z());
    glEnd();
    glPopMatrix();
}