
#include "Device.hpp"


Device::Device(){}

Device::Device(SimScene simScene)
{    
    measurementNoiseMean << 0.0, 0.0, 0.0;
    measurementNoiseVariance << 0.00001, 0.00001, 0.00001;
    
    accelerationNoiseMean << 0.0, 0.0, 0.0;
    accelerationNoiseVariance << 0.001, 0.001, 0.001;
    
    angVelocityNoiseMean << 0.0, 0.0, 0.0;
    angVelocityNoiseVariance << 0.001, 0.001, 0.001;
    
    Eigen::AngleAxisd aa(pi / 2.0, Eigen::Vector3d::UnitY());
    initialImuDirection = aa;
    defaultFocalLength = 1.0;
    intrinsicCalibrationMatrix << defaultFocalLength, 0.0, 0.0,
                                                     0.0, defaultFocalLength, 0.0,
                                                     0.0, 0.0, 1.0;
    
    imu2CameraTranslation << -20.0, 0.0, 0.0;
    Eigen::AngleAxisd aa2(0.0, Eigen::Vector3d::UnitY());
    imu2CameraDirection = aa2;
     
    //defaultTimeStep = 0.033;
    defaultTimeStep = 0.066;
    sizeScale = 20;
    fov = pi / 3.0;
    
    this->simScene = simScene;
    
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
     Eigen::Vector3d acceleration = getAcceleration();
     addNoise(acceleration, accelerationNoiseMean, accelerationNoiseVariance);
     Eigen::Vector3d angVelocity = getAngularVelocity();
     addNoise(angVelocity, accelerationNoiseMean, accelerationNoiseVariance);
     
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

            addNoise(pixel, measurementNoiseMean, measurementNoiseVariance);
            m.add(i, pixel.x(), pixel.y());
        }
    }
    
    return m;
}

Eigen::Vector3d Device::getImuPosition()
{

    Eigen::Vector3d position;
    position << 0.0, std::sin(currTime) * sizeScale,  std::cos(currTime) * sizeScale;
    //position << std::sin(currTime * 1.0) * sizeScale / 2.0, std::sin(currTime) * sizeScale,  std::cos(currTime) * sizeScale;
    
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
    //velocity << std::cos(currTime * 1.0) * sizeScale / 2.0, std::cos(currTime) * sizeScale, -1.0 * std::sin(currTime) * sizeScale;
    return velocity;
}

Eigen::Vector3d Device::getAcceleration()
{
    Eigen::Vector3d acceleration;
    acceleration << 0.0, -1.0 * std::sin(currTime) * sizeScale, -1.0 * std::cos(currTime) * sizeScale;
    //acceleration << -1.0 * std::sin(currTime * 1.0) * sizeScale / 2.0, -1.0 * std::sin(currTime) * sizeScale, -1.0 * std::cos(currTime) * sizeScale;
    return acceleration;
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
    
    camDirection = imu2CameraDirection * getImuDirection();
    
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

void Device::draw()
{
    // IMU
    glPushMatrix();
        glTranslated(getImuPosition()[0], getImuPosition()[1], getImuPosition()[2]);
        Eigen::AngleAxisd aa;
        aa = getImuDirection();

        glRotated(aa.angle() * 180.0 / pi, aa.axis().x(), aa.axis().y(), aa.axis().z());

        Color::setColor(0.8, 0.0, 0.0); //red

        glutSolidCube(cubeWidth);

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
        
        if (visible(getImuPosition(), getImuDirection(), fov, lm))
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
    
    Color::setColor(0.0, 0.8, 0.0); // green
    glLineWidth(4.0);
    glBegin(GL_LINES);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(getAcceleration().x(), getAcceleration().y(), getAcceleration().z());
    glEnd();
    glPopMatrix();
}