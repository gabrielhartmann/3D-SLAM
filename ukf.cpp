#include "ukf.hpp"


UKF::UKF(){}

UKF::UKF(Device simCamera, SimScene scene)
{
    
    this->simCamera = simCamera;
    this->scene = scene;
    
    initialize();
}

void UKF::initialize()
{
    filterStepCount = 0;
    
    alpha = 0.001;
    //alpha = 10.0;
    beta = 2.0;

    lmIndex.clear();
    
    fov = PI;
    
    initializeStateAndCovariance();    
}

Eigen::VectorXd UKF::getState()
{
    Eigen::VectorXd state;
    int numLandmarks = (stateVector.rows() - deviceStateSize) / landmarkSize;
    
    state.resize(14 + 3*numLandmarks); // imuPos(3) imuDir(4) camPos(3) camDir(4)
    state.segment(0, 3) = imuPosition();
    Eigen::Quaterniond imuDir;
    imuDir = imuDirection();
    state[3] = imuDir.w();
    state[4] = imuDir.x();
    state[5] = imuDir.y();
    state[6] = imuDir.z();
    state.segment(7, 3) = cameraPosition();
    Eigen::Quaterniond camDir;
    camDir = cameraDirection();
    state[10] = camDir.w();
    state[11] = camDir.x();
    state[12] = camDir.y();
    state[13] = camDir.z();
    
    
    for (int i=0; i<numLandmarks; i++)
    {
        state.segment(14 + i * 3, 3) = getEuclideanLandmark(i);
    }
    
    return state;
}

Eigen::VectorXd UKF::step(double timeStep, Eigen::VectorXd control, Measurement m)
{
    printf("======================================================================\n");
    printf("                                %d                                    \n", ++filterStepCount);
    printf("======================================================================\n");  
       
    printf("Entering process update\n");
    processUpdate(timeStep, control);
    printf("Exiting process update\n");
    printf("Entering measurement update\n");
    measurementUpdate(m);
    printf("Exiting measurement update\n");
    
    //print("State Vector:", stateVector);
    return stateVector;
}

Eigen::Vector3d UKF::imuPosition()
{
    Eigen::Vector3d pos(stateVector[0], stateVector[1], stateVector[2]);
    return pos;
}

Eigen::Vector3d UKF::imuPosition(Eigen::VectorXd sigmaPoint)
{
    Eigen::Vector3d pos(sigmaPoint[0], sigmaPoint[1], sigmaPoint[2]);
    return pos;
}

Eigen::Vector3d UKF::cameraPosition()
{
    Eigen::Vector3d position(stateVector[10], stateVector[11], stateVector[12]);
    
    Eigen::Matrix3d rotMat;
    rotMat = imuDirection();
    
    Eigen::Vector3d translation;
    translation = imuPosition();
    
    position = (rotMat * position) + translation;
    
    return position;
}

Eigen::Vector3d UKF::cameraPosition(Eigen::VectorXd sigmaPoint)
{
    Eigen::Vector3d position(sigmaPoint[10], sigmaPoint[11], sigmaPoint[12]);
    
    Eigen::Matrix3d rotMat;
    rotMat = imuDirection(sigmaPoint);
    
    Eigen::Vector3d translation;
    translation = imuPosition(sigmaPoint);
    
    position = (rotMat * position) + translation;
    
    return position;
}

Eigen::Quaterniond UKF::imuDirection()
{
    Eigen::Quaterniond dir(stateVector[6], stateVector[7], stateVector[8], stateVector[9]);
    return dir;
}

Eigen::Quaterniond UKF::imuDirection(Eigen::VectorXd sigmaPoint)
{
    Eigen::Quaterniond dir(sigmaPoint[6], sigmaPoint[7], sigmaPoint[8], sigmaPoint[9]);
    return dir;
}

Eigen::Quaterniond UKF::cameraDirection()
{
    Eigen::Quaterniond camDirection;
    
    Eigen::Quaterniond imu2CameraDirection(stateVector[13], stateVector[14], stateVector[15], stateVector[16]);
    
    camDirection = imuDirection() * imu2CameraDirection;
    
    return camDirection;
}

Eigen::Quaterniond UKF::cameraDirection(Eigen::VectorXd sigmaPoint)
{
    Eigen::Quaterniond camDirection;
    
    Eigen::Quaterniond imu2CameraDirection(sigmaPoint[13], sigmaPoint[14], sigmaPoint[15], sigmaPoint[16]);
    
    camDirection =  imuDirection(sigmaPoint) * imu2CameraDirection;
    
    return camDirection;
}

void UKF::draw()
{
    drawImu();
    drawCamera();
    
    // Draw landmarks
    Color::setColor(0.0, 0.0, 0.8); // blue
    for (int i=0; i < landmarks().size(); i++)
    {
        glPushMatrix();
        glTranslated(landmarks()[i].x(), landmarks()[i].y(), landmarks()[i].z());
        glutSolidCube(cubeWidth);
        glPopMatrix();
    }
    
    // Draw hypotheticals
//    for (int i=0; i < sigmaPoints.size(); i++)
//    {
//        Color::setColor(0.0, 0.8, 0.8); // cyan
//        //Draw hypothetical landmarks
//        int numLandmarks = (sigmaPoints[i].rows() - deviceStateSize) / landmarkSize;
//        for (int j=0; j<numLandmarks; j++)
//        {
//            Eigen::Vector3d lm;
//            lm = getEuclideanLandmark(j, sigmaPoints[i]);
//            glPushMatrix();
//            glTranslated(lm.x(), lm.y(), lm.z());
//            glutSolidCube(cubeWidth);
//            glPopMatrix();
//        }
//        
//        //Draw hypothetical camera positions
//        drawCamera(sigmaPoints[i]);
//    }
}

void UKF::drawImu()
{
    glPushMatrix();
    
    glTranslated(imuPosition()[0], imuPosition()[1], imuPosition()[2]);
    
    Eigen::AngleAxisd aa;
    aa = imuDirection();
    glRotated(aa.angle() * 180.0 / simCamera.pi, aa.axis().x(), aa.axis().y(), aa.axis().z());
    
    Color::setColor(0.0, 0.0, 0.9); //blue
    glutSolidCube(cubeWidth);
    
    glPopMatrix();
}

void UKF::drawCamera()
{
    glPushMatrix();
    
    Eigen::Vector3d position = cameraPosition();
    glTranslated(position.x(), position.y(), position.z());
    
    Eigen::AngleAxisd aa;
    aa = cameraDirection();
    glRotated(aa.angle() * 180.0 / PI, aa.axis().x(), aa.axis().y(), aa.axis().z());
    
    glBegin(GL_TRIANGLE_FAN);
        Color::setColor(0.8, 0.8, 0.8); //white
        glVertex3d(0.0, 0.0, 0.0);

        Color::setColor(0.0, 0.0, 0.8); //blue
        glVertex3d(-3.0, 3.0, simCamera.defaultFocalLengthDrawn);
        glVertex3d(3.0, 3.0, simCamera.defaultFocalLengthDrawn);
        glVertex3d(3.0, -3.0, simCamera.defaultFocalLengthDrawn);
        glVertex3d(-3.0, -3.0, simCamera.defaultFocalLengthDrawn);
        glVertex3d(-3.0, 3.0, simCamera.defaultFocalLengthDrawn);
    glEnd();
    
    glPopMatrix();
}

void UKF::drawCamera(Eigen::VectorXd sigmaPoint)
{
    glPushMatrix();
    
    Eigen::Vector3d position = cameraPosition(sigmaPoint);
    glTranslated(position.x(), position.y(), position.z());
    
    Eigen::AngleAxisd aa;
    aa = cameraDirection(sigmaPoint);
    glRotated(aa.angle() * 180.0 / PI, aa.axis().x(), aa.axis().y(), aa.axis().z());
    
    glBegin(GL_TRIANGLE_FAN);
        Color::setColor(0.8, 0.8, 0.8); //white
        glVertex3d(0.0, 0.0, 0.0);

        Color::setColor(0.8, 0.0, 0.8); // magenta
        glVertex3d(-3.0, 3.0, simCamera.defaultFocalLengthDrawn);
        glVertex3d(3.0, 3.0, simCamera.defaultFocalLengthDrawn);
        glVertex3d(3.0, -3.0, simCamera.defaultFocalLengthDrawn);
        glVertex3d(-3.0, -3.0, simCamera.defaultFocalLengthDrawn);
        glVertex3d(-3.0, 3.0, simCamera.defaultFocalLengthDrawn);
    glEnd();
    
    glPopMatrix();
}

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > UKF::landmarks()
{
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > lms;
    for (int i=0, j=0; i<lmIndex.size(); i++, j+=landmarkSize)
    {
        lms.push_back(getEuclideanLandmark(i));
    }
    
    return lms;
}


Eigen::MatrixXd UKF::getMeasurementCovariance(int rows)
{
    Eigen::MatrixXd measurementCovariance;
    measurementCovariance.resize(rows, rows);
    clear(measurementCovariance);
    
    for (int row=0; row<measurementCovariance.rows(); row++)
    {
        measurementCovariance(row, row) = simCamera.measurementNoiseVariance[0]; // X
    }
    
    return measurementCovariance;
}

void UKF::initializeStateAndCovariance()
{    
    Eigen::VectorXd state;
    state.resize(deviceStateSize);
    clear(state);
    //state.segment(0, 3) = simCamera.getImuPosition();
    state.segment(0, 3) = simCamera.getImuPosition();
    //state[2] = state[2] - 8.0;  // A translation so both devices are visible at start
    state.segment(3, 3) = simCamera.getVelocity();
    Eigen::Quaterniond imuDir = simCamera.getImuDirection();
    state[6] = imuDir.w();
    state[7] = imuDir.x();
    state[8] = imuDir.y();
    state[9] = imuDir.z();
    state.segment(10, 3) = simCamera.imu2CameraTranslation * 1.0;
    Eigen::Quaterniond camDirWrtImu = simCamera.imu2CameraDirection;
    state[13] = camDirWrtImu.w();
    state[14] = camDirWrtImu.x();
    state[15] = camDirWrtImu.y();
    state[16] = camDirWrtImu.z();
    state[17] = initialAccBias;
    state[18] = initialAccBias;
    state[19] = initialAccBias;
    state[20] = initialGyroBias;
    state[21] = initialGyroBias;
    state[22] = initialGyroBias;
          
    Eigen::MatrixXd covariance;
    covariance.resize(state.rows(), state.rows());
    clear(covariance);
    covariance(0,0) = 0.01; // Position
    covariance(1,1) = 0.01;
    covariance(2,2) = 0.01;
    covariance(3,3) = simCamera.accelerationNoiseVariance[0]; // Velocity
    covariance(4,4) = simCamera.accelerationNoiseVariance[1];
    covariance(5,5) = simCamera.accelerationNoiseVariance[2];
    covariance(6,6) = 0.0001; // IMU Direction
    covariance(7,7) = 0.0001;
    covariance(8,8) = 0.0001;
    covariance(9,9) = 0.0001;
    covariance(10,10) = 1.0; // IMU 2 Camera Translation in IMU coordinates
    covariance(11,11) = 1.0;
    covariance(12,12) = 1.0;
    covariance(13,13) = 0.0001; // IMU 2 Camera Direction in IMU coordinates
    covariance(14,14) = 0.0001;
    covariance(15,15) = 0.0001;
    covariance(16,16) = 0.0001;
    covariance(17,17) = accBiasVariance; // Accelerometer bias
    covariance(18,18) = accBiasVariance;
    covariance(19,19) = accBiasVariance;
    covariance(20,20) = gyroBiasVariance;
    covariance(21,21) = gyroBiasVariance;
    covariance(22,22) = gyroBiasVariance;
    
    Measurement m = simCamera.measure();
    addNewLandmarks(m, state, covariance);
    
    stateVector.resize(state.rows());
    stateVector = state;
    
    stateCovariance.resize(covariance.rows(), covariance.cols());
    stateCovariance = covariance;
    
    removeZero(stateCovariance, 0.001);
    
//    print("State:", stateVector);
//    printf("\n");
//    print("Covariance:", stateCovariance);
//    printf("\n");
}

Eigen::MatrixXd UKF::getProcessCovariance()
{
    int numUninitLandmarks = numUninitializedLandmarks();
    //printf("Number of uninitialized landmarks = %d\n", numUninitLandmarks);
    
    Eigen::MatrixXd processCovariance;
    processCovariance.resize(processNoiseSize + numUninitLandmarks, processNoiseSize + numUninitLandmarks);
    clear(processCovariance);

    processCovariance(0,0) = simCamera.accelerationNoiseVariance[0] ; // Acceleration Noise
    processCovariance(1,1) = simCamera.accelerationNoiseVariance[1];
    processCovariance(2,2) = simCamera.accelerationNoiseVariance[2];

    processCovariance(3,3) = simCamera.angVelocityNoiseVariance[0]; // Angular Velocity Noise
    processCovariance(4,4) = simCamera.angVelocityNoiseVariance[1];
    processCovariance(5,5) = simCamera.angVelocityNoiseVariance[2];
    
    processCovariance(6,6) = accBiasVariance;
    processCovariance(7,7) = accBiasVariance;
    processCovariance(8,8) = accBiasVariance;
    
    processCovariance(9,9) = gyroBiasVariance;
    processCovariance(10,10) = gyroBiasVariance;
    processCovariance(11,11) = gyroBiasVariance;
    

    for (int i=processNoiseSize; i<processCovariance.rows(); i++)
    {
        processCovariance(i,i) = inverseDepthVariance;
    }
    
    return processCovariance;
}

int UKF::numUninitializedLandmarks()
{
    int count = 0;
    for (std::map<int, std::vector<int> >::iterator iter = lmIndex.begin(); iter != lmIndex.end(); iter++)
    {
        if (iter->second[1] > 0)
        {
            count ++;
        }
    }
    
    return count;
}

int UKF::getLandmarkIndex(int i)
{   
    return deviceStateSize + (i * landmarkSize);
}

Eigen::VectorXd UKF::getColumn(Eigen::MatrixXd M, int colIndex)
{
    Eigen::VectorXd column(M.rows());
    
    //std::cout << "number of rows = " << M.rows() << std::endl;
    for (int i=0; i<M.rows(); i++)
    {
        column(i, 0) = M(i, colIndex);
    }
    
    return column;
}

double UKF::meanWeight(int index, double degree)
{
    double lam = (alpha * alpha) * (degree + beta) - degree;
    
    if (index == 0)
    {
        return lam / (lam + degree);
    }
  
    return 1.0 / (2.0*(lam + degree));
}

double UKF::covarianceWeight(int index, double degree)
{
    if (index == 0)
    {
        return meanWeight(0, degree) + (1.0 - (alpha * alpha) + beta);
    }
    
    return meanWeight(index, degree);
}

void UKF::generateSigmaPoints(Eigen::VectorXd stVector, Eigen::MatrixXd covMatrix, std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> >& sigPoints)
{
    sigPoints.clear();
    double N = stVector.rows();
    double lambda = (alpha * alpha) * (N + beta) - N;
    
    // Scale and square root augmented state covariance
    Eigen::MatrixXd scaledStateCovariance = (lambda + N) * covMatrix;
    Eigen::LLT<Eigen::MatrixXd> lDecomp(scaledStateCovariance);
    Eigen::MatrixXd S = lDecomp.matrixL();
       
    sigPoints.push_back(stVector); // Add the mean
    
    for (int i=0; i<N; i++) // Add the spread points
    {
        Eigen::VectorXd column = getColumn(S, i);    
        Eigen::VectorXd sigmaPoint = stVector + column; // +
        sigPoints.push_back(sigmaPoint);
        
        sigmaPoint = stVector - column; // -
        sigPoints.push_back(sigmaPoint);
    }
}

void UKF::normalizeDirection()
{
  Eigen::Quaterniond dir(stateVector[6], stateVector[7], stateVector[8], stateVector[9]);
  dir.normalize();
  
  stateVector[6] = dir.w();
  stateVector[7] = dir.x();
  stateVector[8] = dir.y();
  stateVector[9] = dir.z();
}

void UKF::augment()
{
    int stateSize = stateVector.rows();
    stateVector.conservativeResize(stateVector.rows() + processNoiseSize + lmIndex.size());
    //Reset all noise to 0 in State
    for (int i=stateSize; i<stateVector.rows(); i++)
    {
        stateVector(i) = 0.0;
    }
    
    Eigen::MatrixXd tmpCovariance;
    tmpCovariance.resize(stateVector.rows(), stateVector.rows());
    clear(tmpCovariance);
    tmpCovariance.block(0,0, stateCovariance.rows(), stateCovariance.cols()) = stateCovariance;
    
    Eigen::MatrixXd processCovariance = getProcessCovariance();
    tmpCovariance.block(stateCovariance.rows(), stateCovariance.cols(), processCovariance.rows(), processCovariance.cols()) = processCovariance;
    
    stateCovariance = tmpCovariance;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//                           PROCESS UPDATE WITH CONTROL
/////////////////////////////////////////////////////////////////////////////////////////////
void UKF::processUpdate(double deltaT, Eigen::VectorXd control)
{
    int tmpStateSize = stateVector.rows();
    augment();    
    unscentedTransform(stateVector, stateCovariance,  &UKF::processFunction, deltaT, control);
    
    // De-augment everything
    stateVector.conservativeResize(tmpStateSize);
    stateCovariance.conservativeResize(tmpStateSize, tmpStateSize);
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        sigmaPoints[i].conservativeResize(tmpStateSize);
    }  
    
    normalizeDirection();
    
    //Decrement number of initialization steps needed
    for (std::map<int, std::vector<int> >::iterator iter = lmIndex.begin(); iter != lmIndex.end(); iter++)
    {
        iter->second[1]--;
    }
    
    generateSigmaPoints(stateVector, stateCovariance, sigmaPoints);
    
    Eigen::Vector3d accBias;
    accBias[0] = stateVector[17];
    accBias[1] = stateVector[18];
    accBias[2] = stateVector[19];
    Eigen::Vector3d gyroBias;
    gyroBias[0] = stateVector[20];
    gyroBias[1] = stateVector[21];
    gyroBias[2] = stateVector[22];
}

void UKF::processFunction(Eigen::VectorXd& sigmaPoint, double deltaT, Eigen::VectorXd control)
{
    Eigen::Vector3d position;
    position[0] = sigmaPoint[0];
    position[1] = sigmaPoint[1];
    position[2] = sigmaPoint[2];
    
    Eigen::Vector3d velocity;
    velocity[0] = sigmaPoint[3];
    velocity[1] = sigmaPoint[4];
    velocity[2] = sigmaPoint[5];
    
    Eigen::Vector3d accBias;
    accBias[0] = sigmaPoint[17];
    accBias[1] = sigmaPoint[18];
    accBias[2] = sigmaPoint[19];
    
    Eigen::Vector3d gyroBias;
    gyroBias[0] = sigmaPoint[20];
    gyroBias[1] = sigmaPoint[21];
    gyroBias[2] = sigmaPoint[22];
    
    Eigen::Quaterniond imuDir(sigmaPoint[6], sigmaPoint[7], sigmaPoint[8], sigmaPoint[9]);

    int stateSize = deviceStateSize + lmIndex.size() * landmarkSize;
    Eigen::Vector3d accelerationNoise;
    accelerationNoise[0] = sigmaPoint[stateSize];
    accelerationNoise[1] = sigmaPoint[stateSize + 1];
    accelerationNoise[2] = sigmaPoint[stateSize + 2];
    
    Eigen::Vector3d angVelocityNoise;
    angVelocityNoise[0] = sigmaPoint[stateSize + 3];
    angVelocityNoise[1] = sigmaPoint[stateSize + 4];
    angVelocityNoise[2] = sigmaPoint[stateSize + 5];
    
    Eigen::Vector3d accBiasNoise;
    accBiasNoise[0] = sigmaPoint[stateSize + 6];
    accBiasNoise[1] = sigmaPoint[stateSize + 7];
    accBiasNoise[2] = sigmaPoint[stateSize + 8];
    
    Eigen::Vector3d gyroBiasNoise;
    gyroBiasNoise[0] = sigmaPoint[stateSize + 9];
    gyroBiasNoise[1] = sigmaPoint[stateSize + 10];
    gyroBiasNoise[2] = sigmaPoint[stateSize + 11];
    
    Eigen::Vector3d accControl = control.segment(0, 3);
    Eigen::Vector3d angVelocityControl = control.segment(3,3);
    
    // Convert to inputs to world coordinates
    Eigen::Matrix3d rotMat;
    rotMat = imuDir;
    accControl = rotMat.transpose() * accControl;
    angVelocityControl = rotMat.transpose() * angVelocityControl;    
    
    Eigen::Vector3d timeSliceVelocity = (accControl - accelerationNoise - accBias) * deltaT;
    position = position + (velocity + timeSliceVelocity) * deltaT;
    velocity = velocity + timeSliceVelocity;
    
    // Compute new direction
    angVelocityControl = angVelocityControl - angVelocityNoise - gyroBias;
    imuDir = getQuaternionFromAngVelocity(angVelocityControl, deltaT) * imuDir;
     
    // Put process results back into the sigma point.
    sigmaPoint[0] = position.x();
    sigmaPoint[1] = position.y();
    sigmaPoint[2] = position.z();
    sigmaPoint[3] = velocity.x();
    sigmaPoint[4] = velocity.y();
    sigmaPoint[5] = velocity.z();
    sigmaPoint[6] = imuDir.w();
    sigmaPoint[7] = imuDir.x();
    sigmaPoint[8] = imuDir.y();
    sigmaPoint[9] = imuDir.z();
    // Skip to biases
    sigmaPoint[17] = sigmaPoint[17] + accBiasNoise[0] * deltaT;
    sigmaPoint[18] = sigmaPoint[18] + accBiasNoise[1] * deltaT;
    sigmaPoint[19] = sigmaPoint[19] + accBiasNoise[2] * deltaT;
    sigmaPoint[20] = sigmaPoint[20] + gyroBiasNoise[0] * deltaT;
    sigmaPoint[21] = sigmaPoint[21] + gyroBiasNoise[1] * deltaT;
    sigmaPoint[22] = sigmaPoint[22] + gyroBiasNoise[2] * deltaT;
    
    int inverseDepthNoiseIndex = stateSize + 5;
    for (std::map<int, std::vector<int> >::iterator iter = lmIndex.begin(); iter != lmIndex.end(); iter++)
    {
        if (iter->second[1] > 0)
        {
            // Add inverse depth noise
            int inverseDepthIndex = iter->second[0] + 5;
            inverseDepthNoiseIndex++;
            double inverseDepthNoise = sigmaPoint[inverseDepthNoiseIndex];
            sigmaPoint[inverseDepthIndex] = sigmaPoint[inverseDepthIndex] + inverseDepthNoise;
        }
    }
}

Measurement UKF::filterNewLandmarks(Measurement &actualMeasurement)
{
    Measurement tmpMeasurement = actualMeasurement;
    Measurement newLandmarks;
    
    // Remove all known landmarks from measurement
    for (std::map<int, std::vector<int> >::iterator iter = lmIndex.begin(); iter != lmIndex.end(); iter++)
    {
        if (tmpMeasurement.contains(iter->first))
        {
            tmpMeasurement.remove(iter->first);
        }
    }
    
    // The remainder are new landmarks
    std::vector<int> tags = tmpMeasurement.getTags();
    for (int i=0; i<tags.size(); i++)
    {
        std::vector<double> pixel = tmpMeasurement.getObservation(tags[i]);
        newLandmarks.add(tags[i], pixel[0], pixel[1]);
    }
    
    // Remove new landmarks from actual measurement
    tags.clear();
    tags = newLandmarks.getTags();
    for (int i=0; i<tags.size(); i++)
    {
        actualMeasurement.remove(tags[i]);
    }
    
    return newLandmarks;
}

void UKF::removeZero(Eigen::MatrixXd &mat, double val)
{
  
    for (int row = 0; row<mat.rows(); row++)
    {
        for(int col = 0; col<mat.cols(); col++)
        {
            if (std::sqrt(mat(row,col) * mat(row,col)) < 1.0e-22)
            {
                mat(row, col) = val;
            }
        }
    }
    
}

void UKF::cleanMeasurement(std::vector<int> tags, Measurement& m)
{
    Measurement tmpM;
    for (int i=0; i<tags.size(); i++)
    {
        if(m.contains(tags[i]))
        {
            std::vector<double> pixel = m.getObservation(tags[i]);
            tmpM.add(tags[i], pixel[0], pixel[1]);
        }
    }
    
//    int removed = m.getTags().size() - tmpM.getTags().size();
//    if(removed != 0)
//    {
//        printf("Removed %d observations\n", removed);
//    }
    
    m = tmpM;
}

Measurement UKF::predictMeasurements(Measurement &actualMeasurement)
{
    Measurement newLandmarks = filterNewLandmarks(actualMeasurement);
    
    std::vector<Measurement> ms;   
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        Measurement m = predictMeasurement(sigmaPoints[i]);
        ms.push_back(m);
    }
    
    
    // Find common landmark tags for all sigma point predictions
    std::vector<int> cTags = commonTags(ms);
    // and those in common with the actual measurement
    cTags = commonTags(cTags, actualMeasurement);
    
    for (int i=0; i<ms.size(); i++)
    {
        cleanMeasurement(cTags, ms[i]);
    }
    cleanMeasurement(cTags, actualMeasurement);
        
    predictedMeasurements.clear();
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        predictedMeasurements.push_back(ms[i].toVector());
    }
    
    double N = (sigmaPoints.size() - 1) / 2.0;
    aPrioriMeasurementsMean.resize(predictedMeasurements[0].rows());
    clear(aPrioriMeasurementsMean);
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        aPrioriMeasurementsMean = aPrioriMeasurementsMean + (meanWeight(i, N) * predictedMeasurements.at(i));
    }
    
    return newLandmarks;
}

Measurement UKF::predictMeasurement(Eigen::VectorXd sigmaPoint)
{
    //printf("Predicting Measurement!\n");
    
    Measurement m;
    
    Eigen::Vector3d camPosition;
//    camPosition[0] = sigmaPoint[0];
//    camPosition[1] = sigmaPoint[1];
//    camPosition[2] = sigmaPoint[2];
    camPosition = cameraPosition(sigmaPoint);
    
    //Eigen::Quaterniond camDirection(sigmaPoint[6], sigmaPoint[7], sigmaPoint[8], sigmaPoint[9]);
    Eigen::Quaterniond camDirection = cameraDirection(sigmaPoint);
    
    Eigen::Vector3d origin;
    Eigen::Vector3d direction;
    double inverseDepth;
    
    for(std::map<int, std::vector<int> >::iterator iter = lmIndex.begin(); iter != lmIndex.end(); iter++)
    {
        int index = iter->second[0];
        origin[0] = sigmaPoint[index];
        origin[1] = sigmaPoint[index + 1];
        origin[2] = sigmaPoint[index + 2];
        double theta, phi;
        theta = sigmaPoint[index + 3];
        phi     = sigmaPoint[index + 4];
        direction = getDirectionFromAngles(theta, phi);
        inverseDepth = sigmaPoint[index+5];
        
        Eigen::Vector3d euclideanLandmark = origin + (1.0/inverseDepth) * direction;
        if (visible(camPosition, camDirection, fov, euclideanLandmark))
        {
            Eigen::Matrix3d rotMat;
            rotMat = camDirection;
            
            Eigen::Vector3d pixel;
            pixel = rotMat.transpose() * (euclideanLandmark - camPosition); // Landmark in camera coordinates
            pixel[0] = pixel.x() / pixel.z();
            pixel[1] = pixel.y() / pixel.z();
            pixel[2] = 1.0;
            pixel = simCamera.intrinsicCalibrationMatrix * pixel; //Projected landmark
            
            Eigen::Vector3d p;
            p << pixel[0], pixel[1], 1.0;       
            p = simCamera.inverseK * p;
            //p.normalize();
            
            //m.add(iter->first, pixel[0], pixel[1]);
            m.add(iter->first, p[0], p[1]);
        }
    }
    
    //m.print("Predicted measurement");
    return m;
}

bool UKF::measureLandmarks(Eigen::VectorXd sigmaPoint, Eigen::VectorXd& measurement)
{
    // Assume deaugmented sigma points
    // 2 values per observation of landmark corresponding to 2 dimensional optical sensor
    int numLandmarks = (sigmaPoint.rows() - deviceStateSize) / landmarkSize;
    measurement.resize(numLandmarks * 2);
    clear(measurement);
    
    Eigen::Vector3d camPosition;
    camPosition[0] = sigmaPoint[0];
    camPosition[1] = sigmaPoint[1];
    camPosition[2] = sigmaPoint[2];
    
    Eigen::Vector3d origin;
    Eigen::Vector3d direction;
    double inverseDepth;
    
    for (int i=0, j=0; i<numLandmarks; i++, j+=2)
    {
        int index = getLandmarkIndex(i);
        origin[0] = sigmaPoint[index];
        origin[1] = sigmaPoint[index + 1];
        origin[2] = sigmaPoint[index + 2];
        double theta, phi;
        theta = sigmaPoint[index + 3];
        phi     = sigmaPoint[index + 4];
        direction = getDirectionFromAngles(theta, phi);
        inverseDepth = sigmaPoint[index+5];
        //printf("------Sigma Point ------\n");
        //std::cout << sigmaPoint << std::endl << std::endl;
        
        Eigen::Vector3d euclideanLandmark = origin + (1.0/inverseDepth) * direction;
        //printf("Euclidean Landmark %d / %d\n", i, numLandmarks);
        //std::cout << euclideanLandmark << std::endl;
        if (euclideanLandmark(0,0) <= camPosition[0])
        {
            //Landmark behind camera
            return false;
        }
        
        Eigen::Matrix3d rotMat;
        Eigen::Quaterniond imuDir(sigmaPoint[6], sigmaPoint[7], sigmaPoint[8], sigmaPoint[9]);
        rotMat = imuDir;
        //rotMat = simCamera.direction;
        
        Eigen::Vector3d pixel;
        pixel = rotMat.transpose() * (euclideanLandmark - camPosition); // Landmark in camera coordinates
        pixel[0] = pixel.x() / pixel.z();
        pixel[1] = pixel.y() / pixel.z();
        pixel[2] = 1.0;
        pixel = simCamera.intrinsicCalibrationMatrix * pixel; //Projected landmark
        
        measurement[j] = pixel.x();
        measurement[j+1] = pixel.y();
    }
    
    print("Predicted Measurement:", measurement);
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//                           MEASUREMENT UPDATE
/////////////////////////////////////////////////////////////////////////////////////////////
void UKF::measurementUpdate(Measurement m)
{
    Measurement newLandmarks = predictMeasurements(m);
        
    Eigen::VectorXd tmpState(stateVector.rows());
    Eigen::VectorXd tmpMeasurement(aPrioriMeasurementsMean.rows());
    
    double N = (sigmaPoints.size() - 1.0) / 2.0;
    
    Pxz.resize(stateVector.rows(), aPrioriMeasurementsMean.rows());
    clear(Pxz);
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        clear(tmpState);
        tmpState = sigmaPoints[i] - stateVector;
        
        clear(tmpMeasurement);
        tmpMeasurement = predictedMeasurements[i] - aPrioriMeasurementsMean;        
        
        Pxz = Pxz + covarianceWeight(i, N) * (tmpState * tmpMeasurement.transpose());
    }
    
    Pzz.resize(aPrioriMeasurementsMean.rows(), aPrioriMeasurementsMean.rows());
    clear(Pzz);
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        clear(tmpMeasurement);
        tmpMeasurement = predictedMeasurements[i] - aPrioriMeasurementsMean;
        Pzz = Pzz + covarianceWeight(i, N) * (tmpMeasurement * tmpMeasurement.transpose());
    }
    
    Eigen::MatrixXd measurementCovariance = getMeasurementCovariance(aPrioriMeasurementsMean.rows());
    Pzz = Pzz + measurementCovariance;    
    
    K.resize(stateVector.rows(), aPrioriMeasurementsMean.rows());
    K = Pxz * Pzz.inverse();
    
    stateVector = stateVector + K * (m.toVector() - aPrioriMeasurementsMean);
    
    normalizeDirection();
    
    stateCovariance = stateCovariance - (K * Pzz * K.transpose());
    
    if (newLandmarks.size() > 0)
    {
        newLandmarks.print("New Landmarks");
        addNewLandmarks(newLandmarks, stateVector, stateCovariance);
    }
}

Eigen::Vector3d UKF::getEuclideanLandmark(int index)
{
    int i = getLandmarkIndex(index);
    
    Eigen::Vector3d origin;
    Eigen::Vector3d direction;
    double inverseDepth;
    
    origin[0] = stateVector[i];
    origin[1] = stateVector[i+1];
    origin[2] = stateVector[i+2];
    double theta, phi;
    theta = stateVector[i+3];
    phi     = stateVector[i+4];
    direction = getDirectionFromAngles(theta, phi);
    inverseDepth = stateVector[i+5];

    Eigen::Vector3d euclideanLandmark = origin + ((1.0/inverseDepth) * direction);
    
    return euclideanLandmark;
}

Eigen::Vector3d UKF::getEuclideanLandmark(int index, Eigen::VectorXd sigmaPoint)
{
    int i = getLandmarkIndex(index);
    
    Eigen::Vector3d origin;
    Eigen::Vector3d direction;
    double inverseDepth;
    
    origin[0] = sigmaPoint[i];
    origin[1] = sigmaPoint[i+1];
    origin[2] = sigmaPoint[i+2];
    double theta, phi;
    theta = sigmaPoint[i+3];
    phi     = sigmaPoint[i+4];
    direction = getDirectionFromAngles(theta, phi);
    inverseDepth = sigmaPoint[i+5];

    Eigen::Vector3d euclideanLandmark = origin + ((1.0/inverseDepth) * direction);
    
    return euclideanLandmark;
}

void UKF::getAnglesFromDirection(Eigen::Vector3d direction, double &theta, double &phi)
{
    double x = direction.x();
    double y = direction.y();
    double z = direction.z();
    
    phi = std::atan2(-y, std::sqrt(x * x + z * z));
    theta = std::atan2(x, z);
}

Eigen::Vector3d UKF::getDirectionFromAngles(double theta, double phi)
{
    Eigen::Vector3d direction;
    direction[0] = std::cos(phi) * std::sin(theta);
    direction[1] = -1.0 * std::sin(phi);
    direction[2] = std::cos(phi) * std::cos(theta);
    
    return direction;
}

Eigen::Quaterniond UKF::getQuaternionFromAngVelocity(Eigen::Vector3d angVelocity, double deltaT)
{
    double angMag = std::sqrt(angVelocity.x() * angVelocity.x() + angVelocity.y() * angVelocity.y() + angVelocity.z() * angVelocity.z());
    double theta = 0.5 * angMag * deltaT;
    double w,x,y,z;
    w = std::cos(theta);
    double s = sinc(theta)*0.5*deltaT;
    x = angVelocity.x() * s;
    y = angVelocity.y() * s;
    z = angVelocity.z() * s;
    Eigen::Quaterniond angQuat(w, x, y, z);
    
    return angQuat;
}

void UKF::unscentedTransform(Eigen::VectorXd& state, Eigen::MatrixXd& covariance, void (UKF::*process)(Eigen::VectorXd&))
{
    generateSigmaPoints(state, covariance, sigmaPoints);
    double N = state.rows();
    
    // Process sigma points
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        (this->*process)(sigmaPoints[i]);
    }
    
    // Create the new state
    state.resize(sigmaPoints[0].rows());
    clear(state);
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        double mean_weight = meanWeight(i, N);
        state = state + mean_weight * sigmaPoints[i];
    }
    
    // Create the new covariance
    Eigen::VectorXd tmpDiff;
    tmpDiff.resize(sigmaPoints[0].rows());
    clear(tmpDiff);
    covariance.resize(sigmaPoints[0].rows(), sigmaPoints[0].rows());
    clear(covariance);
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        tmpDiff = sigmaPoints[i] - state;
        covariance = covariance + covarianceWeight(i, N) * (tmpDiff * tmpDiff.transpose());
    }
}

void UKF::unscentedTransform(Eigen::VectorXd& state, Eigen::MatrixXd& covariance, void (UKF::*process)(Eigen::VectorXd&, double), double deltaT)
{
    //std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > sigPoints;
    generateSigmaPoints(state, covariance, sigmaPoints);
    double N = state.rows();
    
    // Process sigma points
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        (this->*process)(sigmaPoints[i], deltaT);
        //sigmaPoints[i].conservativeResize(sigmaPoints[i].rows() - cameraNoiseSize - numLandmarks);
    }
    
    // Create the new state
    state.resize(sigmaPoints[0].rows());
    clear(state);
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        state = state + meanWeight(i, N) * sigmaPoints[i];
    }
    
    // Create the new covariance
    Eigen::VectorXd tmpDiff;
    tmpDiff.resize(sigmaPoints[0].rows());
    clear(tmpDiff);
    covariance.resize(sigmaPoints[0].rows(), sigmaPoints[0].rows());
    clear(covariance);
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        tmpDiff = sigmaPoints[i] - state;
        covariance = covariance + covarianceWeight(i, N) * (tmpDiff * tmpDiff.transpose());
    }
}

void UKF::unscentedTransform(Eigen::VectorXd& state, Eigen::MatrixXd& covariance, void (UKF::*process)(Eigen::VectorXd&, double, Eigen::VectorXd), double deltaT, Eigen::VectorXd control)
{
    //std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > sigPoints;
    generateSigmaPoints(state, covariance, sigmaPoints);
    double N = state.rows();
    
    // Process sigma points
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        (this->*process)(sigmaPoints[i], deltaT, control);
        //sigmaPoints[i].conservativeResize(sigmaPoints[i].rows() - cameraNoiseSize - numLandmarks);
    }
    
    // Create the new state
    state.resize(sigmaPoints[0].rows());
    clear(state);
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        state = state + meanWeight(i, N) * sigmaPoints[i];
    }
    
    // Create the new covariance
    Eigen::VectorXd tmpDiff;
    tmpDiff.resize(sigmaPoints[0].rows());
    clear(tmpDiff);
    covariance.resize(sigmaPoints[0].rows(), sigmaPoints[0].rows());
    clear(covariance);
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        tmpDiff = sigmaPoints[i] - state;
        covariance = covariance + covarianceWeight(i, N) * (tmpDiff * tmpDiff.transpose());
    }
}

void UKF::addLandmarks(Eigen::VectorXd& sigmaPoint)
{
    int inLandmarkSize = 3; // u, v, inverse depth
    int outLandmarkSize = 6; // origin (3), theta, phi, inverse depth
    
    int indexStart = lmIndex.size();
    int numNewLandmarks = ((sigmaPoint.rows() - deviceStateSize) - (lmIndex.size() * outLandmarkSize)) / inLandmarkSize;
    
    Eigen::VectorXd outState(sigmaPoint.rows());
    outState = sigmaPoint;
    outState.conservativeResize(sigmaPoint.rows() + (outLandmarkSize - inLandmarkSize) * numNewLandmarks);
    
    Eigen::Vector3d origin;
    origin = cameraPosition(sigmaPoint);
    
    Eigen::Quaterniond camDirection = cameraDirection(sigmaPoint);
        
    int in = deviceStateSize + lmIndex.size() * landmarkSize;
    int out = deviceStateSize + lmIndex.size() * landmarkSize;
    for (int inIndex = in, outIndex = out, i =0; i < numNewLandmarks; inIndex += inLandmarkSize, outIndex += outLandmarkSize, i++)
    {
        Eigen::Vector3d landmarkDirection;
        //landmarkDirection << sigmaPoint[inIndex], sigmaPoint[inIndex + 1], simCamera.defaultFocalLength;
        landmarkDirection << sigmaPoint[inIndex], sigmaPoint[inIndex + 1], 1.0;
        
        Eigen::Matrix3d rotMat;
        rotMat = camDirection;
        landmarkDirection = rotMat * landmarkDirection;
        
        landmarkDirection.normalize();
        
        double theta, phi;
        getAnglesFromDirection(landmarkDirection, theta, phi);
        
        double inverseDepth = sigmaPoint[inIndex + 2];
        
        outState.segment(outIndex, 3) = origin;
        outState[outIndex + 3] = theta;
        outState[outIndex + 4] = phi;
        outState[outIndex + 5] = inverseDepth;
    }
    
    sigmaPoint.resize(outState.rows());
    sigmaPoint = outState;
}

void UKF::addNewLandmarks(Measurement m, Eigen::VectorXd& state, Eigen::MatrixXd& covariance)
{
    int initialNumLandmarks = (state.rows() - deviceStateSize) / landmarkSize;
    
    Eigen::Vector3d position;
    position << state[0], state[1], state[2];
    
    Eigen::Quaterniond dir(state[6], state[7], state[8], state[9]);
    
    Eigen::Matrix3d rotMat;
    rotMat = dir;
        
    std::vector<int> tags = m.getTags();
    for (int i=0; i<tags.size(); i++)
    {       
        std::vector<double> pixel = m.getObservation(tags[i]);
        
        Eigen::Vector3d p;
        p << pixel[0], pixel[1], 1.0;
        //p.normalize();
                        
        state.conservativeResize(state.rows() + 3);
//        state[state.rows()-3] = pixel[0];
//        state[state.rows()-2] = pixel[1];
        state[state.rows()-3] = p[0];
        state[state.rows()-2] = p[1];
        state[state.rows()-1] = 1.0 / defaultDepth;
        
        Eigen::MatrixXd tmpCovariance;
        tmpCovariance.resize(covariance.rows() + 3, covariance.rows() + 3);
        clear(tmpCovariance);
        tmpCovariance.block(0,0, covariance.rows(), covariance.cols()) = covariance;
        covariance.resize(covariance.rows() + 3, covariance.rows() + 3);
        covariance = tmpCovariance;
        covariance(covariance.rows()-3, covariance.rows()-3) = simCamera.measurementNoiseVariance.x();
        covariance(covariance.rows()-2, covariance.rows()-2) = simCamera.measurementNoiseVariance.y();
        covariance(covariance.rows()-1, covariance.rows()-1) = inverseDepthVariance;
    }
    
     unscentedTransform(state, covariance, &UKF::addLandmarks);
     
     for (int i=0; i<tags.size(); i++)
     {
         std::vector<int> indexAndInit;
         indexAndInit.push_back(getLandmarkIndex(initialNumLandmarks + i));
         int initSteps = initializeSteps;
         indexAndInit.push_back(initSteps);
         lmIndex[tags[i]] = indexAndInit;
     }
}