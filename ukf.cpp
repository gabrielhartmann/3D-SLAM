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
    beta = 2.0;

    lmIndex.clear();
    
    initializeStateAndCovariance();    
    getProcessCovariance();
}

void UKF::step(double timeStep, Eigen::VectorXd control, Measurement m)
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
}

Eigen::Vector3d UKF::position()
{
    Eigen::Vector3d pos(stateVector[0], stateVector[1], stateVector[2]);
    return pos;
}

Eigen::Quaterniond UKF::direction()
{
    Eigen::Quaterniond dir(stateVector[6], stateVector[7], stateVector[8], stateVector[9]);
    return dir;
}

void UKF::draw()
{
    drawCamera();
    
    Color::setColor(0.0, 0.0, 8.0);
    for (int i=0; i < landmarks().size(); i++)
    {
        glPushMatrix();
        //glTranslated(landmarks()[i].x(), landmarks()[i].y(), 0.0);
        glTranslated(landmarks()[i].x(), landmarks()[i].y(), landmarks()[i].z());
        glutSolidCube(2.0);
        glPopMatrix();
    }
}

void UKF::drawCamera()
{
    glPushMatrix();
    
    glTranslated(position()[0], position()[1], position()[2]);
    
    //Eigen::AngleAxisd aa(90.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd aa;
    aa = direction();
    glRotated(aa.angle() * 180.0 / simCamera.pi, aa.axis().x(), aa.axis().y(), aa.axis().z());
    
    glBegin(GL_TRIANGLE_FAN);
    
    Color::setColor(0.8, 0.8, 0.8); //white
    //glColor3d(0.8, 0.8, 0.8);
    //glNormal3d(0.0, 0.0, 1.0);
    glVertex3d(0.0, 0.0, 0.0);
    
    Color::setColor(0.0, 0.0, 8.0); //blue
    //glNormal3d(-3.0, 3.0, 0.0);
    glVertex3d(-3.0, 3.0, simCamera.defaultFocalLengthDrawn);
    
    //glNormal3d(3.0, 3.0, 0.0);
    glVertex3d(3.0, 3.0, simCamera.defaultFocalLengthDrawn);
    
     //glNormal3d(3.0, -3.0, 0.0);
    glVertex3d(3.0, -3.0, simCamera.defaultFocalLengthDrawn);
      
    //glNormal3d(-3.0, -3.0, 0.0);
    glVertex3d(-3.0, -3.0, simCamera.defaultFocalLengthDrawn);
            
    //glNormal3d(3.0, 3.0, 0.0);
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
    state.segment(0, 3) = simCamera.getPosition();
    state.segment(3, 3) = simCamera.getVelocity();
    Eigen::Quaterniond dir = simCamera.getDirection();
    state[6] = dir.w();
    state[7] = dir.x();
    state[8] = dir.y();
    state[9] = dir.z();
          
    Eigen::MatrixXd covariance;
    covariance.resize(state.rows(), state.rows());
    clear(covariance);
    covariance(0,0) = 0.01; // Position
    covariance(1,1) = 0.01;
    covariance(2,2) = 0.01;
    covariance(3,3) = simCamera.accelerationNoiseVariance[0]; // Velocity
    covariance(4,4) = simCamera.accelerationNoiseVariance[1];
    covariance(5,5) = simCamera.accelerationNoiseVariance[2];
    covariance(6,6) = 0.0001; //Direction
    covariance(7,7) = 0.0001;
    covariance(8,8) = 0.0001;
    covariance(9,9) = 0.0001;
    
    Measurement m = simCamera.measure();
    addNewLandmarks(m, state, covariance);
    
    stateVector.resize(state.rows());
    stateVector = state;
    
    stateCovariance.resize(covariance.rows(), covariance.cols());
    stateCovariance = covariance;
    
    removeZero(stateCovariance, 0.1);
    
    print("State:", stateVector);
    printf("\n");
    print("Covariance:", stateCovariance);
    printf("\n");
}

Eigen::MatrixXd UKF::getProcessCovariance()
{
    Eigen::MatrixXd processCovariance;
    
    processCovariance.resize(processNoiseSize + lmIndex.size(), processNoiseSize + lmIndex.size());
    clear(processCovariance);
    
    processCovariance(0,0) = simCamera.accelerationNoiseVariance[0] ; // Acceleration Noise
    processCovariance(1,1) = simCamera.accelerationNoiseVariance[1];
    processCovariance(2,2) = simCamera.accelerationNoiseVariance[2];
    
    processCovariance(3,3) = simCamera.angVelocityNoiseVariance[0]; // Angular Velocity Noise
    processCovariance(4,4) = simCamera.angVelocityNoiseVariance[1];
    processCovariance(5,5) = simCamera.angVelocityNoiseVariance[2];
    
    for (int i=6; i<processCovariance.rows(); i++)
    {
        processCovariance(i,i) = inverseDepthVariance;
    }
    
//    print("Process Covariance:", processCovariance);
//    printf("\n");
    
    return processCovariance;
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

//void UKF::augmentStateVector()
//{
//    int stateSize = stateVector.rows();
//    stateVector.conservativeResize(stateVector.rows() + processNoiseSize);
//    //Reset all noise to 0 in State
//    for (int i=stateSize; i<stateVector.rows(); i++)
//    {
//        stateVector(i) = 0.0;
//    }
//}
//
//void UKF::augmentStateCovariance()
//{
//    Eigen::MatrixXd tmpCovariance;
//    tmpCovariance.resize(stateVector.rows(), stateVector.rows());
//    clear(tmpCovariance);
//    tmpCovariance.block(0,0, stateCovariance.rows(), stateCovariance.cols()) = stateCovariance;
//    tmpCovariance.block(stateCovariance.rows(), stateCovariance.cols(), processCovariance.rows(), processCovariance.cols()) = processCovariance;
//    
//    stateCovariance = tmpCovariance;
//}

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
    
    generateSigmaPoints(stateVector, stateCovariance, sigmaPoints);
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
    
    Eigen::Quaterniond direction(sigmaPoint[6], sigmaPoint[7], sigmaPoint[8], sigmaPoint[9]);

    int stateSize = deviceStateSize + lmIndex.size() * landmarkSize;
    Eigen::Vector3d accelerationNoise;
    accelerationNoise[0] = sigmaPoint[stateSize];
    accelerationNoise[1] = sigmaPoint[stateSize + 1];
    accelerationNoise[2] = sigmaPoint[stateSize + 2];
    
    Eigen::Vector3d angVelocityNoise;
    angVelocityNoise[0] = sigmaPoint[stateSize + 3];
    angVelocityNoise[1] = sigmaPoint[stateSize + 4];
    angVelocityNoise[2] = sigmaPoint[stateSize + 5];
    
    Eigen::Vector3d accControl = control.segment(0, 3);
    Eigen::Vector3d angVelocityControl = control.segment(3,3);
    
    Eigen::Vector3d timeSliceVelocity = (accControl + accelerationNoise) * deltaT;
    position = position + (velocity + timeSliceVelocity) * deltaT;
    velocity = velocity + timeSliceVelocity;
    
    // Compute new direction
    direction = getQuaternionFromAngVelocity(angVelocityControl, deltaT) * direction;
     
    // Put process results back into the sigma point.
    sigmaPoint[0] = position.x();
    sigmaPoint[1] = position.y();
    sigmaPoint[2] = position.z();
    sigmaPoint[3] = velocity.x();
    sigmaPoint[4] = velocity.y();
    sigmaPoint[5] = velocity.z();
    sigmaPoint[6] = direction.w();
    sigmaPoint[7] = direction.x();
    sigmaPoint[8] = direction.y();
    sigmaPoint[9] = direction.z();
    
    // Add inverse depth noise
    for (int i = 0; i < lmIndex.size(); i++)
    {
        int inverseDepthIndex = getLandmarkIndex(i) + 5;
        int inverseDepthNoiseIndex = stateSize + 6 + i;
        sigmaPoint[inverseDepthIndex] = sigmaPoint[inverseDepthIndex] + sigmaPoint[inverseDepthNoiseIndex];
    }
}

Measurement UKF::filterNewLandmarks(Measurement &actualMeasurement)
{
    Measurement tmpMeasurement = actualMeasurement;
    Measurement newLandmarks;
    
    // Remove all known landmarks from measurement
    for (std::map<int, int>::iterator iter = lmIndex.begin(); iter != lmIndex.end(); iter++)
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
    Measurement m;
    
    Eigen::Vector3d camPosition;
    camPosition[0] = sigmaPoint[0];
    camPosition[1] = sigmaPoint[1];
    camPosition[2] = sigmaPoint[2];
    
    Eigen::Quaterniond camDirection(sigmaPoint[6], sigmaPoint[7], sigmaPoint[8], sigmaPoint[9]);
    
    Eigen::Vector3d origin;
    Eigen::Vector3d direction;
    double inverseDepth;
    
    for(std::map<int,int>::iterator iter = lmIndex.begin(); iter != lmIndex.end(); iter++)
    {
        int index = iter->second;
        origin[0] = sigmaPoint[index];
        origin[1] = sigmaPoint[index + 1];
        origin[2] = sigmaPoint[index + 2];
        double theta, phi;
        theta = sigmaPoint[index + 3];
        phi     = sigmaPoint[index + 4];
        direction = getDirectionFromAngles(theta, phi);
        inverseDepth = sigmaPoint[index+5];
        
        Eigen::Vector3d euclideanLandmark = origin + (1.0/inverseDepth) * direction;
        if (visible(camPosition, camDirection, simCamera.fov, euclideanLandmark))
        {
            Eigen::Matrix3d rotMat;
            rotMat = camDirection;
            
            Eigen::Vector3d pixel;
            pixel = rotMat.transpose() * (euclideanLandmark - camPosition); // Landmark in camera coordinates
            pixel[0] = pixel.x() / pixel.z();
            pixel[1] = pixel.y() / pixel.z();
            pixel[2] = 1.0;
            pixel = simCamera.intrinsicCalibrationMatrix * pixel; //Projected landmark
            
            m.add(iter->first, pixel[0], pixel[1]);
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
        Eigen::Quaterniond direction(sigmaPoint[6], sigmaPoint[7], sigmaPoint[8], sigmaPoint[9]);
        rotMat = direction;
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
    origin << sigmaPoint[0], sigmaPoint[1], sigmaPoint[2];
    
    Eigen::Quaterniond camDirection(sigmaPoint[6], sigmaPoint[7], sigmaPoint[8], sigmaPoint[9]);
        
    int in = deviceStateSize + lmIndex.size() * landmarkSize;
    int out = deviceStateSize + lmIndex.size() * landmarkSize;
    for (int inIndex = in, outIndex = out, i =0; i < numNewLandmarks; inIndex += inLandmarkSize, outIndex += outLandmarkSize, i++)
    {
        Eigen::Vector3d landmarkDirection;
        landmarkDirection << sigmaPoint[inIndex], sigmaPoint[inIndex + 1], simCamera.defaultFocalLength;
        
        Eigen::Matrix3d rotMat;
        rotMat = camDirection;
        landmarkDirection = rotMat * landmarkDirection;
        
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
        
        state.conservativeResize(state.rows() + 3);
        state[state.rows()-3] = pixel[0];
        state[state.rows()-2] = pixel[1];
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
         lmIndex[tags[i]] = getLandmarkIndex(initialNumLandmarks + i);
     }
}