#include "ukf.hpp"


UKF::UKF(){}

UKF::UKF(Device simCamera, SimScene scene)
{
    
    this->simCamera = simCamera;
    this->scene = scene;
    numLandmarks = scene.landmarks.size();
    
    initialize();
}

void UKF::initialize()
{
    filterStepCount = 0;
    stateSize = deviceStateSize + numLandmarks * landmarkSize;
    
    //alpha = 0.001;
    alpha = 0.001;
    //beta = 2.0;
    beta = 2.0;

    lmIndex.clear();
    
    initializeStateAndCovariance();    
    initializeProcessCovariance();
    initializeMeasurementCovariance();
}

void UKF::step(double timeStep, Eigen::VectorXd control, Measurement m)
{
    printf("======================================================================\n");
    printf("                                %d                                    \n", ++filterStepCount);
    printf("======================================================================\n");  
    
    processUpdate(timeStep, control);    
    measurementUpdate(m);
    //print("State:", stateVector);
    //print("Covariance:", stateCovariance);
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
    for (int i=0, j=0; i<numLandmarks; i++, j+=landmarkSize)
    {
        lms.push_back(getEuclideanLandmark(i));
    }
    
    return lms;
}

void UKF::reset(Device simCamera)
{
    this->simCamera = simCamera;
    numLandmarks = simCamera.map.size();

    initialize();
}


void UKF::initializeMeasurementCovariance()
{
    measurementCovariance.resize(numLandmarks * 2, numLandmarks * 2);
    clear(measurementCovariance);
    
    for (int row=0; row<measurementCovariance.rows(); row+=2)
    {
        measurementCovariance(row, row) = simCamera.measurementNoiseVariance[1]; // Y
        measurementCovariance(row+1, row+1) = simCamera.measurementNoiseVariance[2]; // Z
    }
    
    print("Measurement covariance:", measurementCovariance);
    printf("\n");
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
    std::vector<int> tags = m.getTags();
    for (int i=0; i<tags.size(); i++)
    {
        lmIndex[tags[i]] = getLandmarkIndex(i);
        
        Eigen::Vector3d position;
        position << state[0], state[1], state[2];
        
        Eigen::Matrix3d rotMat;
        rotMat = dir;
        
        std::vector<double> pixel = m.getObservation(tags[i]);
        
        state.conservativeResize(state.rows() + 4);
        state[state.rows()-4] = pixel[0];
        state[state.rows()-3] = pixel[1];
        state[state.rows()-2] = simCamera.defaultFocalLength;
        state[state.rows()-1] = 1.0 / defaultDepth;
        
        Eigen::MatrixXd tmpCovariance;
        tmpCovariance.resize(covariance.rows() + 4, covariance.rows() + 4);
        clear(tmpCovariance);
        tmpCovariance.block(0,0, covariance.rows(), covariance.cols()) = covariance;
        covariance.resize(covariance.rows() + 4, covariance.rows() + 4);
        covariance = tmpCovariance;
        covariance(covariance.rows()-4, covariance.rows()-4) = simCamera.measurementNoiseVariance.x();
        covariance(covariance.rows()-3, covariance.rows()-3) = simCamera.measurementNoiseVariance.y();
        covariance(covariance.rows()-2, covariance.rows()-2) = focalLengthVariance;
        covariance(covariance.rows()-1, covariance.rows()-1) = inverseDepthVariance;
    }
    if (numLandmarks > 0)
    {
        unscentedTransform(state, covariance, &UKF::addLandmark);
    }
    
    stateVector.resize(state.rows());
    stateVector = state;
    
    stateCovariance.resize(covariance.rows(), covariance.cols());
    stateCovariance = covariance;
    
    print("State:", stateVector);
    printf("\n");
    print("Covariance:", stateCovariance);
    printf("\n");
}

void UKF::initializeProcessCovariance()
{
    processCovariance.resize(processNoiseSize, processNoiseSize);
    clear(processCovariance);
    
    processCovariance(0,0) = simCamera.accelerationNoiseVariance[0]; // Acceleration Noise
    processCovariance(1,1) = simCamera.accelerationNoiseVariance[1];
    processCovariance(2,2) = simCamera.accelerationNoiseVariance[2];
    
    processCovariance(3,3) = simCamera.angVelocityNoiseVariance[0]; // Angular Velocity Noise
    processCovariance(4,4) = simCamera.angVelocityNoiseVariance[1];
    processCovariance(5,5) = simCamera.angVelocityNoiseVariance[2];
    
    processCovariance(6,6) = 1.0; // Position noise
    processCovariance(7,7) = 1.0;
    processCovariance(8,8) = 1.0;
    
    print("Process Covariance:", processCovariance);
    printf("\n");
}

int UKF::getLandmarkIndex(int i)
{
    if (i > numLandmarks)
    {
        printf("Invalid index (%d) request in getLandmarkIndex2d\n", i);
        return -1;
    }
    
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
    
//    printf("# of sigma points = %d\n", sigmaPoints.size());
    //printSigmaPoints();
}

void UKF::cleanCovariance()
{
   for (int row = stateSize; row<stateCovariance.rows(); row++)
   {
       for (int col = 0; col<stateCovariance.cols(); col++)
       {
           stateCovariance(row,col) = 0.0;
       }
   }
   
   for (int row = 0; row<stateCovariance.rows(); row++)
   {
       for (int col=stateSize; col<stateCovariance.cols(); col++)
       {
           stateCovariance(row,col) = 0.0;
       }
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

void UKF::augmentStateVector()
{
    stateVector.conservativeResize(stateVector.rows() + processNoiseSize);
    //Reset all noise to 0 in State
    for (int i=stateSize; i<stateVector.rows(); i++)
    {
        stateVector(i) = 0.0;
    }
}

void UKF::augmentStateCovariance()
{
    Eigen::MatrixXd tmpCovariance;
    tmpCovariance.resize(stateVector.rows(), stateVector.rows());
    clear(tmpCovariance);
    tmpCovariance.block(0,0, stateCovariance.rows(), stateCovariance.cols()) = stateCovariance;
    tmpCovariance.block(stateCovariance.rows(), stateCovariance.cols(), processCovariance.rows(), processCovariance.cols()) = processCovariance;
    
    stateCovariance = tmpCovariance;
//    stateCovariance.conservativeResize(stateVector.rows(), stateVector.rows());
//    cleanCovariance();
//    stateCovariance.block(stateSize, stateSize, stateCovariance.rows()-stateSize, stateCovariance.cols()-stateSize) = processCovariance;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//                           PROCESS UPDATE WITH CONTROL
/////////////////////////////////////////////////////////////////////////////////////////////
void UKF::processUpdate(double deltaT, Eigen::VectorXd control)
{
    int tmpStateSize = stateVector.rows();
    augmentStateVector();
    //print("State:", stateVector);
    augmentStateCovariance();
    //print("Covariance:", stateCovariance);
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

    Eigen::Vector3d accelerationNoise;
    accelerationNoise[0] = sigmaPoint[stateSize];
    accelerationNoise[1] = sigmaPoint[stateSize+1];
    accelerationNoise[2] = sigmaPoint[stateSize+2];
    
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
    
    // Add noise to position in an attempt to negate translation drift
    Eigen::Vector3d positionNoise;
    positionNoise << sigmaPoint[sigmaPoint.rows()-3], sigmaPoint[sigmaPoint.rows()-2], sigmaPoint[sigmaPoint.rows()-1];
    position = position + positionNoise;
    
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
}

void UKF::predictMeasurements(Measurement actualMeasurement)
{
    predictedMeasurements.clear();
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        Measurement m = predictMeasurement(sigmaPoints[i]);
        predictedMeasurements.push_back(m.toVector());
    }
    
    double N = (sigmaPoints.size() - 1) / 2.0;
    aPrioriMeasurementsMean.resize(numLandmarks * 2);
    clear(aPrioriMeasurementsMean);
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        aPrioriMeasurementsMean = aPrioriMeasurementsMean + (meanWeight(i, N) * predictedMeasurements.at(i));
    }
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
    // 2 values per observation of landmark corresponding to 2 dimensional optical sensor
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
    predictMeasurements(m);
    
    Eigen::VectorXd tmpState(stateVector.rows());
    //Eigen::VectorXd tmpMeasurement(stateVector.rows());
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
    
    // Keep in both pure additive and mixture model
    Pzz = Pzz + measurementCovariance;    
    
    K.resize(stateVector.rows(), aPrioriMeasurementsMean.rows());
    K = Pxz * Pzz.inverse();
    
    //print("Kalman Gain:", K);
    
    stateVector = stateVector + K * (m.toVector() - aPrioriMeasurementsMean);
//    printf("New Position: (%.20f, %.20f %.20f)\n", stateVector[0], stateVector[1], stateVector[2]);
//    printf("\n");
    
    normalizeDirection();
    
    stateCovariance = stateCovariance - (K * Pzz * K.transpose());
    //print("Filtered State Covariance:", stateCovariance);
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
    //std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > sigPoints;
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

void UKF::addLandmark(Eigen::VectorXd& sigmaPoint)
{
    int inLandmarkSize = 4; // u, v, focal length, inverse depth
    int outLandmarkSize = 6; // origin (3), theta, phi, inverse depth
    
    Eigen::VectorXd outState(sigmaPoint.rows());
    outState = sigmaPoint;
    outState.conservativeResize(sigmaPoint.rows() + (outLandmarkSize - inLandmarkSize) * numLandmarks);
    
    //print("state in:", sigmaPoint);
    Eigen::Vector3d origin;
    origin << sigmaPoint[0], sigmaPoint[1], sigmaPoint[2];
    
    Eigen::Quaterniond camDirection(sigmaPoint[6], sigmaPoint[7], sigmaPoint[8], sigmaPoint[9]);
        
    for (int i=0; i<numLandmarks; i++)
    {
        int inIndex = deviceStateSize + i * inLandmarkSize;
        int outIndex = deviceStateSize + i * outLandmarkSize;
        Eigen::Vector3d landmarkDirection;
        landmarkDirection << sigmaPoint[inIndex], sigmaPoint[inIndex + 1], sigmaPoint[inIndex + 2];
        
        Eigen::Matrix3d rotMat;
        //rotMat = simCamera.initialImuDirection;
        rotMat = camDirection;
        landmarkDirection = rotMat * landmarkDirection;
        
        double theta, phi;
        getAnglesFromDirection(landmarkDirection, theta, phi);
        
        double inverseDepth = sigmaPoint[inIndex + 3];
        
        outState.segment(outIndex, 3) = origin;
        outState[outIndex + 3] = theta;
        outState[outIndex + 4] = phi;
        outState[outIndex + 5] = inverseDepth;
    }
    
    sigmaPoint.resize(outState.rows());
    sigmaPoint = outState;
}
