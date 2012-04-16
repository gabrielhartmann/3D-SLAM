#include "ukf.hpp"
#include <iostream>
#include <stdio.h>
#include <eigen3/Eigen/Eigenvalues>

ukf::ukf(){}

ukf::ukf(SimCamera simCamera)
{
    
    this->simCamera = simCamera;
    numLandmarks = simCamera.map.size();

    initialize();
    
    
}

void ukf::initialize()
{
    filterStepCount = 0;
    stateSize = cameraStateSize + numLandmarks * landmarkSize2D;
    
    initializeStateVector2D();
    initializeStateCovariance2D();
    initializeProcessCovariance();
    initializeMeasurementCovariance();
    
    //Initialize sigma point scaling factor
    alpha = 0.001;
    beta = 2.0;
    N = stateSize + processCovariance.rows();
    lambda = (alpha * alpha) * (N + beta) - N;
}

void ukf::step(double timeStep, Eigen::VectorXd measurement)
{
    printf("======================================================================\n");
    printf("                                %d                                    \n", ++filterStepCount);
    printf("======================================================================\n");
    processUpdate(timeStep);
    measurementUpdate(measurement);
    //printStateVector(stateVector);
    //printf("======================================================================\n");
    //printf("                                %d                                    \n", filterStepCount);
    //printf("======================================================================\n");
}

void ukf::step(double timeStep, Eigen::VectorXd control, Eigen::VectorXd measurement)
{
    
}

Eigen::Vector3d ukf::position()
{
    Eigen::Vector3d pos(stateVector(0,0), stateVector(1,0), 0.0);
    //printf("Filtered Camera Position: (%f, %f)\n", pos(0,0), pos(1,0));
    return pos;
}

void ukf::draw()
{
    drawCamera();
    
    Color::setColor(0.0, 0.0, 8.0);
    for (int i=0; i < landmarks().size(); i++)
    {
        glPushMatrix();
        glTranslated(landmarks()[i].x(), landmarks()[i].y(), 0.0);
        glutSolidCube(2.0);
        glPopMatrix();
    }
}

void ukf::drawCamera()
{
    glPushMatrix();
    
    glTranslated(position()[0], position()[1], position()[2]);
    Eigen::AngleAxisd aa(90.0, Eigen::Vector3d::UnitY());
    glRotated(aa.angle(), aa.axis().x(), aa.axis().y(), aa.axis().z());
    
    glBegin(GL_TRIANGLE_FAN);
    
    Color::setColor(0.8, 0.8, 0.8); //white
    //glColor3d(0.8, 0.8, 0.8);
    //glNormal3d(0.0, 0.0, 1.0);
    glVertex3d(0.0, 0.0, 0.0);
    
    Color::setColor(0.0, 0.0, 8.0); //blue
    //glNormal3d(-3.0, 3.0, 0.0);
    glVertex3d(-3.0, 3.0, simCamera.defaultFocalLength);
    
    //glNormal3d(3.0, 3.0, 0.0);
    glVertex3d(3.0, 3.0, simCamera.defaultFocalLength);
    
     //glNormal3d(3.0, -3.0, 0.0);
    glVertex3d(3.0, -3.0, simCamera.defaultFocalLength);
      
    //glNormal3d(-3.0, -3.0, 0.0);
    glVertex3d(-3.0, -3.0, simCamera.defaultFocalLength);
            
    //glNormal3d(3.0, 3.0, 0.0);
    glVertex3d(-3.0, 3.0, simCamera.defaultFocalLength);
    glEnd();
    
    glPopMatrix();
}

std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > ukf::landmarks()
{
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > lms;
    for (int i=0, j=0; i<numLandmarks; i++, j+=landmarkSize2D)
    {
        lms.push_back(getEuclideanLandmark(i));
    }
    
    return lms;
}

void ukf::reset(SimCamera simCamera)
{
    this->simCamera = simCamera;
    numLandmarks = simCamera.map.size();

    initialize();
}

void ukf::initializeStateVector2D()
{
    //stateVector.resize(augmentedstateSize);
    stateVector.resize(cameraStateSize + numLandmarks * landmarkSize2D);
    initializeVector2Zero(stateVector);
    
    stateVector(0,0) = simCamera.camPosition(0,0);
    stateVector(1,0) = simCamera.camPosition(1,0);
    stateVector(2,0) = simCamera.camVelocity(0,0);
    stateVector(3,0) = simCamera.camVelocity(1,0);
    stateVector(4,0) = simCamera.camAcceleration(0,0);
    stateVector(5,0) = simCamera.camAcceleration(1,0);
    
    int mapOffset = cameraStateSize;
    for (int i=0, j=0; i<numLandmarks; i++, j+=landmarkSize2D)
    {
        Eigen::Vector3d origin = simCamera.map.at(i).origin;
        Eigen::Vector3d direction = simCamera.map.at(i).direction;
        double inverseDepth = simCamera.map.at(i).inverseDepth;
        stateVector(mapOffset + j    , 0) = origin(0,0);
        stateVector(mapOffset + j + 1, 0) = origin(1,0);
        stateVector(mapOffset + j + 2, 0) = direction(0,0);
        stateVector(mapOffset + j + 3, 0) = direction(1,0);
        stateVector(mapOffset + j + 4, 0) = inverseDepth;
    }
    
    printf("State Vector Initialized:\n");
    printStateVector(stateVector);
}

void ukf::initializeStateCovariance2D()
{
    stateCovariance.resize(stateVector.rows(), stateVector.rows());
    
    //Set to diagonal
    for (int i=0; i<stateCovariance.rows(); i++)
    {
        for (int j=0; j<stateCovariance.cols(); j++)
        {
            if(i == j)
            {
                if (i==0 || i==1 || i==2 || i==3 || i==4 || i==5) //Initial Camera position, velocity and acceleration perfectly known
                {
                    stateCovariance(i,j) = 0.0;
                }
                else
                {
                    stateCovariance(i,j) = 0.01;
                }
                
            }
            else
            {
                stateCovariance(i,j) = 0.0; 
            }
        }
    }
    
    //Set landmark variances
    for (int i=0; i<numLandmarks; i++)
    {
        int index = getLandmarkIndex2D(i);
        //Origin
        stateCovariance(index, index) = 0.0;
        stateCovariance(index + 1, index + 1) = 0.0;
        //Direction
        stateCovariance(index + 2, index + 2) = 0.0;
        stateCovariance(index + 3, index + 3) = 0.0;
        //Inverse Depth
        stateCovariance(index+4, index+4) = inverseDepthVariance;
    }
}

void ukf::initializeMeasurementCovariance()
{
    measurementCovariance.resize(numLandmarks, numLandmarks);
    initializeMatrix2Zero(measurementCovariance);
    
    for (int row=0; row<measurementCovariance.rows(); row++)
    {
        measurementCovariance(row, row) = simCamera.camMeasurementNoiseVariance(1,0); //Square matrix so can do diagonal this way
    }
}

void ukf::initializeProcessCovariance()
{
    /*
    processCovariance.resize(unaugmentedStateSize, unaugmentedStateSize);
    initializeMatrix2Zero(processCovariance);
    processCovariance(0,0) = 0.001; //Horizontal position
    processCovariance(1,0) = 0.001; //Vertical position ???
    processCovariance(2,0) = 0.001; //Horizontal velocity ??? SHOULDN'T THESE BE ALONG THE DIAGONAL?
    processCovariance(3,0) = 0.001; //Vertical velocity ???
    
    //Set landmark variances
    for (int i=0; i<numLandmarks; i++)
    {
        int index = getLandmarkIndex2D(i);
        //Origin
        processCovariance(index, index) = 0.00;
        processCovariance(index + 1, index + 1) = 0.00;
        //Direction
        processCovariance(index + 2, index + 2) = 0.00;
        processCovariance(index + 3, index + 3) = 0.00;
        //Inverse Depth
        processCovariance(index+4, index+4) = inverseDepthVariance;
    }
    */
    
    processCovariance.resize(cameraStateSize + numLandmarks, cameraStateSize + numLandmarks); //landmarks only vary by inverse depth so only one entry per landmark
    processCovariance(0,0) = simCamera.camPositionNoiseVariance(0,0);     //Horizontal position
    processCovariance(1,1) = simCamera.camPositionNoiseVariance(1,0);     //Vertical position 
    processCovariance(2,2) = simCamera.camVelocityNoiseVariance(0,0);     //Horizontal velocity
    processCovariance(3,3) = simCamera.camVelocityNoiseVariance(1,0);     //Vertical velocity
    processCovariance(4,4) = simCamera.camAccelerationNoiseVariance(0,0); //Horizontal acceleration
    processCovariance(5,5) = simCamera.camAccelerationNoiseVariance(1,0); //Vertical acceleration
    
    for (int i=cameraStateSize; i<processCovariance.rows(); i++)
    {
        processCovariance(i,i) = inverseDepthVariance;
    }
}

int ukf::getLandmarkIndex2D(int i)
{
    if (i > numLandmarks)
    {
        printf("Invalid index (%d) request in getLandmarkIndex2d\n", i);
        return -1;
    }
    
    return cameraStateSize + (i * landmarkSize2D);
}

Eigen::VectorXd ukf::getColumn(Eigen::MatrixXd M, int colIndex)
{
    Eigen::VectorXd column(M.rows());
    
    //std::cout << "number of rows = " << M.rows() << std::endl;
    for (int i=0; i<M.rows(); i++)
    {
        column(i, 0) = M(i, colIndex);
    }
    
    return column;
}

double ukf::meanWeight(int index)
{
    if (index == 0)
    {
        return lambda / (lambda + N);
    }
    
    return 1 / (2*(lambda + N));
}

double ukf::covarianceWeight(int index)
{
    if (index == 0)
    {
        return meanWeight(0) + (1 - (alpha * alpha) + beta);
    }
    
    return meanWeight(index);
}

void ukf::generateSigmaPoints(Eigen::VectorXd stVector, Eigen::MatrixXd covMatrix)
{
    sigmaPoints.clear();
    
    // Scale and square root augmented state covariance
    Eigen::MatrixXd scaledStateCovariance = (lambda + N) * covMatrix;
    //printf("The scaled state covariance:\n");
    //std::cout << scaledStateCovariance << std::endl << std::endl;
    
    Eigen::LLT<Eigen::MatrixXd> lDecomp(scaledStateCovariance);
    Eigen::MatrixXd S = lDecomp.matrixL();
    
    //printf("Square Root of the scaled state covariance\n");
    //std::cout << S << std::endl << std::endl;
    
    //printf("Check that L * L^T = original\n");
    //std::cout << S * S.transpose() << std::endl << std::endl;
    
    sigmaPoints.push_back(stVector); // Add the mean
    
    for (int i=0; i<N; i++) // Add the spread points
    {
        Eigen::VectorXd column = getColumn(S, i);
        
        Eigen::VectorXd sigmaPoint = stVector + column; // +
        sigmaPoints.push_back(sigmaPoint);
        
        sigmaPoint = stVector - column; // -
        sigmaPoints.push_back(sigmaPoint);
    }
    
    //printSigmaPoints();
}

void ukf::cleanCovariance()
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

void ukf::augmentStateVector()
{
    int noiseSize = processCovariance.rows();
    int totalSize = stateSize + noiseSize;
    stateVector.conservativeResize(totalSize);
    //Reset all noise to 0 in State
    for (int i=stateSize; i<totalSize; i++)
    {
        stateVector(i) = 0.0;
    }
    
    //printf("Augmented State Vector:\n");
    //std::cout << stateVector << std::endl << std::endl;
}

void ukf::augmentStateCovariance()
{
    stateCovariance.conservativeResize(stateVector.rows(), stateVector.rows());
    cleanCovariance();
    printf("stateCovariance is %d x %d\n", stateCovariance.rows(), stateCovariance.cols());
    printf("processNoiseCovariance is %d x %d\n", processCovariance.rows(), processCovariance.cols());
    //printf("startIndex = %d    numRows = %d\n", unaugmentedStateSize, augmentedStateSize-unaugmentedStateSize);
    //Add Q (i.e.) process noise covariance
    stateCovariance.block(stateSize, stateSize, stateCovariance.rows()-stateSize, stateCovariance.cols()-stateSize) = processCovariance;
    
    //printf("Augmented and Cleaned State Covariance:\n");
    //std::cout << stateCovariance << std::endl << std::endl;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//                           PROCESS UPDATE
/////////////////////////////////////////////////////////////////////////////////////////////
void ukf::processUpdate(double deltaT)
{
    augmentStateVector();
    augmentStateCovariance();
    
    //printf("Augmented State Vector:\n");
    //std::cout << stateVector << std::endl << std::endl;
    //printf("Augmented State Covariance:\n");
    //std::cout << stateCovariance << std::endl << std::endl;
    
    generateSigmaPoints(stateVector, stateCovariance);
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        //Remove measurement noise from sigmaPoint;
        //sigmaPoints.at(i).conservativeResize(unaugmentedStateSize+processNoiseSize);
        processFunction2D(sigmaPoints.at(i), deltaT);
        //printf("------Processed Sigma Point %d ------\n", i);
        //printStateVector(sigmaPoints.at(i));
    }
    
    //Compute a priori Mean
    aPrioriStateMean.resize(stateSize);
    initializeVector2Zero(aPrioriStateMean);
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        //Remove noise values from sigmaPoint
        sigmaPoints.at(i).conservativeResize(stateSize);
        aPrioriStateMean = aPrioriStateMean + (meanWeight(i) * sigmaPoints.at(i));
    }
    //std::cout << "The a priori state mean estimate:" << std::endl;
    //std::cout << aPrioriStateMean << std::endl << std::endl;
    
    //Compute a priori Covariance
    aPrioriStateCovariance.resize(aPrioriStateMean.rows(), aPrioriStateMean.rows());
    initializeMatrix2Zero(aPrioriStateCovariance);    
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        Eigen::VectorXd tmpState(stateVector.rows(), 1);
        initializeVector2Zero(tmpState);
        
        //printf("------ State Mean ------\n");
        //printStateVector(aPrioriStateMean);
        
        //printf("------ Sigma Point %d ------\n",i);
        //printStateVector(sigmaPoints.at(i));
        
        tmpState = sigmaPoints.at(i) - aPrioriStateMean;
        //printf("------ tmpState ------\n");
        //printStateVector(tmpState);
        aPrioriStateCovariance = aPrioriStateCovariance + covarianceWeight(i) * ( tmpState * tmpState.transpose() );
    }
    
    //Pure additive noise scenario
    //aPrioriStateCovariance = aPrioriStateCovariance + processCovariance;
    //std::cout << "The a priori state covariance:" << std::endl;
    //std::cout << aPrioriStateCovariance << std::endl;
    
    //Pure additive noise scenario
    //generateSigmaPoints(aPrioriStateMean, aPrioriStateCovariance);
    
    predictMeasurements();
    //printf("A Priori Measurement Mean:\n");
    //std::cout << aPrioriMeasurementsMean << std::endl << std::endl;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//                           PROCESS UPDATE WITH CONTROL
/////////////////////////////////////////////////////////////////////////////////////////////
void ukf::processUpdate(double deltaT, Eigen::VectorXd control)
{
    
}

void ukf::processFunction2D(Eigen::VectorXd& sigmaPoint, double deltaT)
{
    Eigen::Vector2d position;
    position(0,0) = sigmaPoint(0,0);
    position(1,0) = sigmaPoint(1,0);
    
    Eigen::Vector2d velocity;
    velocity(0,0) = sigmaPoint(2,0);
    velocity(1,0) = sigmaPoint(3,0);
    
    Eigen::Vector2d acceleration;
    acceleration(0,0) = sigmaPoint(4,0);
    acceleration(1,0) = sigmaPoint(5,0);
    
    Eigen::Vector2d positionNoise;
    positionNoise(0,0) = sigmaPoint(stateSize);
    positionNoise(1,0) = sigmaPoint(stateSize+1);
    
    Eigen::Vector2d velocityNoise;
    velocityNoise(0,0) = sigmaPoint(stateSize+2);
    velocityNoise(1,0) = sigmaPoint(stateSize+3);
    
    Eigen::Vector2d accelerationNoise;
    accelerationNoise(0,0) = sigmaPoint(stateSize+4);
    accelerationNoise(1,0) = sigmaPoint(stateSize+5);
    
    //position = position + (velocity * deltaT) + processNoise;
    acceleration = acceleration + accelerationNoise;
    velocity = velocity + velocityNoise;
    position = position + positionNoise;
    
    position = position + 
            (0.5 * acceleration * (deltaT * deltaT)) +
            (velocity * deltaT);
    
    velocity = velocity + acceleration * deltaT;
     
    
    for (int i=0; i<numLandmarks; i++)
    {
        int inverseDepthIndex = getLandmarkIndex2D(i) + 4;
        double inverseDepthNoise = sigmaPoint(stateSize + cameraStateSize + i); // Gets the ith landmark's noise
        //printf("inverseDepthNoise = %.10f\n", inverseDepthNoise);
        sigmaPoint(inverseDepthIndex, 0) = sigmaPoint(inverseDepthIndex, 0) + inverseDepthNoise;
    }
    
        
    sigmaPoint(0,0) = position(0,0);
    sigmaPoint(1,0) = position(1,0);
    sigmaPoint(2,0) = velocity(0,0);
    sigmaPoint(3,0) = velocity(1,0);
    sigmaPoint(4,0) = acceleration(0,0);
    sigmaPoint(5,0) = acceleration(1,0);
    
    //printf("Processed Sigma Point\n");
    //std::cout << sigmaPoint << std::endl << std::endl;
}

void ukf::predictMeasurements()
{
    predictedMeasurements.clear();
    //Cycle through sigma points
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        Eigen::VectorXd measurement;
        //printf("------ Predicting measurement from sigma point %d ------\n", i);
        bool inFrontofCamera = measureLandmarks2D(sigmaPoints.at(i), measurement);
        if (inFrontofCamera)
        {
            predictedMeasurements.push_back(measurement);
        }
        else
        {
            printf("*******************************************\n");
            printf("*             HUGE PROBLEM                *\n");
            printf("*        Landmark behind CAMERA           *\n");
            printf("*******************************************\n");
        }
        //printf("--- Predicted Measurement %d ------\n", i);
        //std::cout << predictedMeasurements.at(i) << std::endl;
        //printf("\n");
    }
    
    aPrioriMeasurementsMean.resize(numLandmarks);
    initializeVector2Zero(aPrioriMeasurementsMean);
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        aPrioriMeasurementsMean = aPrioriMeasurementsMean + (meanWeight(i) * predictedMeasurements.at(i));
    }
}

bool ukf::measureLandmarks2D(Eigen::VectorXd sigmaPoint, Eigen::VectorXd& measurement)
{
    //Just one value per observation of landmark corresponding to 1 dimensional optical sensor
    measurement.resize(numLandmarks);
    
    Eigen::Vector2d camPosition;
    camPosition(0,0) = sigmaPoint(0,0);
    camPosition(1,0) = sigmaPoint(1,0);
    
    Eigen::Vector2d origin;
    Eigen::Vector2d direction;
    double inverseDepth;
    
    for (int i=0; i<numLandmarks; i++)
    {
        int index = getLandmarkIndex2D(i);
        
        origin(0,0) = sigmaPoint(index, 0);
        origin(1,0) = sigmaPoint(index + 1, 0);
        direction(0,0) = sigmaPoint(index+2, 0);
        direction(1,0) = sigmaPoint(index+3, 0);
        inverseDepth = sigmaPoint(index+4, 0);
        
        
        //printf("------Sigma Point ------\n");
        //std::cout << sigmaPoint << std::endl << std::endl;
        
        Eigen::Vector2d euclideanLandmark = origin + (1.0/inverseDepth) * direction;
        //printf("Euclidean Landmark\n");
        //std::cout << euclideanLandmark << std::endl;
        if (euclideanLandmark(0,0) <= 0.0)
        {
            //Landmark behind camera
            return false;
        }
        
        
        //With focal length of 1, the measurment is just the slope
        double slope = (euclideanLandmark(1,0) - camPosition(1,0)) /
                       (euclideanLandmark(0,0) - camPosition(0,0));
        measurement(i,0) = slope;
    }
    
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//                           MEASUREMENT UPDATE
/////////////////////////////////////////////////////////////////////////////////////////////
void ukf::measurementUpdate(Eigen::VectorXd measurement)
{
    Eigen::VectorXd tmpState(stateVector.rows(), 1);
    Eigen::VectorXd tmpMeasurement(stateVector.rows(), 1);
    
    Pxz.resize(aPrioriStateMean.rows(), aPrioriMeasurementsMean.rows());
    initializeMatrix2Zero(Pxz);
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        initializeVector2Zero(tmpState);
        tmpState = sigmaPoints.at(i) - aPrioriStateMean;
        
        initializeVector2Zero(tmpMeasurement);
        tmpMeasurement = predictedMeasurements.at(i) - aPrioriMeasurementsMean;        
        
        Pxz = Pxz + covarianceWeight(i) * (tmpState * tmpMeasurement.transpose());
    }
//    std::cout << "Pxz" << std::endl;
//    std::cout << Pxz << std::endl << std::endl;
    
    Pzz.resize(aPrioriMeasurementsMean.rows(), aPrioriMeasurementsMean.rows());
    initializeMatrix2Zero(Pzz);
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        initializeVector2Zero(tmpMeasurement);
        tmpMeasurement = predictedMeasurements.at(i) - aPrioriMeasurementsMean;
        Pzz = Pzz + covarianceWeight(i) * (tmpMeasurement * tmpMeasurement.transpose());
    }
    
    // Keep in both pure additive and mixture model
    Pzz = Pzz + measurementCovariance;
    
//    std::cout << "Pzz" << std::endl;
//    std::cout << Pzz << std::endl << std::endl;
//    
//  
//    std::cout << "Pzz^-1" << std::endl;
//    std::cout << Pzz.inverse() << std::endl << std::endl;
//    
//    std::cout << "Identity?" << std::endl;
//    std::cout << Pzz * Pzz.inverse() << std::endl << std::endl;
    
    
    K.resize(stateVector.rows(), aPrioriMeasurementsMean.rows());
    K = Pxz * Pzz.inverse();
    
//    std::cout << "Measurement:" << std::endl;
//    std::cout << measurement << std::endl << std::endl;
//    
//    printf("aPrioriMeasurementsMean:\n");
//    std::cout << aPrioriMeasurementsMean << std::endl << std::endl;
//    
//    printf("measurement - aPrioriMeasurementsMean\n");
//    std::cout << measurement-aPrioriMeasurementsMean << std::endl << std::endl;
//    
//    std::cout << "Kalman Gain:" << std::endl;
//    std::cout << K << std::endl << std::endl;
//    
//    printf("Kalman Gain applied to measurement\n");
//    std::cout << K * (measurement - aPrioriMeasurementsMean) << std::endl << std::endl;
//    
//    printf("aPrioriStateMean:\n");
//    std::cout << aPrioriStateMean << std::endl <<std::endl;
    
    stateVector = aPrioriStateMean + K * (measurement - aPrioriMeasurementsMean);
    //printf("New State Vector:\n");
    //std::cout << stateVector << std::endl << std::endl;
    //printStateVector(stateVector);
    
    stateCovariance = aPrioriStateCovariance - (K * Pzz * K.transpose());
    
//    printf("A Posteriori State covariance: %d X %d\n", aPrioriStateCovariance.rows(), aPrioriStateCovariance.cols());
//    std::cout << stateCovariance << std::endl;
}

void ukf::printStateVector(Eigen::VectorXd vector)
{
    printf("Camera Position:     (%f, %f)\n", vector(0,0), vector(1,0));
    printf("Camera Velocity:     (%f, %f)\n", vector(2,0), vector(3,0));
    //printf("Camera Acceleration: (%f)\n", vector(2,0));
    
    
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > lms = landmarks();
    for (int i=0; i<numLandmarks; i++)
    {
        printf("--- Landmark %d ---\n", i);
        printf("(%f, %f)\n", lms.at(i)(0,0), lms.at(i)(1,0));
       
        int index = getLandmarkIndex2D(i);
        printf("Origin:        (%f, %f)\n", vector(index, 0), vector(index+1, 0));
        printf("Direction:     (%f, %f)\n", vector(index+2, 0), vector(index+3, 0));
        printf("Depth:         (%f) <----------------------\n", 1.0/vector(index+4, 0));
        printf("Inverse Depth: (%f)\n", vector(index+4, 0));
        
    }
    
    //printf("Acceleration Noise: (%f)\n", vector(vector.rows()-2, 0));
    //printf("Measurement  Noise: (%f)\n", vector(vector.rows()-1, 0));
    printf("\n");
}

void ukf::printSigmaPoints()
{
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        printf("------ Sigma Point %d ------\n", i);
        //printStateVector(sigmaPoints.at(i));
        std::cout << sigmaPoints.at(i) << std::endl;
        printf("----------------------------\n");
    }
}

void ukf::initializeMatrix2Zero(Eigen::MatrixXd& matrix)
{
    for (int i=0; i<matrix.rows(); i++)
    {
        for (int j=0; j<matrix.cols(); j++)
        {
            matrix(i,j) = 0.0;
        }
    }
}

void ukf::initializeVector2Zero(Eigen::VectorXd& vector)
{
    for (int i=0; i<vector.rows(); i++)
    {
        vector(i,0) = 0.0;
    }
}

Eigen::Vector2d ukf::getEuclideanLandmark(int index)
{
    int i = getLandmarkIndex2D(index);
    
    Eigen::Vector2d origin;    
    Eigen::Vector2d direction;
    double inverseDepth;
    
    origin(0,0) = stateVector(i, 0);
    origin(1,0) = stateVector(i+1, 0);
    direction(0,0) = stateVector(i+2, 0);
    direction(1,0) = stateVector(i+3, 0);
    inverseDepth = stateVector(i+4, 0);

    Eigen::Vector2d euclideanLandmark = origin + ((1.0/inverseDepth) * direction);
    /*
    printf("Euclidean Landmark %i\n", index);
    printf("Origin\n");
    std::cout << origin << std::endl;
    printf("Direction\n");
    std::cout << direction << std::endl;
    printf("Depth\n");
    std::cout << 1.0/inverseDepth << std::endl;
    printf("Landmark\n");
    std::cout << euclideanLandmark << std::endl;
    */
    return euclideanLandmark;
}