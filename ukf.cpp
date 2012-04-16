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
    stateSize = cameraStateSize + numLandmarks * landmarkSize;
    
    //initializeStateVector2D();
    initializeStateVector3D();
    //initializeStateCovariance2D();
    initializeStateCovariance3D();
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
    printf("Position = (%f, %f, %f)\n", stateVector[0], stateVector[1], stateVector[2]);
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
    //Eigen::Vector3d pos(stateVector(0,0), stateVector(1,0), 0.0);
    Eigen::Vector3d pos(stateVector[0], stateVector[1], stateVector[2]);
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
        //glTranslated(landmarks()[i].x(), landmarks()[i].y(), 0.0);
        glTranslated(landmarks()[i].x(), landmarks()[i].y(), landmarks()[i].z());
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

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > ukf::landmarks()
{
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > lms;
    for (int i=0, j=0; i<numLandmarks; i++, j+=landmarkSize)
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
    stateVector.resize(cameraStateSize + numLandmarks * landmarkSize);
    initializeVector2Zero(stateVector);
    
    stateVector(0,0) = simCamera.getPosition()(0,0);
    stateVector(1,0) = simCamera.getPosition()(1,0);
    stateVector(2,0) = simCamera.velocity(0,0);
    stateVector(3,0) = simCamera.velocity(1,0);
    stateVector(4,0) = simCamera.acceleration(0,0);
    stateVector(5,0) = simCamera.acceleration(1,0);
    
    int mapOffset = cameraStateSize;
    for (int i=0, j=0; i<numLandmarks; i++, j+=landmarkSize)
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

void ukf::initializeStateVector3D()
{
    stateVector.resize(cameraStateSize + numLandmarks * landmarkSize);
    initializeVector2Zero(stateVector);
    
    stateVector[0] = simCamera.getPosition()[0];
    stateVector[1] = simCamera.getPosition()[1];
    stateVector[2] = simCamera.getPosition()[2];
    stateVector[3] = simCamera.velocity[0];
    stateVector[4] = simCamera.velocity[1];
    stateVector[5] = simCamera.velocity[2];
    stateVector[6] = simCamera.acceleration[0];
    stateVector[7] = simCamera.acceleration[1];
    stateVector[8] = simCamera.acceleration[2];
    
    int mapOffset = cameraStateSize;
    for (int i=0, j=0; i<numLandmarks; i++, j+=landmarkSize)
    {
        Eigen::Vector3d origin = simCamera.map.at(i).origin;
        Eigen::Vector3d direction = simCamera.map.at(i).direction;
        double inverseDepth = simCamera.map.at(i).inverseDepth;
        stateVector[mapOffset + j]        = origin[0];
        stateVector[mapOffset + j + 1] = origin[1];
        stateVector[mapOffset + j + 2] = origin[2];
        stateVector[mapOffset + j + 3] = direction[0];
        stateVector[mapOffset + j + 4] = direction[1];
        stateVector[mapOffset + j + 5] = direction[2];
        stateVector[mapOffset + j + 6] = inverseDepth;
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
        int index = getLandmarkIndex(i);
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

void ukf::initializeStateCovariance3D()
{
    stateCovariance.resize(stateVector.rows(), stateVector.rows());
    
    //Set to diagonal
    for (int i=0; i<stateCovariance.rows(); i++)
    {
        for (int j=0; j<stateCovariance.cols(); j++)
        {
            if(i == j)
            {
                if (i >= 0 && i < cameraStateSize) //Initial Camera position, velocity and acceleration perfectly known
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
        int index = getLandmarkIndex(i);
        //Origin
        stateCovariance(index, index) = 0.0;
        stateCovariance(index + 1, index + 1) = 0.0;
        stateCovariance(index + 2, index + 2) = 0.0;
        //Direction
        stateCovariance(index + 3, index + 3) = 0.0;
        stateCovariance(index + 4, index + 4) = 0.0;
        stateCovariance(index + 5, index + 5) = 0.0;
        //Inverse Depth
        stateCovariance(index+6, index+6) = inverseDepthVariance;
    }
}

void ukf::initializeMeasurementCovariance()
{
    measurementCovariance.resize(numLandmarks * 2, numLandmarks * 2);
    initializeMatrix2Zero(measurementCovariance);
    
    for (int row=0; row<measurementCovariance.rows(); row+=2)
    {
        measurementCovariance(row, row) = simCamera.measurementNoiseVariance[1]; // Y
        measurementCovariance(row+1, row+1) = simCamera.measurementNoiseVariance[2]; // Z
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
    initializeMatrix2Zero(processCovariance);
    processCovariance(0,0) = simCamera.positionNoiseVariance(0,0);     //Horizontal position
    processCovariance(1,1) = simCamera.positionNoiseVariance(1,0);     //Vertical position 
    processCovariance(2,2) = simCamera.positionNoiseVariance(2,0);     //Depth position 
    processCovariance(3,3) = simCamera.velocityNoiseVariance(0,0);     //Horizontal velocity
    processCovariance(4,4) = simCamera.velocityNoiseVariance(1,0);     //Vertical velocity
    processCovariance(5,5) = simCamera.velocityNoiseVariance(2,0);     //Depth velocity
    processCovariance(6,6) = simCamera.accelerationNoiseVariance(0,0); //Horizontal acceleration
    processCovariance(7,7) = simCamera.accelerationNoiseVariance(1,0); //Vertical acceleration
    processCovariance(8,8) = simCamera.accelerationNoiseVariance(2,0); //Depthl acceleration
    
    for (int i=cameraStateSize; i<processCovariance.rows(); i++)
    {
        processCovariance(i,i) = inverseDepthVariance;
    }
    
    print("Process Covariance:", processCovariance);
}

int ukf::getLandmarkIndex(int i)
{
    if (i > numLandmarks)
    {
        printf("Invalid index (%d) request in getLandmarkIndex2d\n", i);
        return -1;
    }
    
    return cameraStateSize + (i * landmarkSize);
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
    Eigen::LLT<Eigen::MatrixXd> lDecomp(scaledStateCovariance);
    Eigen::MatrixXd S = lDecomp.matrixL();
       
    sigmaPoints.push_back(stVector); // Add the mean
    
    for (int i=0; i<N; i++) // Add the spread points
    {
        Eigen::VectorXd column = getColumn(S, i);    
        Eigen::VectorXd sigmaPoint = stVector + column; // +
        sigmaPoints.push_back(sigmaPoint);
        
        sigmaPoint = stVector - column; // -
        sigmaPoints.push_back(sigmaPoint);
    }
    
//    printf("# of sigma points = %d\n", sigmaPoints.size());
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
    
//    printf("Enlarged state covariance:\n");
//    std::cout << stateCovariance << std::endl;
    
    printf("stateCovariance is %d x %d\n", stateCovariance.rows(), stateCovariance.cols());
    printf("processNoiseCovariance is %d x %d\n", processCovariance.rows(), processCovariance.cols());
    //printf("startIndex = %d    numRows = %d\n", unaugmentedStateSize, augmentedStateSize-unaugmentedStateSize);
    //Add Q (i.e.) process noise covariance
    stateCovariance.block(stateSize, stateSize, stateCovariance.rows()-stateSize, stateCovariance.cols()-stateSize) = processCovariance;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//                           PROCESS UPDATE
/////////////////////////////////////////////////////////////////////////////////////////////
void ukf::processUpdate(double deltaT)
{
    augmentStateVector();
    augmentStateCovariance();
    
    generateSigmaPoints(stateVector, stateCovariance);
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        //Remove measurement noise from sigmaPoint;
        //sigmaPoints.at(i).conservativeResize(unaugmentedStateSize+processNoiseSize);
        processFunction3D(sigmaPoints.at(i), deltaT);
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
        int inverseDepthIndex = getLandmarkIndex(i) + 4;
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

void ukf::processFunction3D(Eigen::VectorXd& sigmaPoint, double deltaT)
{
    Eigen::Vector3d position;
    position[0] = sigmaPoint[0];
    position[1] = sigmaPoint[1];
    position[2] = sigmaPoint[2];
    
    Eigen::Vector3d velocity;
    velocity[0] = sigmaPoint[3];
    velocity[1] = sigmaPoint[4];
    velocity[2] = sigmaPoint[5];
    
    Eigen::Vector3d acceleration;
    acceleration[0] = sigmaPoint[6];
    acceleration[1] = sigmaPoint[7];
    acceleration[2] = sigmaPoint[8];
    

    Eigen::Vector3d positionNoise;
    positionNoise[0] = sigmaPoint[stateSize];
    positionNoise[1] = sigmaPoint[stateSize+1];
    positionNoise[2] = sigmaPoint[stateSize+2];
    
    Eigen::Vector3d velocityNoise;
    velocityNoise[0] = sigmaPoint[stateSize+3];
    velocityNoise[1] = sigmaPoint[stateSize+4];
    velocityNoise[2] = sigmaPoint[stateSize+5];
    
    Eigen::Vector3d accelerationNoise;
    accelerationNoise[0] = sigmaPoint[stateSize+6];
    accelerationNoise[1] = sigmaPoint[stateSize+7];
    accelerationNoise[2] = sigmaPoint[stateSize+8];
    
    //position = position + (velocity * deltaT) + processNoise;
    acceleration = acceleration + accelerationNoise;
    velocity = velocity + velocityNoise;
    position = position + positionNoise;
    
//     if (accelerationNoise.x() != 0.0)
//    {
//        printf("Acceleration Noise: (%f, %f, %f)\n", accelerationNoise.x(), accelerationNoise.y(), accelerationNoise.z());
//        printf("Acceleration after noise: (%f, %f, %f)\n", acceleration.x(), acceleration.y(), acceleration.z());
//    }
    
    position = position + 
            (0.5 * acceleration * (deltaT * deltaT)) +
            (velocity * deltaT);
    
    velocity = velocity + acceleration * deltaT;
     
//    printf("Position after noise: (%.20f, %f, %f)\n", position.x(), position.y(), position.z());
//    printf("\n");
    
    // Put process results back into the sigma point.
    sigmaPoint[0] = position.x();
    sigmaPoint[1] = position.y();
    sigmaPoint[2] = position.z();
    sigmaPoint[3] = velocity.x();
    sigmaPoint[4] = velocity.y();
    sigmaPoint[5] = velocity.z();
    sigmaPoint[6] = acceleration.x();
    sigmaPoint[7] = acceleration.y();
    sigmaPoint[8] = acceleration.z();
    
    for (int i=0; i<numLandmarks; i++)
    {
        int inverseDepthIndex = getLandmarkIndex(i) + 6;
        double inverseDepthNoise = sigmaPoint(stateSize + cameraStateSize + i); // Gets the ith landmark's noise
        //printf("inverseDepthNoise = %.10f\n", inverseDepthNoise);
        sigmaPoint[inverseDepthIndex] = sigmaPoint[inverseDepthIndex] + inverseDepthNoise;
    }
    
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
        //printf("------ Predicting measurement from sigma point %d / %d ------\n", i, sigmaPoints.size());
//        bool inFrontofCamera = measureLandmarks2D(sigmaPoints.at(i), measurement);
        bool inFrontofCamera = measureLandmarks3D(sigmaPoints.at(i), measurement);
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
    
    aPrioriMeasurementsMean.resize(numLandmarks * 2);
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
        int index = getLandmarkIndex(i);
        
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

bool ukf::measureLandmarks3D(Eigen::VectorXd sigmaPoint, Eigen::VectorXd& measurement)
{
    // 2 values per observation of landmark corresponding to 2 dimensional optical sensor
    measurement.resize(numLandmarks * 2);
    initializeVector2Zero(measurement);
    
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
        direction[0] = sigmaPoint[index+3];
        direction[1] = sigmaPoint[index+4];
        direction[2] = sigmaPoint[index+5];
        inverseDepth = sigmaPoint[index+6];
        //printf("------Sigma Point ------\n");
        //std::cout << sigmaPoint << std::endl << std::endl;
        
        Eigen::Vector3d euclideanLandmark = origin + (1.0/inverseDepth) * direction;
        //printf("Euclidean Landmark %d / %d\n", i, numLandmarks);
        //std::cout << euclideanLandmark << std::endl;
        if (euclideanLandmark(0,0) <= 0.0)
        {
            //Landmark behind camera
            return false;
        }
        
        Eigen::Matrix3d rotMat;
        rotMat = simCamera.direction;
        
        Eigen::Vector3d pixel;
        pixel = rotMat.transpose() * (euclideanLandmark - position()); // Landmark in camera coordinates
        pixel[0] = pixel.x() / pixel.z();
        pixel[1] = pixel.y() / pixel.z();
        pixel[2] = 1.0;
        pixel = simCamera.intrinsicCalibrationMatrix * pixel; //Projected landmark
        
        measurement[j] = pixel.x();
        measurement[j+1] = pixel.y();
//        printf("pixel @ %d = (%.20f, %.20f)\n", j, pixel.x(), pixel.y());
//        print("Measurement:", measurement);
//        printf("\n");
    }
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//                           MEASUREMENT UPDATE
/////////////////////////////////////////////////////////////////////////////////////////////
void ukf::measurementUpdate(Eigen::VectorXd measurement)
{
//    Eigen::VectorXd temp = measurement - aPrioriMeasurementsMean;
//    print("Measurement residual:", temp);
//    printf("\n");
    
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
//    printf("New Position: (%.20f, %.20f %.20f)\n", stateVector[0], stateVector[1], stateVector[2]);
//    printf("\n");
    
    stateCovariance = aPrioriStateCovariance - (K * Pzz * K.transpose());
    
//    printf("A Posteriori State covariance: %d X %d\n", aPrioriStateCovariance.rows(), aPrioriStateCovariance.cols());
//    std::cout << stateCovariance << std::endl;
}

void ukf::printStateVector(Eigen::VectorXd vector)
{
    printf("Camera Position:     (%f, %f, %f)\n", vector[0], vector[1], vector[2]);
    printf("Camera Velocity:     (%f, %f, %f)\n", vector[3], vector[4], vector[5]);
    printf("Camera Acceleration:     (%f, %f, %f)\n", vector[6], vector[7], vector[8]);    
    
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > lms = landmarks();
    for (int i=0; i<numLandmarks; i++)
    {
        printf("--- Landmark %d ---\n", i);
        printf("(%f, %f, %f)\n", lms.at(i)[0], lms.at(i)[1], lms.at(i)[2]);
    }
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

Eigen::Vector3d ukf::getEuclideanLandmark(int index)
{
    int i = getLandmarkIndex(index);
    
    Eigen::Vector3d origin;    
    Eigen::Vector3d direction;
    double inverseDepth;
    
    origin[0] = stateVector[i];
    origin[1] = stateVector[i+1];
    origin[2] = stateVector[i+2];
    direction[0] = stateVector[i+3];
    direction[1] = stateVector[i+4];
    direction[2] = stateVector[i+5];
    inverseDepth = stateVector[i+6];

    Eigen::Vector3d euclideanLandmark = origin + ((1.0/inverseDepth) * direction);
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