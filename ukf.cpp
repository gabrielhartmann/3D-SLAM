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
    augmentedstateSize = stateSize2D + numLandmarks * landmarkSize2D + stateNoiseSize2D;
    unaugmentedStateSize = augmentedstateSize - stateNoiseSize2D;
    
    initializeStateVector2D();
    initializeStateCovariance2D();
    
    //Initialize sigma point scaling factor
    alpha = 0.001;
    beta = 2.0;
    N = stateVector.rows(); //The dimension of the augmented state vector
    lambda = (alpha * alpha) * (N + beta) - N;
}

void ukf::step(double timeStep, Eigen::VectorXd measurement)
{
    printf("======================================================================\n");
    printf("                                %d                                    \n", ++filterStepCount);
    printf("======================================================================\n");
    timeUpdate(timeStep);
    measurementUpdate(measurement);
    printStateVector(stateVector);
    printf("======================================================================\n");
    printf("                                %d                                    \n", filterStepCount);
    printf("======================================================================\n");
}

Eigen::Vector3d ukf::position()
{
    Eigen::Vector3d pos(-1.0, stateVector(0,0), 0.0);
    printf("Filtered Camera Position: (%f, %f)\n", pos(0,0), pos(1,0));
    return pos;
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
    
    //Initialize sigma point scaling factor
    alpha = 0.01;
    beta = 2.0;
    N = stateVector.rows(); //The dimension of the augmented state vector
    lambda = (alpha * alpha) * (N + beta) - N;
}

void ukf::initializeStateVector2D()
{
    stateVector.resize(augmentedstateSize);
    initializeVector2Zero(stateVector);
    
    stateVector(0,0) = simCamera.camPosition(1,0);
    stateVector(1,0) = simCamera.camVelocity(1,0);
    //stateVector(2,0) = simCamera.camAcceleration(1,0);
    
    int mapOffset = stateSize2D;
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
                stateCovariance(i,j) = 0.1;
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
        //stateCovariance(index, index) = 0.0;
        //stateCovariance(index + 1, index + 1) = 0.0;
        //Direction
        //stateCovariance(index + 2, index + 2) = 0.0;
        //stateCovariance(index + 3, index + 3) = 0.0;
        //Inverse Depth
        stateCovariance(index+4, index+4) = inverseDepthVariance;
    }
    
    
    //Set acceleration variance
    //stateCovariance(stateSize2D-1, stateSize2D-1) = accelerationVariance;
    
    //Set acceleration noise variance
    //stateCovariance(stateVector.rows()-2, stateVector.rows()-2) = accelerationVariance;
    
    //Set measurement noise variance
    //stateCovariance(stateVector.rows()-1, stateVector.rows()-1) = measurementVariance;
    
    //printf("State Covariance Matrix Initialized %d x %d:\n", stateCovariance.rows(), stateCovariance.cols());
    //std::cout << stateCovariance << std::endl;
}

int ukf::getLandmarkIndex2D(int i)
{
    if (i > numLandmarks)
    {
        printf("Invalid index (%d) request in getLandmarkIndex2d\n", i);
        return -1;
    }
    
    return stateSize2D + (i * landmarkSize2D);
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

void ukf::generateSigmaPoints()
{
    sigmaPoints.clear();
    
    // Scale and square root augmented state covariance
    Eigen::MatrixXd scaledStateCovariance = (lambda + N) * stateCovariance;
    //printf("The scaled state covariance:\n");
    //std::cout << scaledStateCovariance << std::endl << std::endl;
    
    Eigen::LLT<Eigen::MatrixXd> lDecomp(scaledStateCovariance);
    Eigen::MatrixXd S = lDecomp.matrixL();
    
    //printf("Square Root of the scaled state covariance\n");
    //std::cout << S << std::endl << std::endl;
    
    //printf("Check that L * L^T = original\n");
    //std::cout << S * S.transpose() << std::endl << std::endl;
            
    
    Eigen::VectorXd tmpStateVector = stateVector;
    //std::cout << "The augmented State Vector" << std::endl;
    //std::cout << augmentedStateVector << std::endl;
    
    sigmaPoints.push_back(tmpStateVector); // Add the mean
    
    for (int i=0; i<N; i++) // Add the spread points
    {
        Eigen::VectorXd column = getColumn(S, i);
        
        Eigen::VectorXd sigmaPoint = tmpStateVector + column; // +
        sigmaPoints.push_back(sigmaPoint);
        
        sigmaPoint = tmpStateVector - column; // -
        sigmaPoints.push_back(sigmaPoint);
    }
    
    //printSigmaPoints();
}

void ukf::cleanCovariance()
{
    //Reset cross covariance of noise in state covariance     
    int row = stateCovariance.rows()-1;
    for (int i=0; i<stateNoiseSize2D; i++)
    {
        row -= i;
        for (int j=0; j<stateCovariance.cols()-2; j++)
        {
            stateCovariance(row, j) = 0.0;
            //Also do the column
            stateCovariance(j, row) = 0.0;
        }
    }
    stateCovariance(stateCovariance.rows()-1, stateCovariance.rows()-2) = 0.0;
    stateCovariance(stateCovariance.rows()-2, stateCovariance.rows()-1) = 0.0;
    
    
}

void ukf::augmentStateVector()
{
    stateVector.conservativeResize(stateSize2D + numLandmarks * landmarkSize2D + stateNoiseSize2D);
    //Reset noise to 0 in State
    stateVector(stateVector.rows()-1, 0) = 0.0;
    stateVector(stateVector.rows()-2, 0) = 0.0;
}

void ukf::augmentStateCovariance()
{
    stateCovariance.conservativeResize(augmentedstateSize, augmentedstateSize);
    stateCovariance(augmentedstateSize-2, augmentedstateSize-2) = 0.00001; //State noise variance
    stateCovariance(augmentedstateSize-1, augmentedstateSize-1) = 0.0000000000000001;; //Measurement noise variance
    
    cleanCovariance();
//    printf("Cleaned State Covariance:\n");
//    std::cout << stateCovariance << std::endl << std::endl;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//                           TIME UPDATE
/////////////////////////////////////////////////////////////////////////////////////////////
void ukf::timeUpdate(double deltaT)
{
    augmentStateVector();
    augmentStateCovariance();
    
    generateSigmaPoints();
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        processFunction2D(sigmaPoints.at(i), deltaT);
        //printf("------Processed Sigma Point %d ------\n", i);
        //printStateVector(sigmaPoints.at(i));
    }
    
    //Compute a priori Mean
    aPrioriStateMean.resize(unaugmentedStateSize); //Remove noise positions from the bottom of the state vector
    initializeVector2Zero(aPrioriStateMean);
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        //Remove noise values from sigmaPoint
        sigmaPoints.at(i).conservativeResize(unaugmentedStateSize);
        aPrioriStateMean = aPrioriStateMean + (meanWeight(i) * sigmaPoints.at(i));
    }
    //std::cout << "The a priori state mean estimate:" << std::endl;
    //std::cout << aPrioriStateMean << std::endl << std::endl;
    
    //Compute a priori Covariance
    aPrioriStateCovariance.resize(stateVector.rows() - stateNoiseSize2D, stateVector.rows() - stateNoiseSize2D);
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
    //std::cout << "The a priori state covariance:" << std::endl;
    //std::cout << aPrioriStateCovariance << std::endl;
}

void ukf::processFunction2D(Eigen::VectorXd& sigmaPoint, double deltaT)
{
    double position = sigmaPoint(0,0);
    double velocity = sigmaPoint(1,0);
    double acceleration = sigmaPoint(2,0); //Acceleration only in Y direction
    
    
    position = position + (velocity * deltaT) + sigmaPoint(sigmaPoint.rows()-2, 0);
    //position = position + velocity * deltaT;
    //velocity = velocity + (acceleration + sigmaPoint(sigmaPoint.rows()-2, 0)) * deltaT ; // Add noise
    
    //position = position + sigmaPoint(sigmaPoint.rows()-2, 0);
     
        
    sigmaPoint(0,0) = position;
    sigmaPoint(1,0) = velocity;
    sigmaPoint(2,0) = acceleration;
}

void ukf::predictMeasurements()
{
    predictedMeasurements.clear();
    //Cycle through sigma points
    for (int i=0; i<sigmaPoints.size(); i++)
    {
        Eigen::VectorXd measurement;
        bool inFrontofCamera = measureLandmarks2D(sigmaPoints.at(i), measurement);
        if (inFrontofCamera)
        {
            predictedMeasurements.push_back(measurement);
        }
        else
        {
            printf("*******************************************\n");
            printf("              HUGE PROBLEM                 \n");
            printf("         Landmark behind CAMERA            \n");
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
    camPosition(0,0) = -1.0;
    camPosition(1,0) = sigmaPoint(0,0);
    
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
        
        
        Eigen::Vector2d euclideanLandmark = origin + (1.0/inverseDepth) * direction;
        if (euclideanLandmark(0,0) <= 0.0)
        {
            //Landmark behind camera
            return false;
        }
        //printf("Euclidean Landmark\n");
        //std::cout << euclideanLandmark << std::endl;
        double slope = (euclideanLandmark(1,0) - camPosition(1,0)) /
                       (euclideanLandmark(0,0) - camPosition(0,0));
        double yIntercept = camPosition(1,0) - (slope * camPosition(0,0));
        measurement(i,0) = yIntercept - camPosition(1,0) + sigmaPoint(sigmaPoint.rows()-1, 0); //Add measurement noise
    }
    
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//                           MEASUREMENT UPDATE
/////////////////////////////////////////////////////////////////////////////////////////////
void ukf::measurementUpdate(Eigen::VectorXd measurement)
{
    predictMeasurements();
    
    Eigen::VectorXd tmpState(stateVector.rows(), 1);
    Eigen::VectorXd tmpMeasurement(stateVector.rows(), 1);
    
    Pxz.resize(unaugmentedStateSize, aPrioriMeasurementsMean.rows());
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
//    printf("New State Vector:\n");
//    std::cout << stateVector << std::endl << std::endl;
    //printStateVector(stateVector);
    
    Eigen::MatrixXd tmp;
    tmp.resize(K.rows(), K.rows());
    tmp = K * Pzz * K.transpose();
    stateCovariance = aPrioriStateCovariance - tmp;
    
//    printf("A Posteriori State covariance: %d X %d\n", aPrioriStateCovariance.rows(), aPrioriStateCovariance.cols());
//    std::cout << stateCovariance << std::endl;
}

void ukf::printStateVector(Eigen::VectorXd vector)
{
    printf("Camera Position:     (%f)\n", vector(0,0));
    printf("Camera Velocity:     (%f)\n", vector(1,0));
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
        printStateVector(sigmaPoints.at(i));
        printf("\n");
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

Eigen::MatrixXd ukf::getSquareRoot(Eigen::MatrixXd matrix)
{
   using namespace std;
   using namespace Eigen;
//   Matrix2f A, b;
//   A << 2, -1, -1, 3;
//   b << 1, 2, 3, 1;
//   cout << "Here is the matrix A:\n" << A << endl;
//   cout << "Here is the right hand side b:\n" << b << endl;
//   Matrix2f x = A.ldlt().solve(b);
//   cout << "The solution is:\n" << x << endl;
   //cout << matrix.ldlt().matrixL() << endl;
   //return matrix.ldlt().matrixL();
   
    //typedef Matrix<double, 5, 3> Matrix5x3;
    //typedef Matrix<double, 5, 5> Matrix5x5;
    //Matrix5x3 m = Matrix5x3::Random();
    cout << "Here is the matrix:" << endl << matrix << endl;
    LLT<MatrixXd> lu(matrix);
    MatrixXd L = lu.matrixL();
    
    cout << "The Cholesky factor L is" << endl << L << endl;
    cout << "To check this, let us compute L * L.transpose()" << endl;
    cout << L * L.transpose() << endl;
    cout << "This should equal the matrix" << endl;
    /*
    Eigen::FullPivLU<MatrixXd> lu(matrix);
    cout << "Here is, up to permutations, its LU decomposition matrix:"
        << endl << lu.matrixLU() << endl;
    
    cout << "Here is the L part:" << endl;
    //Matrix5x5 l = Matrix5x5::Identity();
    int rows = matrix.rows();
    int cols = matrix.cols();
    MatrixXd l = MatrixXd::Identity(10, 10);
    l.block<10, 10>(0,0).triangularView<StrictlyLower>() = lu.matrixLU();
    cout << l << endl;
    
    cout << "Here is the U part:" << endl;
    MatrixXd u = lu.matrixLU().triangularView<Upper>();
    cout << u << endl;
    cout << "Let us now reconstruct the original matrix m:" << endl;
    cout << lu.permutationP().inverse() * l * u * lu.permutationQ().inverse() << endl;
    */
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