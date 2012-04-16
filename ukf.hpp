/* 
 * File:   ukf.hpp
 * Author: gabe
 *
 * Created on 26 March 2012, 4:30 PM
 */

#ifndef UKF_HPP
#define	UKF_HPP

#include <GL/glut.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Cholesky>
#include "Color.hpp"
#include "landmark.hpp"
#include "simCamera.hpp"
#include "Utilities.h"

class ukf
{
public:
    ukf();
    ukf(SimCamera simCamera);
    void initialize();
    void step(double timeStep, Eigen::VectorXd measurement);
    void step(double timeStep, Eigen::VectorXd control, Eigen::VectorXd measurement);
    Eigen::Vector3d position();
    void draw();
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > landmarks();
    void reset(SimCamera simCamera);
private:
    int filterStepCount;
    void cleanCovariance();
    void augmentStateVector();
    void augmentStateCovariance();
    void processUpdate(double deltaT);
    void processUpdate(double deltaT, Eigen::VectorXd control);
    void predictMeasurements();
    void measurementUpdate(Eigen::VectorXd measurement);
    
    SimCamera simCamera;
    
    int numLandmarks;
    const static int cameraStateSize = 9;
    const static int landmarkSize = 7;
    int stateSize;
    
    const static double inverseDepthVariance = 0.0625;
    const static double accelerationVariance = 0.0625;
    const static double measurementVariance = 0.025;
   
    Eigen::VectorXd stateVector;
    void initializeStateVector2D();
    void initializeStateVector3D();
       
    Eigen::MatrixXd stateCovariance;
    void initializeStateCovariance2D();
    void initializeStateCovariance3D();
    
    Eigen::MatrixXd measurementCovariance;
    void initializeMeasurementCovariance();
    
    Eigen::MatrixXd processCovariance;
    void initializeProcessCovariance();
    
    int getLandmarkIndex(int i);
    
    
    // Sigma point scaling values
    double lambda;
    double alpha;
    double beta;
    double N;
    double meanWeight(int index);
    double covarianceWeight(int index);
    
    void generateSigmaPoints(Eigen::VectorXd stVector, Eigen::MatrixXd covMatrix);
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > sigmaPoints;    
    
    void processFunction2D(Eigen::VectorXd& sigmaPoint, double deltaT);
    void processFunction3D(Eigen::VectorXd& sigmaPoint, double deltaT);
    Eigen::VectorXd getColumn(Eigen::MatrixXd M, int colIndex);
    
    Eigen::VectorXd aPrioriStateMean;
    Eigen::MatrixXd aPrioriStateCovariance;
    
    
    bool measureLandmarks2D(Eigen::VectorXd sigmaPoint, Eigen::VectorXd& measurement);
    bool measureLandmarks3D(Eigen::VectorXd sigmaPoint, Eigen::VectorXd& measurement);
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > predictedMeasurements;
    Eigen::VectorXd aPrioriMeasurementsMean;
    
    Eigen::MatrixXd Pxz;
    Eigen::MatrixXd Pzz;
    Eigen::MatrixXd K;
    
    void printStateVector(Eigen::VectorXd vector);
    void printSigmaPoints();
    void initializeMatrix2Zero(Eigen::MatrixXd& matrix);
    void initializeVector2Zero(Eigen::VectorXd& vector);
    
    Eigen::Vector3d getEuclideanLandmark(int index);
    
    void drawCamera();
};

#endif	/* UKF_HPP */

