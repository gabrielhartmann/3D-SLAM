/* 
 * File:   ukf.hpp
 * Author: gabe
 *
 * Created on 26 March 2012, 4:30 PM
 */

#ifndef UKF_HPP
#define	UKF_HPP

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Cholesky>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "landmark.hpp"
#include "simCamera.hpp"

class ukf
{
public:
    ukf();
    ukf(SimCamera simCamera);
    void initialize();
    void step(double timeStep, Eigen::VectorXd measurement);
    Eigen::Vector3d position();
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > landmarks();
    void reset(SimCamera simCamera);
private:
    int filterStepCount;
    void cleanCovariance();
    void augmentStateVector();
    void augmentStateCovariance();
    void timeUpdate(double deltaT);
    void predictMeasurements();
    void measurementUpdate(Eigen::VectorXd measurement);
    
    SimCamera simCamera;
    
    int numLandmarks;
    const static int cameraStateSize = 4;
    const static int landmarkSize2D = 5;
    int stateSize;
    
    const static double inverseDepthVariance = 0.0625;
    const static double accelerationVariance = 0.0625;
    const static double measurementVariance = 0.025;
   
    Eigen::VectorXd stateVector;
    void initializeStateVector2D();
       
    Eigen::MatrixXd stateCovariance;
    void initializeStateCovariance2D();
    
    Eigen::MatrixXd measurementCovariance;
    void initializeMeasurementCovariance();
    
    Eigen::MatrixXd processCovariance;
    void initializeProcessCovariance();
    
    int getLandmarkIndex2D(int i);
    
    
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
    Eigen::VectorXd getColumn(Eigen::MatrixXd M, int colIndex);
    
    Eigen::VectorXd aPrioriStateMean;
    Eigen::MatrixXd aPrioriStateCovariance;
    
    
    bool measureLandmarks2D(Eigen::VectorXd sigmaPoint, Eigen::VectorXd& measurement);
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > predictedMeasurements;
    Eigen::VectorXd aPrioriMeasurementsMean;
    
    Eigen::MatrixXd Pxz;
    Eigen::MatrixXd Pzz;
    Eigen::MatrixXd K;
    
    void printStateVector(Eigen::VectorXd vector);
    void printSigmaPoints();
    void initializeMatrix2Zero(Eigen::MatrixXd& matrix);
    void initializeVector2Zero(Eigen::VectorXd& vector);
    
    Eigen::Vector2d getEuclideanLandmark(int index);
};

#endif	/* UKF_HPP */

