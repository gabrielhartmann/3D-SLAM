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
#include <eigen3/Eigen/Cholesky>
#include <GL/glut.h>
#include <math.h>
#include <stdio.h>
#include <vector>

#include "Color.hpp"
//#include "landmark.hpp"
#include "Measurement.hpp"
#include "Device.hpp"
#include "Utilities.h"

class UKF
{
public:
    UKF();
    UKF(Device simCamera, SimScene scene);
    void initialize();
    void step(double timeStep, Eigen::VectorXd control, Measurement m);
    Eigen::Vector3d position();
    Eigen::Quaterniond direction();
    void draw();
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > landmarks();
    void reset(Device simCamera);
    
private:
    int numLandmarks;
    const static int deviceStateSize = 10; // position(3), velocity(3), imu direction (4)
    const static int landmarkSize = 6; // origin(3), theta, phi, inverse dpeth
    const static int processNoiseSize = 9; // translational accleration (3), angular velocity (3), position (3)
    const static double defaultDepth = 100.0;
    int stateSize;
    
    std::map<int, int> lmIndex;
    
    Eigen::VectorXd stateVector;
    Eigen::MatrixXd stateCovariance;
    Eigen::MatrixXd measurementCovariance;
    Eigen::MatrixXd processCovariance;
    
    void initializeStateAndCovariance();
    void initializeMeasurementCovariance();   
    void initializeProcessCovariance();
    
    void cleanCovariance();
    void normalizeDirection();
    void augmentStateVector();
    void augmentStateCovariance();
    void processUpdate(double deltaT, Eigen::VectorXd control);
    void predictMeasurements(Measurement actualMeasurement);
    Measurement predictMeasurement(Eigen::VectorXd sigmaPoint);
    void measurementUpdate(Measurement m);
    
    const static double inverseDepthVariance = 0.1;
    const static double focalLengthVariance = 0.0001;
    const static double accelerationVariance = 0.0625;
    const static double measurementVariance = 0.025;
    
    int filterStepCount;
        
    Device simCamera;
    SimScene scene;
       
    int getLandmarkIndex(int i);
    
    // Sigma point scaling values
    double alpha;
    double beta;
    double meanWeight(int index, double degree);
    double covarianceWeight(int index, double degree);
    
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > sigmaPoints; 
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > predictedMeasurements;
    
    void generateSigmaPoints(Eigen::VectorXd stVector, Eigen::MatrixXd covMatrix, std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> >& sigPoints);
    void processFunction(Eigen::VectorXd& sigmaPoint, double deltaT);
    void processFunction(Eigen::VectorXd& sigmaPoint, double deltaT, Eigen::VectorXd control);
    bool measureLandmarks(Eigen::VectorXd sigmaPoint, Eigen::VectorXd& measurement);
    
    Eigen::VectorXd getColumn(Eigen::MatrixXd M, int colIndex);
       
    
    Eigen::VectorXd aPrioriMeasurementsMean;
    Eigen::MatrixXd Pxz;
    Eigen::MatrixXd Pzz;
    Eigen::MatrixXd K;
    
    Eigen::Vector3d getEuclideanLandmark(int index);
    void getAnglesFromDirection(Eigen::Vector3d direction, double &theta, double &phi);
    Eigen::Vector3d getDirectionFromAngles(double theta, double phi);
    Eigen::Quaterniond getQuaternionFromAngVelocity(Eigen::Vector3d angVelocity, double deltaT);
    
    void drawCamera();
    
    void unscentedTransform(Eigen::VectorXd& state, Eigen::MatrixXd& covariance, void (UKF::*process)(Eigen::VectorXd&));
    void unscentedTransform(Eigen::VectorXd& state, Eigen::MatrixXd& covariance, void (UKF::*process)(Eigen::VectorXd&, double), double deltaT);
    void unscentedTransform(Eigen::VectorXd& state, Eigen::MatrixXd& covariance, void (UKF::*process)(Eigen::VectorXd&, double, Eigen::VectorXd), double deltaT, Eigen::VectorXd control);
    void addLandmark(Eigen::VectorXd& state);
    void addLandmark(int i);
};

#endif	/* UKF_HPP */
