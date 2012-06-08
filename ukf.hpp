/* 
 * File:   ukf.hpp
 * Author: gabe
 *
 * Created on 26 March 2012, 4:30 PM
 */

#ifndef UKF_HPP
#define	UKF_HPP

#include <algorithm>
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
    Eigen::VectorXd step(double timeStep, Eigen::VectorXd control, Measurement m);
    Eigen::Vector3d imuPosition();
    Eigen::Vector3d imuPosition(Eigen::VectorXd sigmaPoint);
    Eigen::Vector3d cameraPosition();
    Eigen::Vector3d cameraPosition(Eigen::VectorXd sigmaPoint);
    Eigen::Quaterniond imuDirection();
    Eigen::Quaterniond imuDirection(Eigen::VectorXd sigmaPoint);
    Eigen::Quaterniond cameraDirection();
    Eigen::Quaterniond cameraDirection(Eigen::VectorXd sigmaPoint);
    void draw();
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > landmarks();
    
    Eigen::VectorXd getState();
    
private:
    const static int deviceStateSize = 23; // position(3), velocity(3), direction(4), imu2CameraT(3), imu2CameraQ(4), accBias(3), gyroBias(3)
    const static int landmarkSize = 6; // origin(3), theta, phi, inverse depth
    const static int processNoiseSize = 12; // translational acceleration (3), angular velocity (3), accelerometer bias(3), gyroscope bias(3)
    const static double defaultDepth = 100.0;
    double fov;
    const static double initialAccBias = 0.0;
    const static double initialGyroBias = 0.0;
    const static double accBiasVariance = 0.0001;
    const static double gyroBiasVariance = 0.0001;
    
    
    std::map<int, std::vector<int> > lmIndex;
    
    Eigen::VectorXd stateVector;
    Eigen::MatrixXd stateCovariance;
    
    void initializeStateAndCovariance();
    Eigen::MatrixXd getMeasurementCovariance(int rows);   
    Eigen::MatrixXd getProcessCovariance();
    
    int numUninitializedLandmarks();
    
    void normalizeDirection();
    void augment();
    void augmentStateVector();
    void augmentStateCovariance();
    void processUpdate(double deltaT, Eigen::VectorXd control);
    
    void constantPosition(double deltaT);
    
    void cleanMeasurement(std::vector<int> tags, Measurement &m);
    Measurement predictMeasurements(Measurement &actualMeasurement);
    Measurement predictMeasurement(Eigen::VectorXd sigmaPoint);
    void measurementUpdate(Measurement m);
    
    Measurement filterNewLandmarks(Measurement &actualMeasurement);
    void removeZero(Eigen::MatrixXd &mat, double val);
    
    //const static double inverseDepthVariance = 0.0000000001;
    //const static double inverseDepthVariance = 0.000000000000000000000001; // Room loop closing!
    //const static double inverseDepthVariance = 0.00000000000000000000000001; // 2x3 loop closing
    const static double inverseDepthVariance = 1e-310;
    const static double focalLengthVariance = 0.0001;
    const static double accelerationVariance = 0.0625;
    const static double measurementVariance = 0.025;
    const static int initializeSteps = 2;
    const static int stepsBetweenNewFeatures = 5;
    const static int numNewFeatures = 1;
    
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
    void processFunction(Eigen::VectorXd& sigmaPoint, double deltaT, Eigen::VectorXd control);
    bool measureLandmarks(Eigen::VectorXd sigmaPoint, Eigen::VectorXd& measurement);
    
    Eigen::VectorXd getColumn(Eigen::MatrixXd M, int colIndex);
       
    
    Eigen::VectorXd aPrioriMeasurementsMean;
    Eigen::MatrixXd Pxz;
    Eigen::MatrixXd Pzz;
    Eigen::MatrixXd K;
    
    Eigen::Vector3d getEuclideanLandmark(int index);
    Eigen::Vector3d getEuclideanLandmark(int index, Eigen::VectorXd sigmaPoint);
    void getAnglesFromDirection(Eigen::Vector3d direction, double &theta, double &phi);
    Eigen::Vector3d getDirectionFromAngles(double theta, double phi);
    Eigen::Quaterniond getQuaternionFromAngVelocity(Eigen::Vector3d angVelocity, double deltaT);
    
    void drawImu();
    void drawCamera();
    void drawCamera(Eigen::VectorXd sigmaPoint);
    
    void unscentedTransform(Eigen::VectorXd& state, Eigen::MatrixXd& covariance, void (UKF::*process)(Eigen::VectorXd&));
    void unscentedTransform(Eigen::VectorXd& state, Eigen::MatrixXd& covariance, void (UKF::*process)(Eigen::VectorXd&, double), double deltaT);
    void unscentedTransform(Eigen::VectorXd& state, Eigen::MatrixXd& covariance, void (UKF::*process)(Eigen::VectorXd&, double, Eigen::VectorXd), double deltaT, Eigen::VectorXd control);
    void addLandmarks(Eigen::VectorXd& state);
    void addLandmarks(int i);
    
    void addNewLandmarks(Measurement m, Eigen::VectorXd &state, Eigen::MatrixXd &covariance);
    
    void removeNegativeLandmarks();
    void removeLandmark(int i);
};

#endif	/* UKF_HPP */
