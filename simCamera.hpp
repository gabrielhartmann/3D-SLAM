/* 
 * File:   simCamera.hpp
 * Author: gabe
 *
 * Created on 28 March 2012, 2:59 PM
 */

#ifndef SIMCAMERA_HPP
#define	SIMCAMERA_HPP

#include <eigen3/Eigen/Dense>
#include "simScene.hpp"
#include "landmark.hpp"

typedef Eigen::Matrix<double, 6, 1> Vector6d;

class SimCamera
{
public:
    SimCamera();
    SimCamera(SimScene simScene);
    void timeStep();
    Eigen::VectorXd measure(SimScene simScene);
    void reset();
    
    Eigen::Vector3d position();
    std::vector<Landmark> map;
    
//private:
    double currTime;
    
    Eigen::Vector3d camInitialVelocity;
    Eigen::Vector3d camInitialPosition;
    
    Eigen::Vector3d camPosition;
    Eigen::Vector3d camDirection;
    Eigen::Vector3d camVelocity;
    Eigen::Vector3d camAcceleration;
    
    Eigen::Vector3d camPositionNoiseMean;
    Eigen::Vector3d camPositionNoiseVariance;
    
    Eigen::Vector3d camVelocityNoiseMean;
    Eigen::Vector3d camVelocityNoiseVariance;
    
    Eigen::Vector3d camAccelerationNoiseMean;
    Eigen::Vector3d camAccelerationNoiseVariance;
    
    Eigen::Vector3d camMeasurementNoiseMean;
    Eigen::Vector3d camMeasurementNoiseVariance;
    
    void addNoise2Acceleration();
    void addNoise2Position();
    void addNoise2Measurement(Eigen::Vector3d& imagePoint);
    
    void initializeMap(SimScene simScene);
    double angleV2V(Eigen::Vector3d v1, Eigen::Vector3d v2);
    
    double defaultInverseDepth;
    double defaultTimeStep;
};

#endif	/* SIMCAMERA_HPP */

