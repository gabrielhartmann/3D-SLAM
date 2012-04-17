/* 
 * File:   simCamera.hpp
 * Author: gabe
 *
 * Created on 28 March 2012, 2:59 PM
 */

#ifndef SIMCAMERA_HPP
#define	SIMCAMERA_HPP

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include "simScene.hpp"
#include "landmark.hpp"
#include "normalRandom.hpp"
#include "Utilities.h"
#include <stdio.h>
#include <math.h>
#include <iostream>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

class SimCamera
{
public:
    SimCamera();
    SimCamera(SimScene simScene);
    
    SimScene simScene;
    
    void timeStep();
    Eigen::VectorXd measure(SimScene simScene);
    void reset();
    
    Eigen::Vector3d getPosition();
    void draw();
    std::vector<Landmark> map;
    

    double currTime;
    
    Eigen::Vector3d initialPosition;
    Eigen::Vector3d initialVelocity;
    Eigen::Vector3d initialAcceleration;
    
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    
    Eigen::Vector3d positionNoiseMean;
    Eigen::Vector3d positionNoiseVariance;
    
    Eigen::Vector3d velocityNoiseMean;
    Eigen::Vector3d velocityNoiseVariance;
    
    Eigen::Vector3d accelerationNoiseMean;
    Eigen::Vector3d accelerationNoiseVariance;
    
    Eigen::Vector3d measurementNoiseMean;
    Eigen::Vector3d measurementNoiseVariance;
    
    void addNoise2Acceleration();
    void addNoise2Position();
    void addNoise2Measurement(Eigen::Vector3d& imagePoint);
    
    void initializeMap(SimScene simScene);
    
    double defaultInverseDepth;
    double defaultTimeStep;
    
    static const double defaultFocalLengthDrawn = 4.0;
    

    double pi;
    
    Eigen::Quaterniond direction;
    Eigen::Matrix3d intrinsicCalibrationMatrix;
    
    double focalLength;
    
    Eigen::VectorXd getStateVector();
    
    int cameraStateSize;
    int numLandmarks;
    int landmarkSize;
};

#endif	/* SIMCAMERA_HPP */

