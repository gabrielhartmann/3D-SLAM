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

class Device
{
public:
    Device();
    Device(SimScene simScene);
    
    SimScene simScene;
    
    void timeStep();
    
    void reset();
    
    Eigen::Vector3d getPosition();
    Eigen::Vector3d getVelocity();
    Eigen::Vector3d getAcceleration();
    Eigen::Quaterniond getImuDirection();
    Eigen::Vector3d getAngularVelocity();
    
    void draw();
    void drawVelocity();
    void drawAcceleration();
    
    
    std::vector<Landmark> map;
    
    Eigen::VectorXd control();
    Eigen::VectorXd measure(SimScene simScene);
    
    void addNoise(Eigen::Vector3d& vec, Eigen::Vector3d noiseMean, Eigen::Vector3d noiseVariance);
        
    void initializeMap(SimScene simScene);
    
    Eigen::Quaterniond initialImuDirection;
    Eigen::Matrix3d intrinsicCalibrationMatrix;
    
    double currTime;
    Eigen::Vector3d origin;
    double defaultTimeStep;
    double defaultFocalLength;
    static const double defaultFocalLengthDrawn = 4.0;
    static const double pi =3.1415926535897932384626433832795028841971693993751058;
    double sizeScale;
    
    Eigen::Vector3d measurementNoiseMean;
    Eigen::Vector3d measurementNoiseVariance;
    
    Eigen::Vector3d accelerationNoiseMean;
    Eigen::Vector3d accelerationNoiseVariance;
    
    Eigen::Vector3d angVelocityNoiseMean;
    Eigen::Vector3d angVelocityNoiseVariance;    
};

#endif	/* SIMCAMERA_HPP */

