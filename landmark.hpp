/* 
 * File:   landmark.hpp
 * Author: gabe
 *
 * Created on 29 March 2012, 12:13 PM
 */

#ifndef LANDMARK_HPP
#define	LANDMARK_HPP

#include "Eigen/Dense"

class Landmark
{
public:
    
    Landmark();
    Landmark(Eigen::Vector3d origin, Eigen::Vector3d direction, double inverseDepth);
    Eigen::Vector3d position();
    Eigen::Vector3d origin;
    Eigen::Vector3d direction;
    double inverseDepth;
};

#endif	/* LANDMARK_HPP */

