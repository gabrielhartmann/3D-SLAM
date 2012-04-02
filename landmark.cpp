#include "landmark.hpp"
#include <iostream>

Landmark::Landmark(){}

Landmark::Landmark(Eigen::Vector3d origin, Eigen::Vector3d direction, double inverseDepth)
{
    this->origin = origin;
    this->direction = direction.normalized();
    this->inverseDepth = inverseDepth;
}

Eigen::Vector3d Landmark::position()
{
    return origin + ((1.0/inverseDepth) * direction);
}