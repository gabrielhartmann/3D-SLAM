/* 
 * File:   simScene.hpp
 * Author: gabe
 *
 * Created on 29 March 2012, 9:53 AM
 */

#ifndef SIMSCENE_HPP
#define	SIMSCENE_HPP

#include "Eigen/Dense"
#include <vector>

class SimScene
{
public:
    SimScene();
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > landmarks;
};


#endif	/* SIMSCENE_HPP */

