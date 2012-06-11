/* 
 * File:   simScene.hpp
 * Author: gabe
 *
 * Created on 29 March 2012, 9:53 AM
 */

#ifndef SIMSCENE_HPP
#define	SIMSCENE_HPP

#include <vector>
#include <GL/glut.h>
#include "Eigen/Dense"
#include "Color.hpp"


class SimScene
{
public:
    SimScene();
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > landmarks;    
    void draw();
    void drawAxes();
    double roomRadius;
private:
    
    static const int axesLength = 150;
    
    void drawLandmarks();
};


#endif	/* SIMSCENE_HPP */

