/* 
 * File:   Measurement.hpp
 * Author: gabe
 *
 * Created on 14 May 2012, 11:29 AM
 */

#ifndef MEASUREMENT_HPP
#define	MEASUREMENT_HPP

#include <eigen3/Eigen/Dense>
#include <map>
#include <vector>

class Measurement
{
public:
    Measurement();
    void clear();
    bool add(int lmTag, double u, double v);
    bool remove(int lmTag);
    bool contains(int lmTag);
    int size();
    std::vector<int> getTags();
    std::vector<double> getObservation(int tag);
    
    Eigen::VectorXd toVector();
private:
    std::map<int, std::vector<double> > m;
};

#endif	/* MEASUREMENT_HPP */

