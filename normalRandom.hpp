/* 
 * File:   normalRandom.hpp
 * Author: gabe
 *
 * Created on 29 March 2012, 10:08 AM
 */

#ifndef NORMALRANDOM_HPP
#define	NORMALRANDOM_HPP

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <math.h>

typedef boost::mt19937                     ENG;    // Mersenne Twister
typedef boost::normal_distribution<double> DIST;   // Normal Distribution
typedef boost::variate_generator<ENG,DIST> GEN;    // Variate generator

namespace
{
    ENG eng;
    DIST dist(0, 1);
    GEN gen(eng,dist);
}


double generateNoise(double mean, double variance);

#endif	/* NORMALRANDOM_HPP */

