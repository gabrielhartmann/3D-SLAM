#include "normalRandom.hpp"
#include <stdio.h>

double generateNoise(double mean, double variance)
{
    //printf("variance = %f\n", variance);
    double sigma = std::sqrt(variance);
    //printf("sigma = %.10f\n", sigma);
    double random = gen();
    double noise = (random * sigma) + mean;
    //printf("noise = %f\n", noise);
    return noise;
}