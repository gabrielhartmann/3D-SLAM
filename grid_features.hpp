/* 
 * File:   grid_features.hpp
 * Author: gabe
 *
 * Created on 26 March 2012, 4:19 PM
 */

#ifndef GRID_FEATURES_HPP
#define	GRID_FEATURES_HPP

#include<opencv2/opencv.hpp>
#include<vector>

class GridFeatures
{
public:
	GridFeatures();
	GridFeatures(int, int);
	std::vector<cv::Point2f> getFeatures(cv::Mat img);
private:
	std::vector <cv::Point2f> _corners;
	cv::Size _patternSize;
};


#endif	/* GRID_FEATURES_HPP */

