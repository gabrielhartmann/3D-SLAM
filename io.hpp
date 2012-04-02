/* 
 * File:   io.hpp
 * Author: gabe
 *
 * Created on 26 March 2012, 4:19 PM
 */

#ifndef IO_HPP
#define	IO_HPP

#include <string>
#include <list>
#include <opencv2/opencv.hpp>

class ImageReader
{
public:
	ImageReader(std::string imageDir);
	double getTimestamp();
	bool hasNext();
	int numImages;
	cv::Mat getNextImage();
private:
	std::string directory;
	int currImgIndex;
	std::vector<std::string> imageNames;
};

#endif	/* IO_HPP */

