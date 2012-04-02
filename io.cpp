
#include "io.hpp"
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;

ImageReader::ImageReader(string imageDir)
{
	directory = imageDir;

	namespace fs = boost::filesystem;
	fs::path imgDir(imageDir);
	fs::directory_iterator end_iter;
	list<string> imgNames;

	if (fs::exists(imgDir) && fs::is_directory(imgDir))
	{
		for(fs::directory_iterator dir_iter(imgDir); dir_iter != end_iter; dir_iter++)
		{
			if (boost::algorithm::iequals(fs::extension(dir_iter->path()), ".ppm"))
			{
				string path = dir_iter->path().string();
				imgNames.push_back(path);
			}
		}
		imgNames.sort();
	}

	imageNames.insert(imageNames.begin(), imgNames.begin(), imgNames.end());

	numImages = imageNames.size();

	currImgIndex = 0;
}

cv::Mat ImageReader::getNextImage()
{
	//string imageName = directory + "frame" + boost::lexical_cast<string>(currImgIndex) + ".ppm";
	string imageName = imageNames[currImgIndex];
	currImgIndex++;

	printf("%s\n",imageName.c_str());

	return cv::imread(imageName);
}