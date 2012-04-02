
#include "grid_features.hpp"

using namespace std;
using namespace cv;

GridFeatures::GridFeatures(int pointsPerRow=8, int pointsPerColumn=6)
{
	_patternSize = Size(pointsPerRow, pointsPerColumn);
}

vector<Point2f> GridFeatures::getFeatures(cv::Mat img)
{
	bool patternFound = findChessboardCorners(img, _patternSize, _corners);
	if (patternFound)
	{
		drawChessboardCorners(img, _patternSize, _corners, patternFound);

	}
	return _corners;
}