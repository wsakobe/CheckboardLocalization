#include "opencv2/opencv.hpp"
#include "FindChessboardCorner.h"
#include "fstream"

using namespace cv;
using namespace std;

void FindCorner::FindCornerAccu(Mat image)
{
	Size boardSize, imageSize;
	boardSize.width = 5;
	boardSize.height = 5;
	int winSize = 11;
	//vector<vector<Point2f> > imagePoints;
	
	vector<Point2f> pointbuf;
	bool found;
	/*found = findChessboardCorners(image, boardSize, pointbuf,
		CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_FILTER_QUADS);*/
	found = findChessboardCornersSB(image, boardSize, pointbuf,
		CALIB_CB_EXHAUSTIVE | CALIB_CB_ACCURACY);
	if (found)
		/*cornerSubPix(imageGray, pointbuf, Size(winSize, winSize),
			Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));*/
		drawChessboardCorners(image, boardSize, Mat(pointbuf), found);

	cv::namedWindow("image", WINDOW_NORMAL);
	cv::imshow("image", image);
	cv::waitKey(1);
}