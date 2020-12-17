#pragma once
#ifndef __FINDCHESSBOARDCORNER_H__
#define __FINDCHESSBOARDCORNER_H__
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"
#include <opencv2/highgui/highgui_c.h>
#include <vector>

using namespace cv;

extern std::vector<KeyPoint> key_points;

class FindCorner
{
public:
	void FindCornerAccu(Mat img);
};

#endif
