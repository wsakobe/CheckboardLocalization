#pragma once
#ifndef  CircleDetector_hpp
#define  CircleDetector_hpp
#include <cstdio>
#include <iostream>
#include <cstdlib>
#include <algorithm>
#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"
#include <opencv2/highgui/highgui_c.h>
#include "opencv2/features2d/features2d.hpp"

using namespace cv;

extern std::vector<KeyPoint> key_points;

class CircleDetect
{
public:
	void DetectCircle(Mat img);
	
};

#endif /* CircleDetector_hpp */