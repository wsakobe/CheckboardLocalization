#pragma once
#ifndef __SHI_TOMASICORNERDETECTOR_H__
#define __SHI_TOMASICORNERDETECTOR_H__
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"
#include <opencv2/highgui/highgui_c.h>
#include <iostream>
#include <cstdlib>
#include <algorithm>
#include <vector>

using namespace cv;

extern std::vector<Point2f> corners;

class CornerShiTomasi
{
private:
    int maxCorners = 100;
    int corr[100][2];
    int Start_x, Start_y;
    int End_x, End_y;
    int MinDis = 100000;
    int MinPos = 0;
    int MaxDis = 0;
    int MaxPos = 0;

public:
    void cornerShiTomasi_demo(Mat dst, Mat dst1, Mat img, void*);
    void CheckCircle(Mat dst1, Mat img, int c1, int c2);
    
};

#endif
