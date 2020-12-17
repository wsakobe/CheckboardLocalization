#pragma once
#ifndef __HARRISCORNERDETECTOR_H__
#define __HARRISCORNERDETECTOR_H__
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"
#include <opencv2/highgui/highgui_c.h>

using namespace cv;

class CornerHarris
{
private:
    Mat g_srcImage, g_srcImage1, g_grayImage;
    int thresh = 15; //当前阈值  
    int max_thresh = 175; //最大阈值  
    int maxTrackbar = 100;

public:
    void on_CornerHarris(Mat dst, Mat img, void*);

};

#endif