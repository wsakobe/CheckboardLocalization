#include<cstdio>
#include<iostream>
#include<cstdlib>
#include<algorithm>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"
#include <opencv2/highgui/highgui_c.h>
#include "Shi_TomasiCornerDetector.h"

using namespace std;
using namespace cv;

struct CenterPoint {
    int err;
    int pos;
}Cp[100];
CenterPoint Sum[100];

bool comparison(const CenterPoint &a, const CenterPoint &b)
{
    return a.err < b.err;
}

void CornerShiTomasi::cornerShiTomasi_demo(Mat dst, Mat dst1, Mat img, void*)
{
    if (maxCorners < 1) { maxCorners = 1; }
    /// Parameters for Shi-Tomasi algorithm
    
    double qualityLevel = 0.1;
    double minDistance = 35;
    int blockSize = 5;
    bool useHarrisDetector = false;
    double k = 0.04;
    vector<Point2f> corners;
    /// Copy the source image
    Mat cormat;
    /// Apply corner detection :Determines strong corners on an image.
    goodFeaturesToTrack(dst1,
        corners,
        maxCorners,
        qualityLevel,
        minDistance,
        Mat(),
        blockSize,
        useHarrisDetector,
        k);
    
    /*
    //指定亚像素计算迭代标注
    TermCriteria criteria = TermCriteria(
        TermCriteria::MAX_ITER + TermCriteria::EPS,
        40,
        0.01);

    //亚像素检测
    cornerSubPix(dst1, corners, Size(5, 5), Size(-1, -1), criteria);
    */

    /// Draw corners detected
    for (int i = 0; i < corners.size(); i++)
        circle(img, corners[i], 4, Scalar(0, 255, 0), 2, 8, 0);
    
    for (int i = 0; i < corners.size(); i++)
    {
        corr[i][0] = corners[i].x;
        corr[i][1] = corners[i].y;
        
        Cp[i].err = 1000;
        Cp[i].pos = i;
        if ((corr[i][0] > 100) && (corr[i][1] > 100) && (corr[i][0] < 500) && (corr[i][1] < 500))
            Cp[i].err = abs(corr[i][0] - 320) + abs(corr[i][1] - 240);
    }
    
    sort(Cp, Cp + corners.size(), comparison);
    
    for (int i = 0; i < 4; i++) {
        //circle(img, corners[Cp[i].pos], 5, Scalar(255), 2, 8, 0);
        if (MinDis > (corr[Cp[i].pos][0] + corr[Cp[i].pos][1])) {
            MinDis = corr[Cp[i].pos][0] + corr[Cp[i].pos][1];
            Start_x = corr[Cp[i].pos][0];
            Start_y = corr[Cp[i].pos][1];
            MinPos = Cp[i].pos;
        }
        if (MaxDis < (corr[Cp[i].pos][0] + corr[Cp[i].pos][1])) {
            MaxDis = corr[Cp[i].pos][0] + corr[Cp[i].pos][1];
            End_x = corr[Cp[i].pos][0];
            End_y = corr[Cp[i].pos][1];
            MaxPos = Cp[i].pos;
        }
    }
   
    //imshow("Result Shi-Tomasi", img);
}

void CornerShiTomasi::CheckCircle(Mat dst1, Mat img, int c1, int c2){
    int SampleCount = 50, WhiteCnt = 0, BlackCnt = 0;
    int Thres = 15;
    for (int i = 2; i <= SampleCount - 2; i++) {
        int x = (double)(corr[c1][0] * ((double)i / SampleCount)) + (double)(corr[c2][0] * (1.0 - (double)i / SampleCount));
        int y = (double)(corr[c1][1] * ((double)i / SampleCount)) + (double)(corr[c2][1] * (1.0 - (double)i / SampleCount));
        if ((x >= 0) && (y >= 0) && (x < 640) && (y < 480) && (dst1.at<uchar>(y, x) < 120))
            BlackCnt++;
        else
            WhiteCnt++;
        if (abs(BlackCnt - WhiteCnt) < Thres) 
            printf("%d %d 1\n", BlackCnt, WhiteCnt);
        else
            printf("%d %d 0\n", BlackCnt, WhiteCnt);
    }
}