#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"
#include <opencv2/highgui/highgui_c.h>
#include "HarrisCornerDetector.h"

using namespace std;
using namespace cv;

void CornerHarris::on_CornerHarris(Mat dst, Mat img, void*)
{
    Mat dstImage;//目标图  
    Mat normImage;//归一化后的图  
    Mat scaledImage;//线性变换后的八位无符号整型的图  

    //置零当前需要显示的两幅图，即清除上一次调用此函数时他们的值  
    dstImage = Mat::zeros(dst.size(), CV_32FC1);
    g_srcImage1 = img.clone();

    //进行角点检测  
    //第三个参数表示邻域大小，第四个参数表示Sobel算子孔径大小，第五个参数表示Harris参数
    cornerHarris(dst, dstImage, 2, 3, 0.04, BORDER_DEFAULT);

    // 归一化与转换  
    normalize(dstImage, normImage, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
    convertScaleAbs(normImage, scaledImage);//将归一化后的图线性变换成8位无符号整型   

    // 将检测到的，且符合阈值条件的角点绘制出来  
    for (int j = 0; j < normImage.rows; j++)
    {
        for (int i = 0; i < normImage.cols; i++)
        {
            //Mat::at<float>(j,i)获取像素值，并与阈值比较
            if ((int)normImage.at<float>(j, i) > thresh + 80)
            {
                circle(g_srcImage1, Point(i, j), 5, Scalar(10, 10, 255), 2, 8, 0);
                circle(scaledImage, Point(i, j), 5, Scalar(0, 10, 255), 2, 8, 0);
            }
        }
    }

    imshow("角点检测", g_srcImage1);
    imshow("角点检测2", scaledImage);
}
