#include "crossMarkDetector.hpp"

using namespace std;
using namespace cv;

Mat img, img1, img2, img_stereo;

const int imageWidth = 640;                             //摄像头的分辨率  
const int imageHeight = 480;
Size imageSize = Size(imageWidth, imageHeight);

Rect validROIL;//图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域  
Rect validROIR;

Mat mapLx, mapLy, mapRx, mapRy;     //映射表  
Mat Rl, Rr, Pl, Pr, Q;              //校正旋转矩阵R，投影矩阵P 重投影矩阵Q

Mat cameraMatrixL = (Mat_<double>(3, 3) << 802.701207888995, 0, 622.575207525117,
    0, 800.420237559492, 395.903381579618,
    0, 0, 1);
Mat distCoeffL = (Mat_<double>(5, 1) << 0.139395252589193, -0.157734461777669, 0.00000, 0.00000, 0.00000);

Mat cameraMatrixR = (Mat_<double>(3, 3) << 799.315400747869, 0, 636.395903640885,
    0, 798.192031709909, 387.126770053478,
    0, 0, 1);
Mat distCoeffR = (Mat_<double>(5, 1) << 0.126360293112346, -0.133870292188476, 0.00000, 0.00000, 0.00000);

Mat T = (Mat_<double>(3, 1) << -133.333218739670, -1.09804644225190, 2.23322531078641);//T平移向量
//Mat rec = (Mat_<double>(3, 1) << -0.00306, -0.03207, 0.00206);//rec旋转向量
Mat R = (Mat_<double>(3, 3) << 0.999685749875707, -0.0205805033034650, 0.0143123855181751,
    0.0204751117383895, 0.999762444054992, 0.00747163010927865,
    -0.0144627554340543, -0.00717623445586060, 0.999869656687455);//R 旋转矩阵

int main(int argc, char* argv[]) {
    const char* imagename = "./img/03.bmp";//此处为测试图片路径
    FILE* stream1;
    freopen_s(&stream1, "linkTabel.txt", "r", stdin);
    
    stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
        0, imageSize, &validROIL, &validROIR);
    initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
    initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);
    /*
    VideoCapture capture1(1);
    capture1 >> img1;
    VideoCapture capture2(2);
    capture2 >> img2;

    //转换为灰度图
    cvtColor(img1, img1, COLOR_BGR2GRAY);
    img1.convertTo(img1, CV_32FC1); img1 = img1 / 255;
    cvtColor(img2, img2, COLOR_BGR2GRAY);
    img2.convertTo(img2, CV_32FC1); img2 = img2 / 255;

    img_stereo.create(img1.rows, img1.cols * 2, CV_32FC1);
  
    crossMarkDetectorParams Dparams;
    Dparams.height = img1.rows;
    Dparams.width = img1.cols * 2;
    crossPointResponderParams Rparams;
    crossMarkDetector filter(Dparams, Rparams);
    
    while (true){
        capture1 >> img1;
        capture2 >> img2;

        if (img1.empty() || img2.empty()) {
            fprintf(stderr, "Can not load image %s\n", imagename);
            return -1;
        }

        //转换为灰度图
        cvtColor(img1, img1, COLOR_BGR2GRAY);
        img1.convertTo(img1, CV_32FC1); img1 = img1 / 255;
        cvtColor(img2, img2, COLOR_BGR2GRAY);
        img2.convertTo(img2, CV_32FC1); img2 = img2 / 255;

        remap(img1, img1, mapLx, mapLy, INTER_LINEAR);
        remap(img2, img2, mapRx, mapRy, INTER_LINEAR);

        //合并成一幅图
        img1.copyTo(img_stereo(Range(0, img1.rows), Range(0, img1.cols)));
        img2.copyTo(img_stereo(Range(0, img1.rows), Range(img1.cols, img1.cols * 2)));
        
        //棋盘格提取
        filter.feed(img_stereo);

        waitKey(1);
    } 
        */
    // 处理单幅图片
    Mat img = imread(imagename);
    cvtColor(img, img, COLOR_BGR2GRAY);
    img.convertTo(img, CV_32FC1); img = img / 255;
      
    crossMarkDetectorParams Dparams;
    Dparams.height = img.rows;
    Dparams.width = img.cols;
    crossPointResponderParams Rparams;

    crossMarkDetector filter(Dparams, Rparams);
    filter.feed(img);
   
    waitKey(0);
    
    return 0;
}
