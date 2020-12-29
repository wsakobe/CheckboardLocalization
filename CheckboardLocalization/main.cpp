#include "crossMarkDetector.hpp"

using namespace std;
using namespace cv;

Mat img;

int main(int argc, char* argv[]) {
    const char* imagename = "C:\\Users\\wsa\\Desktop\\123.bmp";//此处为测试图片路径
    /*
    VideoCapture capture(0);
    img = imread(imagename);
    capture >> img;
    
    cvtColor(img, img, COLOR_BGR2GRAY);
    img.convertTo(img, CV_32FC1); img = img / 255;

    crossMarkDetectorParams Dparams;
    Dparams.height = img.rows;
    Dparams.width = img.cols;
    crossPointResponderParams Rparams;
    crossMarkDetector filter(Dparams, Rparams);
    
    while (true){
        capture >> img;

        if (img.empty()) {
            fprintf(stderr, "Can not load image %s\n", imagename);
            return -1;
        }
        
        Mat img1;
        img1 = img;
        cvtColor(img, img, COLOR_BGR2GRAY);
        
        //棋盘格提取
        img.convertTo(img, CV_32FC1); img = img / 255;
        filter.feed(img, img1);

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
