#include "crossMarkDetector.hpp"
#include <stdio.h>
#include <process.h>
#include <conio.h>
#include "mvcameracontrol.h"

using namespace std;
using namespace cv;

Mat img, img1, img2, img_stereo;

const int imageWidth = 1920;  //摄像头的分辨率  
const int imageHeight = 1200;
Size imageSize = Size(imageWidth, imageHeight);

Rect validROIL;//图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域  
Rect validROIR;

Mat mapLx, mapLy, mapRx, mapRy;     //映射表  
Mat Rl, Rr, Pl, Pr, Q;              //校正旋转矩阵R，投影矩阵P 重投影矩阵Q

//HikVision Camera Preparation
int nRet = MV_OK;
int nRet1 = MV_OK;
int nRet2 = MV_OK;
void* handle1 = NULL;
void* handle2 = NULL;
unsigned char* pData;
unsigned int g_nPayloadSize = 0;
MV_FRAME_OUT_INFO_EX* imageInfo;
Mat Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char* pData);

Mat img_left, img_right, img_left_undistort, img_right_undistort;
vector<Mat>vImgs;

double start, start_last;
double fps;
bool QButtonPressed = false, ZButtonPressed = false;
std::vector<Point3f> endEffectorWorldPoints;

int main(int argc, char* argv[]) {
    const char* imagename = "76.bmp";//此处为测试图片路径

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    while (stDeviceList.nDeviceNum != 2)
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("Enum Devices fail! nRet [0x%x]\n", nRet);
    }
    
    nRet1 = MV_CC_CreateHandle(&handle1, stDeviceList.pDeviceInfo[0]);
    nRet2 = MV_CC_CreateHandle(&handle2, stDeviceList.pDeviceInfo[1]);
    if ((MV_OK != nRet1) || (MV_OK != nRet2))
    {
        printf("Create Handle fail! nRet [0x%x]\n", nRet1);
    }

    nRet1 = MV_CC_OpenDevice(handle1);
    nRet2 = MV_CC_OpenDevice(handle2);
    if ((MV_OK != nRet1) || (MV_OK != nRet2))
    {   
        printf("Open Device fail! nRet [0x%x]\n", nRet1);
    }

    nRet1 = MV_CC_SetEnumValue(handle1, "TriggerMode", 0);
    nRet2 = MV_CC_SetEnumValue(handle2, "TriggerMode", 0);
    if ((MV_OK != nRet1) || (MV_OK != nRet2))
    {
        printf("Set Enum Value fail! nRet [0x%x]\n", nRet1);
    }

    // Get payload size
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle1, "PayloadSize", &stParam);
    g_nPayloadSize = stParam.nCurValue;

    nRet1 = MV_CC_StartGrabbing(handle1);
    nRet2 = MV_CC_StartGrabbing(handle2);
    if ((MV_OK != nRet1) || (MV_OK != nRet2))
    {
        printf("Start Grabbing fail! nRet [0x%x]\n", nRet1);
    }

    MV_FRAME_OUT_INFO_EX stImageInfo1 = { 0 };
    memset(&stImageInfo1, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    MV_FRAME_OUT_INFO_EX stImageInfo2 = { 0 };
    memset(&stImageInfo2, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned char* pData1 = (unsigned char*)malloc(sizeof(unsigned char) * (g_nPayloadSize));
    unsigned char* pData2 = (unsigned char*)malloc(sizeof(unsigned char) * (g_nPayloadSize));
    int cnt = 1;

    while (1) {
        if (kbhit()) {
            char ch = getch();
            if (ch == 27) {
                nRet1 = MV_CC_CloseDevice(handle1);
                nRet2 = MV_CC_CloseDevice(handle2);
                if ((MV_OK != nRet1) || (MV_OK != nRet2))
                {
                    printf("Close Device fail! nRet [0x%x]\n", nRet1);
                    break;
                }

                // Destroy handle
                nRet1 = MV_CC_DestroyHandle(handle1);
                nRet2 = MV_CC_DestroyHandle(handle2);
                if ((MV_OK != nRet1) || (MV_OK != nRet2))
                {
                    printf("Destroy Handle fail! nRet [0x%x]\n", nRet1);
                    break;
                }
                printf("Device successfully closed.\n");
                break;
            }
            if (ch == 113) {
                QButtonPressed = true;
                printf("Q\n");
            }
            if (ch == 122){
                ZButtonPressed ^= true ;
                printf("Z\n");
            }
            ch = 0;
        }
        nRet1 = MV_CC_GetOneFrameTimeout(handle1, pData1, g_nPayloadSize, &stImageInfo1, 100);
        nRet2 = MV_CC_GetOneFrameTimeout(handle2, pData2, g_nPayloadSize, &stImageInfo2, 100);
        if ((MV_OK == nRet1) && (MV_OK == nRet2)) {
            Mat img_left = Convert2Mat(&stImageInfo1, pData1);
            Mat img_right = Convert2Mat(&stImageInfo2, pData2);
            start = getTickCount();
            fps = 1000000.0 * (double)cvGetTickFrequency() / (start - start_last);
            start_last = start;
            img_left.convertTo(img_left, CV_32FC1); img_left = img_left / 255;
            img_right.convertTo(img_right, CV_32FC1); img_right = img_right / 255;

            undistort(img_left, img_left_undistort, cameraMatrixL, distCoeffL, cameraMatrixL);
            undistort(img_right, img_right_undistort, cameraMatrixR, distCoeffR, cameraMatrixR);

            vImgs.push_back(img_left_undistort);
            vImgs.push_back(img_right_undistort);
            hconcat(vImgs, img_stereo);

            crossMarkDetectorParams Dparams;
            Dparams.height = img_stereo.rows;
            Dparams.width = img_stereo.cols;
            crossPointResponderParams Rparams;
            //imshow("img_stereo", img_stereo);
            //imwrite("img_stereo.bmp", img_stereo * 255);

            crossMarkDetector filter(Dparams, Rparams);
            filter.feed(img_stereo, cnt++, fps, QButtonPressed, ZButtonPressed, endEffectorWorldPoints);

            img_stereo.release();
            vImgs.clear();
            QButtonPressed = false;
            waitKey(1);
        }
    }
 
    /*VideoCapture capture1(0);
    VideoCapture capture2(2);

    crossMarkDetectorParams Dparams;
    Dparams.height = imageHeight;
    Dparams.width = imageWidth;
    crossPointResponderParams Rparams;
    crossMarkDetector filter(Dparams, Rparams);*/
    
    /*int nRet = MV_OK;
    prepareImageRead(nRet, handle);
    MV_FRAME_OUT stImageInfo = { 0 };
    MV_DISPLAY_FRAME_INFO stDisplayInfo = { 0 };*/

    //while (true){
    //    nRet = MV_CC_GetImageBuffer(handle, &stImageInfo, 1000);
    //    if (nRet == MV_OK)
    //    {
    //        printf("Get Image Buffer: Width[%d], Height[%d], FrameNum[%d]\n",
    //            stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nFrameNum);

    //        img1 = Convert2Mat(&stImageInfo.stFrameInfo, stImageInfo.pBufAddr);
    //    capture1 >> img1;

    //        if (img1.empty() /*|| img2.empty()*/) {
    //            fprintf(stderr, "Can not load image %s\n", imagename);
    //            return -1;
    //        }

    //        //转换为灰度图
    //        cvtColor(img1, img1, COLOR_BGR2GRAY);
    //        img1.convertTo(img1, CV_32FC1); img1 = img1 / 255;
    //        /*cvtcolor(img2, img2, color_bgr2gray);
    //        img2.convertto(img2, cv_32fc1); img2 = img2 / 255;

    //        img1.copyto(img_stereo(range(0, img1.rows), range(0, img1.cols)));
    //        img2.copyto(img_stereo(range(0, img1.rows), range(img1.cols, img1.cols * 2)));
    //        imwrite("imgor.bmp", 255 * img_stereo);

    //        remap(img1, img1, maplx, maply, inter_linear);
    //        remap(img2, img2, maprx, mapry, inter_linear);

    //        合并成一幅图
    //        img1.copyto(img_stereo(range(0, img1.rows), range(0, img1.cols)));
    //        img2.copyto(img_stereo(range(0, img1.rows), range(img1.cols, img1.cols * 2)));

    //        imwrite("imgst.bmp", 255 * img1);*/
    //        //棋盘格提取
    //        filter.feed(img1);

    //        waitKey(1);
    //        //cv::imshow("MatImage", imgMark);

    //       /* nRet = MV_CC_FreeImageBuffer(handle, &stImageInfo);
    //    }
    //    else
    //    {
    //        printf("Get Image fail! nRet [0x%x]\n", nRet);
    //    }*/
    //} 
    
    //处理视频流
    /*
    VideoCapture capture;
    Mat img;
    img = capture.open("ldy.avi");
    if (!capture.isOpened())
    {
        printf("can not open ...\n");
        return -1;
    }
    int cnt = 0;
    capture.read(img);
    crossMarkDetectorParams Dparams;
    Dparams.height = img.rows;
    Dparams.width = img.cols;
    crossPointResponderParams Rparams;

    crossMarkDetector filter(Dparams, Rparams);
	double start_last = getTickCount();
    while (capture.read(img)) {
		double start = getTickCount();
		double time = (start - start_last) / (double)cvGetTickFrequency() / 1000;
		start_last = start;
		cout << time << std::endl;
        cvtColor(img, img, COLOR_BGR2GRAY);
        img.convertTo(img, CV_32FC1); img = img / 255;
        filter.feed(img, cnt++);
        waitKey(1);
    }
    */
    // 处理单幅图片
    /*Mat img = imread(imagename);
    cvtColor(img, img, COLOR_BGR2GRAY);
    img.convertTo(img, CV_32FC1); img = img / 255;
      
    crossMarkDetectorParams Dparams;
    Dparams.height = img.rows;
    Dparams.width = img.cols;
    crossPointResponderParams Rparams;

    crossMarkDetector filter(Dparams, Rparams);
    filter.feed(img, 1);

    //处理双目视频
    VideoCapture capture_left, capture_right;
    img_left = capture_left.open("left.avi");
    img_right = capture_right.open("right.avi");
    if (!capture_left.isOpened())
    {
        printf("Can not open left camera\n");
        return -1;
    }
    if (!capture_right.isOpened())
    {
        printf("Can not open right camera\n");
        return -1;
    }
    int cnt = 0;
    //capture_left.read(img_left);
    //capture_right.read(img_right);
    //crossMarkDetectorParams Dparams;
    //Dparams.height = img_left.rows;
    //Dparams.width = img_right.cols * 2;
    //crossPointResponderParams Rparams;

    //crossMarkDetector filter(Dparams, Rparams);
    start_last = getTickCount();
    while (capture_left.read(img_left) && capture_right.read(img_right))  {
        start = getTickCount();
        fps = 1000000.0 * (double)cvGetTickFrequency() / (start - start_last);
        start_last = start;

        cvtColor(img_left, img_left, COLOR_BGR2GRAY);
        img_left.convertTo(img_left, CV_32FC1); img_left = img_left / 255;
        cvtColor(img_right, img_right, COLOR_BGR2GRAY);
        img_right.convertTo(img_right, CV_32FC1); img_right = img_right / 255;
  
        undistort(img_left, img_left_undistort, cameraMatrixL, distCoeffL, cameraMatrixL);
        undistort(img_right, img_right_undistort, cameraMatrixR, distCoeffR, cameraMatrixR);

        vImgs.push_back(img_left_undistort);
        vImgs.push_back(img_right_undistort);
        hconcat(vImgs, img_stereo);

        crossMarkDetectorParams Dparams;
        Dparams.height = img_stereo.rows;
        Dparams.width = img_stereo.cols;
        crossPointResponderParams Rparams;
        //imwrite("img_stereo.bmp", img_stereo * 255);

        crossMarkDetector filter(Dparams, Rparams);
        filter.feed(img_stereo, cnt++, fps);
        
        img_stereo.release();
        waitKey(1);
    }*/
 
    //Test
    /*freopen_s(&stream2, "./RandomCrossPointBlur/Blur2/result11.txt", "w", stdout);
    char imagename_test[100];
    for (int i = 1; i <= 1000; i++) {
        sprintf_s(imagename_test, "./RandomCrossPointBlur/Blur2/%d%s", i, ".bmp");
        
        Mat img = imread(imagename_test);
        cvtColor(img, img, COLOR_BGR2GRAY);
        img.convertTo(img, CV_32FC1); img = img / 255;

        crossMarkDetectorParams Dparams;
        Dparams.height = img.rows;
        Dparams.width = img.cols;
        crossPointResponderParams Rparams;

        crossMarkDetector filter(Dparams, Rparams);
        filter.feed(img, 1);
    }*/
    
    waitKey(0);
    
    return 0;
}

int RGB2BGR(unsigned char* pRgbData, unsigned int nWidth, unsigned int nHeight)
{
    if (NULL == pRgbData)
    {
        return MV_E_PARAMETER;
    }

    for (unsigned int j = 0; j < nHeight; j++)
    {
        for (unsigned int i = 0; i < nWidth; i++)
        {
            unsigned char red = pRgbData[j * (nWidth * 3) + i * 3];
            pRgbData[j * (nWidth * 3) + i * 3] = pRgbData[j * (nWidth * 3) + i * 3 + 2];
            pRgbData[j * (nWidth * 3) + i * 3 + 2] = red;
        }
    }

    return MV_OK;
}

// convert data stream in Mat format
cv::Mat Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char* pData)
{
    cv::Mat srcImage;
    if (pstImageInfo->enPixelType == PixelType_Gvsp_Mono8)
    {
        srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC1, pData);
    }
    else if (pstImageInfo->enPixelType == PixelType_Gvsp_RGB8_Packed)
    {
        srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC3, pData);
    }

    cv::waitKey(20);

    return srcImage;
}
