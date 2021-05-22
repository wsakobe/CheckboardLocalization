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

void* handle = NULL;

void prepareImageRead(int nRet, void* handle);
cv::Mat Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char* pData);

int main(int argc, char* argv[]) {
    const char* imagename = "02_reg.jpg";//此处为测试图片路径
    //FILE* stream1;
    FILE* stream2;
    //freopen_s(&stream1, "linkTabel.txt", "r", stdin);
    
    stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
        0, imageSize, &validROIL, &validROIR);
    initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
    initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);
    
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
    /*VideoCapture capture;
    Mat img;
    img = capture.open("test_video.avi");
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

    while (capture.read(img)) {
        cvtColor(img, img, COLOR_BGR2GRAY);
        img.convertTo(img, CV_32FC1); img = img / 255;
        filter.feed(img, cnt++);
        waitKey(1);
    }*/
    
    // 处理单幅图片
    Mat img = imread(imagename);
    cvtColor(img, img, COLOR_BGR2GRAY);
    img.convertTo(img, CV_32FC1); img = img / 255;
      
    crossMarkDetectorParams Dparams;
    Dparams.height = img.rows;
    Dparams.width = img.cols;
    crossPointResponderParams Rparams;

    crossMarkDetector filter(Dparams, Rparams);
    filter.feed(img, 1);

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

//HikVision工业相机读取
bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        printf("Device Number: %d\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
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
        RGB2BGR(pData, pstImageInfo->nWidth, pstImageInfo->nHeight);
        srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC3, pData);
    }

    cv::waitKey(20);

    return srcImage;
}

void prepareImageRead(int nRet, void* handle) {
    do
    {
        // ch:枚举设备 | en:Enum device
        MV_CC_DEVICE_INFO_LIST stDeviceList = { 0 };
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("Enum Devices fail! nRet [0x%x]\n", nRet);
            break;
        }

        if (stDeviceList.nDeviceNum > 0)
        {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                }
                PrintDeviceInfo(pDeviceInfo);
            }
        }
        else
        {
            printf("Find No Devices!\n");
            break;
        }

        printf("Please Input camera index(0-%d):", stDeviceList.nDeviceNum - 1);
        unsigned int nIndex = 0;
        //scanf_s("%d", &nIndex);

        if (nIndex >= stDeviceList.nDeviceNum)
        {
            printf("Input error!\n");
            break;
        }

        // ch:选择设备并创建句柄 | en:Select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("Create Handle fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:打开设备 | en:Open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("Open Device fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
                if (nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
            }
        }

        // ch:设置触发模式为off | en:Set trigger mode as off
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:开始取流 | en:Start grab image
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }

        //// ch:停止取流 | en:Stop grab image
        //nRet = MV_CC_StopGrabbing(handle);
        //if (MV_OK != nRet)
        //{
        //    printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
        //    break;
        //}

        //// ch:关闭设备 | Close device
        //nRet = MV_CC_CloseDevice(handle);
        //if (MV_OK != nRet)
        //{
        //    printf("ClosDevice fail! nRet [0x%x]\n", nRet);
        //    break;
        //}

        //// ch:销毁句柄 | Destroy handle
        //nRet = MV_CC_DestroyHandle(handle);
        //if (MV_OK != nRet)
        //{
        //    printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
        //    break;
        //}
    } while (0);

    if (nRet != MV_OK)
    {
        if (handle != NULL)
        {
            MV_CC_DestroyHandle(handle);
            handle = NULL;
        }
    }

    return ;
}