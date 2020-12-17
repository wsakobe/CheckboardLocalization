//
//  crossPointResponder.cpp
//  Endoscopy
//
//  Created by 朱明珠 on 2020/12/4.
//

#include "crossPointResponder.hpp"

crossPointResponder::crossPointResponder(crossPointResponderParams params)
{
    this->params = params;
    maskL  = 2*params.maskR+1;
    maskFL = 4*maskL-4;
    
    checkOff[0].x = 0;              checkOff[0].y = -params.maskR;
    checkOff[1].x =  params.maskR;  checkOff[1].y = -params.maskR;
    checkOff[2].x =  params.maskR;  checkOff[2].y = 0;
    checkOff[3].x =  params.maskR;  checkOff[3].y =  params.maskR;
    checkOff[4].x = 0;              checkOff[4].y =  params.maskR;
    checkOff[5].x = -params.maskR;  checkOff[5].y =  params.maskR;
    checkOff[6].x = -params.maskR;  checkOff[6].y = 0;
    checkOff[7].x = -params.maskR;  checkOff[7].y = -params.maskR;
    
    frameVal = (float *)malloc(sizeof(float)*maskFL);
    frameSgn = (bool*)  malloc(sizeof(bool)*maskFL);
    frameOff = (Point*) malloc(sizeof(Point)*maskFL);
    
    for (int it=0, ix=-params.maskR; ix<params.maskR; ++it, ++ix) {
        frameOff[it].x = ix;
        frameOff[it].y = -params.maskR;
    }
    for (int it=2*params.maskR, iy=-params.maskR; iy<params.maskR; ++it, ++iy) {
        frameOff[it].x = params.maskR;
        frameOff[it].y = iy;
    }
    for (int it=4*params.maskR, ix=params.maskR; ix>-params.maskR; ++it, --ix) {
        frameOff[it].x = ix;
        frameOff[it].y = params.maskR;
    }
    for (int it=6*params.maskR, iy=params.maskR; iy>-params.maskR; ++it, --iy) {
        frameOff[it].x = -params.maskR;
        frameOff[it].y = iy;
    }
    
    // 在当前目录读取模板查找表, 如果没有, 做一个
    String tmpFile = "template";
    tmpFile.append(std::to_string(maskL));
    tmpFile.append(".bmp");

    struct stat buffer;
    if (stat(tmpFile.c_str(), &buffer) != 0)
    {
        Mat tmpMSAA, tmpCrop;
        tmpMSAA = Mat::zeros(10 * maskL, 10 * maskL, CV_32FC1);
        tmp = Mat::zeros(36 * maskL, 36 * maskL, CV_32FC1);
        for (float B = 0, angleB = 0; B < 36; angleB = angleB + 5, ++B) {
            for (float W = 0, angleW = 0; W < 36; angleW = angleW + 5, ++W) {
                float ix = 0.5 - float(tmpMSAA.cols) / 2;
                float iy;
                for (int x = 0; x < tmpMSAA.cols; ++x, ++ix) {
                    iy = float(tmpMSAA.rows) / 2 - 0.5;
                    for (int y = 0; y <= x; ++y, --iy) {
                        float temp = (atan2(ix, iy)) / CV_PI * 180 + 45;

                        if (angleB == angleW) continue;
                        if (temp > angleW && temp < angleB) {
                            tmpMSAA.at<float>(y, x) = 1;
                            tmpMSAA.at<float>(tmpMSAA.rows - y - 1, tmpMSAA.cols - x - 1) = 1;
                        }
                        else if (angleB < angleW && (temp<angleB || temp>angleW)) {
                            tmpMSAA.at<float>(y, x) = 1;
                            tmpMSAA.at<float>(tmpMSAA.rows - y - 1, tmpMSAA.cols - x - 1) = 1;
                        }
                        else {
                            tmpMSAA.at<float>(y, x) = 0;
                            tmpMSAA.at<float>(tmpMSAA.rows - y - 1, tmpMSAA.cols - x - 1) = 0;
                        }
                    }
                }
                tmpCrop = tmp(Rect(B * maskL, W * maskL, maskL, maskL));
                resize(tmpMSAA, tmpCrop, Size(maskL, maskL), 0, 0, INTER_AREA);
            }
        }
        imwrite(tmpFile, 255 * tmp);
        tmpMSAA.release();
        tmpCrop.release();
    }
    else
    {
        tmp = imread(tmpFile);
        cvtColor(tmp, tmp, COLOR_BGR2GRAY);
        tmp.convertTo(tmp, CV_32FC1);
        tmp = tmp / 255;
    }
}

crossPointResponder::~crossPointResponder()
{
    delete[] frameVal;
    delete[] frameSgn;
    delete[] frameOff;
    tmp.release();
}

bool crossPointResponder::checkPointCheck(const Mat &img, Point curPos)
{
    // 寄存到checkVal
    for (int it=0; it<8; ++it) {
        checkVal[it] = img.at<float>(curPos.y + checkOff[it].y, curPos.x + checkOff[it].x);
    }
    // 判断间隙条件
    float maxGap = -1;
    for (int ia = 0; ia < 3; ++ia) {
        for (int ib = ia + 1; ib < 4; ++ib) {
            if (abs(checkVal[ia] - checkVal[ia + 4]) > params.T_maxInnerGap) continue;
            if (abs(checkVal[ib] - checkVal[ib + 4]) > params.T_maxInnerGap) continue;
            float tempGap = abs((checkVal[ia] + checkVal[ia + 4]) - (checkVal[ib] + checkVal[ib + 4])) / 2;
            if (tempGap > maxGap) {
                maxGap = tempGap;
            }
        }
    }
    if (maxGap < params.T_minOuterGap) return 0;
    return 1;
}

bool  crossPointResponder::contourCheck(const Mat& img, Point curPos)
{
    // 计算边框均值
    float frameMean = 0;
    for (int it = 0; it < maskFL; ++it) {
        frameVal[it] = img.at<float>(curPos.y + frameOff[it].y, curPos.x + frameOff[it].x);
        frameMean = frameMean + frameVal[it];
    }
    frameMean = frameMean / maskFL;

    // 计算边框的二元颜色信息
    for (int it = 0; it < maskFL; ++it) {
        if (frameVal[it] > frameMean) frameSgn[it] = 1;
        else                        frameSgn[it] = 0;
    }
    // 记录跳变信息,检查跳变次数
    int jmpCount[2] = { 0,0 }, jmpIdx[2][2];
    for (int ia = 0; ia < maskFL; ++ia) {
        int ib;
        if (ia == 0) ib = maskFL - 1;
        else       ib = ia - 1;
        if (frameSgn[ia] != frameSgn[ib]) {
            if (jmpCount[frameSgn[ia]] < 2) {
                jmpIdx[frameSgn[ia]][jmpCount[frameSgn[ia]]] = ia;
            }
            ++jmpCount[frameSgn[ia]];
        }
    }
    if (jmpCount[0] != 2 || jmpCount[1] != 2) return 0;
    // 计算跳变界线,检查亚像素交叉点
    for (int ia = 0; ia < 2; ++ia) {
        for (int ib = 0; ib < 2; ++ib) {
            int jmpA = jmpIdx[ia][ib];
            int jmpB = jmpA - 1; if (jmpB < 0) jmpB = maskFL - 1;
    
            float dstA = abs(frameVal[jmpA] - frameMean);
            float dstB = abs(frameVal[jmpB] - frameMean);
            float dstSum = dstA + dstB;
            dstA = dstA / dstSum;
            dstB = dstB / dstSum;
    
            crossLineEnds[ia][ib].x = dstB * frameOff[jmpA].x + dstA * frameOff[jmpB].x;
            crossLineEnds[ia][ib].y = dstB * frameOff[jmpA].y + dstA * frameOff[jmpB].y;
        }
    }
    float center[3];
    crossLine[0][0] = crossLineEnds[0][0].y - crossLineEnds[0][1].y;
    crossLine[0][1] = crossLineEnds[0][1].x - crossLineEnds[0][0].x;
    crossLine[0][2] = crossLineEnds[0][0].x*crossLineEnds[0][1].y-
                  crossLineEnds[0][0].y*crossLineEnds[0][1].x;
    crossLine[1][0] = crossLineEnds[1][0].y - crossLineEnds[1][1].y;
    crossLine[1][1] = crossLineEnds[1][1].x - crossLineEnds[1][0].x;
    crossLine[1][2] = crossLineEnds[1][0].x*crossLineEnds[1][1].y-
                  crossLineEnds[1][0].y*crossLineEnds[1][1].x;
    center[0] = crossLine[0][1]*crossLine[1][2] - crossLine[0][2]*crossLine[1][1];
    center[1] = crossLine[0][2]*crossLine[1][0] - crossLine[0][0]*crossLine[1][2];
    center[2] = crossLine[0][0]*crossLine[1][1] - crossLine[0][1]*crossLine[1][0];
    crossCenter.x = center[0]/center[2];
    crossCenter.y = center[1]/center[2];
    
    if (abs(crossCenter.x) > params.T_maxCenterBias || abs(crossCenter.y) > params.T_maxCenterBias) return 0;

    response_cross = Point2f(crossCenter.x + curPos.x + 0.5, crossCenter.y + curPos.y + 0.5);
    response_crossPos.x = (int)response_cross.x;
    response_crossPos.y = (int)response_cross.y;
    
    if (response_crossPos.x - params.maskR < 0 || response_crossPos.x + params.maskR >= img.cols) return 0;
    if (response_crossPos.y - params.maskR < 0 || response_crossPos.y + params.maskR >= img.rows) return 0;
    
    return 1;
}

bool crossPointResponder::maskCheck(const Mat& Img, Point curPos)
{
    // 计算跳变点的极坐标角度
    for (int ia = 0; ia < 2; ++ia) {
        crossLineEndsAngle[ia] = (atan2(crossLineEnds[ia][0].x - crossCenter.x, 
            crossCenter.y - crossLineEnds[ia][0].y)) / CV_PI * 180 + 45;
        if (crossLineEndsAngle[ia] < 0) crossLineEndsAngle[ia] = 360 + crossLineEndsAngle[ia];
    }
    // 对角度排序
    float angle[2], angleMark[2];
    if (crossLineEndsAngle[0] > crossLineEndsAngle[1]) {
        angle[0] = crossLineEndsAngle[1];
        angle[1] = crossLineEndsAngle[0];
        angleMark[0] = 1;
        angleMark[1] = 0;
    }
    else {
        angle[0] = crossLineEndsAngle[0];
        angle[1] = crossLineEndsAngle[1];
        angleMark[0] = 0;
        angleMark[1] = 1;
    }
    if ((angle[1] - angle[0] < 45) && (angle[1] - angle[0] > 135)) return 0; // 检查两类跳变角的夹角
    
    // 基于跳变信息, 选择模板进行检查
    int angleIdx[2];
    if (angleMark[0] == 1) {
        angleIdx[0] = round(angle[1] / 5);
        angleIdx[1] = round(angle[0] / 5);
    }
    else {
        angleIdx[0] = round(angle[0] / 5);
        angleIdx[1] = round(angle[1] / 5);
    }
    if (angleIdx[0] < 0 || angleIdx[0]>35) angleIdx[0] = 0;
    if (angleIdx[1] < 0 || angleIdx[1]>35) angleIdx[1] = 0;
    
    Mat tmpCrop(tmp, Rect(angleIdx[0] * maskL, angleIdx[1] * maskL, maskL, maskL));
    Mat crop(Img, Rect(response_crossPos.x - params.maskR, response_crossPos.y - params.maskR, maskL, maskL));
    
    Scalar meanTmp, meanCrop, stdTmp, stdCrop;
    meanStdDev(tmpCrop, meanTmp, stdTmp);
    meanStdDev(crop, meanCrop, stdCrop);
    float covar = (tmpCrop - meanTmp).dot(crop - meanCrop) / (maskL * maskL);
    response_score = covar / (stdTmp[0] * stdCrop[0]);
    if (response_score < params.T_minCorrelation) return 0;

//    模板测试
//    if (response_crossPos.y>0)
//    {
//    Mat tmpCropShow, cropShow;
//    cvtColor(tmpCrop, tmpCropShow, COLOR_GRAY2RGB);
//    cvtColor(crop, cropShow, COLOR_GRAY2RGB);
//    resize(tmpCropShow, tmpCropShow, Size(500,500), 0,0,INTER_NEAREST);
//    resize(cropShow, cropShow, Size(500,500), 0,0,INTER_NEAREST);
//    Point A(250+500*sin((angle[0]-45)/180*CV_PI),250-500*cos((angle[0]-45)/180*CV_PI));
//    Point B(250+500*sin((angle[1]-45)/180*CV_PI),250-500*cos((angle[1]-45)/180*CV_PI));
//    line(cropShow, A, Point(250, 250), Scalar(0,0,1));
//    line(cropShow, B, Point(250, 250), Scalar(0,0,1));
//    line(tmpCropShow, A, Point(250, 250), Scalar(0,0,1));
//    line(tmpCropShow, B, Point(250, 250), Scalar(0,0,1));
//
//    imshow("1",cropShow);
//    imshow("2",tmpCropShow);
//        
//    waitKey(1);
//        float a = response_score;
//        int b = 1;
//    }
    
    return 1;
}

void crossPointResponder::feed(const Mat& img, Point curPos)
{
    response_haveCrossPt = 0;
    
    if (!checkPointCheck(img, curPos)) return;
    if (!contourCheck(img, curPos))    return;
    if (!maskCheck(img, curPos))       return;
    
    response_haveCrossPt = 1;

    response_blackLine = crossLineEndsAngle[0] - 45;
    response_whiteLine = crossLineEndsAngle[1] - 45;
    if (response_blackLine < 0) response_blackLine = response_blackLine + 360;
    if (response_whiteLine < 0) response_whiteLine = response_whiteLine + 360;
    if (response_blackLine > 180) response_blackLine = response_blackLine - 180;
    if (response_whiteLine > 180) response_whiteLine = response_whiteLine - 180;
}
