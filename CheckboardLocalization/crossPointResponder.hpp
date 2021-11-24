//
//  crossPointResponder.hpp
//  Endoscopy
//
//  Created by 朱明珠 on 2020/12/4.
//

#ifndef crossPointResponder_hpp
#define crossPointResponder_hpp

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <stdio.h>
#include "opencv2/imgproc/types_c.h"
#include "opencv2/features2d/features2d.hpp"
#include "unistd.h"

#include "crossMarkDetector.hpp"

using namespace cv;

struct crossPointResponderParams
{
    int   maskR = 5;
    float T_maxInnerGap = 0.2;
    float T_minOuterGap = 0.25;
    float T_maxCenterBias = 1;
    float T_minCorrelation = 0.8;
};

class crossPointResponder
{
private:
    int maskL, maskFL;
   
    // 工作空间A, 检查点检查
    Point checkOff[8];        // 掩模8个方向上的检查点相对中心像素的偏移(从上方开始, 顺时针, 末位为左上)
    float checkVal[8];        // 掩模8个方向上的检查点的取值
    
    // 工作空间B, 边框检查
    Point *frameOff;          // 掩模边框像素相对中心像素的偏移(用于快速索引边框像素)
    float *frameVal;          // 掩模边框上的取值
    bool  *frameSgn;          // 掩模边框上像素的极性(0-黑、1-白)
    
    Point2f crossLineEnds[2][2];    // 跳变点的亚像素偏移, 例如crossLineEnds[0][1]表示顺时针跳黑界线的第二个跳变点(curPos为原点,x-y向右-下)
    float   crossLineEndsAngle[2];  // 第一个黑/白色跳变点的极坐标角度(crossCenter为原点,左上为0°,顺时针)
    float   crossLine[2][3];        // 跳黑/白边界的直线参数(curPos为原点,x-y向右-下)
    Point2f crossCenter;            // 跳黑/白边界的亚像素交点(curPos为原点,x-y向右-下)
    
    // 工作空间C, 模板检查
    Mat tmp;                     // 模板,与多重采样模板
    float correl;               // 当前区域与所生产模板的相关系数
    
    bool checkPointCheck(const Mat &img, Point curPos);  // 检查点检查
    bool contourCheck(const Mat &img, Point curPos);     // 边框检查
    bool maskCheck(const Mat &Img, Point curPos);        // 模板检查
    std::vector<Point> gradientCheck(const Mat& img, crossMarkDetectorParams Dparams, crossPointResponderParams Rparams);    // 基于梯度的交叉点亚像素坐标优化
    
public:
    crossPointResponderParams params;
    crossMarkDetectorParams params_mark;
    bool    response_haveCrossPt;  // 指示测试点附近是否存在交叉点
    Point2f response_cross;        // 检测到的交叉点的亚像素坐标(图像坐标系,x-y向右-下)
    Point   response_crossPos;     // 检测到的交叉点的整数坐标(图像坐标系,x-y向右-下)
    float   response_blackLine;    // 检测到的交叉点的跳黑界线的朝向（上为0°,顺时针)
    float   response_whiteLine;    // 检测到的交叉点的跳白界线的朝向（上为0°,顺时针)
    float   response_score;        // 交叉点的得分
    
    crossPointResponder(crossPointResponderParams params);
    // 构造一个交叉点响应器
    // maskR: mask半径, mask直径为 2*maskR+1
    // T_maxInnerGap, T_minOuterGap: 检查点内/外阈值, 若超出, 认为Img(curPos)附近没有交叉点
    // T_maxCenterBias: 亚像素中心与mask中心超过该阈值时, 认为Img(curPos)附近没有交叉点
    // T_minCorrelation: 若亚像素中心的最终得分低于该阈值, 认为Img(curPos)附近没有交叉点
    
    ~crossPointResponder();
    
    void feed(const Mat &img, crossMarkDetectorParams params_mark, crossPointResponderParams params);
    // 向交叉点响应器输入图像和测试点, 响应器以"response_"开头的公有成员会因此改变
};

#endif /* crossPointResponder_hpp */
