//
//  crossPointDetector.hpp
//  Endoscopy
//
//  Created by 朱明珠 on 2020/12/4.
//

#ifndef crossMarkDetector_hpp
#define crossMarkDetector_hpp

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <stdio.h>
#include <stdlib.h>

#include "crossPointResponder.hpp"

using namespace cv;

struct crossMarkDetectorParams
{
    int height, width;        // 图像的预期尺寸
    int maxMatrixR = 100;     // 棋盘格点的最大间隔
    int maxSupportAngle = 15; // 支持者判定中的最大夹角
};

struct pointInform // 用于储存交叉点的结构, 包含交叉点的信息, 和用于近邻搜索的索引
{
    Point   Pos;         // 整数坐标(图像坐标系,x-y向右-下)
    Point2f subPos;      // 亚像素坐标(图像坐标系,x-y向右-下)
    float   Bdirct;      // 跳黑界线的朝向（上为0°,顺时针,两个跳黑界线中更接近0°的那一个)
    float   Wdirct;      // 跳白界线的朝向（上为0°,顺时针)
    float   score;       // 与标准模板的相关性得分
};

struct linkInform
{
    int idx[4] = {-1,-1,-1,-1};     // 连接对象
    int port[4] = {-1,-1,-1,-1};    // 连接对象的连接端口
    float dist[4] = {FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX};  // 连接距离
};

struct matrixInform
{
    int     mLabel=-1;      // 所属矩阵编号
    Point   mPos;           // 矩阵坐标
};

class crossMarkDetector
{
private:
    crossPointResponder responder;           // 交叉点响应器
    std::vector<pointInform> crossPtsList;   // 交叉点存储数组
    int keyMatrix[10][50][50];               
    
    void findCrossPoint(const Mat &img, std::vector<pointInform> &crossPtsList);                                                                       // 寻找交叉点(比响应器的结果多一轮非极大值抑制), 形成crossPtsList
    void buildMatrix(const Mat &img, std::vector<pointInform> &crossPtsList);                                                                          // 基于crossPtsList解读棋盘格信息, 改变mLabel和mPos
    void displayMatrix(const Mat& img, std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, std::vector<linkInform> links);       // 显示最终结果
    void extractLinkTable(const Mat& img, std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, std::vector<linkInform> links, int matrix2[10][50][50], int labelnum); // 提取LinkTable信息，获得Bias
    void circleDetector(const Mat &img);

    std::vector<std::vector<int>> buildNeighbors(const std::vector<pointInform> &crossPtsList, int r);
    // 基于边长2*r+1, 为crossPtsList中的所有点生成近邻索引
    std::vector<linkInform> buildLinkers(std::vector<pointInform> &crossPtsList, float T);
    // 基于夹角阈值T, 为crossPtsList中的所有点生成连接信息
    
    void distAngle(const Point2f A, const Point2f B, float &dist, float &angle); // 计算两点连线的距离和角度,左上0°顺时针
    bool checkIncludedAngle(const float A, const float B, const float T); // 判断两个角度的夹角是否在阈值T以内


public:
    crossMarkDetectorParams Dparams;    // 标记图案检测器的参数
    crossPointResponderParams Rparams;  // 交叉点响应器的参数
    
    crossMarkDetector(crossMarkDetectorParams Dparams, crossPointResponderParams Rparams);
    ~crossMarkDetector();
    
    void feed(const Mat &img);
    // 向交叉点响应器输入图像和测试点
};

#endif /* crossMarkDetector_hpp */
