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
#include <opencv2/calib3d.hpp>
#include <stdio.h>
#include <stdlib.h>

#include "crossPointResponder.hpp"

//相机标定参数！！
const Mat cameraMatrixL = (Mat_<double>(3, 3) << 2177.19085685241, 2.50300520705354, 987.043784480210,
    0, 2172.17693250848, 614.654598960330,
    0, 0, 1);
const Mat distCoeffL = (Mat_<double>(5, 1) << -0.128653266353462, 0.0990289902922354, -0.00151693364889492, -0.0000778074982959638, -0.0537249117228832);
const Mat cameraMatrixR = (Mat_<double>(3, 3) << 2173.01163988693, 2.53683057527207, 993.749976335397,
    0, 2168.79494943487, 610.888340207888,
    0, 0, 1);
const Mat distCoeffR = (Mat_<double>(5, 1) << -0.157813985137686, 0.147713412836913, 0.000225063584088769, 0.000749605380378634, 0.151790070856550);

const Mat Trans = (Mat_<double>(3, 1) << 50.4393564634767, -0.723403549240724, -0.405551487570464);
const Mat Rot = (Mat_<double>(3, 3) << 0.999966026847095, -0.00808560125461077, -0.00160256169534288,
    0.00808368826277052, 0.999966610493402, -0.00119661288970822,
    0.00161218352128088, 0.00118361762782855, 0.999997999954802);

using namespace cv;

struct crossMarkDetectorParams
{
    int height, width;        // 图像的预期尺寸
    int maxMatrixR = 35;     // 棋盘格点的最大间隔
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

struct linkTableInform
{
    Point  mPos = {(-1, -1)}; // 10bit值
    int    dir = -1;            // 方向
};

class crossMarkDetector
{
private:
    crossPointResponder responder;           // 交叉点响应器
    std::vector<pointInform> crossPtsList;   // 交叉点存储数组
    std::vector<Point2f> cartisian_dst;
    int  keyMatrix[10][100][100], matrix2[10][100][100], cnt = 0;
    bool signal = false, updateSuccess[10];
    linkTableInform linkTabel[1024];
   
    void findCrossPoint(const Mat &img, std::vector<pointInform> &crossPtsList);                                                                       // 寻找交叉点(比响应器的结果多一轮非极大值抑制), 形成crossPtsList
    void buildMatrix(const Mat &img, std::vector<pointInform> &crossPtsList, int cnt);                                                                          // 基于crossPtsList解读棋盘格信息, 改变mLabel和mPos
    void displayMatrix(const Mat& img, std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, std::vector<linkInform> links, std::vector<Point>& centerpoint, bool update[10], std::vector<Point2f>& cartisian_dst, int cnt);       // 显示最终结果
    void displayMatrix_crosspoint(const Mat& img, std::vector<pointInform>& crossPtsList);
    std::vector<matrixInform> extractLinkTable(const Mat& img, std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, std::vector<linkInform> links, int matrix2[10][100][100], int labelnum, std::vector<Point>& centerpoint); // 提取LinkTable信息，获得Bias
    void outputLists(std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, bool update[10]);
    void hydraCode_Mono_Homography(const Mat& img, std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, int labelnum, std::vector<Point2f>& cartisian_dst, bool update[10], int cnt);
    void hydraCode_Mono_PnP(const Mat& img, std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, int labelnum, std::vector<Point2f>& cartisian_dst, bool update[10], int cnt);
    void hydraCode_stereo_ICP(const Mat& img, std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, int labelnum, std::vector<Point2f>& cartisian_dst, bool update[10], int cnt);

    std::vector<std::vector<int>> buildNeighbors(const std::vector<pointInform> &crossPtsList, int r);
    // 基于边长2*r+1, 为crossPtsList中的所有点生成近邻索引
    std::vector<linkInform> buildLinkers(std::vector<pointInform> &crossPtsList, float T);
    // 基于夹角阈值T, 为crossPtsList中的所有点生成连接信息
    
    Point3f uv2xyz(Point2f uvLeft, Point2f uvRight); //双目测距
    void distAngle(const Point2f A, const Point2f B, float &dist, float &angle); // 计算两点连线的距离和角度,左上0°顺时针
    bool checkIncludedAngle(const float A, const float B, const float T); // 判断两个角度的夹角是否在阈值T以内
   
public:
    crossMarkDetectorParams Dparams;    // 标记图案检测器的参数
    crossPointResponderParams Rparams;  // 交叉点响应器的参数
    
    crossMarkDetector(crossMarkDetectorParams Dparams, crossPointResponderParams Rparams);
    ~crossMarkDetector();
    
    void feed(const Mat &img, int cnt);
    // 向交叉点响应器输入图像和测试点
};

#endif /* crossMarkDetector_hpp */
