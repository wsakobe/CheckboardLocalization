﻿//
//  crossPointDetector.cpp
//  Endoscopy
//
//  Created by 朱明珠 on 2020/12/4.
//

#include "crossMarkDetector.hpp"
#include <cstring>
#include <iostream>
#include <fstream>

using namespace cv;

std::vector<KeyPoint> key_points;

crossMarkDetector::crossMarkDetector(crossMarkDetectorParams Dparams, crossPointResponderParams Rparams) : responder(Rparams)
{
	this->Dparams = Dparams;
	this->Rparams = Rparams;
}

crossMarkDetector::~crossMarkDetector()
{
}

void crossMarkDetector::feed(const Mat& img, int cnt)
{
	assert(img.type() == CV_32FC1); //判断图片格式是否正确
	std::ifstream OpenFile;
	OpenFile.open("linkTabel.txt");
	if (!signal) {
		signal = true;
		int val;
		while (!OpenFile.eof()){
			OpenFile >> val;
			OpenFile >> linkTabel[val].mPos.x >> linkTabel[val].mPos.y >> linkTabel[val].dir;
			//scanf_s("%d", &val);
			//scanf_s("%d %d %d", &linkTabel[val].mPos.x, &linkTabel[val].mPos.y, &linkTabel[val].dir);
		}
	}
	findCrossPoint(img, crossPtsList);
	buildMatrix(img, crossPtsList, cnt);
	//displayMatrix_crosspoint(img, crossPtsList);
}

void crossMarkDetector::findCrossPoint(const Mat& img, std::vector<pointInform>& crossPtsList)
{
	// 全图搜索交叉点
	crossPtsList.clear();
	responder.feed(img, Dparams, Rparams);

	if (!responder.response_haveCrossPt) continue;
	if (responder.response_crossPos.x < Rparams.maskR || responder.response_crossPos.x >= Dparams.width - Rparams.maskR)   continue;
	if (responder.response_crossPos.y < Rparams.maskR || responder.response_crossPos.y >= Dparams.height - Rparams.maskR)  continue;

	pointInform inform;
	inform.subPos = responder.response_cross;
	inform.Pos = responder.response_crossPos;
	inform.Bdirct = responder.response_blackLine;
	inform.Wdirct = responder.response_whiteLine;
	inform.score = responder.response_score;
	crossPtsList.push_back(inform);
	
	// 非极大值抑制
	std::vector<std::vector<int>> neighbors = buildNeighbors(crossPtsList, Rparams.maskR);
	std::vector<int> locMaxFlag(crossPtsList.size(), -1);

	for (int Aidx = 0; Aidx < crossPtsList.size(); ++Aidx) {
		if (locMaxFlag[Aidx] != -1) continue;
		for (int ib = 0; ib < neighbors[Aidx].size(); ++ib) {
			int   Bidx = neighbors[Aidx][ib];
			if (crossPtsList[Aidx].score > crossPtsList[Bidx].score)  locMaxFlag[Bidx] = 0;
			if (crossPtsList[Aidx].score < crossPtsList[Bidx].score)  locMaxFlag[Aidx] = 0;
			if (crossPtsList[Aidx].score == crossPtsList[Bidx].score) {
				if (Aidx > Bidx)  locMaxFlag[Aidx] = 0;
				else            locMaxFlag[Bidx] = 0;
			}
		}
		if (locMaxFlag[Aidx] == -1) locMaxFlag[Aidx] = 1;
	}

	std::vector<pointInform>::iterator ip = crossPtsList.begin();

	for (int it = 0; it < locMaxFlag.size(); ++it) {
		if (locMaxFlag[it] != 1)  ip = crossPtsList.erase(ip);
		else                   ++ip;
	}
}

std::vector<std::vector<int>> crossMarkDetector::buildNeighbors(const std::vector<pointInform>& crossPtsList, const int r)
{
	std::vector<std::vector<int>> neighbors(crossPtsList.size());
	for (int ia = 0; ia < crossPtsList.size(); ++ia) {
		for (int ib = ia + 1; ib < crossPtsList.size(); ++ib) {
			if (abs(crossPtsList[ia].Pos.x - crossPtsList[ib].Pos.x) > r) continue;
			if (abs(crossPtsList[ia].Pos.y - crossPtsList[ib].Pos.y) > r) continue;
			neighbors[ia].push_back(ib);
			//neighbors[ib].push_back(ia); // !!不记录索引比自己小的近邻
		}
	}
	return neighbors;
}

std::vector<linkInform> crossMarkDetector::buildLinkers(std::vector<pointInform>& crossPtsList, float T)
{
	std::vector<std::vector<int>> neighbor = buildNeighbors(crossPtsList, Dparams.maxMatrixR);
	std::vector<linkInform> links(crossPtsList.size());

	// 检查每一种组合, 建立连接
	for (int Aidx = 0; Aidx < crossPtsList.size(); ++Aidx) {
		for (int in = 0; in < neighbor[Aidx].size(); ++in) {
			int Bidx = neighbor[Aidx][in];
			float dist, angle;
			distAngle(crossPtsList[Aidx].subPos, crossPtsList[Bidx].subPos, dist, angle);
			// 依次检查4个方向
			int A = -1, B = -1;
			if (checkIncludedAngle(angle, crossPtsList[Aidx].Bdirct, T))          A = 0;
			else if (checkIncludedAngle(angle, crossPtsList[Aidx].Wdirct, T))     A = 1;
			else if (checkIncludedAngle(angle, crossPtsList[Aidx].Bdirct + 180, T)) A = 2;
			else if (checkIncludedAngle(angle, crossPtsList[Aidx].Wdirct + 180, T)) A = 3;
			if (checkIncludedAngle(angle, crossPtsList[Bidx].Bdirct, T))          B = 2;
			else if (checkIncludedAngle(angle, crossPtsList[Bidx].Wdirct, T))     B = 3;
			else if (checkIncludedAngle(angle, crossPtsList[Bidx].Bdirct + 180, T)) B = 0;
			else if (checkIncludedAngle(angle, crossPtsList[Bidx].Wdirct + 180, T)) B = 1;
			if (A != -1 && B != -1 &&
				(((A == 0 || A == 2) && (B == 1 || B == 3)) || ((A == 1 || A == 3) && (B == 0 || B == 2)))
				&& dist < links[Aidx].dist[A] && dist < links[Bidx].dist[B]) {
				links[Aidx].idx[A] = Bidx;
				links[Aidx].port[A] = B;
				links[Aidx].dist[A] = dist;
				links[Bidx].idx[B] = Aidx;
				links[Bidx].port[B] = A;
				links[Bidx].dist[B] = dist;
			}
		}
	}

	// 仅保留双向连接
	for (int Aidx = 0; Aidx < links.size(); ++Aidx) {
		for (int it = 0; it < 4; ++it) {
			int Bidx = links[Aidx].idx[it];
			int port = links[Aidx].port[it];
			if ((Bidx >= 0) && (links[Bidx].idx[port] != Aidx)) {
				links[Aidx].idx[it] = -1;
				links[Aidx].port[it + 4] = -1;
			}
		}
	}

	// 检查是否有未连接的黑白界线
	std::vector<bool> wellSupported(links.size(), true);
	std::vector<int> chain(links.size());
	for (int it = 0; it < links.size(); ++it) chain[it] = it;
	for (int ic = 0; ic < chain.size(); ++ic) {
		int it = chain[ic];
		if (!wellSupported[it]) continue;
		if ((links[it].idx[0] == -1 && links[it].idx[2] == -1) || (links[it].idx[1] == -1 && links[it].idx[3] == -1)) {
			wellSupported[it] = false;
			for (int il = 0; il < 4; ++il) {
				int linkedPt = links[it].idx[il];
				if (linkedPt == -1)   continue;
				int linkedPort = links[it].port[il];
				links[linkedPt].idx[linkedPort] = -1;
				links[linkedPt].port[linkedPort] = -1;
				if (wellSupported[linkedPt] && linkedPt < it) chain.push_back(linkedPt);
				links[it].idx[il] = -1;
				links[it].port[il] = -1;
			}
		}
	}
	// 更新crossPtsList和links
	std::vector<linkInform>::iterator iL = links.begin();
	std::vector<pointInform>::iterator iP = crossPtsList.begin();
	std::vector<bool>::iterator iF = wellSupported.begin();
	std::vector<int> bias(links.size(), 0);
	int eraseNum = 0;
	for (int it = 0; iL != links.end(); ++it) {
		if (*iF) {
			++iL;
			++iP;
			++iF;
		}
		else {
			iL = links.erase(iL);
			iP = crossPtsList.erase(iP);
			iF = wellSupported.erase(iF);
			++eraseNum;
		}
		bias[it] = eraseNum;
	}
	for (int it = 0; it < links.size(); ++it) {
		for (int il = 0; il < 4; ++il) {
			if (links[it].idx[il] == -1)   continue;
			links[it].idx[il] = links[it].idx[il] - bias[links[it].idx[il]];
		}
	}

	return links;
}

void crossMarkDetector::buildMatrix(const Mat& img, std::vector<pointInform>& crossPtsList, int cnt)
{
	// 建立连接
	std::vector<linkInform> links = buildLinkers(crossPtsList, Dparams.maxSupportAngle);
	// 建立矩阵
	std::vector<matrixInform> matrix(crossPtsList.size());
	std::vector<Point> centerpoint;
	memset(matrix2, -1, sizeof(matrix2));
	std::vector<std::array<Point, 4>> dict(crossPtsList.size()); // 方向传递
	memset(updateSuccess, false, sizeof(updateSuccess));
	int labelNum = 0;

	for (int io = 0; io < crossPtsList.size(); ++io) {
		// 准备矩阵起始点
		if ((matrix[io].mLabel != -1) || (crossPtsList[io].Bdirct < crossPtsList[io].Wdirct)) continue;
		matrix[io].mPos = Point(20, 20);
		matrix[io].mLabel = labelNum;
		dict[io] = { Point(0,1),Point(1,0),Point(0,-1),Point(-1,0) };
		++labelNum;
		std::vector<int> member;
		member.push_back(io);
		
		// 生长
		for (int ig = 0; ig < member.size(); ++ig) {
			int it = member[ig];
			matrix2[labelNum - 1][matrix[it].mPos.x][matrix[it].mPos.y] = it;

			for (int il = 0; il < 4; ++il) {
				int linkPt = links[it].idx[il];
				if (linkPt == -1 || matrix[linkPt].mLabel != -1) continue;
				int linkPort = links[it].port[il];

				matrix[linkPt].mPos = matrix[it].mPos + dict[it][il];
				matrix[linkPt].mLabel = matrix[it].mLabel;
				member.push_back(linkPt);

				dict[linkPt][linkPort] = -dict[it][il];
				dict[linkPt][(linkPort + 2) % 4] = -dict[linkPt][linkPort];
				if (crossPtsList[linkPt].Bdirct > crossPtsList[linkPt].Wdirct) {
					dict[linkPt][(linkPort + 1) % 4] = Point(dict[linkPt][linkPort].y, -dict[linkPt][linkPort].x);
					dict[linkPt][(linkPort + 3) % 4] = -dict[linkPt][(linkPort + 1) % 4];
				}
				else {
					dict[linkPt][(linkPort + 1) % 4] = Point(-dict[linkPt][linkPort].y, dict[linkPt][linkPort].x);
					dict[linkPt][(linkPort + 3) % 4] = -dict[linkPt][(linkPort + 1) % 4];
				}
				/*if (linkPort == 0)    dict[linkPt][0] = dict[linkPt][linkPort];
				if (linkPort == 1)    dict[linkPt][0] = Point(-dict[linkPt][1].y, dict[linkPt][1].x);
				if (linkPort == 2)    dict[linkPt][0] = -dict[linkPt][2];
				if (linkPort == 3)    dict[linkPt][0] = Point(dict[linkPt][3].y, -dict[linkPt][3].x);

				dict[linkPt][1] = Point(dict[linkPt][0].y, -dict[linkPt][0].x);
				dict[linkPt][2] = Point(dict[linkPt][1].y, -dict[linkPt][1].x);
				dict[linkPt][3] = Point(dict[linkPt][2].y, -dict[linkPt][2].x);*/
			}
		}
	}
	matrix = extractLinkTable(img, crossPtsList, matrix, links, matrix2, labelNum, centerpoint);
	//hydraCode_Mono_Homography(img, crossPtsList, matrix, labelNum, cartisian_dst, updateSuccess, cnt);
	hydraCode_Mono_PnP(img, crossPtsList, matrix, labelNum, cartisian_dst, updateSuccess, cnt);
	//hydraCode_stereo_ICP(img, crossPtsList, matrix, labelNum, cartisian_dst, updateSuccess, cnt);
	//displayMatrix(img, crossPtsList, matrix, links, centerpoint, updateSuccess, cartisian_dst, cnt);
	//outputLists(crossPtsList, matrix, updateSuccess);
}

bool checkLattice(int label, int x, int y, int matrix2[10][100][100]) {
	if ((matrix2[label][x][y] != -1) && (matrix2[label][x + 1][y] != -1) && (matrix2[label][x + 1][y + 1] != -1) && (matrix2[label][x][y + 1] != -1)) return true;
	return false;
}

bool checkNinePatch(int label, int x, int y, int keyMatrix[10][100][100]) {
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			if (keyMatrix[label][x + i][y + j] == -1) return false;
	return true;
}

std::vector<matrixInform> crossMarkDetector::extractLinkTable(const Mat& img, std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, std::vector<linkInform> links, int matrix2[10][100][100], int labelnum, std::vector<Point>& centerpoint) {
	float dist, angle;
	Point2f pos1, pos2, pos3, centerPoint;
	int binary = 1, keyValue = 0, keyMatrixValue;
	float pixel[5], mid_pixel[4];
	bool matrixVisit[500];
	memset(keyMatrix, -1, sizeof(keyMatrix));
	int coeff[4][4] = { 1, 0, 0, 1,  0, -1, 1, 0,  -1, 0, 0, -1,  0, 1, -1, 0 };

	for (int label = 0; label < labelnum; label++)
		for (int i = 0; i < crossPtsList.size(); i++)
			if (matrix2[label][matrix[i].mPos.x][matrix[i].mPos.y] == i && checkLattice(label, matrix[i].mPos.x, matrix[i].mPos.y, matrix2)) {
				pos1 = crossPtsList[matrix2[label][matrix[i].mPos.x + 1][matrix[i].mPos.y]].subPos;
				pos2 = crossPtsList[matrix2[label][matrix[i].mPos.x][matrix[i].mPos.y + 1]].subPos;
				pos3 = crossPtsList[matrix2[label][matrix[i].mPos.x + 1][matrix[i].mPos.y + 1]].subPos;
				centerPoint = (pos1 + pos2 + pos3 + crossPtsList[i].subPos) / 4;
				centerpoint.push_back(Point((int)centerPoint.x, (int)centerPoint.y));
				pixel[0] = img.ptr<float>((int)centerPoint.y)[(int)centerPoint.x];
				pixel[1] = img.ptr<float>((int)centerPoint.y)[(int)centerPoint.x - 1];
				pixel[2] = img.ptr<float>((int)centerPoint.y - 1)[(int)centerPoint.x];
				pixel[3] = img.ptr<float>((int)centerPoint.y + 1)[(int)centerPoint.x];
				pixel[4] = img.ptr<float>((int)centerPoint.y)[(int)centerPoint.x + 1];
				mid_pixel[0] = img.ptr<float>((int)(pos1.y * 0.7 + centerPoint.y * 0.3))[(int)(pos1.x * 0.7 + (int)centerPoint.x * 0.3)];
				mid_pixel[1] = img.ptr<float>((int)(pos2.y * 0.7 + centerPoint.y * 0.3))[(int)(pos2.x * 0.7 + centerPoint.x * 0.3)];
				mid_pixel[2] = img.ptr<float>((int)(pos3.y * 0.7 + centerPoint.y * 0.3))[(int)(pos3.x * 0.7 + centerPoint.x * 0.3)];
				mid_pixel[3] = img.ptr<float>((int)(crossPtsList[i].subPos.y * 0.7 + centerPoint.y * 0.3))[(int)(crossPtsList[i].subPos.x * 0.7 + centerPoint.x * 0.3)];
				float maxv = 0, minv = 1;
				for (int j = 0; j <= 4; j++) {
					if (pixel[j] > maxv)
						maxv = pixel[j];
					if (pixel[j] < minv)
						minv = pixel[j];
				}
				float mid_maxv = 0, mid_minv = 1;
				for (int j = 0; j < 4; j++) {
					if (mid_pixel[j] > mid_maxv)
						mid_maxv = mid_pixel[j];
					if (mid_pixel[j] < mid_minv)
						mid_minv = mid_pixel[j];
				}
				distAngle(crossPtsList[i].subPos, crossPtsList[matrix2[label][matrix[i].mPos.x + 1][matrix[i].mPos.y]].subPos, dist, angle);
				
				if (abs((int)(angle - crossPtsList[i].Bdirct + 180) % 180 - 90) > abs((int)(angle - crossPtsList[i].Wdirct + 180) % 180 - 90)) {
					//printf("Black! ");
					if (maxv - mid_minv > 0.25) keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = 1;
					else if (maxv - mid_minv < 0.1) keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = 0;
					else keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = -1;
				}
				else {
					//printf("White! ");
					if (mid_maxv - minv > 0.25) keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = 1;
					else if (mid_maxv - minv < 0.1) keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = 0;
					else keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = -1;
				}
				/*if (keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] == -1) {
					if (abs(crossPtsList[i].Bdirct - angle) < abs(crossPtsList[i].Wdirct - angle)) {
						if (maxv > 0.7) keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = 1;
						else if (maxv < 0.4) keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = 0;
					}
					else{
						if (minv < 0.45) keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = 1;
						else if (minv > 0.6) keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = 0;
					}
				}*/
				//printf("%d %d %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", matrix[i].mPos.x, matrix[i].mPos.y, crossPtsList[i].Bdirct, crossPtsList[i].Wdirct, angle, mid_maxv, minv, maxv, mid_minv, maxv - mid_minv, mid_maxv - minv);
			}

	/*for (int i = 0; i < crossPtsList.size(); i++)
		printf("%d %d %d\n", matrix[i].mPos.x, matrix[i].mPos.y, keyMatrix[0][matrix[i].mPos.x][matrix[i].mPos.y]);
	*/
	memset(matrixVisit, false, sizeof(matrixVisit));
	keyMatrixValue = 0;

	for (int label = 0; label < labelnum; label++)
		for (int i = 0; i < crossPtsList.size(); i++) {
			keyMatrixValue = 0;
			if ((matrix2[label][matrix[i].mPos.x][matrix[i].mPos.y] == i) && (checkNinePatch(label, matrix[i].mPos.x, matrix[i].mPos.y, keyMatrix)) && (!matrixVisit[i])) {
				binary = 1;
				for (int ib = 0; ib < 3; ib++)
					for (int ia = 0; ia < 3; ia++) {
						if (keyMatrix[label][matrix[i].mPos.x + ia][matrix[i].mPos.y + ib]) 
							keyMatrixValue += binary;
						binary <<= 1;
					}
				distAngle(crossPtsList[i].subPos, crossPtsList[matrix2[label][matrix[i].mPos.x + 1][matrix[i].mPos.y]].subPos, dist, angle);
				if (abs((int)(angle - crossPtsList[i].Bdirct + 180) % 180 - 90) > abs((int)(angle - crossPtsList[i].Wdirct + 180) % 180 - 90)) keyMatrixValue += binary;
			}
			//printf("%d %d %d\n", matrix[i].mPos.x, matrix[i].mPos.y, keyMatrixValue);

			// 更新矩阵绝对坐标
			if ((keyMatrixValue == 0) || (linkTabel[keyMatrixValue].mPos.x == -1)) continue;
			Point relativeMatrixIndex = matrix[i].mPos;
			matrix[i].mPos = linkTabel[keyMatrixValue].mPos;
			matrixVisit[i] = true; 
			for (int j = 0; j < crossPtsList.size(); j++) {
				Point relativeIndex = matrix[j].mPos - relativeMatrixIndex;
				if (!matrixVisit[j] && matrix[j].mLabel == matrix[i].mLabel) {
					matrix[j].mPos.x = matrix[i].mPos.x + coeff[linkTabel[keyMatrixValue].dir][0] * relativeIndex.x + coeff[linkTabel[keyMatrixValue].dir][1] * relativeIndex.y;
					matrix[j].mPos.y = matrix[i].mPos.y + coeff[linkTabel[keyMatrixValue].dir][2] * relativeIndex.x + coeff[linkTabel[keyMatrixValue].dir][3] * relativeIndex.y;
					matrixVisit[j] = true;
				}
			}
			updateSuccess[label] = true;
			break;
		}
	return matrix;
}

void crossMarkDetector::displayMatrix_crosspoint(const Mat& img, std::vector<pointInform>& crossPtsList) {
	Mat imgMark(Dparams.height, Dparams.width, CV_32FC3);
	cvtColor(img, imgMark, COLOR_GRAY2RGB);
	for (int it = 0; it < crossPtsList.size(); ++it) {
		circle(imgMark, crossPtsList[it].Pos, 3, Scalar(0, 1, 0));
	}
	imshow("imgMark", imgMark);
	imwrite("img1.bmp", imgMark * 255);
}

void crossMarkDetector::displayMatrix(const Mat& img, std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, std::vector<linkInform> links, std::vector<Point>& centerpoint, bool update[10], std::vector<Point2f>& cartisian_dst, int cnt) {
	Mat imgMark_disp(Dparams.height, Dparams.width, CV_32FC3);
	cvtColor(img, imgMark_disp, COLOR_GRAY2RGB);
	
	//circle(imgMark, crossPtsList[0].Pos, 10, Scalar(0, 1, 1));
	for (int it = 0; it < crossPtsList.size(); ++it) {
		for (int id = 0; id < 4; ++id) {
			if (links[it].idx[id] != -1) {
				line(imgMark_disp, crossPtsList[links[it].idx[id]].Pos, crossPtsList[it].Pos, Scalar(0, 1, 0), 1);
			}
		}
		circle(imgMark_disp, crossPtsList[it].Pos, 3, Scalar(0, 0, 1));
	}

	for (int i = 0; i < 10; i++)
		//if (update[i])
			for (int it = 0; it < crossPtsList.size(); ++it) 
				if (matrix[it].mLabel == i){
					//String label(std::to_string(it));
					//putText(imgMark, label, crossPtsList[it].Pos+Point(5,-5), FONT_ITALIC, 0.3, Scalar(0,0,1),1);
					String label;
					label.append(std::to_string(matrix[it].mPos.x));
					label.append(",");
					label.append(std::to_string(matrix[it].mPos.y));
					putText(imgMark_disp, label, crossPtsList[it].Pos, FONT_ITALIC, 0.3, Scalar(0, 0, 1), 1);
				}
	
	for (int i = 0; i < centerpoint.size(); i++)
		circle(imgMark_disp, centerpoint[i], 1, Scalar(1, 0, 0));
	
	imshow("imgMark_disp", imgMark_disp);
	char str[100];
	sprintf_s(str, "./img/%d%s", cnt, ".bmp");
	imwrite("imgMark1.bmp", 255 * imgMark_disp);
}

void crossMarkDetector::outputLists(std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, bool update[10]) {
	for (int b = 0; b < 10; b++)
		if (update[b])
			for (int i = 0; i < crossPtsList.size(); i++) {
				if (matrix[i].mLabel == b)
					//if (crossPtsList[i].subPos.x < Dparams.width / 2)
						printf("matrix_coordinate:%d sub-pixel_coordinate:%.3f %.3f\n", matrix[i].mPos.x * 15 + matrix[i].mPos.y, crossPtsList[i].subPos.x, crossPtsList[i].subPos.y);
					//else
					//	printf("[right image] crosspoint_id:%d matrix_coordinate:%d %d matrix_label:%d sub-pixel_coordinate:%.3f %.3f\n", i, matrix[i].mPos.x, matrix[i].mPos.y, matrix[i].mLabel, crossPtsList[i].subPos.x - 640.0, crossPtsList[i].subPos.y);
			}
}

void crossMarkDetector::hydraCode_Mono_Homography(const Mat& img, std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, int labelnum, std::vector<Point2f>& cartisian_dst, bool update[10], int cnt) {
	std::vector<Point2f> srcPoints, dstPoints;
	std::vector<Point3d> srcWorldCoor;

	Mat imgMark(Dparams.height, Dparams.width, CV_32FC3);
	cvtColor(img, imgMark, COLOR_GRAY2RGB);

	int labelnow = 0;
	float squareLength = 0.02;   //每个格子的宽度(单位为米)
	float length = 4 * squareLength; //虚拟立方体的长度

	while (labelnow != labelnum) {
		if (!update[labelnow]) {
			labelnow++;
			continue;
		}
		srcPoints.clear();
		dstPoints.clear();
		for (int i = 0; i < crossPtsList.size(); i++)
			if (matrix[i].mLabel == labelnow) {
				srcPoints.push_back(Point2f((float)matrix[i].mPos.x * 0.020, (float)matrix[i].mPos.y * 0.020));
				dstPoints.push_back(crossPtsList[i].subPos);
			}
		undistortPoints(dstPoints, dstPoints, cameraMatrixL, distCoeffL);
		Mat H = findHomography(srcPoints, dstPoints, RANSAC);
		if (!H.empty()) {
			//decompose homography matrix
			double norm = sqrt(H.at<double>(0, 0) * H.at<double>(0, 0) +
				H.at<double>(1, 0) * H.at<double>(1, 0) +
				H.at<double>(2, 0) * H.at<double>(2, 0));

			H /= norm;

			Mat c1 = H.col(0);
			Mat c2 = H.col(1);
			Mat c3 = c1.cross(c2);

			Mat tvec = H.col(2);
			Mat R(3, 3, CV_64F);

			for (int i = 0; i < 3; i++)
			{
				R.at<double>(i, 0) = c1.at<double>(i, 0);
				R.at<double>(i, 1) = c2.at<double>(i, 0);
				R.at<double>(i, 2) = c3.at<double>(i, 0);
			}
			//std::cout << "R (before polar decomposition):\n" << R << "\ndet(R): " << determinant(R) << std::endl;
			
			//polar decomposition
			Mat W, U, Vt;
			SVDecomp(R, W, U, Vt);
			R = U * Vt;
			std::cout << "R (after polar decomposition):\n" << R << "\ndet(R): " << determinant(R) << std::endl;
			std::cout << tvec << std::endl;

			//display pose
			Mat rvec;
			Rodrigues(R, rvec);

			Point3f CenterPoint = Point3f(7.5 * squareLength - length / 2, 7 * squareLength - length / 2, 0); //Center of the virtual matrix
			// project axes points
			std::vector<Point3f> axesPoints;
			axesPoints.push_back(Point3f(0, 0, 0) + CenterPoint);
			axesPoints.push_back(Point3f(length, 0, 0) + CenterPoint);
			axesPoints.push_back(Point3f(0, length, 0) + CenterPoint);
			axesPoints.push_back(Point3f(0, 0, -length) + CenterPoint);
			axesPoints.push_back(Point3f(0, length, -length) + CenterPoint);
			axesPoints.push_back(Point3f(length, 0, -length) + CenterPoint);
			axesPoints.push_back(Point3f(length, length, 0) + CenterPoint);
			axesPoints.push_back(Point3f(length, length, -length) + CenterPoint);
			std::vector<Point2f> imagePoints;
			projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);

			// draw axes lines
			Point2f center = (imagePoints[0] + imagePoints[1] + imagePoints[2] + imagePoints[6]) / 4;
			circle(imgMark, center, 5, Scalar(0, 255, 0));
			line(imgMark, imagePoints[0], imagePoints[1], Scalar(255, 0, 0), 3);
			line(imgMark, imagePoints[0], imagePoints[2], Scalar(255, 0, 0), 3);
			line(imgMark, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 3);
			line(imgMark, imagePoints[2], imagePoints[4], Scalar(255, 0, 0), 3);
			line(imgMark, imagePoints[2], imagePoints[6], Scalar(255, 0, 0), 3);
			line(imgMark, imagePoints[3], imagePoints[4], Scalar(255, 0, 0), 3);
			line(imgMark, imagePoints[3], imagePoints[5], Scalar(255, 0, 0), 3);
			line(imgMark, imagePoints[1], imagePoints[5], Scalar(255, 0, 0), 3);
			line(imgMark, imagePoints[1], imagePoints[6], Scalar(255, 0, 0), 3);
			line(imgMark, imagePoints[7], imagePoints[4], Scalar(255, 0, 0), 3);
			line(imgMark, imagePoints[7], imagePoints[6], Scalar(255, 0, 0), 3);
			line(imgMark, imagePoints[7], imagePoints[5], Scalar(255, 0, 0), 3);
		}
		labelnow++;
	}
	imshow("imgMark", imgMark);
	char str[100];
	sprintf_s(str, "./img1/%d%s", cnt, ".bmp");
	//imwrite(str, 255 * imgMark);
}

void crossMarkDetector::hydraCode_Mono_PnP(const Mat& img, std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, int labelnum, std::vector<Point2f>& cartisian_dst, bool update[10], int cnt) {
	std::vector<Point3d> srcWorldCoor;
	std::vector<Point2d> dstPoints_pnp;

	Mat imgMark_pnp(Dparams.height, Dparams.width, CV_32FC3);
	cvtColor(img, imgMark_pnp, COLOR_GRAY2RGB);
	int labelnow = 0;
	Point3f Sphere1 = Point3f(0.103119455509292, 0.0202287962890477, 0.332756065064223);
	Point3f Sphere2 = Point3f(0.109737990299978, 0.0498610188026219, 0.334869150792955);
	Point3f Sphere3 = Point3f(0.100385230832603, 0.0520301823252862, 0.334430153267773);
	double sphere_world[3][3];
	double sphere1[3] = { 0.103119455509292, 0.0202287962890477, 0.332756065064223 };
	double sphere2[3] = { 0.109737990299978, 0.0498610188026219, 0.334869150792955 };
	double sphere3[3] = { 0.100385230832603, 0.0520301823252862, 0.334430153267773 };
	
	int val;
	double WorldCoor[16][16][3];
	memset(WorldCoor, 0, sizeof(WorldCoor));
	std::ifstream OpenFile;
	OpenFile.open("registration_new.txt");
	while (!OpenFile.eof()) {
		OpenFile >> val;
		//val = ((val - 1) / 6) * 15 + ((val - 1) % 6);
		OpenFile >> WorldCoor[val / 15][val % 15][0] >> WorldCoor[val / 15][val % 15][1] >> WorldCoor[val / 15][val % 15][2];
	}
	bool sig1 = false;
	
	labelnum = 1; //Delete
	while (labelnow != labelnum) {
		if (!update[labelnow]) {
			labelnow++;
			continue;
		}
		sig1 = true;
		srcWorldCoor.clear();
		dstPoints_pnp.clear();
		for (int i = 0; i < crossPtsList.size(); i++)
			if (matrix[i].mLabel == labelnow) {
				srcWorldCoor.push_back(Point3d(WorldCoor[matrix[i].mPos.x][matrix[i].mPos.y][0] / 1000.0, WorldCoor[matrix[i].mPos.x][matrix[i].mPos.y][1] / 1000.0, WorldCoor[matrix[i].mPos.x][matrix[i].mPos.y][2] / 1000.0));
				dstPoints_pnp.push_back(Point2d((double)crossPtsList[i].subPos.x, (double)crossPtsList[i].subPos.y));
			}
		Mat rvec, tvec, R;
		solvePnPRansac(srcWorldCoor, dstPoints_pnp, cameraMatrixL, distCoeffL, rvec, tvec, false, 50, 1, 0.99, noArray(), SOLVEPNP_UPNP);
		Rodrigues(rvec, R);
		//std::cout << R << std::endl << "det(R): " << determinant(R) << std::endl <<tvec << std::endl;

		memset(sphere_world, 0, sizeof(sphere_world));
		for (int x = 0; x < 3; x++){
			for (int y = 0; y < 3; y++) {
				sphere_world[0][x] += R.at<double>(x, y) * sphere1[y] + tvec.at<double>(x);
				sphere_world[1][x] += R.at<double>(x, y) * sphere2[y] + tvec.at<double>(x);
				sphere_world[2][x] += R.at<double>(x, y) * sphere3[y] + tvec.at<double>(x);
			}	
		}
		//std::cout << "当前为第" << cnt << "帧：" << std::endl;
		//for (int x = 0; x < 3; x++)
		//	printf("%.4f %.4f %.4f\n", sphere_world[x][0], sphere_world[x][1], sphere_world[x][2]);
		
		std::vector<Point3f> axesPoints;
		//绘制轴线
		for (int plotPoints = -5; plotPoints < 12; plotPoints++) {
			axesPoints.push_back(Point3f(0.0483736 + (-0.9980) * plotPoints / 200, 0.0072511 + (-0.0208) * plotPoints / 200, 0.3271322 + (-0.06) * plotPoints / 200));
		}

		axesPoints.push_back(Sphere1);
		axesPoints.push_back(Sphere2);
		axesPoints.push_back(Sphere3);
		std::vector<Point2f> imagePoints;
		projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);
		for (int i = 0; i < axesPoints.size() - 4; i += 2)
			line(imgMark_pnp, imagePoints[i], imagePoints[i + 1], Scalar(120, 0, 0), 2);
		
		circle(imgMark_pnp, imagePoints[axesPoints.size() - 3], 8, Scalar(0, 255, 255));
		circle(imgMark_pnp, imagePoints[axesPoints.size() - 2], 8, Scalar(0, 255, 255));
		circle(imgMark_pnp, imagePoints[axesPoints.size() - 1], 8, Scalar(0, 255, 255));

		//绘制边框
		axesPoints.clear();
		imagePoints.clear();
		for (int i = 0; i < 6; i++)
			axesPoints.push_back(Point3d(WorldCoor[i][0][0] / 1000.0, WorldCoor[i][0][1] / 1000.0, WorldCoor[i][0][2] / 1000.0));
		projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);
		for (int i = 0; i < axesPoints.size() - 1; i++) {
			line(imgMark_pnp, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 0), 2);
			circle(imgMark_pnp, imagePoints[i], 3, Scalar(0, 0, 255));
		}

		axesPoints.clear();
		imagePoints.clear();
		for (int i = 0; i < 6; i++)
			axesPoints.push_back(Point3d(WorldCoor[i][5][0] / 1000.0, WorldCoor[i][5][1] / 1000.0, WorldCoor[i][5][2] / 1000.0));
		projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);
		for (int i = 0; i < axesPoints.size() - 1; i++) {
			line(imgMark_pnp, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 0), 2);
			circle(imgMark_pnp, imagePoints[i], 3, Scalar(0, 0, 255));
		}

		axesPoints.clear();
		imagePoints.clear();
		for (int i = 0; i < 6; i++)
			axesPoints.push_back(Point3d(WorldCoor[0][i][0] / 1000.0, WorldCoor[0][i][1] / 1000.0, WorldCoor[0][i][2] / 1000.0));
		projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);
		for (int i = 0; i < axesPoints.size() - 1; i++) {
			line(imgMark_pnp, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 0), 2);
			circle(imgMark_pnp, imagePoints[i], 3, Scalar(0, 0, 255));
		}

		axesPoints.clear();
		imagePoints.clear();
		for (int i = 0; i < 6; i++)
			axesPoints.push_back(Point3d(WorldCoor[5][i][0] / 1000.0, WorldCoor[5][i][1] / 1000.0, WorldCoor[5][i][2] / 1000.0));
		projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);
		for (int i = 0; i < axesPoints.size() - 1; i++) {
			line(imgMark_pnp, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 0), 2);
			circle(imgMark_pnp, imagePoints[i], 3, Scalar(0, 0, 255));
		}

		axesPoints.clear();
		imagePoints.clear();
		for (int i = 7; i < 13; i++)
			axesPoints.push_back(Point3d(WorldCoor[0][i][0] / 1000.0, WorldCoor[0][i][1] / 1000.0, WorldCoor[0][i][2] / 1000.0));
		projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);
		for (int i = 0; i < axesPoints.size() - 1; i++) {
			line(imgMark_pnp, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 0), 2);
			circle(imgMark_pnp, imagePoints[i], 3, Scalar(0, 0, 255));
		}

		axesPoints.clear();
		imagePoints.clear();
		for (int i = 7; i < 13; i++)
			axesPoints.push_back(Point3d(WorldCoor[15][i][0] / 1000.0, WorldCoor[15][i][1] / 1000.0, WorldCoor[15][i][2] / 1000.0));
		projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);
		for (int i = 0; i < axesPoints.size() - 1; i++) {
			line(imgMark_pnp, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 0), 2);
			circle(imgMark_pnp, imagePoints[i], 3, Scalar(0, 0, 255));
		}

		axesPoints.clear();
		imagePoints.clear();
		for (int i = 0; i < 16; i++)
			axesPoints.push_back(Point3d(WorldCoor[i][12][0] / 1000.0, WorldCoor[i][12][1] / 1000.0, WorldCoor[i][12][2] / 1000.0));
		projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);
		for (int i = 0; i < axesPoints.size() - 1; i++) {
			line(imgMark_pnp, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 0), 2);
			circle(imgMark_pnp, imagePoints[i], 3, Scalar(0, 0, 255));
		}

		axesPoints.clear();
		imagePoints.clear();
		for (int i = 0; i < 16; i++)
			axesPoints.push_back(Point3d(WorldCoor[i][7][0] / 1000.0, WorldCoor[i][7][1] / 1000.0, WorldCoor[i][7][2] / 1000.0));
		projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);
		for (int i = 0; i < axesPoints.size() - 1; i++) {
			line(imgMark_pnp, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 0), 2);
			circle(imgMark_pnp, imagePoints[i], 3, Scalar(0, 0, 255));
		}
		labelnow++;
	}
	if (!sig1)
		printf("0 0 0\n0 0 0\n0 0 0\n");

	imshow("imgMark_pnp", imgMark_pnp);
	char str[100];
	sprintf_s(str, "./img1/%d%s", cnt, ".bmp");
	imwrite(str, 255 * imgMark_pnp);
}

void crossMarkDetector::hydraCode_stereo_ICP(const Mat& img, std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, int labelnum, std::vector<Point2f>& cartisian_dst, bool update[10], int cnt) {
	Mat imgMark_stereo(Dparams.height, Dparams.width, CV_32FC3);
	Mat ide = Mat::eye(3, 3, CV_32FC1);
	cvtColor(img, imgMark_stereo, COLOR_GRAY2RGB);
	
	int val;
	double WorldCoor[16][16][3];
	memset(WorldCoor, 0, sizeof(WorldCoor));
	std::ifstream OpenFile;
	OpenFile.open("registration_new.txt");
	while (!OpenFile.eof()) {
		OpenFile >> val;
		//val = ((val - 1) / 6) * 15 + ((val - 1) % 6);
		OpenFile >> WorldCoor[val / 15][val % 15][0] >> WorldCoor[val / 15][val % 15][1] >> WorldCoor[val / 15][val % 15][2];
	}

	std::vector<Point3f> pts1, pts2;
	for (int i = 0; i < crossPtsList.size() - 1; i++)
		for (int j = i + 1; j < crossPtsList.size(); j++) {
			if ((matrix[i].mPos == matrix[j].mPos) && (update[matrix[i].mLabel]) && (update[matrix[j].mLabel])) {
				if (crossPtsList[i].subPos.x < crossPtsList[j].subPos.x) {
					pts1.push_back(Point3f(WorldCoor[matrix[i].mPos.x][matrix[i].mPos.y][0], WorldCoor[matrix[i].mPos.x][matrix[i].mPos.y][1], WorldCoor[matrix[i].mPos.x][matrix[i].mPos.y][2]));
					//pts2.push_back(Point3f(WorldCoor[matrix[i].mPos.x][matrix[i].mPos.y][0], WorldCoor[matrix[i].mPos.x][matrix[i].mPos.y][1], WorldCoor[matrix[i].mPos.x][matrix[i].mPos.y][2] + 10));
					printf("%d ", matrix[j].mPos.x * 15 + matrix[j].mPos.y);
					pts2.push_back(uv2xyz(crossPtsList[i].subPos, crossPtsList[j].subPos));
				}
				else {
					pts1.push_back(Point3f(WorldCoor[matrix[i].mPos.x][matrix[i].mPos.y][0], WorldCoor[matrix[i].mPos.x][matrix[i].mPos.y][1], WorldCoor[matrix[i].mPos.x][matrix[i].mPos.y][2]));
					//pts2.push_back(Point3f(WorldCoor[matrix[i].mPos.x][matrix[i].mPos.y][0], WorldCoor[matrix[i].mPos.x][matrix[i].mPos.y][1], WorldCoor[matrix[i].mPos.x][matrix[i].mPos.y][2] + 10));
					printf("%d ", matrix[j].mPos.x * 15 + matrix[j].mPos.y);
					pts2.push_back(uv2xyz(crossPtsList[j].subPos, crossPtsList[i].subPos));
				}
				break;
			}
		}
	Point3f p1, p2;
	for (int i = 0; i < pts1.size(); i++) {
		p1 += pts1[i];
		p2 += pts2[i];
	}
	int n = pts1.size();
	p1 = p1 / n; p2 = p2 / n;
	Mat q1 = Mat(3, 1, CV_32FC1); 
	Mat q2 = Mat(1, 3, CV_32FC1);
	Mat W = Mat::zeros(3, 3, CV_32FC1);
	Mat R, tvec, rvec;
	for (int i = 0; i < pts1.size(); i++) {
		q1.ptr<float>(0)[0] = pts1[i].x - p1.x;
		q1.ptr<float>(1)[0] = pts1[i].y - p1.y;
		q1.ptr<float>(2)[0] = pts1[i].z - p1.z;
		q2.ptr<float>(0)[0] = pts2[i].x - p2.x;
		q2.ptr<float>(0)[1] = pts2[i].y - p2.y;
		q2.ptr<float>(0)[2] = pts2[i].z - p2.z;
		W = W + q1 * q2;
	}
	Mat U, S, Vt;
	SVDecomp(W, S, U, Vt);
	R = U * Vt;
	Mat pm1 = (Mat_<float>(3, 1) << p1.x, p1.y, p1.z);
	Mat pm2 = (Mat_<float>(3, 1) << p2.x, p2.y, p2.z);

	tvec = (pm1 - R * pm2) / 1000;
	
	std::cout << R << std::endl << tvec << std::endl;
	Rodrigues(R, rvec);

	std::vector<Point3f> axesPoints;
	//绘制轴线
	for (int plotPoints = -5; plotPoints < 12; plotPoints++) {
		axesPoints.push_back(Point3f(0.0483736 + (-0.9980) * plotPoints / 200, 0.0072511 + (-0.0208) * plotPoints / 200, 0.3271322 + (-0.06) * plotPoints / 200));
	}

	std::vector<Point2f> imagePoints;
	projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);

	// draw axes lines
	for (int i = 0; i < axesPoints.size() - 1; i += 2)
		line(imgMark_stereo, imagePoints[i], imagePoints[i + 1], Scalar(120, 0, 0), 2);
	
	//绘制边框
	axesPoints.clear();
	imagePoints.clear();
	for (int i = 0; i < 6; i++)
		axesPoints.push_back(Point3d(WorldCoor[i][0][0] / 1000.0, WorldCoor[i][0][1] / 1000.0, WorldCoor[i][0][2] / 1000.0));
	projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);
	for (int i = 0; i < axesPoints.size() - 1; i++) {
		line(imgMark_stereo, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 0), 2);
		circle(imgMark_stereo, imagePoints[i], 3, Scalar(0, 0, 255));
	}

	axesPoints.clear();
	imagePoints.clear();
	for (int i = 0; i < 6; i++)
		axesPoints.push_back(Point3d(WorldCoor[i][5][0] / 1000.0, WorldCoor[i][5][1] / 1000.0, WorldCoor[i][5][2] / 1000.0));
	projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);
	for (int i = 0; i < axesPoints.size() - 1; i++) {
		line(imgMark_stereo, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 0), 2);
		circle(imgMark_stereo, imagePoints[i], 3, Scalar(0, 0, 255));
	}

	axesPoints.clear();
	imagePoints.clear();
	for (int i = 0; i < 6; i++)
		axesPoints.push_back(Point3d(WorldCoor[0][i][0] / 1000.0, WorldCoor[0][i][1] / 1000.0, WorldCoor[0][i][2] / 1000.0));
	projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);
	for (int i = 0; i < axesPoints.size() - 1; i++) {
		line(imgMark_stereo, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 0), 2);
		circle(imgMark_stereo, imagePoints[i], 3, Scalar(0, 0, 255));
	}

	axesPoints.clear();
	imagePoints.clear();
	for (int i = 0; i < 6; i++)
		axesPoints.push_back(Point3d(WorldCoor[5][i][0] / 1000.0, WorldCoor[5][i][1] / 1000.0, WorldCoor[5][i][2] / 1000.0));
	projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);
	for (int i = 0; i < axesPoints.size() - 1; i++) {
		line(imgMark_stereo, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 0), 2);
		circle(imgMark_stereo, imagePoints[i], 3, Scalar(0, 0, 255));
	}

	axesPoints.clear();
	imagePoints.clear();
	for (int i = 10; i < 13; i++)
		axesPoints.push_back(Point3d(WorldCoor[0][i][0] / 1000.0, WorldCoor[0][i][1] / 1000.0, WorldCoor[0][i][2] / 1000.0));
	projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);
	for (int i = 0; i < axesPoints.size() - 1; i++) {
		line(imgMark_stereo, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 0), 2);
		circle(imgMark_stereo, imagePoints[i], 3, Scalar(0, 0, 255));
	}

	axesPoints.clear();
	imagePoints.clear();
	for (int i = 7; i < 13; i++)
		axesPoints.push_back(Point3d(WorldCoor[15][i][0] / 1000.0, WorldCoor[15][i][1] / 1000.0, WorldCoor[15][i][2] / 1000.0));
	projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);
	for (int i = 0; i < axesPoints.size() - 1; i++) {
		line(imgMark_stereo, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 0), 2);
		circle(imgMark_stereo, imagePoints[i], 3, Scalar(0, 0, 255));
	}

	axesPoints.clear();
	imagePoints.clear();
	for (int i = 0; i < 16; i++)
		axesPoints.push_back(Point3d(WorldCoor[i][12][0] / 1000.0, WorldCoor[i][12][1] / 1000.0, WorldCoor[i][12][2] / 1000.0));
	projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);
	for (int i = 0; i < axesPoints.size() - 1; i++) {
		line(imgMark_stereo, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 0), 2);
		circle(imgMark_stereo, imagePoints[i], 3, Scalar(0, 0, 255));
	}

	axesPoints.clear();
	imagePoints.clear();
	for (int i = 1; i < 16; i++)
		axesPoints.push_back(Point3d(WorldCoor[i][7][0] / 1000.0, WorldCoor[i][7][1] / 1000.0, WorldCoor[i][7][2] / 1000.0));
	projectPoints(axesPoints, rvec, tvec, cameraMatrixL, distCoeffL, imagePoints);
	for (int i = 0; i < axesPoints.size() - 1; i++) {
		line(imgMark_stereo, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 0), 2);
		circle(imgMark_stereo, imagePoints[i], 3, Scalar(0, 0, 255));
	}

	imshow("imgMark_stereo", imgMark_stereo);
	imwrite("./img_icp/6.bmp", imgMark_stereo * 255);
}

Point3f crossMarkDetector::uv2xyz(Point2f uvLeft, Point2f uvRight)
{
	Mat mLeftRotation = Mat::eye(3, 3, CV_64F);
	Mat mLeftTranslation = Mat::zeros(3, 1, CV_64F);
	Mat mLeftRT = Mat(3, 4, CV_64F);    //左相机M矩阵
	hconcat(mLeftRotation, mLeftTranslation, mLeftRT);
	Mat mLeftM = cameraMatrixL * mLeftRT;
	//cout<<"左相机M矩阵 = "<<endl<<mLeftM<<endl;

	Mat mRightRT = Mat(3, 4, CV_64F);   //右相机M矩阵
	hconcat(Rot, Trans, mRightRT);
	Mat mRightM = cameraMatrixR * mRightRT;
	//cout<<"右相机M矩阵 = "<<endl<<mRightM<<endl;

	uvRight.x -= 1920;
	//最小二乘法A矩阵
	Mat A = Mat(4, 3, CV_64F);
	A.at<double>(0, 0) = uvLeft.x * mLeftM.at<double>(2, 0) - mLeftM.at<double>(0, 0);
	A.at<double>(0, 1) = uvLeft.x * mLeftM.at<double>(2, 1) - mLeftM.at<double>(0, 1);
	A.at<double>(0, 2) = uvLeft.x * mLeftM.at<double>(2, 2) - mLeftM.at<double>(0, 2);

	A.at<double>(1, 0) = uvLeft.y * mLeftM.at<double>(2, 0) - mLeftM.at<double>(1, 0);
	A.at<double>(1, 1) = uvLeft.y * mLeftM.at<double>(2, 1) - mLeftM.at<double>(1, 1);
	A.at<double>(1, 2) = uvLeft.y * mLeftM.at<double>(2, 2) - mLeftM.at<double>(1, 2);

	A.at<double>(2, 0) = uvRight.x * mRightM.at<double>(2, 0) - mRightM.at<double>(0, 0);
	A.at<double>(2, 1) = uvRight.x * mRightM.at<double>(2, 1) - mRightM.at<double>(0, 1);
	A.at<double>(2, 2) = uvRight.x * mRightM.at<double>(2, 2) - mRightM.at<double>(0, 2);

	A.at<double>(3, 0) = uvRight.y * mRightM.at<double>(2, 0) - mRightM.at<double>(1, 0);
	A.at<double>(3, 1) = uvRight.y * mRightM.at<double>(2, 1) - mRightM.at<double>(1, 1);
	A.at<double>(3, 2) = uvRight.y * mRightM.at<double>(2, 2) - mRightM.at<double>(1, 2);

	//最小二乘法B矩阵
	Mat B = Mat(4, 1, CV_64F);
	B.at<double>(0, 0) = mLeftM.at<double>(0, 3) - uvLeft.x * mLeftM.at<double>(2, 3);
	B.at<double>(1, 0) = mLeftM.at<double>(1, 3) - uvLeft.y * mLeftM.at<double>(2, 3);
	B.at<double>(2, 0) = mRightM.at<double>(0, 3) - uvRight.x * mRightM.at<double>(2, 3);
	B.at<double>(3, 0) = mRightM.at<double>(1, 3) - uvRight.y * mRightM.at<double>(2, 3);

	Mat XYZ = Mat(3, 1, CV_64F);
	//采用SVD最小二乘法求解XYZ
	solve(A, B, XYZ, DECOMP_SVD);

	//世界坐标系中坐标
	Point3f world;
	world.x = XYZ.at<double>(0, 0);
	world.y = XYZ.at<double>(1, 0);
	world.z = XYZ.at<double>(2, 0);
	std::cout << world.x << " " << world.y << " " << world.z << std::endl;

	return world;
}

void crossMarkDetector::distAngle(const Point2f A, const Point2f B, float& dist, float& angle)
{
	Point2f v = B - A;
	dist = sqrt(v.x * v.x + v.y * v.y);
	angle = atan2(v.x, -v.y) / CV_PI * 180;
	if (angle < 0) angle = 360 + angle;
}

bool crossMarkDetector::checkIncludedAngle(const float A, const float B, const float T)
{
	float in = abs(A - B);
	if (in > 180) in = 360 - in;
	if (in > T) return 0;
	return 1;
}
