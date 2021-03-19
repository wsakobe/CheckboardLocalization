//
//  crossPointDetector.cpp
//  Endoscopy
//
//  Created by 朱明珠 on 2020/12/4.
//

#include "crossMarkDetector.hpp"
#include <cstring>
#include <iostream>

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

void crossMarkDetector::feed(const Mat& img)
{
	assert(img.type() == CV_32FC1); //判断图片格式是否正确
	if (!signal) {
		signal = true;
		int val;
		for (int i = 0; i < 400; i++) {
			scanf_s("%d", &val);
			scanf_s("%d %d %d", &linkTabel[val].mPos.x, &linkTabel[val].mPos.y, &linkTabel[val].dir);
		}
	}
	findCrossPoint(img, crossPtsList);
	buildMatrix(img, crossPtsList);

	//    imwrite("img.bmp",255*img);
	//    imwrite("imgMark.bmp",255*imgMark);
}

void crossMarkDetector::findCrossPoint(const Mat& img, std::vector<pointInform>& crossPtsList)
{
	// 全图搜索交叉点
	crossPtsList.clear();
	Point curPos;
	for (curPos.y = Rparams.maskR; curPos.y < Dparams.height - Rparams.maskR; ++curPos.y) {
		for (curPos.x = Rparams.maskR; curPos.x < Dparams.width - Rparams.maskR; ++curPos.x) {

			responder.feed(img, curPos);
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
		}
	}
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
		else                    ++ip;
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

void crossMarkDetector::buildMatrix(const Mat& img, std::vector<pointInform>& crossPtsList)
{
	// 建立连接
	std::vector<linkInform> links = buildLinkers(crossPtsList, Dparams.maxSupportAngle);
	// 建立矩阵
	std::vector<matrixInform> matrix(crossPtsList.size());
	std::vector<Point> centerpoint;
	memset(matrix2, -1, sizeof(matrix2));
	std::vector<std::array<Point, 4>> dict(crossPtsList.size()); // 方向传递
	memset(updateSuccess, false, sizeof(updateSuccess));
	int labelNum = 0, angleAB = 180, angleAB_r[500], dir;
	memset(angleAB_r, 0, sizeof(angleAB_r));

	for (int io = 0; io < crossPtsList.size(); ++io) {
		// 准备矩阵起始点
		if (matrix[io].mLabel != -1) continue;
		matrix[io].mPos = Point(10, 10);
		matrix[io].mLabel = labelNum;
		dict[io] = { Point(-1,0),Point(0,-1),Point(1,0),Point(0,1) };
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

				if (crossPtsList[linkPt].Pos.x != crossPtsList[it].Pos.x)
					angleAB = atan2(crossPtsList[linkPt].Pos.y - crossPtsList[it].Pos.y, crossPtsList[linkPt].Pos.x - crossPtsList[it].Pos.x) / CV_PI * 180;
				else if (crossPtsList[linkPt].Pos.y < crossPtsList[it].Pos.y) angleAB = -90;
				else angleAB = 90;
				angleAB += 225;
				if (angleAB >= 360) angleAB = 0;
				/*
				if (angleAB - angleAB_r[it] <= -45){
					if (angleAB - angleAB_r[it] > -135) { 
						dir = 1;
						matrix[linkPt].mPos = matrix[it].mPos + dict[it][dir]; 
						Point a = dict[it][0];
						for (int i = 1; i < 4; i++)
							dict[linkPt][i - 1] = dict[it][i];
						dict[linkPt][3] = a;
					}
					else { 
						dir = 2; 
						matrix[linkPt].mPos = matrix[it].mPos + dict[it][dir];
						Point a = dict[it][0];
						Point b = dict[it][1];
						for (int i = 2; i < 4; i++)
							dict[linkPt][i - 2] = dict[it][i];
						dict[linkPt][2] = a;
						dict[linkPt][3] = b;
					}
				}
				else {
					if (angleAB - angleAB_r[it] > 135) {
						dir = 2; 
						matrix[linkPt].mPos = matrix[it].mPos + dict[it][dir];
						Point a = dict[it][0];
						Point b = dict[it][1];
						for (int i = 2; i < 4; i++)
							dict[linkPt][i - 2] = dict[it][i];
						dict[linkPt][2] = a;
						dict[linkPt][3] = b;
					}
					else if (angleAB - angleAB_r[it] < 45) { 
						dir = 0; 
						matrix[linkPt].mPos = matrix[it].mPos + dict[it][dir];
						dict[linkPt] = dict[it]; 
					}
					else {
						dir = 3; 
						matrix[linkPt].mPos = matrix[it].mPos + dict[it][dir];
						Point a = dict[it][3];
						for (int i = 3; i > 0; i--)
							dict[linkPt][i] = dict[it][i - 1];
						dict[linkPt][0] = a;
					}
				}
				*/
				matrix[linkPt].mPos = matrix[it].mPos + dict[0][angleAB / 90];
				matrix[linkPt].mLabel = matrix[it].mLabel;
				matrix2[labelNum - 1][matrix[linkPt].mPos.x][matrix[linkPt].mPos.y] = linkPt;
				member.push_back(linkPt);
				angleAB_r[linkPt] = angleAB;
			}
		}
	}
	matrix = extractLinkTable(img, crossPtsList, matrix, links, matrix2, labelNum, centerpoint);
	hydraCode(img, crossPtsList, matrix, labelNum, cartisian_dst, updateSuccess);
	//displayMatrix(img, crossPtsList, matrix, links, centerpoint, updateSuccess, cartisian_dst);
	outputLists(crossPtsList, matrix, updateSuccess);
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
				pixel[0] = img.at<float>((int)centerPoint.y, (int)centerPoint.x);
				pixel[1] = img.at<float>((int)centerPoint.y, (int)centerPoint.x - 1);
				pixel[2] = img.at<float>((int)centerPoint.y - 1, (int)centerPoint.x);
				pixel[3] = img.at<float>((int)centerPoint.y + 1, (int)centerPoint.x);
				pixel[4] = img.at<float>((int)centerPoint.y, (int)centerPoint.x + 1);
				//mid_pixel[0] = img.at<float>(((int)pos1.x + (int)centerPoint.x) / 2, ((int)pos1.y + (int)centerPoint.y) / 2);
				//mid_pixel[1] = img.at<float>((int)(pos2.x * 0.7 + centerPoint.x * 0.3), (int)(pos2.y * 0.7 + centerPoint.y * 0.3));
				//mid_pixel[2] = img.at<float>((int)(pos3.x * 0.7 + centerPoint.x * 0.3), (int)(pos3.y * 0.7 + centerPoint.y * 0.3));
				//mid_pixel[3] = img.at<float>((int)(crossPtsList[i].subPos.x * 0.8 + centerPoint.x * 0.2), (int)(crossPtsList[i].subPos.y * 0.8 + centerPoint.y * 0.2));
				float maxv = 0, minv = 1;
				for (int j = 0; j <= 4; j++) {
					if (pixel[j] > maxv)
						maxv = pixel[j];
					if (pixel[j] < minv)
						minv = pixel[j];
				}
				/*float mid_maxv = 0, mid_minv = 1;
				for (int j = 0; j < 4; j++) {
					if (mid_pixel[j] > mid_maxv)
						mid_maxv = mid_pixel[j];
					if (mid_pixel[j] < mid_minv)
						mid_minv = mid_pixel[j];
				}*/
				distAngle(crossPtsList[i].subPos, crossPtsList[matrix2[label][matrix[i].mPos.x + 1][matrix[i].mPos.y]].subPos, dist, angle);
				/*if (abs(crossPtsList[i].Bdirct - angle) < abs(crossPtsList[i].Wdirct - angle)) {
					if (maxv - mid_minv > 0.25) keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = 1;
					else if (maxv - mid_minv < 0) keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = 0;
					else keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = -1;
				}
				else if (mid_maxv - minv > 0.25) keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = 1;
					else if (mid_maxv - minv < 0) keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = 0;
					else keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = -1;
				if ((matrix[i].mPos.x + matrix[i].mPos.y) % 2) printf("%d %d %d %.2f %.2f %.2f %.2f %.2f\n", label, matrix[i].mPos.x, matrix[i].mPos.y, maxv - mid_minv, maxv, minv, mid_maxv, mid_minv);
				else printf("%d %d %d %.2f %.2f %.2f %.2f %.2f\n", label, matrix[i].mPos.x, matrix[i].mPos.y, mid_maxv - minv, maxv, minv, mid_maxv, mid_minv);
				*/
				if (abs(crossPtsList[i].Bdirct - angle) < abs(crossPtsList[i].Wdirct - angle))
					if (maxv > 0.5) keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = 1;
					else if (maxv < 0.3) keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = 0;
					else keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = -1;
				if (abs(crossPtsList[i].Bdirct - angle) >= abs(crossPtsList[i].Wdirct - angle))
					if (minv < 0.55) keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = 1;
					else if (minv > 0.65) keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = 0;
					else keyMatrix[label][matrix[i].mPos.x][matrix[i].mPos.y] = -1;
			}

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
				if (abs(crossPtsList[i].Bdirct - angle) < abs(crossPtsList[i].Wdirct - angle)) keyMatrixValue += binary;
				//printf("%d %d %d %d\n", label, matrix[i].mPos.x, matrix[i].mPos.y, keyMatrixValue);
			}
			
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

void crossMarkDetector::displayMatrix(const Mat& img, std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, std::vector<linkInform> links, std::vector<Point>& centerpoint, bool update[10], std::vector<Point2f>& cartisian_dst) {
	Mat imgMark(Dparams.height, Dparams.width, CV_32FC3);
	cvtColor(img, imgMark, COLOR_GRAY2RGB);
	for (int it = 0; it < crossPtsList.size(); ++it) {
		for (int id = 0; id < 4; ++id) {
			if (links[it].idx[id] != -1) {
				line(imgMark, crossPtsList[links[it].idx[id]].Pos, crossPtsList[it].Pos, Scalar(0, 1, 0), 1);
			}
		}
		circle(imgMark, crossPtsList[it].Pos, 3, Scalar(0, 0, 1));
	}
	for (int i = 0; i < 10; i++)
		if (update[i])
			for (int it = 0; it < crossPtsList.size(); ++it) 
				if (matrix[it].mLabel == i){
					//        String label(std::to_string(it));
					//        putText(imgMark, label, crossPtsList[it].Pos+Point(5,-5), FONT_ITALIC, 0.3, Scalar(0,0,1),1);
					String label;
					label.append(std::to_string(matrix[it].mPos.x));
					label.append(",");
					label.append(std::to_string(matrix[it].mPos.y));
					putText(imgMark, label, crossPtsList[it].Pos, FONT_ITALIC, 0.3, Scalar(0, 0, 1), 1);
				}

	for (int i = 0; i < centerpoint.size(); i++)
		circle(imgMark, centerpoint[i], 1, Scalar(1, 0, 0));
	
	imshow("imgMark", imgMark);
	imwrite("imgMark.bmp", 255 * imgMark);		
}

void crossMarkDetector::outputLists(std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, bool update[10]) {
	for (int b = 0; b < 10; b++)
		if (update[b])
			for (int i = 0; i < crossPtsList.size(); i++) {
				if (matrix[i].mLabel == b)
					if (crossPtsList[i].subPos.x < Dparams.width / 2)
						printf("[left image] crosspoint_id:%d matrix_coordinate:%d %d matrix_label:%d sub-pixel_coordinate:%.3f %.3f\n", i, matrix[i].mPos.x, matrix[i].mPos.y, matrix[i].mLabel, crossPtsList[i].subPos.x, crossPtsList[i].subPos.y);
					else
						printf("[right image] crosspoint_id:%d matrix_coordinate:%d %d matrix_label:%d sub-pixel_coordinate:%.3f %.3f\n", i, matrix[i].mPos.x, matrix[i].mPos.y, matrix[i].mLabel, crossPtsList[i].subPos.x - 640.0, crossPtsList[i].subPos.y);
			}
}

void crossMarkDetector::hydraCode(const Mat& img, std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, int labelnum, std::vector<Point2f>& cartisian_dst, bool update[10]) {
	std::vector<Point2f> srcPoints;
	std::vector<Point2f> dstPoints;
	std::vector<Point2f> cartisian_src(3);

	Mat imgMark(Dparams.height, Dparams.width, CV_32FC3);
	cvtColor(img, imgMark, COLOR_GRAY2RGB);

	int labelnow = 0;
	cartisian_src[0] = Point2f(6, 6); cartisian_src[1] = Point2f(6, 9); cartisian_src[2] = Point2f(9, 6);

	while (labelnow != labelnum) {
		if (!update[labelnow]) {
			labelnow++;
			continue;
		}
		srcPoints.clear();
		dstPoints.clear();
		for (int i = 0; i < crossPtsList.size(); i++)
			if (matrix[i].mLabel == labelnow) {
				srcPoints.push_back(matrix[i].mPos);
				dstPoints.push_back(crossPtsList[i].subPos);
			}
		Mat H = findHomography(srcPoints, dstPoints, RANSAC);
		if (!H.empty()) {
			perspectiveTransform(cartisian_src, cartisian_dst, H);
			arrowedLine(imgMark, cartisian_dst[0], cartisian_dst[1], Scalar(255, 0, 0), 3, 8);
			arrowedLine(imgMark, cartisian_dst[0], cartisian_dst[2], Scalar(0, 255, 0), 3, 8);
		}
		labelnow++;
	}
	imshow("imgMark", imgMark);
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
