//
//  crossPointDetector.cpp
//  Endoscopy
//
//  Created by 朱明珠 on 2020/12/4.
//

#include "crossMarkDetector.hpp"

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

void crossMarkDetector::feed(const Mat& img, const Mat& img1)
{
	assert(img.type() == CV_32FC1); //判断图片格式是否正确
	findCrossPoint(img, crossPtsList);
	circleDetector(img1);
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

void crossMarkDetector::circleDetector(const Mat& img) {
	SimpleBlobDetector::Params params;
	params.filterByColor = false;
	params.minThreshold = 80;
	params.maxThreshold = 160;
	params.thresholdStep = 5;

	params.minArea = 10;
	params.maxArea = 150;

	params.minConvexity = .65f;
	params.minInertiaRatio = .65f;

	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

	detector->detect(img, key_points);
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
	std::vector<std::array<Point, 4>> dict(crossPtsList.size()); // 方向传递
	int matrix2[10][20][20];
	memset(matrix2, -1, sizeof(matrix2));
	dict[0] = { Point(-1,0),Point(0,-1),Point(1,0),Point(0,1) };
	int labelNum = 0;
	for (int io = 0; io < crossPtsList.size(); ++io) {
		// 准备矩阵起始点
		if (matrix[io].mLabel != -1) continue;
		matrix[io].mPos = Point(10, 10);
		matrix[io].mLabel = labelNum;
		++labelNum;
		std::vector<int> member;
		member.push_back(io);
		// 生长
		for (int ig = 0; ig < member.size(); ++ig) {
			int it = member[ig];
			for (int il = 0; il < 4; ++il) {
				int linkPt = links[it].idx[il];
				if (linkPt == -1 || matrix[linkPt].mLabel != -1) continue;

				int angleAB = 270;
				if (crossPtsList[linkPt].Pos.x != crossPtsList[it].Pos.x)
					angleAB = atan2(crossPtsList[linkPt].Pos.y - crossPtsList[it].Pos.y, crossPtsList[linkPt].Pos.x - crossPtsList[it].Pos.x) / CV_PI * 180 + 180;

				matrix[linkPt].mPos = matrix[it].mPos + dict[0][angleAB / 90];
				matrix[linkPt].mLabel = matrix[it].mLabel;
				matrix2[labelNum - 1][matrix[linkPt].mPos.x][matrix[linkPt].mPos.y] = linkPt;
				member.push_back(linkPt);
			}
		}
	}
	extractLinkTable(crossPtsList, matrix, links, matrix2, labelNum);
	displayMatrix(img, crossPtsList, matrix, links);
}

bool checkNinePatch(int point, int label, int x, int y, int matrix2[10][20][20]) {
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			if (matrix2[label][x + i][y + j] == -1) return false;
	return true;
}

void crossMarkDetector::extractLinkTable(std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, std::vector<linkInform> links, int matrix2[10][20][20], int labelnum) {
	int Chessboard[4][4] = { -1 };
	float thresCircleCenter = 10.0;

	for (int label = 0; label < labelnum; label++)
		for (int i = 0; i <= crossPtsList.size(); i++) {
			if (checkNinePatch(i, label, matrix[i].mPos.x, matrix[i].mPos.y, matrix2)) {
				printf("%d %d\n", matrix[i].mPos.x, matrix[i].mPos.y);
				int keyMatrixValue = 0;
				int binary = 1;
				for (int ib = 0; ib < 3; ib++)
					for (int ia = 0; ia < 3; ia++) {
						int pos1 = matrix2[label][matrix[i].mPos.x + ia][matrix[i].mPos.y + ib];
						int pos2 = matrix2[label][matrix[i].mPos.x + ia + 1][matrix[i].mPos.y + ib + 1];
						for (int kp = 0; kp < key_points.size(); kp++) {
							if ((abs(crossPtsList[pos1].subPos.x + crossPtsList[pos2].subPos.x - 2 * key_points[kp].pt.x) < thresCircleCenter) && (abs(crossPtsList[pos1].subPos.y + crossPtsList[pos2].subPos.y - 2 * key_points[kp].pt.y) < thresCircleCenter)) {
								keyMatrixValue += binary;
								break;
							}
						}
						binary <<= 1;
					}
				float dist, angle;
				distAngle(crossPtsList[i].subPos, crossPtsList[matrix2[label][matrix[i].mPos.x + 1][matrix[i].mPos.y]].subPos, dist, angle);
				if (abs(crossPtsList[i].Bdirct - angle) < abs(crossPtsList[i].Wdirct - angle)) keyMatrixValue += binary;
				printf("%d\n", keyMatrixValue);
				break;
			}
		}
}

void crossMarkDetector::displayMatrix(const Mat& img, std::vector<pointInform>& crossPtsList, std::vector<matrixInform> matrix, std::vector<linkInform> links) {
	Mat imgMark(Dparams.height, Dparams.width, CV_32FC3);
	cvtColor(img, imgMark, COLOR_GRAY2RGB);
	for (int it = 0; it < crossPtsList.size(); ++it) {
		for (int id = 0; id < 4; ++id) {
			if (links[it].idx[id] != -1) {
				line(imgMark, crossPtsList[links[it].idx[id]].Pos, crossPtsList[it].Pos, Scalar(0, 1, 0), 1);
			}
		}
	}
	for (int it = 0; it < crossPtsList.size(); ++it) {
		//        String label(std::to_string(it));
		//        putText(imgMark, label, crossPtsList[it].Pos+Point(5,-5), FONT_ITALIC, 0.3, Scalar(0,0,1),1);
		String label;
		label.append(std::to_string(matrix[it].mPos.x));
		label.append(",");
		label.append(std::to_string(matrix[it].mPos.y));
		putText(imgMark, label, crossPtsList[it].Pos, FONT_ITALIC, 0.3, Scalar(0, 0, 1), 1);
	}

	for (int i = 0; i < key_points.size(); i++)
		circle(imgMark, Point((int)key_points[i].pt.x, (int)key_points[i].pt.y), (int)(key_points[i].size / 2), Scalar(1, 0, 0));
	
	imshow("imgMark", imgMark);
	//    imwrite("img.bmp", 255*img);
	imwrite("imgMark.bmp", 255 * imgMark);
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
