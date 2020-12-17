#include "crossMarkDetector.hpp"
#include "CalcMatrix.h"
#include "CircleDetector.hpp"

constexpr auto PI = 3.1415926;

using namespace std;
using namespace cv;

inline float Norm(int x, int y) {
	return sqrt(x * x + y * y);
}

void PredictCorners(CPFilter_WorkSpace& FWS, int idx1[100], int idx2[100], int idx3[100], int size);
void AssignClosestCorners(int Chessboard[100][100], CPFilter_WorkSpace& FWS, Point2f pred[100], int size);

bool Matrix::CalcMat(CPFilter_WorkSpace& FWS) {
	printf("Recoverying...\n");
	memset(UsedPoint, false, sizeof(UsedPoint));
	if (FWS.crossNum < 9) return false;
	for (int i = 0; i < FWS.crossNum; i++) {
		InitChessboard(FWS, i);
		
		if ((IsEmpty(Chessboard, FWS.crossNum)) || (ChessboardEnergy(Chessboard, FWS) > 0)) return false;
		Chess_sizex = 3, Chess_sizey = 3;

		while (1) {
			int dir = -1;
			float EnergySum = ChessboardEnergy(Chessboard, FWS);
			memset(Prop_Chessboard, 0, sizeof(Prop_Chessboard));
			for (int i = 0; i < Chess_sizex; i++)
				for (int j = 0; j < Chess_sizey; j++)
					for (int k = 0; k < 4; k++)
						Prop_Chessboard[k][i][j] = Chessboard[i][j];
			for (int j = 0; j < 4; j++) {
				GrowChessboard(Chessboard, FWS, j);
				Prop_Energy[j] = ChessboardEnergy(Prop_Chessboard[j], FWS);

				if (Prop_Energy[j] < EnergySum) {
					EnergySum = Prop_Energy[j];
					dir = j;
				}
			}
			if (dir == -1) {
				printf("Chessboard Generated\n");
				PlotChessboard(Chessboard, FWS);
				return true;
			}
		}
	}
}

void Matrix::InitChessboard(CPFilter_WorkSpace& FWS, int idx) {
	int v1 = FWS.whiteLine[idx];
	int v2 = FWS.blackLine[idx];
	Chessboard[1][1] = idx;

	Chessboard[0][1] = FindDirectionalNeighbor(idx, -v2, Chessboard, FWS);
	Chessboard[2][1] = FindDirectionalNeighbor(idx,  v2, Chessboard, FWS);
	Chessboard[1][0] = FindDirectionalNeighbor(idx, -v1, Chessboard, FWS);
	Chessboard[1][2] = FindDirectionalNeighbor(idx,  v1, Chessboard, FWS);

	Chessboard[0][0] = FindDirectionalNeighbor(Chessboard[0][1], -v1, Chessboard, FWS);
	Chessboard[2][2] = FindDirectionalNeighbor(Chessboard[2][1],  v1, Chessboard, FWS);
	Chessboard[0][2] = FindDirectionalNeighbor(Chessboard[1][2], -v2, Chessboard, FWS);
	Chessboard[2][0] = FindDirectionalNeighbor(Chessboard[1][0],  v2, Chessboard, FWS);

	mean_dist1 = (dist1[1] + dist1[2] + dist1[3] + dist1[4]) / 4;
	mean_dist2 = (dist2[1] + dist2[2] + dist2[3] + dist2[4]) / 4;
	std_dist1 = sqrt(((dist1[1] - mean_dist1) * (dist1[1] - mean_dist1) + (dist1[2] - mean_dist1) * (dist1[2] - mean_dist1) + (dist1[3] - mean_dist1) * (dist1[3] - mean_dist1) + (dist1[4] - mean_dist1) * (dist1[4] - mean_dist1)) / 4);
	std_dist2 = sqrt(((dist2[1] - mean_dist2) * (dist2[1] - mean_dist2) + (dist2[2] - mean_dist2) * (dist2[2] - mean_dist2) + (dist2[3] - mean_dist2) * (dist2[3] - mean_dist2) + (dist2[4] - mean_dist2) * (dist2[4] - mean_dist2)) / 4);
	
	//剔除错误连接
	if (((std_dist1 / mean_dist1) < 0.3) || ((std_dist2 / mean_dist2) < 0.3)) {
		memset(Chessboard, 0, sizeof(Chessboard));
		memset(UsedPoint, false, sizeof(UsedPoint));
		return;
	}
}

int Matrix::FindDirectionalNeighbor(int idx, int vec, int Chessboard[100][100], CPFilter_WorkSpace& FWS) {
	memset(dist_final, 100000.0, sizeof(dist_final));
	for (int i = 0; i < FWS.crossNum; i++) {
		if ((!UsedPoint[i]) && (i != idx)) {
			dist[i].dir_x = FWS.crossPos[i].x - FWS.crossPos[idx].x;
			dist[i].dir_y = FWS.crossPos[i].y - FWS.crossPos[idx].y;
			dist[i].distance = sqrt(dist[i].dir_x * dist[i].dir_x + dist[i].dir_y * dist[i].dir_y);
			dist[i].theta = (!dist[i].dir_x) ? (dist[i].dir_y > 0 ? 90 : -90) : (dist[i].dir_y * 1.0 / dist[i].dir_x);
			dist_final[i] = dist[i].distance * cos((vec - dist[i].theta) / 180.0 * PI) + 5 * dist[i].distance * sin((vec - dist[i].theta) / 180.0 * PI);
		}
	}
	float MinDist = 100000.0;
	int Pos = -1;
	for (int i = 0; i < FWS.crossNum; i++) {
		if (dist_final[i] < MinDist) {
			MinDist = dist_final[i];
			Pos = i;
		}
	}
	if (Pos != -1) {
		UsedPoint[Pos] = true;
		return Pos;
	}
}

bool Matrix::IsEmpty(int Chessboard[100][100], int cnt) {
	for (int i = 0; i < cnt; i++)
		if (UsedPoint[i]) return false;
	return true;
}

float Matrix::ChessboardEnergy(int Chessboard[100][100], CPFilter_WorkSpace& FWS) {
	Energy.Corner = -Chess_sizex * Chess_sizey;
	Energy.Structure = 0;

	//x方向Energy(Structure)
	for (int i = 0;i < Chess_sizex; i++)
		for (int j = 0; j < Chess_sizey - 2; j++) {
			int idx1 = Chessboard[i][j];
			int idx2 = Chessboard[i][j + 1];
			int idx3 = Chessboard[i][j + 2];
			Energy.Structure = max(Energy.Structure, Norm(FWS.crossPos[idx1].x + FWS.crossPos[idx3].x - 2 * FWS.crossPos[idx2].x, FWS.crossPos[idx1].y + FWS.crossPos[idx3].y - 2 * FWS.crossPos[idx2].y) / Norm(FWS.crossPos[idx1].x - FWS.crossPos[idx3].x, FWS.crossPos[idx1].y - FWS.crossPos[idx3].y));
		}

	//y方向Energy(Structure)
	for (int i = 0; i < Chess_sizey; i++)
		for (int j = 0; j < Chess_sizex - 2; j++) {
			int idx1 = Chessboard[i][j];
			int idx2 = Chessboard[i + 1][j];
			int idx3 = Chessboard[i + 2][j];
			Energy.Structure = max(Energy.Structure, Norm(FWS.crossPos[idx1].x + FWS.crossPos[idx3].x - 2 * FWS.crossPos[idx2].x, FWS.crossPos[idx1].y + FWS.crossPos[idx3].y - 2 * FWS.crossPos[idx2].y) / Norm(FWS.crossPos[idx1].x - FWS.crossPos[idx3].x, FWS.crossPos[idx1].y - FWS.crossPos[idx3].y));
		}

	return Energy.Corner + Chess_sizex * Chess_sizey * Energy.Structure;
}

void Matrix::GrowChessboard(int Chessboard[100][100], CPFilter_WorkSpace& FWS, int dir) {
	int idx1[100], idx2[100], idx3[100];
	switch (dir) {
		case 0: //向上方搜寻
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < Chess_sizey; j++) {
					switch (i) {
						case 0: idx1[j] = Chessboard[i][j]; break;
						case 1:	idx2[j] = Chessboard[i][j]; break;
						case 2:	idx3[j] = Chessboard[i][j]; break;
						default:break;
					}
				}
			PredictCorners(idx1, idx2, idx3, Chess_sizey);
			AssignClosestCorners(Pred, Chess_sizey, dir);
			break;
		case 1: //向右侧搜寻
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < Chess_sizex; j++) {
					switch (i) {
						case 0: idx1[j] = Chessboard[j][i + Chess_sizey - 1]; break;
						case 1:	idx2[j] = Chessboard[j][i + Chess_sizey - 2]; break;
						case 2:	idx3[j] = Chessboard[j][i + Chess_sizey - 3]; break;
						default:break;
					}
				}
			PredictCorners(idx1, idx2, idx3, Chess_sizex);
			AssignClosestCorners(Pred, Chess_sizey, dir);
			break;
		case 2: //向下方搜寻
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < Chess_sizey; j++) {
					switch (i) {
					case 0: idx1[j] = Chessboard[i][j]; break;
					case 1:	idx2[j] = Chessboard[i][j]; break;
					case 2:	idx3[j] = Chessboard[i][j]; break;
					default:break;
					}
				}
			PredictCorners(idx1, idx2, idx3, Chess_sizey);
			AssignClosestCorners(Pred, Chess_sizey, dir);
			break;
		case 3: //向左侧搜寻
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < Chess_sizey; j++) {
					switch (i) {
					case 0: idx1[j] = Chessboard[i][j]; break;
					case 1:	idx2[j] = Chessboard[i][j]; break;
					case 2:	idx3[j] = Chessboard[i][j]; break;
					default:break;
					}
				}
			PredictCorners(idx1, idx2, idx3, Chess_sizey);
			AssignClosestCorners(Pred, Chess_sizey, dir);
			break;
	}
}

void PredictCorners(CPFilter_WorkSpace& FWS, int idx1[100], int idx2[100], int idx3[100], int size) {
	for (int i = 0; i < size; i++) {
		float v1x, v2x, v1y, v2y, a1, a2, s1, s2;
		v1x = FWS.crossPos[idx2[i]].x - FWS.crossPos[idx1[i]].x;
		v2x = FWS.crossPos[idx3[i]].x - FWS.crossPos[idx2[i]].x;
		v1y = FWS.crossPos[idx2[i]].y - FWS.crossPos[idx1[i]].y;
		v2y = FWS.crossPos[idx3[i]].y - FWS.crossPos[idx2[i]].y;
		a1 = atan(v1y / v1x);
		a2 = atan(v2y / v2x);
		s1 = sqrt(v1x * v1x + v1y * v1y);
		s2 = sqrt(v2x * v2x + v2y * v2y);
		Pred[i].x = FWS.crossPos[idx3[i]].x + 0.75 * (2 * s2 - s1) * cos(2 * a2 - a1);
		Pred[i].y = FWS.crossPos[idx3[i]].y + 0.75 * (2 * s2 - s1) * sin(2 * a2 - a1);
	}
}

bool cmp(const delta& x, const delta& y) {
	return x.dis < y.dis;
}

void AssignClosestCorners(int Chessboard[100][100], CPFilter_WorkSpace& FWS, Point2f Pred[100], int size, int dir) {
	for (int i = 0; i < size; i++) {
		memset(Delta, 0, sizeof(Delta));
		for (int j = 0; j < FWS.crossNum; j++) {
			if (!Used[j])
				Delta[i].dis = sqrt((Pred[i].x - FWS.crossPos[j].x) * (Pred[i].x - FWS.crossPos[idx3[i]].x) + (Pred[i].x - FWS.crossPos[j].x) * (Pred[i].y - FWS.crossPos[j].y));
			else
				Delta[i].dis = 10000.0;
		}			
		sort(Delta, Delta + size, cmp);
		if (!i) TransChessboard(dir);
		switch (dir) {
			case 0:
				Prop_Chessboard[dir][0][i] = Delta[0].idx; break;
			case 1:
				Prop_Chessboard[dir][i][size] = Delta[0].idx; break;
			case 2:
				Prop_Chessboard[dir][size][i] = Delta[0].idx; break;
			case 3:
				Prop_Chessboard[dir][i][0] = Delta[0].idx; break;
			default: break;
		}
	}
}

void Matrix::TransChessboard(int dir) {
	switch (dir) {
		case 0:
			for (int i = Chess_sizex; i > 0; i--)
				for (int j = 0; j < Chess_sizey; j++) {
					Prop_Chessboard[dir][i][j] = Chessboard[i - 1][j];
				}
			break;
		case 1:	break;
		case 2:	break;
		case 3:
			for (int i = Chess_sizey; i > 0; i--)
				for (int j = 0; j < Chess_sizex; j++) {
					Prop_Chessboard[dir][i][j] = Chessboard[i - 1][j];
				}
			break;
	}
}

void Matrix::PlotChessboard(int Chessboard[100][100], CPFilter_WorkSpace& FWS) {
	for (int i = 0; i < Chess_sizex; i++) {
		for (int j = 0; j < Chess_sizey; j++)
			printf("%d ", Chessboard[i][j]);
		printf("\n");
	}
}