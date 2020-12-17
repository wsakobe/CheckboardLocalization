#pragma once
#ifndef __CALCMATRIX_H__
#define __CALCMATRIX_H__
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"
#include <opencv2/highgui/highgui_c.h>
#include "Endoscopy.hpp"

class Matrix {
private:
	int Chess_sizex, Chess_sizey;
	int Prop_Chessboard[5][100][100];

	bool UsedPoint[100];
	
	float dist1[5], dist2[5], mean_dist1, mean_dist2, std_dist1, std_dist2, dist_final[100];
	float Prop_Energy[5];

	struct dist {
		int dir_x, dir_y;
		int edge_x, edge_y;
		float distance;
		float theta;
	}dist[100];

	struct energy {
		float Corner;
		float Structure;
	}Energy;

	struct delta {
		float dis;
		int idx;
	}Delta[100];


public:
	int   Chessboard[100][100];
	Point2f Pred[100];
	bool  CalcMat(CPFilter_WorkSpace& FWS);
	void  InitChessboard(CPFilter_WorkSpace& FWS, int idx);
	bool  IsEmpty(int Chessboard[100][100], int cnt);
	float ChessboardEnergy(int Chessboard[100][100], CPFilter_WorkSpace& FWS);
	void  GrowChessboard(int Chessboard[100][100], CPFilter_WorkSpace& FWS, int dir);
	void  PlotChessboard(int Chessboard[100][100], CPFilter_WorkSpace& FWS);
	void  TransChessboard(int dir);
	int   FindDirectionalNeighbor(int idx, int vec, int Chessboard[100][100], CPFilter_WorkSpace& FWS);
};

#endif