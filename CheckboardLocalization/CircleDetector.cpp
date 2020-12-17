#include "CircleDetector.hpp"

using namespace std;
using namespace cv;

void CircleDetect::DetectCircle(Mat img) {
	SimpleBlobDetector::Params params;
	params.filterByColor = false;
	params.minThreshold = 80;
	params.maxThreshold = 160;
	params.thresholdStep = 5;

	params.minArea = 20;
	params.maxArea = 100;

	params.minConvexity = .75f;
	params.minInertiaRatio = .65f;
	
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
	
	vector<KeyPoint> key_points;
	detector->detect(img, key_points);

	//for (int i = 0; i < key_points.size(); i++)
	//	printf("%d %d %d\n", (int)key_points[i].pt.x, (int)key_points[i].pt.y, i);
	
	drawKeypoints(img, key_points, img, Scalar(255, 0, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	imshow("CircleDetector", img);

	waitKey(1);
}