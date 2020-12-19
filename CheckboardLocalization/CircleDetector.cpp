#include "CircleDetector.hpp"

using namespace std;
using namespace cv;

void CircleDetect::DetectCircle(Mat img) {
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
	
	vector<KeyPoint> key_points;
	detector->detect(img, key_points);

	//for (int i = 0; i < key_points.size(); i++)
		//printf("%d %d %d\n", (int)key_points[i].pt.x, (int)key_points[i].pt.y, i);

	Mat output_img;

	drawKeypoints(img, key_points, output_img, Scalar(255, 0, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	namedWindow("SimpleBlobDetector");
	imshow("SimpleBlobDetector", output_img);
	waitKey(1);
}