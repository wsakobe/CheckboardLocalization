#include "crossMarkDetector.hpp"
#include "CircleDetector.hpp"
#include "crossPointResponder.hpp"

using namespace std;
using namespace cv;

void Plot(Mat img) {
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

    imshow("imgMark", imgMark);
    imwrite("imgMark.bmp", 255 * imgMark);
}