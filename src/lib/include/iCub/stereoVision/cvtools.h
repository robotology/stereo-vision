#include <iostream>
#include <fstream>
#include <string>
#include <cv.h>
#include <highgui.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using cv::Mat;
using cv::Point2f;

class Cvtools
{
public:
    static void thresholdColor(IplImage *im, IplImage* mask, int threshold, int x=0, int y=0, double value=0);
    static void thresholdBW(IplImage* im, IplImage* mask, int threshold, int x=0, int y=0, double value=0);
    static void binaryThresh(IplImage* img, IplImage* mask, IplImage * res);
    static bool checkTS(double TSLeft, double TSRight, double th=0.020);
    static void saveStereoImage(const char * dir, IplImage* left, IplImage * right, int num);
    static void saveImgSegDisp(const char * dir, IplImage * img, IplImage* foreground, double norm, int num);
    static void preparePath(const char * dir, char* pathL, char* pathR, int num);
    static void preparePath(const char * dir, char* path1, char* path2, char * path3, int count);
    static void drawPoints(Mat& Img, vector<Point2f> Points);
    static void computeContrastandOrientation(IplImage* img, IplImage* arctan, IplImage* contrast);
    static void computeHOG(IplImage* image, CvHistogram* histTemp);
    static void showHist1D(CvHistogram* hist, int n_bins, int scale, char* nameWindow);
    static void getContour(IplImage* mask, IplImage* maschera, int x = 160, int y=120);
    static void drawMotionField(IplImage* imgU, IplImage* imgV, IplImage* imgMotion, int xSpace, int ySpace, float cutoff, float multiplier, CvScalar color);
};

