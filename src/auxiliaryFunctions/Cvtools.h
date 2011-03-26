#include <iostream>
#include <string>
#include <cv.h>
#include <highgui.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

class Cvtools
{
public:
	static IplImage* readImage(string path);
	static IplImage* thresholdColor(IplImage *im);
	static IplImage* binaryThresh(IplImage *img,IplImage * mask);
    static boolean checkTS(double TSLeft, double TSRight);
    static void saveStereoImage(const char * dir, IplImage* left, IplImage * right, int num);
    static void preparePath(const char * dir, char* pathL, char* pathR, int num);
    static void drawPoints(Mat& Img, vector<Point2f> Points);
};

