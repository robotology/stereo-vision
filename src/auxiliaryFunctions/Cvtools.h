#include <iostream>
#include <fstream>
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
	static IplImage* thresholdColor(IplImage *im, int threshold);
	static void binaryThresh(IplImage* img, IplImage* mask, IplImage * res);
    static bool checkTS(double TSLeft, double TSRight);
    static void saveStereoImage(const char * dir, IplImage* left, IplImage * right, int num);
    static void saveImgSegDisp(const char * dir, IplImage * img, IplImage* foreground, double norm, int num);
    static void preparePath(const char * dir, char* pathL, char* pathR, int num);
    static void preparePath(const char * dir, char* path1, char* path2, char * path3, int count);
    static void drawPoints(Mat& Img, vector<Point2f> Points);
    static void computeContrastandOrientation(IplImage* img, IplImage* arctan, IplImage* contrast);
    static void computeHOG(IplImage* image, CvHistogram* histTemp);
    static void stampaIstogrammi1D(CvHistogram* hist, int n_bins, int scale, char* nameWindow);
    static void getContour(IplImage* mask, IplImage* maschera);
};

