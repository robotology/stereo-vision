#include <iostream>
#include <string>
#include <cv.h>
#include <highgui.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "Camera.h"
#include "levmar.h"
#include "Cvtools.h"

using namespace std;
using namespace cv;

void printMatrix(Mat &matrix);
Point3f triangulation(Point2f& pointleft, Point2f& pointRight, Mat Camera1, Mat Camera2);
void funcMeas(double *p, double *x, int m, int n, void *data);
void getSolutionfromVariables(Mat& Rot, Mat& Tras, vector<Point3f>& WorldPoints, double * vars);
void reprojection(Mat& Rot, Mat& Tras, Mat Kleft, Mat Kright, vector<Point2f> PointsL, vector<Point2f> PointsR, double * proj);


class stereoCamera 
{
private:
    Mat imleft; // Left Image
    Mat imleftgray; // Left Image Gray
    Mat imleftund; // Undistorted Image
    Mat imright; // Right Image
    Mat imrightgray; // Right Image Gray
    Mat imrightund; // Undistorted Image Right
    Mat Disparity; // Disparity Map Image
    Mat Disparity16; // Disparity 16 Bit Signed

    Mat Pleft; // Camera Matrix Left 3x4
    Mat Pright; // Camera Matrix Right 3x4
    Mat Kleft; // Intrinsic Parameters Left 3x3
    Mat Kright; // Intrinsic Parameters Right 3x3
    Mat DistL; // Distortion Coefficients Left 4x1
    Mat DistR; // Distortion Coefficients Right 4x1
    Mat R; // Rotation from Left to Right 3x3
    Mat T; // Translation from Left to Right 3x1
    Mat Q; // Depth Matrix 4x4
    Mat E; // Essential Matrix


    vector<Point2f> PointsL; // Match Left
    vector<Point2f> PointsR; // Match Right
  
    vector<Point2f> InliersL; // Inliers Left
    vector<Point2f> InliersR; // Inliers Right

    bool readStringList( const string& filename, vector<string>& l );
    void runStereoCalib(const vector<string>& imagelist, Size boardSize,float sqsize);
    void getMatch(const Mat& descL, const vector<KeyPoint>& keypointsL, const Mat& descR, const  vector<KeyPoint>& keypointsR,  vector<DMatch>& matches, bool draw);
    int getBestDistance(const float *di, const Mat& allDesc);
    void chierality( Mat& R1,  Mat& R2,  Mat& t1,  Mat& t2, Mat& R, Mat& t);
    double * prepareVariables(Mat& R, Mat& T,vector<Point3f>& WorldPoints); 
    double* reprojectionError(Mat& Rot, Mat& Tras);
    void updatePMatrix();


public:
    stereoCamera() {}; // Costruttore vuoto quando si vuole calibrare Tutto a partire dalle immagini
    stereoCamera(string intrinsicFileName, string exstrinsicFileName); // Costruttore quando si ha già la calibrazione
    stereoCamera(Camera Left, Camera Right); // Costruttore quando sono già noti i parametri intrinseci

    void stereoCalibration(string imagesFilePath, int boardWidth, int boardHeight,float sqsize);
    void stereoCalibration(vector<string> imageList, int boardWidth, int boardHeight, float sqsize);

    void saveCalibration(string extrinsicFilePath, string intrinsicFilePath);
    void setImages(IplImage* left, IplImage* right);
    void findMatch(); 
    void computeDisparity(); 
    double reprojectionErrorAvg();
    void optimization();
    void undistortImages();
   
    Point3f triangulation(Point2f& pointleft, Point2f& pointRight);
    Point3f triangulation(Point2f& pointleft, Point2f& pointRight, Mat Camera1, Mat Camera2); 
    void estimateEssential();
    void essentialDecomposition();
  
    void printStereoIntrinsic();
    void printStereoDist();
    void printExtrinsic();
    void printStereoCameras();
    const Mat getImLeft();
    const Mat getImRight();
    const Mat getImLeftGray();
    const Mat getImRightGray();
    const Mat getImLeftUnd();
    const Mat getImRightUnd();
    const Mat getDisparity();
    const Mat getDisparity16();
    const Mat getQ();
    const Mat getKleft();
    const Mat getKright();
    const Mat getWorldPoints();
    const vector<Point2f> getMatchLeft();
    const vector<Point2f> getMatchRight();
    const Mat getTranslation();
    const Mat getRotation();
    void setRotation(Mat & Rot, int mul=0);
    void setTranslation(Mat &Tras, int mul=0);


};