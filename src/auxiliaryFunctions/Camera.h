#include <iostream>
#include <string>
#include <cv.h>
#include <highgui.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

class Camera
{
private:
    Mat K; // Intrinsic Parameters
    Mat P; // Camera Matrix 3x4
    Mat Dist; // Distortion Coefficents Vector
    bool readStringList( const string& filename, vector<string>& l );
    bool prepareandRunCalibration(const vector<vector<Point2f> >& imagePoints,
                Size imageSize, Size boardSize, float squareSize,
                float aspectRatio, int flags, Mat& cameraMatrix,
                Mat& distCoeffs);
    bool runCalibration( vector<vector<Point2f> > imagePoints,
                    Size imageSize, Size boardSize,
                    float squareSize, float aspectRatio,
                    int flags, Mat& cameraMatrix, Mat& distCoeffs,
                    vector<Mat>& rvecs, vector<Mat>& tvecs,
                    vector<float>& reprojErrs,
                    double& totalAvgErr);
    void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners);
    double computeReprojectionErrors(
    const vector<vector<Point3f> >& objectPoints,
    const vector<vector<Point2f> >& imagePoints,
    const vector<Mat>& rvecs, const vector<Mat>& tvecs,
    const Mat& cameraMatrix, const Mat& distCoeffs,
    vector<float>& perViewErrors );
    void printMatrix(Mat &matrix);

public:
    Camera(string intrinsicFilePath);
    Camera() {};

    void calibrate(string imagesFilePath, int boardWidth, int boardHeight);
    void calibrate(vector<string> imageList, int boardWidth, int boardHeight);
    bool saveCalibration(string intrinsicFilePath);
    IplImage * undistortion(IplImage* image);

    void printCameraMatrix();
    void printDistortionVector();

    Mat getIntrinsic();
    Mat getDistVector();


};