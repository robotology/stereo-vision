/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Vadim Tikhanoff
 * email:   vadim.tikhanoff@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txtd
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <iostream>
#include <string>
#include <cv.h>
#include <highgui.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using cv::Mat;
using cv::Point2f;
using cv::Size;
using cv::Point3f;
using cv::FileStorage;
using cv::FileNodeIterator;
using cv::FileNode;
using cv::DescriptorExtractor;
/**
* \ingroup StereoVisionLib
*
* The base class defining a simple camera. 
*  
* It allows to calibrate the camera and to undistort a pair of images.
*/
class Camera
{
private:
    Mat K; // Intrinsic Parameters 3x3
    Mat P; // Camera Matrix 3x4
    Mat Dist; // Distortion Coefficents Vector 4x1
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
    void printCameraMatrix();
    void printDistortionVector();
    void calibrate(string imagesFilePath, int boardWidth, int boardHeight);

public:

    /**
    * Costructor for initialization from file.
    * @param intrisicFileName the path of the intrinsic parameters file.
    */
    Camera(string intrinsicFilePath);

    /**
    * Default Constructor. You should initialize all the intrinsic and extrinsic parameters
    * using the calibrate method
    */
    Camera() {};

    /**
    * It performs the camera calibration.
    * @param imageList is the list containing the paths of the images with the chessboard patterns.
    * @param boardWidth the number of inner corners in the width direction of the chess board pattern (see \ref stereoCalibration module)
    * @param boardHeight the number of inner corners in the height direction of the chess board pattern (see \ref stereoCalibration module)
    */
    void calibrate(vector<string> imageList, int boardWidth, int boardHeight);

    /**
    * It saves the calibration
    * @param intrinsicFilePath the path of the intrinsic parameters file
    */
    bool saveCalibration(string intrinsicFilePath);

    /** It undistorts an image using the intrinsic parameters. 
    * @note Set undistortion coefficients and camera matrix before using this method.
    * @param image the input distorted image.
    * @return the undistorted image.
    */
    Mat undistortImage(Mat image);


    /**
    * It returns the 3x3 camera matrix.
    * @return 3x3 camera matrix.
    */
    Mat getCameraMatrix();

    /**
    * It returns the 4x1 distortion coefficients.
    * @return 4x1 distortion coefficients.
    */
    Mat getDistVector();

    /**
    * It sets the camera parameters.
    * @param K 3x3 camera matrix.
    */
    void setCameraMatrix(Mat& K);

    /**
    * It sets the distortion parameters.
    * @param D 4x1 camera distortion parameters.
    */
    void setDistCoefficients(Mat& Dist);



};