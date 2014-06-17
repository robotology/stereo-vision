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

#include "iCub/stereoVision/camera.h"

Camera::Camera(string intrinsicFilePath) {
        FileStorage fs(intrinsicFilePath.c_str(), CV_STORAGE_READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", intrinsicFilePath.c_str());
            return;
        }
        fs["camera_matrix"] >> K;
        fs["distortion_coefficients"] >> Dist;

}



void Camera::printMatrix(Mat &matrix) {
    cout << endl;
    int row=matrix.rows;
    int col =matrix.cols;

    for(int i = 0; i < matrix.rows; i++)
    {
        const double* Mi = matrix.ptr<double>(i);
        for(int j = 0; j < matrix.cols; j++)
            cout << Mi[j] << " ";
        cout << endl;
    }
    cout << endl;
}
void Camera::calibrate(vector<string> imageList, int boardWidth, int boardHeight) {
    vector<vector<Point2f> > imagePoints;
    Size boardSize, imageSize;
    boardSize.width=boardWidth;
    boardSize.height=boardHeight;
    int flags=0;
    int i;

    float squareSize = 1.f, aspectRatio = 1.f;

      Mat view, viewGray;

    for(i = 0; i<(int)imageList.size();i++)
    {

         view = cv::imread(imageList[i], 1);
         imageSize = view.size();
         vector<Point2f> pointbuf;
         cvtColor(view, viewGray, CV_BGR2GRAY); 

         bool found = findChessboardCorners( view, boardSize, pointbuf,
                                            CV_CALIB_CB_ADAPTIVE_THRESH & CV_CALIB_CB_FAST_CHECK & CV_CALIB_CB_NORMALIZE_IMAGE);

         if(found) 
         {
            drawChessboardCorners( view, boardSize, Mat(pointbuf), found );
            imagePoints.push_back(pointbuf);
         }

    }
    prepareandRunCalibration(imagePoints, imageSize, boardSize, squareSize, aspectRatio, flags, K, Dist);


}

void Camera::calibrate(string imagesFilePath, int boardWidth, int boardHeight) {
    vector<vector<Point2f> > imagePoints;
    vector<string> imageList;
    Size boardSize, imageSize;
    boardSize.width=boardWidth;
    boardSize.height=boardHeight;
    int flags=0;
    int i;

    float squareSize = 1.f, aspectRatio = 1.f;

    // Read Image Calibration List
    if(!readStringList(imagesFilePath, imageList))
    {
        cout << "Cannot Open Image List File. Calibration Failed" << endl;
        return;
    }

    Mat view, viewGray;

    for(i = 0; i<(int)imageList.size();i++)
    {

        view = cv::imread(imageList[i], 1);
         imageSize = view.size();
         vector<Point2f> pointbuf;
         cvtColor(view, viewGray, CV_BGR2GRAY); 

         bool found = findChessboardCorners( view, boardSize, pointbuf,
                                            CV_CALIB_CB_ADAPTIVE_THRESH & CV_CALIB_CB_FAST_CHECK & CV_CALIB_CB_NORMALIZE_IMAGE);


         if(found) 
         {
            drawChessboardCorners( view, boardSize, Mat(pointbuf), found );
            imagePoints.push_back(pointbuf);
         }

    }
    prepareandRunCalibration(imagePoints, imageSize, boardSize, squareSize, aspectRatio, flags, K, Dist);

}

bool Camera::readStringList( const string& filename, vector<string>& l )
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}




bool Camera::prepareandRunCalibration(const vector<vector<Point2f> >& imagePoints,
                Size imageSize, Size boardSize, float squareSize,
                float aspectRatio, int flags, Mat& cameraMatrix,
                Mat& distCoeffs)
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;
    
    bool ok = runCalibration(imagePoints, imageSize, boardSize, squareSize,
                   aspectRatio, flags, cameraMatrix, distCoeffs,
                   rvecs, tvecs, reprojErrs, totalAvgErr);
    printf("%s. avg reprojection error = %.2f\n",
           ok ? "Calibration succeeded" : "Calibration failed",
           totalAvgErr);
    
     return ok;

}

bool Camera::runCalibration( vector<vector<Point2f> > imagePoints,
                    Size imageSize, Size boardSize,
                    float squareSize, float aspectRatio,
                    int flags, Mat& cameraMatrix, Mat& distCoeffs,
                    vector<Mat>& rvecs, vector<Mat>& tvecs,
                    vector<float>& reprojErrs,
                    double& totalAvgErr)
{
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = aspectRatio;
    
    distCoeffs = Mat::zeros(4, 1, CV_64F);
    
    vector<vector<Point3f> > objectPoints(1);
    calcChessboardCorners(boardSize, squareSize, objectPoints[0]);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);
    
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                    distCoeffs, rvecs, tvecs,CV_CALIB_FIX_K3);
    printf("RMS error reported by calibrateCamera: %g\n", rms);
    
    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);
    
    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}

void Camera::calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners)
{
    corners.resize(0);
    
    for( int i = 0; i < boardSize.height; i++ )
        for( int j = 0; j < boardSize.width; j++ )
            corners.push_back(Point3f(float(j*squareSize),
                                      float(i*squareSize), 0));
}

double Camera::computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs,
        vector<float>& perViewErrors )
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());
    
    for( i = 0; i < (int)objectPoints.size(); i++ )
    {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err*err/n);
        totalErr += err*err;
        totalPoints += n;
    }
    
    return std::sqrt(totalErr/totalPoints);
}


bool Camera::saveCalibration(string intrinsicFilePath) {
  
    if(this->K.empty()) {
        printf("Camera is not calibrated, run calibration method before..\n");
        return false;
    }
   FileStorage fs( intrinsicFilePath, FileStorage::WRITE );
    fs << "camera_matrix" << this->K;
    fs << "distortion_coefficients" << this->Dist;
   return true;
}

void Camera::printCameraMatrix() 
{   if(this->K.empty())
     printf("Camera is not calibrated, run calibration method before..\n");
    else
     printMatrix(this->K);
}

void Camera::printDistortionVector() 
{   if(this->Dist.empty())
     printf("Camera is not calibrated, run calibration method before..\n");
    else
     printMatrix(this->Dist);
}


Mat Camera::getCameraMatrix() {
    return this->K;
}
Mat Camera::getDistVector() {
    return this->Dist;
}

Mat Camera::undistortImage(Mat image) {
    Mat imund;
    undistort(image,imund,K,Dist);
    return imund;
}

void Camera::setCameraMatrix(Mat& K) {
    this->K=K;
}

void Camera::setDistCoefficients(Mat &Dist) {
    this->Dist=Dist;
}
