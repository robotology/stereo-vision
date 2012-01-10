/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Sean Ryan Fanello
 * email:  sean.fanello@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/**
 * \defgroup StereoVisionLib stereoVisionLib
 *  
 * @ingroup icub_stereoVision 
 *  
 * Vision library for dealing with stereo camera calibration,
 * 3D points generation and motion estimation
 *
 * \author Sean Ryan Fanello 
 *  
 * Copyright (C) 2010 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0. 
 *
 * \section intro_sec Description
 *
 * The library relies on OpenCV 2.2 or greater. It implements the Camera and StereoCamera classes.
 *  
 */ 
#include <iostream>
#include <string>
#include <cv.h>
#include <highgui.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iCub/stereoVision/camera.h>
#include <fstream>
#include <yarp/os/all.h>

using namespace std;
using namespace yarp::os;

using cv::Mat;
using cv::Point2f;
using cv::Point3f;
using cv::Size;
using cv::Ptr;
using cv::FileStorage;
using cv::DMatch;
using cv::FileNode;
using cv::FeatureDetector;
using cv::DescriptorMatcher;
using cv::KeyPoint;
using cv::StereoSGBM;
using cv::Rect;
using cv::SVD;
using cv::Vec3f;
using cv::Vector;
using cv::Range;
using cv::Scalar;
using cv::TermCriteria;
/**
* \ingroup StereoVisionLib
*
* The base class defining stereo camera. 
*  
* It allows to calibrate the cameras, to undistort a pair of images, to find matches between two images,
* to triangulate points and to estimate motion between two images. The basic assumption is that the two images come from
* a stereo camera, however this class works also with two arbitrary images.
*/
class StereoCamera 
{
private:
    // stored Images
    Mat imleft; // Left Image
    Mat imleftund; // Undistorted Image
    Mat imright; // Right Image
    Mat imrightund; // Undistorted Image Right
    Mat Disparity; // Disparity Map Image
    Mat Disparity16; // Disparity 16 Bit Signed
    Mat imgLeftRect; // Rectified Left Image
    Mat imgRightRect; // Rectified Right Image

    // stored Matrices
    Mat Pleft; // Camera Matrix Left 3x4
    Mat Pright; // Camera Matrix Right 3x4
    Mat Kleft; // Intrinsic Parameters Left 3x3
    Mat Kright; // Intrinsic Parameters Right 3x3
    Mat DistL; // Distortion Coefficients Left 4x1
    Mat DistR; // Distortion Coefficients Right 4x1
    Mat Rinit; // Initial Rotation 3x3
    Mat Tinit; // Initial Translation 3x3
    Mat R; // Rotation from Left to Right 3x3
    Mat T; // Translation from Left to Right 3x1
    Mat Q; // Depth Matrix 4x4
    Mat E; // Essential Matrix 3x3
    Mat RLrect; // Rotation from Left Camera to Rectified Left Camera 3x3
    Mat RRrect; // Rotation from Right Camera to Rectified Right Camera 3x3
    Mat PLrect; // Rectified Left Camera Matrix
    Mat PRrect; // Rectified Right Camera Matrix
    Mat map11; //Mapping from from rectified to original
    Mat map12; //Mapping from from rectified to original
    Mat map21; //Mapping from from rectified to original
    Mat map22; //Mapping from from rectified to original


    Mat MapperL; // pixels mapping from original left camera to rectified left camera
    Mat MapperR; // pixels mapping from original right camera to rectified right camera

    vector<Point2f> PointsL; // Match Left
    vector<Point2f> PointsR; // Match Right
  
    vector<Point2f> InliersL; // Inliers Left
    vector<Point2f> InliersR; // Inliers Right

    Semaphore* mutex;
    bool cameraChanged;

    bool readStringList( const string& filename, vector<string>& l );
    void runStereoCalib(const vector<string>& imagelist, Size boardSize,float sqsize);
    double* reprojectionError(Mat& Rot, Mat& Tras);
    void crossCheckMatching( Ptr<DescriptorMatcher>& descriptorMatcher, const Mat& descriptors1, const Mat& descriptors2, vector<DMatch>& filteredMatches12, double radius, int knn=1);
    void updatePMatrix();
    void stereoCalibration(string imagesFilePath, int boardWidth, int boardHeight,float sqsize);
    void normalizePoints(Mat & K1, Mat & K2, vector<Point2f> & PointsL, vector<Point2f> & PointsR);
    void getRotation(Mat & q, Mat & R);
    void printMatrix(Mat &matrix);
    double reprojectionErrorAvg();
    void savePoints(string pointsLPath, string pointsRPath, vector<Point2f>  PointL, vector<Point2f>  PointR);
    void printStereoIntrinsic();
    void printExtrinsic();
    Mat buildRotTras(Mat & R, Mat & T);


public:

    /**
    * Default Constructor. You should initialize all the intrinsic and extrinsic parameters
    * using the provided methods.
    */
    StereoCamera() {}; 

    ~StereoCamera() { delete mutex; };

    /**
    * Costructor for initialization from file.
    * @param intrisicFileName the path of the intrinsic parameters file.
    * @param extrinsicFileName the path of the extrinsic parameters file.
    */
    StereoCamera(string intrinsicFileName, string exstrinsicFileName);

    /**
    * Constructor for initialization using two calibrated cameras.
    * @note Only intrinsic parameters are initialized.
    * @param First the first camera (Left eye is assumed but you can use any arbitrary camera). The 3D point coordinates
    * have this reference system.
    * @param Second the second camera (Right eye is assumed).
    */
    StereoCamera(Camera First, Camera Second);

    /**
    * It performs the stereo camera calibration. (see \ref stereoCalibration module)
    * @param imageList is the list containing the paths of the images with the chessboard patterns. even indices refer to
    * Left camera images (i.e. main camera images), while odd indices refer to Right camera images.
    * @param boardWidth the number of inner corners in the width direction of the chess board pattern (see \ref stereoCalibration module)
    * @param boardHeight the number of inner corners in the height direction of the chess board pattern (see \ref stereoCalibration module)
    * @param sqsize the size of the square of the chess board pattern. It is needed for a metric reconstruction.
    */
    void stereoCalibration(vector<string> imageList, int boardWidth, int boardHeight, float sqsize=1.0);


    /**
    * It saves the calibration
    * @param extrinsicFilePath the path of the extrinsic parameters file
    * @param intrinsicFilePath the path of the intrinsic parameters file
    */
    void saveCalibration(string extrinsicFilePath, string intrinsicFilePath);

    /**
    * It stores in memory a couple of images
    * @param firstImg the images acquired from the first (main) camera
    * @param secondImg the images acquired from the second (secondary) camera
    */
    void setImages(IplImage* firstImg, IplImage* secondImg);

    /**
    * It finds matches between two images. SIFT detector and descriptor is used.
    * @note Run setImages and indistortImages methods before using this method.
    * @param visualize true if you want to visualize matches between images
    * @param displacement maximum pixel displacement between first and second camera
    * @param radius maximum radius between the first candidate match and the second one
    */
    void findMatch(bool visualize=false,double displacement=15, double radius=0.25); 

    /** It computes the Disparity Map using H. Hirschmuller Algorithm (CVPR 2006) (see \ref stereoDisparity).
    * @param best set equal true for better accuracy, equal false for save computation.
    * @param uniquenessRatio The margin in percents by which the best (minimum) computed cost function value should “win” the second best value to consider the found match correct. Normally, some value within 5-15 range is good enough.
    * @param speckleWindowSize Maximum size of smooth disparity regions to consider them noise speckles and invdalidate. Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in 50-200 range.
    * @param speckleRange Maximum disparity variation within each connected component. If you do speckle filtering, set it to some positive value, multiple of 16. Normally, 16 or 32 is good enough.
    * @note Run the calibration or set all the parameters before using this method.
    */
    void computeDisparity(bool best=true, int uniquenessRatio=15, int speckleWindowSize=50,int speckleRange=16); 

    /** It undistorts the images. 
    * @note Set undistortion coefficients before using this method.
    */
    void undistortImages();

    /**
    * It performs the horn relative orientations algorithm i.e. it estimates the motion from one camera to another one using a initial guess.
    * A good initial guess can be obtained using the essentialDecomposition method.
    * @param K1 3x3 matrix with intrinsic parameters of the first camera
    * @param K2 3x3 matrix with intrinsic parameters of the second camera
    * @param Points1 matches in the first image
    * @param Points2 matches in the second image
    * @param Rot initial rotation (3x3 matrix) guess. The new output rotation is stored here
    * @param Tras initial translation (3x1 matrix) guess. The new output translation is stored here
    */
    void horn(Mat &K1,Mat &K2, vector<Point2f> &Points1,vector<Point2f> &Points2,Mat &Rot,Mat &Tras);

    /**
    * It performs the horn relative orientations, all the parameters are assumed initialized in the StereoCamera object. The new output Rotation and Translation matrices are
    * stored in the R and T members. 
    */
    void hornRelativeOrientations(); 

    /**
    * It performs the triangulation using the stored in the internal P1 and P2 3x4 Camera Matrices. The triangulation obtained is not metric! Use the method metricTriangulation if you want a metric triangulation.
    * @param point1 the 2D point coordinates in the first image.
    * @param point2 the 2D point coordinates in the second image.
    * @return a 3D point wrt the first camera reference system.
    *
    */
    Point3f triangulation(Point2f& point1, Point2f& point2);

   /**
    * It performs the triangulation. The triangulation obtained is not metric! Use the method metricTriangulation if you want a metric triangulation.
    * @param point1 the 2D point coordinates in the first image.
    * @param point2 the 2D point coordinates in the second image.
    * @param Camera1 the 3x4 camera matrix of the first image. 
    * @param Camera2 the 3x4 camera matrix of the second image. 
    * @return a 3D point wrt the first camera reference system.
    *
    */
    Point3f triangulation(Point2f& point1, Point2f& point2, Mat Camera1, Mat Camera2); 

  /**
    * It performs the metric triangulation given the pixel coordinates on the first image. Run compute disparity before using this method.
    * @param point1 the pixel coordinates in the first image.
    * @return a metric 3D point w.r.t. the first camera reference system.
    *
    */
    Point3f metricTriangulation(Point2f &point1,double thMeters=10);
  /**
    * It performs the metric triangulation given the pixel coordinates on the first image. The 3D Point is w.r.t the system defined by the parameter H. Run compute disparity before using this method.
    * @param point1 the pixel coordinates in the first image.
    * @param H the 4x4 rototranslation matrix of the system.
    * @return a metric 3D point w.r.t. the reference system defined by H.
    *
    */
    Point3f metricTriangulation(Point2f &point1, Mat &H, double thMeters=10);

      /**
    * It performs the metric triangulation given the pixel coordinates on the first image and the disparity between the two RECTIFIED images. The 3D Point is w.r.t the system defined by the parameter H.
    * @param u the pixel x coordinate in the first image.
    * @param v the pixel y coordinate in the first image.
    * @param d the disparity on the x coordinate between the two rectified images.
    * @param H the 4x4 rototranslation matrix of the system can be an empty matrix.
    * @return a metric 3D point w.r.t. the reference system defined by H.
    *
    */
    Point3f triangulateKnownDisparity(float u, float v, float d, Mat &H);

    /** 
    * It estimates the essential matrix (3x3) E between two views. The output is stored in the private member E.
    */
    void estimateEssential();

    /**
    * It decomposes the essential matrix in Rotation and Translation between the two views. The output is stored in the private members R and T.
    */
    void essentialDecomposition();

    /**
    * It performs the chierality test: given a couple of rotation matrices, translation vectors and matches it finds the correct rotation and translation s.t.
    * the triangulated points have their depth coordinates greater than 0. The method is used by essentialDecomposition, indeed an essential matrix generates 2 rotations and 2 translation. The chierality
    * test is needed in order to discard wrong rototranslations.
    * @param R1 first rotation 3x3 matrix
    * @param R2 second rotation 3x3 matrix
    * @param t1 first translation 3x1 matrix
    * @param t2 second translation 3x1 matrix
    * @param R output rotation matrix
    * @param t output translation matrix
    * @param points1 corrispondences in the first image
    * @param points2 corrispondences in the second image
    * 
    */
    void chierality( Mat& R1,  Mat& R2,  Mat& t1,  Mat& t2, Mat& R, Mat& t, Vector<Point2f> points1, Vector<Point2f> points2);


    /**
    * It returns the left (first) image.
    * @return the left (first) image.
    */
    const Mat getImLeft();

    /**
    * It returns the right (second) image.
    * @return the right (second) image.
    */
    const Mat getImRight();

    /**
    * It returns the left undistorted image.
    * @return the left undistorted image.
    */
    const Mat getImLeftUnd();

    /**
    * It returns the right undistorted image.
    * @return the right undistorted image.
    */
    const Mat getImRightUnd();

    /**
    * It returns the disparity image.
    * @return the disparity image computed via computeDisparity(). The image is 8 bit unsigned.
    */
    const Mat getDisparity();

    /**
    * It returns the disparity image.
    * @return the disparity image computed via computeDisparity(). The image is 16 bit signed.
    */
    const Mat getDisparity16();

    /**
    * It returns the 4x4 disparity-to-depth mapping matrix.
    * @return 4x4 disparity-to-depth mapping matrix.
    */
    const Mat getQ();

    /**
    * It returns the 3x3 left camera matrix.
    * @return 3x3 left camera matrix.
    */
    const Mat getKleft();

    /**
    * It returns the 3x3 right camera matrix.
    * @return 3x3 right camera matrix.
    */
    const Mat getKright();

    /**
    * It returns the pixel coordinates of the matches in the left image.
    * @return 3x3 pixel coordinates of the matches in the left image.
    */
    const vector<Point2f> getMatchLeft();

    /**
    * It returns the pixel coordinates of the matches in the right image.
    * @return 3x3 pixel coordinates of the matches in the right image.
    */
    const vector<Point2f> getMatchRight();

    /**
    * It returns the translation vector between the two cameras.
    * @return 3x1 translation matrix between the first and the second camera.
    */
    const Mat getTranslation();

    /**
    * It returns the rotation matrix between the two cameras.
    * @return 3x3 rotation matrix between the first and the second camera.
    */
    const Mat getRotation();


    /**
    * It returns the mapping between the original left camera and the rectified left camera.
    * @return a 16 bit signed 2 channel image containing the mapping from the original left camera to the rectified left camera.
    */
    const Mat getMapperL();

    /**
    * It returns the mapping between the original right camera and the rectified right camera.
    * @return a 16 bit signed 2 channel image containing the mapping from the original right camera to the rectified right camera.
    */
    const Mat getMapperR();

    /**
    * It returns the rotation matrix between the original left camera and the rectified left camera.
    * @return 3x3 rotation matrix between the original left camera and the rectified left camera.
    */
    const Mat getRLrect();

   /**
    * It returns the rotation matrix between the original right camera and the rectified right camera.
    * @return 3x3 rotation matrix between the original right camera and the rectified right camera.
    */
    const Mat getRRrect();

    /**
    * It sets the rotation matrix (if known) between the first and the second camera.
    * @param Rot the 3x3 rotation matrix.
    * @param mode the following values are allowed:
    * @b mode=0 the rotation matrix R is set equal to Rot.
    * @b mode=1 the rotation matrix R is set equal to Rot*R.
    * @b mode=2 the rotation matrix R is set equal to Rot*Rinit.
    */
    void setRotation(Mat& Rot, int mode=0);

    /**
    * It sets the translation vector (if known) between the first and the second camera.
    * @param Tras the 3x1 translation matrix.
    * @param mode the following values are allowed:
    * @b mode=0 the translation vector T is set equal to Tras.
    * @b mode=1 the translation vector T is set equal to Tras+T.
    * @b mode=2 the translation vector T is set equal to Tras+Tinit.
    */
    void setTranslation(Mat& Tras, int mul=0);

    /**
    * It sets the intrinsic parameters.
    * @param K1 3x3 camera matrix of the first camera.
    * @param K2 3x3 camera matrix of the second camera.
    * @param Dist1 4x1 distortion coefficients vector of the first camera.
    * @param Dist2 4x1 distortion coefficients vector of the second camera.
    */
    void setIntrinsics(Mat& K1, Mat& K2, Mat& Dist1, Mat& Dist2);

    /**
    * The method rectifies the two images: it transform each image plane such that pairs
    * conjugate epipolar lines become collinear and parallel to one of the image axes (i.e. there is 0 disparity on the Y axis).
    */
    void rectifyImages();

    /**
    * The method returns the first rectified image.
    * @return The first rectified image.
    */
    Mat getLRectified();

    /**
    * The method returns the second rectified image.
    * @return The second rectified image.
    */
    Mat getRRectified();

    /**
    * The method returns the 2D projection of a set of 3D points in the cartesian space to the specified camera.
    * @param camera "left" or "right" camera
    * @param point3D the list of the 3D position in the reference frame H
    * @param H the transformation from the camera reference system to the H reference system
    * @return The 2D positions.
    */
    vector<Point2f> projectPoints3D(string camera, vector<Point3f> &points3D, Mat &H);

    /**
    * The method returns a 3-Channels float image with the world coordinates w.r.t H reference system.
    * @param H the transformation from the camera reference system to the H reference system
    * @return The 3-Channels float image with the world coordinates w.r.t H reference system.
    */
    Mat computeWorldImage(Mat &H);

    /**
    * It returns the 5x1 right distortion coefficients.
    * @return 5x1 right distortion coefficients.
    */
    Mat getDistCoeffRight();

    /**
    * It returns the 5x1 left distortion coefficients.
    * @return 5x1 left distortion coefficients.
    */
    Mat getDistCoeffLeft();

};
