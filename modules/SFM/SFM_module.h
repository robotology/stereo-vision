/* 
 * Copyright (C) 2013 RobotCub Consortium
 * Author: Sean Ryan Fanello
 * email:   sean.fanello@iit.it
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
\defgroup SFM SFM

@ingroup icub_stereoVision

Structure From Motion (SFM) module for estimation of estrinsics parameter and computation of depth map.

Copyright (C) 2013 RobotCub Consortium
 
Author: Sean Ryan Fanello
 
Date: first release around 24/07/2013

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
The module uses a complete Structure From Motion (SFM) pipeline for the computation of the extrinsics parameters between
two different views. These parameters are then used to rectify the images and to compute a depth map using the H. Hirschmuller Algorithm (CVPR 2006) implemented since Opencv 2.2. The Kinematics
of the iCub is used to guess the current camera positions, then visual features are used to refine this model.
Before start make sure you have calibrated the intrinsics parameters. For the stereo calibration see the module \ref stereoCalib "stereoCalib" . The module provides three output ports: the first one
is the disparity map in grayscale values, the second port is the WorldImage, that is a 3-channels float image, in each pixel are stored the three
X Y Z coordinates with respect to Root reference frame. The third port outputs the current keypoints matched. Non valid points are handled with the value (0,0,0). In addition a RPC port supports requests for
3D/2D points computation (see below).

\note Tested on OpenCV 2.2, 2.3, 2.4
  
\section lib_sec Libraries 
YARP libraries and OpenCV 2.2

\section parameters_sec Parameters
--name \e SFM 
- The parameter \e stemName specifies the stem name of ports 
  created by the module.

--robot \e robotName
- The parameter \e robotName specifies the name of the robot.

--leftPort  \e /left:i
- The parameter \e inputLeft specifies the left image input port.

--rightPort  \e /right:i
- The parameter \e inputRight specifies the right image input port.


--outDispPort   \e /disp:o 
- The parameter \e /disparity:o specifies the output port for the disparity image.

--outMatchPort   \e /match:o 
- The parameter \e /match:o specifies the output port for the match image.

--outWorldPort   \e /world:o 
- The parameter \e /world:o  specifies the output port for the world image.

--CommandPort   \e comm 
- The parameter \e comm specifies the command port for rpc protocol.
  

\section portsc_sec Ports Created
- <i> /SFM/left:i </i> accepts the incoming images from the left eye. 
- <i> /SFM/right:i </i> accepts the incoming images from the right eye. 

- <i> /SFM/disp:o </i> outputs the disparity map in grayscale values.
- <i> /SFM/world:o</i> outputs the world image (3-channel float with X Y Z values). If there exists a connection with <i> /<stemName>/<box:i> </i> the output of the world port is an image with the same size of the box (width,height) specified in the bounding box (this saves computational time). Otherwise all the scene is returned.
- <i> /SFM/match:o</i> outputs the match image.
- <i> /SFM/rpc </i> for terminal commands comunication. 
    - [stopSFM]: It stops the real-time update of the camera positions, ensuring higher framerate in the depth map computation. 
    - [startSFM]: It starts the real-time update of the camera positions.
    - [recalibrate]: It recomputes the camera positions once.
    - [saveCurrentCalib]: It saves the current camera positions and uses it when the module starts.
    - [setNumDisp NumOfDisparities]: It sets the expected number of disparity (in pixel). Values must be divisible by 32. Good values are 64 for 320x240 images and 128 for 640x480 images.
    - [Point x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the LEFT eye. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).
    - [x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z d computed using the depth map wrt the the ROOT reference system, d is the disparity (in pixel) between the left pixel x and the right pixel x+d. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).
    - [Left x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the LEFT eye. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).
    - [Right x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the RIGHT eye. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).
    - [Root x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the ROOT reference system. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).
    - [uL_1 vL_1 uR_1 vR_1 ... uL_n vL_n uR_n vR_n]: Given n quadruples uL_i vL_i uR_i vR_i, where uL_i vL_i are the pixel coordinates in the Left image and uR_i vR_i are the coordinates of the matched pixel in the Right image, the response is a set of 3D points (X1 Y1 Z1 ... Xn Yn Zn) wrt the ROOT reference system.
    - [cart2stereo X Y Z]: Given a world point X Y Z wrt to ROOT reference frame the response is the projection (uL vL uR vR) in the Left and Right images.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Linux (Ubuntu 9.04, Debian Squeeze) and Windows 7.

\author Sean Ryan Fanello
*/ 

#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <deque>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/stereoVision/stereoCamera.h>

#ifdef USING_GPU
    #include <iCub/stereoVision/utils.h>
#endif


YARP_DECLARE_DEVICES(icubmod)

#define LEFT    0
#define RIGHT   1

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iKin;

class SFM: public yarp::os::RFModule
{

    IplImage* left;
    IplImage* right;
    StereoCamera *stereo;
    IplImage* outputD;
    IplImage* output_match;


    cv::Mat leftMat, rightMat;

#ifdef USING_GPU
    /* pointer to the utilities class */
    Utilities                 *utils;
#endif

    yarp::os::Port rpc;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > leftImgPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > rightImgPort;
    BufferedPort<ImageOf<PixelRgbFloat> > worldPort;
    Port handlerPort;

    BufferedPort<ImageOf<PixelBgr> > outDisp;
    BufferedPort<ImageOf<PixelBgr> > outMatch;

    int numberOfTrials;
    string camCalibFile;
    bool useBestDisp;
    int uniquenessRatio; 
    int speckleWindowSize;
    int speckleRange;
    int numberOfDisparities;
    int SADWindowSize;
    int minDisparity;
    int preFilterCap;
    int disp12MaxDiff;
    bool doSFM;
    bool doSFMOnce;
    bool calibUpdated;
    Mutex mutexRecalibration;
    Event calibEndEvent;
    Semaphore* mutexDisp;
    
    PolyDriver* gazeCtrl;
    IGazeControl* igaze;
    Mat HL_root;
    Mat HR_root;    

    bool loadIntrinsics(yarp::os::ResourceFinder &rf, Mat &KL, Mat &KR, Mat &DistL, Mat &DistR);
    Mat buildRotTras(Mat & R, Mat & T);
    Matrix getCameraHGazeCtrl(int camera);
    void convert(Matrix& matrix, Mat& mat);
    void convert(Mat& mat, Matrix& matrix);    
    void fillWorld3D(ImageOf<PixelRgbFloat> &worldImg, int u0, int v0, int width, int height);
    bool loadExtrinsics(yarp::os::ResourceFinder &rf,Mat &R, Mat &T);
    void printMatrix(Mat &matrix);
    void updateViaKinematics(bool exp=false);
    bool init;
    bool updateExtrinsics(Mat &Rot, Mat &Tr, const string& groupname);
    
public:

    bool configure(ResourceFinder &rf);
    bool close();
    bool updateModule();
    double getPeriod();
    bool interruptModule();
    bool respond(const Bottle& command, Bottle& reply);

    void setDispParameters(bool _useBestDisp, int _uniquenessRatio, int _speckleWindowSize,int _speckleRange, int _numberOfDisparities, int _SADWindowSize, int _minDisparity, int _preFilterCap, int _disp12MaxDiff);
    Point3f get3DPoints(int u, int v,string drive="LEFT");
	Point3f get3DPointsAndDisp(int u, int v, int &uR, int &vR, string drive);
  
    Point3f get3DPointMatch(double u1, double v1, double u2, double v2, string drive="LEFT");
    Point2f projectPoint(string camera, double x, double y, double z);    
};
