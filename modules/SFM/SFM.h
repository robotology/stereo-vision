/* 
 * Copyright (C) 2015 RobotCub Consortium
 * Author: Sean Ryan Fanello, Giulia Pasquale
 * email:   sean.fanello@iit.it giulia.pasquale@iit.it
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

Structure From Motion (SFM) module for estimation of estrinsics
parameter and computation of depth map. 

Copyright (C) 2015 RobotCub Consortium
 
Author: Sean Ryan Fanello, Giulia Pasquale
 
Date: first release around 24/07/2013

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
The module uses a complete Structure From Motion (SFM) pipeline 
for the computation of the extrinsics parameters between two 
different views. These parameters are then used to rectify the 
images and to compute a depth map using either the H. Hirschmuller 
Algorithm (CVPR 2006), implemented since Opencv 2.2, or the ELAS library. 
The Kinematics of the iCub is used to guess the current camera positions, 
then visual features are used to refine this model.
Before starting, make sure you have calibrated the intrinsics 
parameters. For the stereo calibration see the module <a 
href="http://wiki.icub.org/iCub/main/dox/html/group__icub__stereoCalib.html">stereoCalib</a>. 
The module provides five output ports: the first one is the 
disparity map in grayscale values, the second port is the 
WorldImage, that is a 3-channels float image, where in each 
pixel are stored the three X Y Z coordinates with respect to 
robot root reference frame. The third port outputs the current 
keypoints match. Non valid points are handled with the special 
value (0,0,0). 
The last two ports output the rectified images used to compute 
the horizontal disparity map.
In addition, a rpc port supports requests for 
3D/2D points computation (see below). 

\section lib_sec Libraries 
YARP libraries and OpenCV 2.4 (at least). \n
For better performance, we suggest you to run the module on a 
machine equipped with GPU functionality along with the 
<a href="http://cs.unc.edu/~ccwu/siftgpu">SiftGPU</a> library 
installed.

\section parameters_sec Parameters
--name \e SFM 
- The parameter \e stemName specifies the stem name of ports 
  created by the module.

--robot \e robotName
- The parameter \e robotName specifies the name of the robot.

--leftPort \e /left:i
- The parameter \e inputLeft specifies the left image input port.

--rightPort \e /right:i
- The parameter \e inputRight specifies the right image input port.
 
--outDispPort \e /disp:o 
- The parameter \e /disparity:o specifies the output port for the disparity image.

--outMatchPort \e /match:o 
- The parameter \e /match:o specifies the output port for the match image.

--outWorldPort \e /world:o 
- The parameter \e /world:o  specifies the output port for the world image.

--CommandPort \e comm 
- The parameter \e comm specifies the command port for rpc protocol. 

\section portsc_sec Ports Created
- <i> /SFM/left:i </i> accepts the incoming images from the left eye. 
- <i> /SFM/right:i </i> accepts the incoming images from the right eye. 

- <i> /SFM/disp:o </i> outputs the disparity map in grayscale values.
- <i> /SFM/world:o</i> outputs the world image (3-channel float with X Y Z values). 
- <i> /SFM/match:o</i> outputs the match image.

- <i> /SFM/rect_left:o</i> outputs the rectified left image.
- <i> /SFM/rect_right:o</i> outputs the rectified right image.

- <i> /SFM/rpc </i> for terminal commands communication. 
    - [calibrate]: It recomputes the camera positions once.
    - [save]: It saves the current camera positions and uses it when the module starts.
    - [getH]: It returns the calibrated stereo matrix.
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
Linux (Ubuntu 9.04, Debian Squeeze) and Windows 7. Tested 
against OpenCV versions: 2.4. 

\author Sean Ryan Fanello, Giulia Pasquale
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
    IplImage*     left;
    IplImage*     right;
    StereoCamera* stereo;
    IplImage*     outputD;
    IplImage*     output_match;

    Mat leftMat, rightMat;

    bool use_sgbm;

#ifdef USING_GPU
    /* pointer to the utilities class */
    Utilities *utils;
#endif

    yarp::os::Port rpc;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > leftImgPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > rightImgPort;
    BufferedPort<ImageOf<PixelRgbFloat> > worldPort;
    Port handlerPort;

    BufferedPort<ImageOf<PixelBgr> > outDisp;
    BufferedPort<ImageOf<PixelBgr> > outMatch;

    BufferedPort<ImageOf<PixelBgr> > outLeftRectImgPort;
    BufferedPort<ImageOf<PixelBgr> > outRightRectImgPort;
    
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
    bool calibUpdated;
    yarp::os::Mutex mutexRecalibration;
    Event calibEndEvent;
    yarp::os::Mutex mutexDisp;
    
    PolyDriver headCtrl,gazeCtrl;
    IEncoders* iencs;
    IGazeControl* igaze;
    yarp::sig::Vector eyes0,eyes;
    int nHeadAxes;
    Mat HL_root;
    Mat HR_root;
    Mat R0,T0;

    bool loadIntrinsics(yarp::os::ResourceFinder &rf, Mat &KL, Mat &KR, Mat &DistL, Mat &DistR);
    Mat buildRotTras(Mat& R, Mat& T);
    Matrix getCameraHGazeCtrl(int camera);
    void convert(Matrix& matrix, Mat& mat);
    void convert(Mat& mat, Matrix& matrix);    
    void fillWorld3D(ImageOf<PixelRgbFloat> &worldImg, int u0, int v0, int width, int height);
    bool loadExtrinsics(yarp::os::ResourceFinder& rf, Mat& Ro, Mat& To, yarp::sig::Vector& eyes);
    bool updateExtrinsics(Mat& Rot, Mat& Tr, yarp::sig::Vector& eyes, const string& groupname);
    void updateViaGazeCtrl(const bool update);
    void updateViaKinematics(const yarp::sig::Vector& deyes);
    bool init;
    
public:

    bool configure(ResourceFinder &rf);
    bool close();
    bool updateModule();
    double getPeriod();
    bool interruptModule();
    bool respond(const Bottle& command, Bottle& reply);

    void setDispParameters(bool _useBestDisp, int _uniquenessRatio, int _speckleWindowSize,
                           int _speckleRange, int _numberOfDisparities, int _SADWindowSize,
                           int _minDisparity, int _preFilterCap, int _disp12MaxDiff);
    Point3f get3DPoints(int u, int v, const string &drive="LEFT");
    Point3f get3DPointsAndDisp(int u, int v, int &uR, int &vR, const string &drive);
  
    Point3f get3DPointMatch(double u1, double v1, double u2, double v2, const string &drive="LEFT");
    Point2f projectPoint(const string &camera, double x, double y, double z);    
};


