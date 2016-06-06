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
Algorithm (CVPR 2006), implemented since Opencv 2.2, or
<a href="http://www.cvlibs.net/software/libelas/">LIBELAS
(Library for Efficient Large-scale Stereo Matching)</a>.
The Kinematics of the iCub is used to guess the current camera positions, 
then visual features are used to refine this model.
Before starting, make sure you have calibrated the intrinsics 
parameters. For the stereo calibration see the module <a 
href="http://wiki.icub.org/iCub/main/dox/html/group__icub__stereoCalib.html">stereoCalib</a>. 
This module provides five output ports: the first one is the
disparity map in grayscale values, the second port is the 
WorldImage, that is a 3-channels float image, where in each 
pixel are stored the three X Y Z coordinates with respect to 
robot root reference frame. The third port outputs the current 
keypoints match. Non valid points are handled with the special 
value (0,0,0). The last two ports output the rectified images used to compute
the horizontal disparity map. In addition, a rpc port supports requests for
3D/2D points computation (see below). 

\section lib_sec Libraries 
YARP libraries and OpenCV 2.4 (at least). \n
For better performance, we suggest you to run the module on a 
machine equipped with GPU functionality along with the 
<a href="http://cs.unc.edu/~ccwu/siftgpu">SiftGPU</a> library 
installed. This module now uses <a href="http://www.cvlibs.net/software/libelas/">LIBELAS </a>
by default to compute the horizontal disparity map from the rectified left and right images.
The source code of LIBELAS is compiled with the stereoVisionLib (with no particular dependences).
The OpenMP accelerated version of the LIBELAS is used under UNIX systems, if OpenMP is available.

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
 
--outLeftRectImgPort \e /rect_left:o
- Specifies the left rectified image output port.

--outRightRectImgPort \e /rect_right:o
- Specifies the right rectified image output port.

--outDispPort \e /disp:o 
- The parameter \e /disparity:o specifies the output port for the disparity image.

--outMatchPort \e /match:o 
- The parameter \e /match:o specifies the output port for the match image.

--outWorldPort \e /world:o 
- The parameter \e /world:o  specifies the output port for the world image.

--CommandPort \e comm 
- The parameter \e comm specifies the command port for rpc protocol. 

--skipBLF
- Disable Bilateral filter.

--use_sgbm
- By default LIBELAS is used to compute the disparity. However, if you prefer to continue using the
OpenCV's SGBM algorithm, you just need to pass the parameter \e use_sgbm.

If you use LIBELAS, there is the possibility of setting the following parameters:

--disp_scaling_factor \e 1.0
- This parameter provides the option of resizing the left and right images
before computing the disparity map (and finally resize again
the resulting map to the original size). It is a multiplicative factor.
For example, if a low-resolution (also less accurate!) disparity map
is sufficient, but needed at high rate, you can set this parameter to 1/N
so that the images width and height are divided by a factor N,
LIBELAS computes the disparity of the downsampled images,
and finally the disparity map's size is rescaled by N before returning.

--elas_setting \e ROBOTICS
- The parameter \e ROBOTICS or \e MIDDLEBURY allow to choose between the two settings
of parameters defined in <a href="https://github.com/robotology/stereo-vision/tree/master/lib/elas/include/elas.h">elas.h</a>.
This module gives the possibility of modifying (eventually tuning to individual needs)
those parameters which differ between the two settings, plus a couple of others
(\e elas_subsampling and \e elas_add_corners). The remaining parameters are supposed to be
fixed to the values proposed by the authors of LIBELAS.

Here we list those LIBELAS parameters that can be passed to this module;
see <a href="https://github.com/robotology/stereo-vision/tree/master/lib/elas/include/elas.h">elas.h</a>
for the complete list of parameters, their definitions and default values.

--elas_subsampling
- Pass the parameter \e elas_subsampling if you want to set the \e subsampling parameter to \e true
in <a href="https://github.com/robotology/stereo-vision/tree/master/lib/elas/include/elas.h">elas.h</a>,
to speedup the computation (at the expenses of accuracy).

--elas_add_corners
- Pass the parameter \e elas_add_corners if you want to set \e add_corners parameter to \e true
in <a href="https://github.com/robotology/stereo-vision/tree/master/lib/elas/include/elas.h">elas.h</a>,
to consider also the image corners in the computation of the disparity map.

--elas_ipol_gap_width \e 40
- This is the only parameter for which we set a default value different from the ones provided
in <a href="https://github.com/robotology/stereo-vision/tree/master/lib/elas/include/elas.h">elas.h</a>, where
the \e ipol_gap_width parameter is set to \e 3 in \e ROBOTICS and \e 5000 in \e MIDDLEBURY. It is the radius of interpolation (in pixel)
of the disparity values found in the keypoints. Small values result in sparser disparity maps (with more
black holes); high values result in denser maps, with the black regions filled with interpolated values.

Also the following LIBELAS parameters can be modified by the user,
however we stick with the values provided by the authors.

--elas_support_threshold \e 0.85
- This is the \e support_threshold parameter in <a href="https://github.com/robotology/stereo-vision/tree/master/lib/elas/include/elas.h">elas.h</a>,
set to 0.95 in \e MIDDLEBURY and 0.85 in \e ROBOTICS.

--elas_gamma \e 3
- This is the \e gamma parameter in <a href="https://github.com/robotology/stereo-vision/tree/master/lib/elas/include/elas.h">elas.h</a>,
set to 5 in \e MIDDLEBURY and 3 in \e ROBOTICS.

--elas_sradius \e 2
- This is the \e sradius parameter in <a href="https://github.com/robotology/stereo-vision/tree/master/lib/elas/include/elas.h">elas.h</a>,
set to 3 in \e MIDDLEBURY and 2 in \e ROBOTICS.

--elas_match_texture \e true
- This is the \e match_texture parameter in <a href="https://github.com/robotology/stereo-vision/tree/master/lib/elas/include/elas.h">elas.h</a>,
set to \e false in \e MIDDLEBURY and \e true in \e ROBOTICS.

--elas_filter_median \e false
- This is the \e filter_median parameter in <a href="https://github.com/robotology/stereo-vision/tree/master/lib/elas/include/elas.h">elas.h</a>,
set to \e true in \e MIDDLEBURY and \e false in \e ROBOTICS.

--elas_filter_adaptive_mean \e true
- This is the \e filter_adaptive_mean parameter in <a href="https://github.com/robotology/stereo-vision/tree/master/lib/elas/include/elas.h">elas.h</a>,
set to \e false in \e MIDDLEBURY and \e true in \e ROBOTICS. 
 
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
    - [setMinDisp minDisparity]: It sets the minimum disparity (in pixel).
    - [Point x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the LEFT eye. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).
    - [x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z ur vr computed using the depth map wrt the the ROOT reference system; (ur vr) is the corresponding pixel in the Right image. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).
    - [Left x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the LEFT eye. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).
    - [Right x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the RIGHT eye. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).
    - [Root x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the ROOT reference system. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).
    - [Rect tlx tly w h step]: Given the pixels in the rectangle defined by {(tlx,tly) (tlx+w,tly+h)} (parsed by columns), the response contains the corresponding 3D points in the ROOT frame. The optional parameter step defines the sampling quantum; by default step=1.
    - [Points u_1 v_1 ... u_n v_n]: Given a list of n pixels, the response contains the corresponding 3D points in the ROOT frame.
    - [Flood3D x y dist]: Perform 3D flood-fill on the seed point (x,y), returning the following info: [u_1 v_1 x_1 y_1 z_1 ...]. The optional parameter dist expressed in meters regulates the fill (by default = 0.004).
    - [uL_1 vL_1 uR_1 vR_1 ... uL_n vL_n uR_n vR_n]: Given n quadruples uL_i vL_i uR_i vR_i, where uL_i vL_i are the pixel coordinates in the Left image and uR_i vR_i are the coordinates of the matched pixel in the Right image, the response is a set of 3D points (X1 Y1 Z1 ... Xn Yn Zn) wrt the ROOT reference system.
    - [cart2stereo X Y Z]: Given a world point X Y Z wrt to ROOT reference frame the response is the projection (uL vL uR vR) in the Left and Right images.
    - [doBLF flag]: activate Bilateral filter for flag = true, and skip it for flag = false (default by config).
    - [bilatfilt sigmaColor sigmaSpace]: Set the parameters for the bilateral filer (default sigmaColor = 10.0, sigmaSpace = 10.0 .

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
#include <cstdio>
#include <iostream>
#include <fstream>
#include <set>
#include <deque>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/stereoVision/stereoCamera.h>

#include "fastBilateral.hpp"

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
    IplImage*     output_match;
    Mat           outputDm;

    Mat leftMat, rightMat;

#ifdef USING_GPU
    /* pointer to the utilities class */
    Utilities *utils;
#endif

    yarp::os::Port rpc;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > leftImgPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > rightImgPort;
    BufferedPort<ImageOf<PixelRgbFloat> > worldPort;
    Port handlerPort;

    BufferedPort<ImageOf<PixelMono> > outDisp;
    BufferedPort<ImageOf<PixelBgr> >  outMatch;

    BufferedPort<ImageOf<PixelRgb> >  outLeftRectImgPort;
    BufferedPort<ImageOf<PixelRgb> >  outRightRectImgPort;
    
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
    double sigmaColorBLF;
    double sigmaSpaceBLF;
    bool doBLF;
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
    Mat buildRotTras(const Mat& R, const Mat& T);
    Matrix getCameraHGazeCtrl(int camera);
    void convert(Matrix& matrix, Mat& mat);
    void convert(Mat& mat, Matrix& matrix);    
    void fillWorld3D(ImageOf<PixelRgbFloat> &worldImg);
    void floodFill(const Point &seed,const Point3f &p0, const double dist, set<int> &visited, Bottle &res);
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


