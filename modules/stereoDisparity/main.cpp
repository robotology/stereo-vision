/* 
 * Copyright (C) 2011 RobotCub Consortium
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
\defgroup stereoDisparity stereoDisparity

@ingroup icub_stereoVision

Builds a depth map using the vision system.

Copyright (C) 2011 RobotCub Consortium
 
Author: Sean Ryan Fanello
 
Date: first release on 26/03/2011

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
The module exploits the stereo visual system to build a depth map
using the H. Hirschmuller Algorithm (CVPR 2006) implemented in Opencv 2.2. The computation of the depth map
is robust to camera movements exploiting the encoders information and updating the camera reference system. 
The module is also able to refine the estimation of the cameras movements using the Horn Relative Orientations Algorithm.
Before start make sure you have calibrated the stereo system. For the stereo calibration see the module \ref stereoCalib "stereoCalib" . The module provides two output ports: the first one
is the disparity map in grayscale values, the second port is the WorldImage, that is a 3-channels float image, in each pixel are stored the three
X Y Z coordinates with respect to Root reference frame. Non valid points are handled with the value (0,0,0). In addition a RPC port supports requests for
3D/2D points computation (see below).

\note Opencv 2.2 is required!
  
\section lib_sec Libraries 
YARP libraries and OpenCV 2.2

\section parameters_sec Parameters
--name \e stemName 
- The parameter \e stemName specifies the stem name of ports 
  created by the module.

--robot \e robotName
- The parameter \e robotName specifies the name of the robot.

--InputPortLeft  \e inputLeft
- The parameter \e inputLeft specifies the left image input port.

--InputPortRight  \e inputRight
- The parameter \e inputRight specifies the right image input port.

--BoxPort  \e inputBox
- The parameter \e inputBox specifies the port for the bounding box.

--OutPort   \e disparity:o 
- The parameter \e disparity:o specifies the output port for the disparity image.

--WorldOutPort   \e world:o 
- The parameter \e world:o specifies the output port for the world image.

--CommandPort   \e comm 
- The parameter \e comm specifies the command port for rpc protocol.
  

\section portsc_sec Ports Created
- <i> /<stemName>/<inputLeft> </i> accepts the incoming images from the left eye. 
- <i> /<stemName>/<inputRight> </i> accepts the incoming images from the right eye. 
- <i> /<stemName>/<box:i> </i> accepts the quadruple (u0,v0,width,height) of the region of interest (ROI). The output port <i> /<stemName>/<world:o> </i> (see below) will compute the three world coordinates only for points belonging to the ROI.

- <i> /<stemName>/<disparity:o> </i> outputs the disparity map in grayscale values.
- <i> /<stemName>/<world:o> </i> outputs the world image (3-channel float with X Y Z values). If there exists a connection with <i> /<stemName>/<box:i> </i> the output of the world port is an image with the same size of the box (width,height) specified in the bounding box (this saves computational time). Otherwise all the scene is returned.
- <i> /<stemName>/<comm> </i> for terminal commands comunication. 
    - [Point x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the LEFT eye. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).
    - [x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the the ROOT reference system. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).
    - [Left x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the LEFT eye. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).
    - [Right x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the RIGHT eye. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).
    - [Root x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the ROOT reference system. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).
    - [uL_1 vL_1 uR_1 vR_1 ... uL_n vL_n uR_n vR_n]: Given n quadruples uL_i vL_i uR_i vR_i, where uL_i vL_i are the pixel coordinates in the Left image and uR_i vR_i are the coordinates of the matched pixel in the Right image, the response is a set of 3D points (X1 Y1 Z1 ... Xn Yn Zn) wrt the ROOT reference system.
    - [cart2stereo X Y Z]: Given a world point X Y Z wrt to ROOT reference frame the response is the projection (uL vL uR vR) in the Left and Right images.
    - [disparity on/off]: It sets on/off the disparity computation (it saves computational time when the disparity is not needed).
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Linux (Ubuntu 9.04, Debian Squeeze) and Windows 7.

\author Sean Ryan Fanello
*/ 
#include <yarp/dev/Drivers.h>
#include "disparityModule.h"

YARP_DECLARE_DEVICES(icubmod)

int main(int argc, char * argv[])
{
   YARP_REGISTER_DEVICES(icubmod)
   Network yarp;
   stereoModule stereoModule; 

   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultConfigFile("icubEyes.ini"); 
   rf.setDefaultContext("cameraCalibration");   
   rf.configure("ICUB_ROOT", argc, argv);
 

   stereoModule.runModule(rf);

    return 0;
}
