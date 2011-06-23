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
*
@ingroup icub_module

\defgroup icub_stereoDisparity stereoDisparity
 
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
Before start make sure you have calibrated the stereo system (in ${ICUB_ROOT}/app/stereoVision/conf you should have the files
intrinsics.yml and extrinsics.yml). For the stereo calibration see the module stereoCalib.

\note Opencv 2.2 is required!
\note When you start the module make sure that vergence, pan and tilt eyes angles are set to 0.
  
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

--DriveEye   \e drive
- The parameter \e drive specifies drive eye. Each 3D point coordinates will be wrt the drive eye.

--OutPort   \e disparityPort 
- The parameter \e disparityPort specifies the output port for the disparity image.

--CommandPort   \e comm 
- The parameter \e comm specifies the command port for rpc protocol.
  

\section portsc_sec Ports Created
- <i> /<stemName>/<inputLeft> </i> accepts the incoming images from the left eye. 
- <i> /<stemName>/<inputRight> </i> accepts the incoming images from the right eye. 


- <i> /<stemName>/<disparityPort> </i> outputs the disparity map
- <i> /<stemName>/<comm> </i> for terminal commands comunication. 
    - [Point x y]: Given the pixel coordinate x,y in the Left image (uncalibrated! i.e. icub/cam/left) the response is the 3D Point: x,y,z computed using the depth map wrt the LEFT eye.
    - [x y]: Given the pixel coordinate x,y in the Left image (uncalibrated! i.e. icub/cam/left) the response is the 3D Point: x,y,z computed using the depth map wrt the LEFT eye.
    - [Left x y]: Given the pixel coordinate x,y in the Left image (uncalibrated! i.e. icub/cam/left)) the response is the 3D Point: x,y,z computed using the depth map wrt the LEFT eye.
    - [Right x y]: Given the pixel coordinate x,y in the Left image (uncalibrated! i.e. icub/cam/left) the response is the 3D Point: x,y,z computed using the depth map wrt the RIGHT eye.
    - [Root x y]: Given the pixel coordinate x,y in the Left image (uncalibrated! i.e. icub/cam/left) the response is the 3D Point: x,y,z computed using the depth map wrt the ROOT reference system.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Linux (Ubuntu 9.04, Debian Squeeze) and Windows 7.

\author Sean Ryan Fanello
**/ 

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
   rf.setDefaultConfigFile("stereoDisparity.ini"); 
   rf.setDefaultContext("stereoVision/conf");   
   rf.configure("ICUB_ROOT", argc, argv);
 

   stereoModule.runModule(rf);

    return 0;
}
