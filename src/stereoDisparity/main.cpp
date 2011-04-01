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
@ingroup icub_module

\defgroup stereoDisparity stereoDisparity
 
Builds a depth map using the vision system.

Copyright (C) 2011 RobotCub Consortium
 
Author: Sean Ryan Fanello
 
Date: first release on 26/03/2011

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
The module exploits the stereo visual system to build a depth map
using the H. Hirschmuller Algorithm (CVPR 2006) implemented in Opencv 2.2. The computation of the depth map
is robust to camera movements exploiting the encoders information and updating the camera reference system. 
Before start make sure you have calibrated the stereo system (in ${ICUB_ROOT}/app/stereoVision/conf you should have the files
intrinsics.yml and extrinsics.yml). For the stereo calibration see the module stereoCalib.

\note Opencv 2.2 is required!
  
\section lib_sec Libraries 
YARP libraries and OpenCV 2.2

\section parameters_sec Parameters
--name \e stemName 
- The parameter \e stemName specifies the stem name of ports 
  created by the module.
  

\section portsc_sec Ports Created
- <i> /<stemName>/cam/left:i </i> accepts the incoming images from the left eye. 
- <i> /<stemName>/cam/right:i </i> accepts the incoming images from the right eye. 

- <i> /<stemName>/disparity:o </i> outputs the disparity map

- <i> /<stemName>/cmd </i> for terminal commands comunication (not used yet). 
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Linux (Ubuntu 9.04) and Windows.

\author Sean Ryan Fanello
*/ 

#include <yarp/dev/Drivers.h>
#include "disparityModule.h"

YARP_DECLARE_DEVICES(icubmod)

int main(int argc, char * argv[])
{

	YARP_REGISTER_DEVICES(icubmod)
   /* initialize yarp network */ 

   Network yarp;

   // create your module 

   stereoModule stereoModule; 

   // prepare and configure the resource finder 

   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultConfigFile("stereoDisparity.ini"); //overridden by --from parameter
   rf.setDefaultContext("stereoVision/conf");   //overridden by --context parameter
   rf.configure("ICUB_ROOT", argc, argv);
 
   // run the module: runModule() calls configure first and, if successful, it then runs 

   stereoModule.runModule(rf);

    return 0;
}
