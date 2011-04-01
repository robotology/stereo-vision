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

\defgroup stereoCalib stereoCalib
 
Calibrate the stereo system (both intrinsics and extrinsics).

Copyright (C) 2011 RobotCub Consortium
 
Author: Sean Ryan Fanello
 
Date: first release on 26/03/2011

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
The module performs the stereo calibration used by the stereoDisp module.
A chessboard pattern is required. 
For convenience, a chessboard calibration image is provided in the 
$ICUB_ROOT/main/app/cameraCalibration/data directory.

\note Opencv 2.2 is required!
  
\section lib_sec Libraries 
YARP libraries and OpenCV 2.2

\section parameters_sec Parameters
--name \e stemName 
- The parameter \e stemName specifies the stem name of ports 
  created by the module.

--boardWidth \e numOfCornW 
- The parameter \e numOfCornW identifies the number of inner corners in the Width direction of the chess
   
--boardHeight \e numOfCornH 
- The parameter \e numOfCornH identifies the number of inner corners in the Height direction of the chess

--boardSize \e squareLength 
- The parameter \e squareLength identifies the length (in meters) of the squares in the chess (usually 0.03m)

\section portsc_sec Ports Created
- <i> /<stemName>/cam/left:i </i> accepts the incoming images from the left eye. 
- <i> /<stemName>/cam/right:i </i> accepts the incoming images from the right eye. 

- <i> /<stemName>/cam/left:o </i> outputs the left eye image synchronized with the right eye. 
- <i> /<stemName>/cam/right:o </i> outputs the right eye image synchronized with the left eye. 

- <i> /<stemName>/cmd </i> for terminal commands comunication. 
 Recognized remote commands:
    - [start]: Starts the calibration procedure, you have to show the chessboard image in different positions and orientations. 
    In the stdout is printed a message each time the pattern has been recognized by both the eyes. After 30 times the module will run the stereo calibration
    and it will produce the files intrisincs.yml and extrinsics.yml in ${ICUB_ROOT}/app/stereoVision/conf.
    These files are required by stereoDisparity for the computation of the depth map.
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Windows.

\author Sean Ryan Fanello
*/ 

#include "stereoCalibModule.h"


int main(int argc, char * argv[])
{
   /* initialize yarp network */ 

   Network yarp;

   /* create your module */

   stereoCalibModule stereoModule; 

   /* prepare and configure the resource finder */

   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultConfigFile("stereoCalib.ini"); //overridden by --from parameter
   rf.setDefaultContext("stereoVision/conf");   //overridden by --context parameter
   rf.configure("ICUB_ROOT", argc, argv);
 
   /* run the module: runModule() calls configure first and, if successful, it then runs */

   stereoModule.runModule(rf);
    return 1;
}
