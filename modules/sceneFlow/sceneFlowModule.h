/* 
 * Copyright (C) 2011 RobotCub Consortium
 * Author: Sean Ryan Fanello, Ilaria Gori
 * email:   sean.fanello@iit.it, ilaria.gori@iit.it
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
\defgroup sceneFlowModule sceneFlowModule

A test module for the image of the world. 

Copyright (C) 2011 RobotCub Consortium
 
Author: Sean Ryan Fanello, Ilaria Gori
 
Date: 19/12/2011

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
This is an example module for the SceneFlow computation.

\section lib_sec Libraries 
YARP libraries and OpenCV 2.4 (or gretear)

\section parameters_sec Parameters
--configDisparity \e file.ini 
- The parameter \e file.ini specifies the cameras config file (e.g icubEyes.ini).

--denseFlow \e val 
- The parameter \e val specifies the flow type: dense if val=1; sparse if val=0.

--leftCamera \e left
- The parameter \e left specifies the left camera port (e.g. /icub/camcalib/left/out).

--rightCamera \e right
- The parameter \e right specifies the right camera port (e.g. /icub/camcalib/right/out).

\section portsc_sec Ports Created


\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Linux (Ubuntu 9.04, Debian Squeeze) and Windows 7.

\author Sean Ryan Fanello, Ilaria Gori
*/ 

#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <deque>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <cv.h>
#include <highgui.h>
#include <iCub/ctrl/math.h>
#include <iCub/stereoVision/sceneFlow.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;

class sceneFlowModule: public yarp::os::RFModule
{

private:
    SceneFlow* sceneFlow;

public:

    bool configure(ResourceFinder &rf);
    bool close();
    bool updateModule();
    double getPeriod();
    bool interruptModule();
};
