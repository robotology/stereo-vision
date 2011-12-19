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
\defgroup testWorldImage testWorldImage

@ingroup icub_stereoVision

A test module for the image of the world.

Copyright (C) 2011 RobotCub Consortium
 
Author: Sean Ryan Fanello
 
Date: 19/12/2011

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
The module simply reads two pixel coordinates (u,v) and the 3-channel float image from stereoDisparity and prints the 3D position of the pixel (u,v) w.r.t ROOT reference system.
In $ICUB_ROOT$/contrib/src/stereoVision/app/scripts is placed a simple application manager testWorldImage.xml in order to run all the modules and to set the connections properly.

\section lib_sec Libraries 
YARP libraries and OpenCV 2.2

\section parameters_sec Parameters
--name \e name 
- The parameter \e stemName specifies the stem name of ports 
  created by the module.

--pixelPort  \e pixelPort
- The parameter \e pixelPort specifies the port for the pixel (u,v) which is wanted to know the 3D position. The coordinates are wrt the LEFT eye.

--WorldOutPort   \e world:i 
- The parameter \e world:i specifies the input port for the world image.


\section portsc_sec Ports Created
- <i> /<stemName>/<pixelPort> </i> accepts the incoming pixels wrt the LEFT eye. 
- <i> /<stemName>/<world:i> </i> the world image input (3-channel float with X Y Z values).

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

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;

class TestWorldImage: public yarp::os::RFModule
{

    IplImage* worldImg;

    yarp::os::Port rpc;
    yarp::os::BufferedPort<Bottle>  pixelPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgbFloat> > imageWorld;

public:

    bool configure(ResourceFinder &rf);
    bool close();
    bool updateModule();
    double getPeriod();
    bool interruptModule();
};
