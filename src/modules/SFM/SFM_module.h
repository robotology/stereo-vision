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
#include <iCub/ctrl/math.h>
#include <iCub/stereoVision/stereoCamera.h>

#ifdef USING_GPU
    #include "utils.h"
#endif


YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;


class SFM: public yarp::os::RFModule
{

    IplImage* left;
    IplImage* right;
    StereoCamera *stereo;
    IplImage* outputD;
    IplImage* output_match;

    cv::Mat leftMat, rightMat, matMatches;

    #ifdef USING_GPU
        /* pointer to the utilities class */
        Utilities                 *utils;
    #endif

    yarp::os::Port rpc;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > leftImgPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > rightImgPort;

    BufferedPort<ImageOf<PixelBgr> > outDisp;
    BufferedPort<ImageOf<PixelBgr> > outMatch;

    bool loadStereoParameters(yarp::os::ResourceFinder &rf, Mat &KL, Mat &KR, Mat &DistL, Mat &DistR, Mat &R, Mat &T);

    bool init;

public:

    bool configure(ResourceFinder &rf);
    bool close();
    bool updateModule();
    double getPeriod();
    bool interruptModule();
};
