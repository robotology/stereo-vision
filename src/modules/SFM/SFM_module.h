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
#include <iCub/iKin/iKinFwd.h>
#include <yarp/dev/GazeControl.h>
#include <iCub/stereoVision/stereoCamera.h>
#include <iCub/iKin/iKinFwd.h>



#include <iCub/stereoVision/utils.h>



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


    cv::Mat leftMat, rightMat, matMatches;


        /* pointer to the utilities class */
    Utilities                 *utils;


    yarp::os::Port rpc;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > leftImgPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > rightImgPort;
    BufferedPort<ImageOf<PixelRgbFloat> > worldPort;
    Port handlerPort;

    BufferedPort<ImageOf<PixelBgr> > outDisp;
    BufferedPort<ImageOf<PixelBgr> > outMatch;

    bool useBestDisp;
    int uniquenessRatio; 
    int speckleWindowSize;
    int speckleRange;
    int numberOfDisparities;
    int SADWindowSize;
    int minDisparity;
    int preFilterCap;
    int disp12MaxDiff;
    Semaphore* mutexDisp;
    
    PolyDriver* gazeCtrl;
    IGazeControl* igaze;
    Mat HL_root;
    Mat HR_root;    

    bool loadStereoParameters(yarp::os::ResourceFinder &rf, Mat &KL, Mat &KR, Mat &DistL, Mat &DistR, Mat &R, Mat &T);
    Mat buildRotTras(Mat & R, Mat & T);
    Matrix getCameraHGazeCtrl(int camera);
    void convert(Matrix& matrix, Mat& mat);
    void convert(Mat& mat, Matrix& matrix);    
    void fillWorld3D(ImageOf<PixelRgbFloat> &worldImg, int u0, int v0, int width, int height);

    void printMatrix(Mat &matrix);
    void updateViaKinematics(bool exp=false);
    bool init;

public:

    bool configure(ResourceFinder &rf);
    bool close();
    bool updateModule();
    double getPeriod();
    bool interruptModule();
    bool respond(const Bottle& command, Bottle& reply);

    void setDispParameters(bool _useBestDisp, int _uniquenessRatio, int _speckleWindowSize,int _speckleRange, int _numberOfDisparities, int _SADWindowSize, int _minDisparity, int _preFilterCap, int _disp12MaxDiff);
    Point3f get3DPoints(int u, int v,string drive="LEFT");
    Point3f get3DPointMatch(double u1, double v1, double u2, double v2, string drive="LEFT");
    Point2f projectPoint(string camera, double x, double y, double z);    
};
