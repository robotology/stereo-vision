#include <iostream>
#include <string>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/dev/PolyDriver.h>
#include <iCub/iKin/iKinFwd.h>
#include <yarp/dev/GazeControl.h>
#include <iCub/ctrl/math.h>
#include <iCub/stereoVision/stereoCamera.h>
#include <iCub/iKin/iKinFwd.h>


#define LEFT    0
#define RIGHT   1

using namespace std; 
using namespace yarp::os; 
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;

class disparityThread : public Thread
{
private:

    ImageOf<PixelRgb> *imageL;
    ImageOf<PixelRgb> *imageR;
    IplImage* imgL;
    IplImage* imgR;
    IplImage disp;
    IplImage* output;
    IplImage* outputWorld;

    double vergence_init;
    double version_init;

    StereoCamera *stereo;
    Semaphore* mutexDisp;

    iCubEye *LeyeKin;
    iCubEye *ReyeKin;
    yarp::dev::PolyDriver polyHead;
    yarp::dev::IEncoders *posHead;
    yarp::dev::IControlLimits *HctrlLim;

    yarp::dev::PolyDriver polyTorso;
    yarp::dev::IEncoders *posTorso;
    yarp::dev::IControlLimits *TctrlLim;

    string inputLeftPortName;
    string inputRightPortName;
    string outName;
    string worldPortName;
    string boxPortName;
    string robotName;

    BufferedPort<ImageOf<PixelRgb> > imagePortInLeft;
    BufferedPort<ImageOf<PixelRgb> > imagePortInRight;
    BufferedPort<ImageOf<PixelBgr> > outPort;
    BufferedPort<ImageOf<PixelRgbFloat> > worldPort;
    BufferedPort<Bottle> boxPort;

    Port *commandPort;
    string dir;

    PolyDriver* gazeCtrl;
    IGazeControl* igaze;
    Matrix H;
    Mat HL_root;
    Mat HR_root;
    double angle;
    bool computeDisparity;
    bool useCalibrated;

    Matrix getCameraHGazeCtrl(int camera);
    Matrix getCameraH(yarp::sig::Vector head_angles,yarp::sig::Vector torso_angles, iCubEye *eyeKin, int camera);
    void printMatrixYarp(Matrix &A);
    void convert(Matrix& matrix, Mat& mat);
    void convert(Mat& mat, Matrix& matrix);
    Mat buildRotTras(Mat & R, Mat & T);
    void printMatrix(Mat &matrix);
    void fillWorld3D(ImageOf<PixelRgbFloat> &worldImg, int u0, int v0, int width, int height);
    bool loadStereoParameters(yarp::os::ResourceFinder &rf, Mat &KL, Mat &KR, Mat &DistL, Mat &DistR, Mat &R, Mat &T);
public:

    disparityThread(yarp::os::ResourceFinder &rf,Port* commPort);

    bool threadInit();
    void threadRelease();
    void run(); 
    void onStop();
    void compute(bool compute);
    bool isComputing();
    Point3f get3DPoints(int u, int v,string drive="LEFT");
    Point3f get3DPointMatch(double u1, double v1, double u2, double v2, string drive="LEFT");
    Point2f projectPoint(string camera, double x, double y, double z);
};
