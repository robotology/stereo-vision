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

    StereoCamera *stereo;
    Semaphore* mutexDisp;

    string inputLeftPortName;
    string inputRightPortName;
    string outName;
    string worldPortName;


    BufferedPort<ImageOf<PixelRgb> > imagePortInLeft;
    BufferedPort<ImageOf<PixelRgb> > imagePortInRight;
    BufferedPort<ImageOf<PixelBgr> > outPort;
    BufferedPort<ImageOf<PixelRgbFloat> > worldPort;

    Port *commandPort;
    string dir;

    PolyDriver* gazeCtrl;
    IGazeControl* igaze;
    Matrix H;
    Mat HL_root;
    Mat HR_root;
    double angle;
    bool computeDisparity;

    Matrix getCameraH(int camera);
    void printMatrixYarp(Matrix &A);
    void convert(Matrix& matrix, Mat& mat);
    void convert(Mat& mat, Matrix& matrix);
    Mat buildRotTras(Mat & R, Mat & T);
    void printMatrix(Mat &matrix);
    void fillWorld3D(ImageOf<PixelRgbFloat> &worldImg);
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
