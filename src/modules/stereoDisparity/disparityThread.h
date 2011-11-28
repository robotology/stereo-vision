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
#include <iCub/stereoVision/StereoCamera.h>

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
   IplImage * imgL;   
   IplImage * imgR;
   IplImage disp;
   IplImage * output;

   StereoCamera *stereo;
   Semaphore* mutexDisp;

   string inputLeftPortName;
   string inputRightPortName;
   string outName;


   BufferedPort<ImageOf<PixelRgb> > imagePortInLeft;
   BufferedPort<ImageOf<PixelRgb> > imagePortInRight;
   BufferedPort<ImageOf<PixelBgr> > outPort;

   Port *commandPort;
   string dir;

   PolyDriver* gazeCtrl;
   IGazeControl* igaze;
   Matrix H;
   Mat HL_root;
   double angle;

   
   void getH();
   Matrix getCameraH(int camera);
   void printMatrixYarp(Matrix &A);
   void convert(Matrix& matrix, Mat& mat);
   void convert(Mat& mat, Matrix& matrix);
   Mat buildRotTras(Mat & R, Mat & T);
   void printMatrix(Mat &matrix);
public:

   disparityThread(string inputLeftPortName, string inputRightPortName, string outName, 
                                 string calibPath,Port* commPort);

   bool threadInit();     
   void threadRelease();
   void run(); 
   void onStop();
   Point3f get3DPoints(int u, int v,string drive="LEFT");
};
