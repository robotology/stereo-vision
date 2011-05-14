#include <iostream>
#include <string>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include "stereoCamera.h"
 
using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
  
class stereoCalibThread : public Thread
{
private:

   ImageOf<PixelRgb> *imageL;
   ImageOf<PixelRgb> *imageR;
   IplImage * imgL;   
   IplImage * imgR;  

   string inputLeftPortName;
   string inputRightPortName;
   string outNameRight;
   string outNameLeft;
   BufferedPort<ImageOf<PixelRgb> > imagePortInLeft;
   BufferedPort<ImageOf<PixelRgb> > imagePortInRight;
   BufferedPort<ImageOf<PixelRgb> > outPortRight;
   BufferedPort<ImageOf<PixelRgb> > outPortLeft;

   Port *commandPort;
   string dir;
   int saveVideo;
   int startCalibration;
   int boardWidth;
   int boardHeight;
   float squareSize;
   char pathL[256];
   char pathR[256];

	void writeSingleCamera(string dir);
public:


   stereoCalibThread(string imageInLeft, string imageInRight, string outputPortNameRight, string outputPortNameLeft, Port* commPort, const char *dir,int bwidth, int bheight, float squareSize);
   void setSave();
   void startCalib();
   bool threadInit();     
   void threadRelease();
   void run(); 
   void onStop();
};


