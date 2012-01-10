#include <iostream>
#include <string>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <iCub/stereoVision/stereoCamera.h>
#include <yarp/os/Stamp.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/math.h>

using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin; 
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
    string camCalibFile;

    BufferedPort<ImageOf<PixelRgb> > imagePortInLeft;
    BufferedPort<ImageOf<PixelRgb> > imagePortInRight;
    BufferedPort<ImageOf<PixelRgb> > outPortRight;
    BufferedPort<ImageOf<PixelRgb> > outPortLeft;

    Port *commandPort;
    string dir;
    int startCalibration;
    int boardWidth;
    int boardHeight;
    float squareSize;
    char pathL[256];
    char pathR[256];
    void printMatrix(Mat &matrix);
    bool checkTS(double TSLeft, double TSRight, double th=0.020);
    void preparePath(const char * dir, char* pathL, char* pathR, int num);
    void saveStereoImage(const char * dir, IplImage* left, IplImage * right, int num);
public:


    stereoCalibThread(ResourceFinder &rf, Port* commPort, const char *dir);
    void startCalib();
    bool threadInit();
    void threadRelease();
    void run(); 
    void onStop();
    bool writeCalibrationToFile( int width, int height, float fx, float fy,float cx, float cy, float k1, float k2, float p1, float p2, string groupname);

};


