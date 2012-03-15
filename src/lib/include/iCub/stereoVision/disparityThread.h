#include <iCub/stereoVision/StereoCamera.h>
#include <yarp/dev/PolyDriver.h>
#include <iCub/iKin/iKinFwd.h>
#include <yarp/dev/GazeControl.h>

#define LEFT    0
#define RIGHT   1

using namespace std; 
using namespace yarp::os; 
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;

/**
* \ingroup StereoVisionLib
*
* The class defining the disparity computation.
* It computes the depth map and it updates the icub's eye relative positions.
*/
class DisparityThread : public RateThread
{
private:

    StereoCamera *stereo;
    bool done;
    bool work;
    bool init;
    bool success;
    bool useCalibrated;

    Matrix yarp_initLeft,yarp_initRight;
    Matrix yarp_H0;
    Semaphore* mutexDisp;
    PolyDriver* gazeCtrl;
    IGazeControl* igaze;
    Matrix H;
    Mat HL_root;
    Mat HR_root;

    void buildRotTras(Mat & R, Mat & T, Mat & A);
    bool loadStereoParameters(yarp::os::ResourceFinder &rf, Mat &KL, Mat &KR, Mat &DistL, Mat &DistR, Mat &Ro, Mat &T);
    Matrix getCameraH(int camera);
    void printMatrixYarp(Matrix &A);
    void convert(Matrix& matrix, Mat& mat);
    void convert(Mat& mat, Matrix& matrix);

public:

    DisparityThread(yarp::os::ResourceFinder &rf);
    ~DisparityThread(void) {};


    void setImages(Mat &left, Mat &right);
    void getDisparity(Mat &Disp);
    void getDisparityFloat(Mat &Disp);
    void getQMat(Mat &Q);
    void getMapper(Mat &Mapper);
    void getRectMatrix(Mat &RL);
    void triangulate(Point2f &pixel,Point3f &point) ;
    bool checkDone();
    void getRootTransformation(Mat & Trans,int eye=LEFT);
    bool isOpen();



    bool threadInit(void);
    void threadRelease(void);
    void run(void); 
    void onStop(void);
 
};
