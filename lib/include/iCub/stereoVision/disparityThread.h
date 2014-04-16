
#include <iCub/stereoVision/stereoCamera.h>
#include <yarp/dev/PolyDriver.h>
#include <iCub/iKin/iKinFwd.h>
#include <yarp/dev/GazeControl.h>

#define LEFT    0
#define RIGHT   1

#ifdef USING_GPU
    #include <iCub/stereoVision/utils.h>
#endif

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
    bool useHorn;
    bool updateCamera;
    bool updateOnce;

    bool useBestDisp;
    int uniquenessRatio; 
    int speckleWindowSize;
    int speckleRange;
    int numberOfDisparities;
    int SADWindowSize;
    int minDisparity;
    int preFilterCap;
    int disp12MaxDiff;
    int widthInit;

    #ifdef USING_GPU
        /* pointer to the utilities class */
        Utilities *utils;
    #endif    

    yarp::sig::Vector QL;
    yarp::sig::Vector QR;

    Matrix yarp_initLeft,yarp_initRight;
    Matrix yarp_H0;
    Mutex mutexDisp;
    PolyDriver gazeCtrl;
    IGazeControl* igaze;

    iCubEye *LeyeKin;
    iCubEye *ReyeKin;
    yarp::dev::PolyDriver polyHead;
    yarp::dev::IEncoders *posHead;
    yarp::dev::IControlLimits *HctrlLim;

    yarp::dev::PolyDriver polyTorso;
    yarp::dev::IEncoders *posTorso;
    yarp::dev::IControlLimits *TctrlLim;

    yarp::sig::Vector eyes0,eyes;
    Mat HL_root,HR_root;
    Mat R0,T0;

    string moduleName;
    string robotName;

    void buildRotTras(Mat &R, Mat &T, Mat &A);
    bool loadStereoParameters(yarp::os::ResourceFinder &rf, Mat &KL, Mat &KR, Mat &DistL, Mat &DistR, Mat &Ro, Mat &To);
    Matrix getCameraHGazeCtrl(int camera);
    Matrix getCameraH(yarp::sig::Vector &head_angles,yarp::sig::Vector &torso_angles, iCubEye *eyeKin, int camera);
    void printMatrixYarp(Matrix &A);
    void convert(Matrix& matrix, Mat& mat);
    void convert(Mat& mat, Matrix& matrix);
    void updateViaGazeCtrl(const bool update);
    void updateViaKinematics(const yarp::sig::Vector& deyes);
    bool loadExtrinsics(yarp::os::ResourceFinder& rf, Mat& Ro, Mat& To, yarp::sig::Vector& eyes);

public:
    DisparityThread(const string &name, yarp::os::ResourceFinder &rf, bool useHorn=true, bool updateCamera=false, bool rectify=true);
    ~DisparityThread() { };

    void setImages(Mat &left, Mat &right);
    void getDisparity(Mat &Disp);
    Point3f get3DPointMatch(double u1, double v1, double u2, double v2, string drive);
    void getDisparityFloat(Mat &Disp);
    void getQMat(Mat &Q);
    void getMapper(Mat &Mapper);
    void getRectMatrix(Mat &RL);
    void triangulate(Point2f &pixel,Point3f &point) ;
    bool checkDone();
    void getRootTransformation(Mat & Trans,int eye=LEFT);
    bool isOpen();
    void setDispParameters(bool _useBestDisp, int _uniquenessRatio, int _speckleWindowSize,int _speckleRange, int _numberOfDisparities, int _SADWindowSize, int _minDisparity, int _preFilterCap, int _disp12MaxDiff);

    void updateCamerasOnce();
    void startUpdate();
    void stopUpdate();

    bool threadInit();
    void threadRelease();
    void run(); 
    void onStop();
};


