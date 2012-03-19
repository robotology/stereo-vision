#include <cv.h>
#include <highgui.h>
#include <yarp/os/RateThread.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <yarp/os/all.h>

#define DENSE    1

using namespace cv;

/**
* \ingroup StereoVisionLib
*
* The base class defining the 2D optical flow.
* It computes the 2D motion field in the image.
*/
class OpticalFlowThread : public yarp::os::RateThread
{
private:

    cv::Mat optFlow;
    cv::Mat leftPrev;
    cv::Mat leftNext;
    bool done;
    bool work;
    bool dense;
    void computeFlowSparse(IplImage* previous, IplImage* current, Mat &optFlow);

public:

     OpticalFlowThread(yarp::os::ResourceFinder &rf);
    ~OpticalFlowThread() {};

    void setImages(cv::Mat &_leftPrev, cv::Mat &_leftNext);
    void getOptFlow(cv::Mat &_optFlow);
    void setFlow(int flowType);

    bool checkDone();

    bool threadInit();
    void threadRelease();
    void run();
    void onStop(void);
 
};
