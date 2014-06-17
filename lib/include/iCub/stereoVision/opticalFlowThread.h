/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Vadim Tikhanoff
 * email:   vadim.tikhanoff@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txtd
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

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
