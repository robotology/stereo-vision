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

#include <iCub/iKin/iKinFwd.h>
#include <yarp/os/all.h>
#include "iCub/stereoVision/disparityThread.h"
#include "iCub/stereoVision/opticalFlowThread.h"
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Stamp.h>

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
* The base class defining the scene flow (optical flow in the 3D space).
* It computes the 3D motion field of a stereo pairs.
*/

class SceneFlow : public RateThread
{
private:

    bool success;
    DisparityThread* disp;
    OpticalFlowThread* opt;
    yarp::os::Stamp TSLeft;
    yarp::os::Stamp TSRight;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* imageL;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* imageR;
    IplImage* imgLPrev;
    IplImage* imgRPrev;
    IplImage* imgLNext;
    IplImage* imgRNext;

    Mat optFlow;
    Mat dispOld;
    Mat dispNew;
    Mat mapperOld;
    Mat mapperNew;
    Mat QOld;
    Mat QNew;
    Mat RLOld;
    Mat RLNew;
    Mat dispFloatOld;
    Mat dispFloatNew;
    Mat HL_root;

    int width;
    int height;
    bool initL,initR;
    bool init;

    Semaphore* flowSem;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortInLeft;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortInRight;
    void triangulate(Point2f &pixel,Point3f &point,Mat &Mapper, Mat &disparity, Mat &Q, Mat &RLrect);
    void buildRotTras(Mat & R, Mat & T, Mat & A);
    void printMatrix(Mat &matrix);
public:

    SceneFlow(yarp::os::ResourceFinder &rf);
    ~SceneFlow(void) {};


    bool isOpen();



    bool threadInit(void);
    void run(void); 
    void onStop(void);
    void close();
    Point3f getSceneFlowPixel(int u, int v);
    IplImage* draw2DMotionField();
    void drawFlowModule(IplImage* imgMotion);
    int getImgWidth();
    int getImgHeight();
    void getSceneFlow(Mat &flow3D);
    void getSceneFlow(Mat &flow3D, int U1, int V1, int U2, int V2);

    void threadRelease();
    void recalibrate();
};
