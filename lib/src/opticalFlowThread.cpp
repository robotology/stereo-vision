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

#include "iCub/stereoVision/opticalFlowThread.h"

using namespace cv;
using namespace yarp::os;

OpticalFlowThread::OpticalFlowThread(yarp::os::ResourceFinder &rf) : RateThread(10) 
{
    work=false;
    done=true;
    int useD= rf.check("denseFlow",Value(1)).asInt();
    this->dense= useD ? true : false;
    fprintf(stdout, "Optical Flow Thread has started...\n");

}

void OpticalFlowThread::run() 
{
    if(work) 
    {
        if(dense)
        {
            Mat leftPrevGray;
            Mat leftNextGray;
            cvtColor(leftPrev,leftPrevGray,CV_RGB2GRAY,1);
            cvtColor(leftNext,leftNextGray,CV_RGB2GRAY,1);
            int flag=0;
            if(!optFlow.empty())
                optFlow.setTo(Scalar(0));
            
            calcOpticalFlowFarneback(leftPrevGray,leftNextGray,optFlow,0.25,5,9,5,7,1.5,flag);
        }
        else
        {
            IplImage previous=leftPrev;
            IplImage current=leftNext;
            computeFlowSparse(&previous,&current,optFlow);
        }
        done=true;
        work=false;
        this->suspend();
    }
}

void OpticalFlowThread::setImages(Mat &_leftPrev, Mat &_leftNext) 
{
    leftPrev=_leftPrev;
    leftNext=_leftNext;
    done=false;
    work=true;
}

void OpticalFlowThread::setFlow(int flowType) 
{
    if(flowType==DENSE)
        dense=true;
    else
        dense=false;
}

void OpticalFlowThread::getOptFlow(Mat &_optFlow) 
{
    _optFlow=optFlow.clone();
}

bool OpticalFlowThread::threadInit() 
{
    return true;
}

void OpticalFlowThread::threadRelease() 
{	
	fprintf(stdout,"Optical Flow Thread Closed... \n");
}

bool OpticalFlowThread::checkDone() 
{
    return done;
}
void OpticalFlowThread::computeFlowSparse(IplImage* previous, IplImage* current, Mat &optFlow)
{
	optFlow.setTo(Scalar(0));
    optFlow.create(previous->height,previous->width,CV_32FC2);

    IplImage* leftPrevGray = cvCreateImage(cvSize(previous->width,previous->height),previous->depth,1);
    IplImage* leftNextGray = cvCreateImage(cvSize(previous->width,previous->height),previous->depth,1);;
    IplImage *eigImg=cvCloneImage(previous);
    IplImage *tmpImg=cvCloneImage(previous);
    CvSize pyrSize=cvSize(previous->width+8,previous->height/3);
    IplImage *prevPyr=cvCreateImage(pyrSize,IPL_DEPTH_32F,1);
    IplImage *currPyr=cvCreateImage(pyrSize,IPL_DEPTH_32F,1);

    cvCvtColor(previous,leftPrevGray,CV_RGB2GRAY);
    cvCvtColor(current,leftNextGray,CV_RGB2GRAY);

    int max_corners=previous->width*previous->height;;
    int winSize=7;
    int lk_error_thresh=200;
    int corner_count=max_corners;

    CvPoint2D32f *prevCorners=new CvPoint2D32f[max_corners];
    CvPoint2D32f *currCorners=new CvPoint2D32f[max_corners];

    char *features_found=new char[max_corners];
    float *feature_errors=new float[max_corners];
    cvGoodFeaturesToTrack(leftPrevGray,eigImg,tmpImg,prevCorners,&corner_count,0.001,5.0,0,3,0,0.04);

    cvCalcOpticalFlowPyrLK(leftPrevGray,leftNextGray,prevPyr,currPyr,prevCorners,currCorners,corner_count,
                                cvSize(winSize,winSize),5,features_found,feature_errors,
                                cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ),0);
    for(int k=0; k<corner_count; k++)
    {
        if(features_found[k]!=0 && feature_errors[k]<lk_error_thresh)
        {
            double deltaX=currCorners[k].x - prevCorners[k].x; //deltaX
            double deltaY=currCorners[k].y - prevCorners[k].y; // deltaY
            double distance=sqrt(deltaX*deltaX + deltaY*deltaY);
            distance=sqrt(distance);
            optFlow.ptr<float>(cvRound(prevCorners[k].y))[2*cvRound(prevCorners[k].x)]=(float)deltaX;
            optFlow.ptr<float>(cvRound(prevCorners[k].y))[2*cvRound(prevCorners[k].x)+1]=(float)deltaY;
        }

    }

    cvReleaseImage(&leftPrevGray);
    cvReleaseImage(&leftNextGray);
    cvReleaseImage(&eigImg);
    cvReleaseImage(&tmpImg);
    cvReleaseImage(&prevPyr);
    cvReleaseImage(&currPyr);
    delete features_found;
    delete feature_errors;
    delete prevCorners;
    delete currCorners;
}


void OpticalFlowThread::onStop()
{
	this->work=false;
	this->done=true;


}
