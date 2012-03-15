#include "iCub/stereoVision/SceneFlow.h"

SceneFlow::SceneFlow(yarp::os::ResourceFinder &rf) : RateThread(10) 
{
    string localPortL="/SceneFlow/left:i";
    string localPortR="/SceneFlow/right:i";

    string inputL=rf.check("leftCamera",Value("/icub/camcalib/left/out")).asString().c_str();
    string inputR=rf.check("rightCamera",Value("/icub/camcalib/right/out")).asString().c_str();

    string configFileDisparity=rf.check("ConfigDisparity",Value("icubEyes.ini")).asString().c_str();
    imageL=new ImageOf<PixelRgb>;
    imageR=new ImageOf<PixelRgb>;

    success=true;
    init=true;
    initL=initR=false;

    success=success & imagePortInLeft.open(localPortL.c_str());
    success=success & imagePortInRight.open(localPortR.c_str());


    flowSem=new Semaphore(1);

    success=success & Network::connect(inputL.c_str(),localPortL.c_str());
    success=success & Network::connect(inputR.c_str(),localPortR.c_str());

    if(success)
    {
        ResourceFinder cameraFinder;
        cameraFinder.setDefaultContext("cameraCalibration/conf");
        cameraFinder.setDefaultConfigFile(configFileDisparity.c_str());
        int argc=0; char *argv[1];
        cameraFinder.configure("ICUB_ROOT",argc,argv);

        disp=new DisparityThread(cameraFinder);
        opt=new OpticalFlowThread(rf);

        disp->getRootTransformation(HL_root,0);
        disp->start();
        opt->start();
    }
    else
        fprintf(stdout,"Initialization Failed. Check the input ports \n");
}


bool SceneFlow::isOpen()
{
    return success;
}

void SceneFlow::close()
{
    disp->stop();
    delete disp;
    opt->stop();
    delete opt;

    imagePortInLeft.interrupt();
    imagePortInLeft.close();
    imagePortInRight.interrupt();
    imagePortInRight.close();

    delete imageL;
    delete imageR;
    delete flowSem;

}

void SceneFlow::run()
{
    if(!success)
        return;

    ImageOf<PixelRgb> *tmpL = imagePortInLeft.read(false);
    ImageOf<PixelRgb> *tmpR = imagePortInRight.read(false);

    if(tmpL!=NULL)
    {
        *imageL=*tmpL;
        imagePortInLeft.getEnvelope(TSLeft);
        initL=true;
    }

    if(tmpR!=NULL) 
    {
        *imageR=*tmpR;
        imagePortInRight.getEnvelope(TSRight);
        initR=true;
    }

    if(initL && initR )
    {
        if (init)
        {
            flowSem->wait();
            imgLNext= (IplImage*) imageL->getIplImage();
            imgRNext= (IplImage*) imageR->getIplImage();
            imgLPrev=NULL;
            imgRPrev=NULL;
            width=imgLNext->width;
            height=imgLNext->height;


            Mat tmpLNext(imgLNext);
            Mat tmpRNext(imgRNext);
            disp->setImages(tmpLNext,tmpRNext);
            while (!disp->checkDone())
                Time::delay(0.01);

            disp->getDisparity(dispNew);
            disp->getRectMatrix(RLNew);
            disp->getMapper(mapperNew);
            disp->getQMat(QNew);
            disp->getDisparityFloat(dispFloatNew);
            RLOld=RLNew;
            QOld=QNew;
            mapperOld=mapperNew;

            init=false;
            initL=initR=false;

            IplImage IplDispNew=dispNew;

            if (imgLPrev!=NULL)
                cvReleaseImage(&imgLPrev);
            if (imgRPrev!=NULL)
                cvReleaseImage(&imgRPrev);

            imgLPrev=(IplImage*)cvClone(imgLNext);
            imgRPrev=(IplImage*)cvClone(imgRNext);
            flowSem->post();
        }
        else
        {
            imgLNext= (IplImage*) imageL->getIplImage();
            imgRNext= (IplImage*) imageR->getIplImage();


            dispOld=dispNew.clone();
            dispFloatOld=dispFloatNew.clone();
            Mat tmpLNext(imgLNext);
            Mat tmpRNext(imgRNext);
            Mat tmpLPrev(imgLPrev);
            disp->setImages(tmpLNext,tmpRNext);
            disp->resume();
            opt->setImages(tmpLPrev,tmpLNext);
            opt->resume();

            while (!disp->checkDone()||!opt->checkDone())
              Time::delay(0.01);

            flowSem->wait();
            disp->getDisparity(dispNew);
            disp->getDisparityFloat(dispFloatNew);
            opt->getOptFlow(optFlow);
            flowSem->post();

            if (imgLPrev!=NULL)
                cvReleaseImage(&imgLPrev);
            if (imgRPrev!=NULL)
                cvReleaseImage(&imgRPrev);

            imgLPrev=(IplImage*)cvClone(imgLNext);
            imgRPrev=(IplImage*)cvClone(imgRNext);
        }
    }

}

void SceneFlow::triangulate(Point2f &pixel,Point3f &point,Mat &Mapper, Mat &disparity, Mat &Q, Mat &RLrect) 
{
    int u=(int) pixel.x; 
    int v=(int) pixel.y;

    // Mapping from Rectified Cameras to Original Cameras
    if(Mapper.empty()) 
    {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        return;
    }

    float usign=Mapper.ptr<float>(v)[2*u];
    float vsign=Mapper.ptr<float>(v)[2*u+1]; 

    u=cvRound(usign);
    v=cvRound(vsign);

    IplImage disp16=disparity;

    if(u<0 || u>=disp16.width || v<0 || v>=disp16.height) 
    {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        return;
    }
    else 
    {
        CvScalar scal= cvGet2D(&disp16,v,u);
        double dispVal=-scal.val[0]/16.0;
        float w= (float) ((float) dispVal*Q.at<double>(3,2)) + ((float)Q.at<double>(3,3));
        point.x= (float)((float) (usign+1)*Q.at<double>(0,0)) + ((float) Q.at<double>(0,3));
        point.y=(float)((float) (vsign+1)*Q.at<double>(1,1)) + ((float) Q.at<double>(1,3));
        point.z=(float) Q.at<double>(2,3);

        point.x=point.x/w;
        point.y=point.y/w;
        point.z=point.z/w;
    }
    // discard points far more than 2.5 meters or with not valid disparity (<0)
    if(point.z>2.5 || point.z<0) 
    {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        return;
    } 
    else 
    {
        Mat RLrecttemp=RLrect.t();
        Mat Tfake = Mat::zeros(0,3,CV_64F);
        Mat P(4,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;
        P.at<double>(3,0)=1;

        Mat Hrect = Mat::eye(4, 4, CV_64F);
        buildRotTras(RLrecttemp,Tfake,Hrect);
      
        P=HL_root*Hrect*P;
        point.x=(float) ((float) P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float) ((float) P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float) ((float) P.at<double>(2,0)/P.at<double>(3,0));
    }
}

void SceneFlow::buildRotTras(Mat & R, Mat & T, Mat & A) 
{
    for(int i = 0; i < R.rows; i++)
    {
        double* Mi = A.ptr<double>(i);
        double* MRi = R.ptr<double>(i);
        for(int j = 0; j < R.cols; j++)
             Mi[j]=MRi[j];
    }
    for(int i = 0; i < T.rows; i++)
    {
        double* Mi = A.ptr<double>(i);
        double* MRi = T.ptr<double>(i);
        Mi[3]=MRi[0];
     }
}



Point3f SceneFlow::getSceneFlowPixel(int u, int v)
{
    flowSem->wait();
    Point3f flowPoint;
    flowPoint.x=0.0;
    flowPoint.y=0.0;
    flowPoint.z=0.0;

    if(v>=dispFloatNew.rows || u>=dispFloatNew.cols || optFlow.empty())
    {
        flowSem->post();
        return flowPoint;
    }

    int valOld=(int)dispFloatOld.ptr<uchar>(v)[u];
    int valNew=(int)dispFloatNew.ptr<uchar>(v+cvRound(optFlow.ptr<float>(v)[2*u+1]))[u+cvRound(optFlow.ptr<float>(v)[2*u])];

    if (valOld==0 || valNew==0)
    {
        flowSem->post();
        return flowPoint;
    }

    Point2f point2Dold;
    point2Dold.x=(float)u;
    point2Dold.y=(float)v;
    Point3f point3Dold;
    triangulate(point2Dold,point3Dold,mapperOld,dispFloatOld,QOld,RLOld);
    if (point3Dold.x==0.0)
    {
        flowSem->post();
        return flowPoint;
    }

    Point2f point2Dnew;
    point2Dnew.x=u+optFlow.ptr<float>(v)[2*u];
    point2Dnew.y=v+optFlow.ptr<float>(v)[2*u+1];
    Point3f point3Dnew;
    triangulate(point2Dnew,point3Dnew,mapperNew,dispFloatNew,QNew,RLNew);
    if (point3Dnew.x==0.0)
    {
        flowSem->post();
        return flowPoint;
    }

    flowPoint.x=point3Dnew.x-point3Dold.x;
    flowPoint.y=point3Dnew.y-point3Dold.y;
    flowPoint.z=point3Dnew.z-point3Dold.z;
    flowSem->post();
    return flowPoint;

}


void SceneFlow::getSceneFlow(Mat &flow3D)
{
    flowSem->wait();
    flow3D.create(optFlow.rows,optFlow.cols,CV_32FC3);
    Point3f flowPoint;
    flowPoint.x=0.0;
    flowPoint.y=0.0;
    flowPoint.z=0.0;

    if(optFlow.empty())
    {
        flowSem->post();
        return;
    }
    
    for (int v=0; v<optFlow.rows; v++)
    {
        for(int u=0; u<optFlow.cols; u++)
        {
            int valOld=(int)dispFloatOld.ptr<uchar>(v)[u];
            int valNew=(int)dispFloatNew.ptr<uchar>(v+cvRound(optFlow.ptr<float>(v)[2*u+1]))[u+cvRound(optFlow.ptr<float>(v)[2*u])];

            if (valOld==0 || valNew==0)
            {
                flow3D.ptr<float>(v)[2*u]=(float)0.0;
                flow3D.ptr<float>(v)[2*u+1]=(float)0.0;
                flow3D.ptr<float>(v)[2*u+2]=(float)0.0;
                continue;
            }

            Point2f point2Dold;
            point2Dold.x=(float)u;
            point2Dold.y=(float)v;
            Point3f point3Dold;
            triangulate(point2Dold,point3Dold,mapperOld,dispFloatOld,QOld,RLOld);
            if (point3Dold.x==0.0)
            {
                flow3D.ptr<float>(v)[2*u]=(float)0.0;
                flow3D.ptr<float>(v)[2*u+1]=(float)0.0;
                flow3D.ptr<float>(v)[2*u+2]=(float)0.0;
                continue;
            }

            Point2f point2Dnew;
            point2Dnew.x=u+optFlow.ptr<float>(v)[2*u];
            point2Dnew.y=v+optFlow.ptr<float>(v)[2*u+1];
            Point3f point3Dnew;
            triangulate(point2Dnew,point3Dnew,mapperNew,dispFloatNew,QNew,RLNew);
            if (point3Dnew.x==0.0)
            {
                flow3D.ptr<float>(v)[2*u]=(float)0.0;
                flow3D.ptr<float>(v)[2*u+1]=(float)0.0;
                flow3D.ptr<float>(v)[2*u+2]=(float)0.0;
                continue;
            }

            flowPoint.x=point3Dnew.x-point3Dold.x;
            flowPoint.y=point3Dnew.y-point3Dold.y;
            flowPoint.z=point3Dnew.z-point3Dold.z;

            flow3D.ptr<float>(v)[2*u]=(float)flowPoint.x;
            flow3D.ptr<float>(v)[2*u+1]=(float)flowPoint.y;
            flow3D.ptr<float>(v)[2*u+2]=(float)flowPoint.z;
            }
    }

    flowSem->post();

}


void SceneFlow::draw2DMotionField(IplImage* imgMotion)
{
    flowSem->wait();
    int xSpace=7;
    int ySpace=7;
    float cutoff=0.2;
    float multiplier=10;
    CvScalar color=CV_RGB(255,0,0);

    CvPoint p0 = cvPoint(0,0);
    CvPoint p1 = cvPoint(0,0);

    if(imgMotion!=NULL)
        cvReleaseImage(&imgMotion);
    imgMotion=(IplImage*) cvClone(imgLPrev);


    if(optFlow.empty())
    {
        flowSem->post();
        return;
    }
    
    float deltaX, deltaY, angle, hyp;

    for(int i=0; i<optFlow.rows; i++) 
    {
        for (int j=0; j<optFlow.cols; j++)
        {
            p0.x = i;
            p0.y = j;
            deltaX = optFlow.ptr<float>(i)[2*j];
            deltaY = -optFlow.ptr<float>(i)[2*j+1];
            angle = atan2(deltaY, deltaX);
            hyp = sqrt(deltaX*deltaX + deltaY*deltaY);

            if(hyp > cutoff){
                p1.x = p0.x + cvRound(multiplier*hyp*cos(angle));
                p1.y = p0.y + cvRound(multiplier*hyp*sin(angle));
                cvLine( imgMotion, p0, p1, color,1, CV_AA, 0);
                p0.x = p1.x + cvRound(3*cos(angle-CV_PI + CV_PI/4));
                p0.y = p1.y + cvRound(3*sin(angle-CV_PI + CV_PI/4));
                cvLine( imgMotion, p0, p1, color,1, CV_AA, 0);

                p0.x = p1.x + cvRound(3*cos(angle-CV_PI - CV_PI/4));
                p0.y = p1.y + cvRound(3*sin(angle-CV_PI - CV_PI/4));
                cvLine( imgMotion, p0, p1, color,1, CV_AA, 0);
            }
        }
    }
    flowSem->post();
}

void SceneFlow::drawFlowModule(IplImage* imgMotion)
{
    flowSem->wait();
    IplImage * module =cvCreateImage(cvSize(imgMotion->width,imgMotion->height),32,1);
    IplImage * moduleU =cvCreateImage(cvSize(imgMotion->width,imgMotion->height),8,1);
    Mat vel[2];
    split(optFlow,vel);
    IplImage tx=(Mat)vel[0];
    IplImage ty=(Mat)vel[1];

    IplImage* velxpow=cvCloneImage(&tx);
    IplImage* velypow=cvCloneImage(&ty);

    cvPow(&tx, velxpow, 2);
    cvPow(&ty, velypow, 2);

    cvAdd(velxpow, velypow, module, NULL);
    cvPow(module, module, 0.5);
    cvNormalize(module, module, 0.0, 1.0, CV_MINMAX, NULL);
    cvZero(imgMotion);
    cvConvertScale(module,moduleU,255,0);
    cvMerge(moduleU,moduleU,moduleU,moduleU,imgMotion);
    cvReleaseImage(&module);
    cvReleaseImage(&moduleU);
    flowSem->post();
}


bool SceneFlow::threadInit()
{
    return true;
}

int SceneFlow::getImgWidth()
{
    return width;
}

int SceneFlow::getImgHeight()
{
    return height;
}
