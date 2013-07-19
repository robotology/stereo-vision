#include "SFM_module.h"



bool SFM::configure(ResourceFinder &rf)
{
    string name=rf.check("name",Value("/SFM")).asString().c_str();
    string left=rf.check("leftPort",Value("/left:i")).asString().c_str();
    left=name+left;
    string right=rf.check("rightPort",Value("/right:i")).asString().c_str();

    string outDispName=rf.check("outDispPort",Value("/disp:o")).asString().c_str();
    string outMatchName=rf.check("outMatchPort",Value("/match:o")).asString().c_str();

    right=name+right;
    outMatchName=name+outMatchName;
    outDispName=name+outDispName;

    int calib= rf.check("useCalibrated",Value(1)).asInt();
    bool useCalibrated= calib ? true : false;

    leftImgPort.open(left.c_str());
    rightImgPort.open(right.c_str());
    outMatch.open(outMatchName.c_str());
    outDisp.open(outDispName.c_str());

    this->stereo=new StereoCamera(true);
    Mat KL, KR, DistL, DistR, R, T;
    loadStereoParameters(rf,KL,KR,DistL,DistR,R,T);

    stereo->setIntrinsics(KL,KR,DistL,DistR);
    stereo->setRotation(R,0);
    stereo->setTranslation(T,0);

    if(useCalibrated)
    {
        Mat KL=this->stereo->getKleft();
        Mat KR=this->stereo->getKright();
        Mat zeroDist=Mat::zeros(1,8,CV_64FC1);
        this->stereo->setIntrinsics(KL,KR,zeroDist,zeroDist);
    }

    output_match=NULL;
    outputD=NULL;
    init=true;

    #ifdef USING_GPU
        utils = new Utilities();
        utils->initSIFT_GPU();
    #endif
    return true;
}

bool SFM::close()
{
    
    leftImgPort.interrupt();
    leftImgPort.close();

    rightImgPort.interrupt();
    rightImgPort.close();

    outDisp.close();
    outDisp.interrupt();

    outMatch.close();
    outMatch.interrupt();

    if(output_match!=NULL)
        cvReleaseImage(&output_match);

    if(outputD!=NULL)
        cvReleaseImage(&outputD);

    #ifdef USING_GPU
        delete utils;
    #endif

    return true;
}

bool SFM::interruptModule()
{
    leftImgPort.interrupt();
    leftImgPort.close();
    rightImgPort.interrupt();
    rightImgPort.close();
    outDisp.close();
    outDisp.interrupt();

    outMatch.close();
    outMatch.interrupt();

    return true;
}
bool SFM::updateModule()
{
    ImageOf<PixelRgb> *yarp_imgL=NULL;
    ImageOf<PixelRgb> *yarp_imgR=NULL;


    yarp_imgL=leftImgPort.read(true);
    yarp_imgR=rightImgPort.read(true);

    if(yarp_imgL==NULL || yarp_imgR==NULL)
        return true;

    left=(IplImage*) yarp_imgL->getIplImage();
    right=(IplImage*) yarp_imgR->getIplImage();

    if(init)
    {
        output_match=cvCreateImage(cvSize(left->width*2,left->height),8,3);
        outputD=cvCreateImage(cvSize(left->width,left->height),8,3);
        // find matches
        init=false;
    }

    #ifdef USING_GPU
        Mat leftMat(left); 
        Mat rightMat(right);
        this->stereo->setImages(left,right);
        matMatches = Mat(rightMat.rows, 2*rightMat.cols, CV_8UC3);
        matMatches.adjustROI(0, 0, 0, -leftMat.cols);
        leftMat.copyTo(matMatches);
        matMatches.adjustROI(0, 0, -leftMat.cols, leftMat.cols);
        rightMat.copyTo(matMatches);
        matMatches.adjustROI(0, 0, leftMat.cols, 0);

        utils->extractMatch_GPU( leftMat, rightMat, matMatches );
        vector<Point2f> leftM;
        vector<Point2f> rightM;
        
        utils->getMatches(leftM,rightM);
        this->stereo->setMatches(leftM,rightM);
        this->stereo->estimateEssential();
        
        


        Mat F= this->stereo->getFundamental();
        vector<Point2f> matchtmp=this->stereo->getMatchRight();
        if(matchtmp.size()>0)
        {
            Mat m(matchtmp);
            vector<Vec3f> lines;
            cv::computeCorrespondEpilines(m,2,F,lines);
            for (cv::vector<cv::Vec3f>::const_iterator it = lines.begin(); it!=lines.end(); ++it)
            {
                cv::line(matMatches, cv::Point(0,-(*it)[2]/(*it)[1]), cv::Point(left->width,-((*it)[2] + (*it)[0]*left->width)/(*it)[1]),cv::Scalar(0,0,255));
            }        
        }
        cvtColor( matMatches, matMatches, CV_BGR2RGB);
        ImageOf<PixelBgr>& imgMatch= outMatch.prepare();
        imgMatch.resize(matMatches.cols, matMatches.rows);
        IplImage tmpR = matMatches;
        cvCopyImage( &tmpR, (IplImage *) imgMatch.getIplImage());        
        outMatch.write();
        

        

        Mat matches=this->stereo->drawMatches();

        this->stereo->essentialDecomposition();
        //this->stereo->hornRelativeOrientations();
        
        this->stereo->computeDisparity(true,15,50,64,16,7,0,63,0);        
        
        if(outDisp.getOutputCount()>0 && matchtmp.size()>0)
        {
            IplImage disp=stereo->getDisparity();
            cvCvtColor(&disp,outputD,CV_GRAY2RGB);
            ImageOf<PixelBgr>& outim=outDisp.prepare();
            outim.wrapIplImage(outputD);
            outDisp.write();
        }        
        
    #else
        // setting undistorted images
        this->stereo->setImages(left,right);

        // find matches
        this->stereo->findMatch(false,15,10.0);

        //Estimating fundamentalMatrix
        this->stereo->estimateEssential();
        Mat F= this->stereo->getFundamental();

        Mat matches=this->stereo->drawMatches();
        vector<Point2f> rightM=this->stereo->getMatchRight();

        this->stereo->essentialDecomposition();
        this->stereo->hornRelativeOrientations();

        this->stereo->computeDisparity(true,15,50,16,64,7,-32,32,0);

        if(outMatch.getOutputCount()>0 && rightM.size()>0)
        {
            Mat m(rightM);
            vector<Vec3f> lines;
            cv::computeCorrespondEpilines(m,2,F,lines);
            for (cv::vector<cv::Vec3f>::const_iterator it = lines.begin(); it!=lines.end(); ++it)
            {
                cv::line(matches,
                cv::Point(0,-(*it)[2]/(*it)[1]),
                cv::Point(left->width,-((*it)[2] + (*it)[0]*left->width)/(*it)[1]),
                cv::Scalar(255,255,255));
            }


            IplImage ipl_matches=matches;
            cvCvtColor(&ipl_matches,output_match,CV_BGR2RGB);

            ImageOf<PixelBgr>& outim=outMatch.prepare();
            outim.wrapIplImage(output_match);
            outMatch.write();
        }
        

        if(outDisp.getOutputCount()>0 && rightM.size()>0)
        {
            IplImage disp=stereo->getDisparity();
            cvCvtColor(&disp,outputD,CV_GRAY2RGB);
            ImageOf<PixelBgr>& outim=outDisp.prepare();
            outim.wrapIplImage(outputD);
            outDisp.write();
        }
    #endif

    return true;
}

double SFM::getPeriod()
{
    return 0.1;
}



bool SFM::loadStereoParameters(yarp::os::ResourceFinder &rf, Mat &KL, Mat &KR, Mat &DistL, Mat &DistR, Mat &Ro, Mat &T)
{

    Bottle left=rf.findGroup("CAMERA_CALIBRATION_LEFT");
    if(!left.check("fx") || !left.check("fy") || !left.check("cx") || !left.check("cy"))
        return false;

    double fx=left.find("fx").asDouble();
    double fy=left.find("fy").asDouble();

    double cx=left.find("cx").asDouble();
    double cy=left.find("cy").asDouble();

    double k1=left.check("k1",Value(0)).asDouble();
    double k2=left.check("k2",Value(0)).asDouble();

    double p1=left.check("p1",Value(0)).asDouble();
    double p2=left.check("p2",Value(0)).asDouble();

    DistL=Mat::zeros(1,8,CV_64FC1);
    DistL.at<double>(0,0)=k1;
    DistL.at<double>(0,1)=k2;
    DistL.at<double>(0,2)=p1;
    DistL.at<double>(0,3)=p2;
    

    KL=Mat::eye(3,3,CV_64FC1);
    KL.at<double>(0,0)=fx;
    KL.at<double>(0,2)=cx;
    KL.at<double>(1,1)=fy;
    KL.at<double>(1,2)=cy;

    Bottle right=rf.findGroup("CAMERA_CALIBRATION_RIGHT");
    if(!right.check("fx") || !right.check("fy") || !right.check("cx") || !right.check("cy"))
        return false;

    fx=right.find("fx").asDouble();
    fy=right.find("fy").asDouble();

    cx=right.find("cx").asDouble();
    cy=right.find("cy").asDouble();

    k1=right.check("k1",Value(0)).asDouble();
    k2=right.check("k2",Value(0)).asDouble();

    p1=right.check("p1",Value(0)).asDouble();
    p2=right.check("p2",Value(0)).asDouble();

    DistR=Mat::zeros(1,8,CV_64FC1);
    DistR.at<double>(0,0)=k1;
    DistR.at<double>(0,1)=k2;
    DistR.at<double>(0,2)=p1;
    DistR.at<double>(0,3)=p2;
    

    KR=Mat::eye(3,3,CV_64FC1);
    KR.at<double>(0,0)=fx;
    KR.at<double>(0,2)=cx;
    KR.at<double>(1,1)=fy;
    KR.at<double>(1,2)=cy;

    Ro=Mat::zeros(3,3,CV_64FC1);
    T=Mat::zeros(3,1,CV_64FC1);

    Bottle extrinsics=rf.findGroup("STEREO_DISPARITY");
    if (Bottle *pXo=extrinsics.find("HN").asList()) {
        for (int i=0; i<(pXo->size()-4); i+=4) {
            Ro.at<double>(i/4,0)=pXo->get(i).asDouble();
            Ro.at<double>(i/4,1)=pXo->get(i+1).asDouble();
            Ro.at<double>(i/4,2)=pXo->get(i+2).asDouble();
            T.at<double>(i/4,0)=pXo->get(i+3).asDouble();
        }
    }
    else
        return false;

    return true;
}


int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("icubEyes.ini"); 
    rf.setDefaultContext("cameraCalibration/conf");   
    rf.configure("ICUB_ROOT",argc,argv);

    SFM mod;

    return mod.runModule(rf);
}


