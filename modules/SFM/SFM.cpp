/* 
 * Copyright (C) 2015 RobotCub Consortium
 * Author: Sean Ryan Fanello, Giulia Pasquale
 * email:   sean.fanello@iit.it giulia.pasquale@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <cmath>
#include <algorithm>
#include "SFM.h"


/******************************************************************************/
bool SFM::configure(ResourceFinder &rf)
{
    string name=rf.check("name",Value("SFM")).asString().c_str();
    string robot=rf.check("robot",Value("icub")).asString().c_str();
    string left=rf.check("leftPort",Value("/left:i")).asString().c_str();
    string right=rf.check("rightPort",Value("/right:i")).asString().c_str();
    string SFMFile=rf.check("SFMFile",Value("SFM.ini")).asString().c_str();

    string sname;
    sname="/"+name;
    left=sname+left;
    right=sname+right;

    string outDispName=rf.check("outDispPort",Value("/disp:o")).asString().c_str();
    string outMatchName=rf.check("outMatchPort",Value("/match:o")).asString().c_str();

    string outLeftRectImgPortName=rf.check("outLeftRectImgPort",Value("/rect_left:o")).asString().c_str();
    string outRightRectImgPortName=rf.check("outRightRectImgPort",Value("/rect_right:o")).asString().c_str();

    ResourceFinder localCalibration;
    localCalibration.setContext("cameraCalibration");
    localCalibration.setDefaultConfigFile(SFMFile.c_str());
    localCalibration.configure(0,NULL);

    this->camCalibFile=localCalibration.getHomeContextPath().c_str();
    this->camCalibFile+="/"+SFMFile;

    outMatchName=sname+outMatchName;
    outDispName=sname+outDispName;

    outLeftRectImgPortName=sname+outLeftRectImgPortName;
    outRightRectImgPortName=sname+outRightRectImgPortName;

    string rpc_name=sname+"/rpc";
    string world_name=sname+rf.check("outWorldPort",Value("/world")).asString().c_str();

    int calib=rf.check("useCalibrated",Value(1)).asInt();
    bool useCalibrated=(calib!=0);

    leftImgPort.open(left.c_str());
    rightImgPort.open(right.c_str());
    outMatch.open(outMatchName.c_str());
    outDisp.open(outDispName.c_str());
    handlerPort.open(rpc_name.c_str());
    worldCartPort.open((world_name+"/cartesian:o").c_str());
    worldCylPort.open((world_name+"/cylindrical:o").c_str());
    attach(handlerPort);

    outLeftRectImgPort.open(outLeftRectImgPortName.c_str());
    outRightRectImgPort.open(outRightRectImgPortName.c_str());

    this->stereo = new StereoCamera(true);

    if (!rf.check("use_sgbm"))
        stereo->initELAS(rf);

    Mat KL, KR, DistL, DistR;

    loadIntrinsics(rf,KL,KR,DistL,DistR);
    loadExtrinsics(localCalibration,R0,T0,eyes0);
    eyes.resize(eyes0.length(),0.0);

    stereo->setIntrinsics(KL,KR,DistL,DistR);

    this->useBestDisp=true;
    this->uniquenessRatio=15;
    this->speckleWindowSize=50;
    this->speckleRange=16;
    this->SADWindowSize=7;
    this->minDisparity=0;
    this->preFilterCap=63;
    this->disp12MaxDiff=0;

    this->numberOfDisparities = 96;

    this->doBLF = true;
    bool skipBLF = rf.check("skipBLF");
    if (skipBLF){
        this->doBLF = false;}
    // this->doBLF = rf.check("doBLF",Value(true)).asBool();
    cout << " Bilateral filter set to " << doBLF << endl;
    this->sigmaColorBLF = 10.0;
    this->sigmaSpaceBLF = 10.0;


    this->HL_root=Mat::zeros(4,4,CV_64F);
    this->HR_root=Mat::zeros(4,4,CV_64F);

    if (useCalibrated)
    {
        Mat KL=this->stereo->getKleft();
        Mat KR=this->stereo->getKright();
        Mat zeroDist=Mat::zeros(1,8,CV_64FC1);
        this->stereo->setIntrinsics(KL,KR,zeroDist,zeroDist);
    }

    output_match=NULL;
    init=true;
    numberOfTrials=0;

#ifdef USING_GPU
    utils=new Utilities();
    utils->initSIFT_GPU();
#endif

    Property optionHead;
    optionHead.put("device","remote_controlboard");
    optionHead.put("remote",("/"+robot+"/head").c_str());
    optionHead.put("local",(sname+"/headClient").c_str());
    if (headCtrl.open(optionHead))
    {
        headCtrl.view(iencs);
        iencs->getAxes(&nHeadAxes);
    }
    else
    {
        cout<<"Devices not available"<<endl;
        return false;
    }

    Property optionGaze;
    optionGaze.put("device","gazecontrollerclient");
    optionGaze.put("remote","/iKinGazeCtrl");
    optionGaze.put("local",(sname+"/gazeClient").c_str());
    if (gazeCtrl.open(optionGaze))
        gazeCtrl.view(igaze);
    else
    {
        cout<<"Devices not available"<<endl;
        headCtrl.close();
        return false;
    }

    if (!R0.empty() && !T0.empty())
    {
        stereo->setRotation(R0,0);
        stereo->setTranslation(T0,0);
    }
    else
    {
        cout << "No local calibration file found in " <<  camCalibFile << " ... Using Kinematics and Running SFM once." << endl;
        updateViaGazeCtrl(true);
        R0=this->stereo->getRotation();
        T0=this->stereo->getTranslation();
    }

    doSFM=false;
    updateViaGazeCtrl(false);

    return true;
}


/******************************************************************************/
void SFM::updateViaKinematics(const yarp::sig::Vector& deyes)
{
    double dpan=CTRL_DEG2RAD*deyes[1];
    double dver=CTRL_DEG2RAD*deyes[2];

    yarp::sig::Vector rot_l_pan(4,0.0);
    rot_l_pan[1]=1.0;
    rot_l_pan[3]=dpan+dver/2.0;
    Matrix L1=axis2dcm(rot_l_pan);

    yarp::sig::Vector rot_r_pan(4,0.0);
    rot_r_pan[1]=1.0;
    rot_r_pan[3]=dpan-dver/2.0;
    Matrix R1=axis2dcm(rot_r_pan);

    Mat RT0=buildRotTras(R0,T0);
    Matrix H0; convert(RT0,H0);
    Matrix H=SE3inv(R1)*H0*L1;

    Mat R=Mat::zeros(3,3,CV_64F);
    Mat T=Mat::zeros(3,1,CV_64F);

    for (int i=0; i<R.rows; i++)
        for(int j=0; j<R.cols; j++)
            R.at<double>(i,j)=H(i,j);

    for (int i=0; i<T.rows; i++)
        T.at<double>(i,0)=H(i,3);

    this->stereo->setRotation(R,0);
    this->stereo->setTranslation(T,0);
}


/******************************************************************************/
void SFM::updateViaGazeCtrl(const bool update)
{
    Matrix L1=getCameraHGazeCtrl(LEFT);
    Matrix R1=getCameraHGazeCtrl(RIGHT);

    Matrix RT=SE3inv(R1)*L1;

    Mat R=Mat::zeros(3,3,CV_64F);
    Mat T=Mat::zeros(3,1,CV_64F);

    for (int i=0; i<R.rows; i++)
        for(int j=0; j<R.cols; j++)
            R.at<double>(i,j)=RT(i,j);

    for (int i=0; i<T.rows; i++)
        T.at<double>(i,0)=RT(i,3);

    if (update)
    {
        stereo->setRotation(R,0);
        stereo->setTranslation(T,0);
    }
    else
        stereo->setExpectedPosition(R,T);
}


/******************************************************************************/
bool SFM::interruptModule()
{
    leftImgPort.interrupt();
    rightImgPort.interrupt();
    outDisp.interrupt();
    handlerPort.interrupt();
    outMatch.interrupt();
    worldCartPort.interrupt();
    worldCylPort.interrupt();

    outLeftRectImgPort.interrupt();
    outRightRectImgPort.interrupt();

    return true;
}


/******************************************************************************/
bool SFM::close()
{
    leftImgPort.close();
    rightImgPort.close();
    outDisp.close();
    outMatch.close();
    handlerPort.close();
    worldCartPort.close();
    worldCylPort.close();

    if (output_match!=NULL)
        cvReleaseImage(&output_match);

    outLeftRectImgPort.close();
    outRightRectImgPort.close();

    headCtrl.close();
    gazeCtrl.close();

#ifdef USING_GPU
    delete utils;
#endif

    delete stereo;

    return true;
}


/******************************************************************************/
bool SFM::updateModule()
{
    ImageOf<PixelRgb> *yarp_imgL=leftImgPort.read(true);
    ImageOf<PixelRgb> *yarp_imgR=rightImgPort.read(true);

    Stamp stamp_left, stamp_right;
    leftImgPort.getEnvelope(stamp_left);
    rightImgPort.getEnvelope(stamp_right);

    if ((yarp_imgL==NULL) || (yarp_imgR==NULL))
        return true;

    // read encoders
    iencs->getEncoder(nHeadAxes-3,&eyes[0]);
    iencs->getEncoder(nHeadAxes-2,&eyes[1]);
    iencs->getEncoder(nHeadAxes-1,&eyes[2]);

    updateViaKinematics(eyes-eyes0);
    updateViaGazeCtrl(false);

    left=(IplImage*)yarp_imgL->getIplImage();
    right=(IplImage*)yarp_imgR->getIplImage();

    if (init)
    {
        output_match=cvCreateImage(cvSize(left->width*2,left->height),8,3);
        this->numberOfDisparities=(left->width<=320)?96:128;

        init=false;
    }

    getCameraHGazeCtrl(LEFT);
    getCameraHGazeCtrl(RIGHT);

    Mat leftMat=cvarrToMat(left);
    Mat rightMat=cvarrToMat(right);
    this->stereo->setImages(left,right);

    mutexRecalibration.lock();
    if (doSFM)
    {
#ifdef USING_GPU
        utils->extractMatch_GPU(leftMat,rightMat);
        vector<Point2f> leftM,rightM;
        utils->getMatches(leftM,rightM);
        mutexDisp.lock();
        this->stereo->setMatches(leftM,rightM);
#else
        this->stereo->findMatch(false);
#endif
        this->stereo->estimateEssential();
        bool ok=this->stereo->essentialDecomposition();
        mutexDisp.unlock();

        if (ok)
        {
            calibUpdated=true;
            doSFM=false;
            calibEndEvent.signal();
        }
        else
        {
            if (++numberOfTrials>5)
            {
                calibUpdated=false;
                doSFM=false;
                calibEndEvent.signal();
            }
        }
    }
    mutexRecalibration.unlock();

    mutexDisp.lock();
    this->stereo->computeDisparity(this->useBestDisp,this->uniquenessRatio,this->speckleWindowSize,
            this->speckleRange,this->numberOfDisparities,this->SADWindowSize,
            this->minDisparity,this->preFilterCap,this->disp12MaxDiff);
    mutexDisp.unlock();

    // DEBUG
    /*int uR,vR;
    Point3f point = this->get3DPointsAndDisp(160,120,uR,vR,"ROOT");
    circle(leftMat,cvPoint(160,120),2,cvScalar(255,0,0),2);
    circle(rightMat,cvPoint(uR,vR),2,cvScalar(0,255,0),2);
     */

    if (outLeftRectImgPort.getOutputCount()>0)
    {
        Mat rectLeft = this->stereo->getLRectified();

        ImageOf<PixelRgb>& rectLeftImage = outLeftRectImgPort.prepare();
        rectLeftImage.resize(rectLeft.cols,rectLeft.rows);

        Mat rectLeftImageMat=cvarrToMat((IplImage*)rectLeftImage.getIplImage());
        rectLeft.copyTo(rectLeftImageMat);

        outLeftRectImgPort.setEnvelope(stamp_left);
        outLeftRectImgPort.write();
    }

    if (outRightRectImgPort.getOutputCount()>0)
    {
        Mat rectRight = this->stereo->getRRectified();

        ImageOf<PixelRgb>& rectRightImage = outRightRectImgPort.prepare();
        rectRightImage.resize(rectRight.cols,rectRight.rows);

        Mat rectRightImageMat=cvarrToMat((IplImage*)rectRightImage.getIplImage());
        rectRight.copyTo(rectRightImageMat);

        outRightRectImgPort.setEnvelope(stamp_right);
        outRightRectImgPort.write();
    }

    if (outMatch.getOutputCount()>0)
    {
        /*Mat F= this->stereo->getFundamental();

        if(matchtmp.size()>0)
        {
            Mat m(matchtmp);
            vector<Vec3f> lines;
            cv::computeCorrespondEpilines(m,2,F,lines);
            for (cv::vector<cv::Vec3f>::const_iterator it = lines.begin(); it!=lines.end(); ++it)
            {
                cv::line(matMatches, cv::Point(0,-(*it)[2]/(*it)[1]), cv::Point(left->width,-((*it)[2] + (*it)[0]*left->width)/(*it)[1]),cv::Scalar(0,0,255));
            }        
        }*/

        Mat matches=this->stereo->drawMatches();
        cvtColor(matches,matches,CV_BGR2RGB);
        ImageOf<PixelBgr>& imgMatch=outMatch.prepare();
        imgMatch.resize(matches.cols,matches.rows);
        IplImage tmpR=matches;

        cvCopy(&tmpR,(IplImage*)imgMatch.getIplImage());
        outMatch.write();
    }

    if (outDisp.getOutputCount()>0)
    {
        outputDm = stereo->getDisparity();

        if (!outputDm.empty())
        {
            ImageOf<PixelMono> &outim = outDisp.prepare();
            if (doBLF)
            {
                Mat outputDfiltm;
                cv_extend::bilateralFilter(outputDm,outputDfiltm, sigmaColorBLF, sigmaSpaceBLF);
                IplImage outputDfilt = outputDfiltm;
                outim.wrapIplImage(&outputDfilt);
            } else
            {
                IplImage outputD = outputDm;
                outim.wrapIplImage(&outputD);
            }
            outDisp.write();
        }
    }

    ImageOf<PixelRgbFloat>& outcart=worldCartPort.prepare();
    ImageOf<PixelRgbFloat>& outcyl=worldCylPort.prepare();
    
    outcart.resize(left->width,left->height);
    outcyl.resize(left->width,left->height);

    fillWorld3D(outcart,outcyl);

    worldCartPort.write();
    worldCylPort.write();

    return true;
}


/******************************************************************************/
double SFM::getPeriod()
{
    // the updateModule() method gets synchronized
    // with camera input => no need for periodicity
    return 0.0;
}


/******************************************************************************/
bool SFM::loadExtrinsics(yarp::os::ResourceFinder& rf, Mat& Ro, Mat& To, yarp::sig::Vector& eyes)
{
    Bottle extrinsics=rf.findGroup("STEREO_DISPARITY");

    eyes.resize(3,0.0);
    if (Bottle *bEyes=extrinsics.find("eyes").asList())
    {
        size_t sz=std::min(eyes.length(),(size_t)bEyes->size());
        for (size_t i=0; i<sz; i++)
            eyes[i]=bEyes->get(i).asDouble();
    }

    cout<<"read eyes configuration = ("<<eyes.toString(3,3).c_str()<<")"<<endl;

    if (Bottle *pXo=extrinsics.find("HN").asList())
    {
        Ro=Mat::zeros(3,3,CV_64FC1);
        To=Mat::zeros(3,1,CV_64FC1);
        for (int i=0; i<(pXo->size()-4); i+=4)
        {
            Ro.at<double>(i/4,0)=pXo->get(i).asDouble();
            Ro.at<double>(i/4,1)=pXo->get(i+1).asDouble();
            Ro.at<double>(i/4,2)=pXo->get(i+2).asDouble();
            To.at<double>(i/4,0)=pXo->get(i+3).asDouble();
        }
    }
    else
        return false;

    return true;
}


/******************************************************************************/
bool SFM::loadIntrinsics(yarp::os::ResourceFinder &rf, Mat &KL, Mat &KR, Mat &DistL,
        Mat &DistR)
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

    return true;
}


/******************************************************************************/
bool SFM::updateExtrinsics(Mat& Rot, Mat& Tr, yarp::sig::Vector& eyes,
        const string& groupname)
{
    ofstream out;
    out.open(camCalibFile.c_str());
    if (out.is_open())
    {
        out << endl;
        out << "["+groupname+"]" << endl;
        out << "eyes (" << eyes.toString().c_str() << ")" << endl;
        out << "HN (" << Rot.at<double>(0,0) << " " << Rot.at<double>(0,1) << " " << Rot.at<double>(0,2) << " " << Tr.at<double>(0,0) << " "
                << Rot.at<double>(1,0) << " " << Rot.at<double>(1,1) << " " << Rot.at<double>(1,2) << " " << Tr.at<double>(1,0) << " "
                << Rot.at<double>(2,0) << " " << Rot.at<double>(2,1) << " " << Rot.at<double>(2,2) << " " << Tr.at<double>(2,0) << " "
                << 0.0                 << " " << 0.0                 << " " << 0.0                 << " " << 1.0                << ")"
                << endl;
        out.close();
        return true;
    }
    else
        return false;
}


/******************************************************************************/
void SFM::setDispParameters(bool _useBestDisp, int _uniquenessRatio,
        int _speckleWindowSize,int _speckleRange,
        int _numberOfDisparities, int _SADWindowSize,
        int _minDisparity, int _preFilterCap, int _disp12MaxDiff)
{
    this->mutexDisp.lock();
    this->useBestDisp=_useBestDisp;
    this->uniquenessRatio=_uniquenessRatio;
    this->speckleWindowSize=_speckleWindowSize;
    this->speckleRange=_speckleRange;
    this->numberOfDisparities=_numberOfDisparities;
    this->SADWindowSize=_SADWindowSize;
    this->minDisparity=_minDisparity;
    this->preFilterCap=_preFilterCap;
    this->disp12MaxDiff=_disp12MaxDiff;
    this->mutexDisp.unlock();

}


/******************************************************************************/
Point3f SFM::get3DPointsAndDisp(int u, int v, int& uR, int& vR, const string &drive)
{
    Point3f point(0.0f,0.0f,0.0f);
    if ((drive!="RIGHT") && (drive!="LEFT") && (drive!="ROOT"))
        return point;

    LockGuard lg(mutexDisp);

    // Mapping from Rectified Cameras to Original Cameras
    const Mat& Mapper=this->stereo->getMapperL();
    if (Mapper.empty())
        return point;

    float usign=Mapper.ptr<float>(v)[2*u];
    float vsign=Mapper.ptr<float>(v)[2*u+1];

    u=cvRound(usign);
    v=cvRound(vsign);

    const Mat& disp16m=this->stereo->getDisparity16();
    if (disp16m.empty() || (u<0) || (u>=disp16m.cols) || (v<0) || (v>=disp16m.rows))
        return point;

    const Mat& Q=this->stereo->getQ();
    IplImage disp16=disp16m;
    CvScalar scal=cvGet2D(&disp16,v,u);
    double disparity=scal.val[0]/16.0;

    uR=u-(int)disparity;
    vR=(int)v;

    Point2f orig=this->stereo->fromRectifiedToOriginal(uR,vR,RIGHT);
    uR=(int)orig.x;
    vR=(int)orig.y;

    float w=(float)(disparity*Q.at<double>(3,2)+Q.at<double>(3,3));
    point.x=(float)((usign+1)*Q.at<double>(0,0)+Q.at<double>(0,3));
    point.y=(float)((vsign+1)*Q.at<double>(1,1)+Q.at<double>(1,3));
    point.z=(float)Q.at<double>(2,3);

    point.x/=w;
    point.y/=w;
    point.z/=w;

    // discard points far more than 10 meters or with not valid disparity (<0)
    if ((point.z>10.0f) || (point.z<0.0f))
        return point;

    if (drive=="ROOT")
    {
        const Mat& RLrect=this->stereo->getRLrect().t();
        Mat Tfake=Mat::zeros(0,3,CV_64F);
        Mat P(4,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;
        P.at<double>(3,0)=1.0;

        Mat Hrect=buildRotTras(RLrect,Tfake);
        P=HL_root*Hrect*P;
        point.x=(float)(P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float)(P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float)(P.at<double>(2,0)/P.at<double>(3,0));
    }
    else if (drive=="LEFT")
    {
        Mat P(3,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;

        P=this->stereo->getRLrect().t()*P;
        point.x=(float)P.at<double>(0,0);
        point.y=(float)P.at<double>(1,0);
        point.z=(float)P.at<double>(2,0);
    }
    else if (drive=="RIGHT")
    {
        const Mat& Rright=this->stereo->getRotation();
        const Mat& Tright=this->stereo->getTranslation();
        const Mat& RRright=this->stereo->getRRrect().t();
        Mat TRright=Mat::zeros(0,3,CV_64F);

        Mat HRL=buildRotTras(Rright,Tright);
        Mat Hrect=buildRotTras(RRright,TRright);

        Mat P(4,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;
        P.at<double>(3,0)=1.0;

        P=Hrect*HRL*P;
        point.x=(float)(P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float)(P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float)(P.at<double>(2,0)/P.at<double>(3,0));
    }

    return point;
}


/******************************************************************************/
Point3f SFM::get3DPoints(int u, int v, const string &drive)
{
    Point3f point(0.0f,0.0f,0.0f);
    if ((drive!="RIGHT") && (drive!="LEFT") && (drive!="ROOT"))
        return point;

    LockGuard lg(mutexDisp);

    // Mapping from Rectified Cameras to Original Cameras
    const Mat& Mapper=this->stereo->getMapperL();
    if (Mapper.empty())
        return point;

    float usign=Mapper.ptr<float>(v)[2*u];
    float vsign=Mapper.ptr<float>(v)[2*u+1];

    u=cvRound(usign);
    v=cvRound(vsign);

    const Mat& disp16m=this->stereo->getDisparity16();
    if (disp16m.empty() || (u<0) || (u>=disp16m.cols) || (v<0) || (v>=disp16m.rows))
        return point;

    const Mat& Q=this->stereo->getQ();
    IplImage disp16=disp16m;
    CvScalar scal=cvGet2D(&disp16,v,u);
    double disparity=scal.val[0]/16.0;
    float w=(float)(disparity*Q.at<double>(3,2)+Q.at<double>(3,3));
    point.x=(float)((usign+1)*Q.at<double>(0,0)+Q.at<double>(0,3));
    point.y=(float)((vsign+1)*Q.at<double>(1,1)+Q.at<double>(1,3));
    point.z=(float)Q.at<double>(2,3);

    point.x/=w;
    point.y/=w;
    point.z/=w;

    // discard points far more than 10 meters or with not valid disparity (<0)
    if ((point.z>10.0f) || (point.z<0.0f))
        return point;

    if (drive=="ROOT")
    {
        const Mat& RLrect=this->stereo->getRLrect().t();
        Mat Tfake=Mat::zeros(0,3,CV_64F);
        Mat P(4,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;
        P.at<double>(3,0)=1.0;

        Mat Hrect=buildRotTras(RLrect,Tfake);
        P=HL_root*Hrect*P;
        point.x=(float)(P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float)(P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float)(P.at<double>(2,0)/P.at<double>(3,0));
    }
    else if (drive=="LEFT")
    {
        Mat P(3,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;

        P=this->stereo->getRLrect().t()*P;
        point.x=(float)P.at<double>(0,0);
        point.y=(float)P.at<double>(1,0);
        point.z=(float)P.at<double>(2,0);
    }
    else if (drive=="RIGHT")
    {
        const Mat& Rright=this->stereo->getRotation();
        const Mat& Tright=this->stereo->getTranslation();
        const Mat& RRright=this->stereo->getRRrect().t();
        Mat TRright=Mat::zeros(0,3,CV_64F);

        Mat HRL=buildRotTras(Rright,Tright);
        Mat Hrect=buildRotTras(RRright,TRright);

        Mat P(4,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;
        P.at<double>(3,0)=1.0;

        P=Hrect*HRL*P;
        point.x=(float)(P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float)(P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float)(P.at<double>(2,0)/P.at<double>(3,0));
    }

    return point;
}


/******************************************************************************/
Point3f SFM::get3DPointMatch(double u1, double v1, double u2, double v2, 
                             const string &drive)
{
    Point3f point(0.0f,0.0f,0.0f);
    if ((drive!="RIGHT") && (drive!="LEFT") && (drive!="ROOT"))
        return point;

    LockGuard lg(mutexDisp);
    // Mapping from Rectified Cameras to Original Cameras
    const Mat& MapperL=this->stereo->getMapperL();
    const Mat& MapperR=this->stereo->getMapperR();
    if (MapperL.empty() || MapperR.empty())
        return point;

    if ((cvRound(u1)<0) || (cvRound(u1)>=MapperL.cols) || (cvRound(v1)<0) || (cvRound(v1)>=MapperL.rows) ||
        (cvRound(u2)<0) || (cvRound(u2)>=MapperL.cols) || (cvRound(v2)<0) || (cvRound(v2)>=MapperL.rows))
        return point;

    float urect1=MapperL.ptr<float>(cvRound(v1))[2*cvRound(u1)];
    float vrect1=MapperL.ptr<float>(cvRound(v1))[2*cvRound(u1)+1];

    float urect2=MapperR.ptr<float>(cvRound(v2))[2*cvRound(u2)];
    float vrect2=MapperR.ptr<float>(cvRound(v2))[2*cvRound(u2)+1];

    const Mat& Q=this->stereo->getQ();
    double disparity=urect1-urect2;
    float w=(float)(disparity*Q.at<double>(3,2)+Q.at<double>(3,3));
    point.x=(float)((urect1+1)*Q.at<double>(0,0)+Q.at<double>(0,3));
    point.y=(float)((vrect1+1)*Q.at<double>(1,1)+Q.at<double>(1,3));
    point.z=(float)Q.at<double>(2,3);

    point.x/=w;
    point.y/=w;
    point.z/=w;

    if (drive=="ROOT")
    {
        const Mat& RLrect=this->stereo->getRLrect().t();
        Mat Tfake=Mat::zeros(0,3,CV_64F);
        Mat P(4,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;
        P.at<double>(3,0)=1.0;

        Mat Hrect=buildRotTras(RLrect,Tfake);
        P=HL_root*Hrect*P;
        point.x=(float)(P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float)(P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float)(P.at<double>(2,0)/P.at<double>(3,0));
    }
    else if (drive=="LEFT")
    {
        Mat P(3,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;

        P=this->stereo->getRLrect().t()*P;
        point.x=(float)P.at<double>(0,0);
        point.y=(float)P.at<double>(1,0);
        point.z=(float)P.at<double>(2,0);
    }
    else if (drive=="RIGHT")
    {
        const Mat& Rright=this->stereo->getRotation();
        const Mat& Tright=this->stereo->getTranslation();
        const Mat& RRright=this->stereo->getRRrect().t();
        Mat TRright=Mat::zeros(0,3,CV_64F);

        Mat HRL=buildRotTras(Rright,Tright);
        Mat Hrect=buildRotTras(RRright,TRright);

        Mat P(4,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;
        P.at<double>(3,0)=1.0;

        P=Hrect*HRL*P;
        point.x=(float)(P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float)(P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float)(P.at<double>(2,0)/P.at<double>(3,0));
    }

    return point;
}


/******************************************************************************/
Mat SFM::buildRotTras(const Mat& R, const Mat& T)
{     
    Mat A=Mat::eye(4,4,CV_64F);
    for (int i=0; i<R.rows; i++)
    {
        double* Mi=A.ptr<double>(i);
        const double* MRi=R.ptr<double>(i);
        for (int j=0; j<R.cols; j++)
            Mi[j]=MRi[j];
    }

    for (int i=0; i<T.rows; i++)
    {
        double* Mi=A.ptr<double>(i);
        const double* MRi=T.ptr<double>(i);
        Mi[3]=MRi[0];
    }

    return A;
}


/******************************************************************************/
Matrix SFM::getCameraHGazeCtrl(int camera)
{
    yarp::sig::Vector x_curr;
    yarp::sig::Vector o_curr;
    bool check=false;
    if(camera==LEFT)
        check=igaze->getLeftEyePose(x_curr, o_curr);
    else
        check=igaze->getRightEyePose(x_curr, o_curr);

    if(!check)
    {
        Matrix H_curr(4, 4);
        return H_curr;
    }

    Matrix R_curr=axis2dcm(o_curr);
    Matrix H_curr=R_curr;
    H_curr.setSubcol(x_curr,0,3);

    if(camera==LEFT)
    {
        mutexDisp.lock();
        convert(H_curr,HL_root);
        mutexDisp.unlock();
    }
    else if(camera==RIGHT)
    {
        mutexDisp.lock();
        convert(H_curr,HR_root);
        mutexDisp.unlock();
    }

    return H_curr;
}


/******************************************************************************/
void SFM::convert(Matrix& matrix, Mat& mat)
{
    mat=cv::Mat(matrix.rows(),matrix.cols(),CV_64FC1);
    for(int i=0; i<matrix.rows(); i++)
        for(int j=0; j<matrix.cols(); j++)
            mat.at<double>(i,j)=matrix(i,j);
}


/******************************************************************************/
void SFM::convert(Mat& mat, Matrix& matrix)
{
    matrix.resize(mat.rows,mat.cols);
    for(int i=0; i<mat.rows; i++)
        for(int j=0; j<mat.cols; j++)
            matrix(i,j)=mat.at<double>(i,j);
}


/******************************************************************************/
bool SFM::respond(const Bottle& command, Bottle& reply) 
{
    if(command.size()==0)
        return false;

    if (command.get(0).asString()=="quit") {
        cout << "closing..." << endl;
        return false;
    }

    if (command.get(0).asString()=="help") {
        reply.addVocab(Vocab::encode("many"));
        reply.addString("Available commands are:");
        reply.addString("- [calibrate]: It recomputes the camera positions once.");
        reply.addString("- [save]: It saves the current camera positions and uses it when the module starts.");
        reply.addString("- [getH]: It returns the calibrated stereo matrix.");
        reply.addString("- [setNumDisp NumOfDisparities]: It sets the expected number of disparity (in pixel). Values must be divisible by 32. ");
        reply.addString("- [Point x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the LEFT eye.");
        reply.addString("- [x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z ur vr computed using the depth map wrt the the ROOT reference system.(ur vr) is the corresponding pixel in the Right image. ");
        reply.addString("- [Left x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the LEFT eye. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0). ");
        reply.addString("- [Right x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the RIGHT eye. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).");
        reply.addString("- [Root x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the ROOT reference system. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).");
        reply.addString("- [Rect tlx tly w h step]: Given the pixels in the rectangle defined by {(tlx,tly) (tlx+w,tly+h)} (parsed by columns), the response contains the corresponding 3D points in the ROOT frame. The optional parameter step defines the sampling quantum; by default step=1.");
        reply.addString("- [Points u_1 v_1 ... u_n v_n]: Given a list of n pixels, the response contains the corresponding 3D points in the ROOT frame.");
        reply.addString("- [Flood3D x y dist]: Perform 3D flood-fill on the seed point (x,y), returning the following info: [u_1 v_1 x_1 y_1 z_1 ...]. The optional parameter dist expressed in meters regulates the fill (by default = 0.004).");
        reply.addString("- [uL_1 vL_1 uR_1 vR_1 ... uL_n vL_n uR_n vR_n]: Given n quadruples uL_i vL_i uR_i vR_i, where uL_i vL_i are the pixel coordinates in the Left image and uR_i vR_i are the coordinates of the matched pixel in the Right image, the response is a set of 3D points (X1 Y1 Z1 ... Xn Yn Zn) wrt the ROOT reference system.");
        reply.addString("- [cart2stereo X Y Z]: Given a world point X Y Z wrt to ROOT reference frame the response is the projection (uL vL uR vR) in the Left and Right images.");
        reply.addString("- [doBLF flag]: activate Bilateral filter for flag = true, and skip it for flag = false.");
        reply.addString("- [bilatfilt sigmaColor sigmaSpace]: Set the parameters for the bilateral filer (default sigmaColor = 10.0, sigmaSpace = 10.0 .");
        reply.addString("For more details on the commands, check the module's documentation");
        return true;
    }

    if (command.get(0).asString()=="calibrate")
    {
        mutexRecalibration.lock();
        numberOfTrials=0;
        doSFM=true;
        mutexRecalibration.unlock();

        calibEndEvent.reset();
        calibEndEvent.wait();

        if (calibUpdated)
        {
            R0=this->stereo->getRotation();
            T0=this->stereo->getTranslation();
            eyes0=eyes;

            reply.addString("ACK");
        }
        else
            reply.addString("Calibration failed after 5 trials.. Please show a non planar scene.");

        return true;
    }

    if (command.get(0).asString()=="save")
    {
        updateExtrinsics(R0,T0,eyes0,"STEREO_DISPARITY");
        reply.addString("ACK");
        return true;
    }

    if (command.get(0).asString()=="getH")
    {
        Mat RT0=buildRotTras(R0,T0);
        Matrix H0; convert(RT0,H0);

        reply.read(H0);
        return true;
    }

    if (command.get(0).asString()=="setNumDisp")
    {
        int dispNum=command.get(1).asInt();
        if(dispNum%32==0)
        {
            this->numberOfDisparities=dispNum;
            this->setDispParameters(useBestDisp,uniquenessRatio,speckleWindowSize,speckleRange,numberOfDisparities,SADWindowSize,minDisparity,preFilterCap,disp12MaxDiff);
            reply.addString("ACK");
            return true;
        }
        else
        {
            reply.addString("Num Disparity must be divisible by 32");
            return true;
        }
    }

    if (command.get(0).asString()=="setMinDisp")
    {
        int dispNum=command.get(1).asInt();
        this->minDisparity=dispNum;
        this->setDispParameters(useBestDisp,uniquenessRatio,speckleWindowSize,
                speckleRange,numberOfDisparities,SADWindowSize,
                minDisparity,preFilterCap,disp12MaxDiff);
        reply.addString("ACK");
        return true;
    }

    if (command.get(0).asString()=="set" && command.size()==10)
    {
        bool bestDisp=command.get(1).asInt() ? true : false;
        int uniquenessRatio=command.get(2).asInt();
        int speckleWindowSize=command.get(3).asInt();
        int speckleRange=command.get(4).asInt();
        int numberOfDisparities=command.get(5).asInt();
        int SADWindowSize=command.get(6).asInt();
        int minDisparity=command.get(7).asInt();
        int preFilterCap=command.get(8).asInt();
        int disp12MaxDiff=command.get(9).asInt();

        this->setDispParameters(bestDisp,uniquenessRatio,speckleWindowSize,
                                speckleRange,numberOfDisparities,SADWindowSize,
                                minDisparity,preFilterCap,disp12MaxDiff);
        reply.addString("ACK");
    }
    else if (command.get(0).asString()=="Point" || command.get(0).asString()=="Left" )
    {
        int u = command.get(1).asInt();
        int v = command.get(2).asInt();
        Point3f point = this->get3DPoints(u,v);
        reply.addDouble(point.x);
        reply.addDouble(point.y);
        reply.addDouble(point.z);
    }
    else if (!command.get(0).isString() && command.size()==2)
    {
        int u = command.get(0).asInt();
        int v = command.get(1).asInt();
        int uR,vR;
        Point3f point = this->get3DPointsAndDisp(u,v,uR,vR,"ROOT");
        reply.addDouble(point.x);
        reply.addDouble(point.y);
        reply.addDouble(point.z);
        reply.addInt(uR);
        reply.addInt(vR);
    }
    else if (command.get(0).asString()=="Right")
    {
        int u = command.get(1).asInt();
        int v = command.get(2).asInt();
        Point3f point = this->get3DPoints(u,v,"RIGHT");
        reply.addDouble(point.x);
        reply.addDouble(point.y);
        reply.addDouble(point.z);
    }
    else if (command.get(0).asString()=="Root")
    {
        int u = command.get(1).asInt();
        int v = command.get(2).asInt();
        Point3f point = this->get3DPoints(u,v,"ROOT");
        reply.addDouble(point.x);
        reply.addDouble(point.y);
        reply.addDouble(point.z);
    }
    else if (command.get(0).asString()=="Rect")
    {
        int tl_u = command.get(1).asInt();
        int tl_v = command.get(2).asInt();
        int br_u = tl_u+command.get(3).asInt();
        int br_v = tl_v+command.get(4).asInt();

        int step = 1;
        if (command.size()>=6)
            step=command.get(5).asInt();

        for (int u=tl_u; u<br_u; u+=step)
        {
            for (int v=tl_v; v<br_v; v+=step)
            {
                Point3f point=this->get3DPoints(u,v,"ROOT");
                reply.addDouble(point.x);
                reply.addDouble(point.y);
                reply.addDouble(point.z);
            }
        }
    }
    else if (command.get(0).asString()=="Points")
    {
        for (int cnt=1; cnt<command.size()-1; cnt+=2)
        {
            int u=command.get(cnt).asInt();
            int v=command.get(cnt+1).asInt();
            Point3f point=this->get3DPoints(u,v,"ROOT");
            reply.addDouble(point.x);
            reply.addDouble(point.y);
            reply.addDouble(point.z);
        }
    }
    else if (command.get(0).asString()=="Flood3D")
    {
        cv::Point seed(command.get(1).asInt(),
                       command.get(2).asInt());

        double dist=0.004;
        if (command.size()>=4)
            dist=command.get(3).asDouble();
        
        Point3f p=get3DPoints(seed.x,seed.y,"ROOT");
        if (cv::norm(p)>0.0)
        {
            reply.addInt(seed.x);
            reply.addInt(seed.y);
            reply.addDouble(p.x);
            reply.addDouble(p.y);
            reply.addDouble(p.z);

            set<int> visited;
            visited.insert(seed.x*outputDm.cols+seed.y);

            floodFill(seed,p,dist,visited,reply);
        }
        else
            reply.addString("NACK");
    }
    else if (command.get(0).asString()=="cart2stereo")
    {
        double x = command.get(1).asDouble();
        double y = command.get(2).asDouble();
        double z = command.get(3).asDouble();

        Point2f pointL = this->projectPoint("left",x,y,z);
        Point2f pointR = this->projectPoint("right",x,y,z);

        reply.addDouble(pointL.x);
        reply.addDouble(pointL.y);
        reply.addDouble(pointR.x);
        reply.addDouble(pointR.y);
    }
    else if (command.get(0).asString()=="bilatfilt" && command.size()==3)
    {
        if (!doBLF){
            doBLF = true;
            reply.addString("Bilateral filter activated.");
        }
        sigmaColorBLF = command.get(1).asDouble();
        sigmaSpaceBLF = command.get(2).asDouble();
        reply.addString("BLF sigmaColor ");
        reply.addDouble(sigmaColorBLF);
        reply.addString("BLF sigmaSpace ");
        reply.addDouble(sigmaSpaceBLF);
    }
    else if (command.get(0).asString()=="doBLF")
    {
        bool onoffBLF = command.get(1).asBool();
        if (onoffBLF == false ){     // turn OFF Bilateral Filtering
            if (doBLF == true){
                doBLF = false;
                reply.addString("Bilateral Filter OFF");
            } else {
                reply.addString("Bilateral Filter already OFF");
            }

        } else {                    // turn ON Bilateral Filtering
            if (doBLF == true){
                reply.addString("Bilateral Filter Already Running");
            } else {                                     // Set any different from 0 to activate bilateral filter.
                doBLF = true;
                reply.addString("Bilateral Filter ON");
            }
        }
        reply.addDouble(sigmaColorBLF);
        reply.addDouble(sigmaSpaceBLF);
    }
    else if(command.size()>0 && command.size()%4==0)
    {
        for (int i=0; i<command.size(); i+=4)
        {
            double ul = command.get(i).asDouble();
            double vl = command.get(i+1).asDouble();
            double ur = command.get(i+2).asDouble();
            double vr = command.get(i+3).asDouble();

            Point3f point= this->get3DPointMatch(ul,vl,ur,vr,"ROOT");
            reply.addDouble(point.x);
            reply.addDouble(point.y);
            reply.addDouble(point.z);
        }
    }
    else
        reply.addString("NACK");

    return true;
}


/******************************************************************************/
Point2f SFM::projectPoint(const string &camera, double x, double y, double z)
{
    Point3f point3D;
    point3D.x=(float)x;
    point3D.y=(float)y;
    point3D.z=(float)z;

    vector<Point3f> points3D;

    points3D.push_back(point3D);

    vector<Point2f> response;

    mutexDisp.lock();

    if(camera=="left")
        response=this->stereo->projectPoints3D("left",points3D,HL_root);
    else
        response=this->stereo->projectPoints3D("right",points3D,HL_root);

    mutexDisp.unlock();
    return response[0];
}


/******************************************************************************/
void SFM::fillWorld3D(ImageOf<PixelRgbFloat> &worldCartImg,
                      ImageOf<PixelRgbFloat> &worldCylImg)
{
    mutexDisp.lock();
    const Mat& Mapper=this->stereo->getMapperL();
    const Mat& disp16m=this->stereo->getDisparity16();
    IplImage disp16=disp16m;
    const Mat& Q=this->stereo->getQ();
    const Mat& RLrect=this->stereo->getRLrect().t();
    mutexDisp.unlock();
    
    worldCartImg.zero(); worldCylImg.zero();
    if (Mapper.empty() || disp16m.empty() ||
        (worldCartImg.width()!=worldCylImg.width()) ||
        (worldCartImg.height()!=worldCylImg.height()))
        return;

    Mat Tfake=Mat::zeros(0,3,CV_64F);
    Mat Hrect=buildRotTras(RLrect,Tfake);
    Hrect=HL_root*Hrect;

    Mat P(4,1,CV_64FC1);
    P.at<double>(3,0)=1.0;
    double &x=P.at<double>(0,0);
    double &y=P.at<double>(1,0);
    double &z=P.at<double>(2,0);

    for (int v=0; v<worldCartImg.height(); v++)
    {
        for (int u=0; u<worldCartImg.width(); u++)
        {
            float usign=Mapper.ptr<float>(v)[2*u];
            float vsign=Mapper.ptr<float>(v)[2*u+1];

            int u_=cvRound(usign); int v_=cvRound(vsign);
            if ((u_<0) || (u_>=disp16m.cols) || (v_<0) || (v_>=disp16m.rows))
                continue;

            x=(usign+1)*Q.at<double>(0,0)+Q.at<double>(0,3);
            y=(vsign+1)*Q.at<double>(1,1)+Q.at<double>(1,3);
            z=Q.at<double>(2,3);

            CvScalar scal=cvGet2D(&disp16,v_,u_);
            double disparity=scal.val[0]/16.0;
            double w=disparity*Q.at<double>(3,2)+Q.at<double>(3,3);
            x/=w; y/=w; z/=w;

            if ((z>10.0) || (z<0.0))
                continue;
            
            P=Hrect*P;

            PixelRgbFloat &pxCart=worldCartImg.pixel(u,v);
            pxCart.r=(float)x;
            pxCart.g=(float)y;
            pxCart.b=(float)z;

            PixelRgbFloat &pxCyl=worldCylImg.pixel(u,v);
            pxCyl.r=(float)sqrt(x*x+y*y);
            pxCyl.g=(float)atan2(y,x);
            pxCyl.b=(float)z;
        }
    }
}


/******************************************************************************/
void SFM::floodFill(const Point &seed, const Point3f &p0, const double dist, 
                    set<int> &visited, Bottle &res)
{
    for (int x=seed.x-1; x<=seed.x+1; x++)
    {
        for (int y=seed.y-1; y<=seed.y+1; y++)
        {
            if ((x<0)||(y<0)||(x>outputDm.cols)||(y>outputDm.rows))
                continue;

            int idx=x*outputDm.cols+y;
            set<int>::iterator el=visited.find(idx);
            if (el==visited.end())
            {
                visited.insert(idx);
                Point3f p=get3DPoints(x,y,"ROOT");
                if ((cv::norm(p)>0.0) && (cv::norm(p-p0)<=dist))
                {
                    res.addInt(x);
                    res.addInt(y);
                    res.addDouble(p.x);
                    res.addDouble(p.y);
                    res.addDouble(p.z);

                    floodFill(Point(x,y),p,dist,visited,res);
                }
            }
        }
    }
}


/******************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return 1;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("icubEyes.ini");
    rf.setDefaultContext("cameraCalibration");
    rf.configure(argc,argv);

    SFM mod;

    return mod.runModule(rf);
}


