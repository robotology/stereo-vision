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

#include "iCub/stereoVision/disparityThread.h"

bool DisparityThread::loadExtrinsics(yarp::os::ResourceFinder& rf, Mat& Ro, Mat& To, yarp::sig::Vector& eyes)
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


DisparityThread::DisparityThread(const string &name, yarp::os::ResourceFinder &rf,
                                 bool useHorn, bool updateCamera, bool rectify) : RateThread(10)
{
    moduleName=name;
    Bottle pars=rf.findGroup("STEREO_DISPARITY");    
    robotName=pars.check("robotName",Value("icub")).asString().c_str();    

    if (Bottle *pXo=pars.find("QL").asList()) 
    {
        QL.resize(pXo->size());
        for (int i=0; i<(pXo->size()); i++) 
            QL[i]=pXo->get(i).asDouble();
    }
    
    if (Bottle *pXo=pars.find("QR").asList()) 
    {
        QR.resize(pXo->size());
        for (int i=0; i<(pXo->size()); i++) 
            QR[i]=pXo->get(i).asDouble();
    }

    int calib=rf.check("useCalibrated",Value(1)).asInt();
    this->useCalibrated=(calib!=0);
    this->useHorn=useHorn;
    Mat KL, KR, DistL, DistR, R, T;
    success=loadStereoParameters(rf,KL,KR,DistL,DistR,R,T);

    ResourceFinder localCalibration;
    localCalibration.setContext("cameraCalibration");
    localCalibration.setDefaultConfigFile("SFM.ini");
    localCalibration.setVerbose();
    localCalibration.configure(0,NULL);

    loadExtrinsics(localCalibration,R0,T0,eyes0);
    eyes.resize(eyes0.length(),0.0);

    this->stereo=new StereoCamera(rectify);
    if (success)
    {
        stereo->setIntrinsics(KL,KR,DistL,DistR);
        this->HL_root=Mat::zeros(4,4,CV_64F);

        if (R0.empty() || T0.empty())
        {
            R0=R;
            T0=T;
        }

        stereo->setRotation(R0,0);
        stereo->setTranslation(T0,0);

        if (useCalibrated)
        {
            Mat KL=this->stereo->getKleft();
            Mat KR=this->stereo->getKright();
            Mat zeroDist=Mat::zeros(1,8,CV_64FC1);
            this->stereo->setIntrinsics(KL,KR,zeroDist,zeroDist);
        }
        
        printf("Disparity Thread has started...\n");
    }

    this->widthInit=320;
    this->useBestDisp=true;
    this->uniquenessRatio=15;
    this->speckleWindowSize=50;
    this->speckleRange=16;
    this->numberOfDisparities=96;
    this->SADWindowSize=7;
    this->minDisparity=0;
    this->preFilterCap=63;
    this->disp12MaxDiff=0;

    this->init=true;
    this->work=false;
    this->done=false;
    this->updateOnce=false;
    this->updateCamera=updateCamera;

#ifdef USING_GPU
    utils=new Utilities();
    utils->initSIFT_GPU();
#endif
}


bool DisparityThread::isOpen()
{
    return success;
}


void DisparityThread::updateViaKinematics(const yarp::sig::Vector& deyes)
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


void DisparityThread::updateViaGazeCtrl(const bool update)
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


void DisparityThread::run() 
{
    if (!success)
    {
        printf("Error. Cannot load camera parameters... Check your config file \n");
        return;
    }

    if (work)
    {
        // read encoders
        posHead->getEncoder(nHeadAxis-3,&eyes[0]);
        posHead->getEncoder(nHeadAxis-2,&eyes[1]);
        posHead->getEncoder(nHeadAxis-1,&eyes[2]);

        updateViaKinematics(eyes-eyes0);
        updateViaGazeCtrl(false);
        
        mutexDisp.lock();
        if (updateCamera || updateOnce)
        {
        #ifdef USING_GPU
            Mat leftMat=this->stereo->getImLeft();
            Mat rightMat=this->stereo->getImRight();
            IplImage left=leftMat;
            IplImage right=rightMat;
            this->stereo->setImages(&left,&right);
            utils->extractMatch_GPU( leftMat, rightMat);
            vector<Point2f> leftM,rightM;
            utils->getMatches(leftM,rightM);
            this->stereo->setMatches(leftM,rightM);
         #else
            this->stereo->findMatch(false,15,10.0);      
         #endif

            this->stereo->estimateEssential();
            if (this->stereo->essentialDecomposition())
            {
                R0=this->stereo->getRotation();
                T0=this->stereo->getTranslation();
                eyes0=eyes;

                if (updateOnce)
                    updateOnce=false;
            }
        }

        // Compute Disparity
        this->stereo->computeDisparity(this->useBestDisp, this->uniquenessRatio, this->speckleWindowSize,
                                       this->speckleRange, this->numberOfDisparities, this->SADWindowSize,
                                       this->minDisparity, this->preFilterCap, this->disp12MaxDiff);
        mutexDisp.unlock();
        work=false;
        done=true;
        this->suspend();
    }
}


void DisparityThread::setImages(Mat &left, Mat &right) 
{
    IplImage l=left;
    IplImage r=right;

    stereo->setImages(&l,&r);

    if (l.width!=widthInit)
    {
        this->numberOfDisparities=(l.width<=320)?96:128;
        widthInit=l.width;
    }

    this->done=false;
    this->work=true;
    this->resume();
}


void DisparityThread::getDisparity(Mat &Disp)
{
    mutexDisp.lock();
    Mat tmp=stereo->getDisparity();
    Disp=tmp.clone();
    mutexDisp.unlock();
}


void DisparityThread::getDisparityFloat(Mat &Disp) 
{
    mutexDisp.lock();
    Mat tmp=stereo->getDisparity16();
    Disp= tmp.clone();
    mutexDisp.unlock();
}


void DisparityThread::getQMat(Mat &Q) 
{
    mutexDisp.lock();
    Mat tmp=stereo->getQ();
    Q= tmp.clone();
    mutexDisp.unlock();
}


void DisparityThread::getMapper(Mat &Mapper) 
{
    mutexDisp.lock();
    Mat tmp=stereo->getMapperL();
    Mapper= tmp.clone();
    mutexDisp.unlock();
}


void DisparityThread::getRectMatrix(Mat &RL) 
{
    mutexDisp.lock();
    Mat tmp=stereo->getRLrect();
    RL= tmp.clone();
    mutexDisp.unlock();
}


bool DisparityThread::threadInit() 
{
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    option.put("local",("/"+moduleName+"/gaze").c_str());
    if (gazeCtrl.open(option))
    {
        mutexDisp.lock();
        gazeCtrl.view(igaze);
        getCameraHGazeCtrl(LEFT);
        getCameraHGazeCtrl(RIGHT);
        mutexDisp.unlock();
    }
    else
    {
        cout<<"Devices not available"<<endl;
        success=false;
        return false;
    }

    Property optHead;
    optHead.put("device","remote_controlboard");
    optHead.put("remote",("/"+robotName+"/head").c_str());
    optHead.put("local",("/"+moduleName+"/head").c_str());
    if (polyHead.open(optHead))
    {
        polyHead.view(posHead);
        polyHead.view(HctrlLim);
        posHead->getAxes(&nHeadAxis);
    }
    else
    {
        cout<<"Devices not available"<<endl;
        success=false;
        return false;
    }

    Property optTorso;
    optTorso.put("device","remote_controlboard");
    optTorso.put("remote",("/"+robotName+"/torso").c_str());
    optTorso.put("local",("/"+moduleName+"/torso").c_str());
    if (polyTorso.open(optTorso))
    {
        polyTorso.view(posTorso);
        polyTorso.view(TctrlLim);
    }
    else
    {
        cout<<"Devices not available"<<endl;
        success=false;
        return false;
    }

    Bottle p;
    igaze->getInfo(p);
    int vHead=(int)p.check(("head_version"),Value(1.0)).asDouble();
    stringstream headType;
    headType << "v";
    headType << vHead;

    LeyeKin=new iCubEye(("left_"+headType.str()).c_str());
    ReyeKin=new iCubEye(("right_"+headType.str()).c_str());
    LeyeKin->releaseLink(0);
    LeyeKin->releaseLink(1);
    LeyeKin->releaseLink(2);
    ReyeKin->releaseLink(0);
    ReyeKin->releaseLink(1);
    ReyeKin->releaseLink(2);
    deque<IControlLimits*> lim;
    lim.push_back(TctrlLim);
    lim.push_back(HctrlLim);
    LeyeKin->alignJointsBounds(lim);
    ReyeKin->alignJointsBounds(lim);

    return true;
}


void DisparityThread::threadRelease() 
{
    delete stereo;

    gazeCtrl.close();

    delete LeyeKin;
    delete ReyeKin;

    if (polyHead.isValid())
        polyHead.close();

    if (polyTorso.isValid())
        polyTorso.close();

    #ifdef USING_GPU
        delete utils;
    #endif

    printf("Disparity Thread Closed... \n");
}


bool DisparityThread::checkDone() 
{
    return done;
}


void DisparityThread::stopUpdate() 
{
    updateCamera=false;
}


void DisparityThread::startUpdate() 
{
    updateCamera=true;
}


void DisparityThread::updateCamerasOnce() 
{
    updateCamera=false;
    updateOnce=true;
}


void DisparityThread::triangulate(Point2f &pixel,Point3f &point) 
{
    mutexDisp.lock();
    Mat disparity=stereo->getDisparity16();
    Mat Q= stereo->getQ();
    Mat Mapper=stereo->getMapperL();
    Mat RLrect=stereo->getRLrect();

    int u=(int) pixel.x; 
    int v=(int) pixel.y;

    // Mapping from Rectified Cameras to Original Cameras
    if(Mapper.empty()) 
    {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        this->mutexDisp.unlock();
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
        this->mutexDisp.unlock();
        return;
    }
    else 
    {
        CvScalar scal=cvGet2D(&disp16,v,u);
        double dispVal=scal.val[0]/16.0;
        float w= (float) ((float) dispVal*Q.at<double>(3,2)) + ((float)Q.at<double>(3,3));
        point.x= (float)((float) (usign+1)*Q.at<double>(0,0)) + ((float) Q.at<double>(0,3));
        point.y=(float)((float) (vsign+1)*Q.at<double>(1,1)) + ((float) Q.at<double>(1,3));
        point.z=(float) Q.at<double>(2,3);

        point.x=point.x/w;
        point.y=point.y/w;
        point.z=point.z/w;
    }
    // discard points far more than 10 meters or with not valid disparity (<0)
    if(point.z>10 || point.z<0) 
    {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        this->mutexDisp.unlock();
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
        Hrect=buildRotTras(RLrecttemp,Tfake);
      
        P=HL_root*Hrect*P;
        point.x=(float) ((float) P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float) ((float) P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float) ((float) P.at<double>(2,0)/P.at<double>(3,0));
    }
    this->mutexDisp.unlock();
    return;
}


Mat DisparityThread::buildRotTras(Mat& R, Mat& T)
{     
    Mat A=Mat::eye(4,4,CV_64F);
    for (int i=0; i<R.rows; i++)
    {
        double* Mi=A.ptr<double>(i);
        double* MRi=R.ptr<double>(i);
        for (int j=0; j<R.cols; j++)
            Mi[j]=MRi[j];
    }

    for (int i=0; i<T.rows; i++)
    {
        double* Mi=A.ptr<double>(i);
        double* MRi=T.ptr<double>(i);
        Mi[3]=MRi[0];
    }

    return A;
}


bool DisparityThread::loadStereoParameters(yarp::os::ResourceFinder &rf, Mat &KL,
                                           Mat &KR, Mat &DistL, Mat &DistR, Mat &Ro, Mat &To)
{

    Bottle left=rf.findGroup("CAMERA_CALIBRATION_LEFT");
    if (!left.check("fx") || !left.check("fy") || !left.check("cx") || !left.check("cy"))
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
    To=Mat::zeros(3,1,CV_64FC1);

    /*Bottle extrinsics=rf.findGroup("STEREO_DISPARITY");
    if (Bottle *pXo=extrinsics.find("HN").asList()) {
        for (int i=0; i<(pXo->size()-4); i+=4) {
            Ro.at<double>(i/4,0)=pXo->get(i).asDouble();
            Ro.at<double>(i/4,1)=pXo->get(i+1).asDouble();
            Ro.at<double>(i/4,2)=pXo->get(i+2).asDouble();
            T.at<double>(i/4,0)=pXo->get(i+3).asDouble();
        }
    }
    else
        return false;*/

    return true;
}


void DisparityThread::printMatrixYarp(Matrix &A)
{
    cout << endl;
    for (int i=0; i<A.rows(); i++)
    {
        for (int j=0; j<A.cols(); j++)
            cout<<A(i,j)<<" ";
        cout<<endl;
    }
    cout << endl;
}


void DisparityThread::convert(Matrix& matrix, Mat& mat)
{
    mat=cv::Mat(matrix.rows(),matrix.cols(),CV_64FC1);
    for(int i=0; i<matrix.rows(); i++)
        for(int j=0; j<matrix.cols(); j++)
            mat.at<double>(i,j)=matrix(i,j);
}


void DisparityThread::convert(Mat& mat, Matrix& matrix)
{
    matrix.resize(mat.rows,mat.cols);
    for(int i=0; i<mat.rows; i++)
        for(int j=0; j<mat.cols; j++)
            matrix(i,j)=mat.at<double>(i,j);
}


void DisparityThread::getRootTransformation(Mat & Trans,int eye)
{
    mutexDisp.lock();

    if(eye==LEFT)
       Trans= HL_root.clone();
    else
       Trans= HR_root.clone();

    mutexDisp.unlock();

}


Matrix DisparityThread::getCameraH(yarp::sig::Vector &head_angles,
                                   yarp::sig::Vector &torso_angles, iCubEye *eyeKin, int camera)
{

    yarp::sig::Vector q(torso_angles.size()+head_angles.size());

    //torso angles are inverted
    for(int i=0; i<torso_angles.size(); i++)
        q[i]=torso_angles[torso_angles.size()-i-1];

    for(int i=0; i<head_angles.size()-2; i++)
        q[i+torso_angles.size()]=head_angles[i];

    // Vs=(L+R)/2  Vg=L-R
    q[7]=head_angles[4]+(0.5-(camera))*head_angles[5];

    q=CTRL_DEG2RAD*q;


    Matrix H_curr=eyeKin->getH(q);

    q=eyeKin->getAng();
    


    if(camera==LEFT)
    {
        /*q=q*CTRL_RAD2DEG;
        cout << " Q Chain" << endl;
        cout << q.toString(5,5).c_str() << endl;*/
        convert(H_curr,HL_root);
    }
    else if(camera==RIGHT)
    {
        convert(H_curr,HR_root);
    }


    return H_curr;
}


Matrix DisparityThread::getCameraHGazeCtrl(int camera)
{
    yarp::sig::Vector x_curr;
    yarp::sig::Vector o_curr;
    bool check=false;
    if(camera==LEFT)
        check=igaze->getLeftEyePose(x_curr, o_curr);
    else
        check=igaze->getRightEyePose(x_curr, o_curr);

    if (!check)
    {
        Matrix H_curr(4, 4);
        return H_curr;    
    }

    Matrix R_curr=axis2dcm(o_curr);

    Matrix H_curr(4, 4);
    H_curr=R_curr;
    H_curr(0,3)=x_curr[0];
    H_curr(1,3)=x_curr[1];
    H_curr(2,3)=x_curr[2];

    if (camera==LEFT)
        convert(H_curr,HL_root);
    else if (camera==RIGHT)
        convert(H_curr,HR_root);

    return H_curr;
}


void DisparityThread::onStop()
{
    this->work=false;
    this->done=true;
}


void DisparityThread::setDispParameters(bool _useBestDisp, int _uniquenessRatio, int _speckleWindowSize,
                                        int _speckleRange, int _numberOfDisparities, int _SADWindowSize,
                                        int _minDisparity, int _preFilterCap, int _disp12MaxDiff)
{
    mutexDisp.lock();

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


Point3f DisparityThread::get3DPointMatch(double u1, double v1, double u2, double v2, string drive)
{
    Point3f point;
    if(drive!="RIGHT" && drive !="LEFT" && drive!="ROOT") {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        return point;
    }

    mutexDisp.lock();

    // Mapping from Rectified Cameras to Original Cameras
    Mat MapperL=this->stereo->getMapperL();
    Mat MapperR=this->stereo->getMapperR();

    if(MapperL.empty() || MapperR.empty()) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;

        this->mutexDisp.unlock();
        return point;
    }

    if(cvRound(u1)<0 || cvRound(u1)>=MapperL.cols || cvRound(v1)<0 || cvRound(v1)>=MapperL.rows) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        this->mutexDisp.unlock();
        return point;
    }
    
    if(cvRound(u2)<0 || cvRound(u2)>=MapperL.cols || cvRound(v2)<0 || cvRound(v2)>=MapperL.rows) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        this->mutexDisp.unlock();
        return point;
    }

    float urect1=MapperL.ptr<float>(cvRound(v1))[2*cvRound(u1)];
    float vrect1=MapperL.ptr<float>(cvRound(v1))[2*cvRound(u1)+1]; 

    float urect2=MapperR.ptr<float>(cvRound(v2))[2*cvRound(u2)];
    float vrect2=MapperR.ptr<float>(cvRound(v2))[2*cvRound(u2)+1]; 

    Mat Q=this->stereo->getQ();
    double disparity=urect1-urect2;
    float w= (float) ((float) disparity*Q.at<double>(3,2)) + ((float)Q.at<double>(3,3));
    point.x= (float)((float) (urect1+1)*Q.at<double>(0,0)) + ((float) Q.at<double>(0,3));
    point.y=(float)((float) (vrect1+1)*Q.at<double>(1,1)) + ((float) Q.at<double>(1,3));
    point.z=(float) Q.at<double>(2,3);

    point.x=point.x/w;
    point.y=point.y/w;
    point.z=point.z/w;

   if(drive=="LEFT") {
        Mat P(3,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;

        P=this->stereo->getRLrect().t()*P;

        point.x=(float) P.at<double>(0,0);
        point.y=(float) P.at<double>(1,0);
        point.z=(float) P.at<double>(2,0);
    }

    if(drive=="RIGHT") {
        Mat Rright = this->stereo->getRotation();
        Mat Tright = this->stereo->getTranslation();
        Mat RRright = this->stereo->getRRrect().t();
        Mat TRright = Mat::zeros(0,3,CV_64F);

        Mat RLrect=stereo->getRLrect();
        Mat RLrecttemp=RLrect.t();
        Mat Tfake = Mat::zeros(0,3,CV_64F);
        Mat Hrect = Mat::eye(4, 4, CV_64F);
        Hrect=buildRotTras(RLrecttemp,Tfake);

        Mat HRL;
        HRL=buildRotTras(Rright,Tright);
        Hrect=buildRotTras(RRright,TRright);

        Mat P(4,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;
        P.at<double>(3,0)=1;
       
        P=Hrect*HRL*P;
        point.x=(float) ((float) P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float) ((float) P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float) ((float) P.at<double>(2,0)/P.at<double>(3,0));
    }

    if(drive=="ROOT") {
        Mat RLrect=this->stereo->getRLrect().t();
        Mat Tfake = Mat::zeros(0,3,CV_64F);
        Mat P(4,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;
        P.at<double>(3,0)=1;

        Mat Hrect;
        Hrect=buildRotTras(RLrect,Tfake);
        P=HL_root*Hrect*P;
        point.x=(float) ((float) P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float) ((float) P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float) ((float) P.at<double>(2,0)/P.at<double>(3,0));
    }

    this->mutexDisp.unlock();
    return point;
}


