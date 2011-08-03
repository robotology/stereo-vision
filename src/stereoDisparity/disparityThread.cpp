#include "disparityThread.h"
#include <yarp/os/Stamp.h> 


disparityThread::disparityThread(string inputLeftPortName, string inputRightPortName, string outName, 
                                 string calibPath,Port* commPort, string driveEye)
{
    this->inputLeftPortName=inputLeftPortName;
    this->inputRightPortName=inputRightPortName;
    this->outName=outName;
    this->commandPort=commPort;
    this->stereo=new stereoCamera(calibPath+"/intrinsics.yml", calibPath+"/extrinsics.yml");
    angle=0;
    this->mutex = new Semaphore(1);
    this->mutexDisp = new Semaphore(1);
    this->driveEye=driveEye;
    this->HL_root= Mat::zeros(4,4,CV_64F);
}



bool disparityThread::threadInit() 
{
    if (!imagePortInLeft.open(inputLeftPortName.c_str())) {
        cout  << ": unable to open port " << inputLeftPortName << endl;
        return false; 
   }

    if (!imagePortInRight.open(inputRightPortName.c_str())) {
        cout << ": unable to open port " << inputRightPortName << endl;
        return false;
   }

    if (!outPort.open(outName.c_str())) {
        cout << ": unable to open port " << outName << endl;
        return false;
   }

    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    option.put("local","/client/gaze");
    gazeCtrl=new PolyDriver(option);
    if (gazeCtrl->isValid()) {
        gazeCtrl->view(igaze);
    }
    else {
        cout<<"Devices not available"<<endl;
        return false;
    }

   return true;
}

void disparityThread::printMatrixYarp(Matrix &A) {
    cout << endl;
    for (int i=0; i<A.rows(); i++) {
        for (int j=0; j<A.cols(); j++) {
            cout<<A(i,j)<<" ";
        }
        cout<<endl;
    }
    cout << endl;

}
void disparityThread::convert(Matrix& R, Mat& Rot) {
    for(int i=0; i<Rot.rows; i++)
        for(int j=0; j<Rot.cols; j++)
            Rot.at<double>(i,j)=R(i,j);
}

void disparityThread::getH() {

    yarp::sig::Vector xl;
    yarp::sig::Vector ol;
    yarp::sig::Vector xr;
    yarp::sig::Vector orr;

    igaze->getLeftEyePose(xl, ol);
    igaze->getRightEyePose(xr, orr);

    Matrix Rl=axis2dcm(ol);
    Matrix Rr=axis2dcm(orr);

    int i=0;
    Matrix Hl(4, 4);
    for (i=0; i<Rl.cols(); i++)
        Hl.setCol(i, Rl.getCol(i));
    for (i=0; i<xl.size(); i++)
        Hl(i,3)=xl[i];

    int j=0;
    Matrix Hr(4, 4);
    for (j=0; j<Rr.cols(); j++)
        Hr.setCol(j, Rr.getCol(j));
    for (j=0; j<xr.size(); j++)
        Hr(j,3)=xr[j];

    

    this->mutexDisp->wait();
    convert(Hl,HL_root);
    this->mutexDisp->post();

    H=SE3inv(Hr)*Hl;
}


void disparityThread::run(){

    imageL=new ImageOf<PixelRgb>;
    imageR=new ImageOf<PixelRgb>;

    Stamp TSLeft;
    Stamp TSRight;

    bool initL=false;
    bool initR=false;


    IplImage * output=cvCreateImage(cvSize(320,240),8,3);
    getH();

    Matrix R=H.submatrix(0,2,0,2);
    yarp::sig::Vector x=dcm2axis(R);
    tras=H.submatrix(0,2,3,3);
    angle=x[3];

    updateCameraThread updator(this->stereo,this->mutex,500);
   // updator.start();

    Point3d point;
    bool init=true;
    while (!isStopping()) {      

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
       


        if(initL && initR && Cvtools::checkTS(TSLeft.getTime(),TSRight.getTime())){        
              getH();
              Matrix R=H.submatrix(0,2,0,2);
              yarp::sig::Vector x=dcm2axis(R);
              Matrix newTras=H.submatrix(0,2,3,3);       
              this->mutex->wait();

              // Update Rotation
              x[3]=x[3]-angle;
              R=axis2dcm(x);
              Mat Rot(3,3,CV_64FC1);
              convert(R,Rot);
              this->stereo->setRotation(Rot,2);
              Mat traslation(3,1,CV_64FC1);

              //Update Translation
              Matrix temp2=newTras;
              convert(temp2,traslation);
              this->stereo->setTranslation(traslation,0);
              this->mutex->post();

              imgL= (IplImage*) imageL->getIplImage();
              imgR= (IplImage*) imageR->getIplImage();
  
              this->mutex->wait();
              this->stereo->setImages(imgL,imgR);
              this->mutex->post();

              if(init) {
                   this->mutex->wait();
                   stereo->undistortImages();
                   stereo->findMatch();
                   stereo->estimateEssential();
                   stereo->hornRelativeOrientations();
                   this->mutex->post();
                   init=false;
              }

              this->mutexDisp->wait();
              this->stereo->computeDisparity();
              this->mutexDisp->post();        

              if(outPort.getOutputCount()>0) {

                 disp=stereo->getDisparity();               
                 cvCvtColor(&disp,output,CV_GRAY2RGB);
                 ImageOf<PixelBgr>& outim=outPort.prepare();                   
                 outim.wrapIplImage(output);
                 outPort.setEnvelope(TSLeft);
                 outPort.write();
             }

              initL=initR=false;
        }
    
      
   }
   updator.stop();
   delete imageL;
   delete imageR;
}

void disparityThread::threadRelease() 
{
    imagePortInRight.close();
    imagePortInLeft.close();
    outPort.close();
    commandPort->close();
    delete this->stereo;
    delete this->mutex;
    delete this->mutexDisp;
    delete gazeCtrl;

}
void disparityThread::onStop() {
    imagePortInRight.interrupt();
    imagePortInLeft.interrupt();
    commandPort->interrupt();
}

Mat disparityThread::buildRotTras(Mat & R, Mat & T) {
        
        Mat A = Mat::eye(4, 4, CV_64F);
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
        return A;
}


Point3f disparityThread::get3DPoints(int u, int v, string drive) {
    u=u-1; // matrix starts from (0,0), pixels from (1,1)
    v=v-1;
    Point3f point;

    if(drive!="RIGHT" && drive !="LEFT" && drive!="ROOT") {
        point.x=-1.0;
        point.y=-1.0;
        point.z=-1.0;
        return point;
    }

    this->mutexDisp->wait();   
    Mat Mapper=this->stereo->getMapperL();

    if(Mapper.size().width==0) {
        point.x=-1.0;
        point.y=-1.0;
        point.z=-1.0;

        this->mutexDisp->post();   
        return point;
    }


    float usign=Mapper.ptr<float>(v)[2*u];
    float vsign=Mapper.ptr<float>(v)[2*u+1]; 

    u=cvRound(usign);
    v=cvRound(vsign);

    /* Mat img=this->stereo->getDepthPoints();
    if(u<0 || u>=img.size().width || v<0 || v>=img.size().height) {
        point.x=-1.0;
        point.y=-1.0;
        point.z=-1.0;
    }
    else {
        point.x= img.ptr<float>(v)[3*u];
        point.y= img.ptr<float>(v)[3*u+1];
        point.z= img.ptr<float>(v)[3*u+2];
    }*/   
    IplImage disp16=this->stereo->getDisparity16();


    if(u<0 || u>=disp.width || v<0 || v>=disp.height) {
        point.x=-1.0;
        point.y=-1.0;
        point.z=-1.0;
    }
    else {
        Mat Q=this->stereo->getQ();
        CvScalar scal= cvGet2D(&disp16,v,u);
        double disparity=-scal.val[0]/16.0;
        float w= (float) ((float) disparity*Q.at<double>(3,2)) + ((float)Q.at<double>(3,3));
        point.x= (float)((float) (usign+1)*Q.at<double>(0,0)) + ((float) Q.at<double>(0,3));
        point.y=(float)((float) (vsign+1)*Q.at<double>(1,1)) + ((float) Q.at<double>(1,3));
        point.z=(float) Q.at<double>(2,3);

        point.x=point.x/w;
        point.y=point.y/w;
        point.z=point.z/w;
    }

   
    if(point.z>2.5 || point.z<0) {
        point.x=-1.0;
        point.y=-1.0;
        point.z=-1.0;
    } 
    else {
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
            Mat Tright = this->stereo->getTraslation();
            Mat RRright = this->stereo->getRRrect().t();
            Mat TRright = Mat::zeros(0,3,CV_64F);

            Mat HRL=buildRotTras(Rright,Tright);
            Mat Hrect=buildRotTras(RRright,TRright);

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

            Mat Hrect=buildRotTras(RLrect,Tfake);           
           
            P=HL_root*Hrect*P;     

            point.x=(float) ((float) P.at<double>(0,0)/P.at<double>(3,0));
            point.y=(float) ((float) P.at<double>(1,0)/P.at<double>(3,0));
            point.z=(float) ((float) P.at<double>(2,0)/P.at<double>(3,0));
       }
  


    }

    this->mutexDisp->post();    
    return point;

}
updateCameraThread::updateCameraThread(stereoCamera * cam, Semaphore * mut, int _period): RateThread(_period) {
    this->stereo=cam;
    this->mutex=mut;
}

void updateCameraThread::run() {
    this->mutex->wait();
    this->stereo->undistortImages();
    this->mutex->post();

    this->stereo->findMatch();
    this->stereo->estimateEssential();

    this->mutex->wait();
    this->stereo->hornRelativeOrientations();
    this->mutex->post();

 }
