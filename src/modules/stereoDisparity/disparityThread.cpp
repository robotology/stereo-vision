#include "disparityThread.h"
#include <yarp/os/Stamp.h> 

void disparityThread::printMatrix(Mat &matrix) {
    int row=matrix.rows;
    int col =matrix.cols;
        cout << endl;
    for(int i = 0; i < matrix.rows; i++)
    {
        const double* Mi = matrix.ptr<double>(i);
        for(int j = 0; j < matrix.cols; j++)
            cout << Mi[j] << " ";
        cout << endl;
    }
        cout << endl;
}

disparityThread::disparityThread(string inputLeftPortName, string inputRightPortName, string outName, 
                                 string calibPath,Port* commPort)
{
    this->inputLeftPortName=inputLeftPortName;
    this->inputRightPortName=inputRightPortName;
    this->outName=outName;
    this->commandPort=commPort;
    this->stereo=new StereoCamera(calibPath+"/intrinsics.yml", calibPath+"/extrinsics.yml");
    angle=0;
    this->mutex = new Semaphore(1);
    this->mutexDisp = new Semaphore(1);
    this->HL_root= Mat::zeros(4,4,CV_64F);
    this->output=NULL;
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
void disparityThread::convert(Matrix& matrix, Mat& mat) {
    mat=cv::Mat(matrix.rows(),matrix.cols(),CV_64FC1);
    for(int i=0; i<matrix.rows(); i++)
        for(int j=0; j<matrix.cols(); j++)
            mat.at<double>(i,j)=matrix(i,j);
}

void disparityThread::convert(Mat& mat, Matrix& matrix) {
    matrix.resize(mat.rows,mat.cols);
    for(int i=0; i<mat.rows; i++)
        for(int j=0; j<mat.cols; j++)
            matrix(i,j)=mat.at<double>(i,j);
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




Matrix disparityThread::getCameraH(int camera) {

    yarp::sig::Vector x_curr;
    yarp::sig::Vector o_curr;

    if(camera==LEFT)
        igaze->getLeftEyePose(x_curr, o_curr);
    else
        igaze->getLeftEyePose(x_curr, o_curr);

    Matrix R_curr=axis2dcm(o_curr);

    Matrix H_curr(4, 4);
    for (int i=0; i<R_curr.cols(); i++)
        H_curr.setCol(i, R_curr.getCol(i));
    for (int i=0; i<x_curr.size(); i++)
        H_curr(i,3)=x_curr[i];

    this->mutexDisp->wait();
    if(camera==LEFT)
        convert(H_curr,HL_root);
    this->mutexDisp->post();

    return H_curr;
}

void disparityThread::run(){

    imageL=new ImageOf<PixelRgb>;
    imageR=new ImageOf<PixelRgb>;

    bool initL=false;
    bool initR=false;


    Matrix yarp_initLeft,yarp_initRight;
    Matrix yarp_H0;

    Point3d point;
    bool init=true;
    while (!isStopping()) {

        ImageOf<PixelRgb> *tmpL = imagePortInLeft.read(false);
        ImageOf<PixelRgb> *tmpR = imagePortInRight.read(false);

        if(tmpL!=NULL && tmpR!=NULL){

              imgL= (IplImage*) imageL->getIplImage();
              imgR= (IplImage*) imageR->getIplImage();

              this->mutex->wait();
              this->stereo->setImages(imgL,imgR);
              this->mutex->post();

              if(init) {
                   stereo->undistortImages();
                   stereo->findMatch();
                   stereo->estimateEssential();
                   stereo->hornRelativeOrientations();
                   output=cvCreateImage(cvSize(imgL->width,imgL->height),8,3);
                   init=false;

                   Mat H0_R=this->stereo->getRotation();
                   Mat H0_T=this->stereo->getTranslation();

                   Mat H0=buildRotTras(H0_R,H0_T);

                   convert(H0,yarp_H0);

                   //get the initial left and right matrices
                   yarp_initLeft=getCameraH(LEFT);
                   yarp_initRight=getCameraH(RIGHT);
              }

              //transformation matrices between prev and curr eye frames
              Matrix yarp_Left=getCameraH(LEFT);
              Matrix yarp_Right=getCameraH(RIGHT);

              yarp_Left=SE3inv(yarp_Left)*yarp_initLeft;
              yarp_Right=SE3inv(yarp_Right)*yarp_initRight;

              Matrix Hcurr=SE3inv(yarp_Left)*yarp_H0*yarp_Right;

              Matrix R=Hcurr.submatrix(0,2,0,2);
              yarp::sig::Vector x=dcm2axis(R);
              Matrix newTras=Hcurr.submatrix(0,2,3,3);
              this->mutex->wait();

              // Update Rotation
              Mat Rot(3,3,CV_64FC1);
              convert(R,Rot);
              this->stereo->setRotation(Rot,0);

              //Update Translation
              Mat translation(3,1,CV_64FC1);
              convert(newTras,translation);
              this->stereo->setTranslation(translation,0);
              this->mutex->post();

              this->mutexDisp->wait();
              this->stereo->computeDisparity();
              this->mutexDisp->post();        

              if(outPort.getOutputCount()>0) {
                 disp=stereo->getDisparity();
                 cvCvtColor(&disp,output,CV_GRAY2RGB);
                 ImageOf<PixelBgr>& outim=outPort.prepare();                   
                 outim.wrapIplImage(output);
                 outPort.write();
             }

              initL=initR=false;
        }
    
      
   }
   delete imageL;
   delete imageR;
}

//
//void disparityThread::run(){
//
//    imageL=new ImageOf<PixelRgb>;
//    imageR=new ImageOf<PixelRgb>;
//
//    Stamp TSLeft;
//    Stamp TSRight;
//
//    bool initL=false;
//    bool initR=false;
//
//    getH();
//
//    Matrix R=H.submatrix(0,2,0,2);
//    yarp::sig::Vector x=dcm2axis(R);
//    angle=x[3];
//
//    updateCameraThread updator(this->stereo,this->mutex,500);
//   // updator.start(); // It is not needed to update cameras at each iteration, only one update during the initialization is enough
//
//    Point3d point;
//    bool init=true;
//    while (!isStopping()) {      
//
//        ImageOf<PixelRgb> *tmpL = imagePortInLeft.read(false);
//        ImageOf<PixelRgb> *tmpR = imagePortInRight.read(false);
//
//        if(tmpL!=NULL)
//        {
//            *imageL=*tmpL;
//            imagePortInLeft.getEnvelope(TSLeft);
//            initL=true;
//        }
//        if(tmpR!=NULL) 
//        {
//            *imageR=*tmpR;
//            imagePortInRight.getEnvelope(TSRight);
//            initR=true;
//        }
//       
//
//
//        if(initL && initR && Cvtools::checkTS(TSLeft.getTime(),TSRight.getTime())){        
//              getH();
//              Matrix R=H.submatrix(0,2,0,2);
//              yarp::sig::Vector x=dcm2axis(R);
//              Matrix newTras=H.submatrix(0,2,3,3);       
//              this->mutex->wait();
//
//              // Update Rotation
//              x[3]=x[3]-angle;
//              R=axis2dcm(x);
//              Mat Rot(3,3,CV_64FC1);
//              convert(R,Rot);
//              this->stereo->setRotation(Rot,2);
//              Mat translation(3,1,CV_64FC1);
//
//              //Update Translation
//              Matrix temp2=newTras;
//              convert(temp2,translation);
//              this->stereo->setTranslation(translation,0);
//              this->mutex->post();
//
//              imgL= (IplImage*) imageL->getIplImage();
//              imgR= (IplImage*) imageR->getIplImage();
//  
//              this->mutex->wait();
//              this->stereo->setImages(imgL,imgR);
//              this->mutex->post();
//
//              if(init) {
//                   stereo->undistortImages();
//                   stereo->findMatch();
//                   stereo->estimateEssential();
//                   stereo->hornRelativeOrientations();
//                   output=cvCreateImage(cvSize(imgL->width,imgL->height),8,3);
//                   init=false;
//              }
//
//              this->mutexDisp->wait();
//              this->stereo->computeDisparity();
//              this->mutexDisp->post();        
//
//              if(outPort.getOutputCount()>0) {
//                 disp=stereo->getDisparity();
//                 cvCvtColor(&disp,output,CV_GRAY2RGB);
//                 ImageOf<PixelBgr>& outim=outPort.prepare();                   
//                 outim.wrapIplImage(output);
//                 outPort.setEnvelope(TSLeft);
//                 outPort.write();
//             }
//
//              initL=initR=false;
//        }
//    
//      
//   }
//   updator.stop();
//   delete imageL;
//   delete imageR;
//}

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
    
    if(output!=NULL)
        cvReleaseImage(&output);

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
    u=u; // matrix starts from (0,0), pixels from (0,0)
    v=v;
    Point3f point;

    if(drive!="RIGHT" && drive !="LEFT" && drive!="ROOT") {
        point.x=-1.0;
        point.y=-1.0;
        point.z=-1.0;
        return point;
    }

    this->mutexDisp->wait();   
    // Mapping from Rectified Cameras to Original Cameras
    Mat Mapper=this->stereo->getMapperL();

    if(Mapper.empty()) {
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

    // discard points far more than 2.5 meters or with not valid disparity (<0)
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
            Mat Tright = this->stereo->getTranslation();
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
updateCameraThread::updateCameraThread(StereoCamera * cam, Semaphore * mut, int _period): RateThread(_period) {
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
