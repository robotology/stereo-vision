#include "disparityThread.h"
#include <yarp/os/Stamp.h> 


disparityThread::disparityThread(string inputLeftPortName, string inputRightPortName, string outName, 
                                 string output3DPointName, string inputFixName, 
                                 string calibPath,Port* commPort, int useFix)
{
 this->inputLeftPortName=inputLeftPortName;
 this->inputRightPortName=inputRightPortName;
 this->outName=outName;
 this->commandPort=commPort;
 this->stereo=new stereoCamera(calibPath+"/intrinsics.yml", calibPath+"/extrinsics.yml");
 angle=0;
 this->mutex = new Semaphore(1);
 this->outWorldPointName=output3DPointName;
 this->inFixationName=inputFixName;
 this->useFixation=useFix;
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

    if (!WorldPointPort.open(outWorldPointName.c_str())) {
      cout << ": unable to open port " << outWorldPointName << endl;
      return false;
   }

    if (!InputFixationPort.open(inFixationName.c_str())) {
      cout << ": unable to open port " << inFixationName << endl;
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

	H=SE3inv(Hr)*Hl;
}


void disparityThread::run(){

	imageL=new ImageOf<PixelRgb>;
	imageR=new ImageOf<PixelRgb>;

    Stamp TSLeft;
    Stamp TSRight;

    bool initL=false;
    bool initR=false;

    int count=1;
    IplImage * output=cvCreateImage(cvSize(320,240),8,3);
	getH();
   
    Matrix R=H.submatrix(0,2,0,2);
    yarp::sig::Vector x=dcm2axis(R);
    tras=H.submatrix(0,2,3,3);
    angle=x[3];

    double thang=0.01; // 0.01
    double thtras=0.005; //0.005

    int pixelX=160;
    int pixelY=120;

    updateCameraThread updator(this->stereo,this->mutex,500);
    Point3d point;

  //  updator.start();
    bool init=true;
    while (!isStopping()) {
        	   
               getH();
			   Matrix R=H.submatrix(0,2,0,2);
			   yarp::sig::Vector x=dcm2axis(R);
               Matrix newTras=H.submatrix(0,2,3,3);
               double norma=iCub::ctrl::norm(tras-newTras,0);
          
			   if (abs(x[3]-angle)>thang || norma>thtras ) {
                       
                   this->mutex->wait();
                   if(abs(x[3]-angle)>thang) {
                       double temp = x[3];
                       x[3]=x[3]-angle;
                       R=axis2dcm(x);
                       Mat Rot(3,3,CV_64FC1);
                       convert(R,Rot);

		               this->stereo->setRotation(Rot,1);
                       angle=temp;
                   }
                   Mat traslation(3,1,CV_64FC1);
                   Matrix temp=newTras-tras;
                   convert(temp,traslation);

                   this->stereo->setTranslation(traslation,1);
                   this->mutex->post();
               
                   tras=newTras;
                   //fprintf(stdout, "Angle: %f, Tras: %f \n", abs(x[3]-angle), norma);
                   //printMatrix((Mat &)this->stereo->getTranslation());


               }

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
               init=false;
              }

              this->stereo->computeDisparity();

              if(!this->useFixation) {
                  pixelX=160;
                  pixelY=120;
              }
              else {
                  Bottle * fix =InputFixationPort.read();
                  if(fix!=NULL) {
                      pixelX=fix->get(0).asInt();
                      pixelY=fix->get(1).asInt();
                  }
              }
              disp=stereo->getDisparity();

              IplImage disp16=stereo->getDisparity16();

              CvScalar scal= cvGet2D(&disp16,pixelY,pixelX);
              double disparity=-scal.val[0]/16.;
                 
             // cout << disparity << endl;

              // If we have a valid disparity, compute the 3D point and write on the output port
              if(disparity<0.0) {
                  Mat Q=this->stereo->getQ();

                  double w= (disparity*Q.at<double>(3,2)) + Q.at<double>(3,3) ;
                  point.x= (pixelX*Q.at<double>(0,0)) + Q.at<double>(0,3);
                  point.y=(pixelY*Q.at<double>(1,1)) + Q.at<double>(1,3);
                  point.z= Q.at<double>(2,3);

                  point.x=point.x/w;
                  point.y=point.y/w;
                  point.z=point.z/w;

                  Bottle& outPoint = WorldPointPort.prepare();
                  outPoint.clear();
                  outPoint.addString("Point 3D");
                  outPoint.addDouble(point.x);
                  outPoint.addDouble(point.y);
                  outPoint.addDouble(point.z);
                  WorldPointPort.write();
               //   cout << "writing " << outPoint.toString().c_str() << endl;       
               //   cout << "X: " << point.x << " Y: " << point.y << " Z: " << point.z << endl;

              } else {
                 // We handle outliers (no valid disparity) with very large distances
                  Bottle& outPoint = WorldPointPort.prepare();
                  outPoint.clear();
                  outPoint.addString("Point 3D");
                  outPoint.addDouble(1E100);
                  outPoint.addDouble(1E100);
                  outPoint.addDouble(1E100);
                  WorldPointPort.write();
              }
            
              cvCvtColor(&disp,output,CV_GRAY2RGB);
              ImageOf<PixelBgr>& outim=outPort.prepare();
               
              outim.wrapIplImage(output);
              outPort.write();

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
    WorldPointPort.close();
    InputFixationPort.close();
    commandPort->close();
    delete this->stereo;
    delete this->mutex;
	igaze->stopControl();
	gazeCtrl->close();

}
void disparityThread::onStop() {
    imagePortInRight.interrupt();
    imagePortInLeft.interrupt();
    outPort.interrupt();
    commandPort->interrupt();
    WorldPointPort.interrupt();
    InputFixationPort.interrupt();

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
