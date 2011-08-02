#include "stereoCalibThread.h"
#include <yarp/os/Stamp.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/math.h>

using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


void printMatrixYarp(Matrix matrix) {

       int row=matrix.rows();
    int col =matrix.cols();
        cout << endl;
   /* for(int i=0; i<row; i++) {
        for(int j=0; j<col; j++) {
            cout << matrix(Range(i,i),Range(j,j));
        }
        cout << endl;
    }*/
    for(int i = 0; i < row; i++)
    {
        for(int j = 0; j < col; j++) {
            double& val =matrix(i,j);
            cout << val << " ";
        }
        cout << endl;
    }
        cout << endl;
}


stereoCalibThread::stereoCalibThread(string inputLeftPortName, string inputRightPortName, string outNameRight, string outNameLeft, Port* commPort, const char *dir, int bwidth, int bheight, float squsize)
{
 this->inputLeftPortName=inputLeftPortName;
 this->inputRightPortName=inputRightPortName;
 this->outNameRight=outNameRight;
 this->outNameLeft=outNameLeft;
 this->commandPort=commPort;
 this->dir=dir;
 this->saveVideo=0;
 this->startCalibration=0;
 this->boardWidth=bwidth;
 this->boardHeight=bheight;
 this->squareSize=squsize;


 fprintf(stdout, "%s \n", dir);
}

bool stereoCalibThread::threadInit() 
{
     if (!imagePortInLeft.open(inputLeftPortName.c_str())) {
      cout  << ": unable to open port " << inputLeftPortName << endl;
      return false; 
   }

   if (!imagePortInRight.open(inputRightPortName.c_str())) {
      cout << ": unable to open port " << inputRightPortName << endl;
      return false;
   }

    if (!outPortLeft.open(outNameLeft.c_str())) {
      cout << ": unable to open port " << outNameLeft << endl;
      return false;
   }

    if (!outPortRight.open(outNameRight.c_str())) {
      cout << ": unable to open port " << outNameRight << endl;
      return false;
   }    

   return true;
}

void stereoCalibThread::run(){

	imageL=new ImageOf<PixelRgb>;
	imageR=new ImageOf<PixelRgb>;

    Stamp TSLeft;
    Stamp TSRight;

    bool initL=false;
    bool initR=false;

    int count=1;
    Size boardSize, imageSize;
    boardSize.width=this->boardWidth;
    boardSize.height=this->boardHeight;

    vector<string> imageListR, imageListL, imageListLR;



   while (!isStopping()) { // the thread continues to run until isStopping() returns true
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


            if(saveVideo>0) {
                imgL= (IplImage*) imageL->getIplImage();
                imgR= (IplImage*) imageR->getIplImage();
    	        cvCvtColor(imgL,imgL,CV_RGB2BGR);
                cvCvtColor(imgR,imgR, CV_RGB2BGR);
                Cvtools::saveStereoImage(dir.c_str(),imgL,imgR,count);
                count=count+1;
                saveVideo=0;
            }

            if(startCalibration>0) {

                string pathImg=dir;
                Cvtools::preparePath(pathImg.c_str(), pathL,pathR,count);
                string iml(pathL);
                string imr(pathR);
                imgL= (IplImage*) imageL->getIplImage();
                imgR= (IplImage*) imageR->getIplImage();
                Mat Left(imgL);
                Mat Right(imgR);


                vector<Point2f> pointbuf;
                bool foundL = findChessboardCorners( Left, boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH & CV_CALIB_CB_FAST_CHECK & CV_CALIB_CB_NORMALIZE_IMAGE);
                bool foundR = findChessboardCorners( Right, boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH & CV_CALIB_CB_FAST_CHECK & CV_CALIB_CB_NORMALIZE_IMAGE);
                
                if(foundL && foundR) {
                        cvCvtColor(imgL,imgL,CV_RGB2BGR);
                        cvCvtColor(imgR,imgR, CV_RGB2BGR);                        
                        Cvtools::saveStereoImage(pathImg.c_str(),imgL,imgR,count);

                        imageListR.push_back(imr);
                        imageListL.push_back(iml);
                        imageListLR.push_back(iml);
                        imageListLR.push_back(imr);
                        Time::delay(2.0);

                        count++;                    
             }

                if(count>30) {
                    cout << " Running Left Camera Calibration..." << endl;
                    Camera left;
                    left.calibrate(imageListL,this->boardWidth,this->boardHeight);

                    cout << " Running Right Camera Calibration..." << endl;
                    Camera right;
                    right.calibrate(imageListR,this->boardWidth,this->boardHeight);

                    cout << " Running Stereo Calibration..." << endl;
                    stereoCamera stereo(left,right);
                    stereo.stereoCalibration(imageListLR, this->boardWidth,this->boardHeight,this->squareSize);
                    cout << " Saving Calibration Results... " << endl;
                    stereo.saveCalibration(dir+"/../extrinsics",dir+"/../intrinsics");
                    cout << "Configuration Files Saved! Now you can run stereoDisparity! " << endl;
                    startCalibration=0;
                }


            }
                ImageOf<PixelRgb>& outimL=outPortLeft.prepare();
                outimL=*imageL;
                outPortLeft.write();

                ImageOf<PixelRgb>& outimR=outPortRight.prepare();
                outimR=*imageR;
                outPortRight.write();
                initL=initR=false;
        }
    
      
   }

   delete imageL;
   delete imageR;
}

void stereoCalibThread::threadRelease() 
{
    imagePortInRight.close();
    imagePortInLeft.close();
    outPortLeft.close();
    outPortRight.close();
    commandPort->close();
}
 void stereoCalibThread::onStop() {
    imagePortInRight.interrupt();
    imagePortInLeft.interrupt();
    outPortLeft.interrupt();
    outPortRight.interrupt();
    commandPort->interrupt();
}
  void stereoCalibThread::setSave() {
   saveVideo=1;
}

  void stereoCalibThread::startCalib() {
    startCalibration=1;

  }
