#include "stereoCalibThread.h"


stereoCalibThread::stereoCalibThread(ResourceFinder &rf, Port* commPort, const char *dir)
{

    string moduleName=rf.check("name", Value("stereoCalib"),"module name (string)").asString().c_str();

    this->inputLeftPortName         = "/"+moduleName;
    this->inputLeftPortName        +=rf.check("imgLeft",Value("/cam/left:i"),"Input image port (string)").asString();
   
    this->inputRightPortName        = "/"+moduleName;
    this->inputRightPortName       += rf.check("imgRight", Value("/cam/right:i"),"Input image port (string)").asString();
	

    this->outNameRight        = "/"+moduleName;
    this->outNameRight       += rf.check("outRight",Value("/cam/right:o"),"Output image port (string)").asString();
	
    this->outNameLeft        = "/"+moduleName;
    this->outNameLeft       +=rf.check("outLeft",Value("/cam/left:o"),"Output image port (string)").asString();

    this->boardWidth=  rf.check("boardWidth", Value(9)).asInt();
    this->boardHeight= rf.check("boardHeight", Value(6)).asInt();
    this->squareSize= (float)rf.check("boardSize", Value(0.03)).asDouble();
    this->commandPort=commPort;
    this->dir=dir;
    this->startCalibration=0;

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

        if(initL && initR && checkTS(TSLeft.getTime(),TSRight.getTime())){

            if(startCalibration>0) {

                string pathImg=dir;
                preparePath(pathImg.c_str(), pathL,pathR,count);
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
                        saveStereoImage(pathImg.c_str(),imgL,imgR,count);

                        imageListR.push_back(imr);
                        imageListL.push_back(iml);
                        imageListLR.push_back(iml);
                        imageListLR.push_back(imr);
                        Time::delay(2.0);

                        count++;                    
             }

                if(count>30) {
                    fprintf(stdout," Running Left Camera Calibration... \n");
                    Camera left;
                    left.calibrate(imageListL,this->boardWidth,this->boardHeight);

                    fprintf(stdout," Running Right Camera Calibration... \n");
                    Camera right;
                    right.calibrate(imageListR,this->boardWidth,this->boardHeight);
                    
                    fprintf(stdout,"Running Stereo Calibration... \n");

                    StereoCamera stereo(left,right);
                    stereo.stereoCalibration(imageListLR, this->boardWidth,this->boardHeight,this->squareSize);
                    fprintf(stdout," Saving Calibration Results... \n");

                    stereo.saveCalibration(dir+"/../extrinsics",dir+"/../intrinsics");
                    fprintf(stdout,"Configuration Files Saved! Now you can run stereoDisparity! \n");
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
void stereoCalibThread::startCalib() {
    startCalibration=1;
  }

void stereoCalibThread::printMatrix(Mat &matrix) {
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


bool stereoCalibThread::checkTS(double TSLeft, double TSRight, double th) {
    double diff=fabs(TSLeft-TSRight);
    if(diff <th)
        return true;
    else return false;

}

void stereoCalibThread::preparePath(const char * dir, char* pathL, char* pathR, int count) {
    char num[5];
    sprintf(num, "%i", count); 


    strncpy(pathL,dir, strlen(dir));
    pathL[strlen(dir)]='\0';
    strcat(pathL,"left");
    strcat(pathL,num);
    strcat(pathL,".png");

    strncpy(pathR,dir, strlen(dir));
    pathR[strlen(dir)]='\0';
    strcat(pathR,"right");
    strcat(pathR,num);
    strcat(pathR,".png");

}


void stereoCalibThread::saveStereoImage(const char * dir, IplImage* left, IplImage * right, int num) {
    char pathL[256];
    char pathR[256];
    preparePath(dir, pathL,pathR,num);
    
    fprintf(stdout,"Saving images number %d \n",num);

    cvSaveImage(pathL,left);
    cvSaveImage(pathR,right);
}