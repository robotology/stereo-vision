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

    ResourceFinder camCalib;
    camCalib.setDefaultContext("cameraCalibration/conf");
    camCalib.configure("ICUB_ROOT",0,NULL);
    camCalibFile=camCalib.getContextPath().c_str();
    camCalibFile=camCalibFile+"/iCubEyes.ini";
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

                    Mat Kright=stereo.getKright();
                    Mat DistR=stereo.getDistCoeffRight();
                    writeCalibrationToFile(imgL->width,imgL->height,Kright.at<double>(0,0),Kright.at<double>(1,1),Kright.at<double>(0,2),Kright.at<double>(1,2),DistR.at<double>(0,0),DistR.at<double>(1,0),DistR.at<double>(2,0),DistR.at<double>(3,0),"CAMERA_CALIBRATION_RIGHT");

                    Mat Kleft=stereo.getKleft();
                    Mat DistL=stereo.getDistCoeffLeft();
                    writeCalibrationToFile(imgL->width,imgL->height,Kleft.at<double>(0,0),Kleft.at<double>(1,1),Kleft.at<double>(0,2),Kleft.at<double>(1,2),DistL.at<double>(0,0),DistL.at<double>(1,0),DistL.at<double>(2,0),DistL.at<double>(3,0),"CAMERA_CALIBRATION_LEFT");

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
bool stereoCalibThread::writeCalibrationToFile( int width, int height, float fx, float fy,float cx, float cy, float k1, float k2, float p1, float p2, string groupname){

    vector<string> lines;

    bool append = false;

    ifstream in;
    in.open(camCalibFile.c_str()); //camCalibFile.c_str());
    
    if(in.is_open()){
        // file exists
        string line;
        bool sectionFound = false;
        bool sectionClosed = false;

        // process lines
        while(std::getline(in, line)){
            // check if we left calibration section
            if (sectionFound == true && line.find("[", 0) != string::npos)
                sectionClosed = true;   // also valid if no groupname specified
            // check if we enter calibration section
            if (line.find(string("[") + groupname + string("]"), 0) != string::npos)
                sectionFound = true;
            // if no groupname specified
            if (groupname == "")
                sectionFound = true;
            // if we are in calibration section (or no section/group specified)
            if (sectionFound == true && sectionClosed == false){
                // replace w line
                if (line.find("w",0) != string::npos){
                    stringstream ss;
                    ss << width;
                    line = "w " + string(ss.str());
                }
                // replace h line
                if (line.find("h",0) != string::npos){
                    stringstream ss;
                    ss << height;
                    line = "h " + string(ss.str());
                }
                // replace fx line
                if (line.find("fx",0) != string::npos){
                    stringstream ss;
                    ss << fx;
                    line = "fx " + string(ss.str());
                }
                // replace fy line
                if (line.find("fy",0) != string::npos){
                    stringstream ss;
                    ss << fy;
                    line = "fy " + string(ss.str());
                }
                // replace cx line
                if (line.find("cx",0) != string::npos){
                    stringstream ss;
                    ss << cx;
                    line = "cx " + string(ss.str());
                }
                // replace cy line
                if (line.find("cy",0) != string::npos){
                    stringstream ss;
                    ss << cy;
                    line = "cy " + string(ss.str());
                }
                // replace k1 line
                if (line.find("k1",0) != string::npos){
                    stringstream ss;
                    ss << k1;
                    line = "k1 " + string(ss.str());
                }
                // replace k2 line
                if (line.find("k2",0) != string::npos){
                    stringstream ss;
                    ss << k2;
                    line = "k2 " + string(ss.str());
                }
                // replace p1 line
                if (line.find("p1",0) != string::npos){
                    stringstream ss;
                    ss << p1;
                    line = "p1 " + string(ss.str());
                }
                // replace p2 line
                if (line.find("p2",0) != string::npos){
                    stringstream ss;
                    ss << p2;
                    line = "p2 " + string(ss.str());
                }
       
            }
            // buffer line
            lines.push_back(line);
        }
        
        in.close();

        // rewrite file
        if (!sectionFound){
            append = true;
            cout << "Camera calibration parameter section " + string("[") + groupname + string("]") + " not found in file " << camCalibFile << ". Adding group..." << endl;
        }
        else{
            // rewrite file
            ofstream out;
            out.open(camCalibFile.c_str(), ios::trunc);
            if (out.is_open()){
                for (int i = 0; i < (int)lines.size(); i++)
                    out << lines[i] << endl;
                out.close();
            }
            else
                return false;
        }
        
    }
    else{
        append = true;
    }

    if (append){
        // file doesn't exist or section is appended 
        ofstream out;
        out.open(camCalibFile.c_str(), ios::app);
        if (out.is_open()){
            out << string("[") + groupname + string("]") << endl;
            out << endl;
            out << "w  " << width << endl;
            out << "h  " << height << endl;
            out << "fx " << fx << endl;
            out << "fy " << fy << endl;
            out << "cx " << cx << endl;
            out << "cy " << cy << endl;
            out << "k1 " << k1 << endl;
            out << "k2 " << k2 << endl;
            out << "p1 " << p1 << endl;
            out << "p2 " << p2 << endl;
            out << endl;
            out.close();
        }
        else
            return false;
    }

    return true;
}


