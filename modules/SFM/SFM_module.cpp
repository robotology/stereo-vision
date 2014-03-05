#include "SFM_module.h"



bool SFM::configure(ResourceFinder &rf)
{
    string name=rf.check("name",Value("/SFM")).asString().c_str();
    string left=rf.check("leftPort",Value("/left:i")).asString().c_str();
    left=name+left;
    string right=rf.check("rightPort",Value("/right:i")).asString().c_str();

    string outDispName=rf.check("outDispPort",Value("/disp:o")).asString().c_str();
    string outMatchName=rf.check("outMatchPort",Value("/match:o")).asString().c_str();

    ResourceFinder localCalibration;
    localCalibration.setContext("cameraCalibration");
    localCalibration.setDefaultConfigFile("SFM_currCalib.ini");
    localCalibration.configure(NULL,0);

    this->camCalibFile=localCalibration.getHomeContextPath().c_str();
    this->camCalibFile=this->camCalibFile+"/SFM_currCalib.ini";

    right=name+right;
    outMatchName=name+outMatchName;
    outDispName=name+outDispName;
    
    string rpc_name=name+"/rpc";
    string world_name=name+rf.check("outWorldPort",Value("/world:o")).asString().c_str();

    int calib= rf.check("useCalibrated",Value(1)).asInt();
    bool useCalibrated= calib ? true : false;

    leftImgPort.open(left.c_str());
    rightImgPort.open(right.c_str());
    outMatch.open(outMatchName.c_str());
    outDisp.open(outDispName.c_str());
    handlerPort.open(rpc_name.c_str());
    worldPort.open(world_name.c_str());
    attach(handlerPort);

    this->stereo=new StereoCamera(true);
    Mat KL, KR, DistL, DistR, R, T;
    

    loadIntrinsics(rf,KL,KR,DistL,DistR);
    loadExtrinsics(localCalibration,R,T);
    
    this->mutexDisp = new Semaphore(1);

    stereo->setIntrinsics(KL,KR,DistL,DistR);



    this->useBestDisp=true;
    this->uniquenessRatio=15;
    this->speckleWindowSize=50;
    this->speckleRange=16;
    this->numberOfDisparities=64;
    this->SADWindowSize=7;
    this->minDisparity=0;
    this->preFilterCap=63;
    this->disp12MaxDiff=0;
    
    this->HL_root= Mat::zeros(4,4,CV_64F);
    this->HR_root= Mat::zeros(4,4,CV_64F);

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
    numberOfTrials=0;
#ifdef USING_GPU
    utils = new Utilities();
    utils->initSIFT_GPU();
#endif
    
    
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    option.put("local","/client/SFMClient");
    gazeCtrl=new PolyDriver(option);
    if (gazeCtrl->isValid()) {
        gazeCtrl->view(igaze);
    }
    else {
        cout<<"Devices not available"<<endl;
        return false;
    }
        
    if(!R.empty() && !T.empty())
    {
        stereo->setRotation(R,0);
        stereo->setTranslation(T,0);
    }
    else
    {
       cout << "No local calibration file found in " <<  camCalibFile <<" ... Using Kinematics and Running SFM once." << endl;
       updateViaKinematics();
    }

    doSFM=false;

    updateViaKinematics(true);
    
    return true;
    
    
}

void SFM::updateViaKinematics(bool exp)
{
        
    Matrix L1=getCameraHGazeCtrl(LEFT);
    Matrix R1=getCameraHGazeCtrl(RIGHT);
    
    Matrix RT=SE3inv(R1)*L1; 
    
    Mat R= Mat::zeros(3,3,CV_64F);
    Mat T= Mat::zeros(3,1,CV_64F);
    
    for (int i=0; i<R.rows; i++)
    {
        for(int j=0; j<R.cols; j++)
        {
            R.at<double>(i,j)=RT(i,j);
        }
    }
    
    
    for (int i=0; i<T.rows; i++)
    {
        T.at<double>(i,0)=RT(i,3);
    }
    

    if(!exp)
    {
        stereo->setRotation(R,0);
        stereo->setTranslation(T,0);
    }
    else
    {
    
        stereo->setExpectedPosition(R,T);
    }

}

bool SFM::close()
{
    
    leftImgPort.interrupt();
    leftImgPort.close();

    rightImgPort.interrupt();
    rightImgPort.close();

    outDisp.interrupt();
    outDisp.close();
    

    outMatch.interrupt();
    outMatch.close();
    
    handlerPort.interrupt();    
    handlerPort.close();
    
    worldPort.interrupt();
    worldPort.close();

    if(output_match!=NULL)
        cvReleaseImage(&output_match);

    if(outputD!=NULL)
        cvReleaseImage(&outputD);
   
    delete gazeCtrl;

#ifdef USING_GPU
    delete utils;
#endif
    

    delete mutexDisp;
    return true;
}


void SFM::printMatrix(Mat &matrix) {
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

bool SFM::interruptModule()
{
    leftImgPort.interrupt();
    leftImgPort.close();
    rightImgPort.interrupt();
    rightImgPort.close();
    outDisp.close();
    outDisp.interrupt();
    
    handlerPort.interrupt();
    outMatch.close();
    outMatch.interrupt();
    worldPort.interrupt();
    return true;
}
bool SFM::updateModule()
{
    ImageOf<PixelRgb> *yarp_imgL=NULL;
    ImageOf<PixelRgb> *yarp_imgR=NULL;

    updateViaKinematics(true);
    
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
        if(left->width==320)
            this->numberOfDisparities=64;
        if(left->width==640)
            this->numberOfDisparities=128l

        init=false;
    }

    Matrix yarp_Left=getCameraHGazeCtrl(LEFT);
    Matrix yarp_Right=getCameraHGazeCtrl(RIGHT);

    Mat leftMat(left); 
    Mat rightMat(right);
    this->stereo->setImages(left,right);
    
    mutexRecalibration.lock();
    if(doSFM || doSFMOnce)
    {
    #ifdef USING_GPU
        utils->extractMatch_GPU( leftMat, rightMat);
        vector<Point2f> leftM;
        vector<Point2f> rightM;
        utils->getMatches(leftM,rightM);
        mutexDisp->wait();
        this->stereo->setMatches(leftM,rightM);
    #else
        this->stereo->findMatch(false,15);
    #endif
        this->stereo->estimateEssential();
        bool success=this->stereo->essentialDecomposition();
        mutexDisp->post();

        if (success)
        {
            calibUpdated=true;
            doSFMOnce=false;
            calibEndEvent.signal();
        }
        else
        {
            if (++numberOfTrials>5)
            {
                calibUpdated=false;
                doSFMOnce=false;
                calibEndEvent.signal();
            }
        }
    }
    mutexRecalibration.unlock();

    mutexDisp->wait();
    this->stereo->computeDisparity(this->useBestDisp, this->uniquenessRatio, this->speckleWindowSize, this->speckleRange, this->numberOfDisparities, this->SADWindowSize, this->minDisparity, this->preFilterCap, this->disp12MaxDiff);
    mutexDisp->post();
    
   // DEBUG
 
   /*int uR,vR;
   Point3f point = this->get3DPointsAndDisp(160,120,uR,vR,"ROOT");
   circle(leftMat,cvPoint(160,120),2,cvScalar(255,0,0),2);
   circle(rightMat,cvPoint(uR,vR),2,cvScalar(0,255,0),2);
   */

    if(outMatch.getOutputCount()>0)
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

        Mat matches =this->stereo->drawMatches();
        cvtColor( matches, matches, CV_BGR2RGB);
        ImageOf<PixelBgr>& imgMatch= outMatch.prepare();
        imgMatch.resize(matches.cols, matches.rows);
        IplImage tmpR = matches;

        cvCopyImage( &tmpR, (IplImage *) imgMatch.getIplImage());       

        outMatch.write();
    }
            
    if(outDisp.getOutputCount()>0)
    {
        IplImage disp=stereo->getDisparity();
        cvCvtColor(&disp,outputD,CV_GRAY2RGB);
        ImageOf<PixelBgr>& outim=outDisp.prepare();
        outim.wrapIplImage(outputD);
        outDisp.write();
    }        
    

    if(worldPort.getOutputCount()>0)
    {
        ImageOf<PixelRgbFloat>& outim=worldPort.prepare(); 
        outim.resize(left->width,left->height);
        fillWorld3D(outim,0,0,left->width,left->height);
        worldPort.write();


    }

    return true;
}

double SFM::getPeriod()
{
    return 0.01;
}


bool SFM::loadExtrinsics(yarp::os::ResourceFinder &rf, Mat &Ro, Mat &T)
{

    
    Bottle extrinsics=rf.findGroup("STEREO_DISPARITY");
    if (Bottle *pXo=extrinsics.find("HN").asList()) {
        Ro=Mat::zeros(3,3,CV_64FC1);
        T=Mat::zeros(3,1,CV_64FC1);
        for (int i=0; i<(pXo->size()-4); i+=4) {
            Ro.at<double>(i/4,0)=pXo->get(i).asDouble();
            Ro.at<double>(i/4,1)=pXo->get(i+1).asDouble();
            Ro.at<double>(i/4,2)=pXo->get(i+2).asDouble();
            T.at<double>(i/4,0)=pXo->get(i+3).asDouble();
        }
    }
    else
    {
        doSFMOnce=true;
        return false;
    }
    doSFMOnce=false;
    return true;
}

bool SFM::loadIntrinsics(yarp::os::ResourceFinder &rf, Mat &KL, Mat &KR, Mat &DistL, Mat &DistR)
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

bool SFM::updateExtrinsics(Mat& Rot, Mat& Tr, const string& groupname)
{

    std::vector<string> lines;

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
                if (line.find("HN",0) != string::npos){
                    stringstream ss;
                    ss << " (" << Rot.at<double>(0,0) << " " << Rot.at<double>(0,1) << " " << Rot.at<double>(0,2) << " " << Tr.at<double>(0,0) << " "
                               << Rot.at<double>(1,0) << " " << Rot.at<double>(1,1) << " " << Rot.at<double>(1,2) << " " << Tr.at<double>(1,0) << " "
                               << Rot.at<double>(2,0) << " " << Rot.at<double>(2,1) << " " << Rot.at<double>(2,2) << " " << Tr.at<double>(2,0) << " "
                               << 0.0                 << " " << 0.0                 << " " << 0.0                 << " " << 1.0                << ")";
                    line = "HN" + string(ss.str());
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
            out << endl;
            out << string("[") + groupname + string("]") << endl;
            out << "HN (" << Rot.at<double>(0,0) << " " << Rot.at<double>(0,1) << " " << Rot.at<double>(0,2) << " " << Tr.at<double>(0,0) << " "
                          << Rot.at<double>(1,0) << " " << Rot.at<double>(1,1) << " " << Rot.at<double>(1,2) << " " << Tr.at<double>(1,0) << " "
                          << Rot.at<double>(2,0) << " " << Rot.at<double>(2,1) << " " << Rot.at<double>(2,2) << " " << Tr.at<double>(2,0) << " "
                          << 0.0                 << " " << 0.0                 << " " << 0.0                 << " " << 1.0                << ")";
            out.close();
        }
        else
            return false;
    }

    return true;
}

void SFM::setDispParameters(bool _useBestDisp, int _uniquenessRatio, int _speckleWindowSize,int _speckleRange, int _numberOfDisparities, int _SADWindowSize, int _minDisparity, int _preFilterCap, int _disp12MaxDiff)
{
    this->mutexDisp->wait();
    this->useBestDisp=_useBestDisp;
    this->uniquenessRatio=_uniquenessRatio;
    this->speckleWindowSize=_speckleWindowSize;
    this->speckleRange=_speckleRange;
    this->numberOfDisparities=_numberOfDisparities;
    this->SADWindowSize=_SADWindowSize;
    this->minDisparity=_minDisparity;
    this->preFilterCap=_preFilterCap;
    this->disp12MaxDiff=_disp12MaxDiff;
    this->mutexDisp->post();

}


Point3f SFM::get3DPointsAndDisp(int u, int v,  int& uR, int& vR, string drive ) {
   u=u; // matrix starts from (0,0), pixels from (0,0)
    v=v;
    Point3f point;

    if(drive!="RIGHT" && drive !="LEFT" && drive!="ROOT") {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        return point;
    }

    this->mutexDisp->wait();


    // Mapping from Rectified Cameras to Original Cameras
    Mat Mapper=this->stereo->getMapperL();

    if(Mapper.empty()) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        this->mutexDisp->post();
        return point;
    }


    float usign=Mapper.ptr<float>(v)[2*u];
    float vsign=Mapper.ptr<float>(v)[2*u+1];

    u=cvRound(usign);
    v=cvRound(vsign);


    IplImage disp16=this->stereo->getDisparity16();


    if(u<0 || u>=disp16.width || v<0 || v>=disp16.height) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        this->mutexDisp->post();
        return point;
    }

    Mat Q=this->stereo->getQ();
    CvScalar scal= cvGet2D(&disp16,v,u);
    double disparity=-scal.val[0]/16.0;
        
    uR=u+(int)disparity;
    vR=(int)v;


    Point2f orig= this->stereo->fromRectifiedToOriginal(uR,vR, RIGHT);
    uR= orig.x;
    vR= orig.y;


    float w= (float) ((float) disparity*Q.at<double>(3,2)) + ((float)Q.at<double>(3,3));
    point.x= (float)((float) (usign+1)*Q.at<double>(0,0)) + ((float) Q.at<double>(0,3));
    point.y=(float)((float) (vsign+1)*Q.at<double>(1,1)) + ((float) Q.at<double>(1,3));
    point.z=(float) Q.at<double>(2,3);

    point.x=point.x/w;
    point.y=point.y/w;
    point.z=point.z/w;

    // discard points far more than 10 meters or with not valid disparity (<0)
    if(point.z>10 || point.z<0) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        this->mutexDisp->post();
        return point;
    }

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

    this->mutexDisp->post();
    return point;


}


Point3f SFM::get3DPoints(int u, int v, string drive) {
    u=u; // matrix starts from (0,0), pixels from (0,0)
    v=v;
    Point3f point;

    if(drive!="RIGHT" && drive !="LEFT" && drive!="ROOT") {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        return point;
    }

    this->mutexDisp->wait();


    // Mapping from Rectified Cameras to Original Cameras
    Mat Mapper=this->stereo->getMapperL();

    if(Mapper.empty()) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        this->mutexDisp->post();
        return point;
    }


    float usign=Mapper.ptr<float>(v)[2*u];
    float vsign=Mapper.ptr<float>(v)[2*u+1];

    u=cvRound(usign);
    v=cvRound(vsign);


    IplImage disp16=this->stereo->getDisparity16();


    if(u<0 || u>=disp16.width || v<0 || v>=disp16.height) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        this->mutexDisp->post();
        return point;
    }

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

    // discard points far more than 10 meters or with not valid disparity (<0)
    if(point.z>10 || point.z<0) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        this->mutexDisp->post();
        return point;
    }

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

    this->mutexDisp->post();
    return point;

}


Point3f SFM::get3DPointMatch(double u1, double v1, double u2, double v2, string drive)
{
    Point3f point;
    if(drive!="RIGHT" && drive !="LEFT" && drive!="ROOT") {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        return point;
    }

    this->mutexDisp->wait();
    // Mapping from Rectified Cameras to Original Cameras
    Mat MapperL=this->stereo->getMapperL();
    Mat MapperR=this->stereo->getMapperR();

    if(MapperL.empty() || MapperR.empty()) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;

        this->mutexDisp->post();
        return point;
    }


    if(cvRound(u1)<0 || cvRound(u1)>=MapperL.cols || cvRound(v1)<0 || cvRound(v1)>=MapperL.rows) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        this->mutexDisp->post();
        return point;
    }
    
        if(cvRound(u2)<0 || cvRound(u2)>=MapperL.cols || cvRound(v2)<0 || cvRound(v2)>=MapperL.rows) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        this->mutexDisp->post();
        return point;
    }

    float urect1=MapperL.ptr<float>(cvRound(v1))[2*cvRound(u1)];
    float vrect1=MapperL.ptr<float>(cvRound(v1))[2*cvRound(u1)+1]; 

    float urect2=MapperR.ptr<float>(cvRound(v2))[2*cvRound(u2)];
    float vrect2=MapperR.ptr<float>(cvRound(v2))[2*cvRound(u2)+1]; 


    Mat Q=this->stereo->getQ();
    double disparity=urect2-urect1;
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
    this->mutexDisp->post();
    return point;
}

Mat SFM::buildRotTras(Mat & R, Mat & T) {
        
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

Matrix SFM::getCameraHGazeCtrl(int camera) {

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

    Matrix H_curr(4, 4);
    H_curr=R_curr;
    H_curr(0,3)=x_curr[0];
    H_curr(1,3)=x_curr[1];
    H_curr(2,3)=x_curr[2];

    if(camera==LEFT)
    {
        this->mutexDisp->wait();
        convert(H_curr,HL_root);
        this->mutexDisp->post();
    }
    else if(camera==RIGHT)
    {
        this->mutexDisp->wait();
        convert(H_curr,HR_root);
        this->mutexDisp->post();
    }


    return H_curr;
}

void SFM::convert(Matrix& matrix, Mat& mat) {
    mat=cv::Mat(matrix.rows(),matrix.cols(),CV_64FC1);
    for(int i=0; i<matrix.rows(); i++)
        for(int j=0; j<matrix.cols(); j++)
            mat.at<double>(i,j)=matrix(i,j);
}

void SFM::convert(Mat& mat, Matrix& matrix) {
    matrix.resize(mat.rows,mat.cols);
    for(int i=0; i<mat.rows; i++)
        for(int j=0; j<mat.cols; j++)
            matrix(i,j)=mat.at<double>(i,j);
}



bool SFM::respond(const Bottle& command, Bottle& reply) 
{
    if(command.size()==0)
        return false;

    if (command.get(0).asString()=="quit") {
        cout << "closing..." << endl;
        return false;
    }

    if(command.get(0).asString()=="stopSFM")
    {
        doSFM=false;
        reply.addString("ACK");
        return true;
    }

    if(command.get(0).asString()=="startSFM")
    {
        doSFM=true;
        reply.addString("ACK");
        return true;
    }

    if(command.get(0).asString()=="recalibrate")
    {
        mutexRecalibration.lock();
        numberOfTrials=0;
        doSFMOnce=true;
        mutexRecalibration.unlock();

        calibEndEvent.reset();
        calibEndEvent.wait();

        if (calibUpdated)
            reply.addString("ACK");
        else
            reply.addString("Calibration failed after 5 trials.. Please show a non planar scene.");

        return true;
    }

    if(command.get(0).asString()=="saveCurrentCalib")
    {
        Mat newR=this->stereo->getRotation();
        Mat newT= this->stereo->getTranslation();
        updateExtrinsics(newR, newT, "STEREO_DISPARITY"); 
        reply.addString("ACK");
        return true;               
    }
    if(command.get(0).asString()=="setNumDisp")
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
    if(command.get(0).asString()=="setMinDisp")
    {
        int dispNum=command.get(1).asInt();
        this->minDisparity=dispNum;
        this->setDispParameters(useBestDisp,uniquenessRatio,speckleWindowSize,speckleRange,numberOfDisparities,SADWindowSize,minDisparity,preFilterCap,disp12MaxDiff);
        reply.addString("ACK");
        return true;  
    }
    if(command.get(0).asString()=="set" && command.size()==10)
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

        this->setDispParameters(bestDisp,uniquenessRatio,speckleWindowSize,speckleRange,numberOfDisparities,SADWindowSize,minDisparity,preFilterCap,disp12MaxDiff);
    }
    else if (command.get(0).asString()=="Point" || command.get(0).asString()=="Left" ) {
        int u = command.get(1).asInt();
        int v = command.get(2).asInt(); 
        Point3f point = this->get3DPoints(u,v);
        reply.addDouble(point.x);
        reply.addDouble(point.y);
        reply.addDouble(point.z);
    }
    else if (!command.get(0).isString() && command.size()==2) {
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
    else if (command.get(0).asString()=="Right") {
        int u = command.get(1).asInt();
        int v = command.get(2).asInt();
        Point3f point = this->get3DPoints(u,v,"RIGHT");
        reply.addDouble(point.x);
        reply.addDouble(point.y);
        reply.addDouble(point.z);
    }

    else if (command.get(0).asString()=="Root") {
        int u = command.get(1).asInt();
        int v = command.get(2).asInt();
        Point3f point = this->get3DPoints(u,v,"ROOT");
        reply.addDouble(point.x);
        reply.addDouble(point.y);
        reply.addDouble(point.z);
    }

    else if (command.get(0).asString()=="cart2stereo") {
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

    else if(command.size()>0 && command.size()%4==0)
    {
        for(int i=0; i<command.size(); i+=4)
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

Point2f SFM::projectPoint(string camera, double x, double y, double z)
{
    Point3f point3D;
    point3D.x=x;
    point3D.y=y;
    point3D.z=z;

    vector<Point3f> points3D;

    points3D.push_back(point3D);

    vector<Point2f> response;

    this->mutexDisp->wait();

    if(camera=="left")
        response=this->stereo->projectPoints3D("left",points3D,HL_root);
    else
        response=this->stereo->projectPoints3D("right",points3D,HL_root);

    this->mutexDisp->post();

    return response[0];
}

void SFM::fillWorld3D(ImageOf<PixelRgbFloat> &worldImg, int u0, int v0, int width, int height)
{
    IplImage* img=(IplImage*) worldImg.getIplImage();
    for(int i=v0; i<(v0+height); i++)
    {
        for(int j=u0; j<(u0+width); j++)
        {
            
            Point3f point=get3DPoints(j,i,"ROOT");
            ((float *)(img->imageData + (i-v0)*img->widthStep))[(j-u0)*img->nChannels + 0]=point.x;
            ((float *)(img->imageData + (i-v0)*img->widthStep))[(j-u0)*img->nChannels + 1]=point.y;
            ((float *)(img->imageData + (i-v0)*img->widthStep))[(j-u0)*img->nChannels + 2]=point.z;
        }
    }
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
    rf.setDefaultContext("cameraCalibration");   
    rf.configure(argc,argv);

    SFM mod;

    return mod.runModule(rf);
}

