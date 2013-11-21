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

disparityThread::disparityThread(yarp::os::ResourceFinder &rf, Port* commPort)
{

    string moduleName = rf.check("name",Value("stereoDisparity"), "module name (string)").asString().c_str();
    this->inputLeftPortName = "/";
    this->inputLeftPortName += moduleName;
    this->inputLeftPortName += rf.check("InputPortLeft",Value("/cam/left:i"),"Input image port (string)").asString().c_str();
    
    Bottle pars=rf.findGroup("STEREO_DISPARITY");
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

    robotName = pars.check("robotName",Value("icub"), "module name (string)").asString().c_str();

    int cmdisp= rf.check("computeDisparity",Value(1)).asInt();
    int calib= rf.check("useCalibrated",Value(1)).asInt();

    this->computeDisparity= cmdisp ? true : false;
    this->useCalibrated= calib ? true : false;

    this->inputRightPortName ="/";
    this->inputRightPortName +=moduleName;
    this->inputRightPortName += rf.check("InputPortRight", Value("/cam/right:i"), "Input image port (string)").asString().c_str();

    this->inputLeftPortName ="/";
    this->inputLeftPortName +=moduleName;
    this->inputLeftPortName += rf.check("InputPortLeft", Value("/cam/left:i"), "Input image port (string)").asString().c_str();


    this->outputLeftRectName ="/";
    this->outputLeftRectName +=moduleName;
    this->outputLeftRectName += rf.check("OutputLeftRectified", Value("/leftRect:o"), "Input image port (string)").asString().c_str();

    this->outputRightRectName ="/";
    this->outputRightRectName +=moduleName;
    this->outputRightRectName += rf.check("OutputRightRectified", Value("/rightRect:o"), "Input image port (string)").asString().c_str();


    this->outName= "/";
    this->outName += moduleName;
    this->outName += rf.check("OutPort", Value("/disparity:o"), "Output image port (string)").asString().c_str();

    this->worldPortName="/";
    this->worldPortName+= moduleName;
    this->worldPortName+=rf.check("WorldOutPort", Value("/world:o"), "Output image port (string)").asString().c_str();

    this->boxPortName="/";
    this->boxPortName+=moduleName;
    this->boxPortName+=rf.check("BoxPort", Value("/box:i"), "Output image port (string)").asString().c_str();

    this->commandPort=commPort;

    this->stereo=new StereoCamera();

    Mat KL, KR, DistL, DistR, R, T;
    loadStereoParameters(rf,KL,KR,DistL,DistR,R,T);

    stereo->setIntrinsics(KL,KR,DistL,DistR);
    stereo->setRotation(R,0);
    stereo->setTranslation(T,0);

    this->useBestDisp=true;
    this->uniquenessRatio=15;
    this->speckleWindowSize=50;
    this->speckleRange=16;
    this->numberOfDisparities=64;
    this->SADWindowSize=7;
    this->minDisparity=0;
    this->preFilterCap=63;
    this->disp12MaxDiff=0;

    angle=0;
    this->mutexDisp = new Semaphore(1);
    this->HL_root= Mat::zeros(4,4,CV_64F);
    this->HR_root= Mat::zeros(4,4,CV_64F);
    this->output=NULL;

    if(useCalibrated)
    {
        Mat KL=this->stereo->getKleft();
        Mat KR=this->stereo->getKright();
        Mat zeroDist=Mat::zeros(1,8,CV_64FC1);
        this->stereo->setIntrinsics(KL,KR,zeroDist,zeroDist);
    }
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

    if (!outLeftRectPort.open(outputLeftRectName.c_str())) {
        cout << ": unable to open port " << outputLeftRectName << endl;
        return false;
    }

    if (!outRightRectPort.open(outputRightRectName.c_str())) {
        cout << ": unable to open port " << outputRightRectName << endl;
        return false;
    }

    if (!outPort.open(outName.c_str())) {
        cout << ": unable to open port " << outName << endl;
        return false;
    }

    
    if (!worldPort.open(worldPortName.c_str())) {
        cout << ": unable to open port " << worldPortName << endl;
        return false;
    }

    
    if (!boxPort.open(boxPortName.c_str())) {
        cout << ": unable to open port " << boxPortName << endl;
        return false;
    }

    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    option.put("local","/client/disparityClient");
    gazeCtrl=new PolyDriver(option);
    if (gazeCtrl->isValid()) {
        gazeCtrl->view(igaze);
    }
    else {
        cout<<"Devices not available"<<endl;
        return false;
    }

    Property optHead;
    optHead.put("device","remote_controlboard");
    optHead.put("remote",("/"+robotName+"/head").c_str());
    optHead.put("local","/disparityClient/head/position");
    if (polyHead.open(optHead))
    {
        polyHead.view(posHead);
        polyHead.view(HctrlLim);
    }
    else {
        cout<<"Devices not available"<<endl;
        return false;
    }

    Property optTorso;
    optTorso.put("device","remote_controlboard");
    optTorso.put("remote",("/"+robotName+"/torso").c_str());
    optTorso.put("local","/disparityClient/torso/position");

    if (polyTorso.open(optTorso))
    {
        polyTorso.view(posTorso);
        polyTorso.view(TctrlLim);
    }
    else {
        cout<<"Devices not available"<<endl;
        return false;
    }

    Bottle p;
    igaze->getInfo(p);
    int vHead=(int)p.check(("head_version"),Value(1.0)).asDouble();

    stringstream headType;
    headType << "v";
    headType << vHead;
    
    LeyeKin=new iCubEye(("left_"+headType.str() ).c_str());
    ReyeKin=new iCubEye(("right_"+headType.str() ).c_str());
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
Matrix disparityThread::getCameraH(yarp::sig::Vector &head_angles, yarp::sig::Vector &torso_angles, iCubEye *eyeKin, int camera)
{

    yarp::sig::Vector q(torso_angles.size()+head_angles.size()-1);

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
Matrix disparityThread::getCameraHGazeCtrl(int camera) {

    yarp::sig::Vector x_curr;
    yarp::sig::Vector o_curr;

    if(camera==LEFT)
        igaze->getLeftEyePose(x_curr, o_curr);
    else
        igaze->getRightEyePose(x_curr, o_curr);

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

void disparityThread::run(){

    Matrix yarp_initLeft,yarp_initRight;
    Matrix yarp_H0;

    Point3f point;
    bool init=true;
    while (!isStopping()) {
        ImageOf<PixelRgb> *imageL = imagePortInLeft.read();
        ImageOf<PixelRgb> *imageR = imagePortInRight.read();

        if(imageL!=NULL && imageR!=NULL){
            imgL= (IplImage*) imageL->getIplImage();
            imgR= (IplImage*) imageR->getIplImage();

            this->stereo->setImages(imgL,imgR);

            if(init) {
                /*yarp::sig::Vector headAngles(6);
                yarp::sig::Vector torsoAngles(3);
                torsoAngles=0.0;                
                headAngles=0.0;
                
                posTorso->getEncoders(torsoAngles.data());
                posHead->getEncoders(headAngles.data());
                
                yarp_initLeft=getCameraH(headAngles,torsoAngles,LeyeKin,LEFT);
                yarp_initRight=getCameraH(headAngles,torsoAngles,ReyeKin,RIGHT);
                
                stereo->undistortImages();
                stereo->findMatch();
                stereo->estimateEssential();
                stereo->hornRelativeOrientations();*/
                
                
                yarp_initLeft=LeyeKin->getH(QL);
                yarp_initRight=ReyeKin->getH(QR);
                
                /*cout << "QL CALIB" << endl << QL.toString(5,5) << endl;
                cout << "POS TORSO" << endl << torsoAngles.toString() << endl;
                cout << "POS HEAD" << endl << headAngles.toString() << endl;*/




                output=cvCreateImage(cvSize(imgL->width,imgL->height),8,3);
                outputWorld=cvCreateImage(cvSize(imgL->width,imgL->height),32,3);
                init=false;

                Mat H0_R=this->stereo->getRotation();
                Mat H0_T=this->stereo->getTranslation();

                // get the initial camera relative position
                Mat H0=buildRotTras(H0_R,H0_T);
                convert(H0,yarp_H0);


                //get the initial left and right positions
                /*headAngles=0.0;
                headAngles[5]=vergence_init;
                headAngles[4]=version_init;*/



                /*yarp_initLeft=getCameraHGazeCtrl(LEFT);
                yarp_initRight=getCameraHGazeCtrl(RIGHT);
                
                Matrix DiffL=yarp_initLeft-yarp_initLeftE;
                Matrix DiffR=yarp_initRight-yarp_initRightE;
                printMatrixYarp(DiffL);
                printMatrixYarp(DiffR);*/
  

            }
           

                //transformation matrices between prev and curr eye frames
                yarp::sig::Vector headAngles(6);
                headAngles=0.0;
                posHead->getEncoders(headAngles.data());
                yarp::sig::Vector torsoAngles(3);
                torsoAngles=0.0;
                posTorso->getEncoders(torsoAngles.data());

                Matrix yarp_Left=getCameraH(headAngles,torsoAngles,LeyeKin,LEFT);
                Matrix yarp_Right=getCameraH(headAngles,torsoAngles,ReyeKin,RIGHT);
                
                //Matrix yarp_Left=getCameraHGazeCtrl(LEFT);
                //Matrix yarp_Right=getCameraHGazeCtrl(RIGHT); 

               /* Matrix yarp_LeftG=getCameraHGazeCtrl(LEFT);
                Matrix DiffM=yarp_Left-yarp_LeftG;
                printMatrixYarp(DiffM);*/



                yarp_Left=SE3inv(yarp_Left)*yarp_initLeft; // Left eye transformation between time t0 and t
                yarp_Right=SE3inv(yarp_Right)*yarp_initRight; // Right eye transformation between time t0 and t

                Matrix Hcurr=yarp_Right*yarp_H0*SE3inv(yarp_Left); // Transformation from Left to Right eye at time t
                
                //printMatrixYarp(Hcurr);

                Matrix R=Hcurr.submatrix(0,2,0,2);
                Matrix newTras=Hcurr.submatrix(0,2,3,3);

                this->mutexDisp->wait();
                // Update Rotation
                Mat Rot(3,3,CV_64FC1);
                convert(R,Rot);
                //this->stereo->setRotation(Rot,0);

                //Update Translation
                Mat translation(3,1,CV_64FC1);
                convert(newTras,translation);
                //this->stereo->setTranslation(translation,0);
                if(this->computeDisparity)
                    this->stereo->computeDisparity(this->useBestDisp, this->uniquenessRatio, this->speckleWindowSize, this->speckleRange, this->numberOfDisparities, this->SADWindowSize, this->minDisparity, this->preFilterCap, this->disp12MaxDiff);
                else
                    this->stereo->rectifyImages();

                this->mutexDisp->post();

                if(outPort.getOutputCount()>0 && this->computeDisparity) {
                    disp=stereo->getDisparity();
                    cvCvtColor(&disp,output,CV_GRAY2RGB);
                    ImageOf<PixelBgr>& outim=outPort.prepare();
                    outim.wrapIplImage(output);
                    outPort.write();
                }

                if(outLeftRectPort.getOutputCount() && outRightRectPort.getOutputCount())
                {
                    IplImage Lrect=stereo->getLRectified();
                    ImageOf<PixelBgr>& outimL=outLeftRectPort.prepare();
                    outimL.wrapIplImage(&Lrect);
                    outLeftRectPort.write();

                    IplImage Rrect=stereo->getRRectified();
                    ImageOf<PixelBgr>& outimR=outRightRectPort.prepare();
                    outimR.wrapIplImage(&Lrect);
                    outRightRectPort.write();

                }

                if(worldPort.getOutputCount()>0 && this->computeDisparity)
                {
                    ImageOf<PixelRgbFloat>& outim=worldPort.prepare();

                    if(boxPort.getInputCount()>0)
                    {
                        Bottle *b =(Bottle *) boxPort.read(false);
                        if(b!=NULL)
                        {
                            int u0=b->get(0).asInt();
                            int v0=b->get(1).asInt();
                            int width=b->get(2).asInt();
                            int height=b->get(3).asInt();
                            outim.resize(width,height);
                            fillWorld3D(outim,u0,v0,width,height);
                            worldPort.write();
                        }
                    }
                    else
                    {
                        outim.resize(imgL->width,imgL->height);
                        fillWorld3D(outim,0,0,imgL->width,imgL->height);
                        worldPort.write();
                    }

                }
        }

    }
}


void disparityThread::threadRelease() 
{
    imagePortInRight.close();
    imagePortInLeft.close();
    outPort.close();
    worldPort.close();
    boxPort.close();
    outRightRectPort.close();
    commandPort->close();
    outRightRectPort.close();

    delete this->stereo;
    delete this->mutexDisp;
    
    delete gazeCtrl;
    delete LeyeKin;
    delete ReyeKin;

    if (polyHead.isValid())
        polyHead.close();

    if (polyTorso.isValid())
        polyTorso.close();
    if(output!=NULL)
        cvReleaseImage(&output);
    if(outputWorld!=NULL)
        cvReleaseImage(&outputWorld);

 


}
void disparityThread::onStop() {
    imagePortInRight.interrupt();
    imagePortInLeft.interrupt();
    commandPort->interrupt();
    worldPort.interrupt();
    boxPort.interrupt();
    outRightRectPort.interrupt();
    outLeftRectPort.interrupt();
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
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        return point;
    }

    this->mutexDisp->wait();

    if(!computeDisparity)
        this->stereo->computeDisparity();

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


    if(u<0 || u>=disp.width || v<0 || v>=disp.height) {
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

    // discard points far more than 2.5 meters or with not valid disparity (<0)
    if(point.z>2.5 || point.z<0) {
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


Point3f disparityThread::get3DPointMatch(double u1, double v1, double u2, double v2, string drive)
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

Point2f disparityThread::projectPoint(string camera, double x, double y, double z)
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

void disparityThread::compute(bool compute)
{ 
    this->computeDisparity=compute;
}

bool disparityThread::isComputing()
{ 
    return this->computeDisparity;
}

void disparityThread::fillWorld3D(ImageOf<PixelRgbFloat> &worldImg, int u0, int v0, int width, int height)
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
bool disparityThread::loadStereoParameters(yarp::os::ResourceFinder &rf, Mat &KL, Mat &KR, Mat &DistL, Mat &DistR, Mat &Ro, Mat &T)
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
void disparityThread::setDispParameters(bool _useBestDisp, int _uniquenessRatio, int _speckleWindowSize,int _speckleRange, int _numberOfDisparities, int _SADWindowSize, int _minDisparity, int _preFilterCap, int _disp12MaxDiff)
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
