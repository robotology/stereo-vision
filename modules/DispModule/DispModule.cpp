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
#include <yarp/cv/Cv.h>
#include "DispModule.h"

// --- DEBUG ---
// this code left on purpose, it was used to carry out profiling
// of the different subroutines in the code
//
// --- SAMPLE USE ---
//
// have a "debug_count" flag and a "debug_timings[]"" array stored stored in your main loop, starting at 0, 
// then for every function to be profiled use:
//
//    PROF_S
//    method_to_be_profiled()
//    PROF_E(<number>)
//
// then you can print every 100 iterations in a way similar to:
//
//    if(++debug_count == 100)
//    {
//        std::cout << std::endl << "---- DISPMODULE: AVERAGE TIMING OVER " << debug_count << " FRAMES ----" << std::endl;
//        for(int i = 0; i < this->debug_num_timings; i++)
//        {
//            std::cout << debug_strings[i] << ": " << (this->debug_timings[i] / debug_count) / 1000. << " ms" << std::endl;
//            this->debug_timings[i] = 0;
//        }
//        debug_count = 0;
//    }
//
// the "debug_strings[]" array stores the strings associated with 
// the different <number> values used in PROF_E(<number>)
//
//#include <chrono>
//#define PROF_S {start = std::chrono::high_resolution_clock::now();}
//#define PROF_E(N) {stop = std::chrono::high_resolution_clock::now();duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); this->debug_timings[N] += duration.count();}
//
// char const *debug_strings[] = {
//     "updateViaKinematics",
//     "updateViaGazeCtrl",
//     "getCameraHGazkillGUIeCtrlx2",
//     "gui.recalibrate",
//     "runRecalibration",
//     "gui.isUpdated",
//     "computeDisparity",
//     "bilateralFilter",
//     "wlsFilter",
//     "ouputDisparity",
//     "outputDepth"
// };

using namespace yarp::cv;


void DispModule::printParameters()
{
    std::cout << "params ("
              << this->stereo_parameters.minDisparity << " "
              << this->useBestDisp << " "
              << this->stereo_parameters.numberOfDisparities << " "
              << this->stereo_parameters.SADWindowSize << " "
              << this->stereo_parameters.disp12MaxDiff << " "
              << this->stereo_parameters.preFilterCap << " "
              << this->stereo_parameters.uniquenessRatio << " "
              << this->stereo_parameters.speckleWindowSize << " "
              << this->stereo_parameters.speckleRange << " "
              << this->stereo_parameters.sigmaColorBLF << " "
              << this->stereo_parameters.sigmaSpaceBLF << " "
              << this->stereo_parameters.wls_lambda << " "
              << this->stereo_parameters.wls_sigma << " "
              << this->stereo_parameters.stereo_matching << " "
              << this->stereo_parameters.BLFfiltering << " "
              << this->stereo_parameters.WLSfiltering
              << ")" << std::endl;
}



bool DispModule::configure(ResourceFinder & rf)
{
    // populate local variables with infos from the context

    string name=rf.check("name",Value("DisparityModule")).asString();
    string robot=rf.check("robot",Value("icub")).asString();
    string left=rf.check("leftPort",Value("/left:i")).asString();
    string right=rf.check("rightPort",Value("/right:i")).asString();
    string SFMFile=rf.check("SFMFile",Value("SFM.ini")).asString();

    string sname="/"+name;
    left=sname+left;
    right=sname+right;

    // set up output disparity and depth ports

    string outDispName=rf.check("outDispPort",Value("/disp:o")).asString();
    string outDepthName=rf.check("outDepthPort",Value("/depth:o")).asString();

    outDepthName=sname+outDepthName;
    outDispName=sname+outDispName;

    // load the calibration infos from file

    this->localCalibration.setDefaultContext("cameraCalibration");
    this->localCalibration.setDefaultConfigFile(SFMFile.c_str());
    this->localCalibration.configure(0,NULL);

    this->camCalibFile=this->localCalibration.getHomeContextPath();
    this->camCalibFile+="/"+SFMFile;

    // prepares the name for the RPC port

    string rpc_name=sname+"/rpc";

    // open the ports

    leftImgPort.open(left);
    rightImgPort.open(right);
    outDepth.open(outDepthName);
    outDisp.open(outDispName);
    handlerPort.open(rpc_name);
    attach(handlerPort);

    // create the stereocamera and load the
    // intrinsic/extrinsic parameters

    this->stereo = new StereoCamera(true);

    Mat KL, KR, DistL, DistR;

    loadIntrinsics(rf,KL,KR,DistL,DistR);
    loadConfigurationFile(this->localCalibration,R0,T0,eyes0);

    // stores the original parameters 

    this->original_parameters = this->stereo_parameters;

    // auxiliary initialization calls

    eyes.resize(eyes0.length(),0.0);

    stereo->setIntrinsics(KL,KR,DistL,DistR);

    // initialize the matrices describing the
    // transformation between reference systems

    this->HL_root=Mat::zeros(4,4,CV_64F);
    this->HR_root=Mat::zeros(4,4,CV_64F);

    bool useCalibrated = rf.check("useCalibrated");

    // TODO: check whether this part next is needed or not

    if (useCalibrated)
    {
        Mat KL=this->stereo->getKleft();
        Mat KR=this->stereo->getKright();
        Mat zeroDist=Mat::zeros(1,8,CV_64FC1);
        this->stereo->setIntrinsics(KL,KR,zeroDist,zeroDist);
    }

    // auxiliary variables for the calibration process

    this->init=true;
    this->numberOfTrials=0;

#ifdef USING_GPU
    utils=new Utilities();
    utils->initSIFT_GPU();
#endif

    Property optionHead;
    optionHead.put("device","remote_controlboard");
    optionHead.put("remote","/"+robot+"/head");
    optionHead.put("local",sname+"/headClient");

    if (headCtrl.open(optionHead))
    {
        headCtrl.view(iencs);
        iencs->getAxes(&nHeadAxes);
    }
    else
    {
        std::cout << "[DisparityModule] Devices not available" << std::endl;
        return false;
    }

    // set up to interface with the GazeController

    Property optionGaze;
    optionGaze.put("device","gazecontrollerclient");
    optionGaze.put("remote","/iKinGazeCtrl");
    optionGaze.put("local",sname+"/gazeClient");
    if (gazeCtrl.open(optionGaze))
        gazeCtrl.view(igaze);
    else
    {
        std::cout << "[DisparityModule] Devices not available" << std::endl;
        headCtrl.close();
        return false;
    }

    // set up the translation and rotation parameters of the stereo system

    if (!R0.empty() && !T0.empty())
    {
        stereo->setRotation(R0,0);
        stereo->setTranslation(T0,0);
    }
    else
    {
        std::cout << "[DisparityModule] No local calibration file found in " <<  camCalibFile
             << "[DisparityModule] ... Using Kinematics and Running SFM once." << std::endl;
        updateViaGazeCtrl(true);
        R0=this->stereo->getRotation();
        T0=this->stereo->getTranslation();
    }

    this->runRecalibration=false;
    this->updateViaGazeCtrl(false);

    // select the stereo matching algorithm, filtering
    // methods on the basis of the parameters the
    // module has been invoked with, in case the user 
    // prefers to specify it via command line and not 
    // using the GUI

   if (rf.check("sgbm"))
       this->stereo_parameters.stereo_matching = SM_MATCHING_ALG::SGBM_OPENCV;
   else if(rf.check("sgbm_cuda"))
       this->stereo_parameters.stereo_matching = SM_MATCHING_ALG::SGBM_CUDA;
   else if(rf.check("libelas"))
       this->stereo_parameters.stereo_matching = SM_MATCHING_ALG::LIBELAS;

   if (rf.check("blf"))
       this->stereo_parameters.BLFfiltering = SM_BLF_FILTER::BLF_ORIGINAL;
   else if(rf.check("blf_cuda"))
       this->stereo_parameters.BLFfiltering = SM_BLF_FILTER::BLF_CUDA;
   else if(rf.check("no_blf"))
       this->stereo_parameters.BLFfiltering = SM_BLF_FILTER::BLF_DISABLED;

   if (rf.check("wls"))
       this->stereo_parameters.WLSfiltering = SM_WLS_FILTER::WLS_ENABLED;
   else if(rf.check("wls_lr"))
       this->stereo_parameters.WLSfiltering = SM_WLS_FILTER::WLS_LRCHECK;
   else if(rf.check("no_wls"))
       this->stereo_parameters.WLSfiltering = SM_WLS_FILTER::WLS_DISABLED;

    // if specified via CMake, compile using the GUI
    // based on cvui.h and here checks whether to initialize
    // it or not

#ifdef USE_GUI
    this->debugWindow = rf.check("gui");

    if(this->debugWindow)
    {
        this->gui.initializeGUI(this->stereo_parameters.minDisparity,
                                this->stereo_parameters.numberOfDisparities,
                                this->stereo_parameters.SADWindowSize,
                                this->stereo_parameters.disp12MaxDiff,
                                this->stereo_parameters.preFilterCap,
                                this->stereo_parameters.uniquenessRatio,
                                this->stereo_parameters.speckleWindowSize,
                                this->stereo_parameters.speckleRange,
                                this->stereo_parameters.sigmaColorBLF,
                                this->stereo_parameters.sigmaSpaceBLF,
                                this->stereo_parameters.wls_lambda,
                                this->stereo_parameters.wls_sigma,
                                this->stereo_parameters.BLFfiltering,
                                this->stereo_parameters.WLSfiltering,
                                this->stereo_parameters.stereo_matching);
    }
#endif

    // initialize the StereoMatcher object

    this->matcher = new StereoMatcherNew(rf, this->stereo);

    this->matcher->setParameters(this->stereo_parameters.minDisparity,
                                 this->stereo_parameters.numberOfDisparities,
                                 this->stereo_parameters.SADWindowSize,
                                 this->stereo_parameters.disp12MaxDiff,
                                 this->stereo_parameters.preFilterCap,
                                 this->stereo_parameters.uniquenessRatio,
                                 this->stereo_parameters.speckleWindowSize,
                                 this->stereo_parameters.speckleRange,
                                 this->stereo_parameters.sigmaColorBLF,
                                 this->stereo_parameters.sigmaSpaceBLF,
                                 this->stereo_parameters.wls_lambda,
                                 this->stereo_parameters.wls_sigma,
                                 this->stereo_parameters.BLFfiltering,
                                 this->stereo_parameters.WLSfiltering,
                                 this->stereo_parameters.stereo_matching);

    this->matcher->updateCUDAParams();

    return true;
}


cv::Mat DispModule::depthFromDisparity(Mat disparity, Mat Q, Mat R)
{
    // compute the depth from the disparity map

    cv::Mat depth;

    depth = disparity.clone();

    depth.convertTo(depth, CV_32FC1, 1./16.);

    // extract the subelements of the R and Q matrices, 
    // to simplify the further calculations

    float r02 = float(R.at<double>(0,2));
    float r12 = float(R.at<double>(1,2));
    float r22 = float(R.at<double>(2,2));

    float q00 = float(Q.at<double>(0,0));
    float q03 = float(Q.at<double>(0,3));
    float q11 = float(Q.at<double>(1,1));
    float q13 = float(Q.at<double>(1,3));
    float q23 = float(Q.at<double>(2,3));
    float q32 = float(Q.at<double>(3,2));
    float q33 = float(Q.at<double>(3,3));

    const Mat& Mapper=this->stereo->getMapperL();

    if (Mapper.empty())
        return depth;

    float usign, vsign;

    for (int u = 0; u < depth.rows; u++)
        for (int v = 0; v < depth.cols; v++)
        {
            usign=cvRound(Mapper.ptr<float>(u)[2*v]);
            vsign=cvRound(Mapper.ptr<float>(u)[2*v+1]);

            if ((usign<0) || (usign>=depth.cols) || (vsign<0) || (vsign>=depth.rows))
                depth.at<float>(u,v) = 0.0;
            else
                depth.at<float>(u,v) = (r02*(float(vsign)*q00+q03)+r12*(float(usign)*q11+q13)+r22*q23)/(depth.at<float>(vsign,usign)*q32+q33);

        }

    return depth;
}


void DispModule::initializeStereoParams()
{
    this->useBestDisp=true;

    this->stereo_parameters.uniquenessRatio=15;
    this->stereo_parameters.speckleWindowSize=50;
    this->stereo_parameters.speckleRange=16;
    this->stereo_parameters.SADWindowSize=7;
    this->stereo_parameters.minDisparity=0;
    this->stereo_parameters.preFilterCap=63;
    this->stereo_parameters.disp12MaxDiff=0;

    this->stereo_parameters.numberOfDisparities = 96;

    this->stereo_parameters.sigmaColorBLF = 10.0;
    this->stereo_parameters.sigmaSpaceBLF = 10.0;

    this->stereo_parameters.wls_lambda = 4500.;
    this->stereo_parameters.wls_sigma = 1.0;

    this->original_parameters = this->stereo_parameters;
}


void DispModule::updateViaKinematics(const yarp::sig::Vector& deyes)
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



void DispModule::updateViaGazeCtrl(const bool update)
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



bool DispModule::interruptModule()
{
    leftImgPort.interrupt();
    rightImgPort.interrupt();
    outDisp.interrupt();
    outDepth.interrupt();
    handlerPort.interrupt();

    return true;
}



bool DispModule::close()
{
    leftImgPort.close();
    rightImgPort.close();
    outDisp.close();
    outDepth.close();
    handlerPort.close();

    headCtrl.close();
    gazeCtrl.close();

    delete stereo;

#ifdef USING_GPU
    delete utils;
#endif

#ifdef USE_GUI
    delete &gui;
#endif

    return true;
}


void DispModule::recalibrate()
{

#ifdef USING_GPU
        utils->extractMatch_GPU(leftMat,rightMat);
        vector<Point2f> leftM,rightM;
        utils->getMatches(leftM,rightM);
        mutexDisp.lock();
        this->stereo->setMatches(leftM,rightM);
#else

        mutexDisp.lock();
        stereo->findMatch(false);
#endif

        stereo->estimateEssential();
        bool ok=stereo->essentialDecomposition();
        mutexDisp.unlock();

        // if the calibration process has been successfull,
        // stores the parameters estimated, otherwise stops the
        // calibration process in case it has failed for five
        // times in a row

        if (ok)
        {
            this->calibUpdated=true;
            this->runRecalibration=false;
            calibEndEvent.signal();

            R0=stereo->getRotation();
            T0=stereo->getTranslation();
            eyes0=eyes;

            std::cout << "[DisparityModule] Calibration Successful!" << std::endl;

        }
        else
        {
            if (++numberOfTrials>5)
            {
                this->calibUpdated=false;
                this->runRecalibration=false;
                calibEndEvent.signal();

                std::cout << "[DisparityModule] Calibration failed after 5 trials.." << std::endl <<
                             "[DisparityModule] ..please show a non planar scene." << std::endl;

            }
        }
}


#ifdef USE_GUI
void DispModule::handleGuiUpdate()
{

    // the calls to gui.set/getParameters and to 
    // matcher->set/getParameters() can be definitely
    // improved, by wrapping everything in a Params struct
    // as defined in StereoMatcher.h, this has to be implemented

    if(this->gui.toRecalibrate())
    {
        std::cout << "[DisparityModule] Updating calibration.." << std::endl;

        mutexRecalibration.lock();
        numberOfTrials=0;
        this->runRecalibration=true;
        mutexRecalibration.unlock();

    }
    else if(this->gui.toSaveCalibration())
    {
        std::cout << "[DisparityModule] Saving current calibration.." << std::endl;
        updateConfigurationFile(R0,T0,eyes0,"STEREO_DISPARITY");
    }
    else if(this->gui.toLoadParameters())
    {
        std::cout << "[DisparityModule] Loading stereo parameters from file.." << std::endl;

        this->stereo_parameters = this->original_parameters;

        this->gui.setParameters(this->stereo_parameters.minDisparity,
                               this->stereo_parameters.numberOfDisparities,
                               this->stereo_parameters.SADWindowSize,
                               this->stereo_parameters.disp12MaxDiff,
                               this->stereo_parameters.preFilterCap,
                               this->stereo_parameters.uniquenessRatio,
                               this->stereo_parameters.speckleWindowSize,
                               this->stereo_parameters.speckleRange,
                               this->stereo_parameters.sigmaColorBLF,
                               this->stereo_parameters.sigmaSpaceBLF,
                               this->stereo_parameters.wls_lambda,
                               this->stereo_parameters.wls_sigma,
                               this->stereo_parameters.BLFfiltering,
                               this->stereo_parameters.WLSfiltering,
                               this->stereo_parameters.stereo_matching);

        this->matcher->setParameters(this->stereo_parameters.minDisparity,
                                     this->stereo_parameters.numberOfDisparities,
                                     this->stereo_parameters.SADWindowSize,
                                     this->stereo_parameters.disp12MaxDiff,
                                     this->stereo_parameters.preFilterCap,
                                     this->stereo_parameters.uniquenessRatio,
                                     this->stereo_parameters.speckleWindowSize,
                                     this->stereo_parameters.speckleRange,
                                     this->stereo_parameters.sigmaColorBLF,
                                     this->stereo_parameters.sigmaSpaceBLF,
                                     this->stereo_parameters.wls_lambda,
                                     this->stereo_parameters.wls_sigma,
                                     this->stereo_parameters.BLFfiltering,
                                     this->stereo_parameters.WLSfiltering,
                                     this->stereo_parameters.stereo_matching);

        this->matcher->updateCUDAParams();
    }
    else if(this->gui.toSaveParameters())
    {
        std::cout << "[DisparityModule] Saving stereo parameters.." << std::endl;
        updateConfigurationFile(R0,T0,eyes0,"STEREO_DISPARITY");
    }
    else
    {

        this->gui.getParameters(this->stereo_parameters.minDisparity,
                                this->stereo_parameters.numberOfDisparities,
                                this->stereo_parameters.SADWindowSize,
                                this->stereo_parameters.disp12MaxDiff,
                                this->stereo_parameters.preFilterCap,
                                this->stereo_parameters.uniquenessRatio,
                                this->stereo_parameters.speckleWindowSize,
                                this->stereo_parameters.speckleRange,
                                this->stereo_parameters.sigmaColorBLF,
                                this->stereo_parameters.sigmaSpaceBLF,
                                this->stereo_parameters.wls_lambda,
                                this->stereo_parameters.wls_sigma,
                                this->stereo_parameters.BLFfiltering,
                                this->stereo_parameters.WLSfiltering,
                                this->stereo_parameters.stereo_matching);

        this->matcher->setParameters(this->stereo_parameters.minDisparity,
                                     this->stereo_parameters.numberOfDisparities,
                                     this->stereo_parameters.SADWindowSize,
                                     this->stereo_parameters.disp12MaxDiff,
                                     this->stereo_parameters.preFilterCap,
                                     this->stereo_parameters.uniquenessRatio,
                                     this->stereo_parameters.speckleWindowSize,
                                     this->stereo_parameters.speckleRange,
                                     this->stereo_parameters.sigmaColorBLF,
                                     this->stereo_parameters.sigmaSpaceBLF,
                                     this->stereo_parameters.wls_lambda,
                                     this->stereo_parameters.wls_sigma,
                                     this->stereo_parameters.BLFfiltering,
                                     this->stereo_parameters.WLSfiltering,
                                     this->stereo_parameters.stereo_matching);

        this->matcher->updateCUDAParams();

    }

    this->gui.resetState();
}
#endif


bool DispModule::updateModule()
{
    // acquire the left and right images

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

    // updated the internal state by reading the information
    // from the kinematics of the robot and from the GazeController

    updateViaKinematics(eyes-eyes0);
    updateViaGazeCtrl(false);

    leftMat=toCvMat(*yarp_imgL);
    rightMat=toCvMat(*yarp_imgR);

    // updates left/right eyes pose

    getCameraHGazeCtrl(LEFT);
    getCameraHGazeCtrl(RIGHT);

    // stores the images in the StereoCamera object

    this->stereo->setImages(leftMat,rightMat);

#ifdef USE_GUI
    if(this->debugWindow && this->gui.isUpdated())
        this->handleGuiUpdate();
#endif

    mutexRecalibration.lock();

    if(this->runRecalibration)
        this->recalibrate();

    mutexRecalibration.unlock();

    // compute the current disparity map

    mutexDisp.lock();

    matcher->compute();
    
    mutexDisp.unlock();

    // execute the bilateral filtering of the
    // disparity map, if selected

    matcher->filterBLF("base");

    // execute the WLS filtering, if selected

    matcher->filterWLS("blf");

    // if the disparity output port is active, sends
    // the current (filtered) one

    if (outDisp.getOutputCount() > 0)
    {

        // here, gets the disparity map after the WLS filtering, 
        // which does not imply that it has been actually been 
        // filtered in that way, it's just a convention so to 
        // get the "most" filtered disparity map available, on the 
        // basis of the selection of parameters by the user

        cv::Mat disp_vis = matcher->getDisparity("wls");

        if(!disp_vis.empty())
        {

#ifdef USE_GUI
            if(gui.toRefine())
           {
                orig = disp_vis.clone();

                if(!old_d.empty())
                    disp_vis = refineDisparity(old_d, disp_vis, gui.getRefineTh());

                old_d = orig.clone();
            }
#endif

            ImageOf<PixelMono> &outim = outDisp.prepare();

            outim = fromCvMat<PixelMono>(disp_vis);
            outDisp.write();
        }
    }

    // if the output port is active, computes the depth
    // map for the current disparity map (the filtered one,
    // in case the bilateral filtering is selected)

    if (outDepth.getOutputCount() > 0)
    {
        cv::Mat disp_depth = matcher->getDisparity16("wls");

        if (disp_depth.empty())
        {
            std::cout << "[DisparityModule] Impossible to compute the depth map.." << std::endl <<
                         "[DisparityModule] ..the disparity map is not available!" << std::endl;
        }
        else
        {

#ifdef USE_GUI
            if(gui.toRefine())
            {
                orig = disp_depth.clone();

                if(!old_de.empty())
                    disp_depth = refineDisparity(old_de, disp_depth, gui.getRefineTh());

                old_de = orig.clone();
            }
#endif

            outputDepth = this->depthFromDisparity(disp_depth, this->stereo->getQ(), this->stereo->getRLrect());

            ImageOf<PixelFloat> &outim = outDepth.prepare();

            outim = fromCvMat<PixelFloat>(outputDepth);

            outDepth.write();
        }
    }

    // if the GUI is used, handles the interaction
    // with it and updates its state

#ifdef USE_GUI
    if(this->debugWindow && !this->gui.isDone())
        this->gui.updateGUI();

    if(this->gui.isDone())
        this->gui.killGUI();
#endif

    return true;
}


double DispModule::getPeriod()
{
    // the updateModule() method gets synchronized
    // with camera input => no need for periodicity
    return 0.0;
}


bool DispModule::loadExtrinsics(yarp::os::ResourceFinder& rf, Mat& Ro, Mat& To, yarp::sig::Vector& eyes)
{
    Bottle extrinsics=rf.findGroup("STEREO_DISPARITY");

    eyes.resize(3,0.0);
    if (Bottle *bEyes=extrinsics.find("eyes").asList())
    {
        size_t sz=std::min(eyes.length(),(size_t)bEyes->size());
        for (size_t i=0; i<sz; i++)
            eyes[i]=bEyes->get(i).asDouble();
    }

    std::cout << "[DisparityModule] Read Eyes Configuration = ("<<eyes.toString(3,3)<<")" << std::endl;

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


bool DispModule::loadConfigurationFile(yarp::os::ResourceFinder& rf, Mat& Ro, Mat& To, yarp::sig::Vector& eyes)
{

    Bottle extrinsics=rf.findGroup("STEREO_DISPARITY");

    // loads the eyes configuration from file

    eyes.resize(3,0.0);
    if (Bottle *bEyes=extrinsics.find("eyes").asList())
    {
        size_t sz = std::min(eyes.length(),(size_t)bEyes->size());
        for (size_t i=0; i<sz; i++)
            eyes[i] = bEyes->get(i).asDouble();
    }

    std::cout << "[DisparityModule] Read Eyes Configuration = ("
         << eyes.toString(3,3) << ")" << std::endl;

    // loads the calibration matrix associated with the
    // stereo system

    if (Bottle *pXo=extrinsics.find("HN").asList())
    {
        Ro=Mat::zeros(3,3,CV_64FC1);
        To=Mat::zeros(3,1,CV_64FC1);
        for (int i=0; i<(pXo->size()-4); i+=4)
        {
            Ro.at<double>(i/4,0) = pXo->get(i).asDouble();
            Ro.at<double>(i/4,1) = pXo->get(i+1).asDouble();
            Ro.at<double>(i/4,2) = pXo->get(i+2).asDouble();
            To.at<double>(i/4,0) = pXo->get(i+3).asDouble();
        }
    }
    else
        return false;

    // loads the stereo parameters from the configuration file

    if (Bottle *pXo=extrinsics.find("params").asList())
    {

        this->stereo_parameters.minDisparity =        pXo->get(0).asInt();
        this->useBestDisp =                           pXo->get(1).asBool();
        this->stereo_parameters.numberOfDisparities = pXo->get(2).asInt();
        this->stereo_parameters.SADWindowSize =       pXo->get(3).asInt();
        this->stereo_parameters.disp12MaxDiff =       pXo->get(4).asInt();
        this->stereo_parameters.preFilterCap =        pXo->get(5).asInt();
        this->stereo_parameters.uniquenessRatio =     pXo->get(6).asInt();
        this->stereo_parameters.speckleWindowSize =   pXo->get(7).asInt();
        this->stereo_parameters.speckleRange =        pXo->get(8).asInt();

        this->stereo_parameters.sigmaColorBLF =       pXo->get(9).asDouble();
        this->stereo_parameters.sigmaSpaceBLF =       pXo->get(10).asDouble();

        this->stereo_parameters.wls_lambda =          pXo->get(11).asDouble();
        this->stereo_parameters.wls_sigma =           pXo->get(12).asDouble();

        this->stereo_parameters.stereo_matching =     static_cast<SM_MATCHING_ALG>(pXo->get(13).asInt());
        this->stereo_parameters.BLFfiltering =        static_cast<SM_BLF_FILTER>(pXo->get(14).asInt());
        this->stereo_parameters.WLSfiltering =        static_cast<SM_WLS_FILTER>(pXo->get(15).asInt());

    }
    else
        this->initializeStereoParams();

    return true;
}


bool DispModule::loadIntrinsics(yarp::os::ResourceFinder &rf, Mat &KL, Mat &KR, Mat &DistL,
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



bool DispModule::updateExtrinsics(Mat& Rot, Mat& Tr, yarp::sig::Vector& eyes,
        const string& groupname)
{
    ofstream out;
    out.open(camCalibFile.c_str());
    if (out.is_open())
    {
        out << endl;
        out << "["+groupname+"]" << endl;
        out << "eyes (" << eyes.toString() << ")" << endl;
        out << "HN ("
            << Rot.at<double>(0,0) << " "
            << Rot.at<double>(0,1) << " "
            << Rot.at<double>(0,2) << " "
            << Tr.at<double>(0,0) << " "
            << Rot.at<double>(1,0) << " "
            << Rot.at<double>(1,1) << " "
            << Rot.at<double>(1,2) << " "
            << Tr.at<double>(1,0) << " "
            << Rot.at<double>(2,0) << " "
            << Rot.at<double>(2,1) << " "
            << Rot.at<double>(2,2) << " "
            << Tr.at<double>(2,0) << " "
            << 0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0                << ")"
            << endl;

        out.close();
        return true;
    }
    else
        return false;
}


bool DispModule::updateConfigurationFile(Mat& Rot, Mat& Tr, yarp::sig::Vector& eyes,
        const string& groupname)
{
    ofstream out;
    out.open(camCalibFile.c_str());
    if (out.is_open())
    {
        out << endl;
        out << "["+groupname+"]" << endl;
        out << "eyes (" << eyes.toString() << ")" << endl;
        out << "HN ("
            << Rot.at<double>(0,0) << " "
            << Rot.at<double>(0,1) << " "
            << Rot.at<double>(0,2) << " "
            << Tr.at<double>(0,0) << " "
            << Rot.at<double>(1,0) << " "
            << Rot.at<double>(1,1) << " "
            << Rot.at<double>(1,2) << " "
            << Tr.at<double>(1,0) << " "
            << Rot.at<double>(2,0) << " "
            << Rot.at<double>(2,1) << " "
            << Rot.at<double>(2,2) << " "
            << Tr.at<double>(2,0) << " "
            << 0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0                << ")"
            << endl;

        // TODO: change the following code and insert the stereo 
        // parameters in the corresponding struct as defined in StereoMatcher.h

        out << "params ("
            << this->stereo_parameters.minDisparity << " "
            << this->useBestDisp << " "
            << this->stereo_parameters.numberOfDisparities << " "
            << this->stereo_parameters.SADWindowSize << " "
            << this->stereo_parameters.disp12MaxDiff << " "
            << this->stereo_parameters.preFilterCap << " "
            << this->stereo_parameters.uniquenessRatio << " "
            << this->stereo_parameters.speckleWindowSize << " "
            << this->stereo_parameters.speckleRange << " "
            << this->stereo_parameters.sigmaColorBLF << " "
            << this->stereo_parameters.sigmaSpaceBLF << " "
            << this->stereo_parameters.wls_lambda << " "
            << this->stereo_parameters.wls_sigma << " "
            << this->stereo_parameters.stereo_matching << " "
            << this->stereo_parameters.BLFfiltering << " "
            << this->stereo_parameters.WLSfiltering << ")"
            << endl;

        out.close();
        return true;
    }
    else
        return false;
}


void DispModule::setDispParameters(bool _useBestDisp, int _uniquenessRatio,
        int _speckleWindowSize,int _speckleRange,
        int _numberOfDisparities, int _SADWindowSize,
        int _minDisparity, int _preFilterCap, int _disp12MaxDiff)
{
    this->mutexDisp.lock();

    this->useBestDisp=_useBestDisp;
    this->stereo_parameters.uniquenessRatio=_uniquenessRatio;
    this->stereo_parameters.speckleWindowSize=_speckleWindowSize;
    this->stereo_parameters.speckleRange=_speckleRange;
    this->stereo_parameters.numberOfDisparities=_numberOfDisparities;
    this->stereo_parameters.SADWindowSize=_SADWindowSize;
    this->stereo_parameters.minDisparity=_minDisparity;
    this->stereo_parameters.preFilterCap=_preFilterCap;
    this->stereo_parameters.disp12MaxDiff=_disp12MaxDiff;

    this->mutexDisp.unlock();

}


Point3f DispModule::get3DPoints(int u, int v, const string &drive)
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

    const Mat& disp16m=this->matcher->getDisparity16("wls");
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

    // TODO: "discards" means it returns the point in the same way?

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


Mat DispModule::buildRotTras(const Mat& R, const Mat& T)
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



Matrix DispModule::getCameraHGazeCtrl(int camera)
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



void DispModule::convert(Matrix& matrix, Mat& mat)
{
    mat=cv::Mat(matrix.rows(),matrix.cols(),CV_64FC1);
    for(int i=0; i<matrix.rows(); i++)
        for(int j=0; j<matrix.cols(); j++)
            mat.at<double>(i,j)=matrix(i,j);
}



void DispModule::convert(Mat& mat, Matrix& matrix)
{
    matrix.resize(mat.rows,mat.cols);
    for(int i=0; i<mat.rows; i++)
        for(int j=0; j<mat.cols; j++)
            matrix(i,j)=mat.at<double>(i,j);
}



bool DispModule::respond(const Bottle& command, Bottle& reply)
{
    if(command.size()==0)
        return false;

    if (command.get(0).asString()=="quit") {
        std::cout << "[DisparityModule] Closing..." << std::endl;
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
        reply.addString("- [bilatfilt sigmaColor sigmaSpace]: Set the parameters for the bilateral filer (default sigmaColor = 10.0, sigmaSpace = 10.0).");
        reply.addString("- [algorithm name]: Set the stereo matching algorithm used (default = SGMB(cpu)).");
        reply.addString("For more details on the commands, check the module's documentation");
        return true;
    }

    if (command.get(0).asString()=="calibrate")
    {
        mutexRecalibration.lock();
        numberOfTrials=0;
        this->runRecalibration=true;
        mutexRecalibration.unlock();

        calibEndEvent.reset();
        calibEndEvent.wait();

        if (this->calibUpdated)
        {
            R0=this->stereo->getRotation();
            T0=this->stereo->getTranslation();
            eyes0=eyes;

            reply.addString("ACK");
        }
        else
            reply.addString("[DisparityModule] Calibration failed after 5 trials.. Please show a non planar scene.");

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
            this->stereo_parameters.numberOfDisparities=dispNum;
            this->setDispParameters(useBestDisp,
                                    this->stereo_parameters.uniquenessRatio,
                                    this->stereo_parameters.speckleWindowSize,
                                    this->stereo_parameters.speckleRange,
                                    this->stereo_parameters.numberOfDisparities,
                                    this->stereo_parameters.SADWindowSize,
                                    this->stereo_parameters.minDisparity,
                                    this->stereo_parameters.preFilterCap,
                                    this->stereo_parameters.disp12MaxDiff);
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
        this->stereo_parameters.minDisparity=dispNum;
        this->setDispParameters(useBestDisp,
                                this->stereo_parameters.uniquenessRatio,
                                this->stereo_parameters.speckleWindowSize,
                                this->stereo_parameters.speckleRange,
                                this->stereo_parameters.numberOfDisparities,
                                this->stereo_parameters.SADWindowSize,
                                this->stereo_parameters.minDisparity,
                                this->stereo_parameters.preFilterCap,
                                this->stereo_parameters.disp12MaxDiff);
        reply.addString("ACK");
        return true;
    }

    if (command.get(0).asString()=="set" && command.size()==10)
    {
        bool useBestDisp=command.get(1).asInt() ? true : false;
        int uniquenessRatio=command.get(2).asInt();
        int speckleWindowSize=command.get(3).asInt();
        int speckleRange=command.get(4).asInt();
        int numberOfDisparities=command.get(5).asInt();
        int SADWindowSize=command.get(6).asInt();
        int minDisparity=command.get(7).asInt();
        int preFilterCap=command.get(8).asInt();
        int disp12MaxDiff=command.get(9).asInt();

        this->setDispParameters(useBestDisp,
                                uniquenessRatio,
                                speckleWindowSize,
                                speckleRange,
                                numberOfDisparities,
                                SADWindowSize,
                                minDisparity,
                                preFilterCap,
                                disp12MaxDiff);

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
        if (!this->doBLF){
            this->doBLF = true;
            reply.addString("Bilateral filter activated.");
        }
        this->stereo_parameters.sigmaColorBLF = command.get(1).asDouble();
        this->stereo_parameters.sigmaSpaceBLF = command.get(2).asDouble();
        reply.addString("BLF sigmaColor ");
        reply.addDouble(this->stereo_parameters.sigmaColorBLF);
        reply.addString("BLF sigmaSpace ");
        reply.addDouble(this->stereo_parameters.sigmaSpaceBLF);
    }
    else if (command.get(0).asString()=="doBLF")
    {
        bool onoffBLF = command.get(1).asBool();
        if (onoffBLF == false ){     // turn OFF Bilateral Filtering
            if (this->doBLF == true){
                this->doBLF = false;
                reply.addString("Bilateral Filter OFF");
            } else {
                reply.addString("Bilateral Filter already OFF");
            }

        } else {                    // turn ON Bilateral Filtering
            if (this->doBLF == true){
                reply.addString("Bilateral Filter Already Running");
            } else {                                     // Set any different from 0 to activate bilateral filter.
                this->doBLF = true;
                reply.addString("Bilateral Filter ON");
            }
        }
        reply.addDouble(this->stereo_parameters.sigmaColorBLF);
        reply.addDouble(this->stereo_parameters.sigmaSpaceBLF);
    }
    else
        reply.addString("NACK");

    return true;
}


Point2f DispModule::projectPoint(const string &camera, double x, double y, double z)
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
        response=this->stereo->projectPoints3D("right",points3D,HL_root); // this should be HR_root

    mutexDisp.unlock();

    return response[0];
}


cv::Mat DispModule::refineDisparity(cv::Mat old_disp, cv::Mat new_disp, int th)
{
    cv::Mat mask, result;

    cv::absdiff(old_disp, new_disp, mask);
    cv::threshold(mask, mask, th, 1, cv::THRESH_BINARY_INV);

    result = new_disp.mul(mask);

    return result;
}


DispModule::~DispModule()
{

#ifdef USE_GUI
    if(this->debugWindow)
        this->gui.killGUI();
#endif

}


DispModule::DispModule()
{
    this->debugWindow = false;
}


int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return 1;

    // rough quick check of the command line parameters, 
    // has to be definitely made in a smart way

    bool use640 = false;

    for(int i = 0; i < argc; i++)
        if(strcmp(argv[i], "--use640") == 0)
        {
            use640 = true;
            break;
        }

    ResourceFinder rf;
    rf.setVerbose(true);

    if(use640)
        rf.setDefaultConfigFile("icubEyes_640x480.ini");
    else
        rf.setDefaultConfigFile("icubEyes.ini");

    rf.setDefaultContext("cameraCalibration");
    rf.configure(argc,argv);

    DispModule mod;
    
    return mod.runModule(rf);
}
