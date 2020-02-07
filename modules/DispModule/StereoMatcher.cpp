

#include "fastBilateral.hpp"
#include "StereoMatcher.h"
#include <opencv2/opencv.hpp>

using cv::Ptr;
using cv::StereoSGBM;
using namespace cv::ximgproc;


Rect computeROI2(Size2i src_sz, Ptr<StereoMatcher> matcher_instance)
{
    int min_disparity = matcher_instance->getMinDisparity();
    int num_disparities = matcher_instance->getNumDisparities();
    int block_size = matcher_instance->getBlockSize();

    int bs2 = block_size/2;
    int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

    int xmin = maxD + bs2;
    int xmax = src_sz.width + minD - bs2;
    int ymin = bs2;
    int ymax = src_sz.height - bs2;

    Rect r(xmin, ymin, xmax - xmin, ymax - ymin);

    return r;
}


void StereoMatcherNew::setParameters(int minDisparity, int numberOfDisparities, int SADWindowSize,
                                  int disp12MaxDiff, int preFilterCap, int uniquenessRatio,
                                  int speckleWindowSize, int speckleRange, double sigmaColorBLF,
                                  double sigmaSpaceBLF, double wls_lambda, double wls_sigma,
                                  SM_BLF_FILTER BLFfiltering, SM_WLS_FILTER WLSfiltering,
                                  SM_MATCHING_ALG stereo_matching)
{
    this->stereo_parameters.minDisparity = minDisparity;
    this->stereo_parameters.numberOfDisparities = numberOfDisparities;
    this->stereo_parameters.SADWindowSize = SADWindowSize;
    this->stereo_parameters.disp12MaxDiff = disp12MaxDiff;
    this->stereo_parameters.preFilterCap = preFilterCap;
    this->stereo_parameters.uniquenessRatio = uniquenessRatio;
    this->stereo_parameters.speckleWindowSize = speckleWindowSize;
    this->stereo_parameters.speckleRange = speckleRange;
    this->stereo_parameters.sigmaColorBLF = sigmaColorBLF;
    this->stereo_parameters.sigmaSpaceBLF = sigmaSpaceBLF;
    this->stereo_parameters.wls_lambda = wls_lambda;
    this->stereo_parameters.wls_sigma = wls_sigma;
    this->stereo_parameters.BLFfiltering = BLFfiltering;
    this->stereo_parameters.WLSfiltering = WLSfiltering;
    this->stereo_parameters.stereo_matching = stereo_matching;
}


void StereoMatcherNew::compute()
{
    // prepares the StereoCamera and  
    // rectifies the input images

    this->stereo->updateMappings();
    this->stereo->rectifyImages();

    // runs the selected stereo matching algorithm

    switch(this->stereo_parameters.stereo_matching)
    {
        case SM_MATCHING_ALG::SGBM_OPENCV:
            this->matchSGBM();
            break;
        case SM_MATCHING_ALG::SGBM_CUDA:
            this->matchSGBMCUDA();
            break;
        case SM_MATCHING_ALG::LIBELAS:
            this->matchLIBELAS();
            break;
    }
}


void StereoMatcherNew::filterBLF(string kind = "base")
{

    cv::Mat input = this->getDisparity(kind);
    cv::Mat input16 = this->getDisparity16(kind);

    switch(this->stereo_parameters.BLFfiltering)
    {
        case SM_BLF_FILTER::BLF_ORIGINAL:

            cv_extend::bilateralFilter(input16,
                                       this->disparity16_blf,
                                       this->stereo_parameters.sigmaColorBLF,
                                       this->stereo_parameters.sigmaSpaceBLF);

            getDisparityVis(this->disparity16_blf, this->disparity_blf, 3);

            break;

        case SM_BLF_FILTER::BLF_CUDA:
        {
            cv::Mat grayL = this->stereo->getLRectified();
            cv::cvtColor(grayL, grayL, CV_BGR2GRAY);

            imageGpu.upload(grayL);
            gpuDisp.upload(input16);

            pCudaBilFilter->apply(gpuDisp, imageGpu, filtGpu);

            filtGpu.download(this->disparity16_blf);

            getDisparityVis(this->disparity16_blf, this->disparity_blf, 3);

            break;
        }
        case SM_BLF_FILTER::BLF_DISABLED:
        default:

            this->disparity_blf = input.clone();
            this->disparity16_blf = input16.clone();
            break;
    }
}


void StereoMatcherNew::filterWLS(string kind = "base")
{
    cv::Mat input = this->getDisparity(kind);
    cv::Mat input16 = this->getDisparity16(kind);
    cv::Rect ROI;

    if(this->stereo_parameters.WLSfiltering != SM_WLS_FILTER::WLS_DISABLED)
    {

        Ptr<StereoSGBM> sgbm =cv::StereoSGBM::create(this->stereo_parameters.minDisparity,
                                                     this->stereo_parameters.numberOfDisparities,
                                                     this->stereo_parameters.SADWindowSize,
                                                     8*3*this->stereo_parameters.SADWindowSize*this->stereo_parameters.SADWindowSize,
                                                     32*3*this->stereo_parameters.SADWindowSize*this->stereo_parameters.SADWindowSize,
                                                     this->stereo_parameters.disp12MaxDiff,
                                                     this->stereo_parameters.preFilterCap,
                                                     this->stereo_parameters.uniquenessRatio,
                                                     this->stereo_parameters.speckleWindowSize,
                                                     this->stereo_parameters.speckleRange,
                                                     this->useBestDisp?StereoSGBM::MODE_HH:StereoSGBM::MODE_SGBM);

        cv::Mat left_rect = this->stereo->getLRectified();

        switch(this->stereo_parameters.WLSfiltering)
        {
            case SM_WLS_FILTER::WLS_ENABLED:

                wls_filter->setLambda(this->stereo_parameters.wls_lambda);
                wls_filter->setSigmaColor(this->stereo_parameters.wls_sigma);
                wls_filter->setDepthDiscontinuityRadius((int)ceil(0.5*this->stereo_parameters.SADWindowSize));
                ROI = computeROI2(left_rect.size(),sgbm);

                wls_filter->filter(input,left_rect,this->disparity_wls,Mat(),ROI);
                wls_filter->filter(input16,left_rect,this->disparity16_wls,Mat(),ROI);

                break;

            case SM_WLS_FILTER::WLS_LRCHECK:                    

                if(this->stereo_matching == SM_MATCHING_ALG::SGBM_OPENCV)
                {

                    wls_filter->setLambda(this->stereo_parameters.wls_lambda);
                    wls_filter->setSigmaColor(this->stereo_parameters.wls_sigma);

                    cv::Mat right_disp;

                    Ptr<StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(sgbm);

                    right_matcher->compute(this->stereo->getRRectified(),this->stereo->getLRectified(), right_disp);

                    wls_filter->filter(input,left_rect,this->disparity_wls,right_disp);
                    wls_filter->filter(input16,left_rect,this->disparity16_wls,right_disp);

                }
                else
                    this->disparity_wls = input.clone();
                    this->disparity16_wls = input16.clone();

                break;

            default:

                break;
        }
    }
    else
    {
        this->disparity_wls = input.clone();
        this->disparity16_wls = input16.clone();
    }
}


cv::Mat StereoMatcherNew::getDisparity(string kind="base")
{
    // return the disparity map specified 
    // via the "kind" parameter

    if(kind == "base")
        return this->disparity;
    else if(kind == "blf")
        return this->disparity_blf;
    else if(kind == "wls")
        return this->disparity_wls;
    else
    {
        std::cout << "[StereoMatcherNew] !! Disparity kind " << kind <<  " not found, returning BASE disparity." << std::endl;
        return this->disparity;
    }
}



cv::Mat StereoMatcherNew::getDisparity16(string kind="base")
{
    // return the disparity map specified 
    // via the "kind" parameter

    if(kind == "base")
        return this->disparity16;
    else if(kind == "blf")
        return this->disparity16_blf;
    else if(kind == "wls")
        return this->disparity16_wls;
    else
    {
        std::cout << "[StereoMatcherNew] !! Disparity16 kind " << kind <<  " not found, returning BASE disparity16." << std::endl;
        return this->disparity16;
    }
}


void StereoMatcherNew::setAlgorithm(string name)
{
    // set the stereo matching algorithm used, defaults 
    // to OpenCV SGBM in case an unknown name is given

    if(name == "sgbm")
        this->stereo_parameters.stereo_matching = SM_MATCHING_ALG::SGBM_OPENCV;
    else if(name == "sgbm_cuda")
        this->stereo_parameters.stereo_matching = SM_MATCHING_ALG::SGBM_CUDA;
    else if(name == "libelas")
        this->stereo_parameters.stereo_matching = SM_MATCHING_ALG::LIBELAS;
    else
    {
        std::cout << "[StereoMatcherNew] !! Stereo Matching algorithm " << name <<  " not found, defaulting to SGBM." << std::endl;
        this->stereo_parameters.stereo_matching = SM_MATCHING_ALG::SGBM_OPENCV;
    }
}


StereoMatcherNew::StereoMatcherNew(yarp::os::ResourceFinder &rf, StereoCamera * stereo)
{
    // initializes the StereoMatcherNew object 
    // and a few internal parameters and objects

    this->stereo = stereo;
    this->wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
    this->initELAS(rf);
    this->initCUDAbilateralFilter();
}


void StereoMatcherNew::updateCUDAParams()
{
    // updates the parameters for the CUDA matching algorithm
    // in the corresponding struct, together to those related 
    // to the CUDA bilateral filter

    this->cuda_params.preFilterCap = this->stereo_parameters.preFilterCap;
    this->cuda_params.BlockSize = this->stereo_parameters.SADWindowSize;
    this->cuda_params.P1 = 8 * this->cuda_params.BlockSize * this->cuda_params.BlockSize;
    this->cuda_params.P2 = 32 * this->cuda_params.BlockSize * this->cuda_params.BlockSize;
    this->cuda_params.uniquenessRatio = this->stereo_parameters.uniquenessRatio;
    this->cuda_params.disp12MaxDiff = this->stereo_parameters.disp12MaxDiff;
    this->cuda_params.numberOfDisparities = this->stereo_parameters.numberOfDisparities;

    cuda_init(&this->cuda_params);

    pCudaBilFilter->setSigmaRange(this->stereo_parameters.sigmaColorBLF);
    pCudaBilFilter->setNumDisparities(this->stereo_parameters.numberOfDisparities);
}


void StereoMatcherNew::initCUDAbilateralFilter()
{
    // initializes the values for the CUDA bilateral filtering

    int radius = 7;
    int iters = 2;

    this->pCudaBilFilter = cuda::createDisparityBilateralFilter(this->stereo_parameters.numberOfDisparities, radius, iters);
    this->pCudaBilFilter->setSigmaRange(this->stereo_parameters.sigmaColorBLF);
}


void StereoMatcherNew::initELAS(yarp::os::ResourceFinder &rf)
{
    // initializes the parameters for the stereo 
    // matching algorithm implemented in libElas

    string elas_string = rf.check("elas_setting",Value("ROBOTICS")).asString();

    double disp_scaling_factor = rf.check("disp_scaling_factor",Value(1.0)).asDouble();

    this->elaswrap = new elasWrapper(disp_scaling_factor, elas_string);

    if (rf.check("elas_subsampling"))
        elaswrap->set_subsampling(true);

    if (rf.check("elas_add_corners"))
        elaswrap->set_add_corners(true);

    elaswrap->set_ipol_gap_width(40);
    if (rf.check("elas_ipol_gap_width"))
        elaswrap->set_ipol_gap_width(rf.find("elas_ipol_gap_width").asInt());

    if (rf.check("elas_support_threshold"))
        elaswrap->set_support_threshold(rf.find("elas_support_threshold").asDouble());

    if(rf.check("elas_gamma"))
        elaswrap->set_gamma(rf.find("elas_gamma").asDouble());

    if (rf.check("elas_sradius"))
        elaswrap->set_sradius(rf.find("elas_sradius").asDouble());

    if (rf.check("elas_match_texture"))
        elaswrap->set_match_texture(rf.find("elas_match_texture").asInt());

    if (rf.check("elas_filter_median"))
        elaswrap->set_filter_median(rf.find("elas_filter_median").asBool());

    if (rf.check("elas_filter_adaptive_mean"))
        elaswrap->set_filter_adaptive_mean(rf.find("elas_filter_adaptive_mean").asBool());


    // the following code to print the libElas parameters has
    // been commented, still is left here in case it's needed
    // in the future 

    // cout << endl << "ELAS parameters:" << endl << endl;

    // cout << "disp_scaling_factor: " << disp_scaling_factor << endl;

    // cout << "setting: " << elas_string << endl;

    // cout << "postprocess_only_left: " << elaswrap->get_postprocess_only_left() << endl;
    // cout << "subsampling: " << elaswrap->get_subsampling() << endl;

    // cout << "add_corners: " << elaswrap->get_add_corners() << endl;

    // cout << "ipol_gap_width: " << elaswrap->get_ipol_gap_width() << endl;

    // cout << "support_threshold: " << elaswrap->get_support_threshold() << endl;
    // cout << "gamma: " << elaswrap->get_gamma() << endl;
    // cout << "sradius: " << elaswrap->get_sradius() << endl;

    // cout << "match_texture: " << elaswrap->get_match_texture() << endl;

    // cout << "filter_median: " << elaswrap->get_filter_median() << endl;
    // cout << "filter_adaptive_mean: " << elaswrap->get_filter_adaptive_mean() << endl;

    // cout << endl;
}



void StereoMatcherNew::matchLIBELAS()
{
    // runs the computation of the disparity map using the libElas wrapper

    cv::Mat map;

    bool success = elaswrap->compute_disparity(this->stereo->getLRectified(), this->stereo->getRRectified(), this->disparity, this->stereo_parameters.numberOfDisparities);

    if (success)
        map = this->disparity * (255.0 / this->stereo_parameters.numberOfDisparities);

    this->disparity.convertTo(this->disparity16, CV_16SC1, 16.0);

    getDisparityVis(this->disparity16, this->disparity, 3);
}


void StereoMatcherNew::matchSGBM()
{
    // computes the disparity map using the implementation 
    // of SGBM available in OpenCV

    cv::Mat map, disp;

    Ptr<StereoSGBM> sgbm =cv::StereoSGBM::create(this->stereo_parameters.minDisparity,
                                                 this->stereo_parameters.numberOfDisparities,
                                                 this->stereo_parameters.SADWindowSize,
                                                 8*3*this->stereo_parameters.SADWindowSize*this->stereo_parameters.SADWindowSize,
                                                 32*3*this->stereo_parameters.SADWindowSize*this->stereo_parameters.SADWindowSize,
                                                 this->stereo_parameters.disp12MaxDiff,
                                                 this->stereo_parameters.preFilterCap,
                                                 this->stereo_parameters.uniquenessRatio,
                                                 this->stereo_parameters.speckleWindowSize,
                                                 this->stereo_parameters.speckleRange,
                                                 this->useBestDisp?StereoSGBM::MODE_HH:StereoSGBM::MODE_SGBM);


    sgbm->compute(this->stereo->getLRectified(), this->stereo->getRRectified(), disp);

    this->disparity16 = disp;

    getDisparityVis(disp, this->disparity, 3);
}


void StereoMatcherNew::matchSGBMCUDA()
{
    // runs the SGBM stereo matching algorithm implemented in CUDA

    cv::Mat outputDm;

    cv::Mat grayL = this->stereo->getLRectified();
    cv::Mat grayR = this->stereo->getRRectified();

    cv::cvtColor(grayL, grayL, CV_BGR2GRAY);
    cv::cvtColor(grayR, grayR, CV_BGR2GRAY);

    outputDm = zy_remap(grayL, grayR);

    cv::Mat map;
    outputDm.convertTo(map,CV_32FC1,255/(this->stereo_parameters.numberOfDisparities*16.));

    getDisparityVis(outputDm, this->disparity, 3);

    this->disparity16 = outputDm;
}


StereoMatcherNew::~StereoMatcherNew()
{

}
