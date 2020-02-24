
#include "cvgui.h"

#define CVUI_IMPLEMENTATION
#include "cvui.h"

const char * WINDOW_NAME = "DisparityModule Parameters";


GUI::GUI()
{

}


bool GUI::isUpdated()
{
    return this->updated;
}


void GUI::initializeGUI(int &minDisparity, int &numberOfDisparities, int &SADWindowSize,
                       int &disp12MaxDiff, int &preFilterCap, int &uniquenessRatio,
                       int &speckleWindowSize, int &speckleRange, double &sigmaColorBLF,
                       double &sigmaSpaceBLF, double &wls_lambda, double &wls_sigma,
                       SM_BLF_FILTER &BLFfiltering, SM_WLS_FILTER &WLSfiltering,
                       SM_MATCHING_ALG &stereo_matching)
{
    this->params.minDisparity = minDisparity;
    this->params.numberOfDisparities = numberOfDisparities;
    this->params.SADWindowSize = SADWindowSize;
    this->params.disp12MaxDiff = disp12MaxDiff;
    this->params.preFilterCap = preFilterCap;
    this->params.uniquenessRatio = uniquenessRatio;
    this->params.speckleWindowSize = speckleWindowSize;
    this->params.speckleRange = speckleRange;
    this->params.sigmaColorBLF = sigmaColorBLF;
    this->params.sigmaSpaceBLF = sigmaSpaceBLF;
    this->params.wls_lambda = wls_lambda;
    this->params.wls_sigma = wls_sigma;
    this->params.BLFfiltering = BLFfiltering;
    this->params.WLSfiltering = WLSfiltering;
    this->params.stereo_matching = stereo_matching;

    this->refine_th = 0;

    this->convertEnumToID();
    this->initializeGUI();
}


void GUI::initializeGUI()
{
    // initializes the GUI internal states

    this->resetState();

    // creates the window on which to plot the GUI

    int gui_width = 450; // separate only for the sake of clarity in the initialization

    frame = cv::Mat(cv::Size(gui_width, cvuiw::estimateHeight(12,4,2)), CV_8UC3);
    cvui::init(WINDOW_NAME, 3);
}


void GUI::killGUI()
{

}


void GUI::updateGUI()
{
    // if the window has been closed by the user, 
    // terminate the execution of the GUI

    if(cv::getWindowProperty(WINDOW_NAME, 0) < 0)
    {
        this->done = false;
        return;
    }

    // set the background color for the GUI frame

    frame = cv::Scalar(20, 22, 23);

    // reset the local state of the GUI

    cvuiw::reset();

    this->updated = false;

    // GUI definition

    // 1. Stereo Matching and filtering parameters

    this->updated |= cvuiw::trackbar<int>("refineTh", frame, &(this->refine_th), 0, 50, 1, "%1.Lf", cvui::TRACKBAR_HIDE_LABELS);
    this->updated |= cvuiw::trackbar<int>("minDisparity", frame, &(this->params.minDisparity), 0, 20, 1, "%1.Lf", cvui::TRACKBAR_HIDE_LABELS);
    this->updated |= cvuiw::trackbar<int>("SADWindowSize", frame, &(this->params.SADWindowSize), 3, 31, 1, "%1.Lf", cvui::TRACKBAR_HIDE_LABELS);
    this->updated |= cvuiw::trackbar<int>("disp12MaxDiff", frame, &(this->params.disp12MaxDiff), 0, 30, 1, "%1.Lf", cvui::TRACKBAR_HIDE_LABELS);
    this->updated |= cvuiw::trackbar<int>("preFilterCap", frame, &(this->params.preFilterCap), 0, 100, 1, "%1.Lf", cvui::TRACKBAR_HIDE_LABELS);
    this->updated |= cvuiw::trackbar<int>("uniquenessRatio", frame, &(this->params.uniquenessRatio), 5, 20, 1, "%1.Lf", cvui::TRACKBAR_HIDE_LABELS);
    this->updated |= cvuiw::trackbar<int>("speckleWindowSize", frame, &(this->params.speckleWindowSize), 0, 200, 1, "%1.Lf", cvui::TRACKBAR_HIDE_LABELS);
    this->updated |= cvuiw::trackbar<int>("speckleRange", frame, &(this->params.speckleRange), 1, 16, 1, "%1.Lf", cvui::TRACKBAR_HIDE_LABELS);
    this->updated |= cvuiw::trackbar<double>("sigmaColorBLF", frame, &(this->params.sigmaColorBLF), 1.0, 30.0, 1, "%.1Lf", cvui::TRACKBAR_HIDE_LABELS);
    this->updated |= cvuiw::trackbar<double>("sigmaSpaceBLF", frame, &(this->params.sigmaSpaceBLF), 1.0, 30.0, 1, "%.1Lf", cvui::TRACKBAR_HIDE_LABELS);
    this->updated |= cvuiw::trackbar<double>("WLS lambda", frame, &(this->params.wls_lambda), 500.0, 30000.0, 1, "%1.Lf", cvui::TRACKBAR_HIDE_LABELS | cvui::TRACKBAR_DISCRETE, 500.0);
    this->updated |= cvuiw::trackbar<double>("WLS sigma", frame, &(this->params.wls_sigma), 0.1, 10.0, 1, "%.1Lf", cvui::TRACKBAR_HIDE_LABELS);

    // 2. Multiple choice controls

    this->updated |= cvuiw::radioButtons(frame, "Num. of Disparities:", {"32", "64", "96", "128"}, {20, 65, 110, 155}, this->num_disparities_id);
    this->updated |= cvuiw::radioButtons(frame, "Stereo Matching Alg.:", {"SGBM", "SGBM_CUDA", "LibElas"}, {20, 90, 190}, this->stereo_matching_id);
    this->updated |= cvuiw::radioButtons(frame, "Bilateral Filtering:", {"No BLF", "Original BLF", "CUDA BLF"}, {20, 90, 190}, this->BLFfiltering_id);
    this->updated |= cvuiw::radioButtons(frame, "Weigthed LS Filtering:", {"No WLS", "WLS", "WLS w/ lr cons."}, {20, 100, 155}, this->WLSfiltering_id);

    // 3. Button to trigger the recalibration of the system

    this->recalibrate = cvuiw::button(frame, "&Recalibrate", 10, 0);
    this->updated |= this->recalibrate;

    // 4. Button to save the calibration parameters to the local file

    this->save_calibration = cvuiw::button(frame, "&Save Calibration", 130, 0);
    this->updated |= this->save_calibration;

    // 5. Button to close the GUI

    if(cvuiw::button(frame, "&Quit", 282, 0))
    {
        this->done = true;
        cv::destroyWindow(WINDOW_NAME);
        return;
    }

    // 6. Button to set the default parameters

    this->load_parameters = cvuiw::button(frame, "&Default Param.", 10, 1);
    this->updated |= this->load_parameters;

    // 7. Save the matching/filtering parameters to a local configuration file

    this->save_parameters = cvuiw::button(frame, "Save &Parameters", 155, 1);
    this->updated |= this->save_parameters;

    // 8. Acquire the selected radiobuttons indexes..

    this->num_disparities_id = cvuiw::getRadioIndex(0);
    this->stereo_matching_id = cvuiw::getRadioIndex(1);
    this->BLFfiltering_id = cvuiw::getRadioIndex(2);
    this->WLSfiltering_id = cvuiw::getRadioIndex(3);

    // 9. ..then convert it to the corresponding ENUM elements

    this->convertIDToEnum();

    // Since cvui::init() received a param regarding waitKey,
    // there is no need to call cv::waitKey() anymore. cvui::update()
    // will do it automatically.

    cvui::update();

    cv::imshow(WINDOW_NAME, frame);

    return;
}


void GUI::convertIDToEnum()
{
    switch(this->stereo_matching_id)
    {
        case 0:
            this->params.stereo_matching = SM_MATCHING_ALG::SGBM_OPENCV;
            break;
        case 1:
            this->params.stereo_matching = SM_MATCHING_ALG::SGBM_CUDA;
            break;
        case 2:
            this->params.stereo_matching = SM_MATCHING_ALG::LIBELAS;
            break;
    }

    switch(this->BLFfiltering_id)
    {
        case 0:
            this->params.BLFfiltering = SM_BLF_FILTER::BLF_DISABLED;
            break;
        case 1:
            this->params.BLFfiltering = SM_BLF_FILTER::BLF_ORIGINAL;
            break;
        case 2:
            this->params.BLFfiltering = SM_BLF_FILTER::BLF_CUDA;
            break;
    }

    switch(this->WLSfiltering_id)
    {
        case 0:
            this->params.WLSfiltering = SM_WLS_FILTER::WLS_DISABLED;
            break;
        case 1:
            this->params.WLSfiltering = SM_WLS_FILTER::WLS_ENABLED;
            break;
        case 2:
            this->params.WLSfiltering = SM_WLS_FILTER::WLS_LRCHECK;
            break;
    }

    this->params.numberOfDisparities = 32 * (this->num_disparities_id+1);
}


void GUI::convertEnumToID()
{
    switch(this->params.stereo_matching)
    {
        case SM_MATCHING_ALG::SGBM_OPENCV:
            this->stereo_matching_id = 0;
            break;
        case SM_MATCHING_ALG::SGBM_CUDA:
            this->stereo_matching_id = 1;
            break;
        case SM_MATCHING_ALG::LIBELAS:
            this->stereo_matching_id = 2;
            break;

    }

    switch(this->params.BLFfiltering)
    {
        case SM_BLF_FILTER::BLF_DISABLED:
            this->BLFfiltering_id = 0;
            break;
        case SM_BLF_FILTER::BLF_ORIGINAL:
            this->BLFfiltering_id = 1;
            break;
        case SM_BLF_FILTER::BLF_CUDA:
            this->BLFfiltering_id = 2;
            break;

    }

    switch(this->params.WLSfiltering)
    {
        case SM_WLS_FILTER::WLS_DISABLED:
            this->WLSfiltering_id = 0;
            break;
        case SM_WLS_FILTER::WLS_ENABLED:
            this->WLSfiltering_id = 1;
            break;
        case SM_WLS_FILTER::WLS_LRCHECK:
            this->WLSfiltering_id = 2;
            break;

    }

    this->num_disparities_id = (this->params.numberOfDisparities/32)-1;
}


void GUI::getParameters(int& minDisparity, int& numberOfDisparities, int& SADWindowSize,
                        int& disp12MaxDiff, int& preFilterCap, int& uniquenessRatio,
                        int& speckleWindowSize, int& speckleRange, double& sigmaColorBLF,
                        double& sigmaSpaceBLF, double& wls_lambda, double& wls_sigma,
                        SM_BLF_FILTER& BLFfiltering, SM_WLS_FILTER& WLSfiltering,
                        SM_MATCHING_ALG& stereo_matching)
{
    minDisparity = this->params.minDisparity;
    numberOfDisparities = this->params.numberOfDisparities;
    SADWindowSize = this->params.SADWindowSize;
    disp12MaxDiff = this->params.disp12MaxDiff;
    preFilterCap = this->params.preFilterCap;
    uniquenessRatio = this->params.uniquenessRatio;
    speckleWindowSize = this->params.speckleWindowSize;
    speckleRange = this->params.speckleRange;
    sigmaColorBLF = this->params.sigmaColorBLF;
    sigmaSpaceBLF = this->params.sigmaSpaceBLF;
    wls_lambda = this->params.wls_lambda;
    wls_sigma = this->params.wls_sigma;
    BLFfiltering = this->params.BLFfiltering;
    WLSfiltering = this->params.WLSfiltering;
    stereo_matching = this->params.stereo_matching;
}


void GUI::setParameters(int& minDisparity, int& numberOfDisparities, int& SADWindowSize,
               int& disp12MaxDiff, int& preFilterCap, int& uniquenessRatio,
               int& speckleWindowSize, int& speckleRange, double& sigmaColorBLF,
               double& sigmaSpaceBLF, double& wls_lambda, double& wls_sigma,
               SM_BLF_FILTER& BLFfiltering, SM_WLS_FILTER& WLSfiltering,
               SM_MATCHING_ALG& stereo_matching)
{
    this->params.minDisparity = minDisparity;
    this->params.numberOfDisparities = numberOfDisparities;
    this->params.SADWindowSize = SADWindowSize;
    this->params.disp12MaxDiff = disp12MaxDiff;
    this->params.preFilterCap = preFilterCap;
    this->params.uniquenessRatio = uniquenessRatio;
    this->params.speckleWindowSize = speckleWindowSize;
    this->params.speckleRange = speckleRange;
    this->params.sigmaColorBLF = sigmaColorBLF;
    this->params.sigmaSpaceBLF = sigmaSpaceBLF;
    this->params.wls_lambda = wls_lambda;
    this->params.wls_sigma = wls_sigma;
    this->params.BLFfiltering = BLFfiltering;
    this->params.WLSfiltering = WLSfiltering;
    this->params.stereo_matching = stereo_matching;
}


bool GUI::isDone()
{
    return this->done;
}


GUI::~GUI()
{
    this->killGUI();
}


bool GUI::toRecalibrate()
{
    return this->recalibrate;
}


bool GUI::toSaveCalibration()
{
    return this->save_calibration;
}


bool GUI::toLoadParameters()
{
    return this->load_parameters;
}


bool GUI::toSaveParameters()
{
    return this->save_parameters;
}


void GUI::resetState()
{
    this->done = false;
    this->updated = false;
    this->recalibrate = false;
    this->save_calibration = false;
    this->load_parameters = false;
    this->save_parameters = false;
}


int GUI::getRefineTh()
{
    return refine_th;
}


bool GUI::toRefine()
{
    return refine_th > 0;
}
