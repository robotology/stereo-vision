/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Giulia Pasquale
 * email:   giulia.pasquale@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txtd
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <iCub/stereoVision/elasWrapper.h>

int64 elasWrapper::workBegin()
{
	return getTickCount();
}

double elasWrapper::workEnd(int64 work_begin)
{
    int64 d = getTickCount() - work_begin;
    double f = getTickFrequency();
    double work_time = d / f;
    return work_time;
}

elasWrapper::elasWrapper(double _io_scaling_factor, string elas_setting) : Elas(parameters(( elas_setting == "MIDDLEBURY") ? MIDDLEBURY : ROBOTICS))
{
	param.postprocess_only_left = true;

	io_scaling_factor = _io_scaling_factor;
}

elasWrapper::elasWrapper() : Elas(parameters(ROBOTICS))
{
	param.postprocess_only_left = true;

	io_scaling_factor = 1.0;
}

double elasWrapper::compute_disparity(cv::Mat &imL, cv::Mat &imR, cv::Mat &dispL, int num_disparities)
{

	int64 start = workBegin();

    // check for correct size
	if (imL.cols <= 0 || imL.rows <= 0 || imR.cols <= 0 || imR.rows <= 0
			|| imL.cols != imR.cols || imL.rows != imR.rows)
	{
		cout << "ERROR: Images must be of same size, but" << endl;
		cout << "       I1: " << imL.cols << " x " << imL.rows << ", I2: "
				<< imR.cols << " x " << imR.rows << endl;
		return -1;
	}

    Size im_size = imL.size();
    
    param.disp_max = num_disparities - 1;
    
    cout << "disp_max: " << param.disp_max  << endl;

    Mat imR_scaled, imL_scaled;
    if (io_scaling_factor!=1.0)
    {
    	resize(imR, imR_scaled, Size(), io_scaling_factor, io_scaling_factor);
    	resize(imL, imL_scaled, Size(), io_scaling_factor, io_scaling_factor);
    } else
    {
    	imR_scaled = imR.clone();
    	imL_scaled = imL.clone();
    }
    int width = imL_scaled.cols;
    int height = imL_scaled.rows;

	int width_disp_data = param.subsampling ? width>>1 : width;
	int height_disp_data = param.subsampling ? height>>1 : height;

	float *dispL_data = (float*) malloc(width_disp_data * height_disp_data * sizeof(float));
	float *dispR_data = (float*) malloc(width_disp_data * height_disp_data * sizeof(float));

	// prepare input images
	if (imL_scaled.channels() == 3)
	{
		cv::cvtColor(imL_scaled, imL_scaled, CV_BGR2GRAY);
		cv::cvtColor(imR_scaled, imR_scaled, CV_BGR2GRAY);
	}
	if (!imL_scaled.isContinuous())
		imL_scaled = imL_scaled.clone();
	if (!imR_scaled.isContinuous())
		imR_scaled = imR_scaled.clone();

    // compute disparity
	const int32_t dims[3] = {width,height,width}; // bytes per line = width

	process((unsigned char*)imL_scaled.data,(unsigned char*)imR_scaled.data, dispL_data, dispR_data, dims);

	Mat dispL_scaled = Mat(height_disp_data, width_disp_data, CV_32FC1, dispL_data);

    if (io_scaling_factor!=1.0 || param.subsampling==true)
    	resize(dispL_scaled, dispL, im_size);
    else
    	dispL = dispL_scaled.clone();

	free(dispL_data);
	free(dispR_data);

	return workEnd(start);

}

int elasWrapper::get_disp_min()
{
	return param.disp_min;
}
int elasWrapper::get_disp_max()
{
	return param.disp_max;
}
float elasWrapper::get_support_threshold()
{
	return param.support_threshold;
}
int elasWrapper::get_support_texture()
{
	return param.support_texture;
}
int elasWrapper::get_candidate_stepsize()
{
	return param.candidate_stepsize;
}
int elasWrapper::get_incon_window_size()
{
	return param.incon_window_size;
}
int elasWrapper::get_incon_threshold()
{
	return param.incon_threshold;
}
int elasWrapper::get_incon_min_support()
{
	return param.incon_min_support;
}
bool elasWrapper::get_add_corners()
{
	return param.add_corners;
}
int elasWrapper::get_grid_size()
{
	return param.grid_size;
}
float elasWrapper::get_beta()
{
	return param.beta;
}
float elasWrapper::get_gamma()
{
	return param.gamma;
}
float elasWrapper::get_sigma()
{
	return param.sigma;
}
float elasWrapper::get_sradius()
{
	return param.sradius;
}
int elasWrapper::get_match_texture()
{
	return param.match_texture;
}
int elasWrapper::get_lr_threshold()
{
	return param.lr_threshold;
}
float elasWrapper::get_speckle_sim_threshold()
{
	return param.speckle_sim_threshold;
}
int elasWrapper::get_speckle_size()
{
	return param.speckle_size;
}
int elasWrapper::get_ipol_gap_width()
{
	return param.ipol_gap_width;
}
bool elasWrapper::get_filter_median()
{
	return param.filter_median;
}
bool elasWrapper::get_filter_adaptive_mean()
{
	return param.filter_adaptive_mean;
}
bool elasWrapper::get_postprocess_only_left()
{
	return param.postprocess_only_left;
}
bool elasWrapper::get_subsampling()
{
	return param.subsampling;
}


void elasWrapper::set_disp_min(int param_value)
{
	param.disp_min = param_value;
}
void elasWrapper::set_disp_max(int param_value)
{
	param.disp_max = param_value;
}
void elasWrapper::set_support_threshold(float param_value)
{
	param.support_threshold = param_value;
}
void elasWrapper::set_support_texture(int param_value)
{
	param.support_texture = param_value;
}
void elasWrapper::set_candidate_stepsize(int param_value)
{
	param.candidate_stepsize = param_value;
}
void elasWrapper::set_incon_window_size(int param_value)
{
	param.incon_window_size = param_value;
}
void elasWrapper::set_incon_threshold(int param_value)
{
	param.incon_threshold = param_value;
}
void elasWrapper::set_incon_min_support(int param_value)
{
	param.incon_min_support = param_value;
}
void elasWrapper::set_add_corners(bool param_value)
{
	param.add_corners = param_value;
}
void elasWrapper::set_grid_size(int param_value)
{
	param.grid_size = param_value;
}
void elasWrapper::set_beta(float param_value)
{
	param.beta = param_value;
}
void elasWrapper::set_gamma(float param_value)
{
	param.gamma = param_value;
}
void elasWrapper::set_sigma(float param_value)
{
	param.sigma = param_value;
}
void elasWrapper::set_sradius(float param_value)
{
	param.sradius = param_value;
}
void elasWrapper::set_match_texture(int param_value)
{
	param.match_texture = param_value;
}
void elasWrapper::set_lr_threshold(int param_value)
{
	param.lr_threshold = param_value;
}
void elasWrapper::set_speckle_sim_threshold(float param_value)
{
	param.speckle_sim_threshold = param_value;
}
void elasWrapper::set_speckle_size(int param_value)
{
	param.speckle_size = param_value;
}
void elasWrapper::set_ipol_gap_width(int param_value)
{
	param.ipol_gap_width = param_value;
}
void elasWrapper::set_filter_median(bool param_value)
{
	param.filter_median = param_value;
}
void elasWrapper::set_filter_adaptive_mean(bool param_value)
{
	param.filter_adaptive_mean = param_value;
}
void elasWrapper::set_postprocess_only_left(bool param_value)
{
	param.postprocess_only_left = param_value;
}
void elasWrapper::set_subsampling(bool param_value)
{
	param.subsampling = param_value;
}
