/*
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Sean Ryan Fanello, Giulia Pasquale
 * email:  sean.fanello@iit.it giulia.pasquale@iit.it
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

/**
* \ingroup StereoVisionLib
*
* The class defining a basic interface to LIBELAS.
* It provides read/write access to Elas parameters and an interface with OpenCV's cv::Mat images.
*/
#ifndef ELASWRAPPER_H_
#define ELASWRAPPER_H_

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdexcept>

// OpenCV
#include <opencv/highgui.h>
#include <opencv/cv.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "elas.h"

using namespace cv;
using namespace std;

class elasWrapper : public Elas {

	double io_scaling_factor;

public:

	int64 workBegin();
	double workEnd(int64 work_begin);
    
	elasWrapper();
	elasWrapper(double scaling_factor, string elas_setting);

	double compute_disparity(cv::Mat &imL, cv::Mat &imR, cv::Mat &dispL, int num_disparities);

	int get_disp_min();
	int get_disp_max();
	float get_support_threshold();
	int get_support_texture();
	int get_candidate_stepsize();
	int get_incon_window_size();
	int get_incon_threshold();
	int get_incon_min_support();
	bool get_add_corners();
	int get_grid_size();
	float get_beta();
	float get_gamma();
	float get_sigma();
	float get_sradius();
	int get_match_texture();
	int get_lr_threshold();
	float get_speckle_sim_threshold();
	int get_speckle_size();
	int get_ipol_gap_width();
	bool get_filter_median();
	bool get_filter_adaptive_mean();
	bool get_postprocess_only_left();
	bool get_subsampling();

	void set_disp_min(int param_value);
	void set_disp_max(int param_value);
	void set_support_threshold(float param_value);
	void set_support_texture(int param_value);
	void set_candidate_stepsize(int param_value);
	void set_incon_window_size(int param_value);
	void set_incon_threshold(int param_value);
	void set_incon_min_support(int param_value);
	void set_add_corners(bool param_value);
	void set_grid_size(int param_value);
	void set_beta(float param_value);
	void set_gamma(float param_value);
	void set_sigma(float param_value);
	void set_sradius(float param_value);
	void set_match_texture(int param_value);
	void set_lr_threshold(int param_value);
	void set_speckle_sim_threshold(float param_value);
	void set_speckle_size(int param_value);
	void set_ipol_gap_width(int param_value);
	void set_filter_median(bool param_value);
	void set_filter_adaptive_mean(bool param_value);
	void set_postprocess_only_left(bool param_value);
	void set_subsampling(bool param_value);
};

#endif /* ELASWRAPPER_H_ */
