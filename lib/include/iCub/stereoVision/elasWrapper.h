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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "elas.h"

using namespace cv;
using namespace std;

class elasWrapper {

	double io_scaling_factor;

	Elas::parameters *param;
	Elas *elas;

public:

	int64 workBegin();
	double workEnd(int64 work_begin);
    
	void init_elas(string _s, double disp_scaling_factor, bool elas_subsampling, bool add_corners, int ipol_gap_width);
    void release_elas();

	double compute_disparity(cv::Mat &imL, cv::Mat &imR, cv::Mat &dispL, int num_disparities);

};

#endif /* ELASWRAPPER_H_ */
