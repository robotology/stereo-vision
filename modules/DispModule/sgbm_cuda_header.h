void free_gpu_mem();
cv::Mat compute_disparity(cv::Mat *left_img, cv::Mat *right_img, float *cost_time);
void cuda_init(SGM_PARAMS *params);
cv::Mat zy_remap(cv::Mat &img1, cv::Mat &img2);




