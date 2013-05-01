//
//  kuwaharaFilter.h
//  VC.proj
//
//  Created by  on 12/5/23.
//  Copyright (c) 2012年 __MyCompanyName__. All rights reserved.
//

#ifndef KUWAHARA_FILTER_H
#define KUWAHARA_FILTER_H

#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <ctime>
#include <sstream>
#include "Rosaniline.h"






using namespace cv;
using namespace std;

class kuwaharaFilter {
public:
    
    kuwaharaFilter();
    //    kuwaharaFilter();
    
    void image_filter(const string& fileName, int iterations);
    void video_filter(char *filePath, int start_frame, int frame_count);
    ~kuwaharaFilter();
    
    
    
private:
    
    static const double PI              = 3.14159265;
    static const double GAUSSIAN_SIGMA  = 2.0;
    static const double ECCEN_TUNING    = 1.0;
    static const double SIGMA_R         = 3.0;
    static const double SIGMA_S         = 3.0;
    static const double SHARPNESS_Q     = 8.0;
    static const int    SECTOR_N        = 8;

    
    
    void tensor_computation();
    void anisotropic_kuwahara();
    void div_circle_initialize();
    void dynamic_initialize();
    void back_to_zero();
    void dynamic_release();
    
    
    Mat getGaussianKernel2D ( int ksize, double sigma );
    
    Mat src_img;
    Mat filtered_img;
    Mat src_rgb_split[3];
    Mat filtered_rgb_split[3];
    Mat div_circle_weight[SECTOR_N];
    
    
    double **eigen_vec_ori;
    double **Amo_Anisotropy;
    
    double ***src_rgb;
    double ***filter_rgb;
    
    double ***div_circle;
    
    
    int rows, cols;
    int map_circle_width, local_circle_width;
    
    
//#define PI              3.14159265
//    
//#define GAUSSIAN_SIGMA  2.0
//#define ECCEN_TUNING    1.0
//#define SIGMA_R         3.0
//#define SIGMA_S         3.0
//#define SECTOR_N        8
//#define SHARPNESS_Q     8.0
    
    
    
    
};



#endif

