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
#include <sstream>
#include "Rosaniline.h"

#include <sys/time.h>


using namespace cv;
using namespace std;

class kuwaharaFilter {
public:
    
    kuwaharaFilter();
    ~kuwaharaFilter();
    
    Mat imageFilter(const Mat& input_img, int iterations);
    Mat imageFileFilter (const string& img_path, int iterations);
    
    void videoFilter(const string& src_path, const string& dst_path);

    
private:
    
    static const double PI              = 3.14159265;
    static const double GAUSSIAN_SIGMA  = 4.0;
    static const double ECCEN_TUNING    = 1.0;
    static const double SIGMA_R         = 3.0;
    static const double SIGMA_S         = 3.0;
    static const double SHARPNESS_Q     = 8.0;
    static const int    SECTOR_N        = 8;
    
    
    void tensorComputation (const Mat& src_img, Mat& eigenVec_ori_cos, Mat& eigenVec_ori_sin, Mat& amo_anisotropy);
    
    Mat anisotropic_kuwahara(const Mat& src_img);
    
    Mat computationKernel(const Mat src_image, const Mat& eigenVec_ori_cos, const Mat& eigenVec_ori_sin, const Mat& amo_anisotropy);
    
    void div_circle_initialize();
    
    
    
    
    Mat getGaussianKernel2D ( int ksize, double sigma );
    
    Mat div_circle_weight[SECTOR_N];
    

    
    double ***div_circle;
    
    int map_circle_width, local_circle_width;
    
    
    
    
};



#endif

