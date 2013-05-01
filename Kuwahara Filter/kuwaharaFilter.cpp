//
//  kuwaharaFilter.cpp
//  VC.proj
//
//  Created by  on 12/5/23.
//  Copyright (c) 2012年 __MyCompanyName__. All rights reserved.
//

#include <iostream>
#include "kuwaharaFilter.h"


kuwaharaFilter::kuwaharaFilter() {

    map_circle_width = (int)(2*ceil(2*SIGMA_R) + 1);
    local_circle_width = (int)(2*ceil(2*SIGMA_R) + 1);
    
    div_circle_initialize();

}




Mat kuwaharaFilter::image_filter(const string& fileName, int iterations) {
    
    Mat src_img = imread(fileName.c_str());
    cvtColor(src_img, src_img, CV_BGR2YCrCb);
    src_img.convertTo(src_img, CV_64FC3);
    src_img /= 255.0;
    
    
    
    resizeMat(src_img, 0.8);
    
    Mat filtered_img = src_img.clone().setTo(0);
    
    
    cout<<"image resolution : "<<src_img.rows<<"x"<<src_img.cols<<endl;
    
    Mat input_img = src_img.clone();
    
    for (int i = 0; i < iterations; i ++) {
                

        timeval tim;
        gettimeofday(&tim, NULL);
        double t1 = tim.tv_sec + (tim.tv_usec/1000000.0);
        
        filtered_img = anisotropic_kuwahara(input_img);
        
        gettimeofday(&tim, NULL);
        double t2 = tim.tv_sec + (tim.tv_usec/1000000.0);
        
        cout<<"anisotropic_kuwahara computation : "<<t2 - t1<<" secs.\n";
        
        input_img = filtered_img.clone();
        
//        showMat(filtered_img, "filtered image", 1);

    }
    
//    showMat(filtered_img, "filtered image", 0);
    
    
    
    filtered_img *= 255;
    filtered_img.convertTo(filtered_img, CV_8UC3);
    cvtColor(filtered_img, filtered_img, CV_YCrCb2BGR);
    
    return filtered_img;

}


void kuwaharaFilter::video_filter(const string &filePath) {
    
    VideoCapture video = VideoCapture(filePath.c_str());
    
    double reszie_ratio = 1.0;
    
    if ( video.isOpened() ) {
        
        Mat temp;
        video >> temp;
        
        resizeMat(temp, reszie_ratio);
        
        VideoWriter v_writer = VideoWriter("/Users/xup6qup3/Dropbox/code/Rosani.Dev/Kuwahara Filter/Kuwahara Filter/2.avi", CV_FOURCC('P','I','M','1'), 30, temp.size());
        
        while (video.grab()) {

            resizeMat(temp, reszie_ratio);
            
            temp.convertTo(temp, CV_64FC3);
            temp /= 255;
            
            Mat temp2 = temp.clone();
            
            for (int j = 0; j < 1; j ++) {
                
                temp = anisotropic_kuwahara(temp2);
                
                temp2 = temp.clone();
                
            }
            
            temp *= 255;
            temp.convertTo(temp, CV_8UC3);
            
            v_writer.write(temp);
            video >> temp;

        }
        
        v_writer.release();
    
    }
    
    video.release();
    
    
}



kuwaharaFilter::~kuwaharaFilter() {
    

    
}


void kuwaharaFilter::tensorComputation(const cv::Mat &src_img, cv::Mat &eigenVec_ori_cos, cv::Mat &eigenVec_ori_sin, cv::Mat &amo_anisotropy) {

    
    Mat src_gau = src_img.clone().setTo(0);
    GaussianBlur(src_img, src_gau, Size(3, 3), GAUSSIAN_SIGMA);
    
    Mat src_dx_split[3], src_dy_split[3], src_gau_split[3];
    split(src_gau, src_gau_split);
    
    for (int i = 0; i < 3; i ++) {
        
        src_dx_split[i] = Mat(src_img.rows, src_img.cols, CV_64FC1).setTo(0);
        src_dy_split[i] = Mat(src_img.rows, src_img.cols, CV_64FC1).setTo(0);
        
        Sobel(src_gau_split[i], src_dx_split[i], CV_64FC1, 1, 0, 1);
        Sobel(src_gau_split[i], src_dy_split[i], CV_64FC1, 0, 1, 1);
        
    }
    
    Mat src_dx = src_img.clone().setTo(0), src_dy = src_img.clone().setTo(0);
    
    merge(src_dx_split, 3, src_dx);
    merge(src_dy_split, 3, src_dy);
    
    
    Mat struct_tensor, eigen_val, eigen_vec;
    
    
    double lambda_one = 0.0, lambda_two = 0.0;


    
    eigen_val.create(2, 1, CV_64FC1);
    eigen_vec.create(2, 1, CV_64FC1);

    

    
    for (int i = 0; i < src_img.rows; i ++) {
        for (int j = 0; j < src_img.cols; j ++) {

            
            struct_tensor = Mat(2, 2, CV_64FC1).setTo(0);
            
            Vec3d dx_temp = src_dx.at<Vec3d>(i, j) , dy_temp = src_dy.at<Vec3d>(i, j);
            
            for (int k = 0; k < 3; k ++) {
                struct_tensor.at<double>(0, 0) += pow(dx_temp[k], 2.0);
                struct_tensor.at<double>(1, 0) += dx_temp[k]*dy_temp[k];
                struct_tensor.at<double>(0, 1) += dx_temp[k]*dy_temp[k];
                struct_tensor.at<double>(1, 1) += pow(dy_temp[k], 2.0);
            }
            
            
            eigen(struct_tensor, eigen_val, eigen_vec);
            
            lambda_one = ((double*)eigen_val.data )[0];
            lambda_two = ((double*)eigen_val.data + 1 )[0];
            
            double eigenVec_ori = atan2(lambda_one - struct_tensor.at<double>(0, 0), (-1)*struct_tensor.at<double>(1, 0));
            
            eigenVec_ori_cos.at<double>(i, j) = cos(eigenVec_ori);
            eigenVec_ori_sin.at<double>(i, j) = sin(eigenVec_ori);
            amo_anisotropy.at<double>(i, j) = (lambda_one - lambda_two)/(lambda_one + lambda_two);
        }
    }

    

}


Mat kuwaharaFilter::anisotropic_kuwahara(const cv::Mat &src_img) {
    
    
    Mat eigenVec_ori_cos = Mat(src_img.rows, src_img.cols, CV_64FC1).setTo(0);
    Mat eigenVec_ori_sin = Mat(src_img.rows, src_img.cols, CV_64FC1).setTo(0);
    Mat amo_anisotropy = Mat(src_img.rows, src_img.cols, CV_64FC1).setTo(0);
    

    tensorComputation(src_img, eigenVec_ori_cos, eigenVec_ori_sin, amo_anisotropy);
    
    
    
    Mat src_split[3], filtered_split[3];
    Mat filtered_img = src_img.clone().setTo(0);
    
    split(src_img, src_split);
    
        
    #pragma omp parallel for 
    for (int i = 0; i < 3; i ++) {
        filtered_split[i] = computationKernel(src_split[i], eigenVec_ori_cos, eigenVec_ori_sin, amo_anisotropy);
    }
    
    
    merge(filtered_split, 3, filtered_img);
    
    
    return filtered_img;

}


Mat kuwaharaFilter::computationKernel(const cv::Mat src_image, const cv::Mat &eigenVec_ori_cos, const cv::Mat &eigenVec_ori_sin, const cv::Mat &amo_anisotropy) {
    
    Mat filtered_image = src_image.clone().setTo(0);
    
    

    // temp use
    double temp_i = 0, temp_j = 0, temp_A = 0.0, temp_B = 0.0, temp_C = 0.0, temp_D = 0.0, temp_cos = 0.0, temp_sin = 0.0, temp_adA = 0.0, temp_Ada = 0.0, temp_det = 0.0;
    
    int map_i = 0, map_j = 0, half_l_width = 0, half_m_width = 0;
    double div_mean[8] = {0.0}, div_s[8] = {0.0}, weight_alpha[8] = {0.0}, normalize_k = 0.0, normalize_alpha = 0.0, eccen_S_A[2][2];
    
    double div_temp_data = 0.0, map_circle_data = 0.0, add_temp = 0.0, l_m_ratio = 0.0;
    
    Point2d map_centroid = Point2d(map_circle_width/2, map_circle_width/2);
    
    
    
    half_m_width = map_circle_width/2;
    half_l_width = local_circle_width/2;
    
    l_m_ratio = (double)local_circle_width/map_circle_width;
    
    double **m_circle, **l_circle;
    m_circle = new double*[map_circle_width];
    l_circle = new double*[local_circle_width];
    
    
    for (int i = 0; i < map_circle_width; i ++) {
        m_circle[i] = new double[map_circle_width];
        
        if ( i < local_circle_width ) {
            l_circle[i] = new double[local_circle_width];
        }
        
    }

    
    
    for (int i = 1; i < src_image.rows - 1; i ++) {
        for (int j = 1; j < src_image.cols - 1; j ++) {
            
            
            temp_cos = eigenVec_ori_cos.at<double>(i, j);
            temp_sin = eigenVec_ori_sin.at<double>(i, j);
            temp_adA = (ECCEN_TUNING)/(ECCEN_TUNING + amo_anisotropy.at<double>(i, j));
            temp_Ada = (ECCEN_TUNING + amo_anisotropy.at<double>(i, j))/(ECCEN_TUNING);
            
            temp_A = temp_cos*temp_adA;
            temp_B = (-1)*temp_sin*temp_Ada;
            temp_C = temp_sin*temp_adA;
            temp_D = temp_cos*temp_Ada;
            
            temp_det = 1.0/(temp_A*temp_D - temp_B*temp_C);
            
            eccen_S_A[0][0] = temp_D*temp_det;
            eccen_S_A[1][0] = (-1)*temp_B*temp_det;
            eccen_S_A[0][1] = (-1)*temp_C*temp_det;
            eccen_S_A[1][1] = temp_A*temp_det;
            
            
            
            for (int c_i = 0; c_i < map_circle_width; c_i ++) {
                for (int c_j = 0; c_j < map_circle_width; c_j ++) {
                    
                    m_circle[c_i][c_j] = 0;
                    
                    // temp_A = c_i upper, temp_B = c_j upper
                    temp_i = (int)(l_m_ratio*c_i) - half_l_width;
                    temp_j = (int)(l_m_ratio*c_j) - half_l_width;
                    
                    
                    if ( sqrt( temp_i*temp_i + temp_j*temp_j ) <= half_l_width ) {
                        
                        map_i = (int)eccen_S_A[0][0]*temp_i + eccen_S_A[0][1]*temp_j;
                        map_j = (int)eccen_S_A[1][0]*temp_i + eccen_S_A[1][1]*temp_j;
                        
                        
                        // map eclipse shape local texture to circle
                        if ( map_i + i >= 0 && map_i + i < src_image.rows && map_j + j >= 0 && map_j + j < src_image.cols ) {
                            
                            m_circle[c_i][c_j] = src_image.at<double>(map_i + i, map_j + j);
//                            src_rgb.at<Vec3d>(map_i + i, map_j + j)[rgb];
                            
                        }
                        
                    }
                    
                }
            }
            
            
            
            
            for (int s = 0; s < SECTOR_N; s ++) {
                
                
                div_mean[s] = 0.0;
                div_s[s] = 0.0;
                weight_alpha[s] = 0.0;
                normalize_k = 0.00000001;
                
//                temp_A = 0, temp_B = map_circle_width - 1, temp_C = 0, temp_D = map_circle_width - 1;
                
                
                for (int c_i = 0; c_i <= map_circle_width - 1; c_i ++) {
                    for (int c_j = 0; c_j <= map_circle_width - 1; c_j ++) {
                        
                        div_temp_data = div_circle[s][c_i][c_j];
                        map_circle_data = m_circle[c_i][c_j];
                        
                        
                        if ( map_circle_data <= 1 && map_circle_data >= 0.0000001
                            && div_temp_data <= 1 && div_temp_data >= 0.0000001 ) {
                            
                            div_mean[s] += map_circle_data*div_temp_data;
                            div_s[s] += map_circle_data*map_circle_data*div_temp_data;
                            normalize_k += div_temp_data;
                            
                        }
                        
                        
                    }
                }
                
                
                
                div_mean[s] = div_mean[s]/normalize_k;
                div_s[s] = sqrt(div_s[s]/normalize_k - div_mean[s]*div_mean[s]);
                
                
            }
            
            
            int min_index = 0;
            double min = INFINITY;
            for (int s = 0; s < SECTOR_N; s ++) {
                
                
                if ( min > div_s[s] ) {
                    min = div_s[s];
                    min_index = s;
                }
                
            }
            
            
            filtered_image.at<double>(i, j) = MAX(MIN(div_mean[min_index], 1), 0);
            
        }
    }
    
//    double max, min;
//    minMaxIdx(filtered_image, &min, &max);
//    
//    min = 0;
//    
//    filtered_image = (filtered_image - min)/(max - min);
    
//    showMat(filtered_image);
    
    

    
    for (int i = 0; i < map_circle_width; i ++) {
        delete [] m_circle[i];
        
    }
    
    delete [] m_circle;
    
    return filtered_image;
}


void kuwaharaFilter::div_circle_initialize() {
    
    double map_angle = 0.0, w_min = 0.0, w_max = 0.0;

    Mat gau_kernel = getGaussianKernel2D(map_circle_width, SIGMA_R);
    Mat div_rotate;
    
    Point2d map_centroid;
    
    int c_i = 0, c_j = 0, map_i = 0, map_j = 0;
    

    div_circle = new double**[SECTOR_N];
    for (int s = 0; s < SECTOR_N; s ++) {
        
        div_circle_weight[s].create(map_circle_width, map_circle_width, CV_64FC1);
        div_circle_weight[s].setTo(0);
        
        div_circle[s] = new double*[map_circle_width];
        for (int c_i = 0; c_i < map_circle_width; c_i ++) {
            div_circle[s][c_i] = new double[map_circle_width];
            
        }
    }

    
    for (int s = 0; s < SECTOR_N; s ++) {
        
        if ( s <= 1 ) {
            for (int c_i = 0; c_i < map_circle_width; c_i ++) {
                for (int c_j = 0; c_j < map_circle_width; c_j ++) {
                    
                    map_i = c_i - map_circle_width/2;
                    map_j = c_j - map_circle_width/2;
                    
                    if ( sqrt(pow( map_i, 2.0 ) + pow( map_j , 2.0)) < map_circle_width/2 ) {
                        
                        map_angle = atan2( map_i , map_j );
                        
                        if ( map_angle >= ( 2.0*s - 1 )*PI/SECTOR_N && map_angle <= ( 2.0*s + 1 )*PI/SECTOR_N ) {
                            
                            
                            div_circle_weight[s].at<double>(c_i, c_j) = 1.0;
                        }
                        
                    }
                    
                }
            }
            
            
            
            GaussianBlur(div_circle_weight[s], div_circle_weight[s], Size(13, 13), SIGMA_S);
            
            // multiply per element
            multiply(div_circle_weight[s], gau_kernel, div_circle_weight[s]);
            
            
            // calculate the max & min value of map_circle
            minMaxLoc(div_circle_weight[s], &w_min, &w_max);
            
            // Normalize
            div_circle_weight[s] = div_circle_weight[s]/w_max;
            
        }
        
        
        else {
            
            map_centroid.x = map_circle_width/2;
            map_centroid.y = map_circle_width/2;
            
            if ( s % 2 == 0 ) {    
                div_rotate = getRotationMatrix2D(map_centroid, s*(-360.0/SECTOR_N), 1.0);                    
                warpAffine(div_circle_weight[0], div_circle_weight[s], div_rotate, div_circle_weight[s].size());
            }
            
            else {
                div_rotate = getRotationMatrix2D(map_centroid, (s - 1)*(-360.0/SECTOR_N), 1.0);   
                warpAffine(div_circle_weight[1], div_circle_weight[s], div_rotate, div_circle_weight[s].size());
            }
            
            
        }
        

        for (int c_i = 0; c_i < map_circle_width; c_i ++) {
            for (int c_j = 0; c_j < map_circle_width; c_j ++) {
                
                div_circle[s][c_i][c_j] = div_circle_weight[s].at<double>(c_i, c_j);
                
            }
        }
        
    }
    
    gau_kernel.release();
    div_rotate.release();
    map_centroid.~Point_();

}


Mat kuwaharaFilter::getGaussianKernel2D(int ksize, double sigma) {
    
    double p_max = 0.0, p_temp = 0.0;
    
    sigma = sigma*2.0;
    
    Mat kernel = Mat(ksize, ksize, CV_64FC1).setTo(0);
    
    for (int i = 0; i < ksize; i ++) {
        for (int j = 0; j < ksize; j ++) {
            
            p_temp = (1.0/(2*PI*sigma))*exp((-1)*(pow(i - ksize/2, 2.0) + pow(j - ksize/2, 2.0))/(pow(2*sigma, 2)));
            
            kernel.at<double>(i, j) = p_temp;
            
            if ( p_temp > p_max ) {
                p_max = p_temp;
            }
        }
    }
    
    kernel /= p_max;
    

    
    return kernel;
}



