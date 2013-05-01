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

    map_circle_width = (2*ceil(2*SIGMA_R) + 1);
    local_circle_width = (2*ceil(2*SIGMA_R) + 1);
    
    rows = 0;
    cols = 0;
    
    div_circle_initialize();

    
}


void kuwaharaFilter::dynamic_initialize() {
    
    src_rgb = new double**[3];
    filter_rgb = new double**[3];
    

    for (int i = 0; i < 3; i ++) {
        
        src_rgb_split[i].create(rows, cols, CV_64FC1);
        src_rgb_split[i].setTo(0);
        
        filtered_rgb_split[i].create(rows, cols, CV_64FC1);
        filtered_rgb_split[i].setTo(0);

        src_rgb[i] = new double*[rows];
        filter_rgb[i] = new double*[rows];
        
        for (int j = 0; j < rows; j ++) {
            
            src_rgb[i][j] = new double[cols];
            filter_rgb[i][j] = new double[cols];
            
        }
    }
    
    eigen_vec_ori = new double*[rows];
    Amo_Anisotropy = new double*[rows];
    
    for (int i = 0; i < rows; i ++) {
            
        eigen_vec_ori[i] = new double[cols];
        Amo_Anisotropy[i] = new double[cols];
        
    }
    
    
    
}


void kuwaharaFilter::dynamic_release() {

    for (int i = 0; i < 3; i ++) {
        
        src_rgb_split[i].release();
        filtered_rgb_split[i].release();
        
        
        for (int j = 0; j < rows; j ++) {
            delete [] src_rgb[i][j];
            delete [] filter_rgb[i][j];
            
        }
    }
    
    delete [] src_rgb;
    delete [] filter_rgb;
    

    for (int i = 0; i < rows; i ++) {    
        delete [] eigen_vec_ori[i];
        delete [] Amo_Anisotropy[i];
        
    }
    
}


void kuwaharaFilter::image_filter(const string& fileName, int iterations) {
    
    src_img = imread(fileName.c_str(), 1);
    
    src_img.convertTo(src_img, CV_64F);
    src_img = src_img/255.0;
    

    
    clock_t start, end;
    
    cout<<"image resolution : "<<src_img.rows<<"x"<<src_img.cols<<endl;
    
    for (int i = 0; i < iterations; i ++) {
        
        if ( src_img.rows != rows && src_img.cols != cols ) {
            
            if ( ! ( rows == 0 && cols == 0 )) {
                dynamic_release();
            }
            
            rows = src_img.rows;
            cols = src_img.cols;
            
            dynamic_initialize();
        }
        
        else {
            dynamic_release();
            dynamic_initialize();
        }
        
        
        start = clock();
        tensor_computation();
        end = clock();
        cout<<"tensor computation : "<<end - start<<" sec : "<<(end - start)/CLOCKS_PER_SEC<<endl;
        
        start = clock();
        anisotropic_kuwahara();
        end = clock();
        cout<<"anisotropic_kuwahara computation : "<<end - start<<" sec : "<<(end - start)/CLOCKS_PER_SEC<< endl<<endl;
        
        showMat(filtered_img, "filtered image", 0);

    }
    


}


void kuwaharaFilter::video_filter(char *filePath, int start_frame, int frame_count) {
    
    Mat frame_img;
    stringstream file;
    clock_t start, end;
    
    for (int i = start_frame; i <= frame_count; i ++) {
        
        cout<<"frame #"<<i<<endl;
        
        
        file.str(string());
        if ( i < 10 ) {
            file<<filePath<<" 000"<<i<<".jpg";
        }
        
        else if ( i < 100 ) {
            file<<filePath<<" 00"<<i<<".jpg";
        }
        
        else if ( i < 1000 ) {
            file<<filePath<<" 0"<<i<<".jpg";
        }
        
        else {
            file<<filePath<<" "<<i<<".jpg";
        }
        

        cout<<file.str();
        
        
        frame_img = imread(file.str().c_str(), 1);
        
        

        src_img.create(frame_img.rows, frame_img.cols, CV_64F);
        resize(frame_img, src_img, src_img.size());
        
        
        if ( rows == 0 && cols == 0 ) {
        
            rows = src_img.rows;
            cols = src_img.cols;
            
            dynamic_initialize();
        }
        
        else {
            
            dynamic_release();
            dynamic_initialize();
        }
        
        
        
        src_img.convertTo(src_img, CV_64F);
        src_img = src_img/255.0;
        

        cout<<"resolution : "<<src_img.rows<<"*"<<src_img.cols<<endl;
        
        start = clock();
        tensor_computation();
        end = clock();
        cout<<"tensor computation : "<<end - start<<" clocks,  "<<(end - start)/CLOCKS_PER_SEC<<" seconds"<<endl;
        
        start = clock();
        anisotropic_kuwahara();
        end = clock();
        cout<<"anisotropic_kuwahara computation : "<<end - start<<" clocks,  "<<(end - start)/CLOCKS_PER_SEC<<" seconds"<<endl<<endl;
        
        
        
        file.str(string());
        file<<filePath<<"/output/"<<i<<".jpg";
        filtered_img = filtered_img*255.0;
        filtered_img.convertTo(filtered_img, CV_8U);
//        imshow("f", filtered_img);
//        waitKey(0);
        imwrite(file.str().c_str(), filtered_img);
        filtered_img.setTo(0);
        src_img.setTo(0);
        
        
        
        
    }
    
    
}


kuwaharaFilter::~kuwaharaFilter() {
    

    
}


void kuwaharaFilter::tensor_computation() {
    
    Mat src_gau, struct_tensor, eigen_val, eigen_vec;
    Mat src_RGB_dx[3], src_RGB_dy[3], src_gau_split[3];
    
    
    double tensor_E = 0.0, tensor_F = 0.0, tensor_G = 0.0, dx_temp = 0.0, dy_temp = 0.0, lambda_one = 0.0, lambda_two = 0.0;
    
    // for loop usage
    int i = 0, j = 0, rgb = 0;

    
    GaussianBlur(src_img, src_gau, Size(3, 3), GAUSSIAN_SIGMA);


    for (i = 0; i < 3; i ++) {
        src_RGB_dx[i].create(rows, cols, CV_64FC1);
        src_RGB_dy[i].create(rows, cols, CV_64FC1);
        
    }
    
    
    // split RGB image into R,G,B
    split(src_gau, src_gau_split);
    
    
    for (rgb = 0; rgb < 3; rgb ++) {

        Sobel(src_gau_split[rgb], src_RGB_dx[rgb], CV_64FC1, 1, 0, 1);
        Sobel(src_gau_split[rgb], src_RGB_dy[rgb], CV_64FC1, 0, 1, 1);
        
    }    
    
    
    split(src_img, src_rgb_split);
    
    for (rgb = 0; rgb < 3; rgb ++) {
        for (int i = 0; i < rows; i ++) {
            for (int j = 0; j < cols; j ++) {
                
                src_rgb[rgb][i][j] = src_gau_split[rgb].at<double>(i, j);
                
            }
        }
    }

    
    struct_tensor.create(2, 2, CV_64FC1);
    eigen_val.create(2, 1, CV_64FC1);
    eigen_vec.create(2, 1, CV_64FC1);

    

    
    for (i = 0; i < rows; i ++) {
        for (j = 0; j < cols; j ++) {

            tensor_E = 0.0, tensor_F = 0.0, tensor_G = 0.0;
            
            for (rgb = 0; rgb < 3; rgb ++) {
                
                dx_temp = ((double*)src_RGB_dx[rgb].data + cols*i + j )[0];
                dy_temp = ((double*)src_RGB_dy[rgb].data + cols*i + j )[0];
                
                tensor_E += pow(dx_temp, 2.0);
        
                tensor_F += dx_temp * dy_temp;

                tensor_G += pow(dy_temp, 2.0);
                
            }
            
            struct_tensor.at<double>(0, 0) = tensor_E;
            struct_tensor.at<double>(1, 0) = tensor_F;
            struct_tensor.at<double>(0, 1) = tensor_F;
            struct_tensor.at<double>(1, 1) = tensor_G;
            
            eigen(struct_tensor, eigen_val, eigen_vec);
            
            lambda_one = ((double*)eigen_val.data )[0];
            lambda_two = ((double*)eigen_val.data + 1 )[0];
            
            
            eigen_vec_ori[i][j] = atan2(lambda_one - tensor_E, (-1)*tensor_F);
            Amo_Anisotropy[i][j] = (lambda_one - lambda_two)/(lambda_one + lambda_two);
        }
    }
    
    
    
    src_gau.release();
    struct_tensor.release();
    eigen_val.release();
    eigen_vec.release();
    
    for (i = 0; i < 3; i ++) {
        src_RGB_dx[i].release();
        src_RGB_dy[i].release();
        src_gau_split[i].release();
    }
    

}


void kuwaharaFilter::anisotropic_kuwahara() {
    
    // for loop usage
    int i = 0, j = 0, rgb = 0, c_i = 0, c_j = 0, s = 0;

    
    // temp use
    double temp_i = 0, temp_j = 0, temp_A = 0.0, temp_B = 0.0, temp_C = 0.0, temp_D = 0.0, temp_cos = 0.0, temp_sin = 0.0, temp_adA = 0.0, temp_Ada = 0.0, temp_det = 0.0;
    
    int map_i = 0, map_j = 0, half_l_width = 0, half_m_width = 0;
    double div_mean[8] = {0.0}, div_s[8] = {0.0}, weight_alpha[8] = {0.0}, normalize_k = 0.0, normalize_alpha = 0.0, eccen_S_A[2][2];
    
    double div_temp_data = 0.0, map_circle_data = 0.0, add_temp = 0.0, l_m_ratio = 0.0;
    
    
    Point2d map_centroid;
    map_centroid.x = map_circle_width/2;
    map_centroid.y = map_circle_width/2;    
    

    half_m_width = map_circle_width/2;
    half_l_width = local_circle_width/2;
    
    l_m_ratio = (double)local_circle_width/map_circle_width;

    
    double **m_circle, **l_circle;
    m_circle = new double*[map_circle_width];
    l_circle = new double*[local_circle_width];
    
    
    for (i = 0; i < map_circle_width; i ++) {
        m_circle[i] = new double[map_circle_width];
        
        if ( i < local_circle_width ) {
            l_circle[i] = new double[local_circle_width];
        }
        
    }
    

    
    for (rgb = 0; rgb < 3; rgb ++) {
        
        cout<<"Computing channel ";
        switch (rgb) {
            case 0:
                cout<<"B ..."<<endl;
                break;
                
            case 1:
                cout<<"G ..."<<endl;
                break;
                
            case 2:
                cout<<"R ..."<<endl;
                break;
        }
        
        for (i = 1; i < rows - 1; i ++) {
            for (j = 1; j < cols - 1; j ++) {
                
                
                temp_cos = cos(eigen_vec_ori[i][j]);
                temp_sin = sin(eigen_vec_ori[i][j]);
                temp_adA = (ECCEN_TUNING)/(ECCEN_TUNING + Amo_Anisotropy[i][j]);
                temp_Ada = (ECCEN_TUNING + Amo_Anisotropy[i][j])/(ECCEN_TUNING);
                
                temp_A = temp_cos*temp_adA;
                temp_B = (-1)*temp_sin*temp_Ada;
                temp_C = temp_sin*temp_adA;
                temp_D = temp_cos*temp_Ada;
                
                temp_det = 1.0/(temp_A*temp_D - temp_B*temp_C);
                
                eccen_S_A[0][0] = temp_D*temp_det;
                eccen_S_A[1][0] = (-1)*temp_B*temp_det;
                eccen_S_A[0][1] = (-1)*temp_C*temp_det;
                eccen_S_A[1][1] = temp_A*temp_det;
                
                
                
                for (c_i = 0; c_i < map_circle_width; c_i ++) {
                    for (c_j = 0; c_j < map_circle_width; c_j ++) {
                        
                        m_circle[c_i][c_j] = 0;
                        
                        // temp_A = c_i upper, temp_B = c_j upper                        
                        temp_i = (int)(l_m_ratio*c_i) - half_l_width;
                        temp_j = (int)(l_m_ratio*c_j) - half_l_width;
                        
                        
                        if ( sqrt( temp_i*temp_i + temp_j*temp_j ) <= half_l_width ) {
                            
                            map_i = eccen_S_A[0][0]*temp_i + eccen_S_A[0][1]*temp_j;
                            map_j = eccen_S_A[1][0]*temp_i + eccen_S_A[1][1]*temp_j;
                            
                            
                            // map eclipse shape local texture to circle
                            if ( map_i + i >= 0 && map_i + i < rows && map_j + j >= 0 && map_j + j < cols ) {
                                
                                m_circle[c_i][c_j] = src_rgb[rgb][map_i + i][map_j + j];

                            }
                            
                        }

                    }
                }
                


                
                for (s = 0; s < SECTOR_N; s ++) {
                    

                    div_mean[s] = 0.0;
                    div_s[s] = 0.0;
                    weight_alpha[s] = 0.0;
                    normalize_k = 0.00000001;
//                    
//                    switch (s) {
//                        case 0:
//                            temp_A = (1.0/4.0)*map_circle_width;
//                            temp_B = (3.0/4.0)*map_circle_width;
//                            temp_C = (1.0/2.0)*map_circle_width;
//                            temp_D = map_circle_width - 1;
//                            break;
//                            
//                        case 1:
//                            temp_A = (1.0/2.0)*map_circle_width;
//                            temp_B = map_circle_width - 1;
//                            temp_C = (1.0/2.0)*map_circle_width;
//                            temp_D = map_circle_width - 1;
//                            break;
//                            
//                        case 2:
//                            temp_A = (1.0/2.0)*map_circle_width;
//                            temp_B = map_circle_width - 1;
//                            temp_C = (1.0/4.0)*map_circle_width;
//                            temp_D = (3.0/4.0)*map_circle_width;
//                            break;
//                            
//                        case 3:
//                            temp_A = (1.0/2.0)*map_circle_width;
//                            temp_B = map_circle_width - 1;
//                            temp_C = 0;
//                            temp_D = map_circle_width/2;
//                            break;
//                            
//                        case 4:
//                            temp_A = (1.0/4.0)*map_circle_width;
//                            temp_B = (3.0/4.0)*map_circle_width;
//                            temp_C = 0;
//                            temp_D = map_circle_width/2;
//                            break;
//                            
//                        case 5:
//                            temp_A = 0;
//                            temp_B = (1.0/2.0)*map_circle_width;
//                            temp_C = 0;
//                            temp_D = (1.0/2.0)*map_circle_width;
//                            break;                            
//                            
//                        case 6:
//                            temp_A = 0;
//                            temp_B = (1.0/2.0)*map_circle_width;
//                            temp_C = (1.0/4.0)*map_circle_width;
//                            temp_D = (3.0/4.0)*map_circle_width;
//                            break;
//                            
//                        case 7:
//                            temp_A = 0;
//                            temp_B = (1.0/2.0)*map_circle_width;
//                            temp_C = (1.0/2.0)*map_circle_width;
//                            temp_D = map_circle_width - 1;
//                            break;
//                                                        
//                            
//                            
//                    }
                    
                    temp_A = 0, temp_B = map_circle_width - 1, temp_C = 0, temp_D = map_circle_width - 1;
                    
//                    Mat a, b, c, d;
//                    a.create(map_circle_width, map_circle_width, CV_64FC1);
//                    
//                    
//                    for (int m = 0; m < map_circle_width; m ++) {
//                        for (int n = 0; n < map_circle_width; n ++) {
//                            
//                            a.at<double>(m, n) = m_circle[m][n];
//                            
//                        }
//                    }
//                    
//                    b.create(500, 500, CV_64FC1);
//                    resize(a, b, b.size());
//                    imshow("b", b);
//                    waitKey(0);
//                    a.setTo(0);
//                    b.setTo(0);
                    
                    for (c_i = temp_A; c_i <= temp_B; c_i ++) {
                        for (c_j = temp_C; c_j <= temp_D; c_j ++) { 

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
                    
//                    cout<<"m : "<<div_mean[s]<<" s : "<<div_s[s]<<" , "<<endl;;
                    
//                    weight_alpha[s] = 1.0/(1 + pow(div_s[s], SHARPNESS_Q));
                    
                }
                
//                cout<<endl;
                
                
//                normalize_alpha = 0.0;
//                add_temp = 0.0;
//                
//                for (s = 0; s < SECTOR_N; s ++) {
//                    
//                    add_temp += weight_alpha[s]*div_mean[s];
//                    normalize_alpha += weight_alpha[s];
//                    
//                }
//                
//                for (s = 0; s < SECTOR_N; s ++) {
//                    
//                    
//                    weight_alpha[s] = weight_alpha[s]/normalize_alpha;
//                    filter_rgb[rgb][i][j] = add_temp*weight_alpha[s];
//                }
                
                
                int min_index = 0;
                double min = INFINITY;
                for (s = 0; s < SECTOR_N; s ++) {
                    
                    
                    if ( min > div_s[s] ) {
                        min = div_s[s];
                        min_index = s;
                    }
                    
                }
                
                
                filter_rgb[rgb][i][j] = MAX(MIN(div_mean[min_index], 255), 0);

            }
        }
        
    }
    
    
    for (rgb = 0; rgb < 3; rgb ++) {
        for (i = 0; i < rows; i ++) {
            for (j = 0; j < cols; j ++) {
                
                filtered_rgb_split[rgb].at<double>(i, j) = filter_rgb[rgb][i][j];
                
            }
        }
    }
    

    filtered_img.create(rows, cols, CV_64FC3);
    filtered_img.setTo(0);
    merge(filtered_rgb_split, 3, filtered_img);
    
    
    
    
    for (i = 0; i < map_circle_width; i ++) {
        delete [] m_circle[i];
        
    }

    delete [] m_circle;

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
            for (c_i = 0; c_i < map_circle_width; c_i ++) {
                for (c_j = 0; c_j < map_circle_width; c_j ++) {
                    
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
        

        for (c_i = 0; c_i < map_circle_width; c_i ++) {
            for (c_j = 0; c_j < map_circle_width; c_j ++) {
                
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
    
    Mat kernel;
    kernel.create(ksize, ksize, CV_64FC1);
    kernel.setTo(0);

    
    for (int i = 0; i < ksize; i ++) {
        for (int j = 0; j < ksize; j ++) {
            
            p_temp = (1.0/(2*PI*sigma))*exp((-1)*(pow(i - ksize/2, 2.0) + pow(j - ksize/2, 2.0))/(pow(2*sigma, 2)));
            
            kernel.at<double>(i, j) = p_temp;
            
            if ( p_temp > p_max ) {
                p_max = p_temp;
            }
        }
    }
    
    
    for (int i = 0; i < ksize; i ++) {
        for (int j = 0; j < ksize; j ++) {

            kernel.at<double>(i, j) = kernel.at<double>(i, j)/p_max;
            
        }
    }

    
    return kernel;
}



