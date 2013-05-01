//
//  main.cpp
//  Kuwahara Filter
//
//  Created by Rosani Lin on 12/8/3.
//  Copyright (c) 2012年 Rosani Lin. All rights reserved.
//

#include <iostream>
#include "kuwaharaFilter.h"

int main(int argc, const char * argv[])
{
    kuwaharaFilter *k_filter = new kuwaharaFilter();
//    Mat temp = k_filter->image_filter("/Users/xup6qup3/Dropbox/Photos/anne-hathaway-movie-catwoman-the-dark-knight-rises-christopher-nolan-800x1280.jpeg", 3);
//
//    imwrite("/Users/xup6qup3/Dropbox/code/Rosani.Dev/Kuwahara Filter/Kuwahara Filter/7.JPG", temp);
    
    k_filter->video_filter("/Users/xup6qup3/Dropbox/yeon_T-ARA N4 - 泬埶暮 (Dance ver.)_CNSUB.mkv");
    
    
//    Mat temp = imread("/Users/xup6qup3/Dropbox/Photos/anne-hathaway-movie-catwoman-the-dark-knight-rises-christopher-nolan-800x1280.jpeg");
//    
//    VideoWriter vwriter = VideoWriter("/Users/xup6qup3/Dropbox/code/Rosani.Dev/Kuwahara Filter/Kuwahara Filter/1.avi", CV_FOURCC('P','I','M','1'), 30, temp.size());
//    
//    if ( vwriter.isOpened() ) {
//        for (int i = 0; i < 300; i ++) {
//            
//            vwriter.write(temp);
//        }
//    }
//    
//    vwriter.release();

    return 0;
}

