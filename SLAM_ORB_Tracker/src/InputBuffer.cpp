//
//  InputBuffer.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "InputBuffer.hpp"


void InputBuffer::threadRun() {
    cv::Mat matImage;
    for(int idx = mBeginIdx; idx < mEndIdx; idx+=10 ) {
        cv::waitKey(100);
        std::string imgPath = cv::format(mLoadFormat.c_str(), idx);
        matImage = cv::imread(imgPath);
        cv::imshow("test", matImage);
        cv::waitKey(100);

        std::cout<< "show "<< imgPath <<std::endl;
        //for(int j=0;j<1e9;j++);
    }
    
}
    
    

    
void InputBuffer::run() {
    cv::namedWindow("test");
    boost::thread thread( boost::bind(&InputBuffer::threadRun, this) );
    //thread.join();
    
    //threadRun();
    return;
    
}