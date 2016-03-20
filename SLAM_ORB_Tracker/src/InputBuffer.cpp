//
//  InputBuffer.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "InputBuffer.hpp"


int InputBuffer::threadRun() {
    cv::Mat matImage;
    for(int idx = mBeginIdx; idx < mEndIdx; idx+=10 ) {
        cv::waitKey(100);
        std::string imgPath = cv::format(mLoadFormat.c_str(), idx);
        matImage = cv::imread(imgPath);
        cv::imshow(mWindowName, matImage);
        cv::waitKey(100);

        std::cout<< "show "<< imgPath <<std::endl;
    }
    return 0;
}
    

void InputBuffer::setWindows(std::string _windowName) {
    mWindowName = _windowName;
}
    
int InputBuffer::run() {
   
    return threadRun();
}