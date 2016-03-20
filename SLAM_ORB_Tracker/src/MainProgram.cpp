//
//  MainProgram.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/19/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "stdafx.hpp"
#include <boost/thread.hpp>
#include "Config.hpp"
#include "InputBuffer.hpp"

int main(int argc, char * argv[]) {
  
    // initialize Config for config load
    Config::parse(argc, argv);
    Config::loadConfig(Config::sPathConfigFile);
    
    // initialize InputBuffer for image read
    InputBuffer inputBuffer(Config::sPathImageLoad, Config::iImageLoadBegin, Config::iImageLoadEnd);
    boost::thread inputBufferThread( boost::bind(&InputBuffer::run, &inputBuffer) );
    
    
    while(1) {
        
        shared_ptr<FrameBuffer> ptrFrame = inputBuffer.get();
        cv::imshow("inputBuffer", ptrFrame->mImage);
        cv::waitKey(1000);
    }
    
    // wait for quit
    getchar();
    return 0;
}
