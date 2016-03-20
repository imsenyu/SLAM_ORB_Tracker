//
//  MainProgram.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/19/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "stdafx.hpp"
#include "Config.hpp"
#include "InputBuffer.hpp"

int main(int argc, char * argv[]) {
  
    // initialize Config for config load
    Config::parse(argc, argv);
    Config::loadConfig(Config::sPathConfigFile);
    
    // initialize InputBuffer for image read
    InputBuffer inputBuffer(Config::sPathImageLoad, Config::iImageLoadBegin, Config::iImageLoadEnd);
    inputBuffer.setWindows("buffer");
    boost::thread inputBufferThread( boost::bind(&InputBuffer::run, &inputBuffer) );
    
    
    while(1) {
        std::cout<<"wait"<<std::endl;
        cv::waitKey(100);
    }
    
    // wait for quit
    getchar();
    return 0;
}
