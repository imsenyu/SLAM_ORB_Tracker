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
    
    
    // initialize InputBuffer for image read
    InputBuffer inputBuffer("/Volumes/HDD/Workspace/Git/ZJU_Summer/Dataset/00/image_0/%06d.png", 1, 1000);
    inputBuffer.run();
    
    while(1) {
        std::cout<<"wait"<<std::endl;
        cv::waitKey(100);
    }
    
    // wait for quit
    getchar();
    return 0;
}
