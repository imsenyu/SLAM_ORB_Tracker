//
//  FrameState.hpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef FrameState_hpp
#define FrameState_hpp

#include "stdafx.hpp"

class FrameState {
private:
    bool loadImage(int _id);
    
public:
    int mId;
    bool mLoaded;
    cv::Mat mImage;
    
    std::vector<cv::KeyPoint> mvKeyPoint;
    cv::Mat mDescriptor;
    
    FrameState(int _id);
    FrameState(const FrameState& _FS);
    FrameState(const FrameState* _pFS);
    ~FrameState();
    
    
    
    int extract();

};

#endif /* FrameState_hpp */
