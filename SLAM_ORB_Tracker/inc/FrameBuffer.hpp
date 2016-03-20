//
//  FrameBuffer.hpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef FrameBuffer_hpp
#define FrameBuffer_hpp

#include "stdafx.hpp"

class FrameBuffer {
private:
    bool loadImage(int _id);
    
public:
    int mId;
    bool mLoaded;
    cv::Mat mImage;
    
    FrameBuffer(int _id);
};

#endif /* FrameBuffer_hpp */
