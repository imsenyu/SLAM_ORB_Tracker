//
//  FrameDrawer.hpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/21/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef FrameDrawer_hpp
#define FrameDrawer_hpp

#include "stdafx.hpp"
#include "FrameState.hpp"
#include "ConcurrentQueue.hpp"

class FrameDrawer {
private:
    int mId;
    cv::Mat mImage;
    cv::Mat mImageDraw;
    std::vector<cv::KeyPoint> mvKeyPoint;
    ConcurrentQueue<shared_ptr<FrameState>> mBuffer;
    
    
    void drawFeaturePoint();
    void drawText();
    void get();
public:
    FrameDrawer();
    void update(shared_ptr<FrameState> _pFS );
    void show();
};

#endif /* FrameDrawer_hpp */
