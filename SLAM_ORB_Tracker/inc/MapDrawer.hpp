//
//  MapDrawer.hpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/21/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef MapDrawer_hpp
#define MapDrawer_hpp

#include "stdafx.hpp"
#include "PoseState.hpp"
#include "MotionState.hpp"
#include "ConcurrentQueue.hpp"

class MapDrawer {
private:
    void initCanvas();
    void drawCanvas();
    
    PoseState mPrePose;
    PoseState mCurPose;
    
    cv::Point2f mDrawBase;
    cv::Mat mPathCanvas;
    cv::Mat mPathCanvasWithDir;
    
    ConcurrentQueue<PoseState> mBuffer;
    void get();
    bool inited;
public:
    MapDrawer();
    
    void update(PoseState& _poseState);
    void show();
};

#endif /* MapDrawer_hpp */
