//
//  Tracker.hpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef Tracker_hpp
#define Tracker_hpp

#include "stdafx.hpp"
#include "FrameState.hpp"
#include "InputBuffer.hpp"
#include "Vocabulary.hpp"
#include "FrameDrawer.hpp"
#include "MapDrawer.hpp"
#include "PoseState.hpp"
#include "MotionState.hpp"

class Tracker {
private:
    
    typedef enum {
        Start,
        Fail,
        InitStep0,
        InitStep1,
        Normal,
    } WorkMode;
    
    typedef enum {
        Good,
        Bad
    } TrackResult;
    
    WorkMode meMode;
    WorkMode meLastMode;
    
    int threadRun();
    
    InputBuffer* mpInputBuffer;
    shared_ptr<FrameState> mpPreFrame;
    shared_ptr<FrameState> mpCurFrame;
    
    PoseState mCurPose;
    void initPose();
    
    
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    void updateDrawer();
    
    std::vector<cv::Point2f> mvPair[2];
    int match(shared_ptr<FrameState> pCurFrame, shared_ptr<FrameState> pPreFrame);
    bool computeMotion(shared_ptr<FrameState> pCurFrame, shared_ptr<FrameState> pPreFrame, MotionState& motion);
    
public:
    Tracker(InputBuffer* _pIB, FrameDrawer* _pFD, MapDrawer* _pMD);
    ~Tracker();
    
    int run();
    
    bool hasNext();
    
    
};

#endif /* Tracker_hpp */
