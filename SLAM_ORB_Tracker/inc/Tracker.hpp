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
#include "MapInitializer.hpp"
#include "Initializer.h"

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


private:
    int threadRun();
    
    InputBuffer* mpInputBuffer;
    shared_ptr<FrameState> mpPreFrame;
    shared_ptr<FrameState> mpCurFrame;
    shared_ptr<FrameState> mpIniFrame;
    
    PoseState mCurPose;
    void initPose();
    
    
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    void updateDrawer();
    
    std::vector<int> mvMatchPair12;
    std::vector<uchar> mvMatchMask12;
    std::vector<cv::Point2f> mvMatchPoint[2];
    int match(shared_ptr<FrameState> pPreFrame, shared_ptr<FrameState> pCurFrame, std::vector<int>& vMatchPair12, std::vector<uchar>& vMatchMask12, std::vector<cv::Point2f> vMatchPoint[2]);
    int filterByOpticalFlow(shared_ptr<FrameState> pPreFrame, shared_ptr<FrameState> pCurFrame, std::vector<int>& vMatchPair12, std::vector<uchar>& vMatchMask12, std::vector<cv::Point2f> vMatchPoint[2]);
    //bool computeMotion(shared_ptr<FrameState> pPreFrame, shared_ptr<FrameState> pCurFrame, std::vector<cv::Point2f> *vMatchPoint, MotionState &motion, std::vector<uchar> &vMatchMask);
    //void triangulateDLT(cv::Point2f &xl,cv::Point2f &xr,cv::Mat &Pl,cv::Mat &Pr, cv::Mat &point3d);
    //int checkRT(cv::Mat matR, cv::Mat matT, std::vector<cv::Point2f> *vMatchPoint, std::vector<uchar> &vMatchMask);
    //void drawFilter(shared_ptr<FrameState> pFrame, std::vector<cv::Point2f>& mvPoint);

    ORB_SLAM::Initializer* mpIniter;
    void initStepFirstKeyFrame();
    MotionState mMotion;
    bool initStepSecondKeyFrame();
    bool initStepBuildMap(MotionState initMotion);

public:
    Tracker(InputBuffer* _pIB, FrameDrawer* _pFD, MapDrawer* _pMD);
    ~Tracker();
    
    int run();

    
};

#endif /* Tracker_hpp */
