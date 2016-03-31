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
#include "BlockingQueue.hpp"
#include "Map.hpp"

#include <opencv2/viz/viz3d.hpp>
#include <opencv2/viz/vizcore.hpp>

class MapDrawer {
private:
    void initCanvas();
    void drawCanvas();

    cv::viz::Viz3d* mpVizWin;
    cv::Affine3f mCamPose;
    Map* mpMap;

    void initViz();
    void drawViz();
    
    PoseState mPrePose;
    PoseState mCurPose;
    
    cv::Point2f mDrawBase;
    cv::Mat mPathCanvas;
    cv::Mat mPathCanvasWithDir;

    shared_ptr<FrameState> mpCurFrame;
    BlockingQueue<shared_ptr<FrameState>> mBuffer;
    void take();
    bool inited;
public:
    MapDrawer(Map *_pMap);
    
    void update(shared_ptr<FrameState> pFS);
    void show();

};

#endif /* MapDrawer_hpp */
