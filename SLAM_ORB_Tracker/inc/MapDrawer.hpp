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
#include "Tracker.hpp"
#include "Map.hpp"
#include "GLWindow.hpp"

#include <opencv2/viz/viz3d.hpp>
#include <opencv2/viz/vizcore.hpp>
#include <boost/thread/pthread/mutex.hpp>

class Tracker;

class GLWindow;

class MapDrawer {
private:
    void initCanvas();
    void drawCanvas();

    cv::viz::Viz3d* mpVizWin;
    cv::Affine3f mCamPose;
    Map* mpMap;
    Tracker* mpTracker;

    std::ofstream fsout;
//    void initViz();
//    int drawViz();
    
    PoseState mPrePose;
    PoseState mCurPose;
    
    cv::Point2f mDrawBase;
    cv::Mat mPathCanvas;
    cv::Mat mPathCanvasWithDir;

    shared_ptr<FrameState> mpCurFrame;
    BlockingQueue<shared_ptr<FrameState>> mBuffer;

    void take();
    bool inited;


    boost::mutex mMutexGUI;
    boost::mutex mMutexPathCanvasWithDir;
    boost::mutex mMutexVizWin;

//
//    void drawMapPoint();

    GLWindow* mpGLWin;


    boost::mutex mMutexFrame;
    std::set<shared_ptr<FrameState>> mspFrame;
    bool cache_mbFrame;
    Tick cache_mtFrame;
    std::set<shared_ptr<FrameState>> cache_mspFrame;

public:
    MapDrawer(Map *_pMap);
    
    void update(shared_ptr<FrameState> pFS);
    void show();

    void threadRun();
    void setTracker(Tracker* _pTracker);
    void setGLWindow(GLWindow* _p);

    std::set<shared_ptr<FrameState>> getAllSetFrame();
    std::vector<shared_ptr<FrameState>> getAllVectorFrame();

    std::set<shared_ptr<FrameState>>& cacheRefGetAllSetFrame();
};

#endif /* MapDrawer_hpp */
