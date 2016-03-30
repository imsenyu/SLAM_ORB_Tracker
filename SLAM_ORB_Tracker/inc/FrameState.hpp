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
#include "MapPoint.hpp"

// ORB
#include "ORBextractor.h"

class MapPoint;
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

using std::vector;
using std::max;
using std::min;

class FrameState: public boost::enable_shared_from_this<FrameState> {
private:
    bool loadImage(int _id);
    
public:
    int mId;
    bool mLoaded;
    cv::Mat mImage;
    cv::Mat mT2w; // the 4x4 matrix transform one point from startposition's coord to this cameraposition's coord
    std::vector<cv::KeyPoint> mvKeyPoint;
    std::vector<uchar>  mvMatchMask;
    std::vector<uchar>  mvbOutlier;
    cv::Mat mDescriptor;
    std::vector<shared_ptr<MapPoint>> mvpMapPoint;
    
    FrameState(int _id);
    FrameState(const FrameState& _FS);
    FrameState(const FrameState* _pFS);
    ~FrameState();


    int extract();
    int extractInit();
    int extractNormal();
    void insertMapPoint(shared_ptr<MapPoint> pMp, int nP) {
        mvpMapPoint[nP] = pMp;
    }
    std::vector<size_t> getFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel = -1, const int maxLevel = -1) const;
    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    static int mnMinX;
    static int mnMaxX;
    static int mnMinY;
    static int mnMaxY;
    static bool mbInitialComputations;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
};

#endif /* FrameState_hpp */
