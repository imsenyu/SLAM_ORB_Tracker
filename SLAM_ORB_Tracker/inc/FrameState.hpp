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
    std::string mLoadFormat;
    cv::Mat mImage;
    cv::Mat mT2w;
    cv::Mat mMatR;
    cv::Mat mMatT;
    cv::Mat mO2w;
    void updatePose(cv::Mat _mT) {
        mT2w = _mT.clone();
        updatePose();
    }
    void updatePose() {

        mMatR = mT2w.rowRange(0,3).colRange(0,3);
        mMatT = mT2w.rowRange(0,3).col(3);
        mO2w = - mMatR.t() * mMatT;

        setPainted(false);
    }
    std::vector<cv::KeyPoint> mvKeyPoint;
    std::vector<uchar>  mvMatchMask;
    std::vector<uchar>  mvbOutlier;
    cv::Mat mDescriptor;
    std::vector<shared_ptr<MapPoint>> mvpMapPoint;
    int mnTrackedType;

    FrameState(int _id, const std::string& _load = Config::sPathImageLoad);
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
    bool IsInImage(const float &x, const float &y) const
    {
        return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
    }
    bool isInFrustum(shared_ptr<MapPoint> pMP, float viewingCosLimit);

    boost::mutex mMutexPainted;
    bool mbPainted;
    bool isPainted() {
        boost::mutex::scoped_lock lock(mMutexPainted);
        return mbPainted;
    }
    void setPainted(bool _b = true) {
        boost::mutex::scoped_lock lock(mMutexPainted);
        mbPainted = _b;
    }
};

#endif /* FrameState_hpp */
