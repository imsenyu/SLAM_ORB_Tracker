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

class MapPoint;

class FrameState {
private:
    bool loadImage(int _id);
    
public:
    int mId;
    bool mLoaded;
    cv::Mat mImage;
    cv::Mat mT2w; // the 4x4 matrix transform one point from startposition's coord to this cameraposition's coord
    std::vector<cv::KeyPoint> mvKeyPoint;
    std::vector<uchar>  mvMatchMask;
    cv::Mat mDescriptor;
    std::vector<shared_ptr<MapPoint>> mvpMapPoint;
    
    FrameState(int _id);
    FrameState(const FrameState& _FS);
    FrameState(const FrameState* _pFS);
    ~FrameState();


    int extract();
    void insertMapPoint(shared_ptr<MapPoint> pMp, int nP) {
        mvpMapPoint[nP] = pMp;
    }

};

#endif /* FrameState_hpp */
