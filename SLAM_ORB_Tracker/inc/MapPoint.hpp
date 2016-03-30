//
// Created by Sen Yu on 3/25/16.
// Copyright (c) 2016 Sen Yu. All rights reserved.
//

#ifndef SLAM_ORB_TRACKER_MAPPOINT_HPP
#define SLAM_ORB_TRACKER_MAPPOINT_HPP

#include "stdafx.hpp"
#include "KeyFrameState.hpp"

class KeyFrameState;

class MapPoint: public boost::enable_shared_from_this<MapPoint> {
private:




public:
    std::map<shared_ptr<KeyFrameState>, int> msKeyFrame2FeatureId;
    shared_ptr<KeyFrameState> mpKeyFrameFirst;
    int mIdFromKeyFrame;

    MapPoint(cv::Mat _matMapPointPos, shared_ptr<KeyFrameState> _pKFS, int nP);
    void setKeyFrame(shared_ptr<KeyFrameState> _pKFS, int nP) {
        msKeyFrame2FeatureId[_pKFS] = nP;
    }
    int getUID();
    cv::Mat mPos;
};


#endif //SLAM_ORB_TRACKER_MAPPOINT_HPP
