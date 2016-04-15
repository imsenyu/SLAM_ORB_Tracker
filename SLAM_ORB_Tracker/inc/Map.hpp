//
// Created by Sen Yu on 3/22/16.
// Copyright (c) 2016 Sen Yu. All rights reserved.
//

#ifndef SLAM_ORB_TRACKER_MAP_HPP
#define SLAM_ORB_TRACKER_MAP_HPP

#include "stdafx.hpp"
#include "KeyFrameState.hpp"
#include "MapPoint.hpp"

class MapPoint;
class KeyFrameState;

class Map {
private:
    std::set<shared_ptr<MapPoint>> mspMapPoint;
    std::set<shared_ptr<KeyFrameState>> mspKeyFrame;

    boost::mutex mMutexMP;
    boost::mutex mMutexKF;


public:
    Map();

    void insertKeyFrame(shared_ptr<KeyFrameState> _pKeyFrame);

    void insertMapPoint(shared_ptr<MapPoint> _pMapPoint);

    void EraseMapPoint(shared_ptr<MapPoint> pMP);

    // origin copy
    std::set<shared_ptr<MapPoint>> getAllSetMapPoint();
    std::set<shared_ptr<KeyFrameState>> getAllSetKeyFrame();
    std::vector<shared_ptr<MapPoint>> getAllVectorMapPoint();
    std::vector<shared_ptr<KeyFrameState>> getAllVectorKeyFrame();




};


#endif //SLAM_ORB_TRACKER_MAP_HPP
