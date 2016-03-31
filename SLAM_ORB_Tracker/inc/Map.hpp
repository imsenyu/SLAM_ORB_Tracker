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
public:
    std::set<shared_ptr<MapPoint>> mspMapPoint;
    std::set<shared_ptr<KeyFrameState>> mspKeyFrame;
public:
    Map();

    void insertKeyFrame(shared_ptr<KeyFrameState> _pKeyFrame) {
        mspKeyFrame.insert( _pKeyFrame );
    }
    void insertMapPoint(shared_ptr<MapPoint> _pMapPoint) {
        mspMapPoint.insert( _pMapPoint );
    }
    void EraseMapPoint(shared_ptr<MapPoint> pMP)
    {
        mspMapPoint.erase(pMP);
    }
};


#endif //SLAM_ORB_TRACKER_MAP_HPP
