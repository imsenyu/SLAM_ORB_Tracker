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

    bool cache_mbMP;
    Tick cache_mtMP;
    std::set<shared_ptr<MapPoint>> cache_mspMapPoint;
    std::vector<shared_ptr<MapPoint>> cache_mvpMapPoint;

    bool cache_mbKF;
    Tick cache_mtKF;
    std::set<shared_ptr<KeyFrameState>> cache_mspKeyFrame;
    std::vector<shared_ptr<KeyFrameState>> cache_mvpKeyFrame;



public:
    Map();

    void insertKeyFrame(shared_ptr<KeyFrameState> _pKeyFrame);

    void insertMapPoint(shared_ptr<MapPoint> _pMapPoint);

    void EraseMapPoint(shared_ptr<MapPoint> pMP);

    // origin copy
    std::set<shared_ptr<MapPoint>> getAllSetMapPoint();
    std::vector<shared_ptr<MapPoint>> getAllVectorMapPoint();

    std::set<shared_ptr<KeyFrameState>> getAllSetKeyFrame();
    std::vector<shared_ptr<KeyFrameState>> getAllVectorKeyFrame();


    // cache reference
    std::set<shared_ptr<MapPoint>>& cacheRefGetAllSetMapPoint();
    std::vector<shared_ptr<MapPoint>>& cacheRefGetAllVectorMapPoint();

    std::set<shared_ptr<KeyFrameState>>& cacheRefGetAllSetKeyFrame();
    std::vector<shared_ptr<KeyFrameState>>& cacheRefGetAllVectorKeyFrame();

};


#endif //SLAM_ORB_TRACKER_MAP_HPP
