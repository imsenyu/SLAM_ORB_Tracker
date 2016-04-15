//
// Created by Sen Yu on 3/22/16.
// Copyright (c) 2016 Sen Yu. All rights reserved.
//

#include "Map.hpp"

Map::Map() {

}


void Map::insertKeyFrame(shared_ptr<KeyFrameState> _pKeyFrame) {
    boost::mutex::scoped_lock lock(mMutexKF);
    mspKeyFrame.insert( _pKeyFrame );
}
void Map::insertMapPoint(shared_ptr<MapPoint> _pMapPoint) {
    boost::mutex::scoped_lock lock(mMutexMP);
    mspMapPoint.insert( _pMapPoint );
}
void Map::EraseMapPoint(shared_ptr<MapPoint> pMP) {
    boost::mutex::scoped_lock lock(mMutexMP);
    mspMapPoint.erase(pMP);
}


std::set<shared_ptr<MapPoint>> Map::getAllSetMapPoint() {
    boost::mutex::scoped_lock lock(mMutexMP);
    return mspMapPoint;
}

std::set<shared_ptr<KeyFrameState>> Map::getAllSetKeyFrame() {
    boost::mutex::scoped_lock lock(mMutexKF);
    return mspKeyFrame;
}

std::vector<shared_ptr<MapPoint>> Map::getAllVectorMapPoint() {
    boost::mutex::scoped_lock lock(mMutexMP);
    return std::vector<shared_ptr<MapPoint>>( mspMapPoint.begin(), mspMapPoint.end() );
}

std::vector<shared_ptr<KeyFrameState>> Map::getAllVectorKeyFrame() {
    boost::mutex::scoped_lock lock(mMutexKF);
    return std::vector<shared_ptr<KeyFrameState>>( mspKeyFrame.begin(), mspKeyFrame.end() );
}
