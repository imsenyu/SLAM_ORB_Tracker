//
// Created by Sen Yu on 3/22/16.
// Copyright (c) 2016 Sen Yu. All rights reserved.
//

#include "Map.hpp"

Map::Map():
    cache_mbMP(false),
    cache_mtMP(15),
    cache_mbKF(false),
    cache_mtKF(10)
{

}


void Map::insertKeyFrame(shared_ptr<KeyFrameState> _pKeyFrame) {
    boost::mutex::scoped_lock lock(mMutexKF);
    cache_mbKF = true;
    mspKeyFrame.insert( _pKeyFrame );
}
void Map::insertMapPoint(shared_ptr<MapPoint> _pMapPoint) {
    boost::mutex::scoped_lock lock(mMutexMP);
    cache_mbMP = true;
    mspMapPoint.insert( _pMapPoint );
}
void Map::EraseMapPoint(shared_ptr<MapPoint> pMP) {
    boost::mutex::scoped_lock lock(mMutexMP);
    cache_mbMP = true;
    // ptrMapPoint has been setted to bad
    // non need to release or release delay
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














std::set<shared_ptr<MapPoint>> &Map::cacheRefGetAllSetMapPoint() {
    if ( cache_mbMP && cache_mtMP.try_tock() ) {
        boost::mutex::scoped_lock lock(mMutexMP);
        cache_mspMapPoint = mspMapPoint;
        cache_mbMP = false;
    }
    return cache_mspMapPoint;
}

std::vector<shared_ptr<MapPoint>> &Map::cacheRefGetAllVectorMapPoint() {
    if ( cache_mbMP && cache_mtMP.try_tock() ) {
        boost::mutex::scoped_lock lock(mMutexMP);
        cache_mvpMapPoint = std::vector<shared_ptr<MapPoint>>( mspMapPoint.begin(), mspMapPoint.end() );
        cache_mbMP = false;
    }
    return cache_mvpMapPoint;
}

std::set<shared_ptr<KeyFrameState>>& Map::cacheRefGetAllSetKeyFrame() {
    if ( cache_mbMP && cache_mtMP.try_tock() ) {
        boost::mutex::scoped_lock lock(mMutexKF);
        cache_mspKeyFrame = mspKeyFrame;
        cache_mbKF = false;
    }
    return cache_mspKeyFrame;
}

std::vector<shared_ptr<KeyFrameState>>& Map::cacheRefGetAllVectorKeyFrame() {
    if ( cache_mbMP && cache_mtMP.try_tock() ) {
        boost::mutex::scoped_lock lock(mMutexKF);
        cache_mvpKeyFrame = std::vector<shared_ptr<KeyFrameState>>( mspKeyFrame.begin(), mspKeyFrame.end() );
        cache_mbKF = false;
    }
    return cache_mvpKeyFrame;
}

std::set<shared_ptr<MapPoint>> &Map::refGetAllSetMapPoint() {
    return mspMapPoint;
}
