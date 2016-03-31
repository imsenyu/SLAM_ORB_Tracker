//
// Created by Sen Yu on 3/25/16.
// Copyright (c) 2016 Sen Yu. All rights reserved.
//

#ifndef SLAM_ORB_TRACKER_MAPPOINT_HPP
#define SLAM_ORB_TRACKER_MAPPOINT_HPP

#include "stdafx.hpp"
#include "KeyFrameState.hpp"
#include "Map.hpp"

class KeyFrameState;
class Map;

class MapPoint: public boost::enable_shared_from_this<MapPoint> {
private:


    bool mbBad;

public:
    std::map<shared_ptr<KeyFrameState>, int> msKeyFrame2FeatureId;
    shared_ptr<KeyFrameState> mpRefKF;
    int mIdFromKeyFrame;
    cv::Mat mDescriptor;
    Map* mpMap;

    MapPoint(cv::Mat _matMapPointPos, shared_ptr<KeyFrameState> _pKFS, int nP, Map *_pMap);
    void setKeyFrame(shared_ptr<KeyFrameState> _pKFS, int nP) {
        msKeyFrame2FeatureId[_pKFS] = nP;
    }
    int getUID();
    cv::Mat mPos;

    int mnFuseCandidateForKF;
    int mnBALocalForKF;
    bool isBad() {
        return mbBad;
    }

    void UpdateNormalAndDepth();
    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    void ComputeDistinctiveDescriptors();
    cv::Mat GetNormal()
    {
        return mNormalVector.clone();
    }
    cv::Mat GetDescriptor()
    {
        return mDescriptor.clone();
    }
    float mfMinDistance;
    float mfMaxDistance;
    cv::Mat mNormalVector;
    void Replace(shared_ptr<MapPoint> pMP);
    void SetBadFlag();
};


#endif //SLAM_ORB_TRACKER_MAPPOINT_HPP
