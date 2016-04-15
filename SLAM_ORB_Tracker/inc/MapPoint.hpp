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
    static int _counterId;
    int mId;
    bool mbBad;

    boost::mutex mMutexMPos;
    cv::Mat mPos;

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


    void lockMutexMPos(bool toLock = 0);
    cv::Mat getMPos() ;
    cv::Mat& getMPosRef() ;
    void setMPos( cv::Mat mat);

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
    void EraseObservation(shared_ptr<KeyFrameState> pKF);
    int GetIndexInKeyFrame(shared_ptr<KeyFrameState> pKF)
    {
        if(msKeyFrame2FeatureId.count(pKF))
            return msKeyFrame2FeatureId[pKF];
        else
            return -1;
    }
    int mnVisible;
    int mnFound;
    void IncreaseVisible()
    {
        mnVisible++;
    }

    void IncreaseFound()
    {
        mnFound++;
    }

    float GetFoundRatio()
    {
        return ((float)(mnFound))/mnVisible;
    }
    int mnLastFrameSeen;
    bool mbTrackInView;
    float mTrackProjX;
    float mTrackProjY;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    int mnTrackReferenceForFrame;

    boost::mutex mMutexPainted;
    bool mbPainted;
    void setPainted(bool _b = true) {
        boost::mutex::scoped_lock lock(mMutexPainted);
        mbPainted = _b;
    }
    bool isPainted() {
        boost::mutex::scoped_lock lock(mMutexPainted);
        return mbPainted;
    }
};


#endif //SLAM_ORB_TRACKER_MAPPOINT_HPP
