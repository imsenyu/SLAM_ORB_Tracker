//
//  KeyFrameState.hpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef KeyFrameState_hpp
#define KeyFrameState_hpp

#include "stdafx.hpp"

#include "FrameState.hpp"
#include "Vocabulary.hpp"
#include "MapPoint.hpp"

//DBoW2
#include "third/DBoW2/DBoW2/BowVector.h"
#include "third/DBoW2/DBoW2/FeatureVector.h"

class MapPoint;
class FrameState;

class KeyFrameState: public boost::enable_shared_from_this<KeyFrameState> {
private:

    Vocabulary* mpVocabulary;
    std::vector<shared_ptr<KeyFrameState>> mvpOrderedConnectedKeyFrames;
    std::map<shared_ptr<KeyFrameState>,int> mConnectedKeyFrameWeights;
    std::vector<int> mvOrderedWeights;


public:
    KeyFrameState(shared_ptr<FrameState> _pFrame, Vocabulary *_pVocabulary);

    DBoW2::BowVector mvBow;
    DBoW2::FeatureVector mvFeature;
    shared_ptr<FrameState> mpFrame;
    //std::vector<shared_ptr<MapPoint>> mvpMapPoint;

    cv::Mat mT2w;
    cv::Mat mMatR;
    cv::Mat mMatT;
    cv::Mat mO2w;
    int mId;

    int mnFuseTargetForKF;


    void getBoW();
    void insertMapPoint(shared_ptr<MapPoint> pMp, int nP);
    void updatePose(cv::Mat _mT);
    float ComputeSceneMedianDepth(int r);
    std::vector<shared_ptr<KeyFrameState>> GetBestCovisibilityKeyFrames(const int &N);
    void AddConnection(shared_ptr<KeyFrameState> pKF, const int &weight);

    void UpdateBestCovisibles();

    void UpdateConnections();

    bool IsInImage(const float &x, const float &y);
    std::vector<size_t> getFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel = -1, const int maxLevel = -1);

    int GetKeyPointScaleLevel(const size_t &idx) const;

    cv::Mat GetDescriptor(const size_t &idx);

    shared_ptr<MapPoint> GetMapPoint(const size_t &idx);

    std::vector<shared_ptr<MapPoint>> GetMapPointMatch();

    void EraseMapPointMatch(shared_ptr<MapPoint> pMP);
    void EraseMapPointMatch(const size_t &idx);

    void ReplaceMapPointMatch(const size_t &idx, shared_ptr<MapPoint> pMP);
};



#endif /* KeyFrameState_hpp */
