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
//DBoW2
#include "third/DBoW2/DBoW2/BowVector.h"
#include "third/DBoW2/DBoW2/FeatureVector.h"

class MapPoint;
class FrameState;
class KeyFrameState;

namespace std {
    template <>
    struct less<shared_ptr<KeyFrameState>> {
        bool operator () (const shared_ptr<KeyFrameState> &x, const shared_ptr<KeyFrameState> &y) const;    };
}

class KeyFrameState: public std::enable_shared_from_this<KeyFrameState> {
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


    int mId;
    int mnBAFixedForKF;
    int mnBALocalForKF;
    int mnFuseTargetForKF;
    int mnTrackReferenceForFrame;
    bool isBad() {return mbBad;}

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
    cv::KeyPoint GetKeyPoint(int i);
    shared_ptr<MapPoint> GetMapPoint(const size_t &idx);

    std::vector<shared_ptr<MapPoint>> GetMapPointMatch();

    void EraseConnection(shared_ptr<KeyFrameState> pKF);
    void EraseMapPointMatch(shared_ptr<MapPoint> pMP);
    void EraseMapPointMatch(const size_t &idx);

    void ReplaceMapPointMatch(const size_t &idx, shared_ptr<MapPoint> pMP);
    std::vector<shared_ptr<KeyFrameState>> GetVectorCovisibleKeyFrames()
    {
        return mvpOrderedConnectedKeyFrames;
    }
    cv::Mat getMatT2w();
    cv::Mat getMatR2w();
    cv::Mat getMatt2w();
    cv::Mat getMatO2w();


    bool mbBad;
    bool mbNotErase;
    bool mbToBeErased;
    void SetBadFlag();

    bool isPainted();
    void setPainted(bool _b = true);

};



#endif /* KeyFrameState_hpp */
