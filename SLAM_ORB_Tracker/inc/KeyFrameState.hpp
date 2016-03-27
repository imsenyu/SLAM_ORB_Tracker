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

class KeyFrameState {
private:

    Vocabulary* mpVocabulary;





public:
    KeyFrameState(shared_ptr<FrameState> _pFrame, Vocabulary *_pVocabulary);

    DBoW2::BowVector mvBow;
    DBoW2::FeatureVector mvFeature;
    shared_ptr<FrameState> mpFrame;
    std::vector<shared_ptr<MapPoint>> mvpMapPoint;

    cv::Mat mT2w;
    cv::Mat mMatR;
    cv::Mat mMatT;
    cv::Mat mO2w;

    void getBoW();
    void insertMapPoint(shared_ptr<MapPoint> pMp, int nP) {
        mvpMapPoint[nP] = pMp;
    }
    void updatePose(cv::Mat _mT);
    float ComputeSceneMedianDepth(int r);
};

#endif /* KeyFrameState_hpp */
