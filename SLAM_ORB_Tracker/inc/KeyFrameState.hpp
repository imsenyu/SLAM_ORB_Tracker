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
#include "BowVector.h"
#include "FeatureVector.h"

class MapPoint;
class FrameState;

class KeyFrameState {
private:

    Vocabulary* mpVocabulary;

    std::vector<shared_ptr<MapPoint>> mvpMapPoint;

    void updatePose(cv::Mat _mT);

public:
    KeyFrameState(shared_ptr<FrameState> _pFrame, Vocabulary *_pVocabulary);

    DBoW2::BowVector mvBow;
    DBoW2::FeatureVector mvFeature;
    shared_ptr<FrameState> mpFrame;

    cv::Mat mT2w;
    cv::Mat mMatR;
    cv::Mat mMatT;
    cv::Mat mO2w;

    void getBoW();
    void insertMapPoint(shared_ptr<MapPoint> pMp, int nP) {
        mvpMapPoint[nP] = pMp;
    }

};

#endif /* KeyFrameState_hpp */
