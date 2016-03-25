//
//  KeyFrameState.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "KeyFrameState.hpp"

KeyFrameState::KeyFrameState(shared_ptr<FrameState> _pFrame, Vocabulary *_pVocabulary):
    mpFrame( _pFrame ), mpVocabulary(_pVocabulary){

    updatePose(_pFrame->mT2w);

    int nKP =_pFrame->mvKeyPoint.size();
    mvpMapPoint = std::vector<shared_ptr<MapPoint>>( nKP, shared_ptr<MapPoint>(NULL) );

}

void KeyFrameState::getBoW() {
    if(mvBow.empty() || mvFeature.empty())
    {
        std::vector<cv::Mat> vDesc;
        cv::Mat& mDesc = mpFrame->mDescriptor;
        int N = mDesc.rows;
        for(int i=0;i<N;i++) {
            vDesc.push_back( mDesc.row(i) );
        }

        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpVocabulary->transform(vDesc,mvBow,mvFeature,4);
    }

}

void KeyFrameState::updatePose(cv::Mat _mT) {
    mT2w = _mT.clone();

    mMatR = mT2w.rowRange(0,3).colRange(0,3);
    mMatT = mT2w.rowRange(0,3).col(3);

    mO2w = - mMatR.t() * mMatT;
}
