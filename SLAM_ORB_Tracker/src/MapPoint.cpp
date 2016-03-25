//
// Created by Sen Yu on 3/25/16.
// Copyright (c) 2016 Sen Yu. All rights reserved.
//

#include "MapPoint.hpp"

MapPoint::MapPoint(cv::Mat _matMapPointPos, shared_ptr<KeyFrameState> _pKFS, int nP):
    mpKeyFrameFirst(_pKFS) {
    mPos = _matMapPointPos.clone();
    mIdFromKeyFrame = nP;
}

int MapPoint::getUID() {
    return 10000*(mpKeyFrameFirst->mpFrame->mId) + mIdFromKeyFrame + 1;
}