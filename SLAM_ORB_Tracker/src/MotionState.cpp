//
//  MotionState.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/21/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "MotionState.hpp"

MotionState::MotionState():
    mvIds(2, 0) {
    
}

MotionState::MotionState(const MotionState& _MS) {
    mvIds = _MS.mvIds;
    mMatR = _MS.mMatR.clone();
    mMatT = _MS.mMatT.clone();
}

MotionState::MotionState(const MotionState* _pMS) {
    mvIds = _pMS->mvIds;
    mMatR = _pMS->mMatR.clone();
    mMatT = _pMS->mMatT.clone();
}