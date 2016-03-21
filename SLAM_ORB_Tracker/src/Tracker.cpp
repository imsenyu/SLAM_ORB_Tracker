//
//  Tracker.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "Tracker.hpp"

Tracker::Tracker(InputBuffer* _pIB, FrameDrawer* _pFD, MapDrawer* _pMD):
    mpInputBuffer(_pIB), mpFrameDrawer(_pFD), mpMapDrawer(_pMD),
    meMode(Tracker::WorkMode::InitStep0), meLastMode(Tracker::WorkMode::Start) {
    
}

Tracker::~Tracker() {
    
}

int Tracker::run() {
    return threadRun();
}

int Tracker::threadRun() {
    while ( true ) {
        meLastMode = meMode;

        // this get method is sync.
        mpCurFrame = mpInputBuffer->get();
        mpCurFrame->extract();
        
        
        
        if ( meMode == WorkMode::InitStep0 ) {
            // init first keyFrame
            // init first camera pose
            initPose();
            meMode = WorkMode::Normal;
        }
        else if ( meMode == WorkMode::InitStep1 ) {
            // obtain second keyframe to build
            
            MotionState motion;
            motion.mvIds[0] = mpPreFrame->mId;
            motion.mvIds[1] = mpCurFrame->mId;
            motion.mMatR = Const::mat33_111.clone();
            motion.mMatT = Const::mat31_001.clone();
            
            mCurPose = mCurPose.move(motion);
            meMode = WorkMode::Normal;
        }
        // inited
        else {
            bool bStatus = true;
            
            // 1. get camera pose
            if ( meMode == WorkMode::Normal ) {
                // tracking
                MotionState motion;
                motion.mvIds[0] = mpPreFrame->mId;
                motion.mvIds[1] = mpCurFrame->mId;
                motion.mMatR = Const::mat33_111.clone();
                motion.mMatT = Const::mat31_001.clone();
                
                mCurPose = mCurPose.move(motion);
                
            }
            else {
                // relocalise
            }
            
            // 2. build map track and map point
            
            
            // 3. insert keyframe
            
            
            // 4. if any failed
            if ( bStatus == false ) {
                meMode = WorkMode::Fail;
            }
            
        }
        
        
        
        updateDrawer();
        
        // clone current to previous
        mpPreFrame = shared_ptr<FrameState>( mpCurFrame );
        //meLastMode = meMode;
    }
}


void Tracker::updateDrawer() {
    mpFrameDrawer->update(mpCurFrame);
    mpMapDrawer->update(mCurPose);
}

void Tracker::initPose() {
    mCurPose = PoseState(mpCurFrame->mId);
    mCurPose.mDir = Const::pnt3d_001;
    mCurPose.mPos = Const::pnt3d_000;
    mCurPose.mDir3 = Const::mat33_111.clone();
}
