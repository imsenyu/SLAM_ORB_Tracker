//
//  Tracker.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "Tracker.hpp"

Tracker::Tracker(InputBuffer* _pIB):
mpInputBuffer(_pIB) {
    
}

Tracker::~Tracker() {
    
}

int Tracker::run() {
    return threadRun();
}

int Tracker::threadRun() {
    while ( true ) {
        
        mpCurFrame = mpInputBuffer->get();
        mpCurFrame->extract();
        mpCurFrame->drawKeyPoint();
        
        
        
        mBuffer.put(mpCurFrame);
        
        mpPreFrame = mpCurFrame;
    }
}

//void Tracker::put(shared_ptr<FrameState> _pFS) {
//    mBuffer.put(_pFS);
//}
//
//shared_ptr<FrameState> Tracker::get() {
//    return mBuffer.get();
//}
//
bool Tracker::hasNext() {
    return mBuffer.hasNext();
}

void Tracker::showFrame() {
    shared_ptr<FrameState> pFS = mBuffer.get();
    cv::imshow("Feature", pFS->mImagePointed);
    cv::waitKey(10);
}
