//
//  Tracker.hpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef Tracker_hpp
#define Tracker_hpp

#include "stdafx.hpp"
#include "InputBuffer.hpp"
#include "ConcurrentQueue.hpp"

class Tracker {
private:
    int threadRun();
    InputBuffer* mpInputBuffer;
    
    shared_ptr<FrameState> mpPreFrame;
    shared_ptr<FrameState> mpCurFrame;
    
    ConcurrentQueue<shared_ptr<FrameState>> mBuffer;
//    void put(shared_ptr<FrameState> _pFS);
//    bool hasNext();
//    shared_ptr<FrameState> get();
    
public:
    Tracker(InputBuffer* _pIB);
    ~Tracker();
    
    int run();
    
    bool hasNext();
    void showFrame();
    
};

#endif /* Tracker_hpp */
