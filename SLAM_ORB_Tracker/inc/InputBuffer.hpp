//
//  InputBuffer.hpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef InputBuffer_hpp
#define InputBuffer_hpp

#include "stdafx.hpp"
#include "FrameState.hpp"
#include "ConcurrentQueue.hpp"


class InputBuffer {
private:
    std::string mLoadFormat;
    std::string mWindowName;
    int mBeginIdx;
    int mEndIdx;
    int mCurIdx;
    int threadRun();
    
    ConcurrentQueue<shared_ptr<FrameState>> mBuffer;
    void put(shared_ptr<FrameState> ptr);

public:
    InputBuffer(std::string _loadFormat, int _beginIdx, int _endIdx);
    int run();
    
    shared_ptr<FrameState> get();
    
};

#endif /* InputBuffer_hpp */
