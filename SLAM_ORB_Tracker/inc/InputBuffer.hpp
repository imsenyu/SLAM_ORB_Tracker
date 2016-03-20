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
#include "FrameBuffer.hpp"
#include "ConcurrentQueue.hpp"

class InputBuffer {
private:
    std::string mLoadFormat;
    std::string mWindowName;
    int mBeginIdx;
    int mEndIdx;
    int mCurIdx;
    int threadRun();
    
    ConcurrentQueue<shared_ptr<FrameBuffer> > mBuffer;
    void put(shared_ptr<FrameBuffer> ptr);

public:
    InputBuffer(std::string _loadFormat, int _beginIdx, int _endIdx);
    int run();
    
    shared_ptr<FrameBuffer> get();
    
};

#endif /* InputBuffer_hpp */
