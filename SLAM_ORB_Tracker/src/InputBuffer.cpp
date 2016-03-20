//
//  InputBuffer.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "InputBuffer.hpp"

InputBuffer::InputBuffer(std::string _loadFormat, int _beginIdx, int _endIdx):
mLoadFormat(_loadFormat), mBeginIdx(_beginIdx), mEndIdx(_endIdx),
mBuffer(100) {
    
    mCurIdx = mBeginIdx;
    mWindowName = "InputBuffer";
    std::cout<< mBuffer.getCapacity() << std::endl;

}

int InputBuffer::threadRun() {
    cv::Mat matImage;
    for(mCurIdx = mBeginIdx; mCurIdx < mEndIdx; mCurIdx+=1 ) {

        shared_ptr<FrameBuffer> ptrFrame(new FrameBuffer(mCurIdx));
        put(ptrFrame);
    }
    return 0;
}
    

int InputBuffer::run() {
   
    return threadRun();
}

void InputBuffer::put(shared_ptr<FrameBuffer> ptr) {
    
    mBuffer.put(ptr);
    //std::cout<< "put " << ptr->mId << std::endl;
}

shared_ptr<FrameBuffer> InputBuffer::get() {
    auto ret = mBuffer.get();
    //std::cout<< "get " << ret->mId << std::endl;
    return ret;
}

