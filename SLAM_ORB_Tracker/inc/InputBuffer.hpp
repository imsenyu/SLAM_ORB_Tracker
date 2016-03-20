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

class InputBuffer {
private:
    std::string mLoadFormat;
    int mBeginIdx, mEndIdx;
    
    void threadRun();
    
public:
    InputBuffer(std::string _loadFormat, int _beginIdx, int _endIdx):
        mLoadFormat(_loadFormat), mBeginIdx(_beginIdx), mEndIdx(_endIdx) {}

    
    void run();
};

#endif /* InputBuffer_hpp */
