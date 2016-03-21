//
//  MotionState.hpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/21/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef MotionState_hpp
#define MotionState_hpp

#include "stdafx.hpp"

class MotionState {
private:
    
public:
    MotionState();
    MotionState(const MotionState& _MS);
    MotionState(const MotionState* _pMS);
    
    std::vector<int> mvIds;
    cv::Mat mMatR, mMatT; /** \var 旋转和位移矩阵 */
};

#endif /* MotionState_hpp */
