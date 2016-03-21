//
//  PoseState.hpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/21/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef PoseState_hpp
#define PoseState_hpp

#include "stdafx.hpp"
#include "MotionState.hpp"

class PoseState {
private:
    
public:
    int mId; /** \var 当前帧编号 */
    cv::Point3d mPos;/** \var 记录当时位置点 */
    cv::Point3d mDir;/** \var 记录当时镜头方向 */
    cv::Mat mDir3; /** 从eye(3,3)旋转到现在的镜头方向 */
    
    PoseState(int _id = -1);
    PoseState(const PoseState& _PS);
    PoseState(const PoseState* _pPS);
    PoseState move(MotionState& _MS);
    
    friend std::ostream& operator<<(std::ostream& out, const PoseState& ps);
};

#endif /* PoseState_hpp */
