//
//  PoseState.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/21/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "PoseState.hpp"

PoseState::PoseState(int _id):
    mId(_id){
        
}

PoseState::PoseState(const PoseState& _PS) {
    mId = _PS.mId;
    mPos = _PS.mPos;
    mDir = _PS.mDir;
    mDir3 = _PS.mDir3.clone();
}

PoseState::PoseState(const PoseState* _pPS) {
    mId = _pPS->mId;
    mPos = _pPS->mPos;
    mDir = _pPS->mDir;
    mDir3 = _pPS->mDir3.clone();
}

std::ostream& operator<<(std::ostream& out, const PoseState& ps) {
    out << "PoseState[" << ps.mId << "]" << std::endl;
    out << "Pos: " << ps.mPos << std::endl;
    out << "Dir: " << ps.mDir << std::endl;
    return out;
}

PoseState PoseState::move(MotionState& _motion) {
    std::vector<int> vMotionId = _motion.mvIds;
    //校验motion与当前Pose是否匹配
    if (mId != vMotionId[0]) {
        //throw std::exception(cv::format("当前Pose[%d]与Motion[%d-%d]不匹配", idxImg, motion.getIdxImg(0), motion.getIdxImg(1)).c_str());
    }
    
    PoseState retPose( vMotionId[1] );
    

    //cv::Point3转cv::Mat(3,1)
    cv::Mat matLocalPos = Utils::convertToCvMat31(mPos);
    cv::Mat matLocalDir = Utils::convertToCvMat31(mDir);

    
    cv::Mat matTmpRotate;
    Utils::getRodriguesRotation(matLocalDir, matTmpRotate);
    
    
    //如果不考虑y轴影响,则要重新把 模长置为1
    if (true) {
        cv::Mat tT = _motion.mMatT.clone();
        tT.at<float>(1, 0) = 0.0f;

        if(fabs(tT.at<float>(0, 0))+fabs(tT.at<float>(2, 0))>0.0f) tT *= 1.0f / cv::norm(tT);
        matLocalPos = matLocalPos + mDir3 * tT;// *  1.65f / motion.getScale();
    }
    else {
        matLocalPos = matLocalPos + mDir3 * _motion.mMatT;// *  1.65f / motion.getScale();
    }
    
    
    
    //如果不考虑y轴影响,则要重新把 模长置为1
    if (true) {
        cv::Mat matRC = _motion.mMatR * Const::mat31_001;
        
        matRC.at<float>(1, 0) = 0.0f;
        matRC = matRC / cv::norm(matRC);
        
        Utils::getRodriguesRotation(matRC, matRC);
        
        matLocalDir = matRC * matLocalDir;
        matLocalDir.at<float>(1, 0) = 0.0f;
        matLocalDir = matLocalDir / cv::norm(matLocalDir);
    
        mDir3 = mDir3 * matRC;
        matRC = mDir3 * Const::mat31_001;
        matRC.at<float>(1, 0) = 0.0f;
        Utils::getRodriguesRotation(matRC, matRC);
        mDir3 = matRC.clone();
        
    }
    //考虑Y轴
    else {
        matLocalDir = _motion.mMatR * matLocalDir;
        mDir3 = mDir3 * _motion.mMatR ;
    }

    retPose.mId = vMotionId[1];
    retPose.mDir = Utils::convertToPoint3d(matLocalDir);
    retPose.mPos = Utils::convertToPoint3d(matLocalPos);
    
    retPose.mDir3 = mDir3.clone();
    
    return retPose;
}