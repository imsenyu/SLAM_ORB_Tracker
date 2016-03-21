//
//  MapDrawer.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/21/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "MapDrawer.hpp"

MapDrawer::MapDrawer():inited(false) {
    initCanvas();
}

void MapDrawer::update(PoseState& _poseState) {
    
    mBuffer.put(_poseState);
    
}

void MapDrawer::show() {
    
    get();
    if ( false == inited ) {
        std::cout<< mCurPose << std::endl;
        mPrePose = mCurPose;
        inited = true;
    }
    else {
        std::cout<< mCurPose << std::endl;
        drawCanvas();
        
        cv::imshow("Map", mPathCanvasWithDir);
        cv::waitKey(10);
        
    }
    
}

void MapDrawer::get() {
    mCurPose = mBuffer.get();
}

void MapDrawer::initCanvas() {
    

    // 画布相关定义,行车路径(x,-z)轴
    mPathCanvas.create(cv::Size(800, 800), CV_8UC3);
    mPathCanvas = cv::Scalar(255, 255, 255);
    

    //设定画布绘制偏移
    mDrawBase = cv::Point2f(400, 500);
    
}


void MapDrawer::drawCanvas(){
    
    //画连线 gPose->curPose
    cv::line(mPathCanvas,
             mDrawBase + Config::dDrawFrameStep*cv::Point2f(mPrePose.mPos.x, -mPrePose.mPos.z),
             mDrawBase + Config::dDrawFrameStep*cv::Point2f(mCurPose.mPos.x, -mCurPose.mPos.z),
             cv::Scalar(-1));
    
    
    //画新点
    cv::circle(mPathCanvas, mDrawBase + Config::dDrawFrameStep* cv::Point2f(mCurPose.mPos.x, -mCurPose.mPos.z), 1, cv::Scalar(-1));
    
    
    //克隆一下 matCanvas 画一下方向
    mPathCanvasWithDir = mPathCanvas.clone();
    cv::line(mPathCanvasWithDir,
             mDrawBase + Config::dDrawFrameStep*cv::Point2f(mCurPose.mPos.x, -mCurPose.mPos.z),
             mDrawBase + Config::dDrawFrameStep*cv::Point2f(mCurPose.mPos.x, -mCurPose.mPos.z) + 10.0f*Config::dDrawFrameStep * cv::Point2f(mCurPose.mDir.x, -mCurPose.mDir.z),
             cv::Scalar(255, 0, 0));

    
    //更新 gPose到新的坐标
    mPrePose = mCurPose;
}
