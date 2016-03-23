//
//  MapDrawer.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/21/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//


#include "MapDrawer.hpp"

MapDrawer::MapDrawer() : inited(false), mpVizWin(NULL) {
    initCanvas();
    initViz();
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


        drawViz();
        if ( mpVizWin != NULL )
            mpVizWin->spinOnce(10, true);



        //更新 gPose到新的坐标
        mPrePose = mCurPose;
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


}

void MapDrawer::initViz() {

    if ( mpVizWin != NULL )
        delete mpVizWin;
    mpVizWin = new cv::viz::Viz3d("Map Visualizer");
    mpVizWin->showWidget("Coord", cv::viz::WCoordinateSystem());

    // R G B x,y,z
    // campos   相机观察点的位置
    // cam focal point   ,相机焦点位置, 观察方向从相机观察点朝向相机焦点
    // cam_y_dir   ,相机的y轴的朝向,      默认 位于  \downarrow  是y轴,  \rightarrow 是x轴, \inside 是z轴
    double cameraHeight = 50.0f;
    cv::Point3d cam_y_dir(0.0f,0.0f,-1.0f);
    cv::Point3d cam_pos(0.0f, -cameraHeight, 0.0f);
    cv::Point3d cam_focal_point = cam_pos + cv::Point3d(0.0f, cameraHeight*0.1f, 0.0f);

    mCamPose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);

    mpVizWin->setViewerPose(mCamPose);
    mpVizWin->setBackgroundColor(cv::viz::Color::white());
}

void MapDrawer::drawViz() {
    cv::viz::WPlane curPlane(mCurPose.mPos, mCurPose.mDir, cv::Point3d(0,0,-1.0f), cv::Size2d(2.0f, 1.0f), cv::viz::Color::blue());
    cv::viz::WArrow curArrow(mCurPose.mPos, mCurPose.mPos+mCurPose.mDir*0.5f, 0.1f, cv::viz::Color::red());
    mpVizWin->showWidget(cv::format("id-%d",mCurPose.mId), curPlane);
    mpVizWin->showWidget(cv::format("id2-%d",mCurPose.mId), curArrow);

    double cameraHeight = 50.0f;
    cv::Point3d cam_y_dir(0.0f,0.0f,-1.0f);
    cv::Point3d cam_pos = mCurPose.mPos + cv::Point3d(0.0f, -cameraHeight, 0.0f);
    cv::Point3d cam_focal_point = cam_pos + cv::Point3d(0.0f, cameraHeight*0.1f, 0.0f);
    mCamPose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);

    mpVizWin->setViewerPose(mCamPose);
}
