//
//  MapDrawer.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/21/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//


#include "MapDrawer.hpp"

MapDrawer::MapDrawer(Map *_pMap) : inited(false), mpVizWin(NULL), mpMap(_pMap) {
    initCanvas();
    initViz();
}

void MapDrawer::update(shared_ptr<FrameState> pFS) {
    
    mBuffer.put(pFS);
    
}

void MapDrawer::show() {

    if ( mBuffer.hasNext() ) {
        take();
        if (false == inited) {
            std::cout << "firstshow"<<mCurPose << std::endl;
            mPrePose = mCurPose;
            inited = true;
        }
        else {
            //std::cout << "show" <<mCurPose << std::endl;
            drawCanvas();
            drawViz();
            cv::imshow("Map", mPathCanvasWithDir);

            //更新 gPose到新的坐标
            mPrePose = mCurPose;
        }
    }

    cv::waitKey(10);
    if ( mpVizWin != NULL )
        mpVizWin->spinOnce(10, true);
    
}

void MapDrawer::take() {
    mpCurFrame = mBuffer.take();
    cv::Mat pop = mpCurFrame->mT2w.clone();
    //std::cout<<"show take "<<pop<<std::endl;

    cv::Mat R = pop.rowRange(0,3).colRange(0,3);
    cv::Mat t = pop.rowRange(0,3).col(3);

    t = -R.inv() * t;
    R = R.inv();
    mCurPose.mId = mpCurFrame->mId;
    mCurPose.mPos = Utils::convertToPoint3d(t);
    mCurPose.mDir3 = R;
    mCurPose.mDir = Utils::convertToPoint3d(R * Const::mat31_001);

    //std::cout<<"------------draw---------"<<std::endl<<mCurPose.mPos<<mCurPose.mDir<<std::endl;
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
             mDrawBase + Config::dDrawFrameStep*cv::Point2f(mCurPose.mPos.x, -mCurPose.mPos.z) + 1.0f*Config::dDrawFrameStep * cv::Point2f(mCurPose.mDir.x, -mCurPose.mDir.z),
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

cv::viz::Color getColor(shared_ptr<FrameState> mpCurFrame) {
    if ( mpCurFrame->mnTrackedType == 1 ) return cv::viz::Color::blue();
    else if ( mpCurFrame->mnTrackedType == 2 ) return cv::viz::Color::red();
    return cv::viz::Color::green();
}

void MapDrawer::drawViz() {
    cv::viz::WPlane curPlane(mCurPose.mPos*20, mCurPose.mDir*20, cv::Point3d(0,1.0f,0), cv::Size2d(2.0f, 1.0f), getColor(mpCurFrame)      );
    cv::viz::WArrow curArrow(mCurPose.mPos*20, mCurPose.mPos*20+mCurPose.mDir*0.5f, 0.1f, cv::viz::Color::red());
    mpVizWin->showWidget(cv::format("id-%d",mCurPose.mId), curPlane);
    mpVizWin->showWidget(cv::format("id2-%d",mCurPose.mId), curArrow);

    double cameraHeight = 50.0f;
    cv::Point3d cam_y_dir(0.0f,0.0f,-1.0f);
    cv::Point3d cam_pos = mCurPose.mPos*20 + cv::Point3d(0.0f, -cameraHeight, 0.0f);
    cv::Point3d cam_focal_point = cam_pos + cv::Point3d(0.0f, cameraHeight*0.1f, 0.0f);
    mCamPose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);

    mpVizWin->setViewerPose(mCamPose);

//    std::set<shared_ptr<MapPoint>> spMapPoint = mpMap->mspMapPoint;
//    auto iter = spMapPoint.begin();
//    for(; iter!=spMapPoint.end(); iter++) {
//        shared_ptr<MapPoint> pMP = *iter;
//        cv::Point3f pos = Utils::convertToPoint3d(pMP->mPos*20);
//
//        // -2.0  ->  0.5;
//        float radio = (pos.y - (-2.0f))/2.5f;
//        radio = radio > 1.0f ? 1.0f : radio;
//        radio = radio < 0.0f ? 0.0f : radio;
//        cv::viz::WSphere curPoint(pos,0.10f, 10, cv::viz::Color(255*radio,0,255*(1.0f-radio)));
//        mpVizWin->showWidget( cv::format("mp-%d", pMP->getUID()), curPoint );
//    }
}
