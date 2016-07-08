//
//  MapDrawer.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/21/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//


#include "MapDrawer.hpp"
#include "Tracker.hpp"
#include "Map.hpp"
#include "GLWindow.hpp"
#include "FrameState.hpp"


MapDrawer::MapDrawer(Map *_pMap) :
    inited(false),
    mpVizWin(NULL),
    mpMap(_pMap),
    mpTracker(NULL),
    mpGLWin(NULL),

    cache_mbFrame(false),
    cache_mtFrame(5)
{
    initCanvas();
    //initViz();
    std::string outputFilePath =
        cv::format("%s/%s_%s.txt",
            Config::sPathOutput.c_str(),
            Config::sProjectName.c_str(),
            Config::tLaunchTime.c_str(),
            ".txt");
    fsout = std::ofstream(outputFilePath);
    //cv::waitKey(10);
}

void MapDrawer::setTracker(Tracker* _pTracker) {
    mpTracker = _pTracker;
}

void MapDrawer::update(shared_ptr<FrameState> pFS) {
    
    mBuffer.put(pFS);
    
}

void MapDrawer::show() {

//    if ( mBuffer.hasNext() ) {
//        take();
//        if (false == inited) {
//            std::cout << "firstshow"<<mCurPose << std::endl;
//            mPrePose = mCurPose;
//            inited = true;
//        }
//        else {
//            //std::cout << "show" <<mCurPose << std::endl;
//            drawCanvas();
//            drawViz();
//            cv::imshow("Map", mPathCanvasWithDir);
//
//            //更新 gPose到新的坐标
//            mPrePose = mCurPose;
//        }
//    }


    {
        boost::mutex::scoped_lock lock(mMutexPathCanvasWithDir);
        cv::imshow("Map", mPathCanvasWithDir);

        if ( mpCurFrame && mpCurFrame->mId % 50 == 0 ) {
            cv::imwrite(cv::format("%s/%s_%s_%04d.jpg", Config::sPathOutput.c_str(), Config::sProjectName.c_str(), Config::tLaunchTime.c_str(), mpCurFrame->mId ), mPathCanvasWithDir);
        }
    }
//    if ( mpVizWin != NULL )  {
//        boost::mutex::scoped_lock lockUI(mMutexGUI);
//        boost::mutex::scoped_lock lock(mMutexVizWin);
//        //mpVizWin->spinOnce(10, true);
//    }

    
}

void MapDrawer::take() {
    mpCurFrame = mBuffer.take();
    {
        boost::mutex::scoped_lock lock(mMutexFrame);
        mspFrame.insert( mpCurFrame );
        cache_mbFrame = true;
    }
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

    //fsout << cv::format("%d\t%f\t%f\t%f",mpCurFrame->mId,mCurPose.mPos.x,mCurPose.mPos.y,mCurPose.mPos.z) << std::endl;
    for(int i=0;i<3;i++) {
        for(int j=0;j<3;j++) {
            fsout << R.at<float>(i,j) << " ";
        }

        fsout << ( t.at<float>(i)  );
        fsout << (i==2 ? "\n" : " ");
    }

    fsout.flush();
}

void MapDrawer::initCanvas() {
    

    // 画布相关定义,行车路径(x,-z)轴
    mPathCanvas.create(cv::Size(800, 800), CV_8UC3);
    mPathCanvas = cv::Scalar(255, 255, 255);
    mPathCanvasWithDir = mPathCanvas.clone();

    //设定画布绘制偏移
    mDrawBase = cv::Point2f(400, 600);
    cv::namedWindow("Map", CV_GUI_NORMAL | CV_WINDOW_AUTOSIZE );
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
    cv::Mat _PathCanvasWithDir = mPathCanvas.clone();

    cv::line(_PathCanvasWithDir,
             mDrawBase + Config::dDrawFrameStep*cv::Point2f(mCurPose.mPos.x, -mCurPose.mPos.z),
             mDrawBase + Config::dDrawFrameStep*cv::Point2f(mCurPose.mPos.x, -mCurPose.mPos.z) + 1.0f*Config::dDrawFrameStep * cv::Point2f(mCurPose.mDir.x, -mCurPose.mDir.z),
             cv::Scalar(255, 0, 0));

    Config::time("SHOW2");
    std::set<shared_ptr<KeyFrameState>> spKF = mpMap->getAllSetKeyFrame();
    for(auto iter=spKF.begin();iter!=spKF.end();iter++) {
        shared_ptr<KeyFrameState> pKF = *iter;
        PoseState pose;

        cv::Mat pop = pKF->getMatT2w();
        cv::Mat R = pop.rowRange(0,3).colRange(0,3);
        cv::Mat t = pop.rowRange(0,3).col(3);

        t = -R.inv() * t;
        R = R.inv();
        pose.mPos = Utils::convertToPoint3d(t);
        pose.mDir = Utils::convertToPoint3d(R * Const::mat31_001);
        cv::circle(_PathCanvasWithDir, mDrawBase + Config::dDrawFrameStep* cv::Point2f(pose.mPos.x, -pose.mPos.z), 1, cv::Scalar(0,0,255));
    }

    {
        boost::mutex::scoped_lock lock(mMutexPathCanvasWithDir);
        mPathCanvasWithDir = _PathCanvasWithDir.clone();
    }

    Config::timeEnd("SHOW2");

}

//void MapDrawer::initViz() {
//
//    if ( mpVizWin != NULL )
//        delete mpVizWin;
//    mpVizWin = new cv::viz::Viz3d("Map Visualizer");
//    mpVizWin->showWidget("Coord", cv::viz::WCoordinateSystem());
//
//    // R G B x,y,z
//    // campos   相机观察点的位置
//    // cam focal point   ,相机焦点位置, 观察方向从相机观察点朝向相机焦点
//    // cam_y_dir   ,相机的y轴的朝向,      默认 位于  \downarrow  是y轴,  \rightarrow 是x轴, \inside 是z轴
//    double cameraHeight = 50.0f;
//    cv::Point3d cam_y_dir(0.0f,0.0f,-1.0f);
//    cv::Point3d cam_pos(0.0f, -cameraHeight, 0.0f);
//    cv::Point3d cam_focal_point = cam_pos + cv::Point3d(0.0f, cameraHeight*0.1f, 0.0f);
//
//    mCamPose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
//
//    mpVizWin->setViewerPose(mCamPose);
//    mpVizWin->setBackgroundColor(cv::viz::Color::white());
//}

//cv::viz::Color getColor(shared_ptr<FrameState> mpCurFrame) {
//    if ( mpCurFrame->mnTrackedType == 1 ) return cv::viz::Color::blue();
//    else if ( mpCurFrame->mnTrackedType == 2 ) return cv::viz::Color::red();
//    return cv::viz::Color::green();
//}

//int MapDrawer::drawViz() {
//    if ( mpCurFrame->isPainted() == false ) {
//        boost::mutex::scoped_lock lockUI(mMutexGUI);
//        boost::mutex::scoped_lock lock(mMutexVizWin);
//        cv::viz::WPlane curPlane(mCurPose.mPos*20, mCurPose.mDir*20, cv::Point3d(0,1.0f,0), cv::Size2d(2.0f, 1.0f), getColor(mpCurFrame)      );
//        mpVizWin->showWidget(cv::format("id-%d",mCurPose.mId), curPlane);
//    }
//    {
//        boost::mutex::scoped_lock lockUI(mMutexGUI);
//        boost::mutex::scoped_lock lock(mMutexVizWin);
//        cv::viz::WArrow curArrow(mCurPose.mPos * 20, mCurPose.mPos * 20 + mCurPose.mDir * 0.5f, 0.1f, cv::viz::Color::red());
//        mpVizWin->showWidget(cv::format("id2-%d", mCurPose.mId), curArrow);
//    }
//
//    double cameraHeight = 50.0f;
//    cv::Point3d cam_y_dir(0.0f,0.0f,-1.0f);
//    cv::Point3d cam_pos = mCurPose.mPos*20 + cv::Point3d(0.0f, -cameraHeight, -cameraHeight);
//    cv::Point3d cam_focal_point = cam_pos + cv::Point3d(0.0f, cameraHeight*0.00001f, cameraHeight*0.00001f);
//    mCamPose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
//
//
//    int ret = 0;
//    std::set<shared_ptr<KeyFrameState>> spKF = mpMap->getAllSetKeyFrame();
//    {
//        boost::mutex::scoped_lock lockUI(mMutexGUI);
//        boost::mutex::scoped_lock lock(mMutexVizWin);
//        Config::time("SHOW");
//        int numUpdateKeyFrame = 0;
//        for(auto iter=spKF.begin();iter!=spKF.end();iter++) {
//            shared_ptr<KeyFrameState> pKF = *iter;
//            if ( pKF->isPainted() )
//                continue;
//
//            PoseState pose;
//
//            cv::Mat pop = pKF->getMatT2w();
//            cv::Mat R = pop.rowRange(0,3).colRange(0,3);
//            cv::Mat t = pop.rowRange(0,3).col(3);
//
//            t = -R.inv() * t;
//            R = R.inv();
//            pose.mPos = Utils::convertToPoint3d(t);
//            pose.mDir = Utils::convertToPoint3d(R * Const::mat31_001);
//            cv::viz::WPlane curPlane(pose.mPos*20, pose.mDir*20, cv::Point3d(0,1.0f,0), cv::Size2d(2.0f, 1.0f), cv::viz::Color::green()  );
//            std::string name = cv::format("id-%d",pKF->mId);
//
//            mpVizWin->showWidget(name,curPlane);
//
//            pKF->setPainted();
//            numUpdateKeyFrame++;
//        }
//        printf("====mpFrame=== %d updateKeyFrame %d\n", mpCurFrame->mId, numUpdateKeyFrame);
//        Config::timeEnd("SHOW");
//        ret = numUpdateKeyFrame;
//    }
//
//    {
//        boost::mutex::scoped_lock lockUI(mMutexGUI);
//        boost::mutex::scoped_lock lock(mMutexVizWin);
//        mpVizWin->setViewerPose(mCamPose);
//    }
//
//    return ret;
//}

void MapDrawer::threadRun() {
    Tick tMapDrawer(60);
    while(tMapDrawer.tock()){

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

                mpGLWin->setNewCameraPos( mCurPose.mPos.x, mCurPose.mPos.y, mCurPose.mPos.z);
//                int ret = drawViz();
//                if ( ret ) drawMapPoint();


                //更新 gPose到新的坐标
                mPrePose = mCurPose;
            }

        }
    }

//    cv::waitKey(10);
//    if ( mpVizWin != NULL )
//        mpVizWin->spinOnce(10, true);
}
//
//void MapDrawer::drawMapPoint() {
//    std::set<shared_ptr<MapPoint>> spMP = mpMap->getAllSetMapPoint();
//
//    std::vector<shared_ptr<MapPoint>> vpLocalMP = mpTracker->mvpLocalMapPoints;
//    std::set<shared_ptr<MapPoint>> spLocalMP = std::set<shared_ptr<MapPoint>>(vpLocalMP.begin(), vpLocalMP.end());
//    printf("MapPoint_ALL %d\n", spMP.size());
//    printf("MapPoint_Local %d\n", spLocalMP.size());
//    {
//        boost::mutex::scoped_lock lockUI(mMutexGUI);
//        boost::mutex::scoped_lock lock(mMutexVizWin);
//        Config::time("MapPoint_ALL");
//        int numUpdateMapPoint = 0;
//        for(auto iter=spMP.begin();iter!=spMP.end();iter++) {
//            shared_ptr<MapPoint> pMP = *iter;
//            if ( pMP->isPainted() )
//                continue;
//            if ( pMP->isBad() )
//                continue;
//            if ( spLocalMP.count( pMP ) )
//                continue;
//
//            cv::Point3f pos = Utils::convertToPoint3d(pMP->mPos*20);
//
//
//            cv::viz::WSphere curPoint(pos,0.10f, 1, cv::viz::Color(0,0,0));
//            mpVizWin->showWidget( cv::format("mp-%d", pMP->getUID()), curPoint );
//
//            pMP->setPainted();
//            numUpdateMapPoint++;
//        }
//        printf("===Frame %d MapPoint_ALL %d\n", mpCurFrame->mId,numUpdateMapPoint );
//        Config::timeEnd("MapPoint_ALL");
//    }
//    {
//        boost::mutex::scoped_lock lockUI(mMutexGUI);
//        boost::mutex::scoped_lock lock(mMutexVizWin);
//        int numUpdateMapPoint = 0;
//        Config::time("MapPoint_Local");
//        for(auto iter=spLocalMP.begin();iter!=spLocalMP.end();iter++) {
//            shared_ptr<MapPoint> pMP = *iter;
//            if ( pMP->isPainted() )
//                continue;
//            if ( pMP->isBad() )
//                continue;
//
//            cv::Point3f pos = Utils::convertToPoint3d(pMP->mPos*20);
//
//            cv::viz::WSphere curPoint(pos,0.20f, 6, cv::viz::Color(0,0,255));
//            mpVizWin->showWidget( cv::format("mp-%d", pMP->getUID()), curPoint );
//
//            pMP->setPainted();
//            numUpdateMapPoint++;
//        }
//        printf("===Frame %d MapPoint_Local %d\n", mpCurFrame->mId,numUpdateMapPoint );
//        Config::timeEnd("MapPoint_Local");
//    }
//}

void MapDrawer::setGLWindow(GLWindow *_p) {
    mpGLWin = _p;
}

std::set<shared_ptr<FrameState>> MapDrawer::getAllSetFrame() {
    boost::mutex::scoped_lock lock(mMutexFrame);
    return mspFrame;
}

std::vector<shared_ptr<FrameState>> MapDrawer::getAllVectorFrame() {
    boost::mutex::scoped_lock lock(mMutexFrame);
    return std::vector<shared_ptr<FrameState>>( mspFrame.begin(), mspFrame.end() );
}

std::set<shared_ptr<FrameState>> &MapDrawer::cacheRefGetAllSetFrame() {
    if ( cache_mbFrame && cache_mtFrame.try_tock() ) {
        boost::mutex::scoped_lock lock(mMutexFrame);
        cache_mspFrame = mspFrame;
        cache_mbFrame = false;
    }
    return cache_mspFrame;
}
