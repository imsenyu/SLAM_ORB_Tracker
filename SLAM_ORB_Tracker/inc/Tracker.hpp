//
//  Tracker.hpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef Tracker_hpp
#define Tracker_hpp

#include "stdafx.hpp"
#include "FrameState.hpp"
#include "InputBuffer.hpp"
#include "Vocabulary.hpp"

#include "PoseState.hpp"
#include "MotionState.hpp"
#include "KeyFrameState.hpp"
#include "Map.hpp"
#include "MapPoint.hpp"
#include "Optimizer.hpp"
#include "LocalMapper.hpp"
//ORB
#include "Initializer.h"
#include "ORBmatcher.h"
#include "ORBextractor.h"

class MapDrawer;
class FrameDrawer;
class KeyFrameState;

class Tracker {
private:
    
    typedef enum {
        Start,
        Fail,
        InitStep0,
        InitStep1,
        Normal,
    } WorkMode;
    
    typedef enum {
        Good,
        Bad
    } TrackResult;
    
    WorkMode meMode;
    WorkMode meLastMode;


private:
    int threadRun();
    
    InputBuffer* mpInputBuffer;
    shared_ptr<FrameState> mpPreFrame;
    shared_ptr<FrameState> mpCurFrame;
    shared_ptr<FrameState> mpIniFrame;
    shared_ptr<FrameState> mpRefFrame;
    
    PoseState mCurPose;
    void initPose();
    
    
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    void updateDrawer();
    
    std::vector<int> mvMatchPair12;
    std::vector<uchar> mvMatchMask12;
    std::vector<cv::Point2f> mvMatchPoint[2];
    std::vector<cv::Point2f> mvIniMatched;
    int match(shared_ptr<FrameState> pPreFrame, shared_ptr<FrameState> pCurFrame, std::vector<int>& vMatchPair12, std::vector<uchar>& vMatchMask12, std::vector<cv::Point2f> vMatchPoint[2]);
    int filterByOpticalFlow(shared_ptr<FrameState> pPreFrame, shared_ptr<FrameState> pCurFrame, std::vector<int>& vMatchPair12, std::vector<uchar>& vMatchMask12, std::vector<cv::Point2f> vMatchPoint[2]);
    //bool computeMotion(shared_ptr<FrameState> pPreFrame, shared_ptr<FrameState> pCurFrame, std::vector<cv::Point2f> *vMatchPoint, MotionState &motion, std::vector<uchar> &vMatchMask);
    //void triangulateDLT(cv::Point2f &xl,cv::Point2f &xr,cv::Mat &Pl,cv::Mat &Pr, cv::Mat &point3d);
    //int checkRT(cv::Mat matR, cv::Mat matT, std::vector<cv::Point2f> *vMatchPoint, std::vector<uchar> &vMatchMask);
    //void drawFilter(shared_ptr<FrameState> pFrame, std::vector<cv::Point2f>& mvPoint);

    ORB_SLAM::Initializer* mpIniter;
    Vocabulary* mpVocabulary;
    Map* mpMap;
    void initStepFirstKeyFrame();
    MotionState mMotion;
    LocalMapper* mpLocalMapper;
    bool initStepSecondKeyFrame();
    bool initStepBuildMap(MotionState initMotion, vector<cv::Point3f> &vP3D);
    bool TrackFromPreFrame();
    bool UpdateLocal();

    int mnMatchesInliers;

    shared_ptr<KeyFrameState> mpReferenceKF;
    bool TrackLocalMap();
    void UpdateReferenceKeyFrames();
    void UpdateReferencePoints();
    void SearchReferencePointsInFrustum();

    bool TrackMotion();
    cv::Mat mVelocity;
    void createKeyFrame();
    int mLastMapperId;
    int mLastBundleAdjustmentId;



    boost::mutex mMutexLMP;
    boost::mutex mMutexLKF;
    std::vector<shared_ptr<KeyFrameState>> mvpLocalKeyFrames;
    std::vector<shared_ptr<MapPoint>> mvpLocalMapPoints;

    bool cache_mbLMP;
    Tick cache_mtLMP;
    std::vector<shared_ptr<MapPoint>> cache_mvpLocalMapPoints;

public:
    Tracker(InputBuffer *_pIB, FrameDrawer *_pFD, MapDrawer *_pMD, Vocabulary *_pVocabulary, Map *_pMap, LocalMapper* _pLocalMapper);
    ~Tracker();
    
    int run();



    std::vector<shared_ptr<MapPoint>> getAllVectorLocalMapPoint();
    std::vector<shared_ptr<KeyFrameState>> getAllVectorLocalKeyFrame();

    std::vector<shared_ptr<MapPoint>>& cacheRefGetAllVectorLocalMapPoint();
};

#endif /* Tracker_hpp */
