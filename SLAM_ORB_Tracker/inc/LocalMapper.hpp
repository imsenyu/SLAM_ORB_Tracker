//
//  LocalMapper.hpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef LocalMapper_hpp
#define LocalMapper_hpp

#include "stdafx.hpp"
#include "KeyFrameState.hpp"
#include "BlockingQueue.hpp"
#include "Map.hpp"

// ORB
#include "ORBmatcher.h"

class LocalMapper {
private:
    BlockingQueue<shared_ptr<KeyFrameState>> mBuffer;
    shared_ptr<KeyFrameState> mpCurKeyFrame;
    Map* mpMap;

    cv::Mat computeF12(shared_ptr<KeyFrameState> pKF1, shared_ptr<KeyFrameState> pKF2);

public:

    LocalMapper(Map *_pMap);

    // insert to queue
    void addKeyFrame(shared_ptr<KeyFrameState> pKeyFrameState);

    // pop one and use this KeyFrame and its neighbour to create MapPoint
    void createNewMapPoint();

    int processKeyFrameLoop();

    int run();

    int triangleNewMapPoint();
};

#endif /* LocalMapper_hpp */
