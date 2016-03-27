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

class LocalMapper {
private:


public:

    std::queue<shared_ptr<KeyFrameState>> mvpKeyFrame;

    LocalMapper();

    // insert to queue
    void addKeyFrame(shared_ptr<KeyFrameState> pKeyFrameState);

    // pop one and use this KeyFrame and its neighbour to create MapPoint
    void createNewMapPoint();
};

#endif /* LocalMapper_hpp */
