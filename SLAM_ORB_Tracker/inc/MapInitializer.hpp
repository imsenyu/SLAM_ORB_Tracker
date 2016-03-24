//
// Created by Sen Yu on 3/22/16.
// Copyright (c) 2016 Sen Yu. All rights reserved.
//

#ifndef SLAM_ORB_TRACKER_MAPINITIALIZER_HPP
#define SLAM_ORB_TRACKER_MAPINITIALIZER_HPP

#include "stdafx.hpp"
#include "FrameState.hpp"
#include "KeyFrameState.hpp"

class MapInitializer {
private:
    shared_ptr<FrameState> mpInitFrame;
    shared_ptr<FrameState> mpSecondFrame;

public:
    MapInitializer(shared_ptr<FrameState> _pFS);
    bool init(shared_ptr<FrameState> _pFS, std::vector<int> vMatchPair12);
};


#endif //SLAM_ORB_TRACKER_MAPINITIALIZER_HPP
