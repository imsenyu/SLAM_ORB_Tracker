//
//  FrameBuffer.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "FrameState.hpp"

FrameState::FrameState(int _id): mId(_id), mLoaded(false) {
    loadImage(mId);
}

FrameState::FrameState(const FrameState& _FS) {
    mId = _FS.mId;
    mLoaded = _FS.mLoaded;
    mImage = _FS.mImage.clone();
    mvKeyPoint = _FS.mvKeyPoint;
    mDescriptor = _FS.mDescriptor.clone();
}

FrameState::FrameState(const FrameState* _pFS) {
    mId = _pFS->mId;
    mLoaded = _pFS->mLoaded;
    mImage = _pFS->mImage.clone();
    mvKeyPoint = _pFS->mvKeyPoint;
    mDescriptor = _pFS->mDescriptor;
}

bool FrameState::loadImage(int _id) {
    if (_id < 0) return false;
    mId = _id;
    std::string imgPath = cv::format(Config::sPathImageLoad.c_str(), mId);
    mImage = cv::imread(imgPath);
    
    // image load error
    if (mImage.rows == 0 || mImage.cols == 0) {
        std::string error = "Image Load Error: " + imgPath;
        std::cout<<error <<std::endl;
        //throw std::exception( error.c_str() );
    }

    mLoaded = true;
    return mLoaded;
}

FrameState::~FrameState() {
}

int FrameState::extract() {
    
    switch ( 1 ) {
        case 1: {
            cv::ORB orbDetector(Config::iFeatureNum, 1.2f, 8);
            orbDetector.detect(mImage, mvKeyPoint);
            orbDetector.compute(mImage, mvKeyPoint, mDescriptor);
        } break;
        case 2: {
            cv::SiftFeatureDetector siftDetector(Config::iFeatureNum);
            siftDetector.detect(mImage, mvKeyPoint);
            cv::SiftDescriptorExtractor siftExtractor(Config::iFeatureNum);
            siftExtractor.compute(mImage, mvKeyPoint, mDescriptor);
        } break;
    }

    int nKP = mvKeyPoint.size();
    mvpMapPoint = std::vector<shared_ptr<MapPoint>>( nKP, shared_ptr<MapPoint>(NULL) );
    mvMatchMask = std::vector<uchar>( nKP, false );
    if ( nKP == 0 ) return 0;

    return nKP;
}


