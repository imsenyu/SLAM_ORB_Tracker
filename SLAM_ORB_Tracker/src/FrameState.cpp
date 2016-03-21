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
    mImage.release();
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
    if ( nKP == 0 ) return 0;
    
    
    return nKP;
}

void FrameState::drawKeyPoint() {
    
    mImagePointed = mImage.clone();
    
    for(int i=0;i<mvKeyPoint.size();i++) {
        cv::rectangle(mImagePointed, mvKeyPoint[i].pt - cv::Point2f(1.5,1.5), mvKeyPoint[i].pt + cv::Point2f(1.5,1.5), cv::Scalar(255,0,0));
        
    }
    
}