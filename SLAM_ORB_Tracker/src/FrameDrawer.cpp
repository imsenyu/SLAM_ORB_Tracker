//
//  FrameDrawer.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/21/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "FrameDrawer.hpp"

FrameDrawer::FrameDrawer() {
    
}

void FrameDrawer::update(shared_ptr<FrameState> _pFS ) {
    shared_ptr<FrameState> _push = shared_ptr<FrameState>(_pFS);
    mBuffer.put(_push);
}

void FrameDrawer::drawFeaturePoint() {
    if ( !mImageDraw.cols && !mImageDraw.rows )
        mImageDraw = mImage.clone();
    
    for(int i=0;i<mvKeyPoint.size();i++) {
        cv::rectangle(mImageDraw, mvKeyPoint[i].pt - cv::Point2f(1.5,1.5), mvKeyPoint[i].pt + cv::Point2f(1.5,1.5), cv::Scalar(255,0,0,128));
    }
}


void FrameDrawer::drawText() {
    if ( !mImageDraw.cols && !mImageDraw.rows )
        mImageDraw = mImage.clone();
    
    cv::putText(mImageDraw, cv::format("[%d]", mId), cv::Point2f(mImageDraw.cols -70, mImageDraw.rows - 20), CV_FONT_NORMAL, 0.5f, cv::Scalar(255, 0, 255));

}

void FrameDrawer::get() {
    shared_ptr<FrameState> top = mBuffer.get();
    mImage = top->mImage;
    mvKeyPoint = top->mvKeyPoint;
    mId = top->mId;
}

void FrameDrawer::show() {

    get();   
    
    drawFeaturePoint();
    drawText();
    
    cv::imshow("FeaturedFrame", mImageDraw);
    cv::waitKey(10);

    mImageDraw = cv::Mat();
    
}


