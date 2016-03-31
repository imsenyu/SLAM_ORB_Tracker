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
        cv::cvtColor(mImage,mImageDraw,CV_GRAY2RGB);
    
    for(int i=0;i<mvKeyPoint.size();i++) {
        if ( !!mvpMapPoint[i] ) continue;
        cv::rectangle(mImageDraw, mvKeyPoint[i].pt - cv::Point2f(1.5,1.5), mvKeyPoint[i].pt + cv::Point2f(1.5,1.5), cv::Scalar(0,0,255,128));
    }
}


void FrameDrawer::drawText() {
    if ( !mImageDraw.cols && !mImageDraw.rows )
        cv::cvtColor(mImage,mImageDraw,CV_GRAY2RGB);
    
    cv::putText(mImageDraw, cv::format("[%d]", mId), cv::Point2f(mImageDraw.cols -70, mImageDraw.rows - 20), CV_FONT_NORMAL, 0.5f, cv::Scalar(255, 0, 255));

}

void FrameDrawer::take() {
    shared_ptr<FrameState> top = mBuffer.take();
    mImage = top->mImage;
    mvKeyPoint = top->mvKeyPoint;
    mId = top->mId;
    mvpMapPoint = top->mvpMapPoint;
    mnTrackedType = top->mnTrackedType;
}

void FrameDrawer::show() {

    if ( mBuffer.hasNext() ) {
        take();

        drawFeaturePoint();
        drawText();

        cv::imshow("FeaturedFrame", mImageDraw);

        mImageDraw = cv::Mat();
    }


    cv::waitKey(10);
    
}


