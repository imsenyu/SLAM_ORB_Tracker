//
//  FrameBuffer.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "FrameBuffer.hpp"

FrameBuffer::FrameBuffer(int _id): mId(_id), mLoaded(false) {
    loadImage(mId);
}

bool FrameBuffer::loadImage(int _id) {
    if (_id < 0) return false;
    mId = _id;
    std::string imgPath = cv::format(Config::sPathImageLoad.c_str(), mId);
    mImage = cv::imread(imgPath);
    
    // 如果读取图像失败
    if (mImage.rows == 0 || mImage.cols == 0) {
        std::string error = "Image Load Error: " + imgPath;
        std::cout<<error <<std::endl;
        //throw std::exception( error.c_str() );
    }

    mLoaded = true;
    return mLoaded;
}