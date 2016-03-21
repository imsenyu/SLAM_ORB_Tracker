//
//  Utils.hpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/21/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef Utils_hpp
#define Utils_hpp

#include "stdafx.hpp"
#include "Constant.hpp"

namespace Utils {
    /** \fn 获得 \var rtDir(3,1,CV_F64C1) 相对于 [0,0,1]' 方向的旋转矩阵 \var matRotatoin */
    double getRodriguesRotation(cv::Mat rtDir, cv::Mat& matRotation, cv::Mat orignDir = Const::mat31_001,double ratio = 1.0f);
    
    /** \fn cv::Mat(3,1,CV_F64C1) to cv::Point3d */
    cv::Point3d convert(cv::Mat& mat31);
    cv::Mat     convert(cv::Point3f p3);
};


#endif /* Utils_hpp */
