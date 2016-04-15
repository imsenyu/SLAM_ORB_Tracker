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
#include <g2o/types/slam3d/se3quat.h>


namespace Utils {
    /** \fn 获得 \var rtDir(3,1,CV_F64C1) 相对于 [0,0,1]' 方向的旋转矩阵 \var matRotatoin */
    double getRodriguesRotation(cv::Mat rtDir, cv::Mat& matRotation, cv::Mat orignDir = Const::mat31_001,double ratio = 1.0f);
    
    /** \fn cv::Mat(3,1,CV_F64C1) to cv::Point3d */
    cv::Point3d convertToPoint3d(const cv::Mat &mat31);
    cv::Mat     convertToCvMat31(const cv::Point3f p3);
    g2o::SE3Quat convertToSE3Quat(const cv::Mat &cvT);
    Eigen::Matrix<double,3,1> convertToEigenMat31(const cv::Mat &cvVector);
    cv::Mat convertToCvMat44(const g2o::SE3Quat &SE3);
    cv::Mat convertToCvMat44(const Eigen::Matrix<double, 4, 4> &m);
    cv::Mat convectToSymmetricMatrix(const cv::Mat &v);
    cv::Mat convertToCvMat31(const Eigen::Matrix<double,3,1> &m);
    std::string timenow();
};

class Tick {
private:
    time_t mtStart;
    time_t mtEnd;
    double step;
    double fps;
    bool inited;

public:
    Tick(double _fps);

    bool tock();
    bool try_tock();
};
#endif /* Utils_hpp */
