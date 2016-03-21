//
//  Utils.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/21/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "Utils.hpp"

double Utils::getRodriguesRotation(cv::Mat _rtDir, cv::Mat& matRotation, cv::Mat _oriDir, double ratio) {
    //使用 叉乘求出 旋转轴[1*3]，然后 用 cos(theta) = P·Q / |P|*|Q| 求出theta弧度作为旋转轴向量的 normest
    //带入函数 算出旋转矩阵，然后再说
    
    cv::Mat matUnitVector = _oriDir.clone();
    cv::Mat rtDir = _rtDir / cv::norm(_rtDir);
    
    double thetaInR = 0.0f;
    thetaInR = (matUnitVector.dot(rtDir)) / std::sqrt(matUnitVector.dot(matUnitVector)) / std::sqrt(rtDir.dot(rtDir));
    

    if (abs(thetaInR) > 1 - 1e-7) {
        matRotation = cv::Mat(3, 3, CV_64FC1);
        matRotation = 0.0f;
        matRotation.at<double>(0, 0) = matRotation.at<double>(1, 1) = matRotation.at<double>(2, 2) = 1.0f;

        return 0.0f;
    }
    
    matUnitVector = matUnitVector.cross(rtDir);
    matUnitVector = (matUnitVector / cv::norm(matUnitVector))* std::acos(thetaInR)*ratio;
    cv::Rodrigues(matUnitVector, matRotation);

    return std::acos(thetaInR)*180.0f / std::acos(-1) * (rtDir.at<double>(0,0) >0 ? 1.0f : -1.0f );
}

cv::Point3d Utils::convert(cv::Mat& mat31) {
    cv::Point3d ret;
    ret.x = mat31.at<double>(0, 0);
    ret.y = mat31.at<double>(1, 0);
    ret.z = mat31.at<double>(2, 0);
    return ret;
}

cv::Mat Utils::convert(cv::Point3f p3) {
    cv::Mat ret(3, 1, CV_64FC1);
    ret.at<double>(0,0) = p3.x;
    ret.at<double>(1,0) = p3.y;
    ret.at<double>(2,0) = p3.z;
    return ret;
}