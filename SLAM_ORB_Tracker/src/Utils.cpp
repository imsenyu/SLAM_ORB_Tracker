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
        matRotation = cv::Mat(3, 3, CV_32FC1);
        matRotation = 0.0f;
        matRotation.at<float>(0, 0) = matRotation.at<float>(1, 1) = matRotation.at<float>(2, 2) = 1.0f;

        return 0.0f;
    }
    
    matUnitVector = matUnitVector.cross(rtDir);
    matUnitVector = (matUnitVector / cv::norm(matUnitVector))* std::acos(thetaInR)*ratio;
    cv::Rodrigues(matUnitVector, matRotation);

    return std::acos(thetaInR)*180.0f / std::acos(-1) * (rtDir.at<double>(0,0) >0 ? 1.0f : -1.0f );
}

cv::Point3d Utils::convertToPoint3d(const cv::Mat &mat31) {
    cv::Point3d ret;
    if ( mat31.type() == CV_32FC1 ) {
        ret.x = mat31.at<float>(0);
        ret.y = mat31.at<float>(1);
        ret.z = mat31.at<float>(2);
    }
    else if ( mat31.type() == CV_64FC1 ) {
        ret.x = mat31.at<double>(0);
        ret.y = mat31.at<double>(1);
        ret.z = mat31.at<double>(2);
    }
    else {
        std::cout<<"matrix type error"<<std::endl;
    }
    return ret;
}

cv::Mat Utils::convertToCvMat31(const cv::Point3f p3) {
    cv::Mat ret(3, 1, CV_32FC1);
    ret.at<float>(0,0) = p3.x;
    ret.at<float>(1,0) = p3.y;
    ret.at<float>(2,0) = p3.z;
    return ret;
}

g2o::SE3Quat Utils::convertToSE3Quat(const cv::Mat &cvT) {

    Eigen::Matrix<double,3,3> R;
    Eigen::Matrix<double,3,1> t;
    if( cvT.type()==CV_32FC1 ) {
        R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
            cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
            cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);
        t = Eigen::Matrix<double,3,1>(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));
    }
    else if( cvT.type() == CV_64FC1 ) {
        R << cvT.at<double>(0,0), cvT.at<double>(0,1), cvT.at<double>(0,2),
            cvT.at<double>(1,0), cvT.at<double>(1,1), cvT.at<double>(1,2),
            cvT.at<double>(2,0), cvT.at<double>(2,1), cvT.at<double>(2,2);
        t = Eigen::Matrix<double,3,1>(cvT.at<double>(0,3), cvT.at<double>(1,3), cvT.at<double>(2,3));
    }
    else {
        std::cout<<"matrix type error"<<std::endl;
    }




    return g2o::SE3Quat(R,t);
}

Eigen::Matrix<double,3,1> Utils::convertToEigenMat31(const cv::Mat &cvVector)
{
    Eigen::Matrix<double,3,1> v;
    if ( cvVector.type() == CV_32FC1 ) {
        v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);
    }
    else if ( cvVector.type() == CV_64FC1 ) {
        v << cvVector.at<double>(0), cvVector.at<double>(1), cvVector.at<double>(2);
    }
    else {
        std::cout<<"matrix type error"<<std::endl;
    }

    return v;
}

cv::Mat Utils::convertToCvMat44(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    return convertToCvMat44(eigMat);
}

cv::Mat Utils::convertToCvMat44(const Eigen::Matrix<double, 4, 4> &m)
{
    cv::Mat cvMat(4,4,CV_32FC1);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Utils::convertToCvMat31(const Eigen::Matrix<double,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_32FC1);
    for(int i=0;i<3;i++)
        cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

cv::Mat Utils::convectToSymmetricMatrix(const cv::Mat &v)
{
    cv::Mat_<float> ret(3,3);
    if ( v.type() == CV_32FC1 ) {
        ret << 0.0f, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0.0f;
    }
    else if ( v.type() == CV_64FC1 ) {
        ret << 0.0f, -v.at<double>(2), v.at<double>(1),
            v.at<double>(2),               0,-v.at<double>(0),
            -v.at<double>(1),  v.at<double>(0),              0.0f;
    }
    else {
        std::cout<<"err"<<std::endl;
    }

    return ret;
}

std::string  Utils::timenow() {
    namespace pt = boost::posix_time;
    return pt::to_iso_string(pt::second_clock::local_time());
}


Tick::Tick(double _fps): fps(_fps), inited(false) {
    fps = fabs(fps);
    if ( fps < 0.01 ) fps = 0.01;
    step = 1.0f/fps;
}

bool Tick::tock() {
    if ( !inited ) {
        mtStart = clock();
        inited = true;
    }
    else {
        mtEnd = clock();
        double duration = (mtEnd - mtStart) / ((double) CLOCKS_PER_SEC);
        if ( step > duration ) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(  1000*(step - duration)  ));
        }
        mtStart = clock();
    }
    return true;
}

bool Tick::try_tock() {
    if ( !inited ) {
        mtStart = clock();
        inited = true;
    }
    else {
        mtEnd = clock();
        double duration = (mtEnd - mtStart) / ((double) CLOCKS_PER_SEC);
        if ( step > duration ) {
            return false;
        }
        mtStart = clock();
    }
    return true;
}
