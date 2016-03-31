//
//  Config.hpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef Config_hpp
#define Config_hpp

#include "stdafx.hpp"
#include <boost/program_options.hpp>


namespace Config {

    /** \var timer */
    extern std::map<std::string, double> mTimer;
    void time(std::string str);
    void timeEnd(std::string str);
    
    /** \fn parse cmd line arguments and config path */
    int parse(int argc, char * argv[]);
    /** \fn load config detail */
    int loadConfig(std::string configFilePath);
    /** \fn get config defalut wrap */
    template<class _Type>
    _Type getDefault(_Type defaultVal, cv::FileNode fn) {
        if (fn.empty() == true)
            return defaultVal;
        else {
            _Type ret;
            fn >> ret;
            return ret;
        }
    }
    
    extern std::string sPathConfigFile; /** \var string:配置文件读取路径 */
    extern std::string sPathImageLoad; /** \var string:数据集图像序列读取路径 {idx:%06d} */
    extern int iImageLoadBegin; /** \var int:读取图像起始标号 */
    extern int iImageLoadEnd; /** \var int:读取图像结束标号 */
    extern std::string sPathVocabulary; /** \var string:词典*/
    extern int iFeatureNum;
    extern double dDrawFrameStep;
    extern cv::Mat mCameraParameter;
    extern double dOpticalFlowThreshold;
    extern int dScaleLevel;
    extern double dScaleFactor;
    extern std::vector<double> vScaleFactors;
    extern std::vector<double> vLevelSigma2;
    extern std::vector<double> vInvLevelSigma2;
    extern double dFx;
    extern double dFy;
    extern double dCx;
    extern double dCy;

};



#endif /* Config_hpp */
