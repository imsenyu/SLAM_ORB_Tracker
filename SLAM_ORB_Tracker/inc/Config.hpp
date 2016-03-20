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
};



#endif /* Config_hpp */
