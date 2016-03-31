//
//  Config.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "Config.hpp"



std::string Config::sPathConfigFile = "./slam_config.xml";
std::string Config::sPathImageLoad = "";
int Config::iImageLoadBegin = 0;
int Config::iImageLoadEnd = 0;
std::string Config::sPathVocabulary = "";
int Config::iFeatureNum = 0;
double Config::dDrawFrameStep = 0.0f;
cv::Mat Config::mCameraParameter;
double Config::dOpticalFlowThreshold;
∑int Config::dScaleLevel;
double Config::dScaleFactor;
std::vector<double> Config::vScaleFactors;
std::vector<double> Config::vLevelSigma2;
std::vector<double> Config::vInvLevelSigma2;
double Config::dFx;
double Config::dFy;
double Config::dCx;
double Config::dCy;


int Config::parse(int argc, char * argv[]) {
    namespace po = boost::program_options;
    po::options_description desc("Options");
    
    desc.add_options()
    ("help,h", "Print help messages")
    ("config,c", po::value<std::string>(), "Set config file path");
    
    po::variables_map vm;
    
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        
        if ( vm.count("config") ) {
            sPathConfigFile = vm["config"].as<std::string>();
            std::cout<<"set Config to :'"<< sPathConfigFile <<"'"<<std::endl;
        }
    }
    catch(po::required_option& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    }
    catch(po::error& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
        
    }
    
    return 0;
}

int Config::loadConfig(std::string cfgPath) {
    cv::FileStorage fs;
    bool opened = fs.open(cfgPath, cv::FileStorage::READ);
    
    //////////////////////////////////////////////////////////////////////////
    // 根据默认配置 和 配置文件自定义设置更新 参数
    sPathImageLoad			= getDefault<std::string>("/Volumes/HDD/Workspace/Git/ZJU_Summer/Dataset/00/image_0/%06d.png", fs["sPathImageLoad"]);
    iImageLoadBegin			= getDefault<int>(0, fs["iImageBeginIdx"]);
    iImageLoadEnd			= getDefault<int>(1, fs["iImageEndIdx"]);
    sPathVocabulary         = getDefault<std::string>("/tmp/voc.txt", fs["sPathVocabulary"]);
    iFeatureNum             = getDefault<int>(1000, fs["iFeatureNum"]);
    dDrawFrameStep          = getDefault<double>(1.0, fs["dDrawFrameStep"]);
    mCameraParameter        = getDefault<cv::Mat>(Const::mat33_111, fs["mCameraParameter"]);
    dOpticalFlowThreshold   = getDefault<double>(1.0f, fs["dOpticalFlowThreshold"]);
    dScaleLevel             = getDefault<int>(8, fs["dScaleLevel"]);
    dScaleFactor            = getDefault<double>(1.2f, fs["dScaleFactor"]);

    fs.release();
    fs.open(cfgPath, cv::FileStorage::WRITE);
    
    fs << "sPathImageLoad"			<< sPathImageLoad;
    fs << "iImageBeginIdx"			<< iImageLoadBegin;
    fs << "iImageEndIdx"			<< iImageLoadEnd;
    fs << "sPathVocabulary"         << sPathVocabulary;
    fs << "iFeatureNum"             << iFeatureNum;
    fs << "dDrawFrameStep"          << dDrawFrameStep;
    fs << "mCameraParameter"        << mCameraParameter;
    fs << "dOpticalFlowThreshold"   << dOpticalFlowThreshold;
    fs << "dScaleLevel"             << dScaleLevel;
    fs << "dScaleFactor"            << dScaleFactor;
    fs.release();

    // do some MATH

    vScaleFactors.resize(dScaleLevel);
    vLevelSigma2.resize(dScaleLevel);
    vInvLevelSigma2.resize(vLevelSigma2.size());

    vScaleFactors[0]=1.0f;
    vLevelSigma2[0]=1.0f;
    vInvLevelSigma2[0]=1.0f;
    for(int i=1; i<dScaleLevel; i++)
    {
        vScaleFactors[i]=vScaleFactors[i-1]*dScaleFactor;
        vLevelSigma2[i]=vScaleFactors[i]*vScaleFactors[i];
        vInvLevelSigma2[i]=1/vLevelSigma2[i];
    }

    dFx = mCameraParameter.at<double>(0,0);
    dFy = mCameraParameter.at<double>(1,1);
    dCx = mCameraParameter.at<double>(0,2);
    dCy = mCameraParameter.at<double>(1,2);

    return !opened;
}

std::map<std::string, double> Config::mTimer;

void Config::time(std::string str) {
    std::cout << "-----------Time_Begin[" << str << "]:" << std::endl;
    if (mTimer.find(str) == mTimer.end())
        mTimer.insert(make_pair(str, clock()));
    else
        mTimer.find(str)->second = clock();
}

void Config::timeEnd(std::string str) {
#undef OUTPUTTIMESCALE
#if defined(WIN32) || defined(_WIN32)
#define OUTPUTTIMESCALE 1.0
#else
#define OUTPUTTIMESCALE 0.001
#endif
    
    if (mTimer.find(str) == mTimer.end())\
        std::cout << "-----------Time_End[" << str << "] ERROR" << std::endl;
    else
        std::cout << "-----------Time_End[" << str << "]:(" <<
        (clock() - mTimer.find(str)->second)*OUTPUTTIMESCALE <<
        ")ms" << std::endl;

    
}
