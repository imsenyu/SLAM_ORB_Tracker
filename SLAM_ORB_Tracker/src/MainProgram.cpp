//
//  MainProgram.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/19/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "stdafx.hpp"
#include <boost/thread.hpp>

#include "Config.hpp"
#include "InputBuffer.hpp"
#include "Vocabulary.hpp"
#include "Tracker.hpp"
#include "FrameDrawer.hpp"
#include "MapDrawer.hpp"
#include "Map.hpp"

#include <opencv2/viz/vizcore.hpp>
#include <opencv2/calib3d/calib3d.hpp>

int main(int argc, char * argv[]) {



    // initialize Config for config load
    Config::parse(argc, argv);
    Config::loadConfig(Config::sPathConfigFile);

    // vocabulary initialize
    Vocabulary vocabulary(10,6);


//
//    if ( true )  {
//        std::vector<std::vector<cv::Mat>> allFeatureVoc;
//        int cate[]={4541,1101,4661,801,271,2761,1101,1101,4071,1591,1201};
//        //int cate[]={10,10,10,10,10,10,10,10,10,10,10,10,10,10};
//        for(int j=4;j<=10;j++) {
//            Vocabulary vocabulary(10,6);
//            Config::time(cv::format("voc create[%02d]",j));
//            std::vector<std::vector<cv::Mat>> vvFeature;
//
//            // initialize InputBuffer for image read
//            InputBuffer inputBuffer(cv::format("/Volumes/HDD/Workspace/Git/ZJU_Summer/Dataset/%02d/image_0/%%06d.png",j), 0, cate[j]);
//            boost::thread inputBufferThread( boost::bind(&InputBuffer::run, &inputBuffer) );
//
//            for(int i=0;i<cate[j];i++) {
//                shared_ptr<FrameState> pCurFrame = inputBuffer.get();
//                pCurFrame->extractInit();
//                std::vector<cv::Mat> vDesc;
//
//                cv::Mat matDesc = pCurFrame->mDescriptor.clone();
//                for(int i=0;i< matDesc.rows ;i++) {
//                    vDesc.push_back( matDesc.row(i) );
//                }
//                allFeatureVoc.push_back( vDesc );
//                vvFeature.push_back( vDesc );
//                printf("[%02d]desc %d\n", j, pCurFrame->mId);
//            }
//            vocabulary.create(vvFeature);
//            vocabulary.saveBinary(cv::format("/tmp/kitti_%02d.L6.bin",j));
//            Config::timeEnd(cv::format("voc create[%02d]",j));
//
//        }
//        Vocabulary vocabulary(10,6);
//        Config::time("voc create");
//        vocabulary.create(allFeatureVoc);
//        Config::timeEnd("voc create");
//
//        Config::time("voc save");
//        vocabulary.saveBinary("/tmp/allkittiVoc.L6.bin");
//        //vocabulary.saveToTextFile("/tmp/testVoc.txt");
//        Config::timeEnd("voc save");
//
//        return 0;
//    }
//

    // initialize InputBuffer for image read
    InputBuffer inputBuffer(Config::sPathImageLoad, Config::iImageLoadBegin, Config::iImageLoadEnd);
    boost::thread inputBufferThread( boost::bind(&InputBuffer::run, &inputBuffer) );

    Config::time("voc");
    std::cout<<"voc loading: "<<Config::sPathVocabulary<<std::endl;
    //bool isVocLoaded =  vocabulary.loadFromTextFile(Config::sPathVocabulary);
    bool isVocLoaded =  vocabulary.loadBinary(Config::sPathVocabulary);
    Config::timeEnd("voc");

    if ( !isVocLoaded ) {
        std::cerr << "vocabulary not loaded" << std::endl;
        exit(1);
    }
    //vocabulary.saveBinary("/tmp/ORBvoc.bin");

    // initialize Tracker for localization
    Map map;
    FrameDrawer frameDrawer;
    MapDrawer mapDrawer(&map);
    LocalMapper localMapper(&map);
    Tracker tracker(&inputBuffer, &frameDrawer, &mapDrawer, &vocabulary, &map, &localMapper);
    boost::thread trackerThread( boost::bind(&Tracker::run, &tracker) );
    
    
    
    while(1) {
        frameDrawer.show();
        mapDrawer.show();
    }
    
    // wait for quit
    getchar();
    return 0;
}
