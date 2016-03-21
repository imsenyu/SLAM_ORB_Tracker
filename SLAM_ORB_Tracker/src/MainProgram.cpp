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

int main(int argc, char * argv[]) {
  
    // initialize Config for config load
    Config::parse(argc, argv);
    Config::loadConfig(Config::sPathConfigFile);
    
    // vocabulary initialize
    Vocabulary vocabulary;
    Config::time("voc");
    bool isVocLoaded = true || vocabulary.loadFromTextFile(Config::sPathVocabulary);
    Config::timeEnd("voc");
    
    if ( !isVocLoaded ) {
        std::cerr << "vocabulary not loaded" << std::endl;
        exit(1);
    }
    
    
    
    // initialize InputBuffer for image read
    InputBuffer inputBuffer(Config::sPathImageLoad, Config::iImageLoadBegin, Config::iImageLoadEnd);
    boost::thread inputBufferThread( boost::bind(&InputBuffer::run, &inputBuffer) );
    
    // initialize Tracker for localization
    FrameDrawer frameDrawer;
    MapDrawer mapDrawer;
    Tracker tracker(&inputBuffer, &frameDrawer, &mapDrawer);
    boost::thread trackerThread( boost::bind(&Tracker::run, &tracker) );
    
    
    
    while(1) {
        frameDrawer.show();
        mapDrawer.show();
    }
    
    // wait for quit
    getchar();
    return 0;
}
