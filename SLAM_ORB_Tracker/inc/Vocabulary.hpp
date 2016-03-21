//
//  Vocabulary.hpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef Vocabulary_hpp
#define Vocabulary_hpp

#include "stdafx.hpp"
#include "FORB.h"
#include "TemplatedVocabulary.h"
#include "KeyFrameState.hpp"

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> Vocabulary;

//class KeyFrameDatabase {
//private:
//    const Vocabulary* mpVoc;
//
//public:
//    KeyFrameDatabase(const Vocabulary &_pVoc);
//    
//    void insert(KeyFrameState* _pKeyFrame);
//    
//    void remove(KeyFrameState* _pKeyFrame);
//    
//
//};

#endif /* Vocabulary_hpp */
