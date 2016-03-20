//
//  ConcurrentContainer.hpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef ConcurrentQueue_hpp
#define ConcurrentQueue_hpp

#include "stdafx.hpp"
#include <boost/interprocess/sync/interprocess_semaphore.hpp>

template <class _Ele>
class ConcurrentQueue {
    typedef boost::interprocess::interprocess_semaphore semaphore;
    
    semaphore _semFull;
    semaphore _semEmpty;
    semaphore _semMutex;
    int capacity;
    std::queue<_Ele> mContainer;
    
    
public:
    
    ConcurrentQueue(int _capacity = 100):
    capacity(_capacity), _semMutex(1), _semFull(0), _semEmpty(_capacity)  {
        
    }
    int getCapacity() {
        return capacity;
    }

    void put(_Ele ptrEle) {
        _semEmpty.wait();
        _semMutex.wait();
        
        mContainer.push(ptrEle);
        
        _semMutex.post();
        _semFull.post();
    }

    _Ele get() {
        _semFull.wait();
        _semMutex.wait();
        
        _Ele ret = mContainer.front();
        mContainer.pop();
        
        _semMutex.post();
        _semEmpty.post();
        
        return ret;
        
    }};

#endif /* ConcurrentQueue_hpp */
