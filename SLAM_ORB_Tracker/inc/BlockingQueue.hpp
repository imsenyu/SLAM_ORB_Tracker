//
//  ConcurrentContainer.hpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef BlockingQueue_hpp
#define BlockingQueue_hpp

#include "stdafx.hpp"
#include <boost/interprocess/sync/interprocess_semaphore.hpp>

template <class _Ele>
class BlockingQueue {
    typedef boost::interprocess::interprocess_semaphore semaphore;
    
    semaphore mSemFull;
    semaphore mSemEmpty;
    semaphore mSemMutex;
    int capacity;
    std::queue<_Ele> mContainer;
    
    
public:
    
    BlockingQueue(int _capacity = 5000):
    capacity(_capacity), mSemMutex(1), mSemFull(0), mSemEmpty(_capacity)  {
        
    }
    int getCapacity() {
        return capacity;
    }

    void put(_Ele ptrEle) {
        mSemEmpty.wait();
        mSemMutex.wait();
        
        mContainer.push(ptrEle);
        
        mSemMutex.post();
        mSemFull.post();
    }

    _Ele take() {
        mSemFull.wait();
        mSemMutex.wait();
        
        _Ele ret = mContainer.front();
        mContainer.pop();
        
        mSemMutex.post();
        mSemEmpty.post();
        
        return ret;
        
    }

    bool hasNext() {
        bool ret;
        mSemMutex.wait();
        ret = mContainer.size();
        mSemMutex.post();
        return ret;
    }
};
#endif /* ConcurrentQueue_hpp */
