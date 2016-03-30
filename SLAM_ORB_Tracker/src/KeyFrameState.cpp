//
//  KeyFrameState.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "KeyFrameState.hpp"

KeyFrameState::KeyFrameState(shared_ptr<FrameState> _pFrame, Vocabulary *_pVocabulary):
    mpFrame( _pFrame ), mpVocabulary(_pVocabulary),
    mvpMapPoint( _pFrame->mvpMapPoint )
{

    updatePose(_pFrame->mT2w);

    //int nKP =_pFrame->mvKeyPoint.size();
    //mvpMapPoint = std::vector<shared_ptr<MapPoint>>( nKP, shared_ptr<MapPoint>(NULL) );

}

void KeyFrameState::getBoW() {
    if(mvBow.empty() || mvFeature.empty())
    {
        std::vector<cv::Mat> vDesc;
        cv::Mat& mDesc = mpFrame->mDescriptor;
        int N = mDesc.rows;
        for(int i=0;i<N;i++) {
            vDesc.push_back( mDesc.row(i) );
        }

        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpVocabulary->transform(vDesc,mvBow,mvFeature,4);
    }

}

void KeyFrameState::updatePose(cv::Mat _mT) {
    mT2w = _mT.clone();

    mMatR = mT2w.rowRange(0,3).colRange(0,3);
    mMatT = mT2w.rowRange(0,3).col(3);

    mO2w = - mMatR.t() * mMatT;
}

float KeyFrameState::ComputeSceneMedianDepth(int r) {

    std::vector<shared_ptr<MapPoint>> vpMapPoints;
    cv::Mat Tcw_;
        vpMapPoints = mvpMapPoint;
        Tcw_ = mT2w.clone();


    std::vector<float> vDepths;
    vDepths.reserve(mvpMapPoint.size());
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(size_t i=0; i<mvpMapPoint.size(); i++)
    {
        if(mvpMapPoint[i])
        {
            shared_ptr<MapPoint> pMP = mvpMapPoint[i];
            cv::Mat x3Dw = pMP->mPos;
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/r];
}

std::vector<shared_ptr<KeyFrameState>> KeyFrameState::GetBestCovisibilityKeyFrames(const int &N)
{
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return std::vector<shared_ptr<KeyFrameState>>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}
void KeyFrameState::AddConnection(shared_ptr<KeyFrameState> pKF, const int &weight)
{

    if(!mConnectedKeyFrameWeights.count(pKF))
        mConnectedKeyFrameWeights[pKF]=weight;
    else if(mConnectedKeyFrameWeights[pKF]!=weight)
        mConnectedKeyFrameWeights[pKF]=weight;
    else
        return;


    UpdateBestCovisibles();
}

void KeyFrameState::UpdateBestCovisibles()
{
    //std::cout<<"UpdateBestCovisibles at frameId"<<(this->mpFrame->mId)<<std::endl;

    std::vector<std::pair<int,shared_ptr<KeyFrameState>> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(std::map<shared_ptr<KeyFrameState>,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        vPairs.push_back(std::make_pair(mit->second,mit->first));

    std::sort(vPairs.begin(),vPairs.end());
    std::list<shared_ptr<KeyFrameState>> lKFs;
    std::list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = std::vector<shared_ptr<KeyFrameState>>(lKFs.begin(),lKFs.end());
    //std::cout<<"mvpOrderedConnectedKeyFrames size "<<mvpOrderedConnectedKeyFrames.size()<<std::endl;
    mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());
}

void KeyFrameState::UpdateConnections()
{
    //std::cout<<"updateConnection at frameId"<<(this->mpFrame->mId)<<std::endl;
    std::map<shared_ptr<KeyFrameState>,int> KFcounter;

    std::vector<shared_ptr<MapPoint>> vpMP = mvpMapPoint;

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(std::vector<shared_ptr<MapPoint>>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {

        shared_ptr<MapPoint> pMP = *vit;

        if(!pMP)
            continue;

        //if(pMP->isBad())
        //    continue;

        std::map<shared_ptr<KeyFrameState>,int> observations = pMP->msKeyFrame2FeatureId;

        for(std::map<shared_ptr<KeyFrameState>,int>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            //std::cout<<"saw mapPoint in frame "<<(mit->first->mpFrame->mId)<<" featureId "<<mit->second<<std::endl;
            if(mit->first->mpFrame->mId==mpFrame->mId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    //std::cout<<"KFCounter.size "<<KFcounter.size()<<std::endl;
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    shared_ptr<KeyFrameState> pKFmax=shared_ptr<KeyFrameState>(NULL);
    int th = 15;

    std::vector<std::pair<int,shared_ptr<KeyFrameState>> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(std::map<shared_ptr<KeyFrameState>,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(std::make_pair(mit->second,mit->first));

                //(mit->first)->AddConnection(shared_ptr<KeyFrameState>(this),mit->second);
                (mit->first)->AddConnection(shared_from_this(),mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(std::make_pair(nmax,pKFmax));
        //pKFmax->AddConnection(shared_ptr<KeyFrameState>(this),nmax);
        pKFmax->AddConnection(shared_from_this(),nmax);
    }

    std::sort(vPairs.begin(),vPairs.end());
    std::list<shared_ptr<KeyFrameState>> lKFs;
    std::list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }



    // mspConnectedKeyFrames = spConnectedKeyFrames;
    // copy mvpOrderedConnectedKeyFrames for not destory them by shared_ptr<>
    //std::vector<shared_ptr<KeyFrameState>> copy = mvpOrderedConnectedKeyFrames;


    mConnectedKeyFrameWeights = KFcounter;
    mvpOrderedConnectedKeyFrames = std::vector<shared_ptr<KeyFrameState>>(lKFs.begin(),lKFs.end());

    //std::cout<<"mvpOrderedConnectedKeyFrames size "<<mvpOrderedConnectedKeyFrames.size()<<std::endl;
    mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());

//    if(mbFirstConnection && mpFrame->mId!=0)
//    {
//        mpParent = mvpOrderedConnectedKeyFrames.front();
//        mpParent->AddChild(this);
//        mbFirstConnection = false;
//    }


}
void KeyFrameState::insertMapPoint(shared_ptr<MapPoint> pMp, int nP) {
    mvpMapPoint[nP] = pMp;
    mpFrame->insertMapPoint(pMp, nP);
}