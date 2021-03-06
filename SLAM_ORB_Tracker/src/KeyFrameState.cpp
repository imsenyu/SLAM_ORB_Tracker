//
//  KeyFrameState.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "KeyFrameState.hpp"
#include "MapPoint.hpp"

bool std::less<std::shared_ptr<KeyFrameState>>::operator () (const std::shared_ptr<KeyFrameState> &x, const std::shared_ptr<KeyFrameState> &y) const {
    return x->mId < y->mId;
}

KeyFrameState::KeyFrameState(shared_ptr<FrameState> _pFrame, Vocabulary *_pVocabulary):
    mpFrame( _pFrame ), mpVocabulary(_pVocabulary),
    /*mvpMapPoint( _pFrame->mvpMapPoint ),*/ mId(_pFrame->mId),
    mbNotErase(true), mbToBeErased(false), mbBad(false),
    mnBAFixedForKF(0),
    mnBALocalForKF(0),
    mnFuseTargetForKF(0),
    mnTrackReferenceForFrame(0)
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
    mpFrame->updatePose(_mT);
}

float KeyFrameState::ComputeSceneMedianDepth(int r) {

    std::vector<shared_ptr<MapPoint>> vpMapPoints;
    cv::Mat Tcw_;
        vpMapPoints = GetMapPointMatch();
        Tcw_ = mpFrame->mT2w.clone();


    std::vector<float> vDepths;
    vDepths.reserve(vpMapPoints.size());
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(size_t i=0; i<vpMapPoints.size(); i++)
    {
        if(vpMapPoints[i])
        {
            shared_ptr<MapPoint> pMP = vpMapPoints[i];
            cv::Mat x3Dw = pMP->getMPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    std::stable_sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/r];
}

std::vector<shared_ptr<KeyFrameState>> KeyFrameState::GetBestCovisibilityKeyFrames(const int &N)
{
    if((int) mvpNeighbourKeyFrames.size()<N)
        return mvpNeighbourKeyFrames;
    else
        return std::vector<shared_ptr<KeyFrameState>>(mvpNeighbourKeyFrames.begin(), mvpNeighbourKeyFrames.begin()+N);

}
void KeyFrameState::AddConnection(shared_ptr<KeyFrameState> pKF, const int &weight)
{

    if(!mKeyFrameWeightsMap.count(pKF))
        mKeyFrameWeightsMap[pKF]=weight;
    else if(mKeyFrameWeightsMap[pKF]!=weight)
        mKeyFrameWeightsMap[pKF]=weight;
    else
        return;


    UpdateBestCovisibles();
}

bool pairSortFunc_int_pKF(const std::pair<int,shared_ptr<KeyFrameState>>& l, const std::pair<int,shared_ptr<KeyFrameState>>& r) {
    if ( l.first != r.first ) return l.first < r.first;
    return l.second->mId > r.second->mId;
}


void KeyFrameState::UpdateBestCovisibles()
{
    //std::cout<<"UpdateBestCovisibles at frameId"<<(this->mpFrame->mId)<<std::endl;

    std::vector<std::pair<int,shared_ptr<KeyFrameState>> > vPairs;
    vPairs.reserve(mKeyFrameWeightsMap.size());
    for(std::map<shared_ptr<KeyFrameState>,int>::iterator mit= mKeyFrameWeightsMap.begin(), mend= mKeyFrameWeightsMap.end(); mit!=mend; mit++)
        vPairs.push_back(std::make_pair(mit->second,mit->first));

    std::stable_sort(vPairs.begin(), vPairs.end(), pairSortFunc_int_pKF);
    std::list<shared_ptr<KeyFrameState>> lKFs;
    std::list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpNeighbourKeyFrames = std::vector<shared_ptr<KeyFrameState>>(lKFs.begin(),lKFs.end());
    //std::cout<<"mvpNeighbourKeyFrames size "<<mvpNeighbourKeyFrames.size()<<std::endl;
    mvWeights = std::vector<int>(lWs.begin(), lWs.end());
}

void KeyFrameState::UpdateConnections()
{
    //std::cout<<"updateConnection at frameId"<<(this->mpFrame->mId)<<std::endl;
    std::map<shared_ptr<KeyFrameState>,int> KFcounter;

    std::vector<shared_ptr<MapPoint>> vpMP = GetMapPointMatch();

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(std::vector<shared_ptr<MapPoint>>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {

        shared_ptr<MapPoint> pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

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

    std::stable_sort(vPairs.begin(),vPairs.end(), pairSortFunc_int_pKF);
    std::list<shared_ptr<KeyFrameState>> lKFs;
    std::list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }



    // mspConnectedKeyFrames = spConnectedKeyFrames;
    // copy mvpNeighbourKeyFrames for not destory them by shared_ptr<>
    //std::vector<shared_ptr<KeyFrameState>> copy = mvpNeighbourKeyFrames;


    mKeyFrameWeightsMap = KFcounter;
    mvpNeighbourKeyFrames = std::vector<shared_ptr<KeyFrameState>>(lKFs.begin(),lKFs.end());

    //std::cout<<"mvpNeighbourKeyFrames size "<<mvpNeighbourKeyFrames.size()<<std::endl;
    mvWeights = std::vector<int>(lWs.begin(), lWs.end());

//    if(mbFirstConnection && mpFrame->mId!=0)
//    {
//        mpParent = mvpNeighbourKeyFrames.front();
//        mpParent->AddChild(this);
//        mbFirstConnection = false;
//    }


}

std::vector<shared_ptr<MapPoint>> KeyFrameState::GetMapPointMatch() {
    return mpFrame->mvpMapPoint;
}

void KeyFrameState::insertMapPoint(shared_ptr<MapPoint> pMp, int nP) {
    //mvpMapPoint[nP] = pMp;
    mpFrame->insertMapPoint(pMp, nP);
}

bool KeyFrameState::IsInImage(const float &x, const float &y)
{
    return mpFrame->IsInImage(x,y);
}
std::vector<size_t> KeyFrameState::getFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel, const int maxLevel)
{
    return mpFrame->getFeaturesInArea(x,y,r,minLevel,maxLevel);
}
int KeyFrameState::GetKeyPointScaleLevel(const size_t &idx) const
{
    return mpFrame->mvKeyPoint[idx].octave;
}

cv::Mat KeyFrameState::GetDescriptor(const size_t &idx)
{
    return mpFrame->mDescriptor.row(idx).clone();
}
shared_ptr<MapPoint> KeyFrameState::GetMapPoint(const size_t &idx)
{
    return mpFrame->mvpMapPoint[idx];
}
cv::KeyPoint KeyFrameState::GetKeyPoint(int i) {
    return mpFrame->mvKeyPoint[i];
}
void KeyFrameState::EraseMapPointMatch(shared_ptr<MapPoint> pMP)
{
    int idx = -1;
    if ( pMP->msKeyFrame2FeatureId.count(shared_from_this()) )
        idx = pMP->msKeyFrame2FeatureId[shared_from_this()];
    if(idx>=0)
        mpFrame->mvpMapPoint[idx]=shared_ptr<MapPoint>(NULL);
}


void KeyFrameState::EraseMapPointMatch(const size_t &idx)
{
    mpFrame->mvpMapPoint[idx]=shared_ptr<MapPoint>(NULL);
}

void KeyFrameState::ReplaceMapPointMatch(const size_t &idx, shared_ptr<MapPoint> pMP)
{
    mpFrame->mvpMapPoint[idx]=pMP;
}

cv::Mat KeyFrameState::getMatT2w() {
    return mpFrame->mT2w.clone();
}

cv::Mat KeyFrameState::getMatR2w() {
    return mpFrame->mMatR.clone();
}

cv::Mat KeyFrameState::getMatt2w() {
    return mpFrame->mMatT.clone();
}

cv::Mat KeyFrameState::getMatO2w() {
    return mpFrame->mO2w.clone();
}

void KeyFrameState::SetBadFlag()
{
    {

        if(mId==0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    for(std::map<shared_ptr<KeyFrameState>,int>::iterator mit = mKeyFrameWeightsMap.begin(), mend= mKeyFrameWeightsMap.end(); mit!=mend; mit++)
        mit->first->EraseConnection(shared_from_this());

    for(size_t i=0; i<mpFrame->mvpMapPoint.size(); i++)
        if(mpFrame->mvpMapPoint[i])
            mpFrame->mvpMapPoint[i]->EraseObservation(shared_from_this());
    {


        mKeyFrameWeightsMap.clear();
        mvpNeighbourKeyFrames.clear();

        mbBad = true;
    }


    //mpMap->EraseKeyFrame(this);
    //mpKeyFrameDB->erase(this);
}

void KeyFrameState::EraseConnection(shared_ptr<KeyFrameState> pKF)
{
    bool bUpdate = false;
    {

        if(mKeyFrameWeightsMap.count(pKF))
        {
            mKeyFrameWeightsMap.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

bool KeyFrameState::isPainted() {
    return mpFrame->isPainted();
}

void KeyFrameState::setPainted(bool _b) {
    mpFrame->setPainted(_b);
}
