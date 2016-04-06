//
// Created by Sen Yu on 3/25/16.
// Copyright (c) 2016 Sen Yu. All rights reserved.
//

#include "MapPoint.hpp"
#include "ORBmatcher.h"

int MapPoint::_counterId = 1;

MapPoint::MapPoint(cv::Mat _matMapPointPos, shared_ptr<KeyFrameState> _pKFS, int nP, Map *_pMap) :
    mpRefKF(_pKFS),
    mbBad(false),
    mpMap(_pMap),
    mnVisible(0),
    mnFound(0)
{
    mPos = _matMapPointPos.clone();
    mIdFromKeyFrame = nP;
    mId = _counterId++;
}

int MapPoint::getUID() {
    return mId;
}

void MapPoint::UpdateNormalAndDepth()
{
    std::map<shared_ptr<KeyFrameState>,int> observations;
    shared_ptr<KeyFrameState> pRefKF;
    cv::Mat Pos;
    {

        if(mbBad)
            return;
        observations = msKeyFrame2FeatureId;
        pRefKF=mpRefKF;
        Pos = mPos.clone();
    }

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32FC1);
    int n=0;
    for(std::map<shared_ptr<KeyFrameState>,int>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        shared_ptr<KeyFrameState> pKF = mit->first;
        cv::Mat Owi = pKF->getMatO2w();
        cv::Mat normali = mPos - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    }

    cv::Mat PC = Pos - pRefKF->getMatO2w();
    const float dist = cv::norm(PC);
    const int level = pRefKF->GetKeyPointScaleLevel(observations[pRefKF]);
    const float scaleFactor = Config::dScaleFactor;//pRefKF->GetScaleFactor();
    const float levelScaleFactor =  Config::vScaleFactors[level];//pRefKF->GetScaleFactor(level);
    const int nLevels = Config::dScaleLevel;//pRefKF->GetScaleLevels();

    {

        mfMinDistance = (1.0f/scaleFactor)*dist / levelScaleFactor;
        //mfMaxDistance = scaleFactor*dist * pRefKF->GetScaleFactor(nLevels-1-level);
        mfMaxDistance = scaleFactor*dist * Config::vScaleFactors[nLevels-1-level];
        mNormalVector = normal/n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    return mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    return mfMaxDistance;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    std::map<shared_ptr<KeyFrameState>,int> observations;

    {

        if(mbBad)
            return;
        observations=msKeyFrame2FeatureId;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(std::map<shared_ptr<KeyFrameState>,int>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        shared_ptr<KeyFrameState> pKF = mit->first;

        if(true /*!pKF->isBad()*/ )
            vDescriptors.push_back(pKF->GetDescriptor(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORB_SLAM::ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}




void MapPoint::Replace(shared_ptr<MapPoint> pMP)
{
    if(pMP->getUID()==this->getUID())
        return;

    std::map<shared_ptr<KeyFrameState>,int> obs;
    {

        obs=msKeyFrame2FeatureId;
        msKeyFrame2FeatureId.clear();
        mbBad=true;
    }

    for(std::map<shared_ptr<KeyFrameState>,int>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        shared_ptr<KeyFrameState> pKF = mit->first;

        //if(!pMP->IsInKeyFrame(pKF))
        if(!pMP->msKeyFrame2FeatureId.count(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->setKeyFrame(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }

    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(shared_from_this());

}

void MapPoint::SetBadFlag()
{
    std::map<shared_ptr<KeyFrameState>,int> obs;
    {
        mbBad=true;
        obs = msKeyFrame2FeatureId;
        msKeyFrame2FeatureId.clear();
    }
    for(std::map<shared_ptr<KeyFrameState>,int>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        shared_ptr<KeyFrameState> pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }

    mpMap->EraseMapPoint(shared_from_this());
}


void MapPoint::EraseObservation(shared_ptr<KeyFrameState> pKF)
{
    bool bBad=false;
    {

        if(msKeyFrame2FeatureId.count(pKF))
        {
            msKeyFrame2FeatureId.erase(pKF);

            if(mpRefKF==pKF)
                mpRefKF=msKeyFrame2FeatureId.begin()->first;

            // If only 2 observations or less, discard point
            if(msKeyFrame2FeatureId.size()<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}