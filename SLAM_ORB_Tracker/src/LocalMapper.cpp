//
//  LocalMapper.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "LocalMapper.hpp"

LocalMapper::LocalMapper(Map *_pMap) :
    mpCurKeyFrame( shared_ptr<KeyFrameState>(NULL) ),
    mpMap(_pMap)
{

}

void LocalMapper::addKeyFrame(shared_ptr<KeyFrameState> pKeyFrameState) {
    mBuffer.put(pKeyFrameState);
}

void LocalMapper::createNewMapPoint() {

}

int LocalMapper::processKeyFrameLoop() {

    // detect whether new KeyFrame inserted
    if ( mBuffer.hasNext() == false ) return -1;

    // take it
    // compute BoW
    mpCurKeyFrame = mBuffer.take();
    mpCurKeyFrame->getBoW();

    // add (tracked MapPoint is in this KeyFrame) to Map
    vector<shared_ptr<MapPoint>> vpMapPointMatch = mpCurKeyFrame->GetMapPointMatch();
    //int numMatch = 0;
    if( true ) //This operations are already done in the tracking for the first two keyframes
    {
        for(size_t i=0; i<vpMapPointMatch.size(); i++)
        {
            shared_ptr<MapPoint> pMP = vpMapPointMatch[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    //numMatch ++;
                    pMP->setKeyFrame(mpCurKeyFrame, i);
                    //pMP->AddObservation(mpCurrentKeyFrame, i);
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
            }
        }
    }
    if(mpCurKeyFrame->mId==1)
    {
        for(size_t i=0; i<vpMapPointMatch.size(); i++)
        {
            shared_ptr<MapPoint> pMP = vpMapPointMatch[i];
            if(pMP)
            {
                mlpRecentAddedMapPoints.push_back(pMP);
            }
        }
    }
    //std::cout<<"numMatch "<<numMatch<<std::endl;
    // update Connection
    mpCurKeyFrame->UpdateConnections();

    // add KeyFrame to Map
    mpMap->insertKeyFrame(mpCurKeyFrame);


    // if initStep2 ,pre-build RecentMapPointSet ; it should be built in next step


    // erase Bad MapPoint
    MapPointCulling();

    // according to computed mT2w, searchForTriangle with neighbour KeyFrame
    int numMatch = triangleNewMapPoint();
    std::cout<<"triangle mapPoint size"<< numMatch<<std::endl;

    // TODO: process duplicated MapPoint
    removeDuplicatedMapPoint();

    Config::time("LocalBundleAdjustment");
    Optimizer::LocalBundleAdjustment(mpCurKeyFrame,false);
    Config::timeEnd("LocalBundleAdjustment");

    mpCurKeyFrame = shared_ptr<KeyFrameState>(NULL);

    return 0;
}

int LocalMapper::run() {
    while(true) processKeyFrameLoop();
    return 0;
}


int LocalMapper::triangleNewMapPoint() {

    // Take neighbor keyframes in covisibility graph
    std::vector<shared_ptr<KeyFrameState>> vpNeighKFs = mpCurKeyFrame->GetBestCovisibilityKeyFrames(20);

    ORB_SLAM::ORBmatcher matcher(0.6,false);

    cv::Mat Rcw1 = mpCurKeyFrame->getMatR2w();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurKeyFrame->getMatt2w();
    cv::Mat Tcw1(3,4,CV_64FC1);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = mpCurKeyFrame->getMatO2w();

    const float fx1 = Config::dFx;
    const float fy1 = Config::dFy;
    const float cx1 = Config::dCx;
    const float cy1 = Config::dCy;
    const float invfx1 = 1.0f/fx1;
    const float invfy1 = 1.0f/fy1;

    const float ratioFactor = 1.5f* 1.2;//mpCurKeyFrame->GetScaleFactor();

    // Search matches with epipolar restriction and triangulate
    int numMatchMP = 0;
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        shared_ptr<KeyFrameState> pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        // Small translation errors for short baseline keyframes make scale to diverge
        cv::Mat Ow2 = pKF2->getMatO2w();//->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);
        const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
        const float ratioBaselineDepth = baseline/medianDepthKF2;

        if(ratioBaselineDepth<0.01)
            continue;

        // Compute Fundamental Matrix
        cv::Mat F12 = computeF12(mpCurKeyFrame,pKF2);

//        std::cout<<"F12 "<<F12<<std::endl;

        // Search matches that fulfil epipolar constraint
        std::vector<cv::KeyPoint> vMatchedKeysUn1;
        std::vector<cv::KeyPoint> vMatchedKeysUn2;
        std::vector<std::pair<size_t,size_t> > vMatchedIndices;
        int numMatch = matcher.SearchForTriangulation(mpCurKeyFrame,pKF2,F12,vMatchedKeysUn1,vMatchedKeysUn2,vMatchedIndices);

        cv::Mat Rcw2 = pKF2->getMatR2w();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->getMatt2w();
        cv::Mat Tcw2(3,4,CV_64FC1);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

//        std::cout<<"Rcw2"<<Rcw2<<std::endl;
//        std::cout<<"tcw2"<<tcw2<<std::endl;
//        std::cout<<"Tcw2"<<Tcw2<<std::endl;

        const float fx2 = Config::dFx;
        const float fy2 = Config::dFy;
        const float cx2 = Config::dCx;
        const float cy2 = Config::dCy;
        const float invfx2 = 1.0f/fx2;
        const float invfy2 = 1.0f/fy2;

        // Triangulate each match
        //std::cout<<"triangulate each size"<<vMatchedKeysUn1.size()<<std::endl;
        std::vector<int> cnt(10,0);
        for(size_t ikp=0, iendkp=vMatchedKeysUn1.size(); ikp<iendkp; ikp++)
        {
            const int idx1 = vMatchedIndices[ikp].first;
            const int idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = vMatchedKeysUn1[ikp];
            const cv::KeyPoint &kp2 = vMatchedKeysUn2[ikp];

            // Check parallax between rays
            // TODO: rewrite float to double in cv::Mat
            cv::Mat_<double>  xn1(3,1);
            cv::Mat_<double>  xn2(3,1);


            xn1 << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0f;
            xn2 << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0f;
            //cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0 );
            cv::Mat ray1 = Rwc1*xn1;
            //cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0 );
            cv::Mat ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            if(cosParallaxRays<0 || cosParallaxRays>0.9998) {
                cnt[0]++;
                continue;
            }


            // Linear Triangulation Method
            //cv::Mat A(4,4,CV_32F);
            cv::Mat A(4,4,CV_64FC1);
            A.row(0) = xn1.at<double>(0)*Tcw1.row(2)-Tcw1.row(0);
            A.row(1) = xn1.at<double>(1)*Tcw1.row(2)-Tcw1.row(1);
            A.row(2) = xn2.at<double>(0)*Tcw2.row(2)-Tcw2.row(0);
            A.row(3) = xn2.at<double>(1)*Tcw2.row(2)-Tcw2.row(1);

            cv::Mat w,u,vt;
            cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

            cv::Mat x3D = vt.row(3).t();


            if(x3D.at<double>(3)==0){
                cnt[1]++;
                continue;
            }

            // Euclidean coordinates
            x3D = x3D.rowRange(0,3)/x3D.at<double>(3);
            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<double>(2);
            if(z1<=0){
                cnt[2]++;
                continue;
            }

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<double>(2);
            if(z2<=0){
                cnt[3]++;
                continue;
            }

            //Check reprojection error in first keyframe

            float sigmaSquare1 = Config::vLevelSigma2[kp1.octave];
            float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<double>(0);
            float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<double>(1);
            float invz1 = 1.0/z1;
            float u1 = fx1*x1*invz1+cx1;
            float v1 = fy1*y1*invz1+cy1;
            float errX1 = u1 - kp1.pt.x;
            float errY1 = v1 - kp1.pt.y;
            if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1){
                cnt[4]++;
                continue;
            }

            //Check reprojection error in second keyframe
            float sigmaSquare2 = Config::vLevelSigma2[kp2.octave];
            float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<double>(0);
            float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<double>(1);
            float invz2 = 1.0/z2;
            float u2 = fx2*x2*invz2+cx2;
            float v2 = fy2*y2*invz2+cy2;
            float errX2 = u2 - kp2.pt.x;
            float errY2 = v2 - kp2.pt.y;
            if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2){
                cnt[5]++;
                continue;
            }

            //Check scale consistency
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0){
                cnt[6]++;
                continue;
            }

            float ratioDist = dist1/dist2;
            float ratioOctave = Config::vScaleFactors[kp1.octave] / Config::vScaleFactors[kp2.octave];//mpCurKeyFrame->GetScaleFactor(kp1.octave)/pKF2->GetScaleFactor(kp2.octave);
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor){
                cnt[7]++;
                continue;
            }

            // Triangulation is succesfull
            shared_ptr<MapPoint> pMP = shared_ptr<MapPoint>(new MapPoint(x3D, mpCurKeyFrame, idx1, mpMap));

            pMP->setKeyFrame(pKF2, idx2);
            pMP->setKeyFrame(mpCurKeyFrame, idx1);
            ////pMP->AddObservation(pKF2,idx2);
            ////pMP->AddObservation(mpCurrentKeyFrame,idx1);

            mpCurKeyFrame->insertMapPoint(pMP,idx1);
            pKF2->insertMapPoint(pMP,idx2);

            //mpCurKeyFrame->mpFrame->insertMapPoint(pMP,idx1);
            //pKF2->mpFrame->insertMapPoint(pMP,idx2);
            ////mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            ////pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->insertMapPoint(pMP);
            ////mpMap->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);
            numMatchMP ++;
            cnt[8]++;
        }

//        for(int i=0;i<=8;i++) {
//            std::cout<<"["<<i<<"] "<<cnt[i]<<std::endl;
//        }
    }
    return numMatchMP;
}


cv::Mat LocalMapper::computeF12(shared_ptr<KeyFrameState> pKF1, shared_ptr<KeyFrameState> pKF2)
{
    cv::Mat R1w = pKF1->getMatR2w();
    cv::Mat t1w = pKF1->getMatt2w();
    cv::Mat R2w = pKF2->getMatR2w();
    cv::Mat t2w = pKF2->getMatt2w();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = Utils::convectToSymmetricMatrix(t12);
    //std::cout<<"t12"<<t12<<std::endl;
    //std::cout<<"t12x"<<t12x<<std::endl;

    cv::Mat K1 = Config::mCameraParameter;
    cv::Mat K2 = Config::mCameraParameter;


    return K1.t().inv()*t12x*R12*K2.inv();
}


int LocalMapper::removeDuplicatedMapPoint()
{
    // Retrieve neighbor keyframes
    vector<shared_ptr<KeyFrameState>> vpNeighKFs = mpCurKeyFrame->GetBestCovisibilityKeyFrames(20);
    vector<shared_ptr<KeyFrameState>> vpTargetKFs;
    for(vector<shared_ptr<KeyFrameState>>::iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        shared_ptr<KeyFrameState> pKFi = *vit;
        if(/*pKFi->isBad() ||*/ pKFi->mnFuseTargetForKF == mpCurKeyFrame->mId)
        //    continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurKeyFrame->mId;

        // Extend to some second neighbors
        vector<shared_ptr<KeyFrameState>> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<shared_ptr<KeyFrameState>>::iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            shared_ptr<KeyFrameState> pKFi2 = *vit2;
            if(/*pKFi2->isBad() ||*/ pKFi2->mnFuseTargetForKF==mpCurKeyFrame->mId || pKFi2->mId==mpCurKeyFrame->mId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }


    // Search matches by projection from current KF in target KFs
    ORB_SLAM::ORBmatcher matcher(0.7);
    vector<shared_ptr<MapPoint>> vpMapPointMatches = mpCurKeyFrame->GetMapPointMatch();
    for(vector<shared_ptr<KeyFrameState>>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        shared_ptr<KeyFrameState> pKFi = *vit;

        matcher.Fuse(pKFi,vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    vector<shared_ptr<MapPoint>> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<shared_ptr<KeyFrameState>>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        shared_ptr<KeyFrameState> pKFi = *vitKF;

        vector<shared_ptr<MapPoint>> vpMapPointsKFi = pKFi->GetMapPointMatch();

        for(vector<shared_ptr<MapPoint>>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            shared_ptr<MapPoint> pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurKeyFrame->mpFrame->mId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurKeyFrame->mpFrame->mId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurKeyFrame,vpFuseCandidates);


    // Update points
    vpMapPointMatches = mpCurKeyFrame->GetMapPointMatch();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        shared_ptr<MapPoint> pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurKeyFrame->UpdateConnections();

    return 0;
}


void LocalMapper::MapPointCulling()
{
    // Check Recent Added MapPoints
    std::list<shared_ptr<MapPoint>>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurKeyFrame->mId;
    while(lit!=mlpRecentAddedMapPoints.end())
    {
        shared_ptr<MapPoint> pMP = *lit;
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
//        else if(pMP->GetFoundRatio()<0.25f )
//        {
//            pMP->SetBadFlag();
//            lit = mlpRecentAddedMapPoints.erase(lit);
//        }
//        else if((nCurrentKFid-pMP->mIdFromKeyFrame)>=2 && pMP->msKeyFrame2FeatureId.size()<=2)
//        {
//
//            pMP->SetBadFlag();
//            lit = mlpRecentAddedMapPoints.erase(lit);
//        }
        else if((nCurrentKFid-pMP->mIdFromKeyFrame)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}