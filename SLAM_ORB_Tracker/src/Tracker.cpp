//
//  Tracker.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "Tracker.hpp"
#include "Initializer.h"
Tracker::Tracker(InputBuffer *_pIB, FrameDrawer *_pFD, MapDrawer *_pMD, Vocabulary *_pVocabulary, Map *_pMap, LocalMapper* _pLocalMapper) :
    mpInputBuffer(_pIB), mpFrameDrawer(_pFD), mpMapDrawer(_pMD),
    meMode(Tracker::WorkMode::InitStep0), meLastMode(Tracker::WorkMode::Start),
    mpIniter(NULL),mpVocabulary(_pVocabulary), mpMap(_pMap),
    mpLocalMapper(_pLocalMapper),

    cache_mbLMP(false),
    cache_mtLMP(10)

{
    
}

Tracker::~Tracker() {
    
}

int Tracker::run() {
    while(true) threadRun();

    return 0;
}

int Tracker::threadRun() {

    if ( meMode == WorkMode::Fail ) {
        return -1;
    }

    meLastMode = meMode;

    // this get method is sync.
    mpCurFrame = mpInputBuffer->get();
    mpCurFrame->mnTrackedType = 0;


    if ( meMode == WorkMode::InitStep0 ) {
        // init first keyFrame
        // init first camera pose
        mpCurFrame->extractInit();
        //mpCurFrame->extract();


        initStepFirstKeyFrame();
        initPose();
        updateDrawer();
    }
    else if ( meMode == WorkMode::InitStep1 ) {
        // obtain second keyframe to build
        mpCurFrame->extractInit();
        //mpCurFrame->extract();
        bool st = initStepSecondKeyFrame();
        if ( st ) {
            mpPreFrame = shared_ptr<FrameState>( mpCurFrame );
            //mpRefFrame = shared_ptr<FrameState>( mpCurFrame );
            updateDrawer();
        }

    }
    // inited
    else {
        mpCurFrame->extractNormal();

        bool bStatus = false;
        //MotionState motion;

        // 1. get camera pose
        if ( meMode == WorkMode::Normal ) {
            // tracking
            std::cout<< "normal at" << mpCurFrame->mId << std::endl;

            Config::time("TrackMotion");
            bStatus = TrackMotion();
            Config::timeEnd("TrackMotion");
            if ( bStatus ) mpCurFrame->mnTrackedType = 1;



            std::cout<<"trackMotion Status "<<bStatus<<std::endl;
            Config::time("TrackFromPreFrame");
            if ( bStatus == false ) {

                bStatus = TrackFromPreFrame();
                if ( bStatus ) mpCurFrame->mnTrackedType = 2;
                std::cout<<"trackPre Status "<<bStatus<<std::endl;
            }
            Config::timeEnd("TrackFromPreFrame");

            if ( bStatus ) {


                bStatus = TrackLocalMap();
                if ( bStatus == false ) {
                    printf("TrackMap %d\n", bStatus);
                }


                // TODO: how to insert KeyFrame
                Config::time("localMapper");
                int localMapStep = 2;

                bool bNeedKeyFrame = false;
                // TODO: optimize parameter for checkNewKeyFrame
                {
                    int nRefMatches = 0;
                    int N = mpReferenceKF->GetMapPointMatch().size();
                    for(int i=0; i<N; i++)
                    {
                        if(mpReferenceKF->GetMapPoint(i) )
                            nRefMatches++;
                    }
                    bNeedKeyFrame = mnMatchesInliers<nRefMatches*0.6 || mnMatchesInliers < 90 ;
                    bNeedKeyFrame = bNeedKeyFrame && mnMatchesInliers < 200;
                    bNeedKeyFrame = bNeedKeyFrame || mpCurFrame->mId < 5;
                    bNeedKeyFrame = bNeedKeyFrame || mpCurFrame->mId - mLastMapperId > 10;
                    // TODO:  should not > 100 ,if > 100, continue;
                    printf("========need %d %f\n",mnMatchesInliers, nRefMatches*0.6);
                }


                if( bNeedKeyFrame /*mpCurFrame->mId - mLastMapperId >= localMapStep*/ /*NeedNewKeyFrame()*/) {
                    bool bBundleAdjustment = true;

                    createKeyFrame();
                    mpLocalMapper->processKeyFrameLoop(bBundleAdjustment);

                    //if ( bBundleAdjustment )
                    mLastBundleAdjustmentId = bBundleAdjustment;
                    mLastMapperId = mpCurFrame->mId;

                    for(size_t i=0; i<mpCurFrame->mvbOutlier.size();i++)
                    {
                        if(mpCurFrame->mvpMapPoint[i] && mpCurFrame->mvbOutlier[i])
                            mpCurFrame->mvpMapPoint[i]=shared_ptr<MapPoint>(NULL);
                    }

                }
                Config::timeEnd("localMapper");

                if ( mpPreFrame->mT2w.empty() == false )
                {
                    cv::Mat LastRwc = mpPreFrame->mT2w.rowRange(0,3).colRange(0,3).t();
                    cv::Mat Lasttwc = -LastRwc*mpPreFrame->mT2w.rowRange(0,3).col(3) ;
                    cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                    LastRwc.copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                    Lasttwc.copyTo(LastTwc.rowRange(0,3).col(3));
                    mVelocity = mpCurFrame->mT2w*LastTwc;
                    //mVelocity.rowRange(0,3).col(3) *= 0.8f;
                    std::cout<<"Velocity"<< mVelocity<<std::endl;
                }

                Config::time("updateDrawer");
                updateDrawer();
                Config::timeEnd("updateDrawer");

                // update velocity



                mpPreFrame = shared_ptr<FrameState>( mpCurFrame );
            }
            else {

                meMode = WorkMode::Fail;
                return -1;
                // should reset to relocation
            }





            //int numMatch = match(mpPreFrame, mpCurFrame, mvMatchPair12, mvMatchMask12, mvMatchPoint);
            //std::cout<< "match: " << numMatch << std::endl;
            //numMatch = filerByOpticalFlow(mpPreFrame, mpCurFrame, mvMatchPoint);
            //std::cout<< "match: " << numMatch << std::endl;
            //bStatus = computeMotion(mpPreFrame, mpCurFrame, mvMatchPoint, motion, mvMatchMask12);
            //std::cout<< "motion " << bStatus << " "<<motion << std::endl;

            //drawFilter(mpCurFrame, mvMatchPoint[1]);
            //bStatus = getMotion(motion);
        }
        else {
            // relocalise
        }

        // 2. build map track and map point


        // 3. insert keyframe


        // 4. if any failed
//        if ( bStatus == false ) {
//            //meMode = WorkMode::Fail;
//            //updateDrawer();
//            return -1;
//        }

    }





    // clone current to previous
    //mpPreFrame = shared_ptr<FrameState>( mpCurFrame );
    //meLastMode = meMode;
    return 0;
}


void Tracker::updateDrawer() {
    mpFrameDrawer->update(mpCurFrame);

    std::cout<<"update MapDrawer at"<<mpCurFrame->mId<<std::endl<<mpCurFrame->mT2w<<std::endl;
    mpMapDrawer->update(mpCurFrame);
}

void Tracker::initPose() {

    mCurPose = PoseState(mpCurFrame->mId);
    mCurPose.mDir = Const::pnt3d_001;
    mCurPose.mPos = Const::pnt3d_000;
    mCurPose.mDir3 = Const::mat33_111.clone();

    mpIniFrame->updatePose( Const::mat44_1111.clone() );
    //mCurPose.mDir3.copyTo( mpIniFrame->mT2w.rowRange(0,3).colRange(0,3) );
    //mCurPose.mPos.copyTo( mpIniFrame->mT2w.rowRange(0,3).col(3) );
}


int Tracker::match(shared_ptr<FrameState> pPreFrame, shared_ptr<FrameState> pCurFrame, std::vector<int>& vMatchPair12, std::vector<uchar>& vMatchMask12, std::vector<cv::Point2f> vMatchPoint[2]) {

    vMatchPair12.clear();
    vMatchMask12.clear();

    cv::BruteForceMatcher<cv::L2<float>> BFMatcher;
    std::vector<cv::DMatch> vBFMatches;

    int N = pPreFrame->mvKeyPoint.size();
    //暴力匹配
    BFMatcher.match(pPreFrame->mDescriptor, pCurFrame->mDescriptor, vBFMatches);
    vMatchPair12 = std::vector<int>(N, -1);
    vMatchMask12 = std::vector<uchar>(N, false);

    int numMatch = 0;//vBFMatches.size();

    vMatchPoint[0].clear();
    vMatchPoint[1].clear();

    //匹配完毕之后放到 vecPairPoint[2] 中
    for (auto& match : vBFMatches) {

        vMatchPair12[ match.queryIdx ] = match.trainIdx;
        vMatchMask12[ match.queryIdx ] = true;

        vMatchPoint[0].push_back( pPreFrame->mvKeyPoint[ match.queryIdx ].pt );
        vMatchPoint[1].push_back( pCurFrame->mvKeyPoint[ match.trainIdx ].pt );

        numMatch++;
    }

    return numMatch;
}


//bool Tracker::computeMotion(shared_ptr<FrameState> pPreFrame, shared_ptr<FrameState> pCurFrame, std::vector<cv::Point2f> *vMatchPoint, MotionState &motion, std::vector<uchar> &vMatchMask) {
//
//    //是否能够解基础矩阵(满足匹配点数大于minFundamentMatches)
//    bool _isUseFundamentalMatrix = vMatchPoint[0].size() >= 10;
//
//    motion.mvIds[0] = pPreFrame->mId;
//    motion.mvIds[1] = pCurFrame->mId;
//
//    //默认返回值
//    bool retStatus = true;
//
//
//
//
//
//    cv::Mat matFundamental;
//
//    //如果可以解基础矩阵
//    if (_isUseFundamentalMatrix) {
//
//        matFundamental = cv::findFundamentalMat(vMatchPoint[0], vMatchPoint[1], vMatchMask, CV_FM_RANSAC);
//
//        //本质矩阵计算
//        cv::Mat matE(3, 3, CV_64FC1);
//        matE = Config::mCameraParameter.t() * matFundamental* Config::mCameraParameter;
//
//        {
//            printf("FundamentalMatrix=");
//            std::cout << matFundamental << std::endl;
//            printf("EssentialMatrix=");
//            std::cout << matE << std::endl;
//            printf("CameraParameter=");
//            std::cout << Config::mCameraParameter << std::endl;
//        }
//
//        //SVD分解
//        cv::SVD svdComputer;
//        cv::Mat matU(3, 3, CV_64FC1), matS(3, 3, CV_64FC1), matVT(3, 3, CV_64FC1);
//
//        svdComputer.compute(matE, matS, matU, matVT, cv::SVD::FULL_UV);
//
//        //如果基础矩阵解挂了，考虑默认直线
//        if (cv::sum(matS)[0] < 0.1f) {
//            motion.mMatR = Const::mat33_111.clone();
//            motion.mMatT = Const::mat31_001.clone();
//            _isUseFundamentalMatrix = false;
//            retStatus = false;
//        }
//        else {
//
//            //如果 matU 或 matVT 的行列式小于0，反一下
//            if (cv::determinant(matU) < 0)
//                matU = -matU;
//            if (cv::determinant(matVT) < 0)
//                matVT = -matVT;
////            {
////                printf("SVD\n");
////                std::cout << matU << std::endl << matS << std::endl << matVT << std::endl;
////            }
//
//            cv::Mat matR[4],
//            matT[4];
//            cv::Mat matW(3, 3, CV_64FC1);
//            matW = 0.0f;
//            matW.at<double>(0, 1) = -1.0f;
//            matW.at<double>(1, 0) = 1.0f;
//            matW.at<double>(2, 2) = 1.0f;
//
//            //matR 和 matT 有四种可能解
//            int score[4];
//            int maxScore = 0, maxId = 0;
//            for (int i = 0; i < 4; i++) {
//                matR[i] = matU * (i % 2 ? matW.t() : matW) * matVT;
//                matT[i] = (i / 2 ? 1.0f : -1.0f) * matU.col(2);
//
//                matT[i] = -matR[i].inv() * matT[i];
//                matR[i] = matR[i].inv();
//
//                if (matT[i].at<double>(2, 0) < 0.0f)  matT[i] = -matT[i];
//
//                score[i] = checkRT(matR[i], matT[i], vMatchPoint, vMatchMask);
//                if ( score[i] > maxScore ) {
//                    maxId = i;
//                    maxScore = score[i];
//                }
//                printf("score[%d]=%d\n",i,score[i]);
//
//            }
//
//            //test all
//            cv::Mat Q(4, 4, CV_64FC1);
//            cv::Mat u1(4, 1, CV_64FC1), u2(4, 1, CV_64FC1);
//            cv::Mat res(1, 1, CV_64FC1);
//            double compSEL[4];
//
//            for (int i = 0; i < 4; i++) {
//                //Trick: 选择在转弯上转的最小的那个
//                cv::Mat tmp = (matR[i] * Const::mat31_100);
//                compSEL[i] = tmp.at<double>(0, 0);
//                printf("valid[%d] = %f\n", i, compSEL[i]);
//            }
//            double maxValidation = -100.0f; int selectDirectionIdx = 0;
//            for (int i = 0; i < 2; i++) {
//                if (compSEL[i] > maxValidation) {
//                    maxValidation = compSEL[i];
//                    selectDirectionIdx = i;
//                }
//            }
////
////            printf("selectDirectionIdx=%d\n", selectDirectionIdx);
//            if ( selectDirectionIdx != maxId ) {
//                printf("select %d maxId %d\n", selectDirectionIdx, maxId);
//            }
//
//            if ( maxScore == 0 ) {
//                printf("asdf");
//            }
//            motion.mMatR = matR[selectDirectionIdx].clone();
//            motion.mMatT = matT[selectDirectionIdx].clone();
//            // 要先把 match的线画出来再说,否则可能有问题
//            //不能保证 fundementalMatrix求出来就是好的
//        }
//
//    }
//    else {
//        motion.mMatR = Const::mat33_111.clone();
//        motion.mMatT = Const::mat31_001.clone();
//        retStatus = false;
//
//    }
//
//
//    return retStatus;
//}


int Tracker::filterByOpticalFlow(shared_ptr<FrameState> pPreFrame, shared_ptr<FrameState> pCurFrame, std::vector<int>& vMatchPair12, std::vector<uchar>& vMatchMask12, std::vector<cv::Point2f> vMatchPoint[2]) {
    double threshold = Config::dOpticalFlowThreshold;

    // 光流运算匹配
    std::vector<cv::Point2f> vFound;
    std::vector<uchar> vStatus;
    std::vector<float> vErr;

//    printf("%d %d %d %d\n", pPreFrame->mImage.rows,
//        pCurFrame->mImage.rows,
//        vMatchPoint[0].size(), vMatchPoint[1].size());

    cv::calcOpticalFlowPyrLK(
            pPreFrame->mImage, pCurFrame->mImage,
            vMatchPoint[0], vFound,
            vStatus, vErr);

    int N = pPreFrame->mvKeyPoint.size();
    vMatchPoint[0].clear();
    vMatchPoint[1].clear();
    for(int ni = 0 ;ni!= N; ni++) {
        if ( vMatchMask12[ni] == false ) continue;
        cv::Point2f
            &ptPre = vMatchPoint[0][ni],
            &ptCur = vMatchPoint[1][ni],
            &ptFound = vFound[ni];

        double dx = fabs(ptCur.x - ptFound.x),
                dy = fabs(ptCur.y - ptFound.y);

        if ( dx*dx+dy*dy > threshold || false == vStatus[ni]) {
            vMatchMask12[ni] = false;
            vMatchPair12[ni] = -1;
        }
        else {
            vMatchPoint[0].push_back( ptPre );
            vMatchPoint[1].push_back( ptCur );
        }

    }

    return vMatchPoint[0].size();
}

//void Tracker::drawFilter(shared_ptr<FrameState> pFrame, std::vector<cv::Point2f> &mvPoint) {
//
//    for(int i=0;i<mvPoint.size();i++) {
//        cv::rectangle(pFrame->mImage, mvPoint[i] - cv::Point2f(0.5,0.5), mvPoint[i] + cv::Point2f(0.5,0.5), cv::Scalar(0,0,255));
//    }
//
//}

void Tracker::initStepFirstKeyFrame() {
    std::cout<< "init0 at" << mpCurFrame->mId << " "<<mpCurFrame->mvKeyPoint.size()<<std::endl;
    if ( mpCurFrame->mvKeyPoint.size() < 100 ) return;

    mpIniFrame = shared_ptr<FrameState>( mpCurFrame );

    //mvIniMatched
    mvIniMatched.clear();
    for(int i=0;i<mpIniFrame->mvKeyPoint.size();i++) {
        mvIniMatched.push_back( mpIniFrame->mvKeyPoint[i].pt );
    }

    if ( mpIniter ) delete mpIniter;
    mpIniter = new ORB_SLAM::Initializer(mpIniFrame,1.0,200);

    meMode = WorkMode::InitStep1;


}

//void Tracker::initStepSecondKeyFrmae_BAK() {
//    if ( mpCurFrame->mvKeyPoint.size() < 100 ) {
//        meMode = WorkMode::InitStep0;
//        return;
//    }
//
//    // getMatch
//    int numMatch;
//
//    numMatch = match(mpPreFrame, mpCurFrame, mvMatchPair12, mvMatchMask12, mvMatchPoint );
//    if (numMatch < 100) {
//        meMode = WorkMode::InitStep0;
//        return;
//    }
//
//    numMatch = filerByOpticalFlow(mpPreFrame, mpCurFrame, mvMatchPoint);
//    if (numMatch < 100) {
//        meMode = WorkMode::InitStep0;
//        return;
//    }
//
//    cv::Mat Rcw; // Current Camera Rotation
//    cv::Mat tcw; // Current Camera Translation
//    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
//    vector<cv::Point3f> vP3D;
//    MotionState motion;
//    if ( mpIniter->Initialize(mpCurFrame, mvMatchPair12, Rcw, tcw, vP3D, vbTriangulated)) {
//        for(size_t i=0, iend=mvMatchPair12.size(); i<iend;i++)
//        {
//            if(mvMatchPair12[i]>=0 && !vbTriangulated[i])
//            {
//                mvMatchPair12[i]=-1;
//                numMatch--;
//            }
//        }
//
//        motion.mvIds[0] = mpPreFrame->mId;
//        motion.mvIds[1] = mpCurFrame->mId;
//        motion.mMatR = Rcw.clone();
//        motion.mMatT = tcw.clone();
//        meMode = WorkMode::Normal;
//    }
//
//    return ;
///*   void Tracking::Initialize()
//    {
//        // Check if current frame has enough keypoints, otherwise reset initialization process
//        if(mCurrentFrame.mvKeys.size()<=100)
//        {
//            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
//            mState = NOT_INITIALIZED;
//            return;
//        }
//
//        // Find correspondences
//        ORBmatcher matcher(0.9,true);
//        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);
//
//        // Check if there are enough correspondences
//        if(nmatches<100)
//        {
//            mState = NOT_INITIALIZED;
//            return;
//        }
//
//        cv::Mat Rcw; // Current Camera Rotation
//        cv::Mat tcw; // Current Camera Translation
//        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
//
//        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
//        {
//            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
//            {
//                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
//                {
//                    mvIniMatches[i]=-1;
//                    nmatches--;
//                }
//            }
//
//            CreateInitialMap(Rcw,tcw);
//        }
//
//    }*/
//
//}

//int Tracker::checkRT(cv::Mat matR, cv::Mat matT, std::vector<cv::Point2f> *vMatchPoint, std::vector<uchar> &vMatchMask) {
//
//    int N = vMatchPoint[0].size();
//
//    cv::Mat matProj0(3,4,CV_64FC1,cv::Scalar(0));
//    Config::mCameraParameter.copyTo(matProj0.rowRange(0,3).colRange(0,3));
//
//    cv::Mat matProj1(3,4,CV_64FC1);
//    matR.copyTo(matProj1.rowRange(0,3).colRange(0,3));
//    matT.copyTo(matProj1.rowRange(0,3).col(3));
//
//
//    matProj1 = Config::mCameraParameter * matProj1;
//
//    cv::Mat matOrigin0 = cv::Mat::zeros(3,1,CV_64FC1);
//    cv::Mat matOrigin1 = -matR.t()*matT;
//
//
//    const double fx = Config::dFx;
//    const double fy = Config::dFy;
//    const double cx = Config::dCx;
//    const double cy = Config::dCy;
//
//    double th2 = 25.0f;
//
//    int numMatch = 0;
//
//    if ( mpCurFrame->mId >= 58 ) {
//        std::cout<<" "<<std::endl;
//    }
//
//    for(int i=0;i<N;i++) {
//        if ( vMatchMask[i] == true ) {
//
//            cv::Point2f &p0 = vMatchPoint[0][i],
//                        &p1 = vMatchPoint[1][i];
//
//            cv::Mat triPnt0(3,1,CV_64FC1);
//            triangulateDLT( p0, p1, matProj0, matProj1, triPnt0 );
//
//
//            cv::Mat triPnt1 = matR*triPnt0+matT;
//
//            if (! fabs(triPnt0.at<double>(0))  > 1e7 || !fabs(triPnt0.at<double>(1))  > 1e7 || !fabs(triPnt0.at<double>(2))  > 1e7 ) {
//                continue;
//            }
//
//            cv::Mat normal0 = triPnt0 - matOrigin0;
//            cv::Mat normal1 = triPnt0 - matOrigin1;
//
//            double cosParallax = normal0.dot(normal1)/(cv::norm(normal0)*cv::norm(normal1));
//
//            if(triPnt0.at<double>(2)<=0 && cosParallax < 0.99998)
//                continue;
//            if(triPnt1.at<double>(2)<=0 && cosParallax < 0.99998)
//                continue;
//
//            // Check reprojection error in first image
//            double im1x, im1y;
//            double invZ1 = 1.0/triPnt0.at<double>(2);
//            im1x = fx*triPnt0.at<double>(0)*invZ1+cx;
//            im1y = fy*triPnt0.at<double>(1)*invZ1+cy;
//
//            double squareError1 = (im1x-p0.x)*(im1x-p0.x)+(im1y-p0.y)*(im1y-p0.y);
//
//            if(squareError1>th2)
//                continue;
//
//            // Check reprojection error in second image
//            double im2x, im2y;
//            double invZ2 = 1.0/triPnt1.at<double>(2);
//            im2x = fx*triPnt1.at<double>(0)*invZ2+cx;
//            im2y = fy*triPnt1.at<double>(1)*invZ2+cy;
//
//            double squareError2 = (im2x-p1.x)*(im2x-p1.x)+(im2y-p1.y)*(im2y-p1.y);
//
//            if(squareError2>th2)
//                continue;
//
//            numMatch++;
//        }
//    }
//
//
//    return numMatch;
//}

//void Tracker::triangulateDLT(cv::Point2f &xl,cv::Point2f &xr,cv::Mat &Pl,cv::Mat &Pr, cv::Mat &point3d)
//{
//    cv::Mat design(4,4,CV_64FC1);
//    for (int i = 0; i < 4; ++i)
//    {
//        design.at<double>(0,i) = xl.x * Pl.at<double>(2,i) - Pl.at<double>(0,i);
//        design.at<double>(1,i) = xl.y * Pl.at<double>(2,i) - Pl.at<double>(1,i);
//        design.at<double>(2,i) = xr.x * Pr.at<double>(2,i) - Pr.at<double>(0,i);
//        design.at<double>(3,i) = xr.y * Pr.at<double>(2,i) - Pr.at<double>(1,i);
//    }
//
//    cv::Mat u,w,vt;
//    cv::SVD::compute(design,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
//    cv::Mat point4d = vt.row(3).t();
//    point4d = point4d/point4d.at<double>(3);
//    point3d.at<double>(0) = point4d.at<double>(0);
//    point3d.at<double>(1) = point4d.at<double>(1);
//    point3d.at<double>(2) = point4d.at<double>(2);
//}

bool Tracker::initStepSecondKeyFrame() {
    bool bStatus = false;
    std::cout<< "init1 try at" << mpIniFrame->mId<<"-"<<mpCurFrame->mId <<" "<<mpCurFrame->mvKeyPoint.size()<< std::endl;
    mMotion.mvIds[0] = mpIniFrame->mId;
    mMotion.mvIds[1] = mpCurFrame->mId;

    if ( mpCurFrame->mvKeyPoint.size() < 100 ) {
        meMode = WorkMode::InitStep0;
        return false;
    }

    // getMatch
    int numMatch;
    ORB_SLAM::ORBmatcher matcher(0.9,true);
    //  TODO: is orbSLAM 's matcher good enough?
    numMatch = matcher.SearchForInitialization(mpIniFrame,mpCurFrame,mvIniMatched,mvMatchPair12,100);

    //numMatch = match(mpIniFrame, mpCurFrame, mvMatchPair12, mvMatchMask12, mvMatchPoint );

    std::cout<<"init1 match "<<numMatch<<std::endl;
    if (numMatch < 100) {
        meMode = WorkMode::InitStep0;
        return false;
    }

    //不修改 KeyPoint的数量, 只修改是否可用
    //numMatch = filterByOpticalFlow(mpIniFrame, mpCurFrame, mvMatchPair12, mvMatchMask12, mvMatchPoint );

    //std::cout<<"init1 flow "<<numMatch<<std::endl;
    if (numMatch < 100) {
        meMode = WorkMode::InitStep0;
        return false;
    }

    cv::Mat Rcw; // Current Camera Rotation
    cv::Mat tcw; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
    vector<cv::Point3f> vP3D;
    bool bInited = mpIniter->Initialize(mpCurFrame, mvMatchPair12, Rcw, tcw, vP3D, vbTriangulated);

    std::cout<<"init1 init"<<bInited<<std::endl;
    if ( bInited) {
        for(size_t i=0, iend=mvMatchPair12.size(); i<iend;i++)
        {
            if(mvMatchPair12[i]>=0 && !vbTriangulated[i])
            {
                mvMatchPair12[i]=-1;
                numMatch--;
            }

        }

        mMotion.mMatR = cv::Mat(3,3,CV_32FC1);//Rcw.clone();
        mMotion.mMatT = cv::Mat(3,1,CV_32FC1);//tcw.clone();
        //meMode = WorkMode::Normal;

        Rcw.convertTo(mMotion.mMatR, CV_32FC1 );
        tcw.convertTo(mMotion.mMatT, CV_32FC1);

        //confirm direction forward
        if ( mMotion.mMatT.at<float>(2) > 0.0f ) {
            mMotion.mMatT = - mMotion.mMatT;
        }


  //          mMotion.mMatT = -mMotion.mMatR.inv() * mMotion.mMatT;
 //           mMotion.mMatR = mMotion.mMatR.inv();

        // this is matrix which transform point in frame(0) to frame(1)
        // not the matrix transform frame0's origin to frame1's origin
        std::cout<<"init1 initMat"<<Rcw << std::endl<<tcw<<std::endl;
        std::cout<<mMotion.mMatR << std::endl<<mMotion.mMatT<<std::endl;


        bStatus = initStepBuildMap(mMotion, vP3D);

        if ( false == bStatus ) return false;

        meMode = WorkMode::Normal;
        std::cout<< "init1 done at" << mpCurFrame->mId << std::endl;
        return true;

    }

    meMode = WorkMode::InitStep1;
    return false;

}


bool Tracker::initStepBuildMap(MotionState initMotion, vector<cv::Point3f> &vP3D) {
    mpIniFrame->mT2w = Const::mat44_1111.clone();
    mpCurFrame->mT2w = Const::mat44_1111.clone();

    initMotion.mMatR.copyTo( mpCurFrame->mT2w.rowRange(0,3).colRange(0,3) );
    initMotion.mMatT.copyTo( mpCurFrame->mT2w.rowRange(0,3).col(3) );
    mpCurFrame->updatePose();

    //init KeyFrame;
    shared_ptr<KeyFrameState> pIniKeyFrame = shared_ptr<KeyFrameState>( new KeyFrameState(mpIniFrame, mpVocabulary) );
    shared_ptr<KeyFrameState> pCurKeyFrame = shared_ptr<KeyFrameState>( new KeyFrameState(mpCurFrame, mpVocabulary) );

    pIniKeyFrame->getBoW();
    pCurKeyFrame->getBoW();

    // ok
    mpMap->insertKeyFrame(pIniKeyFrame);
    mpMap->insertKeyFrame(pCurKeyFrame);

    int N = mvMatchPair12.size();
    for(int i=0;i<N;i++) {
        if ( mvMatchPair12[i] <0 ) continue;

        cv::Mat matMapPointPos = Utils::convertToCvMat31(vP3D[i]);
        // ok
        shared_ptr<MapPoint> pMapPoint = shared_ptr<MapPoint>(new MapPoint(matMapPointPos, pCurKeyFrame, i, mpMap));

        // ok
        pMapPoint->setKeyFrame(pIniKeyFrame, i);
        pMapPoint->setKeyFrame(pCurKeyFrame, mvMatchPair12[i]);


        // ok
        pIniKeyFrame->insertMapPoint(pMapPoint, i);
        pCurKeyFrame->insertMapPoint(pMapPoint, mvMatchPair12[i] );

        pMapPoint->ComputeDistinctiveDescriptors();
        pMapPoint->UpdateNormalAndDepth();

        // ok
        mpCurFrame->insertMapPoint( pMapPoint, mvMatchPair12[i] );

        // ok
        mpMap->insertMapPoint(pMapPoint);

    }

    //  should construct  the connection from this KeyFrame to other KeyFrame by covisible MapPoint
    pIniKeyFrame->UpdateConnections();
    pCurKeyFrame->UpdateConnections();
    Optimizer::GlobalBundleAdjustemnt(mpMap,5);




    // Set median depth to 1
    float medianDepth = pIniKeyFrame->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;
    std::cout<<"medianDepth"<<medianDepth<<std::endl;
    if(medianDepth<0 )
    {
        return false;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pCurKeyFrame->getMatT2w();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pCurKeyFrame->updatePose(Tc2w);

    std::cout<<"updated Tc2w"<<Tc2w<<std::endl;

    // Scale points
    std::vector<shared_ptr<MapPoint>> vpAllMapPoints = pIniKeyFrame->GetMapPointMatch();
    for(int iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            std::cout<<"MapPoint["<<iMP<<"] "<< vpAllMapPoints[iMP]->getMPos() <<std::endl;
            shared_ptr<MapPoint> pMP = vpAllMapPoints[iMP];
            pMP->setMPos(pMP->getMPos() * invMedianDepth);
        }
    }



    mpLocalMapper->addKeyFrame(pIniKeyFrame);
    mpLocalMapper->addKeyFrame(pCurKeyFrame);

    mpLocalMapper->processKeyFrameLoop(false);
    mpLocalMapper->processKeyFrameLoop(true);

    mLastMapperId = pCurKeyFrame->mpFrame->mId;
    {
        boost::mutex::scoped_lock lock(mMutexLKF);
        mvpLocalKeyFrames.push_back(pCurKeyFrame);
        mvpLocalKeyFrames.push_back(pIniKeyFrame);
    }
    {
        boost::mutex::scoped_lock lock(mMutexLMP);
        mvpLocalMapPoints = mpMap->getAllVectorMapPoint();
        cache_mbLMP = true;
    }
    mpReferenceKF = pCurKeyFrame;

    std::cout<< "Init Map's MapPoint size"<<mvpLocalMapPoints.size()<<std::endl;

    return true;
}


bool Tracker::TrackFromPreFrame() {
    ORB_SLAM::ORBmatcher matcher(0.9,true);
    std::vector<shared_ptr<MapPoint>> vpMapPoint;
    bool bStatus = false;
    int numMatch = 0;

    // TODO: match
    numMatch = matcher.WindowSearch(mpPreFrame,mpCurFrame,100,vpMapPoint,0);
    //numMatch = match(mpPreFrame, mpCurFrame, mvMatchPair12, mvMatchMask12, mvMatchPoint );


    //numMatch = filterByOpticalFlow(mpPreFrame, mpCurFrame, mvMatchPair12, mvMatchMask12, mvMatchPoint );
    //std::cout<<"TrackLast flow "<<numMatch<<std::endl;

    // mpLsatFrame has its MapPoint vector
    //std::cout<< "numMatchMapPoint " <<numMatchMapPoint<<std::endl;

    // TODO: set estimation mT2w
    mpCurFrame->updatePose( mpPreFrame->mT2w.clone() );
    mpCurFrame->mvpMapPoint = vpMapPoint;
    //std::cout<< "estimation mT2w!!! "<<mpPreFrame->mT2w<<std::endl;
    // TODO: if matches > 10  poseOptimization
//    if ( numMatch > 10 ) {
//        Optimizer::PoseOptimization(mpCurFrame);
//        for(int i =0; i<mpCurFrame->mvbOutlier.size(); i++)
//            if(mpCurFrame->mvbOutlier[i])
//            {
//                mpCurFrame->mvpMapPoint[i]=shared_ptr<MapPoint>(NULL);
//                mpCurFrame->mvbOutlier[i]=false;
//                numMatch--;
//            }
//    }
    // TODO: use normal projection match again
//    numMatch = matcher.SearchByProjection(mpPreFrame,mpCurFrame,50,vpMapPoint);
//    mpCurFrame->mvpMapPoint = vpMapPoint;
//    std::cout<<"TrackLast proj "<<numMatch<<std::endl;


    // TODO: if numMatch < 10 return
    std::cout<<"TrackLast0 match "<<numMatch<<std::endl;
    if ( numMatch < std::max(Config::iFeatureNum/60,10) ) {
        return false;
    }

    // TODO: pose Optimization
    Optimizer::PoseOptimization(mpCurFrame);

    for(int i =0; i<mpCurFrame->mvbOutlier.size(); i++) {
        if(mpCurFrame->mvbOutlier[i])
        {
            mpCurFrame->mvpMapPoint[i]=shared_ptr<MapPoint>(NULL);
            mpCurFrame->mvbOutlier[i]=false;
            numMatch--;
        }
    }
    std::cout<< "estimation mT2w "<<std::endl<<mpPreFrame->mT2w<<std::endl;
    std::cout<< "optimization mT2w "<<std::endl<<mpCurFrame->mT2w<<std::endl;

    std::cout<<"TrackLast match1 "<<numMatch<<std::endl;
    return numMatch > std::max(Config::iFeatureNum/60,10);
}

bool Tracker::UpdateLocal() {



    return false;
}

void Tracker::createKeyFrame() {
    mpLocalMapper->addKeyFrame( shared_ptr<KeyFrameState>( new KeyFrameState(mpCurFrame, mpVocabulary) ) );

}

bool Tracker::TrackMotion()
{
    ORB_SLAM::ORBmatcher matcher(0.9,true);
    vector<shared_ptr<MapPoint>> vpMapPointMatches;

    // Compute current pose by motion model
    if ( mVelocity.empty() ) return false;
    mpCurFrame->updatePose( mVelocity*mpPreFrame->mT2w );

    //fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<shared_ptr<MapPoint>>(NULL));
    for(size_t i =0; i<mpCurFrame->mvpMapPoint.size(); i++) {
        mpCurFrame->mvpMapPoint[i] = shared_ptr<MapPoint>(NULL);
    }

    // Project points seen in previous frame
    int nmatches = matcher.SearchByProjection(mpCurFrame,mpPreFrame,15);

    std::cout<<"trackMotion match0 "<<nmatches<<std::endl;
    if(nmatches<Config::iFeatureNum/40.0f)
        return false;

    // Optimize pose with all correspondences
    std::cout<< "motion estimation mT2w "<<std::endl<<mpCurFrame->mT2w<<std::endl;
    Optimizer::PoseOptimization(mpCurFrame);

    std::cout<< "motion optimization mT2w "<<std::endl<<mpCurFrame->mT2w<<std::endl;

    // Discard outliers
    for(size_t i =0; i<mpCurFrame->mvpMapPoint.size(); i++)
    {
        if(mpCurFrame->mvpMapPoint[i])
        {
            if(mpCurFrame->mvbOutlier[i])
            {
                mpCurFrame->mvpMapPoint[i]=shared_ptr<MapPoint>(NULL);
                mpCurFrame->mvbOutlier[i]=false;
                nmatches--;
            }
        }
    }
    std::cout<<"trackMotion match1 "<<nmatches<<std::endl;
    return nmatches>=Config::iFeatureNum/40.0f;
}


bool Tracker::TrackLocalMap()
{
    // Tracking from previous frame or relocalisation was succesfull and we have an estimation
    // of the camera pose and some map points tracked in the frame.
    // Update Local Map and Track

    // Update Local Map
    UpdateReferenceKeyFrames();
    UpdateReferencePoints();

    // Search Local MapPoints
    SearchReferencePointsInFrustum();

    // Optimize Pose
    mnMatchesInliers = Optimizer::PoseOptimization(mpCurFrame);

    // Update MapPoints Statistics
    for(size_t i=0; i<mpCurFrame->mvpMapPoint.size(); i++)
        if(mpCurFrame->mvpMapPoint[i])
        {
            if(!mpCurFrame->mvbOutlier[i])
                mpCurFrame->mvpMapPoint[i]->IncreaseFound();
        }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
//    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
//        return false;
//
//    if(mnMatchesInliers<30)
//        return false;
//    else
        return true;
}

void Tracker::UpdateReferenceKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    std::map<shared_ptr<KeyFrameState>,int> keyframeCounter;
    for(int i=0, iend=mpCurFrame->mvpMapPoint.size(); i<iend;i++)
    {
        if(mpCurFrame->mvpMapPoint[i])
        {
            shared_ptr<MapPoint> pMP = mpCurFrame->mvpMapPoint[i];
            if(!pMP->isBad())
            {
                std::map<shared_ptr<KeyFrameState>,int> observations = pMP->msKeyFrame2FeatureId;
                for(std::map<shared_ptr<KeyFrameState>,int>::iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mpCurFrame->mvpMapPoint[i]=shared_ptr<MapPoint>(NULL);
            }
        }
    }

    int max=0;
    shared_ptr<KeyFrameState> pKFmax= shared_ptr<KeyFrameState>(NULL);
    {
        boost::mutex::scoped_lock lock(mMutexLKF);
        mvpLocalKeyFrames.clear();
    }
    //mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(std::map<shared_ptr<KeyFrameState>,int>::iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        shared_ptr<KeyFrameState> pKF = it->first;
        if ( !pKF ) continue;
        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }
        {
            boost::mutex::scoped_lock lock(mMutexLKF);
            mvpLocalKeyFrames.push_back(it->first);
        }
        pKF->mnTrackReferenceForFrame = mpCurFrame->mId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    std::set<shared_ptr<KeyFrameState>> sKF;
    for(std::vector<shared_ptr<KeyFrameState>>::iterator itKF=mvpLocalKeyFrames.begin(); itKF!=mvpLocalKeyFrames.end(); itKF++)
    {
        // Limit the number of keyframes

        if(sKF.size()>80)
            break;

        shared_ptr<KeyFrameState> pKF = *itKF;
        if ( !pKF ) continue;

        sKF.insert(pKF);

        std::vector<shared_ptr<KeyFrameState>> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(std::vector<shared_ptr<KeyFrameState>>::iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            shared_ptr<KeyFrameState> pNeighKF = *itNeighKF;
            if ( !pNeighKF ) continue;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame != mpCurFrame->mId)
                {
                    //mvpLocalKeyFrames.push_back(pNeighKF);
                    sKF.insert( pNeighKF );
                    pNeighKF->mnTrackReferenceForFrame = mpCurFrame->mId;
                    break;
                }
            }
        }

    }
    {
        boost::mutex::scoped_lock lock(mMutexLKF);
        mvpLocalKeyFrames = std::vector<shared_ptr<KeyFrameState>>( sKF.begin(), sKF.end() );
    }

    mpReferenceKF = pKFmax;
}

void Tracker::UpdateReferencePoints()
{
    {
        boost::mutex::scoped_lock lock(mMutexLMP);
        mvpLocalMapPoints.clear();
        cache_mbLMP = true;
    }

    for(std::vector<shared_ptr<KeyFrameState>>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        shared_ptr<KeyFrameState> pKF = *itKF;
        std::vector<shared_ptr<MapPoint>> vpMPs = pKF->GetMapPointMatch();

        for(std::vector<shared_ptr<MapPoint>>::iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            shared_ptr<MapPoint> pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mpCurFrame->mId)
                continue;
            if(!pMP->isBad())
            {
                {
                    boost::mutex::scoped_lock lock(mMutexLMP);
                    mvpLocalMapPoints.push_back(pMP);
                    cache_mbLMP = true;
                }
                pMP->mnTrackReferenceForFrame=mpCurFrame->mId;
            }
        }
    }
}


void Tracker::SearchReferencePointsInFrustum()
{
    // Do not search map points already matched
    for(std::vector<shared_ptr<MapPoint>>::iterator vit=mpCurFrame->mvpMapPoint.begin(), vend=mpCurFrame->mvpMapPoint.end(); vit!=vend; vit++)
    {
        shared_ptr<MapPoint> pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit =shared_ptr<MapPoint>( NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mpCurFrame->mId;
                pMP->mbTrackInView = false;
            }
        }
    }

    mpCurFrame->updatePose();

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(std::vector<shared_ptr<MapPoint>>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        shared_ptr<MapPoint> pMP = *vit;
        if(pMP->mnLastFrameSeen == mpCurFrame->mId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mpCurFrame->isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }


    if(nToMatch>0)
    {
        ORB_SLAM::ORBmatcher matcher(0.8);
        int th = 1;
        // If the camera has been relocalised recently, perform a coarser search
//        if(mpCurFrame->mId<mnLastRelocFrameId+2)
//            th=5;
        matcher.SearchByProjection(mpCurFrame,mvpLocalMapPoints,th);
    }
}

//std::set<shared_ptr<MapPoint>> Tracker::getAllSetLocalMapPoint() {
//    boost::mutex::scoped_lock lock(mMutexLMP);
//    return std::set<shared_ptr<MapPoint>>(mvpLocalMapPoints.begin(), mvpLocalMapPoints.end());
//}

std::vector<shared_ptr<MapPoint>> Tracker::getAllVectorLocalMapPoint() {
    boost::mutex::scoped_lock lock(mMutexLMP);
    return mvpLocalMapPoints;
}
//
//std::set<shared_ptr<KeyFrameState>> Tracker::getAllSetLocalKeyFrame() {
//    boost::mutex::scoped_lock lock(mMutexLKF);
//    return std::set<shared_ptr<KeyFrameState>>(mvpLocalKeyFrames.begin(), mvpLocalKeyFrames.end());
//}

std::vector<shared_ptr<KeyFrameState>> Tracker::getAllVectorLocalKeyFrame() {
    boost::mutex::scoped_lock lock(mMutexLKF);
    return mvpLocalKeyFrames;
}

std::vector<shared_ptr<MapPoint>> &Tracker::cacheRefGetAllVectorLocalMapPoint() {
    if ( cache_mbLMP && cache_mtLMP.try_tock() ) {
        boost::mutex::scoped_lock lock(mMutexLMP);
        cache_mvpLocalMapPoints = mvpLocalMapPoints;
        cache_mbLMP = false;
    }
    return cache_mvpLocalMapPoints;
}
