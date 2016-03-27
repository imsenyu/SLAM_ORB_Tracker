//
//  Tracker.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "Tracker.hpp"
#include "Initializer.h"
Tracker::Tracker(InputBuffer *_pIB, FrameDrawer *_pFD, MapDrawer *_pMD, Vocabulary *_pVocabulary, Map *_pMap) :
    mpInputBuffer(_pIB), mpFrameDrawer(_pFD), mpMapDrawer(_pMD),
    meMode(Tracker::WorkMode::InitStep0), meLastMode(Tracker::WorkMode::Start),
    mpIniter(NULL),mpVocabulary(_pVocabulary), mpMap(_pMap) {
    
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



    if ( meMode == WorkMode::InitStep0 ) {
        // init first keyFrame
        // init first camera pose
        mpCurFrame->extractInit();

        initPose();

        initStepFirstKeyFrame();
        updateDrawer();
    }
    else if ( meMode == WorkMode::InitStep1 ) {
        // obtain second keyframe to build
        mpCurFrame->extractInit();

        bool st = initStepSecondKeyFrame();
        if ( st ) {
            mpPreFrame = shared_ptr<FrameState>( mpCurFrame );
        }
        updateDrawer();
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
            bStatus = TrackFromPreFrame();

            if ( bStatus ) {

                // TODO: how to insert KeyFrame
                //if(NeedNewKeyFrame())
                //    CreateNewKeyFrame();
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
    mpMapDrawer->update(mCurPose);
}

void Tracker::initPose() {
    mCurPose = PoseState(mpCurFrame->mId);
    mCurPose.mDir = Const::pnt3d_001;
    mCurPose.mPos = Const::pnt3d_000;
    mCurPose.mDir3 = Const::mat33_111.clone();
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
//    const double fx = Config::mCameraParameter.at<double>(0,0);
//    const double fy = Config::mCameraParameter.at<double>(1,1);
//    const double cx = Config::mCameraParameter.at<double>(0,2);
//    const double cy = Config::mCameraParameter.at<double>(1,2);
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
//            if ( mvMatchPair12[i]<0 ) {
//                mvMatchMask12[i] = false;
//            }

            if(mvMatchPair12[i]>=0 && !vbTriangulated[i])
            {
                mvMatchPair12[i]=-1;
                //mvMatchMask12[i]=false;

                numMatch--;
            }

            //if ( mvMatchMask12[i] ) {
            //    mpCurFrame->mvMatchMask[  mvMatchPair12[i] ] = true;
            //}
        }

        mMotion.mMatR = cv::Mat(3,3,CV_64FC1);//Rcw.clone();
        mMotion.mMatT = cv::Mat(3,1,CV_64FC1);//tcw.clone();
        //meMode = WorkMode::Normal;

        Rcw.convertTo(mMotion.mMatR, CV_64FC1 );
//        for(int i=0;i<3;i++)
//            for(int j=0;j<3;j++)
//                mMotion.mMatR.at<double>(i,j) = Rcw.at<float>(i,j);

        tcw.convertTo(mMotion.mMatT, CV_64FC1);
//        for(int i=0;i<3;i++)
//            mMotion.mMatT.at<double>(i) = tcw.at<float>(i);

        //confirm direction forward
        if ( mMotion.mMatT.at<double>(2) > 0.0f ) {
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
    mpIniFrame->mT2w = Const::mat44_1111;
    mpCurFrame->mT2w = cv::Mat(4,4,CV_64FC1);

    initMotion.mMatR.copyTo( mpCurFrame->mT2w.rowRange(0,3).colRange(0,3) );
    initMotion.mMatT.copyTo( mpCurFrame->mT2w.rowRange(0,3).col(3) );
    mpCurFrame->mT2w.at<double>(3,3) = 1.0f;

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
        shared_ptr<MapPoint> pMapPoint = shared_ptr<MapPoint>( new MapPoint(matMapPointPos, pCurKeyFrame, i) );

        // ok
        pMapPoint->setKeyFrame(pIniKeyFrame, i);
        pMapPoint->setKeyFrame(pCurKeyFrame, mvMatchPair12[i]);


        // ok
        pIniKeyFrame->insertMapPoint(pMapPoint, i);
        pCurKeyFrame->insertMapPoint(pMapPoint, mvMatchPair12[i] );

        // ok
        mpCurFrame->insertMapPoint( pMapPoint, mvMatchPair12[i] );

        // ok
        mpMap->insertMapPoint(pMapPoint);

    }

    // TODO: should construct  the connection from this KeyFrame to other KeyFrame by covisible MapPoint
    //pKFini->UpdateConnections();
    //pKFcur->UpdateConnections();
    // TODO: Optimizer::GlobalBundleAdjustemnt(mpMap,20);




    // Set median depth to 1
    float medianDepth = pIniKeyFrame->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;
    std::cout<<"medianDepth"<<medianDepth<<std::endl;
    if(medianDepth<0 )
    {
        return false;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pCurKeyFrame->mT2w.clone();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pCurKeyFrame->updatePose(Tc2w);

    std::cout<<"updated Tc2w"<<Tc2w<<std::endl;

    // Scale points
    std::vector<shared_ptr<MapPoint>> vpAllMapPoints = pIniKeyFrame->mvpMapPoint;
    for(int iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            std::cout<<"MapPoint["<<iMP<<"] "<< vpAllMapPoints[iMP]->mPos <<std::endl;
            shared_ptr<MapPoint> pMP = vpAllMapPoints[iMP];
            pMP->mPos = pMP->mPos*invMedianDepth;
        }
    }

    mpCurFrame->mT2w = pCurKeyFrame->mT2w.clone();




    std::cout<< "Init Map's MapPoint size"<<mpMap->mspMapPoint.size()<<std::endl;

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
    std::cout<<"TrackLast match "<<numMatch<<std::endl;

    // TODO: filterByOptical
    //numMatch = filterByOpticalFlow(mpPreFrame, mpCurFrame, mvMatchPair12, mvMatchMask12, mvMatchPoint );
    //std::cout<<"TrackLast flow "<<numMatch<<std::endl;

    // mpLsatFrame has its MapPoint vector
    // TODO: change matchPoint to matchMapPoint
//    vpMapPoint = std::vector<shared_ptr<MapPoint>>( mpCurFrame->mvKeyPoint.size(), shared_ptr<MapPoint>(NULL) );
//    int N = mvMatchPair12.size();
//    int numMatchMapPoint = 0;
//    for(int i=0;i<N;i++) {
//        if(false == mvMatchMask12[i]) continue;
//        if (!mpPreFrame->mvpMapPoint[ i ]) continue;
//        vpMapPoint[ mvMatchPair12[ i ] ] = mpPreFrame->mvpMapPoint[ i ];
//        numMatchMapPoint++;
//    }
    //std::cout<< "numMatchMapPoint " <<numMatchMapPoint<<std::endl;

    // TODO: set estimation mT2w
    mpCurFrame->mT2w = mpPreFrame->mT2w.clone();
    mpCurFrame->mvpMapPoint = vpMapPoint;

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
    if ( numMatch < 10 ) {
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
    std::cout<< "estimation mT2w "<<mpPreFrame->mT2w<<std::endl;
    std::cout<< "optimization mT2w "<<mpCurFrame->mT2w<<std::endl;
    return numMatch > 10;
}
