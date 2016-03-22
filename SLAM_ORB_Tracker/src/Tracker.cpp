//
//  Tracker.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "Tracker.hpp"

Tracker::Tracker(InputBuffer* _pIB, FrameDrawer* _pFD, MapDrawer* _pMD):
    mpInputBuffer(_pIB), mpFrameDrawer(_pFD), mpMapDrawer(_pMD),
    meMode(Tracker::WorkMode::InitStep0), meLastMode(Tracker::WorkMode::Start) {
    
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
    mpCurFrame->extract();



    if ( meMode == WorkMode::InitStep0 ) {
        // init first keyFrame
        // init first camera pose
        initPose();
        meMode = WorkMode::Normal;
    }
    else if ( meMode == WorkMode::InitStep1 ) {
        // obtain second keyframe to build

//            MotionState motion;
//            motion.mvIds[0] = mpPreFrame->mId;
//            motion.mvIds[1] = mpCurFrame->mId;
//            motion.mMatR = Const::mat33_111.clone();
//            motion.mMatT = Const::mat31_001.clone();
//            
//            mCurPose = mCurPose.move(motion);
//            meMode = WorkMode::Normal;
    }
    // inited
    else {
        bool bStatus = true;
        MotionState motion;

        // 1. get camera pose
        if ( meMode == WorkMode::Normal ) {
            // tracking

            int nmatches = match(mpPreFrame, mpCurFrame, mvPair);
            std::cout<< "match: " <<nmatches << std::endl;
            nmatches = filerByOpticalFlow(mpPreFrame, mpCurFrame, mvPair);
            std::cout<< "match: " <<nmatches << std::endl;
            bStatus = computeMotion(mpPreFrame, mpCurFrame, motion);
            std::cout<< "motion " << bStatus << " "<<motion << std::endl;

            drawFilter(mpCurFrame, mvPair[1]);

        }
        else {
            // relocalise
        }

        // 2. build map track and map point


        // 3. insert keyframe


        // 4. if any failed
        if ( bStatus == false ) {
            //meMode = WorkMode::Fail;
            //updateDrawer();
            return -1;
        }

        std::cout << mCurPose << std::endl;
        mCurPose = mCurPose.move(motion);
        std::cout << mCurPose << std::endl;

    }



    updateDrawer();

    // clone current to previous
    mpPreFrame = shared_ptr<FrameState>( mpCurFrame );
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


int Tracker::match(shared_ptr<FrameState> pPreFrame, shared_ptr<FrameState> pCurFrame, std::vector<cv::Point2f> *mvPair) {

    cv::BruteForceMatcher<cv::L2<float>> BFMatcher;
    std::vector<cv::DMatch> vBFMatches;

    //暴力匹配
    BFMatcher.match(pPreFrame->mDescriptor, pCurFrame->mDescriptor, vBFMatches);
    mvPair[0].clear();
    mvPair[1].clear();
    
    //匹配完毕之后放到 vecPairPoint[2] 中
    for (auto& match : vBFMatches) {
       
        mvPair[0].push_back( pPreFrame->mvKeyPoint[match.queryIdx].pt );
        mvPair[1].push_back( pCurFrame->mvKeyPoint[match.trainIdx].pt );
    }
    
    return mvPair[0].size();
}


bool Tracker::computeMotion(shared_ptr<FrameState> pPreFrame, shared_ptr<FrameState> pCurFrame, MotionState& motion) {

    //是否能够解基础矩阵(满足匹配点数大于minFundamentMatches)
    bool _isUseFundamentalMatrix = mvPair[0].size() >= 10;
    
    motion.mvIds[0] = pPreFrame->mId;
    motion.mvIds[1] = pCurFrame->mId;
    
    //默认返回值
    bool retStatus = true;
    
    cv::Mat matFundamental, matFundStatus;

    
    //如果可以解基础矩阵
    if (_isUseFundamentalMatrix) {
        
        matFundamental = cv::findFundamentalMat(mvPair[0], mvPair[1], matFundStatus, CV_FM_RANSAC);
        
        //本质矩阵计算
        cv::Mat matE(3, 3, CV_64FC1);
        matE = Config::mCameraParameter.t() * matFundamental* Config::mCameraParameter;
        
        
        {
            printf("FundamentalMatrix=");
            std::cout << matFundamental << std::endl;
            printf("EssentialMatrix=");
            std::cout << matE << std::endl;
            printf("CameraParameter=");
            std::cout << Config::mCameraParameter << std::endl;
        }
        
        //SVD分解
        cv::SVD svdComputer;
        cv::Mat matU(3, 3, CV_64FC1), matS(3, 3, CV_64FC1), matVT(3, 3, CV_64FC1);
        
        svdComputer.compute(matE, matS, matU, matVT, cv::SVD::FULL_UV);
        
        //如果基础矩阵解挂了，考虑默认直线
        if (cv::sum(matS)[0] < 1.0f) {
            motion.mMatR = Const::mat33_111.clone();
            motion.mMatT = Const::mat31_001.clone();
            _isUseFundamentalMatrix = false;
            retStatus = false;
        }
        else {
            //如果 matU 或 matVT 的行列式小于0，反一下
            if (cv::determinant(matU) < 0)
                matU = -matU;
            if (cv::determinant(matVT) < 0)
                matVT = -matVT;
            {
                printf("SVD\n");
                std::cout << matU << std::endl << matS << std::endl << matVT << std::endl;
            }
            
            cv::Mat matR[4],
            matT[4];
            cv::Mat matW(3, 3, CV_64FC1);
            matW = 0.0f;
            matW.at<double>(0, 1) = -1.0f;
            matW.at<double>(1, 0) = 1.0f;
            matW.at<double>(2, 2) = 1.0f;
            
            
            //matR 和 matT 有四种可能解
            for (int i = 0; i < 4; i++) {
                matR[i] = matU * (i % 2 ? matW.t() : matW) * matVT;
                matT[i] = (i / 2 ? 1.0f : -1.0f) * matU.col(2);
                
                matT[i] = -matR[i].inv() * matT[i];
                matR[i] = matR[i].inv();
                
                //Trick: 强制默认往前方向走, 如果遇到数据集往后的 再说.
                if (matT[i].at<double>(2, 0) < 0.0f)  matT[i] = -matT[i];
                

                    {
                        printf("i=%d\n R=\n", i);
                        std::cout << matR[i] << std::endl;
                        printf("T=\n");
                        std::cout << matT[i] << std::endl;
                    }
            }
            //test all
            cv::Mat Q(4, 4, CV_64FC1);
            cv::Mat u1(4, 1, CV_64FC1), u2(4, 1, CV_64FC1);
            cv::Mat res(1, 1, CV_64FC1);
            double compSEL[4];
            
            for (int i = 0; i < 4; i++) {
                //Trick: 选择在转弯上转的最小的那个
                cv::Mat tmp = (matR[i] * Const::mat31_100);
                compSEL[i] = tmp.at<double>(0, 0);
                printf("valid[%d] = %f\n", i, compSEL[i]);
            }
            double maxValidation = -100.0f; int selectDirectionIdx = 0;
            for (int i = 0; i < 2; i++) {
                if (compSEL[i] > maxValidation) {
                    maxValidation = compSEL[i];
                    selectDirectionIdx = i;
                }
            }
            
            //原则上来说应该使用三角测量来选择,但似乎效果并不好
            // 已验证，有问题。。。
            //int preSelect = selectDirectionIdx;
            //if (true){
            //	// 三角测量
            //	ScaleEstimator sEstimator;
            //	motion.setMatT(matT[0].clone());
            //	int num_inlier = 0;
            //	
            //	for (int i = 0; i < 2; i++) {
            //		motion.setMatR(matR[i].clone());
            //		sEstimator.updateMotion(&motion);
            //		int num = sEstimator.triangulate();
            //		if (num > num_inlier) {
            //			selectDirectionIdx = i;
            //			num_inlier = num;
            //		}
            //		//if (CFG_bIsLogGlobal)
            //		printf("[%d]=%d\n", i, num);
            //	}		
            //}
            //if (preSelect != selectDirectionIdx) {
            
            //	printf("preSel=%d sel=%d\n", preSelect, selectDirectionIdx);
            //	//printf("%f %f\n%d %d\n",)
            //}
            
            printf("selectDirectionIdx=%d\n", selectDirectionIdx);
            motion.mMatR = matR[selectDirectionIdx].clone();
            motion.mMatT = matT[selectDirectionIdx].clone();
            
        }
        
    }
    else {
        motion.mMatR = Const::mat33_111.clone();
        motion.mMatT = Const::mat31_001.clone();
        retStatus = false;
               //TODO： 解挂了返回直线，点数不够返回 跳帧。
    }
    

    return retStatus;
}


int Tracker::filerByOpticalFlow(shared_ptr<FrameState> pPreFrame, shared_ptr<FrameState> pCurFrame, std::vector<cv::Point2f> *mvPair) {
    double threshold = Config::dOpticalFlowThreshold;

    // 光流运算匹配
    std::vector<cv::Point2f> vFound;
    std::vector<uchar> vStatus;
    std::vector<float> vErr;

    printf("%d %d %d %d\n", pPreFrame->mImage.rows,
        pCurFrame->mImage.rows,
        mvPair[0].size(), mvPair[1].size());

    cv::calcOpticalFlowPyrLK(
            pPreFrame->mImage, pCurFrame->mImage,
            mvPair[0], vFound,
            vStatus, vErr);

    //根据阈值筛点
/*    for (int nStatus = 0; nStatus < vStatus.size(); nStatus++) {
        if (true == vStatus[nStatus]) {
            cv::Point2f
                    &p_Pre = mvPair[0][nStatus],
                    &p_BF = mvPair[1][nStatus],
                    &p_OF = vFound[nStatus];
            double dx = abs(p_BF.x - p_OF.x),
                    dy = abs(p_BF.y - p_OF.y);
            if (dx*dx + dy*dy > threshold) {
                vStatus[nStatus] = false;
            }
        }
    }*/

    std::vector<cv::Point2f>::iterator iterPoint[2] = { mvPair[0].begin(), mvPair[1].begin() };
    for(int nStatus = 0 ;
        nStatus!= vStatus.size() &&
        iterPoint[0] != mvPair[0].end() &&
        iterPoint[1] != mvPair[1].end();
        nStatus++)
    {

        cv::Point2f
            &ptPre = *iterPoint[0],
            &ptCur = *iterPoint[1],
            &ptFound = vFound[nStatus];

        double dx = fabs(ptCur.x - ptFound.x),
                dy = fabs(ptCur.y - ptFound.y);

        if ( dx*dx+dy*dy > threshold || false == vStatus[nStatus]) {
            iterPoint[0] = mvPair[0].erase(iterPoint[0]);
            iterPoint[1] = mvPair[1].erase(iterPoint[1]);
        }
        else {
            iterPoint[0]++;
            iterPoint[1]++;
        }

    }


    return mvPair[0].size();
}

void Tracker::drawFilter(shared_ptr<FrameState> pFrame, std::vector<cv::Point2f> &mvPoint) {

    for(int i=0;i<mvPoint.size();i++) {
        cv::rectangle(pFrame->mImage, mvPoint[i] - cv::Point2f(0.5,0.5), mvPoint[i] + cv::Point2f(0.5,0.5), cv::Scalar(0,0,255));
    }

}
