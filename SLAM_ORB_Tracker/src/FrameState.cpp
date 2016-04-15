//
//  FrameBuffer.cpp
//  SLAM_ORB_Tracker
//
//  Created by Sen Yu on 3/20/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "FrameState.hpp"


bool FrameState::mbInitialComputations=true;
float FrameState::mfGridElementWidthInv;
float FrameState::mfGridElementHeightInv;
int FrameState::mnMinX;
int FrameState::mnMaxX;
int FrameState::mnMinY;
int FrameState::mnMaxY;

FrameState::FrameState(int _id, const std::string& _load): mId(_id), mLoaded(false), mLoadFormat(_load),mbPainted(false) {
    loadImage(mId);
}

FrameState::FrameState(const FrameState& _FS) {
    mId = _FS.mId;
    mLoaded = _FS.mLoaded;
    mImage = _FS.mImage.clone();
    mvKeyPoint = _FS.mvKeyPoint;
    mvMatchMask = _FS.mvMatchMask;
    mvbOutlier = _FS.mvbOutlier;
    mT2w = _FS.mT2w.clone();
    mDescriptor = _FS.mDescriptor.clone();
    mvpMapPoint = _FS.mvpMapPoint;
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=_FS.mGrid[i][j];
}

FrameState::FrameState(const FrameState* _pFS) {
    mId = _pFS->mId;
    mLoaded = _pFS->mLoaded;
    mImage = _pFS->mImage.clone();
    mvKeyPoint = _pFS->mvKeyPoint;
    mvMatchMask = _pFS->mvMatchMask;
    mvbOutlier = _pFS->mvbOutlier;
    mT2w = _pFS->mT2w.clone();
    mDescriptor = _pFS->mDescriptor.clone();
    mvpMapPoint = _pFS->mvpMapPoint;
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=_pFS->mGrid[i][j];
}

bool FrameState::loadImage(int _id) {
    if (_id < 0) return false;
    mId = _id;
    std::string imgPath = cv::format(mLoadFormat.c_str(), mId);
    mImage = cv::imread(imgPath, CV_LOAD_IMAGE_GRAYSCALE);
    
    // image load error
    if (mImage.rows == 0 || mImage.cols == 0) {
        std::string error = "Image Load Error: " + imgPath;
        std::cout<<error <<std::endl;
        //throw std::exception( error.c_str() );
    }

    mLoaded = true;

    if ( mbInitialComputations ) {
        mnMinX = 0;
        mnMaxX = mImage.cols;
        mnMinY = 0;
        mnMaxY = mImage.rows;


        mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
        mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);
        mbInitialComputations = false;
    }
    return mLoaded;
}

FrameState::~FrameState() {
}

int FrameState::extract() {


    cv::ORB orbDetector(Config::iFeatureNum, Config::dScaleFactor, Config::dScaleLevel);
    orbDetector.detect(mImage, mvKeyPoint);
    orbDetector.compute(mImage, mvKeyPoint, mDescriptor);


    int nKP = mvKeyPoint.size();
    mvpMapPoint = std::vector<shared_ptr<MapPoint>>( nKP, shared_ptr<MapPoint>(NULL) );
    mvMatchMask = std::vector<uchar>( nKP, false );
    mvbOutlier = std::vector<uchar>( nKP, false);

    for(size_t i=0;i<mvKeyPoint.size();i++)
    {
        cv::KeyPoint &kp = mvKeyPoint[i];

        int posX, posY;
        posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
        posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
            continue;

        mGrid[posX][posY].push_back(i);
    }


    if ( nKP == 0 ) return 0;

    return nKP;
}




int FrameState::extractInit() {
    ORB_SLAM::ORBextractor ORBextractor(/*Config::iFeatureNum*4*/4000, Config::dScaleFactor, Config::dScaleLevel,1,20);
    ORBextractor(mImage,cv::Mat(),mvKeyPoint,mDescriptor);

    int nKP = mvKeyPoint.size();
    mvpMapPoint = std::vector<shared_ptr<MapPoint>>( nKP, shared_ptr<MapPoint>(NULL) );
    mvMatchMask = std::vector<uchar>( nKP, false );
    mvbOutlier = std::vector<uchar>( nKP, false);

    for(size_t i=0;i<mvKeyPoint.size();i++)
    {
        cv::KeyPoint &kp = mvKeyPoint[i];

        int posX, posY;
        posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
        posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
            continue;

        mGrid[posX][posY].push_back(i);
    }


    if ( nKP == 0 ) return 0;

    return nKP;
}

int FrameState::extractNormal() {

    ORB_SLAM::ORBextractor ORBextractor(Config::iFeatureNum, Config::dScaleFactor, Config::dScaleLevel,1,20);
    ORBextractor(mImage,cv::Mat(),mvKeyPoint,mDescriptor);

    int nKP = mvKeyPoint.size();
    mvpMapPoint = std::vector<shared_ptr<MapPoint>>( nKP, shared_ptr<MapPoint>(NULL) );
    mvMatchMask = std::vector<uchar>( nKP, false );
    mvbOutlier = std::vector<uchar>( nKP, false);

    for(size_t i=0;i<mvKeyPoint.size();i++)
    {
        cv::KeyPoint &kp = mvKeyPoint[i];

        int posX, posY;
        posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
        posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
            continue;

        mGrid[posX][posY].push_back(i);
    }


    if ( nKP == 0 ) return 0;

    return nKP;
}

std::vector<size_t> FrameState::getFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel, const int maxLevel) const
{
    std::vector<size_t> vIndices;
    vIndices.reserve(mvKeyPoint.size());


    int nMinCellX = floor((x-mnMinX-r)*mfGridElementWidthInv);
    nMinCellX = std::max(0,nMinCellX);
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    int nMaxCellX = ceil((x-mnMinX+r)*mfGridElementWidthInv);
    nMaxCellX = std::min(FRAME_GRID_COLS-1,nMaxCellX);
    if(nMaxCellX<0)
        return vIndices;

    int nMinCellY = floor((y-mnMinY-r)*mfGridElementHeightInv);
    nMinCellY = max(0,nMinCellY);
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    int nMaxCellY = ceil((y-mnMinY+r)*mfGridElementHeightInv);
    nMaxCellY = min(FRAME_GRID_ROWS-1,nMaxCellY);
    if(nMaxCellY<0)
        return vIndices;

    bool bCheckLevels=true;
    bool bSameLevel=false;
    if(minLevel==-1 && maxLevel==-1)
        bCheckLevels=false;
    else
    if(minLevel==maxLevel)
        bSameLevel=true;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            std::vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeyPoint[vCell[j]];
                if(bCheckLevels && !bSameLevel)
                {
                    if(kpUn.octave<minLevel || kpUn.octave>maxLevel)
                        continue;
                }
                else if(bSameLevel)
                {
                    if(kpUn.octave!=minLevel)
                        continue;
                }

                if(abs(kpUn.pt.x-x)>r || abs(kpUn.pt.y-y)>r)
                    continue;

                vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;

}

bool FrameState::isInFrustum(shared_ptr<MapPoint> pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->getMPos();

    // 3D in camera coordinates
    const cv::Mat Pc = mMatR*P+mMatT;
    const float PcX = Pc.at<float>(0);
    const float PcY= Pc.at<float>(1);
    const float PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0/PcZ;
    const float u=Config::dFx*PcX*invz+Config::dCx;
    const float v=Config::dFy*PcY*invz+Config::dCy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mO2w;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

    // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale level acording to the distance
    float ratio = dist/minDistance;

    vector<double>::iterator it = std::lower_bound(Config::vScaleFactors.begin(), Config::vScaleFactors.end(), ratio);
    int nPredictedLevel = it-Config::vScaleFactors.begin();

    if(nPredictedLevel>=Config::dScaleLevel)
        nPredictedLevel = Config::dScaleLevel -1;

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}