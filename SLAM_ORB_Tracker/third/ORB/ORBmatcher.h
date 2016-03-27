/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include"MapPoint.hpp"
#include"KeyFrameState.hpp"
#include"FrameState.hpp"


namespace ORB_SLAM
{

class ORBmatcher
{    
public:

    ORBmatcher(float nnratio=0.6, bool checkOri=true);

    // Computes the Hamming distance between two ORB descriptors
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);
//
//    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
//    // Used to track the local map (Tracking)
//    int SearchByProjection(FrameState &F, const std::vector<shared_ptr<MapPoint>> &vpMapPoints, const float th=3);
//
//    // Project MapPoints tracked in last frame into the current frame and search matches.
//    // Used to track from previous frame (Tracking)
//    int SearchByProjection(FrameState &CurrentFrame, const FrameState &LastFrame, float th);
//
//    // Project MapPoints seen in KeyFrame into the Frame and search matches.
//    // Used in relocalisation (Tracking)
//    int SearchByProjection(FrameState &CurrentFrame, shared_ptr<KeyFrameState> pKF, const std::set<shared_ptr<MapPoint>> &sAlreadyFound, float th, int ORBdist);
//
//    // Project MapPoints using a Similarity Transformation and search matches.
//    // Used in loop detection (Loop Closing)
//     int SearchByProjection(shared_ptr<KeyFrameState> pKF, cv::Mat Scw, const std::vector<shared_ptr<MapPoint>> &vpPoints, std::vector<shared_ptr<MapPoint>> &vpMatched, int th);
//
//    // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
//    // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
//    // Used in Relocalisation and Loop Detection
//    //int SearchByBoW(shared_ptr<KeyFrameState>pKF, FrameState &F, std::vector<shared_ptr<MapPoint>> &vpMapPointMatches);
//    //int SearchByBoW(shared_ptr<KeyFrameState>pKF1, shared_ptr<KeyFrameState> pKF2, std::vector<shared_ptr<MapPoint>> &vpMatches12);
//
//    // Search MapPoints tracked in Frame1 in Frame2 in a window centered at their position in Frame1
    int WindowSearch(shared_ptr<FrameState> pF1, shared_ptr<FrameState> pF2, int windowSize, std::vector<shared_ptr<MapPoint>> &vpMapPointMatches2, int minOctave=-1, int maxOctave=INT_MAX);
//    // Refined matching when we have a guess of FrameState 2 pose
    int SearchByProjection(shared_ptr<FrameState> pF1, shared_ptr<FrameState> pF2, int windowSize, std::vector<shared_ptr<MapPoint>> &vpMapPointMatches2);

    // Matching for the Map Initialization
    int SearchForInitialization(shared_ptr<FrameState> pF1, shared_ptr<FrameState> pF2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);

    // Matching to triangulate new MapPoints. Check Epipolar Constraint
//    int SearchForTriangulation(shared_ptr<KeyFrameState>pKF1, shared_ptr<KeyFrameState> pKF2, cv::Mat F12,
//                               std::vector<cv::KeyPoint> &vMatchedKeys1, std::vector<cv::KeyPoint> &vMatchedKeys2,
//                               std::vector<std::pair<size_t, size_t> > &vMatchedPairs);

    // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
    //int SearchBySim3(shared_ptr<KeyFrameState> pKF1, shared_ptr<KeyFrameState> pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, float th);

    // Project MapPoints into KeyFrameState and search for duplicated MapPoints.
    //int Fuse(shared_ptr<KeyFrameState> pKF, std::vector<MapPoint *> &vpMapPoints, float th=2.5);

    // Project MapPoints into KeyFrameState using a given Sim3 and search for duplicated MapPoints.
   // int Fuse(shared_ptr<KeyFrameState> pKF, cv::Mat Scw, const std::vector<shared_ptr<MapPoint>> &vpPoints, float th=2.5);

public:

    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;


protected:

//    bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const shared_ptr<KeyFrameState>pKF);
//
//    float RadiusByViewingCos(const float &viewCos);
//
    void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

    float mfNNratio;
    bool mbCheckOrientation;
};

}// namespace ORB_SLAM

#endif // ORBMATCHER_H
