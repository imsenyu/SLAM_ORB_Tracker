//
// Created by Sen Yu on 3/26/16.
// Copyright (c) 2016 Sen Yu. All rights reserved.
//

#ifndef SLAM_ORB_TRACKER_OPTIMIZER_HPP
#define SLAM_ORB_TRACKER_OPTIMIZER_HPP

#include "stdafx.hpp"
#include "FrameState.hpp"
#include "KeyFrameState.hpp"
#include "MapPoint.hpp"

// g2o
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include "third/g2o/g2o/core/block_solver.h"
#include "third/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "third/g2o/g2o/core/robust_kernel_impl.h"

// search recur
#include "third/g2o/g2o/solvers/eigen/linear_solver_eigen.h"
#include "third/g2o/g2o/solvers/dense/linear_solver_dense.h"
#include "third/g2o/g2o/types/sba/types_six_dof_expmap.h"
#include "third/g2o/g2o/types/sim3/types_seven_dof_expmap.h"


namespace g2o {
    using namespace Eigen;
    class EdgeSE3ProjectXYZ : public BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectXYZ();

        bool read(std::istream &is);

        bool write(std::ostream &os) const;

        void computeError() {
            const VertexSE3Expmap *v1 = static_cast<const VertexSE3Expmap *>(_vertices[1]);
            const VertexSBAPointXYZ *v2 = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
            Vector2d obs(_measurement);
            _error = obs - cam_project(v1->estimate().map(v2->estimate()));
        }

        bool isDepthPositive() {
            const VertexSE3Expmap *v1 = static_cast<const VertexSE3Expmap *>(_vertices[1]);
            const VertexSBAPointXYZ *v2 = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
            return (v1->estimate().map(v2->estimate()))(2) > 0.0;
        }


        virtual void linearizeOplus();

        Vector2d cam_project(const Vector3d &trans_xyz) const;

        double fx, fy, cx, cy;
    };
};

class Optimizer {
public:

    static int PoseOptimization(shared_ptr<FrameState> _pFrame);
    static void LocalBundleAdjustment(shared_ptr<KeyFrameState> pKF, bool pbStopFlag = false);
    static void GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool bStopFlag = false);
    static void BundleAdjustment(const vector<shared_ptr<KeyFrameState>> &vpKFs, const vector<shared_ptr<MapPoint>> &vpMP, int nIterations, bool bStopFlag = false);
};


#endif //SLAM_ORB_TRACKER_OPTIMIZER_HPP
