#pragma once

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <memory>
#include <iostream>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"

#include "tictoc.h"

using SCPointType = pcl::PointXYZI;
using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;


void coreImportTest ( void );


// sc param-independent helper functions
float xy2theta( const float & _x, const float & _y );
Eigen::MatrixXd circshift( Eigen::MatrixXd &_mat, int _num_shift );
std::vector<float> eig2stdvec( Eigen::MatrixXd _eigmat );


class SCManager
{
public:
    SCManager( ) = default;

    Eigen::MatrixXd makeScancontext( pcl::PointCloud<SCPointType> & _scan_down );
    Eigen::MatrixXd makeRingkeyFromScancontext( Eigen::MatrixXd &_desc );
    Eigen::MatrixXd makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc );

    int fastAlignUsingVkey ( Eigen::MatrixXd & _vkey1, Eigen::MatrixXd & _vkey2 );
    double distDirectSC ( Eigen::MatrixXd &_sc1, Eigen::MatrixXd &_sc2 );
    std::pair<double, int> distanceBtnScanContext ( Eigen::MatrixXd &_sc1, Eigen::MatrixXd &_sc2 );

    // User-side API
    void makeAndSaveScancontextAndKeys( pcl::PointCloud<SCPointType> & _scan_down );
    std::pair<int, float> detectLoopClosureID( void );

    const Eigen::MatrixXd& getConstRefRecentSCD(void);

public:
    // hyper parameters (tuned for mine tunnels)
    double LIDAR_HEIGHT = 0.0;            // 0 for robot-base-coord LiDAR scans

    int    PC_NUM_RING = 20;              // 20 in original paper
    int    PC_NUM_SECTOR = 60;            // 60 in original paper
    double PC_MAX_RADIUS = 60.0;          // 60m for mine tunnels (was 80m)

    double getSectorAngle() const { return 360.0 / double(PC_NUM_SECTOR); }
    double getRingGap() const { return PC_MAX_RADIUS / double(PC_NUM_RING); }

    // tree
    int    NUM_EXCLUDE_RECENT = 30;       // keyframe gap for loop detection
    int    NUM_CANDIDATES_FROM_TREE = 3;

    // loop thres
    double SEARCH_RATIO = 0.1;
    double SC_DIST_THRES = 0.25;          // tuned for mine tunnels (was 0.3)

    // config
    int    TREE_MAKING_PERIOD_ = 10;
    int    tree_making_period_conter = 0;

    // data
    std::vector<double> polarcontexts_timestamp_;
    std::vector<Eigen::MatrixXd> polarcontexts_;
    std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
    std::vector<Eigen::MatrixXd> polarcontext_vkeys_;

    KeyMat polarcontext_invkeys_mat_;
    KeyMat polarcontext_invkeys_to_search_;
    std::unique_ptr<InvKeyTree> polarcontext_tree_;

}; // SCManager
