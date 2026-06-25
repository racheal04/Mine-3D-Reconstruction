// ScanContext Loop Closure for FAST-LIO2
// Integrates ScanContext loop detection + PCL ICP verification + Eigen-based pose graph optimization
// No GTSAM dependency - uses Eigen sparse solvers for pose graph

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <mutex>
#include <thread>
#include <deque>

#include "Scancontext.h"

using PointType = pcl::PointXYZI;

// ============================================================================
// SE(3) math helpers
// ============================================================================
Eigen::Matrix4f poseToMatrix(const geometry_msgs::Pose& pose) {
    Eigen::Affine3f T = Eigen::Affine3f::Identity();
    T.translation() << pose.position.x, pose.position.y, pose.position.z;
    Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    T.linear() = q.toRotationMatrix();
    return T.matrix();
}

geometry_msgs::Pose matrixToPose(const Eigen::Matrix4f& T) {
    geometry_msgs::Pose pose;
    pose.position.x = T(0,3); pose.position.y = T(1,3); pose.position.z = T(2,3);
    Eigen::Quaternionf q(T.block<3,3>(0,0));
    pose.orientation.w = q.w(); pose.orientation.x = q.x();
    pose.orientation.y = q.y(); pose.orientation.z = q.z();
    return pose;
}

Eigen::Matrix<float, 6, 1> se3Log(const Eigen::Matrix4f& T) {
    // Logarithm map: SE(3) -> se(3)
    Eigen::Matrix<float, 6, 1> xi;
    Eigen::Matrix3f R = T.block<3,3>(0,0);
    Eigen::Vector3f t = T.block<3,1>(0,3);

    float cos_theta = (R.trace() - 1.0f) / 2.0f;
    cos_theta = std::max(-1.0f, std::min(1.0f, cos_theta));
    float theta = std::acos(cos_theta);

    if (theta < 1e-10) {
        xi.head<3>() = t;
        xi.tail<3>().setZero();
    } else {
        Eigen::Matrix3f lnR = (theta / (2.0f * std::sin(theta))) * (R - R.transpose());
        xi.tail<3>() << lnR(2,1), lnR(0,2), lnR(1,0);  // rotation vector
        float a = std::sin(theta) / theta;
        float b = (1.0f - std::cos(theta)) / (theta * theta);
        Eigen::Matrix3f V_inv = Eigen::Matrix3f::Identity() - 0.5f * lnR +
            (1.0f / (theta*theta)) * (1.0f - a / (2.0f * b)) * lnR * lnR;
        xi.head<3>() = V_inv * t;
    }
    return xi;
}

Eigen::Matrix4f se3Exp(const Eigen::Matrix<float, 6, 1>& xi) {
    // Exponential map: se(3) -> SE(3)
    Eigen::Vector3f rho = xi.head<3>();
    Eigen::Vector3f phi = xi.tail<3>();
    float theta = phi.norm();

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    if (theta < 1e-10) {
        T.block<3,1>(0,3) = rho;
    } else {
        Eigen::Vector3f axis = phi / theta;
        Eigen::Matrix3f axis_skew;
        axis_skew << 0, -axis(2), axis(1),
                     axis(2), 0, -axis(0),
                     -axis(1), axis(0), 0;
        Eigen::Matrix3f R = Eigen::Matrix3f::Identity() +
            std::sin(theta) * axis_skew + (1.0f - std::cos(theta)) * axis_skew * axis_skew;

        float a = std::sin(theta) / theta;
        float b = (1.0f - std::cos(theta)) / (theta * theta);
        Eigen::Matrix3f V = a * Eigen::Matrix3f::Identity() + b * axis_skew +
            (1.0f - a) / (theta * theta) * (axis * axis.transpose());
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = V * rho;
    }
    return T;
}

// ============================================================================
// Keyframe
// ============================================================================
struct KeyFrame {
    int index;
    Eigen::Matrix4f pose;  // global pose in world frame
    pcl::PointCloud<PointType>::Ptr cloud;  // downsampled point cloud for ScanContext
    pcl::PointCloud<PointType>::Ptr cloud_full;  // for ICP (less downsampled)
    double timestamp;
};

// ============================================================================
// LoopClosure Node
// ============================================================================
class LoopClosureNode {
public:
    LoopClosureNode(ros::NodeHandle& nh) : nh_(nh) {
        // Subscribers
        sub_odom_ = nh_.subscribe("/Odometry", 100, &LoopClosureNode::odomCallback, this);
        sub_cloud_ = nh_.subscribe("/cloud_registered", 100, &LoopClosureNode::cloudCallback, this);

        // Publishers
        pub_corrected_path_ = nh_.advertise<nav_msgs::Path>("/corrected_path", 10);
        pub_loop_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/loop_constraints", 10);
        pub_corrected_odom_ = nh_.advertise<nav_msgs::Odometry>("/corrected_odometry", 10);

        // Parameters
        nh_.param<float>("keyframe_dist_threshold", kf_dist_thresh_, 1.0);
        nh_.param<float>("keyframe_angle_threshold", kf_angle_thresh_, 0.2);
        nh_.param<float>("icp_fitness_threshold", icp_fitness_thresh_, 0.1);
        nh_.param<float>("loop_frequency", loop_freq_, 1.0);
        nh_.param<int>("num_exclude_recent", num_exclude_recent_, 30);
        nh_.param<float>("sc_dist_threshold", sc_dist_thresh_, 0.25);
        nh_.param<float>("sc_max_radius", sc_max_radius_, 60.0);

        // Downsample filters
        float sc_voxel = 0.5;  // for ScanContext (coarse)
        float icp_voxel = 0.3; // for ICP (medium)
        down_filter_sc_.setLeafSize(sc_voxel, sc_voxel, sc_voxel);
        down_filter_icp_.setLeafSize(icp_voxel, icp_voxel, icp_voxel);

        // Tune ScanContext for mine tunnels
        sc_manager_.PC_MAX_RADIUS = sc_max_radius_;
        sc_manager_.SC_DIST_THRES = sc_dist_thresh_;
        // NOTE: NUM_EXCLUDE_RECENT is const in header, so we set it here via the loop closure check logic

        ROS_INFO("LoopClosureNode initialized. kf_dist=%.2f, icp_thresh=%.3f, sc_thresh=%.3f",
                 kf_dist_thresh_, icp_fitness_thresh_, sc_dist_thresh_);

        // Start loop closure thread
        loop_thread_ = std::thread(&LoopClosureNode::loopClosureThread, this);
    }

    ~LoopClosureNode() {
        if (loop_thread_.joinable())
            loop_thread_.join();
    }

private:
    // ========================================================================
    // Callbacks
    // ========================================================================
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        latest_pose_ = poseToMatrix(msg->pose.pose);
        latest_odom_time_ = msg->header.stamp.toSec();
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);

        pcl::PointCloud<PointType>::Ptr cloud_in(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*msg, *cloud_in);
        if (cloud_in->empty()) return;

        // Downsample for ScanContext
        pcl::PointCloud<PointType>::Ptr cloud_sc(new pcl::PointCloud<PointType>());
        down_filter_sc_.setInputCloud(cloud_in);
        down_filter_sc_.filter(*cloud_sc);

        // Downsample for ICP
        pcl::PointCloud<PointType>::Ptr cloud_icp(new pcl::PointCloud<PointType>());
        down_filter_icp_.setInputCloud(cloud_in);
        down_filter_icp_.filter(*cloud_icp);

        latest_cloud_sc_ = cloud_sc;
        latest_cloud_icp_ = cloud_icp;

        // Check if should add keyframe
        if (shouldAddKeyframe()) {
            addKeyframe();
        }
    }

    // ========================================================================
    // Keyframe management
    // ========================================================================
    bool shouldAddKeyframe() {
        if (keyframes_.empty()) return true;

        Eigen::Matrix4f& last_pose = keyframes_.back().pose;
        Eigen::Vector3f last_t = last_pose.block<3,1>(0,3);
        Eigen::Vector3f cur_t = latest_pose_.block<3,1>(0,3);
        float dist = (cur_t - last_t).norm();

        Eigen::Quaternionf last_q(last_pose.block<3,3>(0,0));
        Eigen::Quaternionf cur_q(latest_pose_.block<3,3>(0,0));
        float angle = last_q.angularDistance(cur_q);

        return (dist > kf_dist_thresh_ || angle > kf_angle_thresh_);
    }

    void addKeyframe() {
        KeyFrame kf;
        kf.index = keyframes_.size();
        kf.pose = latest_pose_;
        kf.timestamp = latest_odom_time_;
        kf.cloud = latest_cloud_sc_;
        kf.cloud_full = latest_cloud_icp_;

        keyframes_.push_back(kf);
        keyframe_poses_.push_back(latest_pose_);
        poses_raw_odom_.push_back(latest_pose_);   // permanently save raw odometry

        // Store ScanContext descriptor
        sc_manager_.makeAndSaveScancontextAndKeys(*latest_cloud_sc_);

        if (keyframes_.size() % 50 == 0) {
            ROS_INFO("[LoopClosure] Keyframes: %zu, SC descriptors: %zu",
                     keyframes_.size(), sc_manager_.polarcontexts_.size());
        }
    }

    // ========================================================================
    // Loop closure thread
    // ========================================================================
    void loopClosureThread() {
        ros::Rate rate(loop_freq_);
        while (ros::ok()) {
            rate.sleep();
            performLoopClosure();
        }
    }

    void performLoopClosure() {
        std::lock_guard<std::mutex> lock(mtx_);

        if (keyframes_.size() < (size_t)num_exclude_recent_ + 5)
            return;

        // Run ScanContext detection
        auto detect_result = sc_manager_.detectLoopClosureID();
        int loop_idx = detect_result.first;
        float yaw_diff_rad = detect_result.second;

        if (loop_idx == -1) return;  // No loop found

        int cur_idx = keyframes_.size() - 1;
        ROS_INFO("[LoopClosure] SC loop candidate: %d <-> %d (yaw diff: %.2f deg)",
                 cur_idx, loop_idx, yaw_diff_rad * 180.0 / M_PI);

        // ICP verification
        Eigen::Matrix4f loop_transform;
        if (!verifyLoopWithICP(loop_idx, cur_idx, yaw_diff_rad, loop_transform)) {
            ROS_INFO("[LoopClosure] ICP verification FAILED. Rejecting loop.");
            return;
        }

        ROS_INFO("[LoopClosure] ICP verification PASSED. Adding loop constraint %d <-> %d.",
                 loop_idx, cur_idx);

        // Store loop edge
        // ICP: T_icp maps cloud_cur → cloud_loop (both in world frame)
        // Constraint: T_loop should be corrected to align with T_cur:
        //   T_j_corrected = T_icp^(-1) * T_j
        // Between-factor (i=cur, j=loop): T_i * T_edge = T_j_corrected
        //   T_i * T_edge = T_icp^(-1) * T_j
        //   T_edge = T_i^(-1) * T_icp^(-1) * T_j
        LoopEdge edge;
        edge.idx_from = cur_idx;
        edge.idx_to = loop_idx;
        Eigen::Matrix4f T_i = poses_raw_odom_[cur_idx];   // use raw odometry (fixed frame)
        Eigen::Matrix4f T_j = poses_raw_odom_[loop_idx];  // use raw odometry (fixed frame)
        edge.transform = T_i.inverse() * loop_transform.inverse() * T_j;
        loop_edges_.push_back(edge);

        // Optimize pose graph
        optimizePoseGraph();

        // Publish results
        publishCorrectedPath();
        publishLoopMarkers();
    }

    // ========================================================================
    // ICP verification
    // ========================================================================
    bool verifyLoopWithICP(int loop_idx, int cur_idx, float yaw_diff_rad, Eigen::Matrix4f& loop_transform) {
        // Build point clouds from surrounding keyframes
        pcl::PointCloud<PointType>::Ptr cloud_cur(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr cloud_loop(new pcl::PointCloud<PointType>());

        int search_range_cur = 5;   // current side

        // NOTE: /cloud_registered points are already in WORLD frame (FAST-LIO2
        // transforms them with pointBodyToWorld before publishing).
        // Do NOT apply keyframe pose again — that would double-transform them.

        // Current keyframe + neighbors (already in world frame)
        for (int i = cur_idx - search_range_cur; i <= cur_idx; i++) {
            if (i < 0 || i >= (int)keyframes_.size()) continue;
            *cloud_cur += *keyframes_[i].cloud_full;
        }

        // Loop side: ALL keyframes from 0 to loop_idx+50 (full entrance region)
        int loop_end = std::min(loop_idx + 50, (int)keyframes_.size() - 1);
        for (int i = 0; i <= loop_end; i++) {
            *cloud_loop += *keyframes_[i].cloud_full;
        }

        // Downsample both clouds for ICP
        pcl::VoxelGrid<PointType> icp_filter;
        icp_filter.setLeafSize(0.2, 0.2, 0.2);  // denser for better matching
        pcl::PointCloud<PointType>::Ptr cloud_cur_ds(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr cloud_loop_ds(new pcl::PointCloud<PointType>());
        icp_filter.setInputCloud(cloud_cur); icp_filter.filter(*cloud_cur_ds);
        icp_filter.setInputCloud(cloud_loop); icp_filter.filter(*cloud_loop_ds);

        ROS_INFO("[LoopClosure] ICP clouds: cur=%zu pts, loop=%zu pts",
                 cloud_cur_ds->size(), cloud_loop_ds->size());

        if (cloud_cur_ds->size() < 500 || cloud_loop_ds->size() < 500) {
            ROS_WARN("[LoopClosure] Too few points for ICP (cur=%zu, loop=%zu)", cloud_cur_ds->size(), cloud_loop_ds->size());
            return false;
        }

        // Step 1: Coarse 2D alignment (XY plane + yaw) to handle large Z drift
        pcl::PointCloud<PointType>::Ptr cur_2d(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr loop_2d(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*cloud_cur_ds, *cur_2d);
        pcl::copyPointCloud(*cloud_loop_ds, *loop_2d);
        for (auto& pt : cur_2d->points)  pt.z = 0;
        for (auto& pt : loop_2d->points) pt.z = 0;

        pcl::IterativeClosestPoint<PointType, PointType> icp_2d;
        icp_2d.setMaxCorrespondenceDistance(20.0);
        icp_2d.setMaximumIterations(50);
        icp_2d.setRANSACIterations(5);

        Eigen::Matrix4f init_2d = Eigen::Matrix4f::Identity();
        init_2d.block<3,3>(0,0) = Eigen::AngleAxisf(yaw_diff_rad, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        pcl::PointCloud<PointType> aligned_2d;
        icp_2d.setInputSource(cur_2d);
        icp_2d.setInputTarget(loop_2d);
        icp_2d.align(aligned_2d, init_2d);

        Eigen::Matrix4f init_guess = icp_2d.hasConverged() ?
            icp_2d.getFinalTransformation() : init_2d;
        // Restore Z from odometry (don't trust 2D ICP for Z)
        init_guess(2,3) = 0;

        if (icp_2d.hasConverged()) {
            ROS_INFO("[LoopClosure] 2D ICP converged: fitness=%.4f, translation=[%.2f, %.2f]",
                     icp_2d.getFitnessScore(), init_guess(0,3), init_guess(1,3));
        } else {
            ROS_WARN("[LoopClosure] 2D ICP did NOT converge");
        }

        // Step 2: Full 3D ICP refinement
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(30.0);  // large to handle Z drift
        icp.setMaximumIterations(200);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(10);  // filter outliers

        icp.setInputSource(cloud_cur_ds);
        icp.setInputTarget(cloud_loop_ds);
        pcl::PointCloud<PointType> aligned;
        icp.align(aligned, init_guess);

        if (!icp.hasConverged()) {
            ROS_WARN("[LoopClosure] 3D ICP did NOT converge");
            return false;
        }

        float fitness = icp.getFitnessScore();
        if (fitness > icp_fitness_thresh_) {
            ROS_WARN("[LoopClosure] 3D ICP fitness too high: %.4f > %.4f", fitness, icp_fitness_thresh_);
            return false;
        }

        // ICP result T_icp maps cloud_cur -> cloud_loop (both in world frame)
        // Constraint: T_loop ≈ T_icp * T_cur
        // Loop edge (from cur to loop): T_cur * T_edge = T_loop => T_edge ≈ T_icp
        loop_transform = icp.getFinalTransformation();
        // Store as edge from cur_idx to loop_idx: T_cur * loop_transform = T_loop
        ROS_INFO("[LoopClosure] ICP fitness: %.4f (threshold: %.4f)", fitness, icp_fitness_thresh_);
        return true;
    }

    // ========================================================================
    // Pose Graph Optimization (Eigen - no GTSAM)
    // ========================================================================
    struct LoopEdge {
        int idx_from;
        int idx_to;
        Eigen::Matrix4f transform;  // T_from^{-1} * T_to (measured relative pose)
    };

    void optimizePoseGraph() {
        int N = keyframes_.size();
        if (N < 2) return;

        // Use RAW SLAM odometry as fixed measurements (never modified by optimization).
        // Previously re-read from keyframes_[i].pose each call, which caused
        // odometry constraints to "drift" with optimization — preventing loop
        // closure corrections from accumulating (max_update=0 bug).
        const auto& poses_original = poses_raw_odom_;

        int max_iterations = 15;
        for (int iter = 0; iter < max_iterations; iter++) {
            int dim = 6 * N;
            std::vector<Eigen::Triplet<float>> triplets;
            Eigen::VectorXf b = Eigen::VectorXf::Zero(dim);
            float total_err = 0;

            // Odometry edges: constrain current relative to match original relative
            for (int i = 0; i < N - 1; i++) {
                Eigen::Matrix4f T_i = keyframe_poses_[i];          // current mutable
                Eigen::Matrix4f T_j = keyframe_poses_[i+1];        // current mutable
                Eigen::Matrix4f T_meas = poses_original[i].inverse() * poses_original[i+1]; // fixed odometry

                // Error: log( (T_i^{-1} * T_j)^{-1} * T_meas )
                Eigen::Matrix<float, 6, 1> e = se3Log(T_j.inverse() * T_i * T_meas);

                Eigen::Matrix<float, 6, 6> info = Eigen::Matrix<float, 6, 6>::Identity() * 500.0;

                int idx_i = 6 * i, idx_j = 6 * (i + 1);
                addJacobianBlock(triplets, b, idx_i, idx_j, info, e);
                total_err += e.norm();
            }

            // Loop closure edges
            for (const auto& edge : loop_edges_) {
                int i = edge.idx_from, j = edge.idx_to;
                if (i >= N || j >= N || i == j) continue;

                Eigen::Matrix4f T_i = keyframe_poses_[i];
                Eigen::Matrix4f T_j = keyframe_poses_[j];
                Eigen::Matrix4f T_meas = edge.transform;

                // Error: log( (T_i^{-1} * T_j)^{-1} * T_meas )
                Eigen::Matrix<float, 6, 1> e = se3Log(T_j.inverse() * T_i * T_meas);

                Eigen::Matrix<float, 6, 6> info = Eigen::Matrix<float, 6, 6>::Identity() * 500.0;  // same weight as odometry

                int idx_i = 6 * i, idx_j = 6 * j;
                addJacobianBlock(triplets, b, idx_i, idx_j, info, e);
                total_err += e.norm();
            }

            // Fix first node (anchor) — strong constraint
            for (int k = 0; k < 6; k++) {
                triplets.push_back(Eigen::Triplet<float>(k, k, 1e12));
            }
            b.segment<6>(0).setZero();

            // Solve sparse linear system
            Eigen::SparseMatrix<float> H(dim, dim);
            H.setFromTriplets(triplets.begin(), triplets.end());

            Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> solver;
            solver.compute(H);
            if (solver.info() != Eigen::Success) {
                ROS_WARN("[LoopClosure] Pose graph solver failed at iteration %d", iter);
                break;
            }
            Eigen::VectorXf dx = solver.solve(-b);

            // Update poses
            float max_update = 0;
            for (int i = 1; i < N; i++) {  // skip first node (fixed)
                Eigen::Matrix<float, 6, 1> dxi = dx.segment<6>(6 * i);
                keyframe_poses_[i] = keyframe_poses_[i] * se3Exp(dxi);
                max_update = std::max(max_update, dxi.norm());
            }

            // Update keyframe poses
            for (int i = 0; i < N; i++) {
                keyframes_[i].pose = keyframe_poses_[i];
            }

            ROS_INFO("[LoopClosure]   iter %d: max_update=%.4f, errors: odom=%.4f, loop=%.4f",
                     iter, max_update,
                     (b.segment(0, 6*(N-1))).norm(),
                     (b.segment(6*(N-1), b.size()-6*(N-1))).norm());

            if (max_update < 1e-4) {
                ROS_INFO("[LoopClosure] Pose graph converged in %d iterations", iter+1);
                break;
            }
        }

        // Publish corrected odometry (latest pose)
        nav_msgs::Odometry corrected_odom;
        corrected_odom.header.stamp = ros::Time::now();
        corrected_odom.header.frame_id = "camera_init";
        corrected_odom.pose.pose = matrixToPose(keyframe_poses_.back());
        pub_corrected_odom_.publish(corrected_odom);
    }

    void addJacobianBlock(std::vector<Eigen::Triplet<float>>& triplets, Eigen::VectorXf& b,
                          int idx_i, int idx_j,
                          const Eigen::Matrix<float, 6, 6>& info, const Eigen::Matrix<float, 6, 1>& error) {
        // J_i = -I_6 (approx), J_j = I_6 (approx)
        // H_ii += J_i^T * info * J_i = info
        // H_ij += J_i^T * info * J_j = -info
        // H_ji += J_j^T * info * J_i = -info
        // H_jj += J_j^T * info * J_j = info
        // b_i += J_i^T * info * error = -info * error
        // b_j += J_j^T * info * error = info * error

        Eigen::Matrix<float, 6, 6> H_block = info;
        Eigen::Matrix<float, 6, 1> b_block_i = -info * error;
        Eigen::Matrix<float, 6, 1> b_block_j = info * error;

        for (int r = 0; r < 6; r++) {
            for (int c = 0; c < 6; c++) {
                float val = H_block(r, c);
                if (std::abs(val) > 1e-10) {
                    triplets.push_back(Eigen::Triplet<float>(idx_i + r, idx_i + c, val));
                    triplets.push_back(Eigen::Triplet<float>(idx_i + r, idx_j + c, -val));
                    triplets.push_back(Eigen::Triplet<float>(idx_j + r, idx_i + c, -val));
                    triplets.push_back(Eigen::Triplet<float>(idx_j + r, idx_j + c, val));
                }
            }
            b(idx_i + r) += b_block_i(r);
            b(idx_j + r) += b_block_j(r);
        }
    }

    // ========================================================================
    // Visualization
    // ========================================================================
    void publishCorrectedPath() {
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "camera_init";

        for (const auto& kf : keyframes_) {
            geometry_msgs::PoseStamped ps;
            ps.header = path.header;
            ps.pose = matrixToPose(kf.pose);
            path.poses.push_back(ps);
        }
        pub_corrected_path_.publish(path);
    }

    void publishLoopMarkers() {
        visualization_msgs::MarkerArray markers;
        int id = 0;

        for (const auto& edge : loop_edges_) {
            if (edge.idx_from >= (int)keyframes_.size() || edge.idx_to >= (int)keyframes_.size()) continue;

            visualization_msgs::Marker edge_marker;
            edge_marker.header.frame_id = "camera_init";
            edge_marker.header.stamp = ros::Time::now();
            edge_marker.action = visualization_msgs::Marker::ADD;
            edge_marker.type = visualization_msgs::Marker::LINE_STRIP;
            edge_marker.ns = "loop_edges";
            edge_marker.id = id++;
            edge_marker.scale.x = 0.05;
            edge_marker.color.r = 0.0; edge_marker.color.g = 1.0; edge_marker.color.b = 0.0;
            edge_marker.color.a = 1.0;

            geometry_msgs::Point p1, p2;
            Eigen::Vector3f t1 = keyframes_[edge.idx_from].pose.block<3,1>(0,3);
            Eigen::Vector3f t2 = keyframes_[edge.idx_to].pose.block<3,1>(0,3);
            p1.x = t1(0); p1.y = t1(1); p1.z = t1(2);
            p2.x = t2(0); p2.y = t2(1); p2.z = t2(2);
            edge_marker.points.push_back(p1);
            edge_marker.points.push_back(p2);
            markers.markers.push_back(edge_marker);
        }
        pub_loop_markers_.publish(markers);
    }

    // ========================================================================
    // Members
    // ========================================================================
    ros::NodeHandle nh_;
    ros::Subscriber sub_odom_, sub_cloud_;
    ros::Publisher pub_corrected_path_, pub_loop_markers_, pub_corrected_odom_;

    std::mutex mtx_;
    std::thread loop_thread_;

    // Latest data
    Eigen::Matrix4f latest_pose_ = Eigen::Matrix4f::Identity();
    double latest_odom_time_ = 0.0;
    pcl::PointCloud<PointType>::Ptr latest_cloud_sc_;
    pcl::PointCloud<PointType>::Ptr latest_cloud_icp_;

    // Keyframe database
    std::vector<KeyFrame> keyframes_;
    std::vector<Eigen::Matrix4f> keyframe_poses_;  // mutable copy for optimization
    std::vector<Eigen::Matrix4f> poses_raw_odom_;   // raw SLAM odometry (fixed, never optimized)
    std::vector<LoopEdge> loop_edges_;

    // ScanContext manager
    SCManager sc_manager_;

    // Downsample filters
    pcl::VoxelGrid<PointType> down_filter_sc_;
    pcl::VoxelGrid<PointType> down_filter_icp_;

    // Parameters
    float kf_dist_thresh_, kf_angle_thresh_;
    float icp_fitness_thresh_, loop_freq_;
    int num_exclude_recent_;
    float sc_dist_thresh_, sc_max_radius_;
};

// ============================================================================
// Main
// ============================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_loop_closure");

    ros::NodeHandle nh("~");
    LoopClosureNode lc_node(nh);

    ros::spin();

    return 0;
}
