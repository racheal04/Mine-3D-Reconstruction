// ScanContext + GTSAM Loop Closure for FAST-LIO2
// Pattern: SC-LIO-SAM (mapOptmization.cpp) → adapted to subscribe FAST-LIO2 topics

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
#include <Eigen/Geometry>
#include <mutex>
#include <thread>
#include <deque>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>

#include "Scancontext.h"

using PointType = pcl::PointXYZI;

// ============================================================================
// Helpers
// ============================================================================
gtsam::Pose3 eigenToGtsam(const Eigen::Matrix4f& T) {
    Eigen::Matrix3d R = T.block<3,3>(0,0).cast<double>();
    Eigen::Vector3d t = T.block<3,1>(0,3).cast<double>();
    return gtsam::Pose3(gtsam::Rot3(R), t);
}

Eigen::Matrix4f gtsamToEigen(const gtsam::Pose3& pose) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = pose.rotation().matrix().cast<float>();
    T.block<3,1>(0,3) = pose.translation().cast<float>();
    return T;
}

geometry_msgs::Pose eigenToPoseMsg(const Eigen::Matrix4f& T) {
    geometry_msgs::Pose p;
    p.position.x = T(0,3); p.position.y = T(1,3); p.position.z = T(2,3);
    Eigen::Quaternionf q(T.block<3,3>(0,0));
    p.orientation.w = q.w(); p.orientation.x = q.x();
    p.orientation.y = q.y(); p.orientation.z = q.z();
    return p;
}

// ============================================================================
// Keyframe
// ============================================================================
struct KeyFrame {
    int index;
    Eigen::Matrix4f pose;
    pcl::PointCloud<PointType>::Ptr cloud;
    double timestamp;
};

// ============================================================================
// LoopClosureNode
// ============================================================================
class LoopClosureNode {
public:
    LoopClosureNode(ros::NodeHandle& nh) : nh_(nh) {
        sub_odom_ = nh_.subscribe("/Odometry", 100, &LoopClosureNode::odomCallback, this);
        sub_cloud_ = nh_.subscribe("/cloud_registered", 100, &LoopClosureNode::cloudCallback, this);

        pub_corrected_path_ = nh_.advertise<nav_msgs::Path>("/corrected_path", 10);
        pub_loop_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/loop_constraints", 10);

        // Params
        nh_.param<float>("keyframe_dist_threshold", kf_dist_thresh_, 0.5);
        nh_.param<float>("keyframe_angle_threshold", kf_angle_thresh_, 0.3);
        nh_.param<float>("icp_fitness_threshold", icp_fitness_thresh_, 0.3);
        nh_.param<float>("loop_frequency", loop_freq_, 1.0);

        // Filters
        down_filter_sc_.setLeafSize(0.4, 0.4, 0.4);   // ScanContext
        down_filter_icp_.setLeafSize(0.3, 0.3, 0.3);   // ICP

        // Tune ScanContext for mine tunnels
        sc_manager_.PC_MAX_RADIUS = 50.0;
        sc_manager_.SC_DIST_THRES = 0.22;
        sc_manager_.NUM_EXCLUDE_RECENT = 30;
        sc_manager_.LIDAR_HEIGHT = 0.0;

        // Initialize GTSAM ISAM2
        gtsam::ISAM2Params isam_params;
        isam_params.relinearizeThreshold = 0.01;  // aggressive relinearization
        isam_params.relinearizeSkip = 1;
        isam_ = new gtsam::ISAM2(isam_params);
        first_pose_fixed_ = false;

        ROS_INFO("LoopClosureNode (GTSAM) started. kf_dist=%.2f, icp_thresh=%.3f, sc_thresh=%.3f",
                 kf_dist_thresh_, icp_fitness_thresh_, sc_manager_.SC_DIST_THRES);

        loop_thread_ = std::thread(&LoopClosureNode::loopClosureThread, this);
    }

    ~LoopClosureNode() {
        if (loop_thread_.joinable()) loop_thread_.join();
        delete isam_;
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        Eigen::Affine3f aff;
        aff.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
        Eigen::Quaternionf q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        aff.linear() = q.toRotationMatrix();
        latest_pose_ = aff.matrix();
        latest_odom_time_ = msg->header.stamp.toSec();
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        std::lock_guard<std::recursive_mutex> lock(mtx_);

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

        if (shouldAddKeyframe()) {
            addKeyframe();
        }
    }

    bool shouldAddKeyframe() {
        if (keyframes_.empty()) return true;
        auto& last = keyframes_.back().pose;
        float dist = (latest_pose_.block<3,1>(0,3) - last.block<3,1>(0,3)).norm();
        Eigen::Quaternionf q1(latest_pose_.block<3,3>(0,0)), q2(last.block<3,3>(0,0));
        float angle = q1.angularDistance(q2);
        return dist > kf_dist_thresh_ || angle > kf_angle_thresh_;
    }

    void addKeyframe() {
        std::lock_guard<std::recursive_mutex> lock(mtx_);

        KeyFrame kf;
        kf.index = (int)keyframes_.size();
        kf.pose = latest_pose_;
        kf.timestamp = latest_odom_time_;
        kf.cloud = latest_cloud_icp_;
        keyframes_.push_back(kf);
        odom_poses_.push_back(latest_pose_);  // FIXED original odometry

        // Store ScanContext descriptor
        sc_manager_.makeAndSaveScancontextAndKeys(*latest_cloud_sc_);

        // Add odometry factor to GTSAM graph
        addOdomFactor();

        if (keyframes_.size() <= 5 || keyframes_.size() % 20 == 0) {
            ROS_INFO("[LoopClosure-GTSAM] Keyframes: %zu", keyframes_.size());
        }
    }

    // ===== GTSAM Factor Graph =====
    void addOdomFactor() {
        int idx = (int)keyframes_.size() - 1;

        // FIXED odometry measurement (original trajectory)
        // FIXED measurement from original odometry (never modified by ISAM2)
        gtsam::Pose3 pose_to_odom = eigenToGtsam(odom_poses_[idx]);

        if (idx == 0) {
            auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());
            graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, pose_to_odom, prior_noise));
            estimates_.insert(0, pose_to_odom);
            first_pose_fixed_ = true;
        } else {
            gtsam::Pose3 pose_from_odom = eigenToGtsam(odom_poses_[idx - 1]);
            gtsam::Pose3 rel_odom = pose_from_odom.between(pose_to_odom);

            auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 0.5, 0.5, 0.5, 0.3, 0.3, 0.3).finished());
            graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(idx - 1, idx, rel_odom, odom_noise));

            // Initial guess: propagate from LAST CORRECTED pose (not raw odometry)
            int prev_N = (int)keyframe_poses_corrected_.size();
            if (prev_N > idx - 1) {
                gtsam::Pose3 prev_corrected = eigenToGtsam(keyframe_poses_corrected_[idx - 1]);
                gtsam::Pose3 guess = prev_corrected * rel_odom;
                estimates_.insert(idx, guess);
            } else {
                estimates_.insert(idx, pose_to_odom);
            }
        }

        if (idx > 0 && idx % 5 == 0) {
            optimizeGtsam();
        }
    }

    void addLoopFactor(int from_idx, int to_idx, const Eigen::Matrix4f& T_icp) {
        gtsam::Pose3 pose_from = eigenToGtsam(odom_poses_[from_idx]);
        gtsam::Pose3 pose_to = eigenToGtsam(odom_poses_[to_idx]);
        gtsam::Pose3 T_icp_gtsam = eigenToGtsam(T_icp);

        // Loop closure: these two keyframes are the SAME physical place.
        // Constraint: pose_from ≈ pose_to (identity relative).
        // Use ICP rotation only (ignore ICP translation — that IS the drift).
        gtsam::Rot3 R_icp = T_icp_gtsam.rotation();
        gtsam::Pose3 rel_constraint(R_icp, gtsam::Point3(0, 0, 0));

        // Robust noise model (Cauchy kernel prevents false positives from destroying the graph)
        auto robust_noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1.0),
            gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()));

        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(from_idx, to_idx, rel_constraint, robust_noise));

        // Log pre-optimization poses (from FIXED odometry)
        Eigen::Vector3f t_from_pre = odom_poses_[from_idx].block<3,1>(0,3);
        Eigen::Vector3f t_to_pre = odom_poses_[to_idx].block<3,1>(0,3);
        float dz_pre = t_from_pre.z() - t_to_pre.z();
        ROS_INFO("[LoopClosure-GTSAM] Loop factor added: %d <-> %d (pre-opt dz=%.3f)", from_idx, to_idx, dz_pre);

        // Also store for visualization
        LoopEdge edge;
        edge.idx_from = from_idx;
        edge.idx_to = to_idx;
        loop_edges_.push_back(edge);

        optimizeGtsam();
        publishResults();

        // Log post-optimization poses
        if ((int)keyframe_poses_corrected_.size() > std::max(from_idx, to_idx)) {
            Eigen::Vector3f t_from_post = keyframe_poses_corrected_[from_idx].block<3,1>(0,3);
            Eigen::Vector3f t_to_post = keyframe_poses_corrected_[to_idx].block<3,1>(0,3);
            float dz_post = t_from_post.z() - t_to_post.z();
            ROS_INFO("[LoopClosure-GTSAM] Post-opt: dz=%.3f (was %.3f), delta=%.3f",
                     dz_post, dz_pre, dz_pre - dz_post);
        }
    }

    void optimizeGtsam() {
        try {
            gtsam::ISAM2Result result = isam_->update(graph_, estimates_);
            graph_.resize(0);
            estimates_.clear();

            if (loop_edges_.size() > 0) {
                // Extra iterations to spread loop constraint through the whole graph
                for (int k = 0; k < 100; k++) isam_->update();
            }

            isam_current_estimate_ = isam_->calculateEstimate();

            int N = isam_current_estimate_.size();
            if (N > 0) {
                keyframe_poses_corrected_.resize(N);
                for (int i = 0; i < N; i++) {
                    keyframe_poses_corrected_[i] = gtsamToEigen(
                        isam_current_estimate_.at<gtsam::Pose3>(i));
                }
            }
        } catch (const std::exception& e) {
            ROS_WARN("[LoopClosure-GTSAM] Optimization failed: %s", e.what());
        }
    }

    // ===== Loop Detection Thread =====
    void loopClosureThread() {
        ros::Rate rate(loop_freq_);
        while (ros::ok()) {
            rate.sleep();
            detectLoop();
        }
    }

    void detectLoop() {
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        if (keyframes_.size() < 35) return;

        auto result = sc_manager_.detectLoopClosureID();
        int loop_idx = result.first;
        float yaw_diff = result.second;
        if (loop_idx == -1) return;

        int cur_idx = (int)keyframes_.size() - 1;
        ROS_INFO("[LoopClosure-GTSAM] SC candidate: %d <-> %d (yaw=%.1f deg)",
                 cur_idx, loop_idx, yaw_diff * 180.0 / M_PI);

        // ICP verification
        Eigen::Matrix4f T_icp;
        if (!verifyICP(loop_idx, cur_idx, yaw_diff, T_icp)) {
            ROS_INFO("[LoopClosure-GTSAM] ICP rejected.");
            return;
        }

        ROS_INFO("[LoopClosure-GTSAM] ICP passed (fitness OK). Adding loop constraint.");
        addLoopFactor(cur_idx, loop_idx, T_icp);
    }

    bool verifyICP(int loop_idx, int cur_idx, float yaw_diff, Eigen::Matrix4f& T_icp) {
        pcl::PointCloud<PointType>::Ptr cloud_cur(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr cloud_loop(new pcl::PointCloud<PointType>());

        // Gather keyframe clouds (world-frame, no transform needed)
        int sr = 5;
        for (int i = cur_idx - sr; i <= cur_idx; i++) {
            if (i < 0 || i >= (int)keyframes_.size()) continue;
            *cloud_cur += *keyframes_[i].cloud;
        }
        int loop_end = std::min(loop_idx + 50, (int)keyframes_.size() - 1);
        for (int i = 0; i <= loop_end; i++) {
            *cloud_loop += *keyframes_[i].cloud;
        }

        // Re-downsample combined clouds for ICP
        pcl::VoxelGrid<PointType> vg;
        vg.setLeafSize(0.2, 0.2, 0.2);
        pcl::PointCloud<PointType>::Ptr cur_ds(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr loop_ds(new pcl::PointCloud<PointType>());
        vg.setInputCloud(cloud_cur);  vg.filter(*cur_ds);
        vg.setInputCloud(cloud_loop); vg.filter(*loop_ds);

        if (cur_ds->size() < 500 || loop_ds->size() < 500) return false;

        // 2D ICP pre-alignment
        pcl::PointCloud<PointType>::Ptr cur_2d(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr loop_2d(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*cur_ds, *cur_2d);
        pcl::copyPointCloud(*loop_ds, *loop_2d);
        for (auto& p : cur_2d->points) p.z = 0;
        for (auto& p : loop_2d->points) p.z = 0;

        pcl::IterativeClosestPoint<PointType, PointType> icp_2d;
        icp_2d.setMaxCorrespondenceDistance(20.0);
        icp_2d.setMaximumIterations(50);
        icp_2d.setRANSACIterations(5);

        Eigen::Matrix4f init = Eigen::Matrix4f::Identity();
        init.block<3,3>(0,0) = Eigen::AngleAxisf(yaw_diff, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        pcl::PointCloud<PointType> tmp;
        icp_2d.setInputSource(cur_2d); icp_2d.setInputTarget(loop_2d);
        icp_2d.align(tmp, init);

        Eigen::Matrix4f init_3d = icp_2d.hasConverged() ? icp_2d.getFinalTransformation() : init;
        init_3d(2,3) = 0;  // don't trust Z from 2D ICP

        // 3D ICP refinement
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(30.0);
        icp.setMaximumIterations(200);
        icp.setRANSACIterations(10);
        icp.setInputSource(cur_ds); icp.setInputTarget(loop_ds);
        pcl::PointCloud<PointType> aligned;
        icp.align(aligned, init_3d);

        if (!icp.hasConverged()) return false;
        if (icp.getFitnessScore() > icp_fitness_thresh_) return false;

        T_icp = icp.getFinalTransformation();
        ROS_INFO("[LoopClosure-GTSAM] ICP fitness=%.4f", icp.getFitnessScore());
        return true;
    }

    // ===== Publish =====
    void publishResults() {
        if (keyframe_poses_corrected_.empty()) return;

        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "camera_init";
        for (size_t i = 0; i < keyframe_poses_corrected_.size(); i++) {
            geometry_msgs::PoseStamped ps;
            ps.header = path.header;
            ps.pose = eigenToPoseMsg(keyframe_poses_corrected_[i]);
            path.poses.push_back(ps);
        }
        pub_corrected_path_.publish(path);

        // Loop markers
        if (loop_edges_.size() > 0) {
            visualization_msgs::MarkerArray markers;
            visualization_msgs::Marker edge_marker;
            edge_marker.header = path.header;
            edge_marker.type = visualization_msgs::Marker::LINE_LIST;
            edge_marker.ns = "loop_edges";
            edge_marker.id = 0;
            edge_marker.scale.x = 0.05;
            edge_marker.color.r = 0; edge_marker.color.g = 1.0; edge_marker.color.b = 0;
            edge_marker.color.a = 1.0;
            edge_marker.action = visualization_msgs::Marker::ADD;

            for (const auto& e : loop_edges_) {
                if (e.idx_from >= (int)keyframe_poses_corrected_.size() ||
                    e.idx_to >= (int)keyframe_poses_corrected_.size()) continue;
                geometry_msgs::Point p;
                auto t1 = keyframe_poses_corrected_[e.idx_from].block<3,1>(0,3);
                auto t2 = keyframe_poses_corrected_[e.idx_to].block<3,1>(0,3);
                p.x = t1(0); p.y = t1(1); p.z = t1(2); edge_marker.points.push_back(p);
                p.x = t2(0); p.y = t2(1); p.z = t2(2); edge_marker.points.push_back(p);
            }
            markers.markers.push_back(edge_marker);
            pub_loop_markers_.publish(markers);
        }
    }

    // ===== Members =====
    ros::NodeHandle nh_;
    ros::Subscriber sub_odom_, sub_cloud_;
    ros::Publisher pub_corrected_path_, pub_loop_markers_;

    std::recursive_mutex mtx_;
    std::thread loop_thread_;

    Eigen::Matrix4f latest_pose_ = Eigen::Matrix4f::Identity();
    double latest_odom_time_ = 0.0;
    pcl::PointCloud<PointType>::Ptr latest_cloud_sc_, latest_cloud_icp_;

    std::vector<KeyFrame> keyframes_;
    std::vector<Eigen::Matrix4f> odom_poses_;       // FIXED original odometry (never modified)
    std::vector<Eigen::Matrix4f> keyframe_poses_corrected_;

    struct LoopEdge { int idx_from, idx_to; };
    std::vector<LoopEdge> loop_edges_;

    SCManager sc_manager_;
    pcl::VoxelGrid<PointType> down_filter_sc_, down_filter_icp_;

    float kf_dist_thresh_, kf_angle_thresh_, icp_fitness_thresh_, loop_freq_;

    // GTSAM
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values estimates_;
    gtsam::ISAM2* isam_;
    gtsam::Values isam_current_estimate_;
    bool first_pose_fixed_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_loop_closure");
    ros::NodeHandle nh("~");
    LoopClosureNode node(nh);
    ros::spin();
    return 0;
}
