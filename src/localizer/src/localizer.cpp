#include <memory>
#include <mutex>
#include <string>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;

class LocalizerNode : public rclcpp::Node {
public:
    LocalizerNode() : Node("pcl_localizer"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // --- 1. 参数声明与初始化 ---
        this->declare_parameter("map_voxel_size", 0.05);
        this->declare_parameter("scan_voxel_size", 0.2);
        this->declare_parameter("localization_th", 0.95);
        this->declare_parameter("map_pcd_path", "/home/hustlyrm/slam_source/odom_ws/pcd/map.pcd");

        map_voxel_size_ = this->get_parameter("map_voxel_size").as_double();
        scan_voxel_size_ = this->get_parameter("scan_voxel_size").as_double();
        localization_th_ = this->get_parameter("localization_th").as_double();
        std::string map_pcd_path = this->get_parameter("map_pcd_path").as_string();

        // --- 2. 内存初始化 ---
        global_map_ = std::make_shared<PointCloudXYZI>();
        current_scan_ = std::make_shared<PointCloudXYZI>();
        T_map_to_odom_ = Eigen::Matrix4f::Identity();

        // --- 3. 加载全局地图 ---
        if (!map_pcd_path.empty()) {
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_pcd_path, *global_map_) == -1) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load PCD: %s", map_pcd_path.c_str());
            } else {
                // downsample(global_map_, global_map_, map_voxel_size_);
                has_map_ = true;
                RCLCPP_INFO(this->get_logger(), "Loaded map with %zu points", global_map_->size());
            }
        }

        // --- 4. 订阅与广播 ---
        // sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     "/cloud_registered", 10, std::bind(&LocalizerNode::cbScan, this, std::placeholders::_1));
        sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/dlio/odom_node/pointcloud/deskewed", 10, std::bind(&LocalizerNode::cbScan, this, std::placeholders::_1));
        sub_init_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&LocalizerNode::cbInitialPose, this, std::placeholders::_1));
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // --- 5. 定时器 (2.0s 执行一次定位修正) ---
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2000), std::bind(&LocalizerNode::localizationCallback, this));

        RCLCPP_INFO(this->get_logger(), "ROS 2 Localizer Initialized.");
    }

private:
    // 回调：接收点云扫描
    void cbScan(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Received new scan with %u points", msg->width * msg->height);
        std::lock_guard<std::mutex> lock(data_mutex_);
        pcl::fromROSMsg(*msg, *current_scan_);
        has_scan_ = true;
    }

    // 回调：接收手动初始位姿 (Rviz 2D Pose Estimate)
    void cbInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;

        Eigen::Quaternionf quat(q.w, q.x, q.y, q.z);
        T_map_to_odom_.setIdentity();
        T_map_to_odom_.block<3,3>(0,0) = quat.toRotationMatrix();
        T_map_to_odom_.block<3,1>(0,3) = Eigen::Vector3f(p.x, p.y, p.z);
        
        has_initial_pose_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial pose received.");
    }

    void downsample(const PointCloudXYZI::Ptr& input, PointCloudXYZI::Ptr& output, double leaf_size) {
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(input);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.filter(*output);
    }

    void localizationCallback() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!has_map_ || !has_scan_ || !has_initial_pose_) {
            if (!has_map_) RCLCPP_WARN(this->get_logger(), "No map loaded.");
            if (!has_scan_) RCLCPP_WARN(this->get_logger(), "No scan received.");
            if (!has_initial_pose_) RCLCPP_WARN(this->get_logger(), "No initial pose set.");
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for data...");
            return;
        }

        // 获取当前 odom -> base_link 的变换
        geometry_msgs::msg::TransformStamped tf_odom_base;
        try {
            tf_odom_base = tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }

        Eigen::Quaternionf q_ob(
            tf_odom_base.transform.rotation.w, tf_odom_base.transform.rotation.x,
            tf_odom_base.transform.rotation.y, tf_odom_base.transform.rotation.z);
        Eigen::Matrix4f T_odom_to_base = Eigen::Matrix4f::Identity();
        T_odom_to_base.block<3,3>(0,0) = q_ob.toRotationMatrix();
        T_odom_to_base.block<3,1>(0,3) = Eigen::Vector3f(
            tf_odom_base.transform.translation.x, tf_odom_base.transform.translation.y, tf_odom_base.transform.translation.z);

        // 初始猜测位姿 (Map -> Base)
        Eigen::Matrix4f initial_guess = T_map_to_odom_ * T_odom_to_base;

        PointCloudXYZI::Ptr scan_down(new PointCloudXYZI);
        downsample(current_scan_, scan_down, scan_voxel_size_);

        // --- 配准逻辑 ---
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
        icp.setInputSource(scan_down);
        icp.setInputTarget(global_map_);
        icp.setMaximumIterations(30);
        icp.setMaxCorrespondenceDistance(2.0);

        PointCloudXYZI aligned;
        icp.align(aligned, initial_guess);
        RCLCPP_INFO(this->get_logger(), "ICP fitness score: %f", icp.getFitnessScore());

        Eigen::Matrix4f final_transformation;
        bool success = false;

        if (icp.hasConverged() && icp.getFitnessScore() < localization_th_) {
            final_transformation = icp.getFinalTransformation();
            success = true;
        } else {
            // NDT 粗配准逻辑
            pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
            ndt.setResolution(2.0);
            ndt.setInputSource(scan_down);
            ndt.setInputTarget(global_map_);
            ndt.align(aligned, initial_guess);
            RCLCPP_INFO(this->get_logger(), "NDT fitness score: %f", ndt.getFitnessScore());

            if (ndt.hasConverged()) {
                // NDT 后再接一轮 ICP 精修
                RCLCPP_INFO(this->get_logger(), "NDT converged, refining with ICP...");
                icp.align(aligned, ndt.getFinalTransformation());
                if (icp.hasConverged() && icp.getFitnessScore() < localization_th_) {
                    final_transformation = icp.getFinalTransformation();
                    success = true;
                    RCLCPP_INFO(this->get_logger(), "Refined ICP fitness score: %f", icp.getFitnessScore());
                }
                else {
                    RCLCPP_WARN(this->get_logger(), "Refined ICP failed to converge or fitness too high.");
                    RCLCPP_WARN(this->get_logger(), "ICP fitness score: %f", icp.getFitnessScore());
                }
            }
        }

        // --- 更新并发布 TF ---
        if (success) {
            T_map_to_odom_ = final_transformation * T_odom_to_base.inverse();

            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "map";
            t.child_frame_id = "odom";

            Eigen::Matrix3f R = T_map_to_odom_.block<3,3>(0,0);
            Eigen::Quaternionf q_final(R);
            t.transform.translation.x = T_map_to_odom_(0,3);
            t.transform.translation.y = T_map_to_odom_(1,3);
            t.transform.translation.z = T_map_to_odom_(2,3);
            t.transform.rotation.x = q_final.x();
            t.transform.rotation.y = q_final.y();
            t.transform.rotation.z = q_final.z();
            t.transform.rotation.w = q_final.w();

            tf_broadcaster_->sendTransform(t);
        }
    }

    // 成员变量
    double map_voxel_size_, scan_voxel_size_, localization_th_;
    bool has_map_ = false, has_scan_ = false, has_initial_pose_ = false;
    Eigen::Matrix4f T_map_to_odom_;
    std::mutex data_mutex_;

    PointCloudXYZI::Ptr global_map_, current_scan_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// --- Main 函数 ---
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}