#pragma once
#include "robot_model.h"
#include "rclcpp/rclcpp.hpp"
#include <nlopt.hpp>
#include <vector>
#include <iostream>
#include <random>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <cmath>

namespace mpc_follower
{
    class MpcController : public rclcpp::Node
    {
        public:
        explicit MpcController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        Control computeControl();
        // void updateRobotState(const Control &u,MpcController *mpc);
        void publishVisualization();
        void configureGlobalOptimizer();
        void configureLocalOptimizer();
        void controlLoop();
        void cmdVelPublish(const Control &u);
        // void updateVelocity(const nav_msgs::msg::Odometry::SharedPtr msg);
        void updateRobotState(const nav_msgs::msg::Odometry::SharedPtr msg);
        void onReferencePath(const nav_msgs::msg::Path::SharedPtr msg);
        void detectObstacles(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        // bool get_transform();
        int findNearestWaypoint(const RobotState &state, size_t start_index);
        static double objectiveFunction(const std::vector<double> &x,
         std::vector<double> &grad,void *data);           // 目标函数

        static double accelerationConstraints(const std::vector<double> &x,
         std::vector<double> &grad,
          void *data);      // 加速度约束函数

        static double turningRadiusConstraint(const std::vector<double> &x,std::vector<double> &grad,void *data);  // 最小转弯半径约束函数
            // 车辆运动学模型
        RobotState predictState(const RobotState &current, const Control &u, double dt, MpcController *mpc);

        
        private:
        struct Params {
        double Ts, MaxLinear, MinLinear, MaxAngular, MinAngular;
        double MaxAccelLinear, MinAccelLinear, MaxAccelAngular, MinAccelAngular;
        double MinR;
        double Wx, Wy, Wtheta, Wv, Womega, WRadius;
        double Rv, Romega;
        double OptTolerance;
        int    GlobalMaxEval, LocalMaxEval;
        double GlobalInitNoise;
        double ObstacleInflation, Wobs;
        int    N;   // 预测步数
        bool   UseTurningRadius; // 是否启用转弯半径约束（对全向平台通常禁用）
        } params_;

        void loadParameters();   // 加载参数

        rclcpp::TimerBase::SharedPtr timer_;

        //rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_; //根据实际修改
        //rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;

        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        /*----------------------监听机器人位姿-----------------------*/
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

        RobotState current_state_;
        Control previous_control_;
        std::vector<PathPoint> reference_path_;

        /*---------------------------------优化器相关 ---------------------------------*/
        nlopt::opt global_optimizer_; // 全局优化器
        nlopt::opt local_optimizer_;  // 局部优化器
        std::vector<double> opt_result_;
        double opt_fval_;

        size_t last_index_{0};
        std::mt19937 rng_; // 随机数生成器

        // 动态障碍列表（由感知模块或订阅回调更新）
        std::vector<DynamicObstacle> dynamic_obstacles_;

    };
};