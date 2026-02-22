#ifndef TRAJ_FOLLOWER_H
#define TRAJ_FOLLOWER_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <chrono>

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>

class TrajFollower
{
public:
    TrajFollower(rclcpp::Node &node, double P, double I, double D, double speed_ratio = 0.9) 
        : Kp(P), Ki(I), Kd(D), speed_ratio(speed_ratio), node_(node)
    {
        this->cmd_pub = node.create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        this->global_path_sub = node.create_subscription<nav_msgs::msg::Path>(
            "/sPath", 5, std::bind(&TrajFollower::global_path_callback, this, std::placeholders::_1));
        this->path_frame = "map";
        this->plan_timer = node.create_wall_timer(
            std::chrono::duration<double>(1.0 / 30),
            std::bind(&TrajFollower::manager, this));
        this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node.get_clock());
        this->transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }   

    ~TrajFollower() {}

    void manager();
    bool get_transform();
    void tranform_path();
    void pub_cmd();
    void follow();
    void find_nearest_node();
    void global_path_callback(const nav_msgs::msg::Path::ConstSharedPtr &msg);

private:
    double Kp;
    double Ki;
    double Kd;
    double speed_ratio;
    std::vector<Eigen::Vector2d> path;
    std::vector<Eigen::Vector2d> local_path;
    Eigen::Vector2d pre_node;
    Eigen::Vector2d follow_speed;
    Eigen::MatrixXd map2baselink;
    int pre_i;
    int next_i;
    bool path_geted = false;
    std::string path_frame;
    
    rclcpp::Node &node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::TimerBase::SharedPtr plan_timer;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
};

#endif