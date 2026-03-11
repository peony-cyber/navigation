#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vel_forwarder/msg/vel.hpp>
#include <cmath>
#include <geometry_msgs/msg/vector3.hpp>

namespace vel_forwarder
{
    class Forwarder : public rclcpp::Node
    {
    public:
        Forwarder();  // 只声明，不实现
        
    private:
        void cmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr rep_sub_;
        float rep_vx_ = 0.0f;
        float rep_vy_ = 0.0f;
    
        // 假设你有一个知道目标点的机制，如果没有，需要通过订阅或动作服务器传入
        float target_x_ = 10.0f; // 示例目标X
        float target_y_ = 5.0f;  // 示例目标Y
        float neighborhood_radius_ = 1.0f; // 目标1米内关闭避障
        
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
        // 改成正确的类型
        rclcpp::Publisher<vel_forwarder::msg::Vel>::SharedPtr pub_ly_vel;
    };
} // namespace vel_forwarder