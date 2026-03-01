#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vel_forwarder/msg/vel.hpp>

namespace vel_forwarder
{
    class Forwarder : public rclcpp::Node
    {
    public:
        Forwarder();  // 只声明，不实现
        
    private:
        void cmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
        // 改成正确的类型
        rclcpp::Publisher<vel_forwarder::msg::Vel>::SharedPtr pub_ly_vel;
    };
} // namespace vel_forwarder