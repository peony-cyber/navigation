// main.cpp
#include <rclcpp/rclcpp.hpp>
#include "TrajFollower.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    double Kp, Ki, Kd, speed_ratio;
    auto nh = rclcpp::Node::make_shared("path_follower_node");
    
    nh->declare_parameter<double>("Kp", 2.7);
    nh->declare_parameter<double>("Ki", 0.0);
    nh->declare_parameter<double>("Kd", 0.0);
    nh->declare_parameter<double>("speed_ratio", 0.5);
    nh->declare_parameter<std::vector<double>>("nav_goals", std::vector<double>{});
    
    nh->get_parameter("Kp", Kp);
    nh->get_parameter("Ki", Ki);
    nh->get_parameter("Kd", Kd);
    nh->get_parameter("speed_ratio", speed_ratio);
    
    // 读取一维数组，解析成pair
    std::vector<double> raw_goals = nh->get_parameter("nav_goals").as_double_array();
    std::vector<std::pair<double, double>> nav_goals;
    
    if (raw_goals.size() % 2 != 0)
    {
        RCLCPP_ERROR(nh->get_logger(), "nav_goals must have even number of elements (x,y pairs)");
        rclcpp::shutdown();
        return 1;
    }
    
    for (size_t i = 0; i < raw_goals.size(); i += 2)
    {
        nav_goals.push_back({raw_goals[i], raw_goals[i+1]});
    }
    
    RCLCPP_WARN(nh->get_logger(), "Kp: %f, Ki: %f, Kd: %f, speed_ratio: %f, goals: %zu", 
                Kp, Ki, Kd, speed_ratio, nav_goals.size());
    RCLCPP_INFO(nh->get_logger(), "Trajectory follower node started");

    TrajFollower follower1(*nh, Kp, Ki, Kd, speed_ratio, nav_goals);

    rclcpp::spin(nh);
    rclcpp::shutdown();
    return 0;
}