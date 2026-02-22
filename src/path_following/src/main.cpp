#include <rclcpp/rclcpp.hpp>
#include "TrajFollower.h"

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "path_follower");

//     double Kp, Ki, Kd, speed_ratio;
//     ros::NodeHandle nh("~");
//     // nh.param<double>("Kp", Kp, 2.5);
//     nh.param<double>("Kp", Kp, 2.7);
//     nh.param<double>("Ki", Ki, 0.0);
//     nh.param<double>("Kd", Kd, 0.0);
//     nh.param<double>("speed_ratio", speed_ratio, 0.5);

//     ROS_WARN("Kp: %f, Ki: %f, Kd: %f, speed_ratio: %f", Kp, Ki, Kd, speed_ratio);
//     ROS_INFO("Trajectory follower node started");

//     TrajFollower follower1(nh, Kp, Ki, Kd, speed_ratio);
    
//     ros::spin();
// }
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    double Kp, Ki, Kd, speed_ratio;
    auto nh = rclcpp::Node::make_shared("path_follower_node");
    nh->declare_parameter<double>("Kp", 2.7);
    nh->declare_parameter<double>("Ki", 0.0);
    nh->declare_parameter<double>("Kd", 0.0);
    nh->declare_parameter<double>("speed_ratio", 0.5);
    nh->get_parameter("Kp", Kp);
    nh->get_parameter("Ki", Ki);
    nh->get_parameter("Kd", Kd);
    nh->get_parameter("speed_ratio", speed_ratio);

    RCLCPP_WARN(nh->get_logger(), "Kp: %f, Ki: %f, Kd: %f, speed_ratio: %f", Kp, Ki, Kd, speed_ratio);
    RCLCPP_INFO(nh->get_logger(), "Trajectory follower node started");

    TrajFollower follower1(*nh, Kp, Ki, Kd, speed_ratio);

    rclcpp::spin(nh);
    rclcpp::shutdown();
    return 0;
}