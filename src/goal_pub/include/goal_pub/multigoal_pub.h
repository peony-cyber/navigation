#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <cstdint>
#include <vector>
#include <utility>

namespace goal_pub
{
class GoalPublisher : public rclcpp::Node
{
public:
    GoalPublisher();
private:
    std::vector<std::pair<double, double>> goals;
    std::size_t goal_id = 0;
    geometry_msgs::msg::PoseStamped pose_msg;

    void timerCallback();
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace navigoal_manager    