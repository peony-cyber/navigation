#ifndef MULTIGOAL_PUB_H
#define MULTIGOAL_PUB_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <vector>
#include <utility>
#include <memory>

namespace goal_pub
{
class GoalPublisher : public rclcpp::Node
{
public:
    GoalPublisher();

private:
    void publishNextGoal();
    void checkPoseCallback(); // 替换了原来的 timerCallback 和 poseCallback

    // ROS 相关
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // TF2 相关
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 逻辑变量
    std::vector<std::pair<double, double>> goals;
    size_t goal_id = 0;
    geometry_msgs::msg::PoseStamped pose_msg;
    double xy_goal_tolerance_;
};
} // namespace goal_pub

#endif // MULTIGOAL_PUB_H