#include "goal_pub/multigoal_pub.h"

namespace goal_pub
{
GoalPublisher::GoalPublisher() : Node("goal_publisher")
{
    this->declare_parameter<std::vector<double>>("nav_goals",{});

    std::vector<double> raw_points;

    // 获取参数
    raw_points = this->get_parameter("nav_goals").as_double_array();

    goals.reserve(raw_points.size() / 2);

    for (size_t i = 0; i < raw_points.size(); i += 2)
    {
        goals.emplace_back(raw_points[i], raw_points[i + 1]);
        // RCLCPP_INFO(this->get_logger(), "Loaded goal: (%f, %f);id:(%zu)", raw_points[i], raw_points[i + 1], i/2);
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10000),
        std::bind(&GoalPublisher::timerCallback, this));
    // 发布目标位姿到/goal
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
}

void GoalPublisher::timerCallback()
{
    if(goal_id < goals.size())
    {
        pose_msg.pose.position.x = goals[goal_id].first;
        pose_msg.pose.position.y = goals[goal_id].second;
        pose_msg.header.frame_id = "map";
        goal_publisher_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published goal %zu: (%f, %f)", goal_id, goals[goal_id].first, goals[goal_id].second);
        goal_id++;
    }
    else if (goal_id == goals.size())
    {
        RCLCPP_INFO(this->get_logger(), "All goals have been published.");
        return;
    }
    
}
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<goal_pub::GoalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}