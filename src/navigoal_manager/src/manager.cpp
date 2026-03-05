#include "manager.h"

namespace navigoal_manager
{
GoalManager::GoalManager() : Node("goal_manager")
{
    this->declare_parameter<int>("bias", 20);
    this->declare_parameter<std::vector<double>>("r",{});
    this->declare_parameter<std::vector<double>>("b",{});

    std::vector<double> r_raw, b_raw;

    // 获取参数值
    bias = this->get_parameter("bias").as_int();
    r_raw = this->get_parameter("r").as_double_array();
    b_raw = this->get_parameter("b").as_double_array();

    r_goals.clear();
    b_goals.clear();
    for (size_t i = 0; i < r_raw.size(); i += 2)
    {
        r_goals.emplace_back(r_raw[i], r_raw[i + 1]);
        RCLCPP_INFO(this->get_logger(), "Loaded r_goal: (%f, %f);id:(%zu)", r_raw[i], r_raw[i + 1], i/2);
    }
    for (size_t i = 0; i < b_raw.size(); i += 2)
    {
        b_goals.emplace_back(b_raw[i], b_raw[i + 1]);
        RCLCPP_INFO(this->get_logger(), "Loaded b_goal: (%f, %f);id:(%zu)", b_raw[i], b_raw[i + 1], i/2);
    }


    // 订阅来自ly_goal的目标位姿
    ly_goal_subscription_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/ly/navi/goal", 10, std::bind(&GoalManager::goalCallback, this, std::placeholders::_1));
    // 发布目标位姿到/goal
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
}

void GoalManager::goalCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received goal: %d", msg->data);
    // 将接收到的目标位姿发布到/goal
    std::uint8_t goal_id = msg->data;
    geometry_msgs::msg::PoseStamped pose_msg;
    if (goal_id < bias)
    {
        if (goal_id >= r_goals.size())
        {
            RCLCPP_ERROR(this->get_logger(), "Received invalid goal id: %d", goal_id);
            return;
        }
        pose_msg.pose.position.x = r_goals[goal_id].first;
        pose_msg.pose.position.y = r_goals[goal_id].second;
    }
    else
    {
        if (static_cast<size_t>(goal_id - bias) >= b_goals.size())
        {
            RCLCPP_ERROR(this->get_logger(), "Received invalid goal id: %d", goal_id);
            return;
        }
        pose_msg.pose.position.x = b_goals[goal_id - bias].first;
        pose_msg.pose.position.y = b_goals[goal_id - bias].second;
    }

    goal_publisher_->publish(pose_msg);
}
}
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<navigoal_manager::GoalManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}