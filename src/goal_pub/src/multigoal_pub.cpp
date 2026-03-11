#include "goal_pub/multigoal_pub.h"
#include <cmath> // 需要引入 cmath 计算距离

namespace goal_pub
{
GoalPublisher::GoalPublisher() : Node("multigoal_pub_node")
{
    // 1. 声明参数
    this->declare_parameter<std::vector<double>>("nav_goals", std::vector<double>{});
    this->declare_parameter<double>("xy_goal_tolerance", 0.1); // 新增：目标点小邻域容差，默认 0.5 米

    // 2. 获取参数
    std::vector<double> raw_points = this->get_parameter("nav_goals").as_double_array();
    xy_goal_tolerance_ = this->get_parameter("xy_goal_tolerance").as_double();

    goals.reserve(raw_points.size() / 2);

    for (size_t i = 0; i < raw_points.size(); i += 2)
    {
        goals.emplace_back(raw_points[i], raw_points[i + 1]);
        RCLCPP_INFO(this->get_logger(), "Loaded goal %zu: (%f, %f)", i/2, raw_points[i], raw_points[i + 1]);
    }

    // 3. 创建发布器和订阅器
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    // 假设机器人的位姿话题是 /odom (消息类型 nav_msgs::msg::Odometry)
    // 如果你用的是 /amcl_pose，请改成 geometry_msgs::msg::PoseWithCovarianceStamped
    pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GoalPublisher::poseCallback, this, std::placeholders::_1));

    // 4. 初始化发布第一个目标点
    goal_id = 0;
    if (!goals.empty())
    {
        publishNextGoal();
    }
}

// 将发布逻辑提取成一个单独的函数，保持代码整洁
void GoalPublisher::publishNextGoal()
{
    if (goal_id < goals.size())
    {
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = this->now();
        pose_msg.pose.position.x = goals[goal_id].first;
        pose_msg.pose.position.y = goals[goal_id].second;
        
        // 避坑：Nav2 默认要求有效的四元数，如果 orientation 是全 0，可能会报错。
        pose_msg.pose.orientation.w = 1.0; 

        goal_publisher_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published goal %zu: (%f, %f)", goal_id, goals[goal_id].first, goals[goal_id].second);
        
        goal_id++;
    }
}

// 订阅机器人实时位姿的回调函数
void GoalPublisher::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 如果还没开始，或者点已经发完了，就不做处理
    if (goal_id == 0 || goal_id > goals.size()) {
        return;
    }

    // 1. 获取机器人当前 XY 坐标
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    // 2. 获取当前正在追踪的目标点坐标
    // (因为在 publishNextGoal 中 goal_id 已经加 1 了，所以当前目标是 goal_id - 1)
    double target_x = goals[goal_id - 1].first;
    double target_y = goals[goal_id - 1].second;

    // 3. 计算与目标点的欧氏距离
    double distance = std::hypot(current_x - target_x, current_y - target_y);

    // 4. 判断是否进入小邻域范围
    if (distance < xy_goal_tolerance_)
    {
        RCLCPP_INFO(this->get_logger(), "Reached goal %zu (distance: %.2f m).", goal_id - 1, distance);

        if (goal_id < goals.size())
        {
            // 发布下一个点
            publishNextGoal();
        }
        else
        {
            // 所有点执行完毕
            RCLCPP_INFO(this->get_logger(), "All goals have been reached! Sentinel patrol completed.");
            goal_id++; // 防止重复触发此日志
        }
    }
}
} // namespace goal_pub

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<goal_pub::GoalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}