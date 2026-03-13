#include "goal_pub/multigoal_pub.h"
#include <cmath>
#include "tf2/exceptions.h" // 必须包含 TF 异常处理

namespace goal_pub
{
GoalPublisher::GoalPublisher() : Node("multigoal_pub_node")
{
    // 1. 声明参数
    this->declare_parameter<std::vector<double>>("nav_goals", std::vector<double>{});
    this->declare_parameter<double>("xy_goal_tolerance", 0.5);

    // 2. 获取参数
    std::vector<double> raw_points = this->get_parameter("nav_goals").as_double_array();
    xy_goal_tolerance_ = this->get_parameter("xy_goal_tolerance").as_double();

    goals.reserve(raw_points.size() / 2);

    for (size_t i = 0; i < raw_points.size(); i += 2)
    {
        goals.emplace_back(raw_points[i], raw_points[i + 1]);
        RCLCPP_INFO(this->get_logger(), "Loaded goal %zu: (%f, %f)", i/2, raw_points[i], raw_points[i + 1]);
    }

    // 3. 创建发布器
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    // 4. 初始化 TF2 监听器
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 5. 创建状态检查定时器 (10Hz，每 100ms 检查一次当前位姿)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GoalPublisher::checkPoseCallback, this));

    // 6. 初始化发布第一个目标点
    goal_id = 0;
    if (!goals.empty())
    {
        publishNextGoal();
    }
}

void GoalPublisher::publishNextGoal()
{
    if (goal_id < goals.size())
    {
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = this->now();
        pose_msg.pose.position.x = goals[goal_id].first;
        pose_msg.pose.position.y = goals[goal_id].second;
        pose_msg.pose.orientation.w = 1.0; 

        goal_publisher_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published goal %zu: (%f, %f)", goal_id, goals[goal_id].first, goals[goal_id].second);
        
        goal_id++;
    }
}

// 核心逻辑：周期性查询 TF 并计算距离
void GoalPublisher::checkPoseCallback()
{
    // 如果还没开始，或者所有点已经执行完毕，直接返回
    if (goal_id == 0 || goal_id > goals.size()) {
        return;
    }

    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        // 查询从 map 到 base_link 的最新坐标变换
        transformStamped = tf_buffer_->lookupTransform(
            "map", "base_link",
            tf2::TimePointZero); // TimePointZero 表示获取最新可用的变换
    } catch (const tf2::TransformException & ex) {
        // TF 树可能还没建好，或者偶尔丢帧，这里不用报错，直接等下个周期即可
        // RCLCPP_WARN(this->get_logger(), "Could not transform map to base_link: %s", ex.what());
        return;
    }

    // 1. 从 TF 树中获取机器人当前 XY 坐标
    double current_x = transformStamped.transform.translation.x;
    double current_y = transformStamped.transform.translation.y;

    // 2. 获取当前正在追踪的目标点坐标 (goal_id 在发完点后已经加 1)
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
            goal_id++; // 越界处理，防止定时器疯狂打印这句日志
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