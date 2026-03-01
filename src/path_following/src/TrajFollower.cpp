#include "TrajFollower.h"
#include <cmath>

void TrajFollower::manager()
{
    if (path_geted)
    {
        // get_transform();
          if (!get_transform())  // 检查返回值
        {
            RCLCPP_WARN(node_.get_logger(), "Skip cycle: no valid transform");
            return;
        }
        tranform_path();
        follow();
        pub_cmd();
    }
}

bool TrajFollower::get_transform()
{
    // geometry_msgs::TransformStamped robot_global_pose =
    //     tf_buffer_->lookupTransform(path_frame, "base_link", ros::Time(0));
    geometry_msgs::msg::TransformStamped robot_global_pose;
    try
    {
        auto now = node_.get_clock()->now();
        robot_global_pose = tf_buffer_->lookupTransform(
            path_frame, "body", now, 
            rclcpp::Duration::from_seconds(0.1));
        // robot_global_pose = tf_buffer_->lookupTransform(path_frame, "body", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(node_.get_logger(), "Could not get transform: %s", ex.what());
        return false;
    }

    map2baselink = Eigen::MatrixXd::Zero(4, 4);
    map2baselink(3, 3) = 1;
    map2baselink(0, 3) = robot_global_pose.transform.translation.x;
    map2baselink(1, 3) = robot_global_pose.transform.translation.y;
    map2baselink(2, 3) = 0;

    Eigen::Quaterniond q_tmp;
    q_tmp.x() = robot_global_pose.transform.rotation.x;
    q_tmp.y() = robot_global_pose.transform.rotation.y;
    q_tmp.z() = robot_global_pose.transform.rotation.z;
    q_tmp.w() = robot_global_pose.transform.rotation.w;

    Eigen::Matrix3d R;

    R = q_tmp.normalized().toRotationMatrix();

    map2baselink.block<3, 3>(0, 0) = R;
    return true;
}

void TrajFollower::tranform_path()
{
    local_path = path;
    for (auto &p : local_path)
    {
        Eigen::Vector4d p_temp = {p[0], p[1], 0, 1};

        p_temp = map2baselink.inverse() * p_temp;
        p = {p_temp[0], p_temp[1]};
    }
}

void TrajFollower::pub_cmd()
{
    // geometry_msgs::msg::Twist cmd_vel;
    auto cmd_vel = geometry_msgs::msg::Twist();

    cmd_vel.angular.z = 0;
    cmd_vel.linear.x = follow_speed[0];
    cmd_vel.linear.y = follow_speed[1];

    // cmd_pub.publish(cmd_vel);
    cmd_pub->publish(cmd_vel);
}

// 局部坐标系
// 消除横向误差PID
void TrajFollower::follow()
{
    // pid use
    static double last_lat_dist = 0;
    static double sum_lat_dist = 0;

    auto static_speed = Kp * speed_ratio;

    find_nearest_node();
    RCLCPP_INFO(node_.get_logger(), "Nearest node index: %d, dist: %.2f", pre_i, pre_node.norm());
    auto dist = (local_path.back()).norm();
    auto num_left = local_path.size() - pre_i - 1;

    RCLCPP_INFO(node_.get_logger(), "Distance to target: %.2f, nodes left: %d", dist, num_left);

    // 停止条件
    if (dist < 0.1)
    {
        // ROS_WARN("Reach the target, stop.");
        follow_speed = Eigen::Vector2d(0, 0);
        RCLCPP_INFO(node_.get_logger(), "Reached the target, stopping.");
        return;
    }

    // 最终节点
    if (num_left < 2)
    {
        // ROS_WARN("Almost reach the target.");
        RCLCPP_INFO(node_.get_logger(), "Almost reached the target.");
        Eigen::Vector2d follow_vector = local_path.back();
        follow_vector.normalize();
        follow_speed = follow_vector * static_speed * (num_left / 5.0);
        return;
    }

    auto next_next_node = local_path[pre_i + 2];

    Eigen::Vector2d path_vector = next_next_node - pre_node;
    path_vector.normalize();

    follow_speed = path_vector * static_speed;
    auto follow_vector = -pre_node;

    if (num_left <= 5)
        follow_speed *= (num_left / 5.0);

    // 计算横向误差
    auto lat_dist = follow_vector[0] * path_vector[1] - follow_vector[1] * path_vector[0];

    // 横向误差方向（与路径垂直）
    auto lat_direction = Eigen::Vector2d(-path_vector[1], path_vector[0]);
    lat_direction.normalize();

    auto lat_dist_pid = Kp * lat_dist + Kd * (lat_dist - last_lat_dist) + Ki * sum_lat_dist;
    sum_lat_dist += lat_dist;
    last_lat_dist = lat_dist;

    follow_speed += lat_direction * lat_dist_pid;
}

void TrajFollower::find_nearest_node()
{
    double best_dist = INFINITY;
    pre_i = 0;
    for (int i = 0; i < local_path.size(); i++)
    {
        Eigen::Vector2d p = local_path[i];
        double dist = (p).norm();
        if (dist < best_dist)
        {
            best_dist = dist;
            pre_node = p;
            pre_i = i;
        }
    }
    return;
}

void TrajFollower::global_path_callback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
{
    path.clear();
    if (!msg->poses.empty())
    {
        RCLCPP_INFO(node_.get_logger(), "Received new path with %zu poses.", msg->poses.size());
        for (auto p : msg->poses)
        {
            path.push_back(Eigen::Vector2d(p.pose.position.x, p.pose.position.y));
        }
        path_geted = true;
    }
}
