// TrajFollower.cpp
#include "TrajFollower.h"
#include <cmath>

void TrajFollower::manager()
{
    if (all_goals_reached)
    {
        follow_speed = Eigen::Vector2d(0, 0);
        pub_cmd();
        return;
    }

    if (path_geted)
    {
        if (!get_transform())
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
    geometry_msgs::msg::TransformStamped robot_global_pose;
    try
    {
        auto now = node_.get_clock()->now();
        robot_global_pose = tf_buffer_->lookupTransform(
            path_frame, "baselink", now, 
            rclcpp::Duration::from_seconds(0.1));
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(node_.get_logger(), "Could not get transform:1111 %s", ex.what());
        // return false;
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
    auto cmd_vel = geometry_msgs::msg::Twist();
    cmd_vel.angular.z = 0;
    cmd_vel.linear.x = follow_speed[0];
    cmd_vel.linear.y = follow_speed[1];
    cmd_pub->publish(cmd_vel);
}

void TrajFollower::follow()
{
    static double last_lat_dist = 0;
    static double sum_lat_dist = 0;

    auto static_speed = Kp * speed_ratio;

    find_nearest_node();
    auto dist = (local_path.back()).norm();
    auto num_left = local_path.size() - pre_i - 1;

    RCLCPP_INFO(node_.get_logger(), "Goal %zu/%zu | Nearest: %d | Dist: %.2f | Left: %d", 
                goal_id, goals.size(), pre_i, dist, num_left);

    // 当前航点到达，切下一个
    if (dist < 0.1)
    {
        RCLCPP_INFO(node_.get_logger(), "Goal %zu reached, loading next...", goal_id);
        load_next_goal();
        // 重置PID积分项，避免累积误差带到下一段
        sum_lat_dist = 0;
        last_lat_dist = 0;
        return;
    }

    // 最终节点减速
    if (num_left < 2)
    {
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

    auto lat_dist = follow_vector[0] * path_vector[1] - follow_vector[1] * path_vector[0];
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
}

void TrajFollower::load_next_goal()
{
    if (goal_id >= goals.size())
    {
        all_goals_reached = true;
        path_geted = false;
        RCLCPP_INFO(node_.get_logger(), "All goals reached, stopping.");
        return;
    }

    // 构建单点路径（也可以插值成多段，这里简化为直接朝目标走）
    path.clear();
    auto goal = goals[goal_id];
    path.push_back(Eigen::Vector2d(goal.first, goal.second));
    
    // 为了控制器能跑起来，至少塞两个点（当前点+目标点），这里用目标点重复两次
    // 实际场景中你可能需要从当前位姿到目标做一条直线插值
    path.push_back(Eigen::Vector2d(goal.first, goal.second));
    
    path_geted = true;
    goal_id++;
    
    RCLCPP_INFO(node_.get_logger(), "Loaded goal %zu: (%.2f, %.2f)", goal_id, goal.first, goal.second);
}