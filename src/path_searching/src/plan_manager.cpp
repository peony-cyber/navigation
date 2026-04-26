#include <rclcpp/rclcpp.hpp>
#include "esdf_map.h"
#include "Astar.h"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <Eigen/Core>
#include <cmath>
#include "smoother.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <array>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

class PlanManager : public rclcpp::Node
{
public:
  PlanManager()
  : Node("navi_planner"),
    map_geted(false),
    esdf_1(new ESDF_enviroment::esdf),
    planner_1(),
    smoother_1()
  {  
    // 初始化ROS2相关参数
    this->declare_parameter<bool>("enable_downstairs", false);
    this->declare_parameter<double>("obstacle_expand_radius", 0.40);
    this->declare_parameter<double>("check_collision_radius", 0.50);
    this->declare_parameter<double>("obstacle_cost_weight", 0.0);
    this->declare_parameter<double>("dynamic_penalty_weight",100.0);

    this->get_parameter("enable_downstairs", enable_downstaris);
    this->get_parameter("obstacle_expand_radius", obstacle_expand_radius);
    this->get_parameter("check_collision_radius", collision_radius);
    this->get_parameter("obstacle_cost_weight", obstacle_cost_weight);
    this->get_parameter("dynamic_penalty_weight", dynamic_penalty_weight);

    RCLCPP_INFO(this->get_logger(), "[Params] [enable_downstairs] : %s", enable_downstaris ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[Params] [obstacle_expand_radius] : %f", obstacle_expand_radius);
    RCLCPP_INFO(this->get_logger(), "[Params] [check_collision_radius] : %f", collision_radius);
    RCLCPP_INFO(this->get_logger(), "[Params] [obstacle_cost_weight] : %f", obstacle_cost_weight);

    // map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    //   "/map", 1, std::bind(&PlanManager::map_callback, this, std::placeholders::_1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
  "/map", 
  rclcpp::QoS(1).reliable().transient_local(),
  std::bind(&PlanManager::map_callback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 1, std::bind(&PlanManager::goal_callback, this, std::placeholders::_1));
    obstacle_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_livox_obs", 1, std::bind(&PlanManager::setObstacle, this,std::placeholders::_1));
    cost_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap/costmap",1,std::bind(&PlanManager::cost_map_callback,this,std::placeholders::_1));
    

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/sPath", 1);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    start = Eigen::Vector2d::Zero();
    goal = Eigen::Vector2d::Zero();
    has_goal_ = false;
    //TODO
    is_first_goal_ = true;
    last_goal_ = Eigen::Vector2d::Zero();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5000),
      std::bind(&PlanManager::timer_callback, this)
    );
  }

private:
  bool map_geted;
  Eigen::Vector2i map_size;
  Eigen::Vector2d map_offset;
  double resolution;
  ESDF_enviroment::Ptr esdf_1;
  navi_planner::Astar planner_1;
  navi_planner::smoother smoother_1;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_sub_;
  Eigen::Vector2d start, goal;
  std::vector<Eigen::Vector2d> path_now;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  bool enable_downstaris;
  double obstacle_expand_radius, collision_radius;
  double obstacle_cost_weight;
  //新增的stvl动态障碍参数
  double dynamic_penalty_weight;
  bool has_goal_;
  const std::string global_frame_{"map"};
  const std::array<std::string, 2> robot_frame_candidates_{{"baselink", "base_link"}};

  //TODO
  rclcpp::TimerBase::SharedPtr timer_;
  Eigen::Vector2d last_goal_;
  bool is_first_goal_;
  // [修复]: 动态使用地图 resolution，不再硬编码除以 20.0
  Eigen::Vector2d Index2pos(Eigen::Vector2i index_)
  {
    Eigen::Vector2d pos;
    pos[0] = (double)index_[1] * resolution + map_offset[0];
    pos[1] = (double)index_[0] * resolution + map_offset[1];
    return pos;
  }

  // [修复]: 动态使用地图 resolution，并使用 std::floor 确保坐标轴负半区转换正确
  Eigen::Vector2i Pos2index(Eigen::Vector2d pos)
  {
    Eigen::Vector2i index;
    index[1] = (int)std::floor((pos[0] - map_offset[0]) / resolution);
    index[0] = (int)std::floor((pos[1] - map_offset[1]) / resolution);
    return index;
  }

  void Path_pub(const std::vector<Eigen::Vector2d> &path2pub)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = this->now();

    for (auto p : path2pub)
    {
      geometry_msgs::msg::PoseStamped vertex;
      vertex.header = path.header;
      vertex.pose.position.x = p[0];
      vertex.pose.position.y = p[1];
      vertex.pose.orientation.w = 1.0;
      path.poses.push_back(vertex);
    }
    path_pub_->publish(path);
    RCLCPP_INFO(this->get_logger(), "Path has been published");
  }

  bool lookupLatestTransform(
    const std::string &target_frame,
    const std::string &source_frame,
    geometry_msgs::msg::TransformStamped &transform)
  {
    try
    {
      transform = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "TF lookup failed (%s <- %s, using latest available transform): %s",
        target_frame.c_str(),
        source_frame.c_str(),
        ex.what());
      return false;
    }
  }

  bool lookupLatestTransformEitherDirection(
    const std::string &target_frame,
    const std::string &source_frame,
    geometry_msgs::msg::TransformStamped &transform)
  {
    if (lookupLatestTransform(target_frame, source_frame, transform))
    {
      return true;
    }

    geometry_msgs::msg::TransformStamped inverse_transform;
    if (!lookupLatestTransform(source_frame, target_frame, inverse_transform))
    {
      return false;
    }

    Eigen::Isometry3d T_source_target = tf2::transformToEigen(inverse_transform.transform);
    Eigen::Isometry3d T_target_source = T_source_target.inverse();
    transform.header.stamp = inverse_transform.header.stamp;
    transform.header.frame_id = target_frame;
    transform.child_frame_id = source_frame;
    transform.transform = tf2::eigenToTransform(T_target_source).transform;
    return true;
  }

  bool poseToGlobal(
    const geometry_msgs::msg::PoseStamped &pose_in,
    Eigen::Vector2d &pose_xy_out,
    std::string *resolved_source_frame = nullptr)
  {
    const std::string source_frame = pose_in.header.frame_id.empty() ? global_frame_ : pose_in.header.frame_id;
    if (resolved_source_frame != nullptr)
    {
      *resolved_source_frame = source_frame;
    }

    geometry_msgs::msg::PoseStamped pose_latest = pose_in;
    pose_latest.header.frame_id = source_frame;
    pose_latest.header.stamp = builtin_interfaces::msg::Time();

    if (source_frame == global_frame_)
    {
      pose_xy_out[0] = pose_latest.pose.position.x;
      pose_xy_out[1] = pose_latest.pose.position.y;
      return true;
    }

    geometry_msgs::msg::TransformStamped transform;
    if (!lookupLatestTransform(global_frame_, source_frame, transform))
    {
      return false;
    }

    geometry_msgs::msg::PoseStamped pose_in_global;
    tf2::doTransform(pose_latest, pose_in_global, transform);
    pose_xy_out[0] = pose_in_global.pose.position.x;
    pose_xy_out[1] = pose_in_global.pose.position.y;
    return true;
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
  {
    if (map_geted) { return; }
    RCLCPP_INFO(this->get_logger(), "Map received");
    
    int size = map->info.width * map->info.height;
    map_size[0] = map->info.height;
    map_size[1] = map->info.width;
    map_offset[0] = map->info.origin.position.x;
    map_offset[1] = map->info.origin.position.y;
    resolution = map->info.resolution;
    
    std::cout << "map_size: " << map_size.transpose() << std::endl;
    std::cout << "map_offset: " << map_offset.transpose() << " resolution: " << resolution << std::endl;
    
    bool * bin_map_ = new bool[size];
    for (int i = 0; i < size; i++)
    {
      bin_map_[i] = map->data[i] ? true : false;
    }
    
    esdf_1->esdf_init(bin_map_, map->info.height, map->info.width, map_offset, enable_downstaris);
    
    // [修复]: 防止内存泄漏，初始化ESDF后释放临时分配的内存
    delete[] bin_map_; 
    
    RCLCPP_INFO(this->get_logger(), "ESDF map initialized");
    planner_1.setEnvironment(esdf_1);
    planner_1.setParam(obstacle_cost_weight,dynamic_penalty_weight);
    planner_1.init();
    smoother_1.smoother_setEnvironment(esdf_1);
    
    map_geted = true;
  }

  void plan(Eigen::Vector2d start_, Eigen::Vector2d goal_)
  {
    if (!map_geted) return;

    // [修复]: 彻底清理多余的手动计算 sx, sy 逻辑，统一调用 Pos2index
    Eigen::Vector2i start_index = Pos2index(start_);
    Eigen::Vector2i end_index = Pos2index(goal_);

    // 越界保护
    if (start_index[0] < 0 || start_index[0] >= map_size[0] || 
        start_index[1] < 0 || start_index[1] >= map_size[1] ||
        end_index[0] < 0 || end_index[0] >= map_size[0] || 
        end_index[1] < 0 || end_index[1] >= map_size[1])
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Goal/Start out of map bounds! sr=%d sc=%d gr=%d gc=%d  h=%d w=%d",
                     start_index[0], start_index[1], end_index[0], end_index[1], map_size[0], map_size[1]);
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Start planning...");
    
    auto beforeTime = std::chrono::steady_clock::now();
    auto result = planner_1.search(start_index, end_index);
    if (result == navi_planner::Astar::NO_PATH)
    {
      RCLCPP_WARN(this->get_logger(), "A* found NO_PATH");
      planner_1.reset();
      return;
    }
    
    auto endTime = std::chrono::steady_clock::now();
    double duration_millsecond = std::chrono::duration<double, std::milli>(endTime - beforeTime).count();
    std::cout << "A* searching cost: " << duration_millsecond << "ms" << std::endl;
    
    std::vector<Eigen::Vector2i> Path_2i = planner_1.getPath();
    std::vector<Eigen::Vector2d> Path_2d;

    Path_2d.reserve(Path_2i.size() + 2);
    Path_2d.push_back(start_);
    for (auto p : Path_2i)
    {
      Eigen::Vector2d path_point = Index2pos(p);
      if ((path_point - Path_2d.back()).norm() < 1e-6)
      {
        continue;
      }
      Path_2d.push_back(path_point);
    }
    if ((goal_ - Path_2d.back()).norm() >= 1e-6)
    {
      Path_2d.push_back(goal_);
    }

    beforeTime = std::chrono::steady_clock::now();
    std::vector<Eigen::Vector2d> raw_path = Path_2d;
    Path_2d = smoother_1.smooth(Path_2d, 0.3f, 0.04f);
    endTime = std::chrono::steady_clock::now();
    duration_millsecond = std::chrono::duration<double, std::milli>(endTime - beforeTime).count();
    std::cout << "Smooth cost: " << duration_millsecond << "ms" << std::endl;

    if (Path_2d.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Smoothed path is empty, fallback to raw path");
      Path_2d = std::move(raw_path);
    }

    Path_2d.front() = start_;
    if ((Path_2d.back() - goal_).norm() < 1e-6)
    {
      Path_2d.back() = goal_;
    }
    else
    {
      Path_2d.push_back(goal_);
    }
    
    planner_1.reset();
    path_now = Path_2d;
    Path_pub(Path_2d);
  }

  bool getStart()
  {
    std::ostringstream tried_frames;
    for (const auto &robot_frame : robot_frame_candidates_)
    {
      geometry_msgs::msg::TransformStamped robot_global_pose;
      if (!lookupLatestTransform(global_frame_, robot_frame, robot_global_pose))
      {
        tried_frames << robot_frame << " ";
        continue;
      }

      double pose_x = robot_global_pose.transform.translation.x;
      double pose_y = robot_global_pose.transform.translation.y;
      start = Eigen::Vector2d(pose_x, pose_y);
      RCLCPP_INFO(
        this->get_logger(),
        "Plan start resolved from frame '%s' at (%.3f, %.3f)",
        robot_frame.c_str(),
        pose_x,
        pose_y);
      return true;
    }

    const std::array<std::string, 2> anchor_frame_candidates{{"odom", "lidar"}};
    for (const auto &robot_frame : robot_frame_candidates_)
    {
      for (const auto &anchor_frame : anchor_frame_candidates)
      {
        geometry_msgs::msg::TransformStamped map_to_anchor;
        geometry_msgs::msg::TransformStamped anchor_to_robot;
        if (!lookupLatestTransform(global_frame_, anchor_frame, map_to_anchor))
        {
          tried_frames << global_frame_ << "<-" << anchor_frame << " ";
          continue;
        }
        if (!lookupLatestTransformEitherDirection(anchor_frame, robot_frame, anchor_to_robot))
        {
          tried_frames << anchor_frame << "<->" << robot_frame << " ";
          continue;
        }

        Eigen::Isometry3d T_map_anchor = tf2::transformToEigen(map_to_anchor.transform);
        Eigen::Isometry3d T_anchor_robot = tf2::transformToEigen(anchor_to_robot.transform);
        Eigen::Isometry3d T_map_robot = T_map_anchor * T_anchor_robot;
        start = T_map_robot.translation().head<2>();
        RCLCPP_INFO(
          this->get_logger(),
          "Plan start resolved from frame '%s' via anchor '%s' at (%.3f, %.3f)",
          robot_frame.c_str(),
          anchor_frame.c_str(),
          start[0],
          start[1]);
        return true;
      }
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Unable to resolve robot start pose in %s. Tried frames: %s",
      global_frame_.c_str(),
      tried_frames.str().c_str());
    return false;
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr end)
  {
    if (!map_geted) {
      RCLCPP_WARN(this->get_logger(), "Map not received yet!");
      return;
    }

    Eigen::Vector2d goal_pt;
    std::string goal_source_frame;
    if (!poseToGlobal(*end, goal_pt, &goal_source_frame))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to transform goal_pose into %s", global_frame_.c_str());
      return;
    }

    this->goal = goal_pt;
    has_goal_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Goal received in frame '%s', planned goal in %s: (%.3f, %.3f)",
      goal_source_frame.c_str(),
      global_frame_.c_str(),
      goal[0],
      goal[1]);

    double dist_to_last_goal = (goal_pt - last_goal_).norm();
    //TODO
    if (is_first_goal_ || dist_to_last_goal > 0.05) 
    {
      RCLCPP_INFO(this->get_logger(), "新目标点到达！立即触发规划。距离偏差: %f", dist_to_last_goal);
      last_goal_ = goal_pt;
      is_first_goal_ = false;

      if (!getStart()) {
        RCLCPP_WARN(this->get_logger(), "无法获取当前位姿，跳过规划");
        return;
      }
      
      if ((start - goal).norm() < 0.2) {
        RCLCPP_INFO(this->get_logger(), "目标点离起点太近，忽略。");
        return;
      }

      // 立即规划
      plan(start, goal);

      // 更新上一次的目标点和标志位
      
    }
    else 
    {
      // 目标点没变，直接 return，不阻塞回调！交由定时器处理
      // RCLCPP_DEBUG(this->get_logger(), "目标点未改变，交由定时器处理。");
      return; 
    }
    //BAK
    // if (!getStart())
    // {
    //   RCLCPP_WARN(this->get_logger(), "Skip planning because current robot pose is unavailable");
    //   return;
    // }

    // if ((start - goal).norm() < 0.3) {
    //   RCLCPP_INFO(this->get_logger(), "Goal too close to start, ignoring.");
    //   return;
    // }
    
    // plan(start, goal);
  }
  void cost_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    // esdf_1->updateCostmap();
  }

  bool detectCollision()
  {
    // [修复]: 动态计算栅格距离，替换硬编码的 * 20
    double collision_cells = collision_radius / resolution;

    for (size_t i = 1; i < path_now.size(); i++)
    {
      Eigen::Vector2i path_node_index = Pos2index(path_now[i]);
      if (esdf_1->checkCollision(path_node_index))
      {
        RCLCPP_INFO(this->get_logger(), "Collision detected (direct)");
        return true;
      }
      double dist_1 = esdf_1->getDist(path_node_index);
      if (dist_1 < collision_cells)
      {
        RCLCPP_INFO(this->get_logger(), "Collision detected (radius)");
        return true;
      }
    }
    return false;
  }

  void timer_callback()
  {
    // 如果还没接到过目标，直接跳过
    if (!has_goal_) {
      return;
    }

    // 获取当前最新位姿
    if (!getStart()) {
      return;
    }

    // 防抖：如果你离目标已经很近了，就不必一直规划了
    if ((start - goal).norm() < 0.1) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "timer goal callback");
    plan(start, goal);
  }

  void setObstacle(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    if (!map_geted) return;
    pcl::PointCloud<pcl::PointXYZ> buffer;
    pcl::fromROSMsg(*cloud_msg, buffer);

    const std::string cloud_frame = cloud_msg->header.frame_id.empty() ? global_frame_ : cloud_msg->header.frame_id;
    if (cloud_frame != global_frame_)
    {
      geometry_msgs::msg::TransformStamped transform;
      if (!lookupLatestTransform(global_frame_, cloud_frame, transform))
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Skip obstacle update because cloud frame '%s' cannot be transformed to %s",
          cloud_frame.c_str(),
          global_frame_.c_str());
        return;
      }

      Eigen::Matrix4f tf_matrix = tf2::transformToEigen(transform.transform).matrix().cast<float>();
      pcl::PointCloud<pcl::PointXYZ> transformed_buffer;
      pcl::transformPointCloud(buffer, transformed_buffer, tf_matrix);
      buffer.swap(transformed_buffer);
    }

    // [修复]: 动态计算膨胀半径的像素格数，替换硬编码的 * 20.0
    int expand_cells = (int)std::round(obstacle_expand_radius / resolution);
    
    for (auto &p : buffer.points)
    {
      Eigen::Vector2d obs_pt(p.x, p.y);
      Eigen::Vector2i obs_index = Pos2index(obs_pt);
      
      for (int k1 = -expand_cells; k1 <= expand_cells; k1++)
      {
        for (int k2 = -expand_cells; k2 <= expand_cells; k2++)
        {
          if (k1 * k1 + k2 * k2 > expand_cells * expand_cells) continue;
          int ix = obs_index[0] + k1;
          int iy = obs_index[1] + k2;
          if (ix < 0 || iy < 0 || ix >= esdf_1->Size[0] || iy >= esdf_1->Size[1]) continue;
          esdf_1->bin_map[ix][iy] = true;
        }
      }
    }

    esdf_1->updateDistanceField();
    planner_1.setEnvironment(esdf_1);
    planner_1.init();
    smoother_1.smoother_setEnvironment(esdf_1);

    if (!has_goal_)
    {
      return;
    }

    static unsigned int cnt_num = 0;
    if (detectCollision() || cnt_num > 5)
    {
      RCLCPP_INFO(this->get_logger(), "Detect obs, replanning");
      if (!getStart())
      {
        RCLCPP_WARN(this->get_logger(), "Skip replanning because current robot pose is unavailable");
        return;
      }
      plan(start, goal);
      cnt_num = 0; // 重置计数器
    }
    else
    {
      cnt_num++;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
