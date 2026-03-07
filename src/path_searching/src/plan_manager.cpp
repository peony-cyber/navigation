#include <rclcpp/rclcpp.hpp>
#include "esdf_map.h"
#include "Astar.h"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
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
    // 初始化ROS2相关
    // this->declare_parameter("use_sim_time", false);
    
    
    this->declare_parameter<bool>("enable_downstairs", false);
    this->declare_parameter<double>("obstacle_expand_radius", 0.40);
    this->declare_parameter<double>("check_collision_radius", 0.50);
    this->declare_parameter<double>("obstacle_cost_weight", 0.0);

    this->get_parameter("enable_downstairs", enable_downstaris);
    this->get_parameter("obstacle_expand_radius", obstacle_expand_radius);
    this->get_parameter("check_collision_radius", collision_radius);
    this->get_parameter("obstacle_cost_weight", obstacle_cost_weight);

    RCLCPP_INFO(this->get_logger(), "[Params] [enable_downstairs] : %s", enable_downstaris ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[Params] [obstacle_expand_radius] : %f", obstacle_expand_radius);
    RCLCPP_INFO(this->get_logger(), "[Params] [check_collision_radius] : %f", collision_radius);
    RCLCPP_INFO(this->get_logger(), "[Params] [obstacle_cost_weight] : %f", obstacle_cost_weight);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 1, std::bind(&PlanManager::map_callback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 1, std::bind(&PlanManager::goal_callback, this, std::placeholders::_1));
    obstacle_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_livox_obs", 1, std::bind(&PlanManager::setObstacle, this,std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/sPath", 1);

    // tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(
    this->get_clock(),
    tf2::Duration(std::chrono::seconds(1)));

    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    start = Eigen::Vector2d::Zero();
    goal = Eigen::Vector2d::Zero();
  }

private:
  // members
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
  Eigen::Vector2d start, goal;
  std::vector<Eigen::Vector2d> path_now;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  bool enable_downstaris;
  double obstacle_expand_radius, collision_radius;
  double obstacle_cost_weight;

  Eigen::Vector2d Index2pos(Eigen::Vector2i index_)
  {
    Eigen::Vector2d index_d((double)index_[1], (double)index_[0]);
    Eigen::Vector2d pos_ = index_d / 20.0f + map_offset;
    return pos_;
  }

  Eigen::Vector2i Pos2index(Eigen::Vector2d pos)
  {
    Eigen::Vector2d index_ = (pos - map_offset) * 20.0f;
    Eigen::Vector2i index((int)index_[1], (int)index_[0]);
    return index;
  }

  void Path_pub(const std::vector<Eigen::Vector2d> &path2pub)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = this->now();

    for (auto p : path2pub)
    {
      geometry_msgs::msg::PoseStamped vertex;
      vertex.pose.position.x = p[0];
      vertex.pose.position.y = p[1];
      path.poses.push_back(vertex);
    }
    path_pub_->publish(path);
    RCLCPP_INFO(this->get_logger(), "plan_manager has been working, path has been published");
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
  {
    if (map_geted) { return; }
    RCLCPP_INFO(this->get_logger(), "Map received");
    int size = map->info.width * map->info.height;
    map_size[0] = map->info.width;
    map_size[1] = map->info.height;
    map_offset[0] = map->info.origin.position.x;
    map_offset[1] = map->info.origin.position.y;
    resolution = map->info.resolution;
    std::cout << "map_size: "<< map_size << std::endl;
    std::cout << "map_offset: "<< map_offset << std::endl;
    RCLCPP_INFO(this->get_logger(), "before new bool[%d]", size);
    bool * bin_map_ = new bool[size];
    RCLCPP_INFO(this->get_logger(), "after new bool[%d]", size); 
    for (int i = 0; i < size; i++)
    {
      bin_map_[i] = map->data[i] ? true : false;
    }
    esdf_1->esdf_init(bin_map_, map->info.height, map->info.width, map_offset, enable_downstaris);
    RCLCPP_INFO(this->get_logger(), "ESDF map initialized");
    planner_1.setEnvironment(esdf_1);
    // RCLCPP_INFO(this->get_logger(), "A* planner environment set");
    planner_1.setParam(obstacle_cost_weight);
    // RCLCPP_INFO(this->get_logger(), "A* planner parameters set");
    planner_1.init();
    // RCLCPP_INFO(this->get_logger(), "A* planner initialized completely");
    smoother_1.smoother_setEnvironment(esdf_1);
    // RCLCPP_INFO(this->get_logger(), "smoother initialized");
    map_geted = true;
  }

  // void plan(Eigen::Vector2d start_, Eigen::Vector2d goal_)
  // {
  //   if (!map_geted) return;
  //   //米→像素
  //   int sx = (int)((start_.x() - map_offset[0]) / resolution);
  //   int sy = (int)((start_.y() - map_offset[1]) / resolution);
  //   int gx = (int)((goal_.x()   - map_offset[0]) / resolution);
  //   int gy = (int)((goal_.y()   - map_offset[1]) / resolution);

  //   //越界保护
  //   if (sx<0||sx>=map_size[0]||sy<0||sy>=map_size[1]||
  //       gx<0||gx>=map_size[0]||gy<0||gy>=map_size[1])
  //   {
  //       RCLCPP_ERROR(this->get_logger(),
  //                    "Goal/Start out of map sx=%d sy=%d gx=%d gy=%d  w=%d h=%d",
  //                    sx,sy,gx,gy,map_size[0],map_size[1]);
  //       return;
  //   }
  //   //
  //   std::vector<Eigen::Vector2d> Path_2d;
  //   RCLCPP_INFO(this->get_logger(), "start planning");
  //   Eigen::Vector2d start = start_;
  //   Eigen::Vector2d end = goal_;
  //   Eigen::Vector2i start_index = Pos2index(start);
  //   Eigen::Vector2i end_index = Pos2index(end);
  //   auto beforeTime = std::chrono::steady_clock::now();
  //   auto result = planner_1.search(start_index, end_index);
  //   if (result == navi_planner::Astar::NO_PATH)
  //   {
  //     planner_1.reset();
  //     return;
  //   }
  //   auto endTime = std::chrono::steady_clock::now();
  //   double duration_millsecond = std::chrono::duration<double, std::milli>(endTime - beforeTime).count();
  //   std::cout << "A* searching cost: " << duration_millsecond << "ms" << std::endl;
  //   std::vector<Eigen::Vector2i> Path_2i;
  //   Path_2i = planner_1.getPath();
  //   Path_2d.push_back(start);
  //   for (auto p : Path_2i)
  //   {
  //     Path_2d.push_back(Index2pos(p));
  //   }
  //   Path_2d.push_back(end);
  //   beforeTime = std::chrono::steady_clock::now();
  //   Path_2d = smoother_1.smooth(Path_2d, 0.3f, 0.04f);
  //   endTime = std::chrono::steady_clock::now();
  //   duration_millsecond = std::chrono::duration<double, std::milli>(endTime - beforeTime).count();
  //   std::cout << "smooth cost: " << duration_millsecond << "ms" << std::endl;
  //   planner_1.reset();
  //   path_now = Path_2d;
  //   Path_pub(Path_2d);
  // }
void plan(Eigen::Vector2d start_, Eigen::Vector2d goal_)
{
    if (!map_geted) return;

    // 1. 记录原始点击坐标
    RCLCPP_INFO(this->get_logger(), "--- Planning Start ---");
    RCLCPP_INFO(this->get_logger(), "[Stage 0] Input Goal (meters): X:%.4f, Y:%.4f", goal_.x(), goal_.y());

    // 2. 检查分辨率一致性 (对比 resolution 和 20.0f)
    double inv_res = 1.0 / resolution;
    RCLCPP_INFO(this->get_logger(), "[Stage 1] Res Check: map_res:%.4f (inv:%.2f), hardcoded:20.00", resolution, inv_res);

    int sx = (int)((start_.x() - map_offset[0]) / resolution);
    int sy = (int)((start_.y() - map_offset[1]) / resolution);
    int gx = (int)((goal_.x()   - map_offset[0]) / resolution);
    int gy = (int)((goal_.y()   - map_offset[1]) / resolution);

    // 越界保护... (保留原逻辑)
     if (sx<0||sx>=map_size[0]||sy<0||sy>=map_size[1]||
        gx<0||gx>=map_size[0]||gy<0||gy>=map_size[1])
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Goal/Start out of map sx=%d sy=%d gx=%d gy=%d  w=%d h=%d",
                     sx,sy,gx,gy,map_size[0],map_size[1]);
        return;
    }

    Eigen::Vector2i start_index = Pos2index(start_);
    Eigen::Vector2i end_index = Pos2index(goal_);
    
    // 3. 检查像素化后的坐标 (看是否因为取整产生巨大位移)
    Eigen::Vector2d reprojected_goal = Index2pos(end_index);
    RCLCPP_INFO(this->get_logger(), "[Stage 2] Pixel Goal: [%d, %d], Reprojected: [%.4f, %.4f]", 
                end_index.x(), end_index.y(), reprojected_goal.x(), reprojected_goal.y());

    auto result = planner_1.search(start_index, end_index);
    if (result == navi_planner::Astar::NO_PATH) {
        planner_1.reset();
        return;
    }

    std::vector<Eigen::Vector2i> Path_2i = planner_1.getPath();
    std::vector<Eigen::Vector2d> Path_2d;
    Path_2d.push_back(start_);
    for (auto p : Path_2i) Path_2d.push_back(Index2pos(p));
    
    // 4. A* 原始路径终点
    RCLCPP_INFO(this->get_logger(), "[Stage 3] A* Raw End: [%.4f, %.4f]", Path_2d.back().x(), Path_2d.back().y());

    // 5. 加入原始目标点（平滑前）
    Path_2d.push_back(goal_);

    // 6. 执行平滑
    auto before_smooth_end = Path_2d.back();
    Path_2d = smoother_1.smooth(Path_2d, 0.3f, 0.04f);
    
    // 7. 检查平滑后的终点
    RCLCPP_INFO(this->get_logger(), "[Stage 4] Smoothed End: [%.4f, %.4f]", Path_2d.back().x(), Path_2d.back().y());

    double final_error = (Path_2d.back() - goal_).norm();
    RCLCPP_INFO(this->get_logger(), "--- Final Deviation: %.4f meters ---", final_error);

    planner_1.reset();
    path_now = Path_2d;
    Path_pub(Path_2d);
}
  void getStart()
  {
    try
    {
      // lookup latest transform from base_link to map
      geometry_msgs::msg::TransformStamped robot_global_pose =
        tf_buffer_->lookupTransform("map", "body", rclcpp::Time(0));
      double pose_x = robot_global_pose.transform.translation.x;
      double pose_y = robot_global_pose.transform.translation.y;
      start = Eigen::Vector2d(pose_x, pose_y);
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed in getStart: %s", ex.what());
    }
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr end)
  {
    if (!map_geted) return;
    RCLCPP_INFO(this->get_logger(), "Goal received, planning...");
    goal[0] = end->pose.position.x;
    goal[1] = end->pose.position.y;
    getStart();
    if ((start - goal).norm() < 0.3) return;
    plan(start, goal);
  }

  bool detectCollision()
  {
    for (size_t i = 1; i < path_now.size(); i++)
    {
      Eigen::Vector2i path_node_index = esdf_1->Pos2index(path_now[i]);
      if (esdf_1->checkCollision(path_node_index))
      {
        RCLCPP_INFO(this->get_logger(), "Collision detected");
        return true;
      }
      double dist_1 = esdf_1->getDist(path_node_index);
      if (dist_1 < collision_radius * 20)
      {
        RCLCPP_INFO(this->get_logger(), "Collision detected");
        return true;
      }
    }
    return false;
  }

  void setObstacle(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    if (!map_geted) return;
    pcl::PointCloud<pcl::PointXYZ> buffer;
    pcl::fromROSMsg(*cloud_msg, buffer);

    // mark obstacle cells directly in the occupancy grid
    int expand_cells = (int)std::round(obstacle_expand_radius * 20.0);
    for (auto &p : buffer.points)
    {
      Eigen::Vector2d obs_pt(p.x, p.y);
      Eigen::Vector2i obs_index = esdf_1->Pos2index(obs_pt);
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

    // recompute ESDF after obstacle updates
    esdf_1->updateDistanceField();
    planner_1.setEnvironment(esdf_1);
    planner_1.init();
    smoother_1.smoother_setEnvironment(esdf_1);

    static unsigned int cnt_num = 0;
    if (detectCollision() || cnt_num > 5)
    {
      RCLCPP_INFO(this->get_logger(), "detect obs, replanning");
      getStart();
      plan(start, goal);
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