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
#include "point.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


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
    this->declare_parameter<bool>("enable_downstairs", false);
    this->declare_parameter<double>("voronoi_radius", 0.40);
    this->declare_parameter<double>("check_collision_radius", 0.50);

    this->get_parameter("enable_downstairs", enable_downstaris);
    this->get_parameter("voronoi_radius", voronoi_radius);
    this->get_parameter("check_collision_radius", collision_radius);

    RCLCPP_INFO(this->get_logger(), "[Params] [enable_downstairs] : %s", enable_downstaris ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[Params] [voronoi_radius] : %f", voronoi_radius);
    RCLCPP_INFO(this->get_logger(), "[Params] [check_collision_radius] : %f", collision_radius);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 1, std::bind(&PlanManager::map_callback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", 1, std::bind(&PlanManager::goal_callback, this, std::placeholders::_1));
    obstacle_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_livox_obs", 1, std::bind(&PlanManager::setObstacle, this,std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/sPath", 1);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    start = Eigen::Vector2d::Zero();
    goal = Eigen::Vector2d::Zero();
  }

private:
  // members
  bool map_geted;
  Eigen::Vector2i map_size;
  Eigen::Vector2d map_offset;
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
  double voronoi_radius, collision_radius;

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
    // planner_1.setEnvironment(esdf_1);
    RCLCPP_INFO(this->get_logger(), "A* planner initialized");
    // planner_1.setParam();
    RCLCPP_INFO(this->get_logger(), "A* planner parameters set");
    // planner_1.init();
    RCLCPP_INFO(this->get_logger(), "A* planner initialized completely");
    // smoother_1.smoother_setEnvironment(esdf_1);
    RCLCPP_INFO(this->get_logger(), "smoother initialized");
    map_geted = true;
  }

  void plan(Eigen::Vector2d start_, Eigen::Vector2d goal_)
  {
    if (!map_geted) return;
    std::vector<Eigen::Vector2d> Path_2d;
    RCLCPP_INFO(this->get_logger(), "start planning");
    Eigen::Vector2d start = start_;
    Eigen::Vector2d end = goal_;
    Eigen::Vector2i start_index = Pos2index(start);
    Eigen::Vector2i end_index = Pos2index(end);
    esdf_1->voronoi_map.findnearstVoronoi(start_index[0], start_index[1]);
    esdf_1->voronoi_map.findnearstVoronoi(end_index[0], end_index[1]);
    auto beforeTime = std::chrono::steady_clock::now();
    auto result = planner_1.search(start_index, end_index);
    if (result == navi_planner::Astar::NO_PATH)
    {
      planner_1.reset();
      return;
    }
    auto endTime = std::chrono::steady_clock::now();
    double duration_millsecond = std::chrono::duration<double, std::milli>(endTime - beforeTime).count();
    std::cout << "A* searching cost: " << duration_millsecond << "ms" << std::endl;
    std::vector<Eigen::Vector2i> Path_2i;
    Path_2i = planner_1.getPath();
    Path_2d.push_back(start);
    for (auto p : Path_2i)
    {
      Path_2d.push_back(Index2pos(p));
    }
    Path_2d.push_back(end);
    beforeTime = std::chrono::steady_clock::now();
    Path_2d = smoother_1.smooth(Path_2d, 0.3f, 0.04f);
    endTime = std::chrono::steady_clock::now();
    duration_millsecond = std::chrono::duration<double, std::milli>(endTime - beforeTime).count();
    std::cout << "smooth cost: " << duration_millsecond << "ms" << std::endl;
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
        tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0));
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
      if (esdf_1->voronoi_map.isOccupied(path_node_index[0], path_node_index[1]))
      {
        RCLCPP_INFO(this->get_logger(), "Collision detected");
        return true;
      }
      double dist_1 = esdf_1->voronoi_map.data[path_node_index[0]][path_node_index[1]].dist;
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

    std::vector<navi_planner::INTPOINT> Obstacles;
    for (auto &p : buffer.points)
    {
      Eigen::Vector2d obs_pt(p.x, p.y);
      Eigen::Vector2i obs_index = esdf_1->Pos2index(obs_pt);
      bool y_valid = map_size[0] >= (obs_index[1] + 9) && (obs_index[1] - 9) >= 0;
      bool x_valid = map_size[1] >= (obs_index[0] + 9) && (obs_index[0] - 9) >= 0;
      if (!(y_valid && x_valid)) { continue; }
      for (int k1 = -6; k1 < 7; k1++)
      {
        for (int k2 = -6; k2 < 7; k2++)
        {
          if (k1 * k1 + k2 * k2 > voronoi_radius * voronoi_radius * 400) continue;
          Obstacles.push_back(navi_planner::IntPoint(obs_index[0] + k1, obs_index[1] + k2));
        }
      }
    }

    esdf_1->voronoi_map.exchangeObstacles(Obstacles);
    esdf_1->voronoi_map.update();
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