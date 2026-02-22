#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <unordered_map>
#include <vector>
#include <mutex>
#include <cmath>

// ================= 参数（严格对齐 Python） =================
constexpr double RESOLUTION = 0.05;

constexpr double MAP_SIZE_X = 60.0;
constexpr double MAP_SIZE_Y = 60.0;
constexpr double MAP_ORIGIN_X = -10.0;
constexpr double MAP_ORIGIN_Y = -10.0;

constexpr double HEIGHT_FLAT    = 0.05;
constexpr double HEIGHT_MAX     = 0.30;
constexpr double HEIGHT_CEILING = 1.5;

constexpr int8_t UNKNOWN_COST = -1;
constexpr int8_t FREE_COST    = 0;
constexpr int8_t LETHAL_COST  = 100;

constexpr char MAP_FRAME[] = "lidar";

// ================= cell key =================
struct CellKey {
  int ix, iy;
  bool operator==(const CellKey& o) const {
    return ix == o.ix && iy == o.iy;
  }
};

struct CellHash {
  std::size_t operator()(const CellKey& k) const noexcept {
    return std::hash<int>()(k.ix * 73856093 ^ k.iy * 19349663);
  }
};

// ================= Node =================
class HeightCostmapFusion : public rclcpp::Node {
public:
  HeightCostmapFusion() : Node("height_costmap_fusion_cpp") {

    width_  = int(MAP_SIZE_X / RESOLUTION);
    height_ = int(MAP_SIZE_Y / RESOLUTION);

    corrected_cells_.clear();

    sub_world_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/fastlio2/world_cloud", rclcpp::SensorDataQoS(),
      std::bind(&HeightCostmapFusion::onWorldCloud, this, std::placeholders::_1));

    sub_corrected_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/corrected_map_cloud", 1,
      std::bind(&HeightCostmapFusion::onCorrectedMap, this, std::placeholders::_1));

    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/height_costmap", 10);

    RCLCPP_INFO(get_logger(), "Height costmap fusion (C++) started");
  }

private:
  inline bool worldToGrid(float x, float y, int &ix, int &iy) const {
    ix = int(std::floor((x - MAP_ORIGIN_X) / RESOLUTION));
    iy = int(std::floor((y - MAP_ORIGIN_Y) / RESOLUTION));
    return ix >= 0 && iy >= 0 && ix < width_ && iy < height_;
  }

  // =============== corrected_map：只缓存 z =================
  void onCorrectedMap(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::unordered_map<CellKey, std::vector<float>, CellHash> tmp;

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg,"x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg,"y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg,"z");

    for (; it_x!=it_x.end(); ++it_x, ++it_y, ++it_z) {
      if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z))
        continue;
      if (*it_z > HEIGHT_CEILING) continue;

      int ix, iy;
      if (!worldToGrid(*it_x, *it_y, ix, iy)) continue;

      tmp[{ix, iy}].push_back(*it_z);
    }

    std::lock_guard<std::mutex> lk(mutex_);
    corrected_cells_ = std::move(tmp);

    RCLCPP_INFO(get_logger(),
      "corrected_map cached: %zu cells", corrected_cells_.size());
  }

  // =============== world_cloud：主触发 =================
  void onWorldCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    // 复制 corrected（避免持锁）
    std::unordered_map<CellKey, std::vector<float>, CellHash> cells;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      cells = corrected_cells_;
    }

    // 加 world_cloud
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg,"x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg,"y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg,"z");

    for (; it_x!=it_x.end(); ++it_x, ++it_y, ++it_z) {
      if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z))
        continue;
      if (*it_z > HEIGHT_CEILING) continue;

      int ix, iy;
      if (!worldToGrid(*it_x, *it_y, ix, iy)) continue;

      cells[{ix, iy}].push_back(*it_z);
    }

    // 生成 costmap
    std::vector<int8_t> costmap(width_ * height_, UNKNOWN_COST);

    for (const auto &kv : cells) {
      const auto &zs = kv.second;
      if (zs.empty()) continue;

      float zmin = *std::min_element(zs.begin(), zs.end());
      float zmax = *std::max_element(zs.begin(), zs.end());
      float dz = zmax - zmin;

      int8_t cost;
      if (dz < HEIGHT_FLAT) cost = FREE_COST;
      else if (dz > HEIGHT_MAX) cost = LETHAL_COST;
      else {
        cost = int8_t((dz - HEIGHT_FLAT) /
               (HEIGHT_MAX - HEIGHT_FLAT) * LETHAL_COST);
      }

      costmap[kv.first.iy * width_ + kv.first.ix] = cost;
    }

    publish(costmap, msg->header.stamp);
  }

  // =============== publish =================
  void publish(const std::vector<int8_t> &data, const rclcpp::Time &stamp) {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = stamp;
    grid.header.frame_id = MAP_FRAME;

    grid.info.resolution = RESOLUTION;
    grid.info.width  = width_;
    grid.info.height = height_;

    geometry_msgs::msg::Pose origin;
    origin.position.x = MAP_ORIGIN_X;
    origin.position.y = MAP_ORIGIN_Y;
    origin.orientation.w = 1.0;
    grid.info.origin = origin;

    grid.data = data;
    pub_->publish(grid);
  }

private:
  int width_, height_;

  std::unordered_map<CellKey, std::vector<float>, CellHash> corrected_cells_;
  std::mutex mutex_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_world_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_corrected_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeightCostmapFusion>());
  rclcpp::shutdown();
  return 0;
}
