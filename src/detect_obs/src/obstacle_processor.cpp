#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <algorithm>

class ObstacleProcessor : public rclcpp::Node {
public:
    ObstacleProcessor() : Node("obstacle_processor") {
        // --- 参数配置 ---
        this->declare_parameter("resolution", 0.05);   // 栅格分辨率改精细一点 (5cm/格)
        this->declare_parameter("width", 2.5);         // 【已缩小窗口】 2.5米宽 (中心点周围 1.25m)
        this->declare_parameter("height_min", 0.1);    // 过滤地面
        this->declare_parameter("height_max", 1.0);    // 过滤过高障碍物
        this->declare_parameter("inflation_radius", 0.4); // 【新增】膨胀半径/斥力场半径 (米)

        resolution_ = this->get_parameter("resolution").as_double();
        grid_dim_ = static_cast<int>(this->get_parameter("width").as_double() / resolution_);
        h_min_ = this->get_parameter("height_min").as_double();
        h_max_ = this->get_parameter("height_max").as_double();
        inflation_radius_ = this->get_parameter("inflation_radius").as_double();

        // 订阅 Livox 点云
        sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/fastlio2/body_cloud", 10, std::bind(&ObstacleProcessor::pc_callback, this, std::placeholders::_1));

        // 发布局部地图用于 RViz 可视化
        pub_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap", 10);
    }

private:
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        auto grid_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        grid_msg->header.frame_id = "body"; 
        grid_msg->header.stamp = msg->header.stamp;
        grid_msg->info.resolution = resolution_;
        grid_msg->info.width = grid_dim_;
        grid_msg->info.height = grid_dim_;
        
        // 将地图中心对准机器人 body 坐标系原点
        grid_msg->info.origin.position.x = -(grid_dim_ * resolution_) / 2.0;
        grid_msg->info.origin.position.y = -(grid_dim_ * resolution_) / 2.0;
        grid_msg->info.origin.position.z = 0.0;

        grid_msg->data.assign(grid_dim_ * grid_dim_, 0); // 初始清零

        // 第一步：将点云拍扁，标记绝对障碍物核心 (Cost = 100)
        for (const auto& pt : cloud->points) {
            if (pt.z < h_min_ || pt.z > h_max_) continue;

            double ox = pt.x - grid_msg->info.origin.position.x;
            double oy = pt.y - grid_msg->info.origin.position.y;

            int gx = static_cast<int>(ox / resolution_);
            int gy = static_cast<int>(oy / resolution_);

            if (gx >= 0 && gx < grid_dim_ && gy >= 0 && gy < grid_dim_) {
                grid_msg->data[gy * grid_dim_ + gx] = 100;
            }
        }

        // 第二步：梯度膨胀 (生成人工势场斥力区)
        std::vector<int8_t> inflated_data = grid_msg->data;
        int infl_cells = std::ceil(inflation_radius_ / resolution_);

        for (int y = 0; y < grid_dim_; ++y) {
            for (int x = 0; x < grid_dim_; ++x) {
                // 如果发现核心障碍物
                if (grid_msg->data[y * grid_dim_ + x] == 100) {
                    // 遍历周围的细胞
                    for (int dy = -infl_cells; dy <= infl_cells; ++dy) {
                        for (int dx = -infl_cells; dx <= infl_cells; ++dx) {
                            double dist = std::sqrt(dx*dx + dy*dy) * resolution_;
                            
                            if (dist <= inflation_radius_) {
                                int nx = x + dx;
                                int ny = y + dy;
                                
                                if (nx >= 0 && nx < grid_dim_ && ny >= 0 && ny < grid_dim_) {
                                    // 核心思想：距离越近，斥力代价越大 (0~100)
                                    int new_cost = static_cast<int>((1.0 - dist / inflation_radius_) * 100.0);
                                    int current_cost = inflated_data[ny * grid_dim_ + nx];
                                    
                                    // 取多障碍物叠加时的最大代价
                                    if (new_cost > current_cost) {
                                        inflated_data[ny * grid_dim_ + nx] = new_cost;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        grid_msg->data = inflated_data;
        pub_grid_->publish(*grid_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_;
    double resolution_, h_min_, h_max_, inflation_radius_;
    int grid_dim_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleProcessor>());
    rclcpp::shutdown();
    return 0;
}