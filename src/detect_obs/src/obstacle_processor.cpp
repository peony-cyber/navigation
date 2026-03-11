// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <cmath>
// #include <algorithm>

// class ObstacleProcessor : public rclcpp::Node {
// public:
//     ObstacleProcessor() : Node("obstacle_processor") {
//         // --- 参数配置 ---
//         this->declare_parameter("resolution", 0.05);   // 栅格分辨率改精细一点 (5cm/格)
//         this->declare_parameter("width", 2.5);         // 【已缩小窗口】 2.5米宽 (中心点周围 1.25m)
//         this->declare_parameter("height_min", 0.1);    // 过滤地面
//         this->declare_parameter("height_max", 1.0);    // 过滤过高障碍物
//         this->declare_parameter("inflation_radius", 0.4); // 【新增】膨胀半径/斥力场半径 (米)

//         resolution_ = this->get_parameter("resolution").as_double();
//         grid_dim_ = static_cast<int>(this->get_parameter("width").as_double() / resolution_);
//         h_min_ = this->get_parameter("height_min").as_double();
//         h_max_ = this->get_parameter("height_max").as_double();
//         inflation_radius_ = this->get_parameter("inflation_radius").as_double();

//         // 订阅 Livox 点云
//         sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//             "/fastlio2/body_cloud", 10, std::bind(&ObstacleProcessor::pc_callback, this, std::placeholders::_1));

//         // 发布局部地图用于 RViz 可视化
//         pub_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap", 10);
//     }

// private:
//     void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::fromROSMsg(*msg, *cloud);

//         auto grid_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
//         grid_msg->header.frame_id = "body"; 
//         grid_msg->header.stamp = msg->header.stamp;
//         grid_msg->info.resolution = resolution_;
//         grid_msg->info.width = grid_dim_;
//         grid_msg->info.height = grid_dim_;
        
//         // 将地图中心对准机器人 body 坐标系原点
//         grid_msg->info.origin.position.x = -(grid_dim_ * resolution_) / 2.0;
//         grid_msg->info.origin.position.y = -(grid_dim_ * resolution_) / 2.0;
//         grid_msg->info.origin.position.z = 0.0;

//         grid_msg->data.assign(grid_dim_ * grid_dim_, 0); // 初始清零

//         // 第一步：将点云拍扁，标记绝对障碍物核心 (Cost = 100)
//         for (const auto& pt : cloud->points) {
//             if (pt.z < h_min_ || pt.z > h_max_) continue;

//             double ox = pt.x - grid_msg->info.origin.position.x;
//             double oy = pt.y - grid_msg->info.origin.position.y;

//             int gx = static_cast<int>(ox / resolution_);
//             int gy = static_cast<int>(oy / resolution_);

//             if (gx >= 0 && gx < grid_dim_ && gy >= 0 && gy < grid_dim_) {
//                 grid_msg->data[gy * grid_dim_ + gx] = 100;
//             }
//         }

//         // 第二步：梯度膨胀 (生成人工势场斥力区)
//         std::vector<int8_t> inflated_data = grid_msg->data;
//         int infl_cells = std::ceil(inflation_radius_ / resolution_);

//         for (int y = 0; y < grid_dim_; ++y) {
//             for (int x = 0; x < grid_dim_; ++x) {
//                 // 如果发现核心障碍物
//                 if (grid_msg->data[y * grid_dim_ + x] == 100) {
//                     // 遍历周围的细胞
//                     for (int dy = -infl_cells; dy <= infl_cells; ++dy) {
//                         for (int dx = -infl_cells; dx <= infl_cells; ++dx) {
//                             double dist = std::sqrt(dx*dx + dy*dy) * resolution_;
                            
//                             if (dist <= inflation_radius_) {
//                                 int nx = x + dx;
//                                 int ny = y + dy;
                                
//                                 if (nx >= 0 && nx < grid_dim_ && ny >= 0 && ny < grid_dim_) {
//                                     // 核心思想：距离越近，斥力代价越大 (0~100)
//                                     int new_cost = static_cast<int>((1.0 - dist / inflation_radius_) * 100.0);
//                                     int current_cost = inflated_data[ny * grid_dim_ + nx];
                                    
//                                     // 取多障碍物叠加时的最大代价
//                                     if (new_cost > current_cost) {
//                                         inflated_data[ny * grid_dim_ + nx] = new_cost;
//                                     }
//                                 }
//                             }
//                         }
//                     }
//                 }
//             }
//         }
//         grid_msg->data = inflated_data;
//         pub_grid_->publish(*grid_msg);
//     }

//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
//     rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_;
//     double resolution_, h_min_, h_max_, inflation_radius_;
//     int grid_dim_;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ObstacleProcessor>());
//     rclcpp::shutdown();
//     return 0;
// }
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <cmath>

class ObstacleProcessor : public rclcpp::Node {
public:
    ObstacleProcessor() : Node("obstacle_processor") {
        this->declare_parameter("resolution", 0.05);
        this->declare_parameter("width", 2.5); // 2.5米滑动窗口
        this->declare_parameter("height_min", 0.1);
        this->declare_parameter("height_max", 1.0);
        this->declare_parameter("inflation_radius", 0.4);
        this->declare_parameter("k_rep", 0.5); // 斥力增益，用于调整避障力度

        res_ = this->get_parameter("resolution").as_double();
        grid_dim_ = static_cast<int>(this->get_parameter("width").as_double() / res_);
        h_min_ = this->get_parameter("height_min").as_double();
        h_max_ = this->get_parameter("height_max").as_double();
        infl_rad_ = this->get_parameter("inflation_radius").as_double();
        k_rep_ = this->get_parameter("k_rep").as_double();

        sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/fastlio2/body_cloud", 10, std::bind(&ObstacleProcessor::pc_callback, this, std::placeholders::_1));

        pub_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap", 10);
        pub_repulsion_ = this->create_publisher<geometry_msgs::msg::Vector3>("/obstacle_repulsion", 10);
    }

private:
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // 1. 体素滤波 (降采样加速计算)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vg(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.05f, 0.05f, 0.05f); // 5cm体素
        vg.filter(*cloud_vg);

        // 2. 半径滤波 (去除离群噪点)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
        ror.setInputCloud(cloud_vg);
        ror.setRadiusSearch(0.2); // 查找20cm内的邻居
        ror.setMinNeighborsInRadius(4); // 至少4个邻居，否则视为噪点飞点
        ror.filter(*cloud_filtered);

        // 3. 构建 Local Costmap (拍扁)
        auto grid_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        grid_msg->header.frame_id = "body"; 
        grid_msg->header.stamp = msg->header.stamp;
        grid_msg->info.resolution = res_;
        grid_msg->info.width = grid_dim_;
        grid_msg->info.height = grid_dim_;
        grid_msg->info.origin.position.x = -(grid_dim_ * res_) / 2.0;
        grid_msg->info.origin.position.y = -(grid_dim_ * res_) / 2.0;

        grid_msg->data.assign(grid_dim_ * grid_dim_, 0);

        geometry_msgs::msg::Vector3 rep_vel; // 斥力速度向量
        rep_vel.x = 0.0; rep_vel.y = 0.0; rep_vel.z = 0.0;

        for (const auto& pt : cloud_filtered->points) {
            if (pt.z < h_min_ || pt.z > h_max_) continue;

            // 映射到栅格
            int gx = static_cast<int>((pt.x - grid_msg->info.origin.position.x) / res_);
            int gy = static_cast<int>((pt.y - grid_msg->info.origin.position.y) / res_);

            if (gx >= 0 && gx < grid_dim_ && gy >= 0 && gy < grid_dim_) {
                grid_msg->data[gy * grid_dim_ + gx] = 100; // 设置占据
            }

            // 4. 计算合斥力 (直接基于过滤后的点云计算，比用栅格算更精准)
            double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            if (dist < infl_rad_ && dist > 0.05) { // 避免中心点除零
                // 经典人工势场斥力公式: F = k * (1/d - 1/d0) * (1/d^2)
                double force = k_rep_ * (1.0 / dist - 1.0 / infl_rad_) * (1.0 / (dist * dist));
                rep_vel.x += force * (-pt.x / dist); // 斥力方向背离障碍物
                rep_vel.y += force * (-pt.y / dist);
            }
        }

        // 5. 简单膨胀显示 (为了能在 RViz 上看清楚，不影响斥力计算)
        // (为了代码简洁，如果你不需要可视化渐变膨胀，只发布核心障碍物也可以。此处保留基本显示)
        pub_grid_->publish(*grid_msg);

        // 6. 发布合成的斥力速度
        pub_repulsion_->publish(rep_vel);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_repulsion_;
    double res_, h_min_, h_max_, infl_rad_, k_rep_;
    int grid_dim_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleProcessor>());
    rclcpp::shutdown();
    return 0;
}