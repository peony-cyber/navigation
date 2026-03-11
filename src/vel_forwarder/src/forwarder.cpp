#include "forwarder.h"

namespace vel_forwarder
{
    constexpr float MaxSpeed = 2.0f;
    constexpr char MaxSpeedLevel = 50;
    constexpr float SpeedRatio = MaxSpeedLevel / MaxSpeed;

    char speed_mapping(float vel_in)
    {
        if (vel_in > 10.0f)
        {
            RCLCPP_WARN(rclcpp::get_logger("forwarder"), "Bad speed input: %.2f", vel_in);
            return 0;
        }
        vel_in = vel_in > 2.4f ? 2.4f : (vel_in < -2.4f ? -2.4f : vel_in);
        return static_cast<char>(SpeedRatio * vel_in);
    }

    Forwarder::Forwarder() : Node("vel_forwarder")
    {
        sub_cmd_vel = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, 
            std::bind(&Forwarder::cmdvelCallback, this, std::placeholders::_1));
        
        pub_ly_vel = create_publisher<vel_forwarder::msg::Vel>(
            "/ly/control/vel", 10);
            rep_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/obstacle_repulsion", 10, 
            [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
                this->rep_vx_ = msg->x;
                this->rep_vy_ = msg->y;
            });
    }

    void Forwarder::cmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear.x=%.2f, linear.y=%.2f", 
                    msg->linear.x, msg->linear.y);
        auto ly_msg = vel_forwarder::msg::Vel();

        // =========================================================================
        // [避障功能模块 START] 
        // 如果不想使用避障功能，直接把下面这段 /* ... */ 注释掉即可，完全不影响原逻辑
        // =========================================================================

        // 注意：由于 vel_forwarder 可能没有里程计，如果你需要“目标邻域关闭避障”的功能，
        // 需要在这里引入 current_x 和 target_x 的获取逻辑（比如再加个订阅）。
        // 临时测试的话，可以直接假设 dist_to_goal 足够大，强制开启避障融合：
        float dist_to_goal = 10.0f; // 替换为真实的距离计算逻辑
        
        if (dist_to_goal > neighborhood_radius_) {
            // 叠加斥力 (斥力节点发过来的向量)
            final_vx += rep_vx_;
            final_vy += rep_vy_;
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Near goal, APF disabled.");
        }

        // 限幅：防止合成速度超过底盘最大允许速度
        float speed_magnitude = std::sqrt(final_vx * final_vx + final_vy * final_vy);
        if (speed_magnitude > MaxSpeed) {
            final_vx = (final_vx / speed_magnitude) * MaxSpeed;
            final_vy = (final_vy / speed_magnitude) * MaxSpeed;
        }

        // =========================================================================
        // [避障功能模块 END]
        // =========================================================================
        
        // 注意：坐标系转换，xy 交换，x 取反
        // ly_msg.x = -speed_mapping(msg->linear.y);
        // ly_msg.y = speed_mapping(msg->linear.x);
        ly_msg.x = -speed_mapping(final_vy);
        ly_msg.y = speed_mapping(final_vx);

        RCLCPP_INFO(this->get_logger(), "Mapped ly_vel: x=%d, y=%d", 
                    static_cast<int>(ly_msg.x), static_cast<int>(ly_msg.y));
        RCLCPP_DEBUG(this->get_logger(), "Forwarder: X: %d, Y: %d", 
                     static_cast<int>(ly_msg.x), static_cast<int>(ly_msg.y));
        
        
        pub_ly_vel->publish(ly_msg);
    }
    
} // namespace vel_forwarder

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vel_forwarder::Forwarder>());
    rclcpp::shutdown();
    return 0;
}