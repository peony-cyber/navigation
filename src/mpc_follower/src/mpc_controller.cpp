#include "mpc_controller.h"



namespace mpc_follower
{
    MpcController::MpcController(const rclcpp::NodeOptions & options)
        : Node("mpc_controller", options),
          global_optimizer_(nlopt::GN_CRS2_LM, 3 * 30),
          local_optimizer_(nlopt::LD_SLSQP, 3 * 30),
          rng_(std::random_device{}())
    {
        loadParameters();
        global_optimizer_ = nlopt::opt(nlopt::GN_CRS2_LM, 3 * params_.N);
        local_optimizer_ = nlopt::opt(nlopt::LD_SLSQP, 3 * params_.N);
        configureGlobalOptimizer();
        configureLocalOptimizer();
        // 初始化发布器和定时器
        //marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("mpc_predict_marker", 10);
        //path_pub_ = this->create_publisher<nav_msgs::msg::Path>("mpc_reference_path", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(params_.Ts * 1000)),
                                         std::bind(&MpcController::controlLoop, this));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 1, std::bind(&MpcController::updateRobotState, this,std::placeholders::_1));
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/sPath", 10, std::bind(&MpcController::onReferencePath,this,std::placeholders::_1));
        obstacle_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/dynamic_obstacles", 10, std::bind(&MpcController::detectObstacles,this,std::placeholders::_1));  //        
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        // // 初始化TF监听器  注释掉get_transform相关代码后暂时没用到
        // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 初始化TF广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 初始化机器人状态和控制量
        // current_state_ = {0.0, 0.0, 0.0, 0.0, 0.0};
        previous_control_ = {0.0, 0.0, 0.0};

    }
void MpcController::loadParameters()
{
    params_.Ts            = declare_parameter("Ts",           Ts);
    params_.N             = declare_parameter("N",            N);
    params_.MaxLinear     = declare_parameter("max_linear",   MaxLinear);
    params_.MinLinear     = declare_parameter("min_linear",   MinLinear);
    params_.MaxAngular    = declare_parameter("max_angular",  MaxAngular);
    params_.MinAngular    = declare_parameter("min_angular",  MinAngular);
    params_.MaxAccelLinear  = declare_parameter("max_accel_linear",  MaxAccelLinear);
    params_.MinAccelLinear  = declare_parameter("min_accel_linear",  MinAccelLinear);
    params_.MaxAccelAngular = declare_parameter("max_accel_angular", MaxAccelAngular);
    params_.MinAccelAngular = declare_parameter("min_accel_angular", MinAccelAngular);
    params_.MinR          = declare_parameter("min_turning_radius", MinR);
    params_.Wx            = declare_parameter("Wx", Wx);
    params_.Wy            = declare_parameter("Wy", Wy);
    params_.Wtheta        = declare_parameter("Wtheta", Wtheta);
    params_.Wv            = declare_parameter("Wv", Wv);
    params_.Womega        = declare_parameter("Womega", Womega);
    params_.WRadius       = declare_parameter("WRadius", WRadius);
    params_.Rv            = declare_parameter("Rv", Rv);
    params_.Romega        = declare_parameter("Romega", Romega);
    params_.OptTolerance  = declare_parameter("opt_tolerance", OptTolerance);
    params_.GlobalMaxEval = declare_parameter("global_max_eval", GlobalMaxEval);
    params_.LocalMaxEval  = declare_parameter("local_max_eval",  LocalMaxEval);
    params_.GlobalInitNoise= declare_parameter("global_init_noise", GlobalInitNoise);
    params_.ObstacleInflation = declare_parameter("obstacle_inflation", ObstacleInflation);
    params_.Wobs          = declare_parameter("Wobs", Wobs);
    // 是否启用转弯半径约束  全向禁用
    params_.UseTurningRadius = declare_parameter("use_turning_radius", false);
} 
    
//配置全局优化器
void MpcController::configureGlobalOptimizer()
{
    global_optimizer_.set_min_objective(MpcController::objectiveFunction, this);

        // 变量上下界
        std::vector<double> lb(3 * params_.N);
        std::vector<double> ub(3 * params_.N);
        for (int i = 0; i < params_.N; ++i)
        {
            lb[3 * i] = params_.MinLinear;
            ub[3 * i] = params_.MaxLinear;
            lb[3 * i + 1] = params_.MinLinear;
            ub[3 * i + 1] = params_.MaxLinear;
            lb[3 * i + 2] = params_.MinAngular;
            ub[3 * i + 2] = params_.MaxAngular;
        }
        global_optimizer_.set_lower_bounds(lb);
        global_optimizer_.set_upper_bounds(ub);

        // 为全局优化器添加约束（ISRES算法支持约束）
        // global_optimizer_.add_inequality_constraint(MPCController::accelerationConstraints, this, params_.OptTolerance);
        // global_optimizer_.add_inequality_constraint(MPCController::turningRadiusConstraint, this, params_.OptTolerance);

        // 全局优化参数
        global_optimizer_.set_xtol_rel(1e-2);
        global_optimizer_.set_maxeval(params_.GlobalMaxEval);


}
//配置局部优化器
void MpcController::configureLocalOptimizer()
{
    local_optimizer_.set_min_objective(MpcController::objectiveFunction, this);

    // 变量上下界
    std::vector<double> lb(3 * params_.N);
    std::vector<double> ub(3 * params_.N);
    for (int i = 0; i < params_.N; ++i)
    {
        lb[3 * i] = params_.MinLinear;
        ub[3 * i] = params_.MaxLinear;
        lb[3 * i + 1] = params_.MinLinear;
        ub[3 * i + 1] = params_.MaxLinear;
        lb[3 * i + 2] = params_.MinAngular;
        ub[3 * i + 2] = params_.MaxAngular;
    }
    local_optimizer_.set_lower_bounds(lb);
    local_optimizer_.set_upper_bounds(ub);

    // 应用约束
    local_optimizer_.add_inequality_constraint(MpcController::accelerationConstraints, this, params_.OptTolerance);
    // 仅当配置启用时加入转弯半径约束（对全向/舵轮平台通常禁用）
    if (params_.UseTurningRadius)
    {
        local_optimizer_.add_inequality_constraint(MpcController::turningRadiusConstraint, this, params_.OptTolerance);
    }

    // 局部优化参数
    local_optimizer_.set_xtol_rel(params_.OptTolerance);
    local_optimizer_.set_maxeval(params_.LocalMaxEval);
    local_optimizer_.set_param("initial_step", 0.01);

}

int MpcController::findNearestWaypoint(const RobotState &state, size_t last_index)
{
    (void)last_index; // 未使用参数避免编译警告
    if (reference_path_.empty())
            return 0;

    double min_dist = INFINITY;
    int nearest_idx = last_index_;
    int path_size = reference_path_.size();

    // 搜索范围：last_index_附近50个点
    for (int i = 0; i < 50; ++i)
    {
        int idx = (last_index_ + i) % path_size;
        double dx = state.x - reference_path_[idx].x;
        double dy = state.y - reference_path_[idx].y;
        double dist = dx * dx + dy * dy;

        if (dist < min_dist)
        {
            min_dist = dist;
            nearest_idx = idx;
        }
    }

    last_index_ = nearest_idx;
    return nearest_idx;
}
double MpcController::objectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    MpcController *mpc = static_cast<MpcController *> (data);
    const auto &params_ = mpc->params_;
    double cost = 0.0;
    constexpr double kEps = 1e-6;

    // 获取当前状态并找到参考路径最近点
    RobotState current = mpc->current_state_;
    int nearest_idx = mpc->findNearestWaypoint(current, mpc->last_index_);
    mpc->last_index_ = nearest_idx;

    // 预测未来状态并计算成本
    for (int i = 0; i < params_.N; ++i)
    {
        Control u = {x[3 * i], x[3 * i + 1],x[3*i + 2]};
        current = mpc->predictState(current, u, params_.Ts, mpc);
        int ref_idx = (nearest_idx + i) % mpc->reference_path_.size();
        const PathPoint &ref = mpc->reference_path_[ref_idx];

        // 位置误差成本
        cost += params_.Wx * pow(current.x - ref.x, 2);
        cost += params_.Wy * pow(current.y - ref.y, 2);
        cost += params_.Wtheta * pow(angles::shortest_angular_distance(current.theta, ref.theta), 2);

        //动态障碍物成本
        for (const auto &obs : mpc->dynamic_obstacles_)
        {
            double dx_obs = current.x - obs.x;
            double dy_obs = current.y - obs.y;
            double d_obs = hypot(dx_obs, dy_obs);
            double safe_dist = 0.5 + obs.radius; // 安全距离 = 障碍物半径 + 0.5m
            if (d_obs < safe_dist && d_obs > kEps)
                {
                    cost += params_.Wobs * exp(1.0/abs(d_obs - safe_dist));
                }
        }

       // 转弯半径成本
        if (fabs(current.omega) > kEps)  // 舵轮可以考虑转弯半径给0
        {
            double v = hypot(current.vx, current.vy);
            double radius = fabs( v / current.omega);
            if (radius > kEps)
                cost += params_.WRadius * (1.0 / radius);
        }

        // 速度平滑与惩罚成本
        // double ref_v = params_.MaxLinear;
        cost += params_.Wv * (1.0 / pow(M_E, hypot(current.vx, current.vy)));
        cost += params_.Womega * pow(current.omega, 2);

        // 控制量平滑性成本
        if (i == 0)
        {
            cost += params_.Rv * fabs(u.vx - mpc->previous_control_.vx) / params_.Ts;
            cost += params_.Rv * fabs(u.vy - mpc->previous_control_.vy) / params_.Ts;
            cost += params_.Romega * fabs(u.omega - mpc->previous_control_.omega) / params_.Ts;
        }
        else
        {
            cost += params_.Rv * fabs(u.vx - x[3 * (i - 1)]) / params_.Ts;
            cost += params_.Rv * fabs(u.vy - x[3 * (i - 1) + 1]) / params_.Ts;
            cost += params_.Romega * fabs(u.omega - x[3 * (i - 1) + 2]) / params_.Ts;
        }
    }

    // 计算梯度（仅局部优化器需要）
    if (!grad.empty() && mpc->local_optimizer_.get_algorithm() == nlopt::LD_SLSQP)
    {
        double eps = 1e-3;
        std::vector<double> x_eps = x;
        std::vector<double> tp;

        // 数值梯度计算
        for (size_t i = 0; i < x.size(); ++i)
        {
            x_eps[i] += eps;
            double f_plus = objectiveFunction(x_eps, tp, data);
            x_eps[i] -= 2 * eps;
            double f_minus = objectiveFunction(x_eps, tp, data);
            grad[i] = (f_plus - f_minus) / (2 * eps);
            x_eps[i] = x[i];
        }
    }

    return cost;
}
// 加速度约束
double MpcController::accelerationConstraints(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    MpcController *mpc = static_cast<MpcController *>(data);
    const auto &params_ = mpc->params_;
    double max_violation = 0.0;

    // 首步控制量与历史控制量的加速度约束
    double dvx = x[0] - mpc->previous_control_.vx;
    double dvy = x[1] - mpc->previous_control_.vy;
    double domega = x[2] - mpc->previous_control_.omega;
    double dv = hypot(dvx, dvy);
    double accel_linear = dv / params_.Ts;
    double accel_angular = domega / params_.Ts;
    max_violation = std::max({max_violation,
                              accel_linear - params_.MaxAccelLinear,
                              params_.MinAccelLinear - accel_linear,
                              accel_angular - params_.MaxAccelAngular,
                              params_.MinAccelAngular - accel_angular});

    // 后续控制量之间的加速度约束
    for (int i = 1; i < params_.N; ++i)
    {
        double dvx_i = x[3 * i]     - x[3 * (i - 1)];
        double dvy_i = x[3 * i + 1] - x[3 * (i - 1) + 1];
        double domega_i = x[3 * i + 2] - x[3 * (i - 1) + 2];
        double dv_i = hypot(dvx_i, dvy_i);
        double accel_linear_i = dv_i / params_.Ts;
        double accel_angular_i = domega_i / params_.Ts;
        max_violation = std::max({max_violation,
                                  accel_linear_i - params_.MaxAccelLinear,
                                  params_.MinAccelLinear - accel_linear_i,
                                  accel_angular_i - params_.MaxAccelAngular,
                                  params_.MinAccelAngular - accel_angular_i});
    }

    // 计算梯度（仅局部优化器需要）
    if (!grad.empty() && mpc->local_optimizer_.get_algorithm() == nlopt::LD_SLSQP)
        std::fill(grad.begin(), grad.end(), 0.0);

    return max_violation;
    }

// 最小转弯半径约束
double MpcController::turningRadiusConstraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    // 关键修复：从data参数获取mpc指针
    MpcController *mpc = static_cast<MpcController *>(data);
    const auto &params_ = mpc->params_;

    constexpr double eps = 1e-3;
    double max_violation = 0.0;

    // 检查每个时刻的转弯半径
    for (int i = 0; i < params_.N; ++i)
    {
        double vx = x[3 * i];
        double vy = x[3 * i + 1];
        double omega = x[3 * i + 2];
        if (fabs(omega) > eps)
        {
            double v = hypot(vx, vy);
            double radius = fabs(v / omega);
            max_violation = std::max(max_violation, params_.MinR - radius);
        }
    }

    // 计算梯度（仅局部优化器需要）
    if (!grad.empty() && mpc->local_optimizer_.get_algorithm() == nlopt::LD_SLSQP)
        std::fill(grad.begin(), grad.end(), 0.0);

    return max_violation;
}

RobotState MpcController::predictState(const RobotState &current, const Control &u, double dt, MpcController *mpc)
{
    (void)mpc; 
    RobotState next;

    double omega = u.omega;
    double theta_mid = current.theta + 0.5 * omega * dt;
    double c = std::cos(theta_mid);
    double s = std::sin(theta_mid);

    // 机体速度 (vx, vy) -> 世界系位移
    next.x = current.x + (u.vx * c - u.vy * s) * dt;
    next.y = current.y + (u.vx * s + u.vy * c) * dt;

    // 角度归一化
    double theta = current.theta + omega * dt;
    next.theta = std::atan2(std::sin(theta), std::cos(theta));

    // 保持速度量为机体框架速度
    next.vx = u.vx;
    next.vy = u.vy;
    next.omega = u.omega;
    return next;
}


// 核心优化流程
Control MpcController::computeControl()
{
    // 1. 生成全局优化的初始值（带随机扰动）
    std::vector<double> global_x0(3 * params_.N);
    std::uniform_real_distribution<double> dist_v(-params_.GlobalInitNoise, params_.GlobalInitNoise);
    std::uniform_real_distribution<double> dist_omega(-params_.GlobalInitNoise, params_.GlobalInitNoise);

    for (int i = 0; i < params_.N; ++i)
    {
        // 在历史控制量基础上添加小扰动
        global_x0[3 * i] = std::clamp(previous_control_.vx + dist_v(rng_), params_.MinLinear, params_.MaxLinear);
        global_x0[3 * i + 1] = std::clamp(previous_control_.vy + dist_v(rng_), params_.MinLinear, params_.MaxLinear);
        global_x0[3 * i + 2] = std::clamp(previous_control_.omega + dist_omega(rng_), params_.MinAngular, params_.MaxAngular);
    }

    // 2. 执行全局优化
    double global_fval = 0.0;
    nlopt::result global_result = nlopt::FAILURE;
    try
    {
        global_result = global_optimizer_.optimize(global_x0, global_fval);
        RCLCPP_WARN_STREAM(get_logger(),"Global Optimization: result=" << global_result << ", cost=" << global_fval);
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN_STREAM(get_logger(),"Global Optimization failed: " << e.what() << " (fallback to local init)");
        // 全局优化失败时，用历史控制量作为局部优化的初始值
        global_x0.assign(3 * params_.N, 0.0);
        for (int i = 0; i < params_.N; ++i)
        {
            global_x0[3 * i] = previous_control_.vx;
            global_x0[3 * i + 1] = previous_control_.vy;
            global_x0[3 * i + 2] = previous_control_.omega;
        }
    }

    // 3. 执行局部优化
    std::vector<double> local_x0 = global_x0; // 全局结果→局部初始值
    double local_fval = 0.0;
    nlopt::result local_result = nlopt::FAILURE;
    try
    {
        local_result = local_optimizer_.optimize(local_x0, local_fval);
        //ROS_DEBUG_STREAM("Local Optimization: result=" << local_result << ", cost=" << local_fval);
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN_STREAM(get_logger(),"Local Optimization failed: " << e.what() << " (fallback to previous control)");
        return previous_control_;
    }

    // 4. 结果验证与输出
    if (local_result < 0 || std::isnan(local_fval) || std::isinf(local_fval))
    {
        RCLCPP_WARN(get_logger(),"Local Optimization produced invalid result (fallback to previous control)");
        return previous_control_;
    }

    // 保存最终优化结果
    opt_result_ = local_x0; // 优化控制序列
    opt_fval_ = local_fval; // 保存最终成本值

    // 返回第一个控制量
    Control u;
    u.vx = std::clamp(local_x0[0], params_.MinLinear, params_.MaxLinear);
    u.vy = std::clamp(local_x0[1], params_.MinLinear, params_.MaxLinear);
    u.omega = std::clamp(local_x0[2], params_.MinAngular, params_.MaxAngular);
    return u;        
}

void MpcController::updateRobotState(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_state_.vx = msg->twist.twist.linear.x;
    current_state_.vy = msg->twist.twist.linear.y;
    current_state_.omega = msg->twist.twist.angular.z;

    // 更新时间戳以便 controlLoop 能识别新到达的 odom
    current_state_.stamp = msg->header.stamp;

    current_state_.x = msg->pose.pose.position.x;
    current_state_.y = msg->pose.pose.position.y;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    current_state_.theta = tf2::getYaw(q);
} 

void MpcController::onReferencePath(const nav_msgs::msg::Path::SharedPtr msg)
{
    // 在这里处理接收到的路径消息
    RCLCPP_INFO(this->get_logger(), "Received new reference path with %zu points", msg->poses.size());
    // 可以将msg转换为reference_path_格式并存储
    reference_path_.clear();
    for (const auto &pose_stamped : msg->poses)
    {   
        PathPoint wp;
        wp.x = pose_stamped.pose.position.x;
        wp.y = pose_stamped.pose.position.y;
        // 计算航向角
        // double yaw = tf2::getYaw(pose_stamped.pose.orientation);
        tf2::Quaternion q(pose_stamped.pose.orientation.x,
        pose_stamped.pose.orientation.y,
        pose_stamped.pose.orientation.z,
        pose_stamped.pose.orientation.w);
        double yaw = tf2::getYaw(q);
        wp.theta = yaw;
        reference_path_.push_back(wp);
    }
    last_index_ = 0; // 重置最近点索引

}

void MpcController::detectObstacles(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
    dynamic_obstacles_.clear();
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud, *raw);
/*------------------加窗----------------------------*/
    double L = 2.0; // 截取范围半边长
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(raw);
    crop.setMin(Eigen::Vector4f(-L, -L, -1.0, 1.0));
    crop.setMax(Eigen::Vector4f( L,  L,  2.0, 1.0));
    crop.setTranslation(Eigen::Vector3f(current_state_.x, current_state_.y, 0));
    crop.setRotation(Eigen::Vector3f(0, 0, 0));
    pcl::PointCloud<pcl::PointXYZ>::Ptr windowed(new pcl::PointCloud<pcl::PointXYZ>);
    crop.filter(*windowed);

    pcl::PassThrough<pcl::PointXYZ> pass; //直通滤波
    pass.setInputCloud(windowed);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.1, 2.0);// 只保留地面以上 0.1~2 m的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*filtered);

/*------------------------聚类-----------------------------*/
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(filtered);

    std::vector<pcl::PointIndices> cluster_indices; //簇_索引 存储聚类结果
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.5);   // 50cm内
    ec.setMinClusterSize(3);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered);
    ec.extract(cluster_indices);


    for (const auto &indices : cluster_indices)//按簇索引
    {
        DynamicObstacle obs;
        double cx = 0, cy = 0, r = 0;
        for (int idx : indices.indices)//簇内每一点索引
        {
            const auto &p = filtered->points[idx];
            cx += p.x;
            cy += p.y;
        }
        cx /= indices.indices.size();
        cy /= indices.indices.size();//计算中心
        // 半径
        for (int idx : indices.indices)
        {
            const auto &p = filtered->points[idx];
            double dx = p.x - cx, dy = p.y - cy;
            r = std::max(r, hypot(dx, dy));
        }
        obs.x = cx; obs.y = cy; obs.radius = r + 0.1; // 0.1 安全边
        dynamic_obstacles_.push_back(obs);
    }

}

// //更新速度
// void MpcController::updateVelocity(const nav_msgs::msg::Odometry::SharedPtr msg)
// {
//     current_state_.vx = msg->twist.twist.linear.x;
//     current_state_.vy = msg->twist.twist.linear.y;
//     current_state_.omega = msg->twist.twist.angular.z;
// }

//更新位姿
// bool MpcController::get_transform()
// {
//     try{
//         geometry_msgs::msg::TransformStamped t =
//         tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
//         // robot_global_pose = t;
//         current_state_.x     = t.transform.translation.x;
//         current_state_.y     = t.transform.translation.y;
//         tf2::Quaternion q(
//             t.transform.rotation.x,
//             t.transform.rotation.y,
//             t.transform.rotation.z,
//             t.transform.rotation.w);
//         current_state_.theta = tf2::getYaw(q);
//     return true;
//   }
//   catch (const tf2::TransformException & ex)
//   {
//     RCLCPP_WARN_SKIPFIRST_THROTTLE(
//       get_logger(), *get_clock(), 500,
//       "TF lookup failed: %s", ex.what());
//     return false;
//   }
// }

void MpcController::cmdVelPublish(const Control &u)
{
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = u.vx;
    cmd_vel_msg.linear.y = u.vy;
    cmd_vel_msg.angular.z = u.omega;
    cmd_vel_pub_->publish(cmd_vel_msg);
}
        

void MpcController::publishVisualization()
{
        // 发布预测轨迹
        visualization_msgs::msg::Marker predict_marker;
        predict_marker.header.frame_id = "map";
        predict_marker.header.stamp = this->now();
        predict_marker.ns = "mpc_predict";
        predict_marker.id = 0;
        predict_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
}//没用到


void MpcController::controlLoop()
{
    static rclcpp::Time last_time_stamp;
    if(current_state_.stamp == last_time_stamp)
    {
        RCLCPP_WARN(get_logger(),"No new odom data!");
        Control failed_cmd;
        failed_cmd.vx = 0.0;
        failed_cmd.vy = 0.0;
        failed_cmd.omega = 0.0;
        cmdVelPublish(failed_cmd);
        return;
    }
    last_time_stamp = current_state_.stamp;

    Control u = computeControl();   // 1. 求解 MPC
    cmdVelPublish(u);
    // 更新历史控制量，供下次优化使用
    previous_control_ = u;
    // publishVisualization();         // 3. 可视化
}

}

