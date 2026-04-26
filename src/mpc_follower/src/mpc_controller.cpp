#include "mpc_controller.h"

namespace mpc_follower
{
    MpcController::MpcController(const rclcpp::NodeOptions & options)
        : Node("mpc_controller", options),
          global_optimizer_(nlopt::GN_CRS2_LM, 2 * 30),  // 【修改】3->2
          local_optimizer_(nlopt::LD_SLSQP, 2 * 30),       // 【修改】3->2
          rng_(std::random_device{}())
    {
        loadParameters();
        global_optimizer_ = nlopt::opt(nlopt::GN_CRS2_LM, 2 * params_.N);  // 【修改】3->2
        local_optimizer_ = nlopt::opt(nlopt::LD_SLSQP, 2 * params_.N);     // 【修改】3->2
        configureGlobalOptimizer();
        configureLocalOptimizer();
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(params_.Ts * 1000)),
                                         std::bind(&MpcController::controlLoop, this));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 1, std::bind(&MpcController::updateRobotState, this,std::placeholders::_1));
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/sPath", 10, std::bind(&MpcController::onReferencePath,this,std::placeholders::_1));
        obstacle_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/dynamic_obstacles", 10, std::bind(&MpcController::detectObstacles,this,std::placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // 【修改】只初始化 vx, vy
        previous_control_ = {0.0, 0.0};
    }

void MpcController::loadParameters()
{
    params_.Ts            = declare_parameter("Ts",           Ts);
    params_.N             = declare_parameter("N",            N);
    params_.MaxLinear     = declare_parameter("max_linear",   MaxLinear);
    params_.MinLinear     = declare_parameter("min_linear",   MinLinear);
    params_.MaxAccelLinear  = declare_parameter("max_accel_linear",  MaxAccelLinear);
    params_.MinAccelLinear  = declare_parameter("min_accel_linear",  MinAccelLinear);
    params_.MinR          = declare_parameter("min_turning_radius", MinR);  // 【保留但不再使用】
    params_.Wx            = declare_parameter("Wx", Wx);
    params_.Wy            = declare_parameter("Wy", Wy);
    params_.Wv            = declare_parameter("Wv", Wv);
    params_.WRadius       = declare_parameter("WRadius", WRadius);  // 【保留但不再使用】
    params_.Rv            = declare_parameter("Rv", Rv);
    params_.OptTolerance  = declare_parameter("opt_tolerance", OptTolerance);
    params_.GlobalMaxEval = declare_parameter("global_max_eval", GlobalMaxEval);
    params_.LocalMaxEval  = declare_parameter("local_max_eval",  LocalMaxEval);
    params_.GlobalInitNoise= declare_parameter("global_init_noise", GlobalInitNoise);
    params_.ObstacleInflation = declare_parameter("obstacle_inflation", ObstacleInflation);
    params_.Wobs          = declare_parameter("Wobs", Wobs);


    // std::cout << "Ts" << params_.Ts << std::endl;
} 
    
// 【修改】配置全局优化器：维度改为 2*N (vx, vy)
void MpcController::configureGlobalOptimizer()
{
    global_optimizer_.set_min_objective(MpcController::objectiveFunction, this);

        std::vector<double> lb(2 * params_.N);  // 【修改】3->2
        std::vector<double> ub(2 * params_.N);  // 【修改】3->2
        for (int i = 0; i < params_.N; ++i)
        {
            lb[2 * i] = params_.MinLinear;      // vx
            ub[2 * i] = params_.MaxLinear;
            lb[2 * i + 1] = params_.MinLinear;  // vy
            ub[2 * i + 1] = params_.MaxLinear;
        }
        global_optimizer_.set_lower_bounds(lb);
        global_optimizer_.set_upper_bounds(ub);

        global_optimizer_.set_xtol_rel(1e-2);
        global_optimizer_.set_maxeval(params_.GlobalMaxEval);
}

//配置局部优化器
void MpcController::configureLocalOptimizer()
{
    local_optimizer_.set_min_objective(MpcController::objectiveFunction, this);
    local_optimizer_.add_inequality_constraint(
    MpcController::accelerationConstraints, this, params_.OptTolerance);

    std::vector<double> lb(2 * params_.N);  // 【修改】3->2
    std::vector<double> ub(2 * params_.N);  // 【修改】3->2
    for (int i = 0; i < params_.N; ++i)
    {
        lb[2 * i] = params_.MinLinear;      // vx
        ub[2 * i] = params_.MaxLinear;
        lb[2 * i + 1] = params_.MinLinear;  // vy  
        ub[2 * i + 1] = params_.MaxLinear;
    }
    local_optimizer_.set_lower_bounds(lb);
    local_optimizer_.set_upper_bounds(ub);


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

    // 搜索范围：last_index_附近10个点
    for (int i = 0; i < 10; ++i)
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

    RobotState current = mpc->current_state_;
    int nearest_idx = mpc->findNearestWaypoint(current, mpc->last_index_);
    mpc->last_index_ = nearest_idx;

    for (int i = 0; i < params_.N; ++i)
    {
        Control u = {x[2 * i], x[2 * i + 1]};
        
        current = mpc->predictState(current, u, params_.Ts, mpc);
        int ref_idx = (nearest_idx + i) % mpc->reference_path_.size();
        const PathPoint &ref = mpc->reference_path_[ref_idx];

        //位置误差成本
        cost += params_.Wx * pow(current.x - ref.x, 2);
        cost += params_.Wy * pow(current.y - ref.y, 2);

        //动态障碍物成本
        for (const auto &obs : mpc->dynamic_obstacles_)
        {
            double dx_obs = current.x - obs.x;
            double dy_obs = current.y - obs.y;
            double d_obs = hypot(dx_obs, dy_obs);
            double safe_dist = 0.5 + obs.radius;
            if (d_obs < safe_dist && d_obs > kEps)
            {
                cost += params_.Wobs * exp(1.0/abs(d_obs - safe_dist));
            }
        }
        
        cost += params_.Wv * (1.0 / pow(M_E, hypot(current.vx, current.vy)));

        //控制量平滑性
        if (i == 0)
        {
            cost += params_.Rv * fabs(u.vx - mpc->previous_control_.vx) / params_.Ts;
            cost += params_.Rv * fabs(u.vy - mpc->previous_control_.vy) / params_.Ts;
        }
        else
        {
            cost += params_.Rv * fabs(u.vx - x[2 * (i - 1)]) / params_.Ts;       // 【修改】3*(i-1) -> 2*(i-1)
            cost += params_.Rv * fabs(u.vy - x[2 * (i - 1) + 1]) / params_.Ts;    // 【修改】3*(i-1)+1 -> 2*(i-1)+1
        }
    }

    if (!grad.empty() && mpc->local_optimizer_.get_algorithm() == nlopt::LD_SLSQP)
    {
        double eps = 1e-3;
        std::vector<double> x_eps = x;
        std::vector<double> tp;

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

//线加速度约束（如需）或直接删除整个函数 if 只使用边界约束
double MpcController::accelerationConstraints(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    MpcController *mpc = static_cast<MpcController *>(data);
    const auto &params_ = mpc->params_;
    double max_violation = 0.0;

    //只计算线加速度，删除 domega
    double dvx = x[0] - mpc->previous_control_.vx;
    double dvy = x[1] - mpc->previous_control_.vy;
    double dv = hypot(dvx, dvy);
    double accel_linear = dv / params_.Ts;
    
    max_violation = std::max({max_violation,
                              accel_linear - params_.MaxAccelLinear,
                              params_.MinAccelLinear - accel_linear});

    for (int i = 1; i < params_.N; ++i)
    {
        double dvx_i = x[2 * i]     - x[2 * (i - 1)];      // 【修改】3*i -> 2*i
        double dvy_i = x[2 * i + 1] - x[2 * (i - 1) + 1];  // 【修改】3*i+1 -> 2*i+1
        
        double dv_i = hypot(dvx_i, dvy_i);
        double accel_linear_i = dv_i / params_.Ts;
        
        max_violation = std::max({max_violation,
                                  accel_linear_i - params_.MaxAccelLinear,
                                  params_.MinAccelLinear - accel_linear_i});
                                  // 【删除】角加速度约束
    }

    if (!grad.empty() && mpc->local_optimizer_.get_algorithm() == nlopt::LD_SLSQP)
        std::fill(grad.begin(), grad.end(), 0.0);

    return max_violation;
}

double MpcController::turningRadiusConstraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    //
    (void)x; (void)grad; (void)data;
    return 0.0;
}

// 【修改】状态预测：omega 强制为 0，theta 保持不变
RobotState MpcController::predictState(const RobotState &current, const Control &u, double dt, MpcController *mpc)
{
    (void)mpc; 
    RobotState next;

    
    double theta_mid = current.theta;  // 【修改】原：current.theta + 0.5 * omega * dt
    
    double c = std::cos(theta_mid);
    double s = std::sin(theta_mid);

    // 【保留】机体速度 -> 世界系位移
    next.x = current.x + (u.vx * c - u.vy * s) * dt;
    next.y = current.y + (u.vx * s + u.vy * c) * dt;

    // 【修改】theta 保持不变（因为 omega=0）
    next.theta = current.theta; 
    
    // 【保留】速度量传递
    next.vx = u.vx;
    next.vy = u.vy;
    
    return next;
}

Control MpcController::computeControl()
{
    std::vector<double> global_x0(2 * params_.N);  // 【修改】3->2
    std::uniform_real_distribution<double> dist_v(-params_.GlobalInitNoise, params_.GlobalInitNoise);

    for (int i = 0; i < params_.N; ++i)
    {
        global_x0[2 * i] = std::clamp(previous_control_.vx + dist_v(rng_), params_.MinLinear, params_.MaxLinear);
        global_x0[2 * i + 1] = std::clamp(previous_control_.vy + dist_v(rng_), params_.MinLinear, params_.MaxLinear);
    }

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
        global_x0.assign(2 * params_.N, 0.0);
        for (int i = 0; i < params_.N; ++i)
        {
            global_x0[2 * i] = previous_control_.vx;
            global_x0[2 * i + 1] = previous_control_.vy;
        }
    }

    std::vector<double> local_x0 = global_x0;
    double local_fval = 0.0;
    nlopt::result local_result = nlopt::FAILURE;
    try
    {
        local_result = local_optimizer_.optimize(local_x0, local_fval);
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN_STREAM(get_logger(),"Local Optimization failed: " << e.what() << " (fallback to previous control)");
        return previous_control_;
    }

    if (local_result < 0 || std::isnan(local_fval) || std::isinf(local_fval))
    {
        RCLCPP_WARN(get_logger(),"Local Optimization produced invalid result (fallback to previous control)");
        return previous_control_;
    }

    opt_result_ = local_x0;
    opt_fval_ = local_fval;

    Control u;
    u.vx = std::clamp(local_x0[0], params_.MinLinear, params_.MaxLinear);
    u.vy = std::clamp(local_x0[1], params_.MinLinear, params_.MaxLinear);
    
    return u;        
}

void MpcController::updateRobotState(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_state_.vx = msg->twist.twist.linear.x;
    current_state_.vy = msg->twist.twist.linear.y;

    current_state_.stamp = msg->header.stamp;

    current_state_.x = msg->pose.pose.position.x;
    current_state_.y = msg->pose.pose.position.y;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    current_state_.theta = tf2::getYaw(q);
    // 【注意】current_state_.omega 保持为 0 或最后一次值，不再更新
} 

void MpcController::onReferencePath(const nav_msgs::msg::Path::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received new reference path with %zu points", msg->poses.size());
    reference_path_.clear();
    for (const auto &pose_stamped : msg->poses)
    {   
        PathPoint wp;
        wp.x = pose_stamped.pose.position.x;
        wp.y = pose_stamped.pose.position.y;
        tf2::Quaternion q(pose_stamped.pose.orientation.x,
        pose_stamped.pose.orientation.y,
        pose_stamped.pose.orientation.z,
        pose_stamped.pose.orientation.w);
        double yaw = tf2::getYaw(q);
        wp.theta = yaw;
        reference_path_.push_back(wp);
    }
    last_index_ = 0;
}

// 【保留】障碍物检测（无修改）
void MpcController::detectObstacles(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
    dynamic_obstacles_.clear();
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud, *raw);

    double L = 2.0;
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(raw);
    crop.setMin(Eigen::Vector4f(-L, -L, -1.0, 1.0));
    crop.setMax(Eigen::Vector4f( L,  L,  2.0, 1.0));
    crop.setTranslation(Eigen::Vector3f(current_state_.x, current_state_.y, 0));
    crop.setRotation(Eigen::Vector3f(0, 0, 0));
    pcl::PointCloud<pcl::PointXYZ>::Ptr windowed(new pcl::PointCloud<pcl::PointXYZ>);
    crop.filter(*windowed);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(windowed);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.1, 2.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*filtered);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(3);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered);
    ec.extract(cluster_indices);

    for (const auto &indices : cluster_indices)
    {
        DynamicObstacle obs;
        double cx = 0, cy = 0, r = 0;
        for (int idx : indices.indices)
        {
            const auto &p = filtered->points[idx];
            cx += p.x;
            cy += p.y;
        }
        cx /= indices.indices.size();
        cy /= indices.indices.size();
        for (int idx : indices.indices)
        {
            const auto &p = filtered->points[idx];
            double dx = p.x - cx, dy = p.y - cy;
            r = std::max(r, hypot(dx, dy));
        }
        obs.x = cx; obs.y = cy; obs.radius = r + 0.1;
        dynamic_obstacles_.push_back(obs);
    }
}

void MpcController::cmdVelPublish(const Control &u)
{
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = u.vx;
    cmd_vel_msg.linear.y = u.vy;
    cmd_vel_pub_->publish(cmd_vel_msg);
}
        
// 【保留】可视化（未使用）
void MpcController::publishVisualization()
{
    visualization_msgs::msg::Marker predict_marker;
    predict_marker.header.frame_id = "map";
    predict_marker.header.stamp = this->now();
    predict_marker.ns = "mpc_predict";
    predict_marker.id = 0;
    predict_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
}

// 【修改】主循环：previous_control_ 更新不再包含 omega
void MpcController::controlLoop()
{
    static rclcpp::Time last_time_stamp;
    if(current_state_.stamp == last_time_stamp)
    {
        RCLCPP_WARN(get_logger(),"No new odom data!");
        Control failed_cmd;
        failed_cmd.vx = 0.0;
        failed_cmd.vy = 0.0;
        cmdVelPublish(failed_cmd);
        return;
    }
    last_time_stamp = current_state_.stamp;

    Control u = computeControl();
    cmdVelPublish(u);
    previous_control_ = u;  // 【保留】但 u.omega 始终为 0
}
}