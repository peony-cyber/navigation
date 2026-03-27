#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>
#include "mpc_controller.h"

int main()
{
    // 初始化 MPC 控制器
    mpc_follower::MpcController controller;

    // 模拟路径
    std::vector<mpc_follower::PathPoint> reference_path;
    for (double i = 0.0; i < 10.0; i += 0.5) {
        mpc_follower::PathPoint wp;
        wp.x = i;
        wp.y = 0.0;
        wp.theta = 0.0;
        reference_path.push_back(wp);
    }
    controller.reference_path_ = reference_path;

    // 模拟动态障碍物
    std::vector<mpc_follower::DynamicObstacle> dynamic_obstacles;
    mpc_follower::DynamicObstacle obs;
    obs.x = 5.0;
    obs.y = 0.5;
    obs.radius = 0.3;
    dynamic_obstacles.push_back(obs);
    controller.dynamic_obstacles_ = dynamic_obstacles;

    // 模拟机器人状态
    mpc_follower::RobotState current_state;
    current_state.x = 0.0;
    current_state.y = 0.0;
    current_state.theta = 0.0;
    current_state.vx = 0.0;
    current_state.vy = 0.0;
    current_state.omega = 0.0;

    // 测量控制循环的耗时
    auto start_time = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 100; ++i) {  // 模拟 100 次控制循环
        controller.current_state_ = current_state;
        mpc_follower::Control control = controller.computeControl();
        current_state.vx = control.vx;
        current_state.vy = control.vy;
        current_state.omega = control.omega;
        current_state.x += control.vx * controller.params_.Ts;
        current_state.y += control.vy * controller.params_.Ts;
        current_state.theta += control.omega * controller.params_.Ts;
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end_time - start_time;

    // 输出结果
    std::cout << "100 次控制循环总耗时: " << duration.count() << " ms" << std::endl;
    std::cout << "平均每次控制循环耗时: " << (duration.count() / 100.0) << " ms" << std::endl;

    return 0;
}