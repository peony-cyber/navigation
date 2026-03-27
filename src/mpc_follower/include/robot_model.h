#pragma once
#include <vector>
#include <cmath>
#include <iostream>
#include <limits>
#include <angles/angles.h>
#include "rclcpp/rclcpp.hpp"

namespace mpc_follower
{
    class MpcController; // 前向声明

    // 机器人状态结构体 //考虑 v
    struct RobotState
    {
        double x{};     // x坐标
        double y{};     // y坐标
        double theta{}; // 朝向角 (rad)
        double vx{};     // x方向线速度 (m/s)
        double vy{};     // y方向线速度 (m/s)
        double omega{}; // 角速度 (rad/s)
        rclcpp::Time stamp; // 时间戳
    };

    // 控制量结构体
    struct Control
    {
        double vx{};     // 线速度控制量
        double vy{};     // y方向线速度控制量
        double omega{}; // 角速度控制量
    };

    // 路径点结构体
    struct PathPoint
    {
        double x;     // x坐标
        double y;     // y坐标
        double theta; // 期望朝向角
    };

    inline double min_radius = std::numeric_limits<float>::max();

    // 控制参数和约束

    // 时间参数
    inline constexpr double Ts = 0.05; // 控制周期 (s)
    inline constexpr int N = 30;       // 预测时域长度

    // 机器人物理约束
    inline constexpr double MinR = 0;        // 最小转弯半径 (m) 舵轮可以为0
    inline constexpr double MaxLinear = 2.0;   // 最大线速度 (m/s)
    inline constexpr double MinLinear = -2.0;  // 最小线速度 (m/s)
    inline constexpr double MaxAngular = 2.0;  // 最大角速度 (rad/s)
    inline constexpr double MinAngular = -2.0; // 最小角速度 (rad/s)

    // 加速度约束
    inline constexpr double MaxAccelLinear = 1.5;  // 最大线加速度 (m/s²)
    inline constexpr double MinAccelLinear = -1.5; // 最大线减速度 (m/s²)
    inline constexpr double MaxAccelAngular = 6;   // 最大角加速度 (rad/s²)
    inline constexpr double MinAccelAngular = -6;  // 最大角减速度 (rad/s²)

    // MPC权重
    inline constexpr double Wx = 20.0;          // x位置权重
    inline constexpr double Wy = 20.0;          // y位置权重
    inline constexpr double Wtheta = 10.0;      // 朝向角权重
    inline constexpr double Wv = 0.02;          // 线速度权重
    inline constexpr double Womega = 0.1;       // 角速度权重
    inline constexpr double WRadius = 1.0;      // 转弯半径权重
    inline constexpr double kStopPenalty = 0.0; // 停止惩罚
    inline constexpr double Rv = 0.1;           // 线速度变化权重
    inline constexpr double Romega = 0.1;       // 角速度变化权重

    // 优化器参数
    inline constexpr double OptTolerance = 1e-3;   // 优化容差
    inline constexpr int GlobalMaxEval = 500;      // 全局优化最大评估次数
    inline constexpr int LocalMaxEval = 1000;      // 局部优化最大评估次数
    inline constexpr double GlobalInitNoise = 0.2; // 全局优化初始值扰动范围

    inline constexpr double ObstacleInflation = 0.5; // 障碍物膨胀距离 (m)
    inline constexpr double Wobs = 100.0;   // 障碍物避障惩罚权重


    // 动态障碍描述
    struct DynamicObstacle
    {
        double x;    // 当前世界坐标 x
        double y;    // 当前世界坐标 y
         double radius; // 障碍物半径
    };       
}