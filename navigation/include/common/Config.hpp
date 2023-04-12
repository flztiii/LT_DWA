/*
    Copyright [2020] Jian ZhiQiang
*/
#pragma once
#ifndef CONFIG_H_
#define CONFIG_H_

// 定义全局变量
// define global parameters

class Config {
 public:
    // 构造函数
    Config (){}

    // 析构函数
    ~Config () {}

    // --------------------------------基础常量-----------------------------

    // 用于A星碰撞判断的机器人半径
    constexpr static double robot_size_ = 0.3;

    // 真实的机器人长度
    constexpr static double robot_length_ = 0.6;

    // 真实机器人宽度
    constexpr static double robot_width_ = 0.6;

    // 规划点在机器人的比例(0为车头,1为车尾)
    constexpr static double robot_rear_axis_center_scale_ = 0.5;

    // -------------------------------全局导航常量--------------------------

    // 栅格分辨率
    constexpr static double grid_resolution_ = 0.05;

    // 感知半径
    constexpr static double sensor_range_ = 3.5;

    // 盲区损失增量
    constexpr static double blind_cost_increment = Config::sensor_range_;

    // 起点和终点影响范围
    constexpr static double influence_range_ = 3.0;

    // 离障碍物距离最大阈值(路径平滑)
    constexpr static double smooth_max_distance_to_obs_ = 4.0;

    // 最大曲率(路径平滑)
    constexpr static double smooth_max_curvature_ = 0.1;

    // 路径平滑权重(障碍物)
    constexpr static double smooth_obs_weight_ = 1.0;

    // 路径平滑权重(曲率)
    constexpr static double smooth_cur_weight_ = 0.00000001;

    // 路径平滑权重(平滑)
    constexpr static double smooth_smoo_weight_ = 10.0;

    // 路径平滑权重(距离)
    constexpr static double smooth_dis_weight_ = 1.0;

    // 路径平滑迭代次数
    constexpr static int smooth_iter_maximum_ = 30;

    // 路径简化距离
    constexpr static double simplify_distance_ = 0.1;

    // 全局路径采样点间隔
    constexpr static double navigation_point_margin_ = 0.01;

    // 地图路点(waypoint)采样间隔
    constexpr static double waypoint_margin_ = 0.1;

    // 全局导航路径最短长度
    constexpr static double navigation_shortest_length_ = 1.0;

    // --------------------------------局部规划常量---------------------------------

    // 局部路径采样点间隔
    constexpr static double local_planning_point_margin_ = 0.01;

    // 目标点采样数量
    constexpr static double local_planning_sample_num_ = 20;

    // 目标点采样间隔
    constexpr static double local_planning_sample_gap_ = 0.05;

    // 离障碍物距离最大阈值(局部规划)
    constexpr static double local_planning_max_distance_to_obs_ = 4.0;

    // 最大可接受曲率
    constexpr static double max_curvature_ = 2.4;

    // 局部规划长度系数
    constexpr static double expected_length_coef_ = 2.0;

    // 局部规划长度常量
    constexpr static double expected_length_const_ = 1.0;
};

#endif