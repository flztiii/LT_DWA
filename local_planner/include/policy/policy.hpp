/*
    Copyright [2022] Jian ZhiQiang
*/

#ifndef POLICY_HPP_
#define POLICY_HPP_

#include "util/common.hpp"
#include "util/tools.hpp"
#include "util/robot.hpp"
#include "util/grid_map.hpp"

// 策略基类
class Policy {
 public:
    // 构造函数
    Policy(double max_v, double min_v, double max_w, double max_acc, double max_angular_acc, double robot_radius, double time_step) {
        this->max_v_ = max_v;
        this->min_v_ = min_v;
        this->max_w_ = max_w;
        this->max_acc_ = max_acc;
        this->max_angular_acc_ = max_angular_acc;
        this->robot_radius_ = robot_radius;
        this->time_step_ = time_step;
    };

    // 析构函数
    ~Policy() {};

    // 规划函数（需要实现）
    virtual int forward(Robot &robot, const Pose &target_pose, const std::vector<PathPose> &navigation_path, const GridMap &global_map, const std::map<int, Tools::FixedQueue<ObstacleInfo, OBSTACLE_INFO_LEN>> &obstacles_info, Action &planned_action) = 0;

 protected:
    double max_v_;
    double min_v_;
    double max_w_;
    double max_acc_;
    double max_angular_acc_;
    double robot_radius_;
    double time_step_;
};

#endif  // POLICY_HPP_
