/*
    Copyright [2022] Jian ZhiQiang
*/

#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "util/common.hpp"
#include "util/tools.hpp"

class Robot {
 public:
    // 构造函数
    Robot(double max_v, double min_v, double max_w, double max_acc, double max_angular_acc, Pose pose={0.0, 0.0, 0.0}, Action action={0.0, 0.0}) {
        // 初始化属性
        this->max_v_ = max_v;
        this->min_v_ = min_v;
        this->max_w_ = max_w;
        this->max_acc_ = max_acc;
        this->max_angular_acc_ = max_angular_acc;
        // 初始化机器人
        this->init(pose, action);
        // 创建线程
        this->broadcast_thread_ptr_ = std::make_shared<std::thread>(&Robot::tfBroadCast, this);
        this->broadcast_thread_ptr_->detach();
    };

    // 析构函数
    ~Robot() {
        this->stop_ = true;
    };

    // 初始化
    void init(Pose pose, Action action) {
        this->lock_.lock();
        this->pose_ = pose;
        this->action_ = action;
        this->lock_.unlock();
    };

    // 更新
    void step(Action action, double time_gap) {
        this->lock_.lock();
        // 更新速度
        double delta_v = Tools::clip(action.v_ - this->action_.v_, -this->max_acc_ * time_gap, this->max_acc_ * time_gap);
        double delta_w = Tools::clip(action.w_ - this->action_.w_, -this->max_angular_acc_ * time_gap, this->max_angular_acc_ * time_gap);
        this->action_.v_ += delta_v;
        this->action_.v_ = Tools::clip(this->action_.v_, this->min_v_, this->max_v_);
        this->action_.w_ += delta_w;
        this->action_.w_ = Tools::clip(this->action_.w_, -this->max_w_, this->max_w_);
        // 更新位置
        this->pose_.theta_ += this->action_.w_ * time_gap;
        this->pose_.theta_ = Tools::pi2Pi(this->pose_.theta_);
        this->pose_.x_ += this->action_.v_ * cos(this->pose_.theta_) * time_gap;
        this->pose_.y_ += this->action_.v_ * sin(this->pose_.theta_) * time_gap;
        this->lock_.unlock();
    };

    // 获取位置
    Pose getPose() {
        this->lock_.lock();
        Pose pose = this->pose_;
        this->lock_.unlock();
        return pose;
    };

    Action getAction() {
        this->lock_.lock();
        Action action = this->action_;
        this->lock_.unlock();
        return action;
    };

 private:
    double max_v_;  // 最大速度
    double min_v_;  // 最小速度
    double max_w_;  // 最大角速度
    double max_acc_;  // 最大加速度
    double max_angular_acc_;  // 最大角加速度
    Pose pose_;  // 位置
    Action action_;  // 状态
    tf::TransformBroadcaster br_;  // 位置播放
    std::shared_ptr<std::thread> broadcast_thread_ptr_;  // 线程
    bool stop_ = false;  // 程序停止
    std::mutex lock_;  // 锁

    void tfBroadCast() {
        ros::Rate loop_rate(100);
        while (!this->stop_) {
            tf::Transform transform;
            this->lock_.lock();
            transform.setOrigin(tf::Vector3(this->pose_.x_, this->pose_.y_, 0.0));
            tf::Quaternion q;
            q.setRPY(0, 0, this->pose_.theta_);
            transform.setRotation(q);
            this->lock_.unlock();
            this->br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
            loop_rate.sleep();
        }
    };
};

#endif  // ROBOT_HPP_
