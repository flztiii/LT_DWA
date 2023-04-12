/*
    Copyright [2022] Jian ZhiQiang
*/

#ifndef SEED_POLICY_HPP_
#define SEED_POLICY_HPP_

#include "util/grid_map.hpp"
#include "util/kdtree.hpp"
#include "util/obstacle_and_boundary.hpp"
#include "optimization/penalties.h"
#include "optimization/eb_mpc/trajectory_optimizer.hpp"
#include "policy/policy.hpp"
#include <array>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <random>
#include <fstream>
#include <boost/multi_array.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <local_map_generation/GetLocalMap.h>
#include <obstacle_msgs/CircleObstacles.h>
#include <obstacle_msgs/CircleObstacle.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// 种子凸区域策略
class SeedPolicy: public Policy {
 public:
    // 构造函数
    SeedPolicy(double max_v, double min_v, double max_w, double max_acc, double max_angular_acc, double robot_radius, double time_step, const ros::ServiceClient &local_map_service): Policy(max_v, min_v, max_w, max_acc, max_angular_acc, robot_radius, time_step) {
        this->local_map_service_ = local_map_service;
        // 定义轨迹优化器
        this->eb_mpc_trajectory_optimizer_ = EbMpcOptimization::TrajectoryOptimizer(max_v, min_v, max_w, max_acc, max_angular_acc);
        // 配置文件
        std::string file_path = ros::package::getPath("local_planner") + "/config/planning.config";
        if (!std::filesystem::exists(file_path)) {
            std::cerr << "config file not exist" << std::endl;
            exit(0);
        }
        boost::property_tree::ini_parser::read_ini(file_path, this->pt_);
        // [debug]進行可視化
        ros::NodeHandle nh("~");
        this->seed_tree_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/local_planner/seed_tree_vis", 10);
        this->best_seed_sequence_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/local_planner/best_seed_sequence_vis", 10);
        this->debug_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/local_planner/debug_map", 10);
        this->debug_cost_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/local_planner/debug_cost", 10);
        this->debug_nav_pub_= nh.advertise<visualization_msgs::MarkerArray>("/local_planner/debug_nav", 10);
        this->debug_optimize_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/local_planner/debug_optimize", 10);
    };

    // 析构函数
    ~SeedPolicy() {};

    // 生成行为
    int forward(Robot &robot, const Pose &target_pose, const std::vector<PathPose> &navigation_path, const GridMap &global_map, const std::map<int, Tools::FixedQueue<ObstacleInfo, OBSTACLE_INFO_LEN>> &obstacles_info, Action &planned_action);
 
 private:
    ros::ServiceClient local_map_service_;
    boost::property_tree::ptree pt_;
    std::mt19937 rand_gen_ = std::mt19937(0);
    EbMpcOptimization::TrajectoryOptimizer eb_mpc_trajectory_optimizer_;
    ros::Publisher seed_tree_vis_pub_;
    ros::Publisher best_seed_sequence_vis_pub_;
    ros::Publisher debug_map_pub_;
    ros::Publisher debug_cost_pub_;
    ros::Publisher debug_nav_pub_;
    ros::Publisher debug_optimize_pub_;

    // 节点结构体
    struct Node {
        // 属性
        State state_;
        std::shared_ptr<Node> parent_;
        double cost_;
    };

    static inline bool compareNode(const std::shared_ptr<Node> &n1, const std::shared_ptr<Node> & n2) {
        return Tools::isSmall(n1->cost_, n2->cost_);
    }

    // 进行区域搜索
    std::vector<std::vector<Line2d>> searchPolyRegions(const std::vector<State> &states, const std::array<GridMap, PREDICT_TIME_LEN> &grid_maps, const std::array<KDTree, PREDICT_TIME_LEN> &kdtrees);

    // 进行区域搜索
    std::vector<CircleRegion> searchCircleRegions(const std::vector<State> &states, const std::array<GridMap, PREDICT_TIME_LEN> &grid_maps, const std::array<KDTree, PREDICT_TIME_LEN> &kdtrees);

    // 进行序列验证
    void verifySequence(const State &current_state, const std::vector<State> &best_seed_sequence, const std::vector<Acceleration> &accelerations);

    // 根据状态序列估计运动序列
    std::vector<Acceleration> accelerationEstimation(const std::vector<State> &states, double dt);

    // 选择最优种子点
    std::vector<std::vector<State>> selectBestSeeds(const std::vector<std::vector<std::shared_ptr<Node>>> &seed_tree, size_t max_num=1);

    // 生成种子点
    void generateSeeds(const State &current_state, const std::vector<State> &ref_states, const std::array<GridMap, PREDICT_TIME_LEN> &predict_gridmaps, const std::vector<ObstacleAndBoundary> & obstacles_and_boundaries, const std::string &sample_method, std::vector<std::vector<std::shared_ptr<Node>>> &seed_tree);

    // 计算状态序列代价
    double calcStateSequenceCost(const std::vector<State> &states, const std::vector<ObstacleAndBoundary> &obstacles_and_boundaries, const std::vector<State> &ref_states);

    // 计算状态代价
    double calcCost(const State &state, const ObstacleAndBoundary &obstacle_and_boundary, const State &ref_state);

    // 栅格化体素采样
    std::vector<std::shared_ptr<Node>> voxelGridSampling(const std::vector<std::shared_ptr<Node>> &raw_seed_nodes);

    // 速度空间采样的状态拓展
    std::vector<State> stateExpanding(const State &current_state, int max_v_sample_num, int max_w_sample_num);

    // 获取未来的栅格图
    std::array<GridMap, PREDICT_TIME_LEN> predictFutureEnvironment(Robot &robot, const GridMap &global_map, const std::map<int, Tools::FixedQueue<ObstacleInfo, OBSTACLE_INFO_LEN>> &obstacles_info);

    // 生成参考状态序列
    std::vector<State> generateReferenceStates(const std::vector<PathPose> &navigation_path);

    // 计算路径坐标转换
    std::vector<PathPose> transformPathToNewCoordinate(const Pose &new_coordination_origin, const std::vector<PathPose> &path);
};

#endif  // SEED_POLICY_HPP_
