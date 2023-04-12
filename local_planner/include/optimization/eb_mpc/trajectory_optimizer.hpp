/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef TRAJECTORY_OPTIMIZER_HPP_
#define TRAJECTORY_OPTIMIZER_HPP_

// g2o lib stuff
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
// local hpp
#include "util/common.hpp"
#include "util/tools.hpp"
#include "optimization/eb_mpc/trajectory.hpp"
#include "optimization/eb_mpc/edge_reference.hpp"
#include "optimization/eb_mpc/edge_velocity_limit.hpp"
#include "optimization/eb_mpc/edge_acceleration_limit.hpp"
#include "optimization/eb_mpc/edge_kinematic.hpp"
#include "optimization/eb_mpc/edge_obstacle_limit.hpp"
#include "optimization/eb_mpc/edge_acceleration_value.hpp"
#include "optimization/eb_mpc/edge_obstacle_cost.hpp"

namespace EbMpcOptimization {

//! Typedef for the block solver utilized for optimization
typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > G2OBlockSolver;

//! Typedef for the linear solver utilized for optimization
typedef g2o::LinearSolverCSparse<G2OBlockSolver::PoseMatrixType> G2OLinearSolver;

// 轨迹优化器
class TrajectoryOptimizer {
 public:
    // 构造函数
    TrajectoryOptimizer() {};

    // 构造函数
    TrajectoryOptimizer(double robot_max_linear_velocity, double robot_min_linear_velocity, double robot_max_angular_velocity, double robot_max_acceleration, double robot_max_angular_velocity_rate);

    // 析构函数
    ~TrajectoryOptimizer() {};

    // 进行轨迹优化
    int optimizing(const std::vector<State> &reference_states, const std::vector<ObstacleAndBoundary> &obstacles_and_boundaries, double dt, std::vector<State> &opt_states);

 private:
    // 注册自定义g2o类型
    void registerG2OTypes();

    // 初始化优化器
    std::shared_ptr<g2o::SparseOptimizer> initOptimizer();

    // 初始轨迹生成
    void initTrajectory(const std::vector<State> &states);

    // 进行建图
    int buildGraph(const std::vector<State> &reference_states, const std::vector<ObstacleAndBoundary> &obstacles_and_boundaries, double dt);

    // 进行图优化
    int optimizeGraph(int iter_num);

    // 清除图
    void clearGraph();

    // 为图添加顶点
    void addVertices();

    // 添加参考轨迹边
    void addRefEgdes(const std::vector<State> &reference_states);

    // 添加速度限制边
    void addVelocityLimitEdges();

    // 添加加速度限制边
    void addAccelerationLimitEdges(double dt);

    // 添加动力学限制边
    void addKinematicEdges(double dt);

    // 添加障碍物限制边
    void addObstacleLimitEdges(const std::vector<std::vector<Line2d>> &constraint_regions);

    // 添加加速度值边
    void addAccelerationValueEdges(double dt);

    // 添加障碍物代价边
    void addObstacleCostEdges(const std::vector<ObstacleAndBoundary> &obstacles_and_boundaries);

    // 解析轨迹
    void parseTrajectory(std::vector<State> &states);

    // 机器人最大线速度
    double robot_max_linear_velocity_;
    // 机器人最小线速度
    double robot_min_linear_velocity_;
    // 机器人最大角速度
    double robot_max_angular_velocity_;
    // 机器人最大加速度
    double robot_max_acceleration_;
    // 机器人最大角加速度
    double robot_max_angular_velocity_rate_;
    // 优化器
    std::shared_ptr<g2o::SparseOptimizer> optimizer_;
    // 优化轨迹
    Trajectory trajectory_;
};

};

#endif  // TRAJECTORY_OPTIMIZER_HPP_