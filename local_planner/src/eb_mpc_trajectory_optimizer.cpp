#include "optimization/eb_mpc/trajectory_optimizer.hpp"

namespace EbMpcOptimization {

// 构造函数
TrajectoryOptimizer::TrajectoryOptimizer(double robot_max_linear_velocity, double robot_min_linear_velocity, double robot_max_angular_velocity, double robot_max_acceleration, double robot_max_angular_velocity_rate) {
    // 定义基础属性
    this->robot_max_linear_velocity_ = robot_max_linear_velocity;
    this->robot_min_linear_velocity_ = robot_min_linear_velocity;
    this->robot_max_angular_velocity_ = robot_max_angular_velocity;
    this->robot_max_acceleration_ = robot_max_acceleration;
    this->robot_max_angular_velocity_rate_ = robot_max_angular_velocity_rate;
    // 注册自定义g2o类型
    static std::once_flag flag;
    std::call_once(flag, &TrajectoryOptimizer::registerG2OTypes, this);
    // 初始化优化器
    this->optimizer_ = this->initOptimizer();
}

// 进行轨迹优化
int TrajectoryOptimizer::optimizing(const std::vector<State> &reference_states, const std::vector<ObstacleAndBoundary> &obstacles_and_boundaries, double dt, std::vector<State> &opt_states) {
    // 优化结果
    int success = -1;
    assert(opt_states.size() > 1);
    // 设置优化器的是否可视化中间过程
    this->optimizer_->setComputeBatchStatistics(false);
    // 初始化轨迹数据
    this->initTrajectory(opt_states);
    // 建图
    success = this->buildGraph(reference_states, obstacles_and_boundaries, dt);
    // 判断建图是否成功
    if (success < 0) {
        std::cout << "[Error]Cannot build graph, call clearGraph()!" << std::endl;
        // 清除图
        this->clearGraph();
        // 清除轨迹数据
        this->trajectory_.clearTrajectory();
        return -1;
    }
    // 图优化
    success = this->optimizeGraph(SINGLE_OPT_ITER_NUM);
    // 判断优化图是否成功
    if (success < 0) {
        std::cout << "[Error]Cannot optimize graph, call clearGraph()!" << std::endl;
        // 清除图
        this->clearGraph();
        // 清除轨迹数据
        this->trajectory_.clearTrajectory();
        return -1;
    }
    // 解析轨迹数据
    this->parseTrajectory(opt_states);
    // 清除图
    this->clearGraph();
    // 清除轨迹数据
    this->trajectory_.clearTrajectory();
    return 0;
}

// 注册自定义g2o类型
void TrajectoryOptimizer::registerG2OTypes() {
    g2o::Factory* factory = g2o::Factory::instance();
    // 注册顶点
    factory->registerType("VERTEX_STATE", new g2o::HyperGraphElementCreator<VertexState>);

    factory->registerType("EDGE_REFERENCE", new g2o::HyperGraphElementCreator<EdgeReference>);
    factory->registerType("EDGE_VELOCITY_LIMIT", new g2o::HyperGraphElementCreator<EdgeVelocityLimit>);
    factory->registerType("EDGE_ACCELERATION_LIMIT", new g2o::HyperGraphElementCreator<EdgeAccelerationLimit>);
    factory->registerType("EDGE_KINEMATIC", new g2o::HyperGraphElementCreator<EdgeKinematic>);
    factory->registerType("EDGE_OBSTACLE_LIMIT", new g2o::HyperGraphElementCreator<EdgeObstacleLimit>);
    factory->registerType("EDGE_ACCELERATION_VALUE", new g2o::HyperGraphElementCreator<EdgeAccelerationValue>);
    factory->registerType("EDGE_OBSTACLE_COST", new g2o::HyperGraphElementCreator<EdgeObstacleCost>);
    return;
}

// 初始化优化器
std::shared_ptr<g2o::SparseOptimizer> TrajectoryOptimizer::initOptimizer() {
    // 定义优化器
    std::shared_ptr<g2o::SparseOptimizer> optimizer = std::make_shared<g2o::SparseOptimizer>();
    // 定义优化器的线性求解器
    std::unique_ptr<G2OLinearSolver> linear_solver(new G2OLinearSolver());
    linear_solver->setBlockOrdering(true);
    std::unique_ptr<G2OBlockSolver> block_solver(new G2OBlockSolver(std::move(linear_solver)));
    // 设置优化器的优化算法
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
    optimizer->setAlgorithm(solver);
    // 设置优化器为多线程
    optimizer->initMultiThreading(); // required for >Eigen 3.1
    // 返回优化器
    return optimizer;
}

// 初始轨迹生成
void TrajectoryOptimizer::initTrajectory(const std::vector<State> &states) {
    // 清空当前轨迹
    if (!this->trajectory_.isEmpty()) {
        this->trajectory_.clearTrajectory();
    }
    // 根据状态构建轨迹
    for (State state: states) {
        this->trajectory_.addVertex(state);
    }
    // 将轨迹的第一个状态设置为固定状态
    this->trajectory_.setVertexFixed(0, true);
}

// 进行建图
int TrajectoryOptimizer::buildGraph(const std::vector<State> &reference_states, const std::vector<ObstacleAndBoundary> &obstacles_and_boundaries, double dt) {
    // 判断当前图是否为空
    if (!this->optimizer_->edges().empty() || !this->optimizer_->vertices().empty()) {
        std::cout << "[Error] graph is not empty. Call graphClear()!" << std::endl;
        return -1;
    }

    // 为图添加顶点
    this->addVertices();

    // 为图添加边
    // 参考轨迹边
    this->addRefEgdes(reference_states);
    // 速度上限边
    this->addVelocityLimitEdges();
    // 加速度上限边
    this->addAccelerationLimitEdges(dt);
    // 动力学限制边
    this->addKinematicEdges(dt);
    // 加速度值边
    this->addAccelerationValueEdges(dt);
    // 障碍物代价边
    this->addObstacleCostEdges(obstacles_and_boundaries);
    
    return 0;
}

// 进行图优化
int TrajectoryOptimizer::optimizeGraph(int iter_num) {
    // 设置优化器属性
    this->optimizer_->setVerbose(false);
    this->optimizer_->initializeOptimization();
    // 进行优化
    int final_iter = this->optimizer_->optimize(iter_num);
    // 判断优化是否成功
    if(!final_iter) {
        ROS_ERROR("optimizeGraph(): optimization failed! iter=%i", final_iter);
        return -1;
    }
    return 0;
}

// 清除图
void TrajectoryOptimizer::clearGraph() {
    if (this->optimizer_) {
        auto& vertices = this->optimizer_->vertices();
        for(auto& v : vertices) {
            v.second->edges().clear();
        }
        this->optimizer_->vertices().clear();
        this->optimizer_->clear();
    }
}

// 为图添加顶点
void TrajectoryOptimizer::addVertices() {
    // 顶点计数器
    unsigned int id_counter = 0;
    for (int i = 0; i < this->trajectory_.sizeVertices(); i++) {
        this->trajectory_.vertex(i)->setId(i);
        this->optimizer_->addVertex(this->trajectory_.vertex(i));
    }
    return;
}

// 添加参考轨迹边
void TrajectoryOptimizer::addRefEgdes(const std::vector<State> &reference_states) {
    // 初始化权重矩阵
    Eigen::Matrix<double, 5, 5> information = Eigen::Matrix<double, 5, 5>::Zero();
    information(0, 0) = LON_OFFSET_PENALTY_COFF;  // 纵向坐标偏差
    information(1, 1) = LAT_OFFSET_PENALTY_COFF;  // 横向坐标偏差
    information(2, 2) = DIRE_PENALTY_COFF;  // 角度偏差
    information(3, 3) = VEL_PENALTY_COFF;  // 速度偏差
    information(4, 4) = ANGULAR_VEL_PENALTY_COFF;  // 角速度偏差
    // 遍历构建边对象并加入图
    for (int i = 1; i < this->trajectory_.sizeVertices(); i++) {
        double decline_rate = std::pow(DECLINE_RATE, i);
        EdgeReference* edge_reference = new EdgeReference;
        edge_reference->setVertex(0, trajectory_.vertex(i));
        edge_reference->setReference(reference_states[i]);
        edge_reference->setInformation(information * decline_rate);
        this->optimizer_->addEdge(edge_reference);
    }
    return;
}

// 速度上限边
void TrajectoryOptimizer::addVelocityLimitEdges() {
    // 速度限制
    std::array<double, 4> limit;
    limit[0] = this->robot_min_linear_velocity_;
    limit[1] = this->robot_max_linear_velocity_;
    limit[2] = - this->robot_max_angular_velocity_;
    limit[3] = this->robot_max_angular_velocity_;
    // 初始化权重矩阵
    Eigen::Matrix<double, 2, 2> information = Eigen::Matrix<double, 2, 2>::Zero();
    information(0, 0) = V_LIMIT_PENALTY_COFF;  // 速度限制
    information(1, 1) = W_LIMIT_PENALTY_COFF;  // 角速度限制
    // 遍历构建边对象并加入图
    for (int i = 1; i < this->trajectory_.sizeVertices(); i++) {
        EdgeVelocityLimit* edge_velocity_limit = new EdgeVelocityLimit;
        edge_velocity_limit->setVertex(0, trajectory_.vertex(i));
        edge_velocity_limit->setLimit(limit);
        edge_velocity_limit->setInformation(information);
        this->optimizer_->addEdge(edge_velocity_limit);
    }
    return;
}

// 加速度限制
void TrajectoryOptimizer::addAccelerationLimitEdges(double dt) {
    // 加速度限制
    std::array<double, 4> limit;
    limit[0] = - this->robot_max_acceleration_;
    limit[1] = this->robot_max_acceleration_;
    limit[2] = - this->robot_max_angular_velocity_rate_;
    limit[3] = this->robot_max_angular_velocity_rate_;
    // 初始化权重矩阵
    Eigen::Matrix<double, 2, 2> information = Eigen::Matrix<double, 2, 2>::Zero();
    information(0, 0) = ACC_LIMIT_PENALTY_COFF;  // 加速度限制
    information(1, 1) = ANGULAR_ACC_LIMIT_PENALTY_COFF;  // 角加速度限制
    // 遍历构建边对象并加入图
    for (int i = 0; i < this->trajectory_.sizeVertices() - 1; i++) {
        EdgeAccelerationLimit* edge_acceleration_limit = new EdgeAccelerationLimit;
        edge_acceleration_limit->setVertex(0, trajectory_.vertex(i));
        edge_acceleration_limit->setVertex(1, trajectory_.vertex(i + 1));
        edge_acceleration_limit->setLimit(limit);
        edge_acceleration_limit->setTimeDiff(dt);
        edge_acceleration_limit->setInformation(information);
        this->optimizer_->addEdge(edge_acceleration_limit);
    }
    return;
}

// 动力学限制边
void TrajectoryOptimizer::addKinematicEdges(double dt) {
    // 初始化权重矩阵
    Eigen::Matrix<double, 3, 3> information = Eigen::Matrix<double, 3, 3>::Zero();
    information(0, 0) = KINEMATIC_LIMIT_PENALTY_COFF;
    information(1, 1) = KINEMATIC_LIMIT_PENALTY_COFF;
    information(2, 2) = KINEMATIC_LIMIT_PENALTY_COFF;
    // 遍历构建边对象并加入图
    for (int i = 0; i < this->trajectory_.sizeVertices() - 1; i++) {
        EdgeKinematic* edge_kinematic = new EdgeKinematic;
        edge_kinematic->setVertex(0, trajectory_.vertex(i));
        edge_kinematic->setVertex(1, trajectory_.vertex(i + 1));
        edge_kinematic->setTimeDiff(dt);
        edge_kinematic->setInformation(information);
        this->optimizer_->addEdge(edge_kinematic);
    }
    return;
}

// 障碍物限制边
void TrajectoryOptimizer::addObstacleLimitEdges(const std::vector<std::vector<Line2d>> &constraint_regions) {
    // 初始化权重矩阵
    Eigen::Matrix<double, 1, 1> information;
    information.fill(OBS_LIMIT_PENALTY_COFF);
    // 遍历构建边对象并加入图
    for (int i = 1; i < this->trajectory_.sizeVertices(); i++) {
        EdgeObstacleLimit* edge_obstacle = new EdgeObstacleLimit;
        double decline_rate = std::pow(DECLINE_RATE, i);
        edge_obstacle->setVertex(0, trajectory_.vertex(i));
        edge_obstacle->setConstraintRegion(constraint_regions[i]);
        edge_obstacle->setInformation(information * decline_rate);
        this->optimizer_->addEdge(edge_obstacle);
    }
    return;
}

// 添加速度值边
void TrajectoryOptimizer::addAccelerationValueEdges(double dt) {
    // 初始化权重矩阵
    Eigen::Matrix<double, 2, 2> information = Eigen::Matrix<double, 2, 2>::Zero();
    information(0, 0) = ACC_VALUE_PENALTY;  // 加速度值
    information(1, 1) = ANGULAR_ACC_VALUE_PENALTY;  // 角加速度值
    // 遍历构建边对象并加入图
    for (int i = 0; i < this->trajectory_.sizeVertices() - 1; i++) {
        EdgeAccelerationValue* edge_acceleration_value = new EdgeAccelerationValue;
        edge_acceleration_value->setVertex(0, trajectory_.vertex(i));
        edge_acceleration_value->setVertex(1, trajectory_.vertex(i + 1));
        edge_acceleration_value->setTimeDiff(dt);
        edge_acceleration_value->setInformation(information);
        this->optimizer_->addEdge(edge_acceleration_value);
    }
    return;
}

 // 添加障碍物代价边
void TrajectoryOptimizer::addObstacleCostEdges(const std::vector<ObstacleAndBoundary> &obstacles_and_boundaries) {
    // 初始化权重矩阵
    Eigen::Matrix<double, 1, 1> information;
    information.fill(EVALUATE_OBS_PENALTY_COFF);
    // 遍历构建边对象并加入图
    for (int i = 1; i < this->trajectory_.sizeVertices(); i++) {
        EdgeObstacleCost* edge_obstacle = new EdgeObstacleCost;
        double decline_rate = std::pow(DECLINE_RATE, i);
        edge_obstacle->setVertex(0, trajectory_.vertex(i));
        edge_obstacle->setDynamicObstacles(obstacles_and_boundaries[i]);
        edge_obstacle->setInformation(information * decline_rate);
        this->optimizer_->addEdge(edge_obstacle);
    }
    return;
}

// 解析轨迹
void TrajectoryOptimizer::parseTrajectory(std::vector<State> &states) {
    ROS_ASSERT_MSG(states.size() == this->trajectory_.sizeVertices(), "opt state size is not equal to trajectory");
    for (int i = 0; i < this->trajectory_.sizeVertices(); i++) {
        states[i].x_ = this->trajectory_.vertex(i)->x();
        states[i].y_ = this->trajectory_.vertex(i)->y();
        states[i].theta_ = this->trajectory_.vertex(i)->theta();
        states[i].v_ = this->trajectory_.vertex(i)->v();
        states[i].w_ = this->trajectory_.vertex(i)->w();
    }
}

};
