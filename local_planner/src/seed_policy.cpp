#include "policy/seed_policy.hpp"

int SeedPolicy::forward(Robot &robot, const Pose &target_pose, const std::vector<PathPose> &navigation_path, const GridMap &global_map, const std::map<int, Tools::FixedQueue<ObstacleInfo, OBSTACLE_INFO_LEN>> &obstacles_info, Action &planned_action) {
    // 构建当前状态
    State current_state = {0.0, 0.0, 0.0, robot.getAction().v_, robot.getAction().w_};
    // 构建终点状态
    Position target_base_footprint = Tools::calcNewCoordinationPosition(robot.getPose(), {target_pose.x_, target_pose.y_});
    State target_state = {target_base_footprint.x_, target_base_footprint.y_, 0.0, 0.0, 0.0};
    // 将导航路径转到局部坐标系下
    std::vector<PathPose> local_navigation_path = this->transformPathToNewCoordinate(robot.getPose(), navigation_path);
    // 生成参考状态序列
    std::vector<State> ref_states = this->generateReferenceStates(local_navigation_path);
    // 可视化全局路径和参考状态序列
    if (VISUALIZATION) {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker navigation_marker;
        navigation_marker.header.frame_id = "base_footprint";
        navigation_marker.header.stamp = ros::Time::now();
        navigation_marker.id = 0;
        navigation_marker.type = visualization_msgs::Marker::LINE_STRIP;
        navigation_marker.action = visualization_msgs::Marker::ADD;
        navigation_marker.scale.x = 0.02;
        navigation_marker.color.r = 0.0;
        navigation_marker.color.g = 1.0;
        navigation_marker.color.b = 0.0;
        navigation_marker.color.a = 1.0;
        for (auto nav_point: local_navigation_path) {
            geometry_msgs::Point point;
            point.x = nav_point.x_;
            point.y = nav_point.y_;
            navigation_marker.points.push_back(point);
        }
        marker_array.markers.push_back(navigation_marker);

        visualization_msgs::Marker reference_states_marker;
        reference_states_marker.header.frame_id = "base_footprint";
        reference_states_marker.header.stamp = ros::Time::now();
        reference_states_marker.id = 1;
        reference_states_marker.type = visualization_msgs::Marker::LINE_STRIP;
        reference_states_marker.action = visualization_msgs::Marker::ADD;
        reference_states_marker.scale.x = 0.04;
        reference_states_marker.color.r = 0.0;
        reference_states_marker.color.g = 0.0;
        reference_states_marker.color.b = 1.0;
        reference_states_marker.color.a = 1.0;
        for (auto state: ref_states) {
            geometry_msgs::Point point;
            point.x = state.x_;
            point.y = state.y_;
            reference_states_marker.points.push_back(point);
        }
        marker_array.markers.push_back(reference_states_marker);
        this->debug_nav_pub_.publish(marker_array);
    }

    // 得到局部地图信息
    std::array<GridMap, PREDICT_TIME_LEN> predict_gridmaps = this->predictFutureEnvironment(robot, global_map, obstacles_info);
    std::array<KDTree, PREDICT_TIME_LEN> predict_kdtrees;
    for (int i = 0; i < predict_gridmaps.size(); i++) {
        predict_kdtrees[i] = KDTree(predict_gridmaps[i]);
    }
    if (VISUALIZATION) {
        this->debug_map_pub_.publish(predict_gridmaps.front().toRosMessage("base_footprint"));
    }

    // 得到障碍物信息
    std::vector<ObstacleAndBoundary> obstacles_and_boundaries;
    for (int i = 0; i < PREDICT_TIME_LEN; i++) {
        // 转换到局部坐标系下
        std::vector<ObstacleInfo> base_obstacle_info;
        for (auto obstacle_info: obstacles_info) {
            Position local_position = Tools::calcNewCoordinationPosition(robot.getPose(), {obstacle_info.second.back().x_ + static_cast<double>(i) * this->time_step_ * obstacle_info.second.back().vx_, obstacle_info.second.back().y_ + static_cast<double>(i) * this->time_step_ * obstacle_info.second.back().vy_});
            // std::cout << "vel:" << obstacle_info.second.back().vx_ << ", " << obstacle_info.second.back().vy_ << std::endl;
            double vel = sqrt(obstacle_info.second.back().vx_ * obstacle_info.second.back().vx_ + obstacle_info.second.back().vy_ * obstacle_info.second.back().vy_);
            double orientation = atan2(obstacle_info.second.back().vy_, obstacle_info.second.back().vx_) - robot.getPose().theta_;
            Vector2d local_vel = {vel * cos(orientation), vel * sin(orientation)};
            base_obstacle_info.push_back({local_position.x_, local_position.y_, local_vel.x_, local_vel.y_, obstacle_info.second.back().radius_ + this->robot_radius_});
            // std::cout << "local vel:" << local_vel.x_ << ", " << local_vel.y_ << std::endl;
        }
        ObstacleAndBoundary obstacle_and_boundary(base_obstacle_info, predict_kdtrees[i], predict_gridmaps[i].getResolution());
        obstacles_and_boundaries.push_back(obstacle_and_boundary);
    }
    if (VISUALIZATION) {
        // 进行可视化验证
        for (int test_i = 0; test_i < 1; test_i++) {
            std::vector<double> datas;
            for (int j = 0; j < 140; j++) {
                for (int i = 0; i < 140; i++) {
                    Position position = Tools::calcOldCoordinationPosition({-3.5, -3.5, 0.0}, {static_cast<double>(i) * 0.05, static_cast<double>(j)*0.05});
                    double cost = obstacles_and_boundaries[test_i].calcCost(position.x_, position.y_);
                    datas.push_back(cost);
                }
            }
            // 保存文件
            // 导出csv
            std::string data_file_path = ros::package::getPath("local_planner") + "/data/obstacle.csv";
            std::ofstream data_file(data_file_path, std::ios::out|std::ios::trunc);
            for (auto data: datas) {
                data_file << data << ",";
            }
            data_file.close();
            double max_value = *(std::max_element(datas.begin(), datas.end()));
            // std::cout << " max_value: " << max_value << std::endl;
            // 可视化代价地图
            nav_msgs::OccupancyGrid show_cost_map;
            geometry_msgs::Pose pose;
            pose.position.x = -3.5;
            pose.position.y = -3.5;
            pose.orientation.z = sin(0.0);
            pose.orientation.w = cos(0.0);
            std::vector<int8_t> out_data;
            for (auto data: datas) {
                out_data.push_back(static_cast<int8_t>(data / max_value * 100.0));
            }
            show_cost_map.data = out_data;
            show_cost_map.info.resolution = 0.05;
            show_cost_map.info.width = 140;
            show_cost_map.info.height = 140;
            show_cost_map.info.origin = pose;
            show_cost_map.info.map_load_time = ros::Time::now();
            show_cost_map.header.frame_id = "base_footprint";
            show_cost_map.header.stamp = ros::Time::now();
            this->debug_cost_pub_.publish(show_cost_map);
        }
    }

    // 判断机器人是否已经发生碰撞
    std::pair<int, int> start_grid_point = predict_gridmaps.front().getGridMapCoordinate(current_state.x_, current_state.y_);
    if (predict_gridmaps.front().isOccupied(predict_gridmaps.front().getIndex(start_grid_point.first, start_grid_point.second))) {
        // 初始位置发生碰撞
        std::cerr << "robot current gird is occupied, planning failed" << std::endl;
        return -1;
    }
    // 判断机器人是否到达终点
    double dist_to_goal = sqrt(target_state.x_ * target_state.x_ + target_state.y_ * target_state.y_);
    if (Tools::isSmall(dist_to_goal, this->robot_radius_)) {
        std::cout << "goal reached" << std::endl;
        return 0;
    }

    // std::cout << "basic local environment constructed" << std::endl;
    // 生成种子点*
    clock_t start_seed_search_time = clock();
    std::vector<std::vector<std::shared_ptr<Node>>> seed_tree;
    seed_tree.reserve(PREDICT_TIME_LEN);
    // 读取采样方法
    std::string sample_method = this->pt_.get<std::string>("robot.sampler");
    std::cout << "sample method: " << sample_method << std::endl;
    this->generateSeeds(current_state, ref_states, predict_gridmaps, obstacles_and_boundaries, sample_method, seed_tree);
    // 判断种子点是否生成成功
    if (seed_tree.size() <= 1) {
        std::cerr << "seeds searching failed" << std::endl;
        return -1;
    }
    clock_t end_seed_search_time = clock();
    //ROS_INFO_STREAM("seeds searching finished, time consuming is " << static_cast<double>(end_seed_search_time - start_seed_search_time) * 1000.0 / CLOCKS_PER_SEC << " ms");
    // 导出种子点csv
    std::string data_file_path = ros::package::getPath("local_planner") + "/data/seed.csv";
    std::ofstream data_file(data_file_path, std::ios::out|std::ios::trunc);
    for (auto seed_node: seed_tree.back()) {
        data_file << seed_node->state_.x_ << "," << seed_node->state_.y_ << "," << seed_node->state_.theta_ << "\n";
    }
    data_file.close();
    // 可视化种子点及其连接关系
    if (VISUALIZATION) {
        // 删除之前的可视化
        visualization_msgs::MarkerArray delete_marker_array;
        visualization_msgs::Marker delete_marker;
        delete_marker.header.frame_id = "base_footprint";
        delete_marker.header.stamp = ros::Time::now();
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        delete_marker.id = 0;
        delete_marker_array.markers.push_back(delete_marker);
        this->seed_tree_vis_pub_.publish(delete_marker_array);
        // 进行可视化
        visualization_msgs::MarkerArray marker_array;
        for (int i = 0; i < seed_tree.back().size(); i++) {
            std::shared_ptr<Node> seed_node_ptr = seed_tree.back()[i];
            visualization_msgs::Marker node_marker;
            node_marker.header.frame_id = "base_footprint";
            node_marker.header.stamp = ros::Time::now();
            node_marker.id = i;
            node_marker.type = visualization_msgs::Marker::LINE_STRIP;
            node_marker.action = visualization_msgs::Marker::ADD;
            node_marker.scale.x = 0.01;
            node_marker.color.r = 0.0;
            node_marker.color.g = 0.0;
            node_marker.color.b = 1.0;
            node_marker.color.a = 0.6;
            while (seed_node_ptr != nullptr) {
                geometry_msgs::Point point;
                point.x = seed_node_ptr->state_.x_;
                point.y = seed_node_ptr->state_.y_;
                node_marker.points.push_back(point);
                seed_node_ptr = seed_node_ptr->parent_;
            }
            marker_array.markers.push_back(node_marker);
        }
        this->seed_tree_vis_pub_.publish(marker_array);
    }

    // 获取最优种子点集合
    std::vector<std::vector<State>> best_seed_candidates = this->selectBestSeeds(seed_tree, CANDIDATE_NUM);
    assert(best_seed_candidates.size() > 0);
    // 对最优种子序列进行可视化
    visualization_msgs::MarkerArray best_seed_marker_array;
    for (int i = 0; i < best_seed_candidates.size(); i++){
        // 进行可视化
        visualization_msgs::Marker node_marker;
        node_marker.header.frame_id = "base_footprint";
        node_marker.header.stamp = ros::Time::now();
        node_marker.id = i;
        node_marker.type = visualization_msgs::Marker::LINE_STRIP;
        node_marker.action = visualization_msgs::Marker::ADD;
        node_marker.scale.x = 0.02;
        node_marker.color.r = 1.0;
        node_marker.color.g = 0.0;
        node_marker.color.b = 0.0;
        node_marker.color.a = 1.0;
        for (auto best_seed: best_seed_candidates[i]) {
            geometry_msgs::Point point;
            point.x = best_seed.x_;
            point.y = best_seed.y_;
            node_marker.points.push_back(point);
        }
        best_seed_marker_array.markers.push_back(node_marker);
    }
    this->best_seed_sequence_vis_pub_.publish(best_seed_marker_array);
    // std::cout << "best seed sequence selected" << std::endl;
    double seed_sequence_cost = this->calcStateSequenceCost(best_seed_candidates.front(), obstacles_and_boundaries, ref_states);
    // std::cout << "searched best seed sequence cost: " << seed_sequence_cost << std::endl;

    // 进行优化
    std::vector<State> opt_states;
    // 读取优化方法
    std::string optimization_method = this->pt_.get<std::string>("robot.optimizer");
    std::cout << "optimization method: " << optimization_method << std::endl;
    // 进行优化
    if (optimization_method == "eb_mpc") {
        // 使用-EB-MPC方法进行优化
        // auto opt_start_time = ros::WallTime::now();
        int opt_result = -1;
        double best_state_cost = std::numeric_limits<double>::max();
        for (auto best_seed_candidate: best_seed_candidates) {
            std::vector<State> tmp_opt_states = best_seed_candidate;
            // 得到加速度序列
            std::vector<Acceleration> accelerations = this->accelerationEstimation(tmp_opt_states, this->time_step_);
            // 进行状态序列和加速度序列的验证
            this->verifySequence(current_state, tmp_opt_states, accelerations);
            std::vector<State> cut_ref_states(ref_states.begin(), ref_states.begin() + tmp_opt_states.size());
            int opt_times = 0;
            while (opt_times < MAX_OPT_TIMES) {
                // 进行优化
                int opt_result = this->eb_mpc_trajectory_optimizer_.optimizing(cut_ref_states, obstacles_and_boundaries, this->time_step_, tmp_opt_states);
                // 判断是否优化成功
                if (opt_result < 0) {
                    // 优化失败
                    ROS_ERROR("trajectory optimizing failed");
                    exit(0);
                }
                opt_times++;
            }
            // 判断是否优于当前
            double current_opt_cost = this->calcStateSequenceCost(tmp_opt_states, obstacles_and_boundaries, cut_ref_states);
            // std::cout << "current_opt_cost: " << current_opt_cost << std::endl;
            if (Tools::isSmall(current_opt_cost, best_state_cost)) {
                best_state_cost = current_opt_cost;
                opt_states = tmp_opt_states;
            }
        }
        if (Tools::isLarge(best_state_cost, seed_sequence_cost)) {
            opt_states = best_seed_candidates.front();
        }
        // auto opt_end_time = ros::WallTime::now();
        // std::cout << " eb mpc optimization time consuming: " << (opt_end_time - opt_start_time).toNSec() / 1e6 << std::endl;
    }  else {
        opt_states = best_seed_candidates.front();
    }
    // 可视化最终轨迹
    {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker opt_states_marker;
        opt_states_marker.header.frame_id = "base_footprint";
        opt_states_marker.header.stamp = ros::Time::now();
        opt_states_marker.id = 1;
        opt_states_marker.type = visualization_msgs::Marker::LINE_STRIP;
        opt_states_marker.action = visualization_msgs::Marker::ADD;
        opt_states_marker.scale.x = 0.04;
        opt_states_marker.color.r = 0.0;
        opt_states_marker.color.g = 1.0;
        opt_states_marker.color.b = 0.0;
        opt_states_marker.color.a = 1.0;
        for (auto state: opt_states) {
            geometry_msgs::Point point;
            point.x = state.x_;
            point.y = state.y_;
            opt_states_marker.points.push_back(point);
        }
        marker_array.markers.push_back(opt_states_marker);
        this->debug_optimize_pub_.publish(marker_array);
    }

    double opt_cost = this->calcStateSequenceCost(opt_states, obstacles_and_boundaries, ref_states);
    // std::cout << "opt cost: " << opt_cost << std::endl;
    // 将轨迹的首段作为规划的运动进行输出
    planned_action.v_ = opt_states[1].v_;
    planned_action.w_ = opt_states[1].w_;
    return 0;
}

std::vector<CircleRegion> SeedPolicy::searchCircleRegions(const std::vector<State> &states, const std::array<GridMap, PREDICT_TIME_LEN> &grid_maps, const std::array<KDTree, PREDICT_TIME_LEN> &kdtrees) {
    // 初始化
    std::vector<CircleRegion> regions;
    regions.resize(states.size());
    // 遍历每一个状态
    for (int frame_index = 0; frame_index < states.size(); frame_index++) {
        // 计算最大半径
        double max_radius = 0.5 * std::min(grid_maps[frame_index].getWidth(), grid_maps[frame_index].getHeight()) * grid_maps[frame_index].getResolution();
        // 得到状态的坐标点
        Position point;
        point.x_ = states[frame_index].x_;
        point.y_ = states[frame_index].y_;
        // 找到离给出状态最近的障碍物点
        std::vector<std::pair<float, float>> nearest_points;
        std::vector<float> sq_distances;
        int nearest_search_result = kdtrees[frame_index].findKNeighbor(point.x_, point.y_, &nearest_points, &sq_distances, 1);
        // 判断是否找到
        if (nearest_search_result < 0) {
            // 不存在障碍物点
            regions[frame_index].center_ = point;
            regions[frame_index].radius_ = max_radius;
        } else {
            // 存在障碍物点
            Position boundary_point;
            boundary_point.x_ = nearest_points[0].first;
            boundary_point.y_ = nearest_points[0].second;
            double origin_dist = sqrt(sq_distances[0]);
            Position origin_point = point;
            double current_radius = sqrt(sq_distances[0]);
            Position current_center = point;
            double extend_theta = atan2(point.y_ - boundary_point.y_, point.x_ - boundary_point.x_);
            double step = 0.1;
            while(true) {
                // 进行反向拓展
                point.x_ = current_center.x_ + step * cos(extend_theta);
                point.y_ = current_center.y_ + step * sin(extend_theta);
                nearest_search_result = kdtrees[frame_index].findKNeighbor(point.x_, point.y_, &nearest_points, &sq_distances, 1);
                assert(nearest_search_result >= 0);
                // 判断离障碍物距离是否缩小
                if (Tools::isSmall(sqrt(sq_distances[0]), current_radius)) {
                    // 缩小
                    break;
                }
                // 判断离障碍物距离是否大于最大值
                if (Tools::isLarge(sqrt(sq_distances[0]), max_radius)) {
                    // 大于最大值
                    break;
                }
                // 判断当前状态离新边界的距离是否变小
                double dist = sqrt(sq_distances[0]) - sqrt((point.x_ - origin_point.x_) * (point.x_ - origin_point.x_) + (point.y_ - origin_point.y_) * (point.y_ - origin_point.y_));
                if (!Tools::isLarge(dist, 0.0) || Tools::isSmall(dist, origin_dist - 0.5 * step)) {
                    break;
                }
                // 更新区域
                current_center = point;
                current_radius = sqrt(sq_distances[0]);
                // 进行可视化
            }
            // 更新区域
            regions[frame_index].center_ = current_center;
            regions[frame_index].radius_ = current_radius;
        }
    }
    return regions;
}

// 进行序列验证
void SeedPolicy::verifySequence(const State &current_state, const std::vector<State> &best_seed_sequence, const std::vector<Acceleration> &accelerations) {
    // 判断动力学偏差是否都小于阈值
    for (int i = 0; i < best_seed_sequence.size() - 1; i++) {
        double x_deviation = std::abs(best_seed_sequence[i + 1].x_ - (best_seed_sequence[i].x_ + this->time_step_ * best_seed_sequence[i + 1].v_ * cos(best_seed_sequence[i + 1].theta_)));
        double y_deviation = std::abs(best_seed_sequence[i + 1].y_ - (best_seed_sequence[i].y_ + this->time_step_ * best_seed_sequence[i + 1].v_ * sin(best_seed_sequence[i + 1].theta_)));
        double theta_deviation = Tools::pi2Pi(std::abs(best_seed_sequence[i + 1].theta_ - (best_seed_sequence[i].theta_ + this->time_step_ * best_seed_sequence[i + 1].w_)));
        double v_deviation = std::abs(best_seed_sequence[i + 1].v_ - (best_seed_sequence[i].v_ + this->time_step_ * accelerations[i].la_));
        double w_deviation = std::abs(best_seed_sequence[i + 1].w_ - (best_seed_sequence[i].w_ + this->time_step_ * accelerations[i].aa_));
        if (!Tools::isZero(x_deviation) || !Tools::isZero(y_deviation) || !Tools::isZero(theta_deviation) || !Tools::isZero(v_deviation) || !Tools::isZero(w_deviation)) {
            std::cout << "[Error] kinematic not meet: " << x_deviation << ", " << y_deviation << ", " << theta_deviation << ", " << v_deviation << ", " << w_deviation << std::endl;
            exit(0);
        }
    }
    // 判断是否满足初始约束条件
    State first_seed_state = best_seed_sequence.front();
    if (!(Tools::isZero(first_seed_state.x_ - current_state.x_) && Tools::isZero(first_seed_state.y_ - current_state.y_) && Tools::isZero(first_seed_state.theta_ - current_state.theta_) && Tools::isZero(first_seed_state.v_ - current_state.v_) && Tools::isZero(first_seed_state.v_ - current_state.v_))) {
        std::cout << "[Error] init state constraints not meet" << std::endl;
        exit(0);
    }
    // 判断是否满足边界约束条件
    for (int i = 0; i < best_seed_sequence.size(); i++) {
        if (Tools::isLarge(best_seed_sequence[i].v_, this->max_v_) || Tools::isSmall(best_seed_sequence[i].v_, this->min_v_)) {
            std::cout << "[Error] linear v boundary limits not meet" << std::endl;
            exit(0);
        }
        if (Tools::isLarge(best_seed_sequence[i].w_, this->max_w_) || Tools::isSmall(best_seed_sequence[i].w_, -this->max_w_)) {
            std::cout << "[Error] angular v boundary limits not meet" << std::endl;
            exit(0);
        }
        if (i < best_seed_sequence.size() - 1) {
            if (Tools::isLarge(accelerations[i].la_, this->max_acc_) || Tools::isSmall(accelerations[i].la_, -this->max_acc_)) {
                std::cout << "[Error] acceleration boundary limits not meet" << std::endl;
                exit(0);
            }
            if (Tools::isLarge(accelerations[i].aa_, this->max_angular_acc_) || Tools::isSmall(accelerations[i].aa_, -this->max_angular_acc_)) {
                std::cout << "[Error] angular rate boundary limits not meet" << std::endl;
                exit(0);
            }
        }
    }
}

// 根据状态序列估计运动序列
std::vector<Acceleration> SeedPolicy::accelerationEstimation(const std::vector<State> &states, double dt) {
    assert(states.size() > 1);
    std::vector<Acceleration> actions;
    actions.resize(states.size() - 1);
    for (int i = 0; i < states.size() - 1; i++) {
        Acceleration action;
        action.la_ = (states[i + 1].v_ - states[i].v_) / dt;
        action.aa_ = (states[i + 1].w_ - states[i].w_) / dt;
        actions[i] = action;
    }
    return actions;
}

// 选择最优种子点
std::vector<std::vector<State>> SeedPolicy::selectBestSeeds(const std::vector<std::vector<std::shared_ptr<Node>>> &seed_tree, size_t max_num) {
    std::vector<std::shared_ptr<Node>> seed_tree_leafs = seed_tree.back();
    std::sort(seed_tree_leafs.begin(), seed_tree_leafs.end(), compareNode);
    // std::cout << "cost ";
    // for (auto seed_tree_leaf: seed_tree_leafs) {
    //     std::cout << seed_tree_leaf->cost_ << ", ";
    // }
    // std::cout << std::endl;
    std::vector<std::shared_ptr<Node>> best_seed_nodes(seed_tree_leafs.begin(), seed_tree_leafs.begin() + std::min(seed_tree_leafs.size(), max_num));

    std::vector<std::vector<State>> best_seed_candidates;
    for (auto best_seed_node: best_seed_nodes) {
        std::vector<State> best_seed_candidate;
        while (best_seed_node != nullptr) {
            best_seed_candidate.push_back(best_seed_node->state_);
            best_seed_node = best_seed_node->parent_;
        }
        std::reverse(best_seed_candidate.begin(), best_seed_candidate.end());
        best_seed_candidates.push_back(best_seed_candidate);
    }
    return best_seed_candidates;
}

// 生成种子点
void SeedPolicy::generateSeeds(const State &current_state, const std::vector<State> &ref_states, const std::array<GridMap, PREDICT_TIME_LEN> &predict_gridmaps, const std::vector<ObstacleAndBoundary> & obstacles_and_boundaries, const std::string &sample_method, std::vector<std::vector<std::shared_ptr<Node>>> &seed_tree) {
    // 对时间进行遍历
    for (int future_step = 0; future_step < PREDICT_TIME_LEN; future_step++) {
        std::vector<std::shared_ptr<Node>> seed_layer;
        if (future_step == 0) {
            // 直接根据当前状态进行节点构建
            std::shared_ptr<Node> current_seed_node_ptr = std::make_shared<Node>();
            current_seed_node_ptr->parent_ = nullptr;
            current_seed_node_ptr->state_ = current_state;
            current_seed_node_ptr->cost_ = 0.0;
            seed_layer.push_back(current_seed_node_ptr);
        } else {
            // 得到上一层节点
            std::vector<std::shared_ptr<Node>> last_seed_layer = seed_tree[future_step - 1];
            std::vector<std::shared_ptr<Node>> full_expand_states_seed_layer;
            // 遍历每一个节点
            for (auto last_seed_node_ptr: last_seed_layer) {
                // 进行速度空间状态拓展
                std::vector<State> expand_states = this->stateExpanding(last_seed_node_ptr->state_, V_SAMPLE_NUM, W_SAMPLE_NUM);
                // 对于每一个拓展的状态，计算对应节点
                for (State expand_state: expand_states) {
                    // 判断状态是否发生碰撞
                    std::pair<int, int> grid_point = predict_gridmaps[future_step].getGridMapCoordinate(expand_state.x_, expand_state.y_);
                    if (predict_gridmaps[future_step].isOccupied(predict_gridmaps[future_step].getIndex(grid_point.first, grid_point.second))) {
                        // 状态发生碰撞
                        continue;
                    }
                    // 构建节点
                    std::shared_ptr<Node> seed_node_ptr = std::make_shared<Node>();
                    seed_node_ptr->parent_ = last_seed_node_ptr;
                    seed_node_ptr->state_ = expand_state;
                    seed_node_ptr->cost_ = 0.0;
                    // 加入列表
                    full_expand_states_seed_layer.push_back(seed_node_ptr);
                }
            }
            // std::cout << "generate expand states" << std::endl;
            // 判断此层的节点数量
            if (full_expand_states_seed_layer.size() > LAYER_MAX_SEED_NUM) {
                // 超过最大数量，进行数量缩减s
                if (sample_method == "rand") {
                    // 随机采样
                    std::sample(full_expand_states_seed_layer.begin(), full_expand_states_seed_layer.end(), std::back_inserter(seed_layer), LAYER_MAX_SEED_NUM, this->rand_gen_);
                } else if (sample_method == "voxel") {
                    // 栅格化采样
                    seed_layer = this->voxelGridSampling(full_expand_states_seed_layer);
                } else {
                    std::cerr << "error sample method" << std::endl;
                    exit(0);
                }
            } else {
                // 未超过最大数量
                seed_layer = full_expand_states_seed_layer;
            }
            // std::cout << "sampling" << std::endl;
            // 计算种子代价
            for (auto seed_node_ptr: seed_layer) {
                double cost = this->calcCost(seed_node_ptr->state_, obstacles_and_boundaries[future_step], ref_states[future_step]);
                seed_node_ptr->cost_ = std::pow(DECLINE_RATE, future_step) * cost + seed_node_ptr->parent_->cost_;
                // seed_node_ptr->cost_ = cost;
            }
            // std::cout << "cost calculate" << std::endl;
        }
        // 判断当前帧节点列表是否为空
        if (seed_layer.size() == 0) {
            std::cout << "seed layer " << future_step << " is empty" << std::endl;
            return;
        }
        // 保存当前层的节点
        seed_tree.push_back(seed_layer);
    }
}

// 计算状态序列代价
double SeedPolicy::calcStateSequenceCost(const std::vector<State> &states, const std::vector<ObstacleAndBoundary> &obstacles_and_boundaries, const std::vector<State> &ref_states) {
    double cost = 0.0;
    for (int i = 0; i < states.size(); i++) {
        // std::cout << "state " << states[i].x_ << " " << states[i].y_ << " " << states[i].theta_ << " " << states[i].v_ << " " << states[i].w_ << std::endl; 
        cost += std::pow(DECLINE_RATE, i) * this->calcCost(states[i], obstacles_and_boundaries[i], ref_states[i]);
    }
    return cost;
}

// 计算状态代价
double SeedPolicy::calcCost(const State &state, const ObstacleAndBoundary &obsatcle_and_boundary, const State &ref_state) {
    // 计算安全性代价，离障碍物距离远损失越小
    double safety_cost = obsatcle_and_boundary.calcCost(state.x_, state.y_);
    
    // 第二部分是区域离参考状态的横纵向距离越小越好（损失越小）
    double longitudinal_gap = (state.x_ - ref_state.x_) * cos(ref_state.theta_) + (state.y_ - ref_state.y_) * sin(ref_state.theta_);
    double lateral_gap = -(state.x_ - ref_state.x_) * sin(ref_state.theta_) + (state.y_ - ref_state.y_) * cos(ref_state.theta_);
    double longitudinal_gap_cost = longitudinal_gap * longitudinal_gap;
    double lateral_gap_cost = lateral_gap * lateral_gap;
    // 第三部分角度偏差越小越好
    double dire_gap_cost = std::pow(1 - cos(state.theta_ - ref_state.theta_), 2.0);
    // // 第四部分速度偏差越小越好
    // double velocity_gap_cost = std::pow(state.v_ - ref_state.v_, 2);
    // // 第五部分角速度偏差越小越好
    // double angular_velocity_gap_cost = std::pow(state.w_ - ref_state.w_, 2);
    // 计算总损失
    double state_cost = EVALUATE_OBS_PENALTY_COFF * safety_cost + LON_OFFSET_PENALTY_COFF * longitudinal_gap_cost + LAT_OFFSET_PENALTY_COFF * lateral_gap_cost + DIRE_PENALTY_COFF * dire_gap_cost;
    // VEL_PENALTY_COFF * velocity_gap_cost + ANGULAR_VEL_PENALTY_COFF * angular_velocity_gap_cost;
    // std::cout << "evaluate cost: " << LON_OFFSET_PENALTY_COFF * longitudinal_gap_cost << ", " << LAT_OFFSET_PENALTY_COFF * lateral_gap_cost << ", " << DIRE_PENALTY_COFF * dire_gap_cost << "," << EVALUATE_OBS_PENALTY_COFF * safety_cost << std::endl;
    
    return state_cost;
}

// 栅格化体素采样
std::vector<std::shared_ptr<SeedPolicy::Node>> SeedPolicy::voxelGridSampling(const std::vector<std::shared_ptr<Node>> &raw_seed_nodes) {
    std::vector<std::shared_ptr<SeedPolicy::Node>> sampled_seed_nodes;
    // 首先计算栅格边界
    double min_x = std::numeric_limits<double>::max();
    double max_x = - std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_y = - std::numeric_limits<double>::max();
    double min_theta = M_PI;
    double max_theta = - M_PI;
    for (auto raw_seed_node: raw_seed_nodes) {
        if (Tools::isSmall(raw_seed_node->state_.x_, min_x)) {
            min_x = raw_seed_node->state_.x_;
        }
        if (Tools::isLarge(raw_seed_node->state_.x_, max_x)) {
            max_x = raw_seed_node->state_.x_;
        }
        if (Tools::isSmall(raw_seed_node->state_.y_, min_y)) {
            min_y = raw_seed_node->state_.y_;
        }
        if (Tools::isLarge(raw_seed_node->state_.y_, max_y)) {
            max_y = raw_seed_node->state_.y_;
        }
        if (Tools::isSmall(raw_seed_node->state_.theta_, min_theta)) {
            min_theta = raw_seed_node->state_.theta_;
        }
        if (Tools::isLarge(raw_seed_node->state_.theta_, max_theta)) {
            max_theta = raw_seed_node->state_.theta_;
        }
    }
    // std::cout << "boundary: " << std::setprecision(10) << min_x << ", " << max_x << ", " << min_y << "," << max_y << ", " << min_theta << ", " << max_theta << std::endl;
    // 计算分辨率
    double x_resolution = (max_x - min_x) / static_cast<double>(X_SAMPLE_NUM);
    double y_resolution = (max_y - min_y) / static_cast<double>(Y_SAMPLE_NUM);
    double theta_resolution = (max_theta - min_theta) / static_cast<double>(THETA_SAMPLE_NUM);
    // std::cout << "resolution" << std::setprecision(10) << x_resolution << ", " << y_resolution << ", " << theta_resolution << std::endl;
    // 进行采样存储
    typedef boost::multi_array<std::vector<std::shared_ptr<Node>>, 3> array_type;
    array_type::extent_gen extents;
    array_type voxel_nodes_store(extents[X_SAMPLE_NUM + 1][Y_SAMPLE_NUM + 1][THETA_SAMPLE_NUM + 1]);
    for (auto raw_seed_node: raw_seed_nodes) {
        int x_index = static_cast<int>((raw_seed_node->state_.x_ - min_x) / (x_resolution + EPS));
        int y_index = static_cast<int>((raw_seed_node->state_.y_ - min_y) / (y_resolution + EPS));
        int theta_index = static_cast<int>((raw_seed_node->state_.theta_ - min_theta) / (theta_resolution + EPS));
        // if (x_index < 0 || x_index > X_SAMPLE_NUM) {
        //     std::cout << x_index << std::endl;
        //     std::cout << std::setprecision(10) << raw_seed_node->state_.x_ << "," << raw_seed_node->state_.x_ - min_x << std::endl;
        // }
        // if (y_index < 0 || y_index > Y_SAMPLE_NUM) {
        //     std::cout << y_index << std::endl;
        //     std::cout << std::setprecision(10) << raw_seed_node->state_.y_ << ", " << raw_seed_node->state_.y_ - min_y << std::endl;
        // }
        // if (theta_index < 0 || theta_index > THETA_SAMPLE_NUM) {
        //     std::cout << theta_index << std::endl;
        // }
        voxel_nodes_store[x_index][y_index][theta_index].push_back(raw_seed_node);
    }
    // std::cout << "indexes: " << std::endl;
    // 进行采样
    for (int i = 0; i < X_SAMPLE_NUM + 1; i++) {
        for (int j = 0; j < Y_SAMPLE_NUM + 1; j++) {
            for (int k = 0; k < THETA_SAMPLE_NUM + 1; k++) {
                if (voxel_nodes_store[i][j][k].size() > 0) {
                    int sampled_index = rand() % voxel_nodes_store[i][j][k].size();
                    sampled_seed_nodes.push_back(voxel_nodes_store[i][j][k][sampled_index]);
                }
            }
        }
    }
    return sampled_seed_nodes;
}

// 速度空间采样的状态拓展
std::vector<State> SeedPolicy::stateExpanding(const State &current_state, int max_v_sample_num, int max_w_sample_num) {
    std::vector<State> expand_states;
    // 首先计算采样边界
    double v_upper_boundary = std::min(current_state.v_ + this->time_step_ * this->max_acc_, this->max_v_);
    double v_lower_boundary = std::max(current_state.v_ - this->time_step_ * this->max_acc_, this->min_v_);
    double w_upper_boundary = std::min(current_state.w_ + this->time_step_ * this->max_angular_acc_, this->max_w_);
    double w_lower_boundary = std::max(current_state.w_ - this->time_step_ * this->max_angular_acc_, -this->max_w_);
    // 进行采样
    std::vector<double> v_samples = Tools::linspace<double>(v_lower_boundary, v_upper_boundary, max_v_sample_num);
    std::vector<double> w_samples = Tools::linspace<double>(w_lower_boundary, w_upper_boundary, max_w_sample_num);
    for (double v_sample: v_samples) {
        for (double w_sample: w_samples) {
            State expand_state;
            expand_state.v_ = v_sample;
            expand_state.w_ = w_sample;
            expand_state.theta_ = Tools::pi2Pi(current_state.theta_ + this->time_step_ * w_sample);
            expand_state.x_ = current_state.x_ + v_sample * this->time_step_ * cos(expand_state.theta_);
            expand_state.y_ = current_state.y_ + v_sample * this->time_step_ * sin(expand_state.theta_);
            expand_states.push_back(expand_state);
        }
    }
    return expand_states;
}

// 获取未来的栅格图(假设障碍物匀速直线运动)
std::array<GridMap, PREDICT_TIME_LEN> SeedPolicy::predictFutureEnvironment(Robot &robot, const GridMap &global_map, const std::map<int, Tools::FixedQueue<ObstacleInfo, OBSTACLE_INFO_LEN>> &obstacles_info) {
    std::array<GridMap, PREDICT_TIME_LEN> predict_gridmaps;
    // 读取预测方法
    std::string prediction_method = this->pt_.get<std::string>("robot.prediction");
    if (prediction_method == "kf") {
        throw;
    } else {
        // 遍历未来每一个时刻
        for (int i = 0; i < PREDICT_TIME_LEN; i++) {
            // 计算对应时刻障碍物的预测位置
            obstacle_msgs::CircleObstacles current_obstacles;
            current_obstacles.header.frame_id = "odom";
            current_obstacles.header.stamp = ros::Time::now();
            for (auto obstacle_info: obstacles_info) {
                obstacle_msgs::CircleObstacle obstacle;
                obstacle.id = obstacle_info.first;
                obstacle.radius = obstacle_info.second.back().radius_;
                obstacle.twist.linear.x = obstacle_info.second.back().vx_;
                obstacle.twist.linear.y = obstacle_info.second.back().vy_;
                obstacle.point.x = obstacle_info.second.back().x_ + obstacle_info.second.back().vx_ * static_cast<double>(i) * this->time_step_;
                obstacle.point.y = obstacle_info.second.back().y_ + obstacle_info.second.back().vy_ * static_cast<double>(i) * this->time_step_;
                current_obstacles.obstacles.push_back(obstacle);
            }
            // 获得局部地图
            local_map_generation::GetLocalMapRequest get_local_map_req;
            local_map_generation::GetLocalMapResponse get_local_map_res;
            get_local_map_req.pose.position.x = robot.getPose().x_;
            get_local_map_req.pose.position.y = robot.getPose().y_;
            get_local_map_req.pose.orientation.z = sin(robot.getPose().theta_ * 0.5);
            get_local_map_req.pose.orientation.w = cos(robot.getPose().theta_ * 0.5);
            get_local_map_req.obstacles = current_obstacles;
            get_local_map_req.global_map = global_map.toRosMessage("odom");
            get_local_map_req.publish = false;
            if (!this->local_map_service_.call(get_local_map_req, get_local_map_res)) {
                std::cerr << "call local map service failed" << std::endl;
                exit(0);
            }
            GridMap gridmap = GridMap(get_local_map_res.local_map);
            // 根据机器人半径对地图进行膨胀
            int range = std::ceil(this->robot_radius_ / gridmap.getResolution());
            cv::Mat raw_image, dilated_image;
            raw_image = gridmap.toImage();
            cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * range + 1, 2 * range + 1), cv::Point(range, range));
            cv::dilate(raw_image, dilated_image, element);
            gridmap.loadImage(dilated_image);
            // 保存栅格
            predict_gridmaps[i] = gridmap;
        }
    }
    return predict_gridmaps;
}

// 生成参考状态序列
std::vector<State> SeedPolicy::generateReferenceStates(const std::vector<PathPose> &navigation_path) {
    std::vector<State> ref_states;
    ref_states.resize(PREDICT_TIME_LEN);
    std::vector<double> target_speeds;
    target_speeds.resize(navigation_path.size(), this->max_v_);
    for (int i = navigation_path.size() - 1; i >= 0; i--) {
        double distance = std::max(0.0, navigation_path.back().dist_ - navigation_path[i].dist_);
        double velocity = sqrt(2.0 * this->max_acc_ * distance);
        if (Tools::isSmall(velocity, this->max_v_)) {
            target_speeds[i] = velocity;
        } else {
            break;
        }
    }
    // 计算当前状态在全局路径上的对应点
    double min_distance = std::numeric_limits<double>::max();
    int current_position_index = navigation_path.size() - 1;
    for (int i = 0; i < navigation_path.size(); i++) {
        double distance = sqrt(navigation_path[i].x_ * navigation_path[i].x_ + navigation_path[i].y_ * navigation_path[i].y_);
        if (Tools::isSmall(distance, min_distance)) {
            min_distance = distance;
            current_position_index = i;
        }
    }
    // 计算参考状态序列
     for (int frame_index = 0; frame_index < PREDICT_TIME_LEN; frame_index++) {
        double distance = std::max(target_speeds[current_position_index] * cos(navigation_path[current_position_index].theta_), 0.0) * static_cast<double>(frame_index) * this->time_step_;
        int future_index = navigation_path.size() - 1;
        for (int index = current_position_index; index < navigation_path.size(); index++) {
            if (Tools::isLarge(navigation_path[index].dist_ - navigation_path[current_position_index].dist_, distance)) {
                future_index = std::max(index - 1, current_position_index);
                break;
            }
        }
        State future_state;
        future_state.x_ = navigation_path[future_index].x_;
        future_state.y_ = navigation_path[future_index].y_;
        future_state.theta_ = navigation_path[future_index].theta_;
        future_state.v_ = Tools::clip(target_speeds[future_index], this->min_v_, this->max_v_);
        future_state.w_ = 0.0;
        ref_states[frame_index] = future_state;
    }
    return ref_states;
}

// 计算路径坐标转换
std::vector<PathPose> SeedPolicy::transformPathToNewCoordinate(const Pose &new_coordination_origin, const std::vector<PathPose> &path) {
    std::vector<PathPose> new_path;
    for (auto path_point: path) {
        // 进行坐标转换
        Position new_path_poisition = Tools::calcNewCoordinationPosition(new_coordination_origin, {path_point.x_, path_point.y_});
        double new_path_theta = Tools::pi2Pi(path_point.theta_ - new_coordination_origin.theta_);
        // std::cout << "path_point: " << path_point.x_ << ", " << path_point.y_ << ", " << path_point.theta_ << ", " << path_point.dist_ << std::endl;
        // 进行角度转换
        new_path.push_back({new_path_poisition.x_, new_path_poisition.y_, new_path_theta, path_point.dist_});
    }
    return new_path;
}