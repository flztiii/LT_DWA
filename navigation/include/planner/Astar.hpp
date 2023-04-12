/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef ASTAR_HPP_
#define ASTAR_HPP_

#include "BasePlanner.hpp"
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <typeinfo>
#include <thread>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <queue>
#include <set>
#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include <ctime>

// A星规划器
class AstarPlanner: public BasePlanner {
 public:
    // 构造函数
    AstarPlanner(double robot_size){
        this->robot_size_ = robot_size;
        this->motions_.push_back(std::make_pair(-1, -1));
        this->motions_.push_back(std::make_pair(-1, 0));
        this->motions_.push_back(std::make_pair(-1, 1));
        this->motions_.push_back(std::make_pair(0, -1));
        this->motions_.push_back(std::make_pair(0, 1));
        this->motions_.push_back(std::make_pair(1, -1));
        this->motions_.push_back(std::make_pair(1, 0));
        this->motions_.push_back(std::make_pair(1, 1));
    };

    // 构造函数
    AstarPlanner() {};

    // 析构函数
    ~AstarPlanner(){};

    // 进行规划
    int planning(const PathPlanningUtilities::Point2f &start_point, const PathPlanningUtilities::Point2f &goal_point, const GridMap &grid_map, const KDTree &kd_tree, PathPlanningUtilities::Path &path) override {
        // 进行A星搜索
        // 初始化节点记录器
        std::vector<Node> node_recorder;
        node_recorder.resize(grid_map.getWidth() * grid_map.getHeight());
        // 初始化搜索记录器
        std::priority_queue<Node, std::vector<Node>, std::greater<Node> > open_set;  // 未搜索节点
        // 构建起始节点和目标节点
        std::pair<int, int> start_in_grid = grid_map.getGridMapCoordinate(start_point.x_, start_point.y_);
        std::pair<int, int> goal_in_grid = grid_map.getGridMapCoordinate(goal_point.x_, goal_point.y_);
        Node start_node = Node(start_in_grid.first, start_in_grid.second);
        Node goal_node = Node(goal_in_grid.first, goal_in_grid.second);
        // 计算起点的代价和启发
        start_node.cost_ = 0.0;
        start_node.heuristic_ = this->calcHeuristic(start_node, goal_node);
        // 将起始节点加入搜索列表
        start_node.open();
        node_recorder[grid_map.getIndex(start_node.x_, start_node.y_)] = start_node;
        open_set.push(start_node);
        // 开始搜索
        while (!open_set.empty()) {
            // 得到搜索列表中最小的index作为当前节点,并从开集合中删除
            Node current_node = open_set.top();
            open_set.pop();
            // 将当前节点加入闭集合,
            int current_index = grid_map.getIndex(current_node.x_, current_node.y_);
            node_recorder[current_index].close();
            // 判断当前节点是否为终点
            if (current_node.x_ == goal_node.x_ && current_node.y_ == goal_node.y_) {
                // 当前节点为终点,结束搜索
                goal_node = current_node;
                break;
            }
            // 如果当前节点不是终点,搜索其的邻居
            for (auto motion: this->motions_) {
                // 计算邻居节点
                Node neighbor_node = Node(current_node.x_ + motion.first, current_node.y_ + motion.second, current_index);
                // 得到邻居节点下标
                int neighbor_index = grid_map.getIndex(neighbor_node.x_, neighbor_node.y_);
                // 判断邻居节点是否在闭集合内
                if (node_recorder[neighbor_index].is_closed_ == true) {
                    // 存在于闭集合内
                    continue;
                }
                // 判断邻居是否超出边界
                if(!grid_map.isVerify(neighbor_node.x_, neighbor_node.y_)) {
                    continue;
                }
                // 判断此邻居是否与障碍物碰撞
                bool is_neighbor_collide = false;
                int range = static_cast<int>(this->robot_size_ / Config::grid_resolution_);
                for (int i = - range; i < range + 1; i++) {
                    for (int j = -range; j < range + 1; j++) {
                        if (Tools::isLarge(sqrt(static_cast<double>(i * i + j * j)), static_cast<double>(range))) {
                            continue;
                        }
                        if (!grid_map.isVerify(neighbor_node.x_ + i, neighbor_node.y_ + j)) {
                            is_neighbor_collide = true;
                            goto end;
                        } else if (grid_map.isOccupied(grid_map.getIndex(neighbor_node.x_ + i, neighbor_node.y_ + j))) {
                            is_neighbor_collide = true;
                            goto end;
                        }
                    }
                }
                end:
                if (is_neighbor_collide) {
                    continue;
                }
                // 计算损失增量,包括两个部分,走过的距离和离障碍物的距离
                double motion_cost = sqrt(motion.first * motion.first + motion.second * motion.second);
                // 得到邻居的损失
                neighbor_node.cost_ = current_node.cost_ + motion_cost;
                // 得到邻居的启发
                neighbor_node.heuristic_ = this->calcHeuristic(neighbor_node, goal_node);
                // 判断邻居节点是否在开集合内
                if (node_recorder[neighbor_index].is_open_ == true) {
                    // 在开集合内
                    if (neighbor_node < node_recorder[neighbor_index]) {
                        neighbor_node.open();
                        node_recorder[neighbor_index] = neighbor_node;
                        open_set.push(neighbor_node);
                    }
                } else {
                    // 不在开集合内
                    // 加入开集合
                    neighbor_node.open();
                    node_recorder[neighbor_index] = neighbor_node;
                    open_set.push(neighbor_node);
                }
            }
        }
        // 开始生成最终路径
        // 判断是否生成了最终路径
        if (goal_node.pre_index_ == -1) {
            // 没有找到路径
            return -1;
        }
        // 生成了最终路径
        PathPlanningUtilities::Path raw_path;
        Node current_node = goal_node;
        while (current_node.pre_index_ != -1) {
            PathPlanningUtilities::Point2f point = grid_map.getCartesianCoordinate(current_node.x_, current_node.y_);
            raw_path.push_back(point);
            current_node = node_recorder[current_node.pre_index_];
            assert(current_node.is_closed_);
        }
        // 反转路径后输出
        reverse(raw_path.begin(),raw_path.end());
        // 给raw_path补点
        bool interpolation_finished = false;
        while (!interpolation_finished) {
            interpolation_finished = true;
            for (size_t i = 1; i < raw_path.size(); i++) {
                // 计算两点之间的距离
                double distance = PathPlanningUtilities::calcDistance(raw_path[i - 1], raw_path[i]);
                if (Tools::isLarge(distance, 2.0 * Config::grid_resolution_)) {
                    // 在两点之间插入一个点
                    PathPlanningUtilities::Point2f new_point;
                    new_point.x_ = (raw_path[i - 1].x_ + raw_path[i].x_) * 0.5;
                    new_point.y_ = (raw_path[i - 1].y_ + raw_path[i].y_) * 0.5;
                    raw_path.insert(raw_path.begin() + i, new_point);
                    interpolation_finished = false;
                    break;
                }
            }
        }
        path = raw_path;
        return 0;
    };
 
 private:
    // 计算启发函数(使用的启发函数为到终点的距离)
    double calcHeuristic(const Node &current_node, const Node &goal_node, double weight= 1.0) {
        return sqrt((current_node.x_ - goal_node.x_) * (current_node.x_ - goal_node.x_) + (current_node.y_ - goal_node.y_) * (current_node.y_ - goal_node.y_)) * weight;
    }

    double robot_size_;  // 机器人大小
    std::vector<std::pair<int, int>> motions_;  // 行为
};

#endif