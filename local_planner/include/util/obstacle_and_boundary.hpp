/*
    Copyright [2022] Jian ZhiQiang
*/

#ifndef OBSTACLE_AND_BOUNDARY_HPP_
#define OBSTACLE_AND_BOUNDARY_HPP_

#include "util/common.hpp"
#include "util/tools.hpp"
#include "util/kdtree.hpp"
#include "optimization/penalties.h"

class ObstacleAndBoundary {
 public:
    // 构造函数
    ObstacleAndBoundary(const std::vector<ObstacleInfo> &obstacles_info, const KDTree &kdtree, double resolution) {
        // 得到障碍物信息
        this->obstacles_info_ = obstacles_info;
        this->kdtree_ = kdtree;
        this->resolution_ = resolution;
        // 计算必要参数
        this->calcParams();
    };

    ObstacleAndBoundary() {};

    // 析构函数
    ~ObstacleAndBoundary() {};

    // 判断碰撞
    bool collisionJudge(double x, double y) const {
        bool result = false;
        // 障碍物碰撞
        for (auto obstacle_info: this->obstacles_info_) {
            double dist = sqrt(std::pow(obstacle_info.x_ - x, 2) + std::pow(obstacle_info.y_ - y, 2));
            if (Tools::isSmall(dist, obstacle_info.radius_)) {
                result = true;
                break;
            }
        }
        // 边界碰撞
        std::vector<std::pair<float, float>> results;
        std::vector<float> sq_distances;
        if (this->kdtree_.findRangeNeighbor(x, y, &results, &sq_distances, this->resolution_) != -1) {
            result = true;
        }
        return result;
    };

    // 计算代价
    double calcCost(double x, double y) const {
        // std::cout << "-----------------" << std::endl;
        double cost = 0.0;
        // for (int i = 0; i < this->obstacles_info_.size(); i++) {
        //     double dist = sqrt(std::pow(this->obstacles_info_[i].x_ - x, 2) + std::pow(this->obstacles_info_[i].y_ - y, 2));
        //     if (Tools::isSmall(dist, this->obstacles_info_[i].radius_)) {
        //         cost = 1.0;
        //         break;
        //     }
        // }
        // 障碍物代价
        for (int i = 0; i < this->obstacles_info_.size(); i++) {
            // 计算极坐标系坐标
            double dist = sqrt(std::pow(x - this->obstacles_info_[i].x_, 2) + std::pow(y - this->obstacles_info_[i].y_, 2));
            double alpha = Tools::pi2Pi(atan2(y - this->obstacles_info_[i].y_, x - this->obstacles_info_[i].x_) - this->thetas_[i]);
            // 计算标准差
            double sigma_x, sigma_y;
            if (Tools::isSmall(alpha, M_PI_2) && Tools::isLarge(alpha, -M_PI_2)) {
                sigma_x = this->sigma_sets_[i][0];
                sigma_y = this->sigma_sets_[i][1];
            } else {
                sigma_x = this->sigma_sets_[i][1];
                sigma_y = this->sigma_sets_[i][1];
            }
            // std::cout << "sigma: " << sigma_x << ", " << sigma_y << std::endl;
            double value = exp(-(std::pow(dist * cos(alpha), 2) / (2.0 * std::pow(sigma_x, 2)) + std::pow(dist * sin(alpha), 2) / (2.0 * std::pow(sigma_y, 2))));
            // std::cout << "value: " << value << std::endl;
            
            if (Tools::isLarge(value, cost)) {
                cost = value;
                // std::cout << "info: " << dist << ", " << alpha << ", " << value << std::endl;
            }
        }
        // 边界代价
        double min_dist = EVALUATE_MIN_DIST_TO_BOUNDARY;
        std::vector<std::pair<float, float>> results;
        std::vector<float> sq_distances;
        if (this->kdtree_.findKNeighbor(x, y, &results, &sq_distances, 1) != -1) {
            min_dist = sqrt(sq_distances.front());
        }
        if (Tools::isSmall(min_dist, this->resolution_)) {
            min_dist = 0.0;
        }
        double boundary_cost = static_cast<double>(EVALUATE_BOUNDARY_PENALTY_COFF) / static_cast<double>(EVALUATE_OBS_PENALTY_COFF) * std::pow(Optimization::penaltyBoundFromBelow(min_dist, EVALUATE_MIN_DIST_TO_BOUNDARY, 0.0), 2);

        cost = std::max(boundary_cost, cost);

        // std::cout << "-------------------------------cost: " << cost << std::endl;
        return cost;
    };

 private:
    std::vector<ObstacleInfo> obstacles_info_;
    KDTree kdtree_;
    double resolution_;
    std::vector<double> thetas_;
    std::vector<std::array<double, 2>> sigma_sets_;

    // 计算必要参数
    void calcParams() {
        for (auto obstacle_info: this->obstacles_info_) {
            double v = sqrt(obstacle_info.vx_ * obstacle_info.vx_ + obstacle_info.vy_ * obstacle_info.vy_);
            double theta = atan2(obstacle_info.vy_, obstacle_info.vx_);
            thetas_.push_back(theta);
            std::array<double, 2> segma_set;
            segma_set[0] = (obstacle_info.radius_ + EVALUATE_MIN_DIST_TO_OBS + EVALUATE_OBS_VEL_COFF * v) / 3.0;
            segma_set[1] = (obstacle_info.radius_ + EVALUATE_MIN_DIST_TO_OBS) / 3.0;
            // std::cout << "obstacle_info.radius_: " << obstacle_info.radius_ << std::endl;
            sigma_sets_.push_back(segma_set);
        }
    }
};

#endif  // OBSTACLE_AND_BOUNDARY_HPP_