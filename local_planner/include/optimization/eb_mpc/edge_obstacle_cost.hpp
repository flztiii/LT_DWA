/*
    Copyright [2022] Jian ZhiQiang
*/

#ifndef EDGE_OBSTACLE_COST_HPP_
#define EDGE_OBSTACLE_COST_HPP_

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include "optimization/eb_mpc/vertex_state.hpp"
#include "optimization/penalties.h"
#include "util/tools.hpp"
#include "util/obstacle_and_boundary.hpp"

namespace EbMpcOptimization {

// 参考轨迹边
class EdgeObstacleCost: public g2o::BaseUnaryEdge<1, ObstacleAndBoundary, VertexState> {
 public:
    using typename g2o::BaseUnaryEdge<1, ObstacleAndBoundary, VertexState>::ErrorVector;
    using g2o::BaseUnaryEdge<1, ObstacleAndBoundary, VertexState>::computeError;

    // 构造函数
    EdgeObstacleCost() {}

    // 计算损失函数
    void computeError() {
        // 获取顶点
        const VertexState* vertex_ptr = static_cast<const VertexState*>(this->_vertices[0]);
        // 计算损失
        this->_error[0] = sqrt(this->_measurement.calcCost(vertex_ptr->x(), vertex_ptr->y()));
        // 验证损失
        ROS_ASSERT_MSG(std::isfinite(this->_error[0]), "EdgeObstacleCost::computeError() _error[0]=%f\n", this->_error[0]);
    }

    // 获取损失
    ErrorVector& getError() {
        computeError();
        return this->_error;
    }

    // 设置参考状态
    void setDynamicObstacles(const ObstacleAndBoundary &obstacles_and_boundaries) {
        this->_measurement = obstacles_and_boundaries;
    }

    // 复写虚函数 
    virtual bool read(std::istream& is) {
        // TODO generic read
        return true;
    }

    // 复写虚函数 
    virtual bool write(std::ostream& os) const {
        // TODO generic write
        return os.good();
    }
 protected:
    using g2o::BaseUnaryEdge<1, ObstacleAndBoundary, VertexState>::_error;
    using g2o::BaseUnaryEdge<1, ObstacleAndBoundary, VertexState>::_vertices;
 
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};

#endif  // EDGE_OBSTACLE_COST_HPP_