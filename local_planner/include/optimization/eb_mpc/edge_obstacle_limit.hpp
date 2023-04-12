/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef EDGE_OBSTACLE_LIMIT_HPP_
#define EDGE_OBSTACLE_LIMIT_HPP_

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include "optimization/eb_mpc/vertex_state.hpp"
#include "optimization/penalties.h"
#include "util/tools.hpp"

namespace EbMpcOptimization {

// 参考轨迹边
class EdgeObstacleLimit: public g2o::BaseUnaryEdge<1, std::vector<Line2d>, VertexState> {
 public:
    using typename g2o::BaseUnaryEdge<1, std::vector<Line2d>, VertexState>::ErrorVector;
    using g2o::BaseUnaryEdge<1, std::vector<Line2d>, VertexState>::computeError;

    // 构造函数
    EdgeObstacleLimit() {}

    // 计算损失函数
    void computeError() {
        // 获取顶点
        const VertexState* vertex_ptr = static_cast<const VertexState*>(this->_vertices[0]);
        // 计算离边界距离
        double min_dist = std::numeric_limits<double>::max();
        for (auto line: this->_measurement) {
            double dist = line.b_ - (line.norm_.x_ * vertex_ptr->x() + line.norm_.y_ * vertex_ptr->y());
            if (Tools::isSmall(dist, min_dist)) {
                min_dist = dist;
            }
        }
        this->_error[0] = Optimization::penaltyBoundFromBelow(min_dist, 0.0, 0.01);
        // 验证损失
        ROS_ASSERT_MSG(std::isfinite(this->_error[0]), "EdgeObstacleLimit::computeError() _error[0]=%f\n", this->_error[0]);
    }

    // 获取损失
    ErrorVector& getError() {
        computeError();
        return this->_error;
    }

    // 设置参考状态
    void setConstraintRegion(const std::vector<Line2d> &constraint_region) {
        this->_measurement = constraint_region;
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
    using g2o::BaseUnaryEdge<1, std::vector<Line2d>, VertexState>::_error;
    using g2o::BaseUnaryEdge<1, std::vector<Line2d>, VertexState>::_vertices;
 
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};

#endif  // EDGE_OBSTACLE_LIMIT_HPP_