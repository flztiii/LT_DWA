/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef EDGE_VELOCITY_LIMIT_HPP_
#define EDGE_VELOCITY_LIMIT_HPP_

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include "optimization/eb_mpc/vertex_state.hpp"
#include "optimization/penalties.h"

namespace EbMpcOptimization {

// 速度限制边
class EdgeVelocityLimit: public g2o::BaseUnaryEdge<2, std::array<double, 4>, VertexState> {
 public:
    using typename g2o::BaseUnaryEdge<2, std::array<double, 4>, VertexState>::ErrorVector;
    using g2o::BaseUnaryEdge<2, std::array<double, 4>, VertexState>::computeError;

    // 构造函数
    EdgeVelocityLimit() {}

    // 计算损失函数
    void computeError() {
        // 获取顶点
        const VertexState* vertex_ptr = static_cast<const VertexState*>(this->_vertices[0]);
        // 计算损失
        this->_error[0] = Optimization::penaltyBoundToInterval(vertex_ptr->v(), this->_measurement[0], this->_measurement[1], 0.01);
        this->_error[1] = Optimization::penaltyBoundToInterval(vertex_ptr->w(), this->_measurement[2], this->_measurement[3], 0.01);
        ROS_ASSERT_MSG(std::isfinite(this->_error[0]), "EdgeVelocityLimit::computeError() _error[0]=%f\n", this->_error[0]);
        ROS_ASSERT_MSG(std::isfinite(this->_error[1]), "EdgeVelocityLimit::computeError() _error[1]=%f\n", this->_error[1]);
    }

    // 获取损失
    ErrorVector& getError() {
        computeError();
        return this->_error;
    }

    // 设置参考状态
    void setLimit(std::array<double, 4> limit) {
        this->_measurement = limit;
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
    using g2o::BaseUnaryEdge<2, std::array<double, 4>, VertexState>::_error;
    using g2o::BaseUnaryEdge<2, std::array<double, 4>, VertexState>::_vertices;
 
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};

#endif  // EDGE_VELOCITY_LIMIT_HPP_
