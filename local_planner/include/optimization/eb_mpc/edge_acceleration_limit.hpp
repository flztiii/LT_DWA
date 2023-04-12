/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef EDGE_ACCELERATION_LIMIT_HPP_
#define EDGE_ACCELERATION_LIMIT_HPP_

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include "optimization/eb_mpc/vertex_state.hpp"
#include "optimization/penalties.h"

namespace EbMpcOptimization {

// 速度限制边
class EdgeAccelerationLimit: public g2o::BaseBinaryEdge<2, std::array<double, 4>, VertexState, VertexState> {
 public:
    using typename g2o::BaseBinaryEdge<2, std::array<double, 4>, VertexState, VertexState>::ErrorVector;
    using g2o::BaseBinaryEdge<2, std::array<double, 4>, VertexState, VertexState>::computeError;

    // 构造函数
    EdgeAccelerationLimit() {}

    // 计算损失函数
    void computeError() {
        // 得到节点
        const VertexState* conf1 = static_cast<const VertexState*>(this->_vertices[0]);
        const VertexState* conf2 = static_cast<const VertexState*>(this->_vertices[1]);
        // 计算加速度
        double acceleration = (conf2->v() - conf1->v()) / this->dt_;
        // 计算角加速度
        double angular_acceleration = (conf2->w() - conf1->w()) / this->dt_;
        // 计算损失
        this->_error[0] = Optimization::penaltyBoundToInterval(acceleration, this->_measurement[0], this->_measurement[1], 0.01);
        this->_error[1] = Optimization::penaltyBoundToInterval(angular_acceleration, this->_measurement[2], this->_measurement[3], 0.01);
        ROS_ASSERT_MSG(std::isfinite(this->_error[0]), "EdgeAccelerationLimit::computeError() _error[0]=%f\n", this->_error[0]);
        ROS_ASSERT_MSG(std::isfinite(this->_error[1]), "EdgeAccelerationLimit::computeError() _error[1]=%f\n", this->_error[1]);
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

    void setTimeDiff(double dt) {
        this->dt_ = dt;
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
    using g2o::BaseBinaryEdge<2, std::array<double, 4>, VertexState, VertexState>::_error;
    using g2o::BaseBinaryEdge<2, std::array<double, 4>, VertexState, VertexState>::_vertices;

    double dt_;
 
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};

#endif  // EDGE_ACCELERATION_LIMIT_HPP_
