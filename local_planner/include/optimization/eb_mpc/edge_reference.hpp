/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef EDGE_REFERENCE_HPP_
#define EDGE_REFERENCE_HPP_

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include "optimization/eb_mpc/vertex_state.hpp"

namespace EbMpcOptimization {

// 参考轨迹边
class EdgeReference: public g2o::BaseUnaryEdge<5, State, VertexState> {
 public:
    using typename g2o::BaseUnaryEdge<5, State, VertexState>::ErrorVector;
    using g2o::BaseUnaryEdge<5, State, VertexState>::computeError;

    // 构造函数
    EdgeReference() {}

    // 计算损失函数
    void computeError() {
        // 获取顶点
        const VertexState* vertex_ptr = static_cast<const VertexState*>(this->_vertices[0]);
        // 计算损失
        // 纵向坐标差损失
        this->_error[0] = (vertex_ptr->x() - this->_measurement.x_) * cos(this->_measurement.theta_) + (vertex_ptr->y() - this->_measurement.y_) * sin(this->_measurement.theta_);
        // 横向坐标差损失
        this->_error[1] = -(vertex_ptr->x() - this->_measurement.x_) * sin(this->_measurement.theta_) + (vertex_ptr->y() - this->_measurement.y_) * cos(this->_measurement.theta_);
        // 角度差损失
        this->_error[2] = 1 - cos(vertex_ptr->theta() - this->_measurement.theta_);
        // 速度差损失
        this->_error[3] = vertex_ptr->v() - this->_measurement.v_;
        // 角速度差损失
        this->_error[4] = vertex_ptr->w() - this->_measurement.w_;
        // 验证损失
        ROS_ASSERT_MSG(std::isfinite(this->_error[0]), "EdgeReference::computeError() _error[0]=%f\n", this->_error[0]);
        ROS_ASSERT_MSG(std::isfinite(this->_error[1]), "EdgeReference::computeError() _error[1]=%f\n", this->_error[1]);
        ROS_ASSERT_MSG(std::isfinite(this->_error[2]), "EdgeReference::computeError() _error[2]=%f\n", this->_error[2]);
        ROS_ASSERT_MSG(std::isfinite(this->_error[3]), "EdgeReference::computeError() _error[3]=%f\n", this->_error[3]);
        ROS_ASSERT_MSG(std::isfinite(this->_error[4]), "EdgeReference::computeError() _error[4]=%f\n", this->_error[4]);
    }

    // 获取损失
    ErrorVector& getError() {
        computeError();
        return this->_error;
    }

    // 设置参考状态
    void setReference(State state) {
        this->_measurement = state;
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
    using g2o::BaseUnaryEdge<5, State, VertexState>::_error;
    using g2o::BaseUnaryEdge<5, State, VertexState>::_vertices;
 
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};

#endif  // EDGE_REFERENCE_HPP_
