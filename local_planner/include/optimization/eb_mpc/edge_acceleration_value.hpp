/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef EDGE_ACCELERATION_VALUE_HPP_
#define EDGE_ACCELERATION_VALUE_HPP_

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include "optimization/eb_mpc/vertex_state.hpp"

namespace EbMpcOptimization {

// 加速度值边
class EdgeAccelerationValue: public g2o::BaseBinaryEdge<2, double, VertexState, VertexState> {
 public:
    using typename g2o::BaseBinaryEdge<2, double, VertexState, VertexState>::ErrorVector;
    using g2o::BaseBinaryEdge<2, double, VertexState, VertexState>::computeError;

    // 构造函数
    EdgeAccelerationValue() {
        this->setMeasurement(0.);
    }

    // 损失函数计算
    void computeError() {
        // 得到节点
        const VertexState* conf1 = static_cast<const VertexState*>(this->_vertices[0]);
        const VertexState* conf2 = static_cast<const VertexState*>(this->_vertices[1]);
        // 计算加速度值损失
        this->_error[0] = (conf2->v() - conf1->v()) / this->_measurement;
        // 计算角加速度值损失
        this->_error[1] = (conf2->w() - conf1->w()) / this->_measurement;
        // 验证损失计算结果
        ROS_ASSERT_MSG(std::isfinite(this->_error[0]), "EdgeAccelerationValue::computeError() _error[0]=%f\n", this->_error[0]);
        ROS_ASSERT_MSG(std::isfinite(this->_error[1]), "EdgeAccelerationValue::computeError() _error[1]=%f\n", this->_error[1]);
    }

    // 设置参考状态
    void setTimeDiff(double time_diff) {
        this->_measurement = time_diff;
    }

    // 获取损失
    ErrorVector& getError() {
        computeError();
        return this->_error;
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
    using g2o::BaseBinaryEdge<2, double, VertexState, VertexState>::_error;
    using g2o::BaseBinaryEdge<2, double, VertexState, VertexState>::_vertices;
 
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};

#endif  // EDGE_ACCELERATION_VALUE_HPP_