/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef EDGE_KINEMATIC_HPP_
#define EDGE_KINEMATIC_HPP_

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include "optimization/eb_mpc/vertex_state.hpp"

namespace EbMpcOptimization {

// 动力学限制边
class EdgeKinematic: public g2o::BaseBinaryEdge<3, double, VertexState, VertexState> {
 public:
    using typename g2o::BaseBinaryEdge<3, double, VertexState, VertexState>::ErrorVector;
    using g2o::BaseBinaryEdge<3, double, VertexState, VertexState>::computeError;

    // 构造函数
    EdgeKinematic() {}

    // 计算损失函数
    void computeError() {
        // 得到节点
        const VertexState* conf1 = static_cast<const VertexState*>(this->_vertices[0]);
        const VertexState* conf2 = static_cast<const VertexState*>(this->_vertices[1]);
        this->_error[0] = conf2->x() - (conf1->x() + this->_measurement * conf2->v() * cos(conf2->theta()));
        this->_error[1] = conf2->y() - (conf1->y() + this->_measurement * conf2->v() * sin(conf2->theta()));
        this->_error[2] = 1 - cos(conf2->theta() - (conf1->theta() + this->_measurement * conf2->w()));
        ROS_ASSERT_MSG(std::isfinite(this->_error[0]), "EdgeKinematic::computeError() _error[0]=%f\n", this->_error[0]);
        ROS_ASSERT_MSG(std::isfinite(this->_error[1]), "EdgeKinematic::computeError() _error[1]=%f\n", this->_error[1]);
        ROS_ASSERT_MSG(std::isfinite(this->_error[2]), "EdgeKinematic::computeError() _error[1]=%f\n", this->_error[2]);
    }

    // 获取损失
    ErrorVector& getError() {
        computeError();
        return this->_error;
    }

    void setTimeDiff(double dt) {
        this->_measurement = dt;
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
    using g2o::BaseBinaryEdge<3, double, VertexState, VertexState>::_error;
    using g2o::BaseBinaryEdge<3, double, VertexState, VertexState>::_vertices;
 
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};

#endif  // EDGE_KINEMATIC_HPP_