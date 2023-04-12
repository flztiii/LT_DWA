/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef VERTEX_STATE_HPP_
#define VERTEX_STATE_HPP_

#include <g2o/config.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/stuff/misc.h>
#include "util/common.hpp"

namespace EbMpcOptimization {

class VertexState: public g2o::BaseVertex<5, State> {
 public:
    // 构造函数
    VertexState(bool fixed = false) {
        setToOriginImpl();
        setFixed(fixed);
    }

    // 构造函数
    VertexState(const State& state, bool fixed = false) {
        this->_estimate = state;
        setFixed(fixed);
    }

    // 构造函数
    VertexState(double x, double y, double theta, double v, double w, bool fixed = false) {
        this->_estimate = {x, y, theta, v, w};
        setFixed(fixed);
    }

    // 获取属性
    inline const State state() const {
        return this->_estimate;
    }

    // 获取属性
    inline const double x() const {
        return this->_estimate.x_;
    }

    // 获取属性
    inline const double y() const {
        return this->_estimate.y_;
    }

    // 获取属性
    inline const double theta() const {
        return this->_estimate.theta_;
    }

    // 获取属性
    inline const double v() const {
        return this->_estimate.v_;
    }

    // 获取属性
    inline const double w() const {
        return this->_estimate.w_;
    }


    // 复写虚函数
    virtual void setToOriginImpl() override {
        this->_estimate.x_ = 0;
        this->_estimate.y_ = 0;
        this->_estimate.theta_ = 0;
        this->_estimate.v_ = 0;
        this->_estimate.w_ = 0;
    }

    // 复写虚函数
    virtual void oplusImpl(const double* update) override {
        this->_estimate.x_ += update[0];
        this->_estimate.y_ += update[1];
        this->_estimate.theta_ = g2o::normalize_theta(this->_estimate.theta_ + update[2]);
        this->_estimate.v_ += update[3];
        this->_estimate.w_ += update[4];
    }

    // 复写虚函数
    virtual bool read(std::istream& is) override {
        is >> this->_estimate.x_ >> this->_estimate.y_ >> this->_estimate.theta_ >> this->_estimate.v_ >> this->_estimate.w_;
        return true;
    }

    // 复写虚函数
    virtual bool write(std::ostream& os) const override {
        os << this->_estimate.x_ << " " << this->_estimate.y_ << " " << this->_estimate.theta_ << " " << this->_estimate.v_ << " " << this->_estimate.w_;
        return os.good();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};

#endif  // VERTEX_STATE_HPP_