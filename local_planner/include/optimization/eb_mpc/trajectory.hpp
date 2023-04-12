/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

#include "optimization/eb_mpc/vertex_state.hpp"

namespace EbMpcOptimization {

typedef std::vector<VertexState*> VertexPtrSequence;

// 轨迹数据结构
class Trajectory {
 public:
    // 构造函数
    Trajectory() {};
    // 析构函数
    ~Trajectory() {};

    // 获取顶点列表
    const VertexPtrSequence& vertices() const {
        return this->vertice_ptr_vec_;
    }

    // 获取顶点
    VertexState* vertex(int index) {
        assert(index < this->sizeVertices());
        return this->vertice_ptr_vec_.at(index);
    }

    // 获取最后的顶点
    VertexState* backVertex() {
        return this->vertice_ptr_vec_.back();
    }

    // 添加顶点
    void addVertex(const State& state, bool fixed=false) {
        VertexState* state_vertex_ptr = new VertexState(state, fixed);
        this->vertice_ptr_vec_.push_back(state_vertex_ptr);
        return;
    }

    // 添加顶点
    void addVertex(double x, double y, double theta, double v, double w, bool fixed = false) {
        VertexState* state_vertex_ptr = new VertexState(x, y, theta, v, w, fixed);
        this->vertice_ptr_vec_.push_back(state_vertex_ptr);
        return;
    }

    // 插入顶点
    void insertVertex(int index, const State& state, bool fixed = false) {
        VertexState* state_vertex_ptr = new VertexState(state, fixed);
        this->vertice_ptr_vec_.insert(this->vertice_ptr_vec_.begin() + index, state_vertex_ptr);
        return;
    }

    // 插入顶点
    void insertVertex(int index, double x, double y, double theta, double v, double w, bool fixed = false) {
        VertexState* state_vertex_ptr = new VertexState(x, y, theta, v, w, fixed);
        this->vertice_ptr_vec_.insert(this->vertice_ptr_vec_.begin() + index, state_vertex_ptr);
        return;
    }

    // 删除顶点
    void deleteVertex(int index) {
        assert(index < this->sizeVertices());
        delete this->vertice_ptr_vec_.at(index);
        this->vertice_ptr_vec_.erase(this->vertice_ptr_vec_.begin() + index);
    }

    // 设置顶点为固定
    void setVertexFixed(int index, bool status) {
        assert(index < this->sizeVertices());
        this->vertice_ptr_vec_.at(index)->setFixed(status);   
    }

    // 清空轨迹
    void clearTrajectory() {
        for (VertexPtrSequence::iterator vertex_it = this->vertice_ptr_vec_.begin(); vertex_it != this->vertice_ptr_vec_.end(); ++vertex_it) {
            delete *vertex_it;
        }
        this->vertice_ptr_vec_.clear();
    }

    // 获取顶点的长度
    int sizeVertices() const {
        return static_cast<int>(this->vertice_ptr_vec_.size());
    }

    // 判断是否为空
    bool isEmpty() const {
        return this->vertice_ptr_vec_.empty();
    }

 private:
    // 顶点列表
    VertexPtrSequence vertice_ptr_vec_;
};

};

#endif  // TRAJECTORY_HPP_