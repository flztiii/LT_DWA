/*
    Copyright [2020] Jian ZhiQiang
*/

#ifndef COMMON_H_
#define COMMON_H_

#include "common/Point.hpp"
#include "common/Path.hpp"
#include "utilities/Tools.hpp"
#include <nav_msgs/OccupancyGrid.h>

// 节点类
class Node {
 public:
    // 构造函数
    Node(int x, int y, int pre_index = -1, double cost = 0.0, double heuristic = 0.0) {
        this->x_ = x;
        this->y_ = y;
        this->pre_index_ = pre_index;
        this->cost_ = cost;
        this->heuristic_ = heuristic;
        this->is_open_ = false;
        this->is_closed_ = false;
    };

    // 构造函数
    Node(){
        this->is_open_ = false;
        this->is_closed_ = false;
    };

    // 析构函数
    ~Node(){};

    // 重载大小比较
    bool operator<(const Node &node) const {
        return Tools::isSmall(this->cost_ + this->heuristic_, node.cost_ + node.heuristic_);
    };

    bool operator>(const Node &node) const {
        return Tools::isLarge(this->cost_ + this->heuristic_, node.cost_ + node.heuristic_);
    }

    void open() {
        this->is_open_ = true;
        this->is_closed_ = false;
    }

    void close() {
        this->is_open_ = false;
        this->is_closed_ = true;        
    }

    int x_;  // 节点宽度下标
    int y_;  // 节点高度下标
    int pre_index_;  // 上一个节点的下标
    double cost_;  // 节点的代价
    double heuristic_;  // 节点的启发
    bool is_open_;  // 是否属于open集合
    bool is_closed_;  // 是否属于closed集合
};

// 局部规划路径信息
class LocalPathInfo {
 public:
    // 构造函数
    LocalPathInfo() {}

    LocalPathInfo(const PathPlanningUtilities::Curve &curve, const std::vector<double> &curvature_change_rates, const std::vector<double> &arc_lengths, const std::vector<size_t> &reference_indexes, double offset) {
        this->curve_ = curve;
        this->curvature_change_rates_ = curvature_change_rates;
        this->arc_lengths_ = arc_lengths;
        this->reference_indexes_ = reference_indexes;
        this->offset_ = offset;
    }

    // 析构函数
    ~LocalPathInfo() {}

    PathPlanningUtilities::Curve curve_;  // 路径
    std::vector<double> curvature_change_rates_;  // 曲率变化率
    std::vector<double> arc_lengths_;  // 弧长
    std::vector<size_t> reference_indexes_;  // 相对参考线的下标
    double offset_;  // 离参考线偏移量
};

// 栅格地图类
class GridMap {
 public:
    // 构造函数
    GridMap(const nav_msgs::OccupancyGrid &occupancy_grid){
        for (auto meta_data: occupancy_grid.data) {
            if (meta_data > 50 || meta_data < 0) {   
                this->data_.push_back(true);
            } else {
                this->data_.push_back(false);
            }
        }
        this->width_ = occupancy_grid.info.width;
        this->height_ = occupancy_grid.info.height;
        this->resolution_ = occupancy_grid.info.resolution;
        this->root_x_ = occupancy_grid.info.origin.position.x;
        this->root_y_ = occupancy_grid.info.origin.position.y;
        this->root_theta_ = 2.0 * atan2(occupancy_grid.info.origin.orientation.z, occupancy_grid.info.origin.orientation.w);
    };

    GridMap() {};

    // 析构函数
    ~GridMap() {};

    // 修改分辨率
    void changeResolution(double new_resolution) {
        // 计算比例系数
        double ratio = new_resolution / this->resolution_;
        // 计算新的宽和高
        int new_width = static_cast<int>(this->width_ / ratio);
        int new_height = static_cast<int>(this->height_ / ratio);
        // 得到新的栅格数据
        std::vector<bool> new_data(new_width * new_height, false);
        for (int i = 0; i < this->width_; i++) {
            for (int j = 0; j < this->height_; j++) {
                // 计算是否被占据
                int index = this->getIndex(i, j);
                if (this->isOccupied(index)) {
                    // 如果被占据,判断它对应新栅格的下标
                    int new_index = static_cast<int>(i / ratio) + static_cast<int>(j / ratio) * new_width;
                    new_data[new_index] = true;
                }
            }
        }
        // 更新数据
        this->resolution_ = new_resolution;
        this->width_ = new_width;
        this->height_ = new_height;
        this->data_ = new_data;
        std::cout << "new width: " << this->width_ << ", new height: " << this->height_ << std::endl;
        std::cout << "data size: " << this->data_.size() << std::endl;
    }

    // 获取对应栅格是否被占据
    bool isOccupied(int index) const {
        return data_[index];
    };

    // 求对应点的栅格下标
    int getIndex(int x, int y) const {
        return x + y * this->width_;
    };

    // 计算栅格坐标
    std::pair<int, int> getXY(int index) const {
        int x = index % this->width_;
        int y = index / this->width_;
        return std::make_pair(x, y);
    };

    // 判断是否抄错边界
    bool isVerify(int x, int y) const {
        if (x >= 0 && x < this->width_ && y >= 0 && y < this->height_) {
            return true;
        } else {
            return false;
        }
    }

    // 计算对应的栅格坐标
    std::pair<int, int> getGridMapCoordinate(double px, double py) const {
        PathPlanningUtilities::CurvePoint curve_point;
        curve_point.position_.x_ = this->root_x_;
        curve_point.position_.y_ = this->root_y_;
        curve_point.theta_ = this->root_theta_;
        PathPlanningUtilities::Point2f point;
        point.x_ = px;
        point.y_ = py;
        PathPlanningUtilities::Point2f new_point = Tools::calcNewCoordinationPosition(curve_point, point);
        int x = new_point.x_ / this->resolution_;
        int y = new_point.y_ / this->resolution_;
        return std::pair<int, int>(x, y);
    };
    
    // 计算真实坐标
    PathPlanningUtilities::Point2f getCartesianCoordinate(int x, int y) const {
        PathPlanningUtilities::CurvePoint curve_point;
        curve_point.position_.x_ = this->root_x_;
        curve_point.position_.y_ = this->root_y_;
        curve_point.theta_ = this->root_theta_;
        PathPlanningUtilities::Point2f point;
        point.x_ = static_cast<double>(x * this->resolution_);
        point.y_ = static_cast<double>(y * this->resolution_);
        PathPlanningUtilities::Point2f raw_point = Tools::calcOldCoordinationPosition(curve_point, point);
        return raw_point;
    };

    // 转化成ros消息
    nav_msgs::OccupancyGrid toRosMessage() const{
        nav_msgs::OccupancyGrid occupancy_grid;
        geometry_msgs::Pose pose;
        pose.position.x = this->root_x_;
        pose.position.y = this->root_y_;
        pose.orientation.z = sin(this->root_theta_ * 0.5);
        pose.orientation.w = cos(this->root_theta_ * 0.5);
        std::vector<int8_t> out_data;
        for (auto meta_data: this->data_) {
            if (meta_data) {
                out_data.push_back(100);
            } else {
                out_data.push_back(0);
            }
        }
        occupancy_grid.data = out_data;
        occupancy_grid.info.resolution = this->resolution_;
        occupancy_grid.info.width = this->width_;
        occupancy_grid.info.height = this->height_;
        occupancy_grid.info.origin = pose;
        occupancy_grid.info.map_load_time = ros::Time::now();
        occupancy_grid.header.frame_id = "world";
        occupancy_grid.header.stamp = ros::Time::now();
        return occupancy_grid;
    };

    // 得到栅格地图的宽度
    int getWidth() const {
        return this->width_;
    };

    // 得到栅格地图的高度
    int getHeight() const {
        return this->height_;
    };

    // 得到分辨率
    double getResolution() const {
        return this->resolution_;
    }

    // 清空栅格中全部内容
    void clear() {
        for (size_t i = 0; i < this->data_.size(); i++) {
            this->data_[i] = false;
        }
    }

    // 设置栅格地图
    void set(size_t index, bool value) {
        this->data_[index] = value;
    }

 private:
    int width_;  // 栅格地图的宽度(格数)
    int height_;  // 栅格地图的高度(格数)
    double resolution_;  // 栅格地图的分辨率
    double root_x_;  // 栅格地图根节点x坐标
    double root_y_;  // 栅格地图根节点y坐标
    double root_theta_;  // 栅格地图根节点朝向
    std::vector<bool> data_;  // 栅格地图数据
};

#endif // COMMON_H_