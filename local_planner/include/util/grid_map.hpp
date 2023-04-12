/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef GRID_MAP_HPP_
#define GRID_MAP_HPP_

#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include "tools.hpp"

// 栅格地图类
class GridMap {
 public:
    // 构造函数
    GridMap(const nav_msgs::OccupancyGrid &occupancy_grid){
        for (auto meta_data: occupancy_grid.data) {
            if (meta_data > 50 || meta_data < 0) {   
                this->data_.push_back(255);
            } else {
                this->data_.push_back(0);
            }
        }
        this->width_ = occupancy_grid.info.width;
        this->height_ = occupancy_grid.info.height;
        this->resolution_ = occupancy_grid.info.resolution;
        this->root_x_ = occupancy_grid.info.origin.position.x;
        this->root_y_ = occupancy_grid.info.origin.position.y;
        this->root_theta_ = 2.0 * atan2(occupancy_grid.info.origin.orientation.z, occupancy_grid.info.origin.orientation.w);
    };

    // 构造函数
    GridMap(int width, int height, double resolution, double root_x, double root_y, double root_theta) {
        this->width_ = width;
        this->height_ = height;
        this->resolution_ = resolution;
        this->root_x_ = root_x;
        this->root_y_ = root_y;
        this->root_theta_ = root_theta;
        this->data_ = std::vector<uchar>(width * height, 0);
    };

    // 构造函数
    GridMap(const cv::Mat &mat, double resolution, double root_x, double root_y, double root_theta) {
        this->width_ = mat.cols;
        this->height_ = mat.rows;
        this->resolution_ = resolution;
        this->root_x_ = root_x;
        this->root_y_ = root_y;
        this->root_theta_ = root_theta;
        cv::Mat mat_fliped;
        cv::flip(mat, mat_fliped, 0);
        this->data_ = Tools::convertMat2Vector<uchar>(mat_fliped);
    }

    // 构造函数
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
        std::vector<uchar> new_data(new_width * new_height, 0);
        for (int i = 0; i < this->width_; i++) {
            for (int j = 0; j < this->height_; j++) {
                // 计算是否被占据
                int index = this->getIndex(i, j);
                if (this->isOccupied(index)) {
                    // 如果被占据,判断它对应新栅格的下标
                    int new_index = static_cast<int>(i / ratio) + static_cast<int>(j / ratio) * new_width;
                    new_data[new_index] = 255;
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
        if (this->data_[index] == 255) {
            return true;
        } else {
            return false;
        }
    };

    // 修改对应栅格为被占据
    void setOccupied(int index) {
        this->data_[index] = 255;
    }

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

    // 判断是否抄出边界
    bool isVerify(int x, int y) const {
        if (x >= 0 && x < this->width_ and y >= 0 and y < height_) {
            return true;
        } else {
            return false;
        }
    }

    // 计算对应的栅格坐标
    std::pair<int, int> getGridMapCoordinate(double px, double py) const {
        Pose curve_point;
        curve_point.x_ = this->root_x_;
        curve_point.y_ = this->root_y_;
        curve_point.theta_ = this->root_theta_;
        Position point;
        point.x_ = px;
        point.y_ = py;
        Position new_point = Tools::calcNewCoordinationPosition(curve_point, point);
        int x = new_point.x_ / this->resolution_;
        int y = new_point.y_ / this->resolution_;
        return std::pair<int, int>(x, y);
    };
    
    // 计算真实坐标
    Position getCartesianCoordinate(int x, int y) const {
        Pose curve_point;
        curve_point.x_ = this->root_x_;
        curve_point.y_ = this->root_y_;
        curve_point.theta_ = this->root_theta_;
        Position point;
        point.x_ = static_cast<double>(x * this->resolution_);
        point.y_ = static_cast<double>(y * this->resolution_);
        Position raw_point = Tools::calcOldCoordinationPosition(curve_point, point);
        return raw_point;
    };

    // 转化成ros消息
    nav_msgs::OccupancyGrid toRosMessage(std::string frame_id) const{
        nav_msgs::OccupancyGrid occupancy_grid;
        geometry_msgs::Pose pose;
        pose.position.x = this->root_x_;
        pose.position.y = this->root_y_;
        pose.orientation.z = sin(this->root_theta_ * 0.5);
        pose.orientation.w = cos(this->root_theta_ * 0.5);
        std::vector<int8_t> out_data;
        int count = 0;
        for (auto meta_data: this->data_) {
            if (meta_data == 255) {
                out_data.push_back(100);
            } else {
                out_data.push_back(0);
            }
            count++;
        }
        occupancy_grid.data = out_data;
        occupancy_grid.info.resolution = this->resolution_;
        occupancy_grid.info.width = this->width_;
        occupancy_grid.info.height = this->height_;
        occupancy_grid.info.origin = pose;
        occupancy_grid.info.map_load_time = ros::Time::now();
        occupancy_grid.header.frame_id = frame_id;
        occupancy_grid.header.stamp = ros::Time::now();
        return occupancy_grid;
    };

    // 转换为图像
    cv::Mat toImage() const {
        cv::Mat image = Tools::convertVector2Mat<uchar>(this->data_, 1, this->height_);
        cv::Mat image_fliped;
	    cv::flip(image, image_fliped, 0);
        return image_fliped;
    };

    void loadImage(const cv::Mat &mat) {
        cv::Mat mat_fliped;
        cv::flip(mat, mat_fliped, 0);
        this->data_ = Tools::convertMat2Vector<uchar>(mat_fliped);
    }

    // 得到栅格地图的宽度
    int getWidth() const {
        return this->width_;
    };

    // 得到栅格地图的高度
    int getHeight() const {
        return this->height_;
    };

    double getResolution() const {
        return this->resolution_;
    };

    double getRootX() const {
        return this->root_x_;
    };

    double getRootY() const {
        return this->root_y_;
    };

    double getRootTheta() const {
        return this->root_theta_;
    };

 private:
    int width_;  // 栅格地图的宽度(格数)
    int height_;  // 栅格地图的高度(格数)
    double resolution_;  // 栅格地图的分辨率
    double root_x_;  // 栅格地图根节点x坐标
    double root_y_;  // 栅格地图根节点y坐标
    double root_theta_;  // 栅格地图根节点朝向
    std::vector<uchar> data_;  // 栅格地图数据
};

#endif  // GRID_MAP_HPP_
