/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef KDTREE_H_
#define KDTREE_H_

#include "util/common.hpp"
#include "util/grid_map.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

// 构建障碍物搜索KDtree
class KDTree {
 public:
    // 构造函数
    KDTree (const GridMap &grid_map) {
        pcl::PointCloud<pcl::PointXY>::Ptr obstacles (new pcl::PointCloud<pcl::PointXY>());
        // 遍历grid map,得到障碍物信息
        for (int i = 0; i < grid_map.getHeight() * grid_map.getWidth(); i++) {
            if (grid_map.isOccupied(i)) {
                // 如果栅格是占用的
                auto point_in_grid = grid_map.getXY(i);
                // std::cout << "gird: " << point_in_grid.first << ", " << point_in_grid.second << std::endl;
                Position point_in_cartesian = grid_map.getCartesianCoordinate(point_in_grid.first, point_in_grid.second);
                pcl::PointXY obstacle = {static_cast<float>(point_in_cartesian.x_), static_cast<float>(point_in_cartesian.y_)};
                // std::cout << "true: " << point_in_grid.first << ", " << point_in_grid.second << std::endl;
                obstacles->points.push_back(obstacle);
            }
        }
        obstacles->width = obstacles->points.size();
        obstacles->height = 1;
        this->obstacles_ = obstacles;
        // 障碍物判断是否为空
        if (obstacles->width != 0) {
            // 构造kdtree
            pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXY>>();
            kdtree->setInputCloud(this->obstacles_);
            this->kdtree_ = kdtree;
        }
        
    }

    // 构造函数
    KDTree(const std::vector<Position> &points) {
        pcl::PointCloud<pcl::PointXY>::Ptr obstacles (new pcl::PointCloud<pcl::PointXY>());
        for (size_t i = 0; i < points.size(); i++) {
            pcl::PointXY obstacle = {static_cast<float>(points[i].x_), static_cast<float>(points[i].y_)};
            obstacles->points.push_back(obstacle);
        }
        obstacles->width = obstacles->points.size();
        obstacles->height = 1;
        this->obstacles_ = obstacles;
        // 障碍物判断是否为空
        if (obstacles->width != 0) {
            // 构造kdtree
            pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXY>>();
            kdtree->setInputCloud(this->obstacles_);
            this->kdtree_ = kdtree;
        }
    }

    // 构造函数
    KDTree() {}

    // 析构函数
    ~KDTree() {};

    // 搜索k最近邻
    int findKNeighbor(float x, float y, std::vector<std::pair<float, float>> *results, std::vector<float> *sq_distances, int k = 10) const {
        // 判断kdtree是否为空
        if (this->kdtree_ == nullptr) {
            return -1;
        }
        std::vector<std::pair<float, float>> points;
        // 构造搜索点
        pcl::PointXY search_point = {x, y};
        // 进行搜索
        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);
        if (this->kdtree_->nearestKSearch(search_point, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            for (size_t i = 0; i < pointIdxNKNSearch.size (); i++) {
                points.push_back(std::make_pair(this->obstacles_->points[pointIdxNKNSearch[i]].x, this->obstacles_->points[pointIdxNKNSearch[i]].y));
            }
            *results = points;
            *sq_distances = pointNKNSquaredDistance;
            return 0;
        } else {
            return -1;
        }
    }

    // 搜索范围近邻
    int findRangeNeighbor(float x, float y, std::vector<std::pair<float, float>> *results, std::vector<float> *sq_distances, double range) const {
        // 判断kdtree是否为空
        if (this->kdtree_ == nullptr) {
            return -1;
        }
        std::vector<std::pair<float, float>> points;
        // 构造搜索点
        pcl::PointXY search_point = {x, y};
        // 进行搜索
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if (this->kdtree_->radiusSearch (search_point, range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            for (size_t i = 0; i < pointIdxRadiusSearch.size (); i++) {
                points.push_back(std::make_pair(this->obstacles_->points[pointIdxRadiusSearch[i]].x, this->obstacles_->points[pointIdxRadiusSearch[i]].y));
            }
            *results = points;
            *sq_distances = pointRadiusSquaredDistance;
            return 0;
        } else {
            return -1;
        }
    }
    
    // 判断kdtree是否为空
    bool isEmpty() const {
        if (this->kdtree_ == nullptr) {
            return true;
        } else {
            return false;
        }
    }

    // 得到障碍物
    pcl::PointCloud<pcl::PointXY> getObstacles() const {
        return *(this->obstacles_);
    }
 
 private:
    pcl::PointCloud<pcl::PointXY>::Ptr obstacles_ = nullptr;  // 障碍物信息
    pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree_ = nullptr;  // kdtree
};

#endif