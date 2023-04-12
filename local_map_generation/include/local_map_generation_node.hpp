/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef LOCAL_MAP_GENERATION_NODE_HPP_
#define LOCAL_MAP_GENERATION_NODE_HPP_

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <obstacle_msgs/CircleObstacle.h>
#include <obstacle_msgs/CircleObstacles.h>
#include <local_map_generation/GetLocalMap.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <cmath>
#include <iostream>
#include "grid_map.hpp"
#include "tools.hpp"

class LocalMapGeneration {
 public:
    // 构造函数
    LocalMapGeneration() {
        // 进行ros相关初始化
        ros::NodeHandle nh("~");
        // 局部地图发布
        std::string local_map_pub_topic;
        nh.getParam("local_map_pub_topic", local_map_pub_topic);
        this->local_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(local_map_pub_topic, 10);
        // 获取局部地图服务
        std::string local_map_service_name;
        nh.getParam("local_map_service_name", local_map_service_name);
        this->local_map_service_ = nh.advertiseService(local_map_service_name, &LocalMapGeneration::getLocalMap, this);
        // 获取常量
        nh.getParam("map_width", this->map_width_);
        nh.getParam("map_height", this->map_height_);
        nh.getParam("map_resolution", this->map_resolution_);
    };

    // 析构函数
    ~LocalMapGeneration() {};

 private:

    // 服务
    bool getLocalMap(local_map_generation::GetLocalMap::Request &request, local_map_generation::GetLocalMap::Response &response) {
        CurvePoint current_position;
        current_position.position_.x_ = request.pose.position.x;
        current_position.position_.y_ = request.pose.position.y;
        current_position.theta_ = 2.0 * atan2(request.pose.orientation.z, request.pose.orientation.w);
        GridMap costmap = request.global_map;
        nav_msgs::OccupancyGrid local_map = this->generateLocalMap(current_position, costmap, request.obstacles);
        response.local_map = local_map;
        if (request.publish) {
            this->local_map_pub_.publish(local_map);
        }
        return true;
    }   

    // 生成代价地图
    nav_msgs::OccupancyGrid generateLocalMap(const CurvePoint &current_position, const GridMap &costmap, const obstacle_msgs::CircleObstacles &obstacles) {
        // 开始构建地图(base_footprint坐标系下)
        double root_x = - this->map_width_ * this->map_resolution_ * 0.5;
        double root_y = - this->map_height_ * this->map_resolution_ * 0.5;
        double root_theta = 0;
        GridMap grid_map = GridMap(this->map_width_, this->map_height_, this->map_resolution_, root_x, root_y, root_theta);
        // 加入全局地图
        if (costmap.getWidth() * costmap.getHeight() != 0) {
            for (int i = 0; i < grid_map.getWidth(); i++) {
                for (int j = 0; j < grid_map.getHeight(); j++) {
                    // 获取base_footprint坐标系下的点
                    Point2f cartesian_point = grid_map.getCartesianCoordinate(i, j);
                    // 转换到world坐标系
                    cartesian_point = calcOldCoordinationPosition(current_position, cartesian_point);
                    std::pair<int, int> costmap_point = costmap.getGridMapCoordinate(cartesian_point.x_, cartesian_point.y_);
                    if (costmap.isVerify(costmap_point.first, costmap_point.second)) {
                        if (costmap.isOccupied(costmap.getIndex(costmap_point.first, costmap_point.second))) {
                            grid_map.setOccupied(grid_map.getIndex(i, j));
                        }
                    } else {
                        grid_map.setOccupied(grid_map.getIndex(i, j));
                    }
                }
            }
        }
        // 使用障碍物信息
        // 将障碍物转化为栅格坐标系下的圆形
        std::vector<Circle> circles;
        for (auto obstacle: obstacles.obstacles) {
            Point2f base_point = calcNewCoordinationPosition(current_position, Point2f{obstacle.point.x, obstacle.point.y});
            std::pair<int, int> gridmap_point = grid_map.getGridMapCoordinate(base_point.x_, base_point.y_);
            Circle circle;
            circle.center_ = Point2f{static_cast<double>(gridmap_point.first), static_cast<double>(gridmap_point.second)};
            circle.radius_ = obstacle.radius / grid_map.getResolution();
            circles.push_back(circle);
        }
        // 进行填充
        this->fillGridMap(grid_map, circles);
        // 进行格式转换和发布
        nav_msgs::OccupancyGrid local_map = grid_map.toRosMessage();
        return local_map;
    };
    
    // 填充栅格地图
    void fillGridMap(GridMap &grid_map, const std::vector<Circle> &circles) {
        // 根据栅格构建对应图像
        cv::Mat grid_image(cv::Size(grid_map.getWidth(), grid_map.getHeight()), CV_8UC1, cv::Scalar(0));
        // 填充矩形覆盖区域
        for (auto circle: circles) {
            cv::Point center(round(circle.center_.x_), round(circle.center_.y_));
            cv::circle(grid_image, center, std::ceil(circle.radius_), cv::Scalar(255), -1);
        }
        // 图像翻转
        cv::Mat grid_image_fliped;
	    cv::flip(grid_image, grid_image_fliped, 0);
        // 图像叠加
        cv::Mat final_image;
        cv::add(grid_image_fliped, grid_map.toImage(), final_image);
        grid_map.loadImage(final_image);
    }

    ros::Publisher local_map_pub_;  // 代价地图发布
    ros::ServiceServer local_map_service_;
    int map_width_;  // 局部地图尺寸（分辨率）
    int map_height_;  // 局部地图尺寸（分辨率）
    double map_resolution_;  // 局部地图分辨率
};
#endif // Local_MAP_GENERATION_NODE_HPP_
