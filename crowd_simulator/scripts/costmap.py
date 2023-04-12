#!/usr/bin/env python
#!-*- coding: utf-8 -*-

"""

Costmap
Author: flztiii
2021/3/8

"""

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np
import scipy.spatial
from collections import namedtuple
import rospy

# 点类
Point2f = namedtuple("Point2f", ('x_', 'y_'))
CurvePoint = namedtuple("CurvePoint", ('position_', 'theta_', 'kappa_'))

# 栅格地图类
class GridMap:
    # 构造函数
    def __init__(self, **kwargs):
        if "occupancy_grid" in kwargs.keys():
            occupancy_grid = kwargs["occupancy_grid"]
            # 获取grid_map属性
            self.width_ = occupancy_grid.info.width
            self.height_ = occupancy_grid.info.height
            self.resolution_ = occupancy_grid.info.resolution
            self.root_x_ = occupancy_grid.info.origin.position.x
            self.root_y_ = occupancy_grid.info.origin.position.y
            self.root_theta_ = 2.0 * np.arctan2(occupancy_grid.info.origin.orientation.z, occupancy_grid.info.origin.orientation.w)
            self.data_ = list()
            for data in occupancy_grid.data:
                if data > 50:
                    self.data_.append(1)
                else:
                    self.data_.append(0)
        else:
            self.width_ = kwargs["width"]
            self.height_ = kwargs["height"]
            self.resolution_ = kwargs["resolution"]
            self.root_x_ = kwargs["root_x"]
            self.root_y_ = kwargs["root_y"]
            self.root_theta_ = kwargs["root_theta"]
            self.data_ = list()
            for pixel in kwargs["data"]:
                if pixel > 128:
                    self.data_.append(0)
                else:
                    self.data_.append(1)
    
    # 获取对应栅格是否被占据
    def isOccupied(self, index):
        return self.data_[index]

    # 修改对应栅格为被占据
    def setOccupied(self, index):
        self.data_[index] = 1

    # 求对应点的栅格下标
    def getIndex(self, x, y):
        return x + y * self.width_

    # 计算栅格坐标
    def getXY(self, index):
        x = index % self.width_
        y = int(index / self.width_)
        return x, y
    
    # 判断是否超出边界
    def isVerify(self, x, y):
        if x >= 0 and x < self.width_ and y >= 0 and y < self.height_:
            return True
        else:
            return False
    
    # 生成kd树
    def toKDtree(self):
        # 得到障碍物点
        obstacle_points = list()
        for i in range(0, self.width_ * self.height_):
            if self.isOccupied(i):
                x, y = self.getXY(i)
                px, py = self.getCartesianCoordinate(x, y)
                obstacle_points.append([px, py])
        # 判断障碍物点的数量
        if len(obstacle_points) == 0:
            return None
        else:
            obstacle_points = np.array(obstacle_points)
            return scipy.spatial.KDTree(obstacle_points)

    
    # 计算对应的栅格坐标
    def getGridMapCoordinate(self, px, py):
        curve_point = CurvePoint(Point2f(self.root_x_, self.root_y_), self.root_theta_, 0.0)
        point = Point2f(px, py)
        new_point = self.calcNewCoordinationPosition(curve_point, point)
        x = int(new_point.x_ / self.resolution_)
        y = int(new_point.y_ / self.resolution_)
        assert(self.isVerify(x, y))
        return x, y
    
    # 计算真实坐标
    def getCartesianCoordinate(self, x, y):
        assert(self.isVerify(x, y))
        curve_point = CurvePoint(Point2f(self.root_x_, self.root_y_), self.root_theta_, 0.0)
        point = Point2f(x * self.resolution_, y * self.resolution_)
        raw_point = self.calcOldCoordinationPosition(curve_point, point)
        return raw_point.x_, raw_point.y_

    # 计算坐标系转换
    def calcNewCoordinationPosition(self, new_coordination_origin, position):
        new_position_x = (position.x_ - new_coordination_origin.position_.x_) * np.cos(new_coordination_origin.theta_) + (position.y_ - new_coordination_origin.position_.y_) * np.sin(new_coordination_origin.theta_)
        new_position_y = -(position.x_ - new_coordination_origin.position_.x_) * np.sin(new_coordination_origin.theta_) + (position.y_ - new_coordination_origin.position_.y_) * np.cos(new_coordination_origin.theta_)
        new_position = Point2f(new_position_x, new_position_y)
        return new_position

    # 计算坐标系反转换
    def calcOldCoordinationPosition(self, new_coordination_origin, position):
        old_position_x = new_coordination_origin.position_.x_ + position.x_ *  np.cos(new_coordination_origin.theta_) - position.y_ *  np.sin(new_coordination_origin.theta_)
        old_position_y = new_coordination_origin.position_.y_ + position.x_ *  np.sin(new_coordination_origin.theta_) + position.y_ *  np.cos(new_coordination_origin.theta_)
        old_position = Point2f(old_position_x, old_position_y)
        return old_position

    # 转化成ros消息
    def toRosMessage(self, frame_id="odom"):
        occupancy_grid = OccupancyGrid()
        pose = Pose()
        pose.position.x = self.root_x_
        pose.position.y = self.root_y_
        pose.orientation.z = np.sin(self.root_theta_ * 0.5)
        pose.orientation.w = np.cos(self.root_theta_ * 0.5)
        out_data = []
        for meta_data in self.data_:
            if meta_data:
                out_data.append(100)
            else:
                out_data.append(0)
        occupancy_grid.data = out_data
        occupancy_grid.info.resolution = self.resolution_
        occupancy_grid.info.width = self.width_
        occupancy_grid.info.height = self.height_
        occupancy_grid.info.origin = pose
        occupancy_grid.info.map_load_time = rospy.Time.now()
        occupancy_grid.header.frame_id = frame_id
        occupancy_grid.header.stamp = rospy.Time.now()
        return occupancy_grid
