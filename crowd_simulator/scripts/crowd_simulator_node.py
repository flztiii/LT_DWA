#! /usr/bin/env python
#! -*- coding: utf-8 -*-

import rospy
import roslib
import os
import math
import ujson
from visualization_msgs.msg import Marker, MarkerArray
from obstacle_msgs.msg import CircleObstacle, CircleObstacles
from crowd_simulator.srv import PrepareCrowdSimulation, PrepareCrowdSimulationRequest, PrepareCrowdSimulationResponse, GetObstacles, GetObstaclesResponse

# 人的半径
HUMAN_RADIUS = 0.3

# hsv转rgb
def hsv2rgb(h, s, v):
    h = float(h)
    s = float(s)
    v = float(v)
    h60 = h / 60.0
    h60f = math.floor(h60)
    hi = int(h60f) % 6
    f = h60 - h60f
    p = v * (1 - s)
    q = v * (1 - f * s)
    t = v * (1 - (1 - f) * s)
    r, g, b = 0, 0, 0
    if hi == 0: r, g, b = v, t, p
    elif hi == 1: r, g, b = q, v, p
    elif hi == 2: r, g, b = p, v, t
    elif hi == 3: r, g, b = p, q, v
    elif hi == 4: r, g, b = t, p, v
    elif hi == 5: r, g, b = v, p, q
    return r, g, b

def copyList(origin_list: list):
    l = ujson.loads(ujson.dumps(origin_list))
    return l

# 人群模拟类
class CrowdSimulator:
    def __init__(self) -> None:
        # 初始化ros节点
        rospy.init_node("crowd_simulator_node")
        # 信息发布节点
        self.crowd_vis_pub_ = rospy.Publisher("/crowd_simulator/crowd_vis", MarkerArray, queue_size=10, latch=True)
        # 准备模拟服务
        self.prepare_simulation_server_ = rospy.Service("/crowd_simulator/prepare_crowd_simulation", PrepareCrowdSimulation, self.prepareSimulation)
        # 启动模型服务
        self.get_simulation_server_ = rospy.Service("/crowd_simulator/get_crowd_simulation", GetObstacles, self.getSimulation)
        # 常量
        self.min_simulation_time_ = rospy.get_param("~min_simulation_time", 60.0)
        print("min_simulation_time:", self.min_simulation_time_)
        # 初始化仿真数据
        self.simulation_data_ = list()
    
    # 准备进行模拟
    def prepareSimulation(self, req: PrepareCrowdSimulationRequest):
        # 初始化仿真数据
        self.clearSimulation()
        res = PrepareCrowdSimulationResponse()
        # 得到文件路径
        file_path = roslib.packages.get_pkg_dir("crowd_simulator") + "/data/" + req.crowd_name + ".txt"
        if not os.path.exists(file_path):
            # 返回错误信息
            print("INVALID_CROWD_NAME")
            res.result = PrepareCrowdSimulationResponse().INVALID_CROWD_NAME
        else:
            # 进行文件的解析
            crowd_file = open(file_path, "r")
            crowd_file_data = crowd_file.readlines()
            for line in crowd_file_data:
                line = line.strip().split(",")
                assert(len(line) == 6)
                if float(line[0]) >= req.start_stamp:
                    self.simulation_data_.append([float(line[0]), int(line[1]), float(line[2]), float(line[3]), float(line[4]), float(line[5])])
            crowd_file.close()
            # 判断仿真数据是否满足时长
            if len(self.simulation_data_) <= 1:
                # 不满足时长
                print("INVALID_START_STAMP")
                res.result = PrepareCrowdSimulationResponse().INVALID_START_STAMP
                self.simulation_data_ = list()
                self.frequency_ = 0.0
            elif self.simulation_data_[-1][0] - self.simulation_data_[0][0] <= self.min_simulation_time_:
                # 不满足时长
                print("INVALID_START_STAMP")
                res.result = PrepareCrowdSimulationResponse().INVALID_START_STAMP
                self.simulation_data_ = list()
                self.frequency_ = 0.0
            else:
                # 完成准备
                print("simulation prepared")
                res.result = PrepareCrowdSimulationResponse().READY
                # 得到初始帧障碍物信息
                res.start_obstacles.header.frame_id = "odom"
                res.start_obstacles.header.stamp = rospy.Time.now()
                start_frame_stamp = self.simulation_data_[0][0]
                next_frame_stamp = start_frame_stamp
                for simulation_data in self.simulation_data_:
                    if simulation_data[0] == start_frame_stamp:
                        if simulation_data[1] != -1:
                            obstacle = CircleObstacle()
                            obstacle.id = simulation_data[1]
                            obstacle.radius = HUMAN_RADIUS
                            obstacle.point.x = simulation_data[2]
                            obstacle.point.y = simulation_data[3]
                            obstacle.twist.linear.x = simulation_data[4]
                            obstacle.twist.linear.y = simulation_data[5]
                            res.start_obstacles.obstacles.append(obstacle)
                    else:
                        next_frame_stamp = simulation_data[0]
                        break
                # 发布障碍物可视化
                crowd_marker_array, del_marker_array = MarkerArray(), MarkerArray()
                # 删除之前的可视化
                del_marker = Marker()
                del_marker.header.frame_id = "odom"
                del_marker.header.stamp = rospy.Time.now()
                del_marker.action = Marker().DELETEALL
                del_marker.id = 0
                del_marker_array.markers.append(del_marker)
                self.crowd_vis_pub_.publish(del_marker_array)
                # 进行可视化发布
                for obstacle_info in res.start_obstacles.obstacles:
                    marker = Marker()
                    marker.header.frame_id = "odom"
                    marker.header.stamp = rospy.Time.now()
                    marker.action = Marker().ADD
                    marker.type = Marker().CYLINDER
                    marker.id = int(obstacle_info.id)
                    marker.pose.position.x = obstacle_info.point.x
                    marker.pose.position.y = obstacle_info.point.y
                    marker.scale.x = HUMAN_RADIUS * 2.0
                    marker.scale.y = HUMAN_RADIUS * 2.0
                    marker.scale.z = 1.75
                    color = hsv2rgb(marker.id % 30 * 36, 1, 1)
                    marker.color.r = color[0]
                    marker.color.g = color[1]
                    marker.color.b = color[2]
                    marker.color.a = 0.3
                    crowd_marker_array.markers.append(marker)
                self.crowd_vis_pub_.publish(crowd_marker_array)
                # 计算帧率
                time_gap = next_frame_stamp - start_frame_stamp
                res.time_gap = time_gap
                print("time_gap: ", time_gap)
                self.frequency_ = round(1.0 / time_gap, 2)
        return res

    # 进行模拟
    def getSimulation(self, req):
        # 得到当前障碍物列表
        current_crowd = list()
        if len(self.simulation_data_) == 0:
            self.clearSimulation()
        else:
        # 得到当前帧
            current_stamp = self.simulation_data_[0][0]
            while len(self.simulation_data_) > 0:
                if self.simulation_data_[0][0] == current_stamp:
                    current_data = self.simulation_data_.pop(0)
                    if current_data[1] != -1:
                        current_crowd.append(current_data)
                else:
                    break
        # 构建障碍物
        obstacles_msg = CircleObstacles()
        obstacles_msg.header.frame_id = "odom"
        obstacles_msg.header.stamp = rospy.Time.now()
        for obstacle_info in current_crowd:
            obstacle = CircleObstacle()
            obstacle.id = int(obstacle_info[1])
            obstacle.radius = HUMAN_RADIUS
            obstacle.point.x = obstacle_info[2]
            obstacle.point.y = obstacle_info[3]
            obstacle.twist.linear.x = obstacle_info[4]
            obstacle.twist.linear.y = obstacle_info[5]
            obstacles_msg.obstacles.append(obstacle)
        # 发布障碍物可视化
        crowd_marker_array, del_marker_array = MarkerArray(), MarkerArray()
        # 删除之前的可视化
        del_marker = Marker()
        del_marker.header.frame_id = "odom"
        del_marker.header.stamp = rospy.Time.now()
        del_marker.action = Marker().DELETEALL
        del_marker.id = 0
        del_marker_array.markers.append(del_marker)
        self.crowd_vis_pub_.publish(del_marker_array)
        # 进行可视化发布
        for obstacle_info in current_crowd:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.action = Marker().ADD
            marker.type = Marker().CYLINDER
            marker.id = int(obstacle_info[1])
            marker.pose.position.x = obstacle_info[2]
            marker.pose.position.y = obstacle_info[3]
            marker.scale.x = HUMAN_RADIUS * 2.0
            marker.scale.y = HUMAN_RADIUS * 2.0
            marker.scale.z = 1.75
            color = hsv2rgb(marker.id % 30 * 36, 1, 1)
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.3
            crowd_marker_array.markers.append(marker)
        self.crowd_vis_pub_.publish(crowd_marker_array)
        # 返回
        res = GetObstaclesResponse()
        res.obstacles = obstacles_msg
        return res

    # 清除模拟
    def clearSimulation(self) -> None:
        del_marker_array = MarkerArray()
        # 删除之前的可视化
        del_marker = Marker()
        del_marker.header.frame_id = "odom"
        del_marker.header.stamp = rospy.Time.now()
        del_marker.action = Marker().DELETEALL
        del_marker.id = 0
        del_marker_array.markers.append(del_marker)
        self.crowd_vis_pub_.publish(del_marker_array)
        # 修改数据
        self.simulation_data_ = list()
        print("end simulation")


# 主函数
def main():
    crowd_simulator = CrowdSimulator()
    rospy.spin()

if __name__ == "__main__":
    main()
