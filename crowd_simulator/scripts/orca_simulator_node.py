#! /usr/bin/env python
#! -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from obstacle_msgs.msg import CircleObstacle, CircleObstacles
from crowd_simulator.srv import PrepareORCASimulation, GetObstacles, PrepareORCASimulationRequest, PrepareORCASimulationResponse, GetObstaclesRequest, GetObstaclesResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import numpy as np
from numpy.linalg import norm
from human import Human
from robot import Robot
import configparser
import os
import math
import random

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

class ORCASimulator:
    def __init__(self) -> None:
        # 初始化ros节点
        rospy.init_node("orca_simulator_node")
        
        # 定义属性
        self.robot = None
        self.humans = None
        self.time_step = None
        self.discomfort_dist = None
        self.config = None
        self.case_capacity = None
        self.case_size = None
        self.case_counter = None
        self.randomize_attributes = None
        self.train_val_sim = None
        self.test_sim = None
        self.square_width = None
        self.circle_radius = None
        self.dynamic_human_num = None
        self.static_human_num = None

        # 加载配置
        self.configure()

        # 初始化ros相关
        # 信息发布节点
        self.crowd_vis_pub_ = rospy.Publisher("/crowd_simulator/crowd_vis", MarkerArray, queue_size=10)
        # 准备模拟服务
        self.prepare_simulation_server_ = rospy.Service("/crowd_simulator/prepare_orca_simulation", PrepareORCASimulation, self.prepareSimulation)
        # 启动模型服务
        self.get_simulation_server_ = rospy.Service("/crowd_simulator/get_orca_simulation", GetObstacles, self.getSimulation)
        # 结束模型服务
        self.end_simulation_server_ = rospy.Service("/crowd_simulator/end_orca_simulation", Empty, self.endSimulation)
        
        # 设置随机种子
        self.setRandomSeeds(42)

    def setRandomSeeds(self, random_seed: int):
        """
        Setup all possible random seeds so results can be reproduced
        """
        os.environ["PYTHONHASHSEED"] = str(random_seed)
        random.seed(random_seed)
        np.random.seed(random_seed)

    def prepareSimulation(self, req: PrepareORCASimulationRequest):
        ob = self.reset(req.robot_pose.position.x, req.robot_pose.position.y, 2.0 * np.arctan2(req.robot_pose.orientation.z, req.robot_pose.orientation.w), req.target.x, req.target.y, req.phase)
        res = PrepareORCASimulationResponse()
        res.obstacles = self.obToMsg(ob)
        return res

    def getSimulation(self, req: GetObstaclesRequest):
        ob = self.step()
        res = GetObstaclesResponse()
        res.obstacles = self.obToMsg(ob)
        return res

    def endSimulation(self, req: EmptyRequest):
        res = EmptyResponse()
        # 删除之前的可视化
        del_marker_array = MarkerArray()
        del_marker = Marker()
        del_marker.header.frame_id = "odom"
        del_marker.header.stamp = rospy.Time.now()
        del_marker.action = Marker().DELETEALL
        del_marker.id = 0
        del_marker_array.markers.append(del_marker)
        self.crowd_vis_pub_.publish(del_marker_array)
        return res

    def obToMsg(self, ob):
        circle_obstacles = CircleObstacles()
        circle_obstacles.header.stamp = rospy.Time.now()
        circle_obstacles.header.frame_id = "odom"
        for i, human_state in enumerate(ob):
            circle_obstacle = CircleObstacle()
            circle_obstacle.id = i
            circle_obstacle.radius = human_state.radius
            circle_obstacle.point.x = human_state.px
            circle_obstacle.point.y = human_state.py
            circle_obstacle.twist.linear.x = human_state.vx
            circle_obstacle.twist.linear.y = human_state.vy
            circle_obstacles.obstacles.append(circle_obstacle)
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
        for obstacle_info in circle_obstacles.obstacles:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.action = Marker().ADD
            marker.type = Marker().CYLINDER
            marker.id = int(obstacle_info.id) * 2
            marker.pose.position.x = obstacle_info.point.x
            marker.pose.position.y = obstacle_info.point.y
            marker.scale.x = obstacle_info.radius * 2.0
            marker.scale.y = obstacle_info.radius * 2.0
            marker.scale.z = 1.75
            color = hsv2rgb(marker.id % 30 * 36, 1, 1)
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.3
            crowd_marker_array.markers.append(marker)
            vel_marker = Marker()
            vel_marker.header.frame_id = "odom"
            vel_marker.header.stamp = rospy.Time.now()
            vel_marker.action = Marker().ADD
            vel_marker.type = Marker().ARROW
            vel_marker.id = int(obstacle_info.id) * 2 + 1
            vel_marker.pose.position.x = obstacle_info.point.x
            vel_marker.pose.position.y = obstacle_info.point.y
            vel_marker.pose.orientation.z = np.sin(0.5 * np.arctan2(obstacle_info.twist.linear.y, obstacle_info.twist.linear.x))
            vel_marker.pose.orientation.w = np.cos(0.5 * np.arctan2(obstacle_info.twist.linear.y, obstacle_info.twist.linear.x))
            vel_marker.scale.x = np.sqrt(obstacle_info.twist.linear.x ** 2 + obstacle_info.twist.linear.y ** 2)
            vel_marker.scale.y = 0.05
            vel_marker.scale.z = 0.05
            vel_marker.color.r = 1.0
            vel_marker.color.g = 0.0
            vel_marker.color.b = 0.0
            vel_marker.color.a = 1.0
            crowd_marker_array.markers.append(vel_marker)
        self.crowd_vis_pub_.publish(crowd_marker_array)
        return circle_obstacles
    
    def configure(self):
        self.config = configparser.RawConfigParser()
        self.config.read(os.path.join(os.path.dirname(__file__), "../config/env.config"))
        self.robot = Robot(self.config, "robot")
        self.time_step = self.config.getfloat("env", "time_step")
        self.randomize_attributes = self.config.getboolean("env", "randomize_attributes")
        self.discomfort_dist = self.config.getfloat("sim", "discomfort_dist")
        self.case_capacity = {
            "train": 0,
            "val": 0,
            "test": 0,
        }
        self.case_size = {
            "train": np.iinfo(np.uint32).max - 2000,
            "val": self.config.getint("env", "val_size"),
            "test": self.config.getint("env", "test_size"),
        }
        self.test_sim = self.config.get("sim", "test_sim")
        self.train_val_sim = self.config.get("sim", "train_val_sim")
        self.square_width = self.config.getfloat("sim", "square_width")
        self.circle_radius = self.config.getfloat("sim", "circle_radius")
        self.dynamic_human_num = self.config.getint("sim", "human_num")
        self.static_human_num = self.config.getint("sim", "static_human_num")
        self.case_counter = {"train": 0, "test": 0, "val": 0}
        self.multiagent_training = True

    def reset(self, px, py, ptheta, gx, gy, phase="test", test_case=None):
        assert phase in ["train", "val", "test"]
        if test_case is not None:
            self.case_counter[phase] = test_case
        if not self.multiagent_training:
            self.train_val_sim = "circle_crossing"
        counter_offset = {
            "train": self.case_capacity["val"] + self.case_capacity["test"],
            "val": 0,
            "test": self.case_capacity["val"],
        }
        self.robot.set(px, py, gx, gy, 0, 0, ptheta)
        if self.case_counter[phase] >= 0:
            if phase != "train":
                np.random.seed(counter_offset[phase] + self.case_counter[phase])
            if phase in ["train", "val"]:
                human_num = self.dynamic_human_num if self.multiagent_training else 1
                self.generate_random_human_position(human_num=human_num, rule=self.train_val_sim)
            else:
                self.generate_random_human_position(human_num=self.dynamic_human_num, rule=self.test_sim)
            self.add_static_humans(self.static_human_num, self.humans)
            # case_counter is always between 0 and case_size[phase]
            self.case_counter[phase] = (self.case_counter[phase] + 1) % self.case_size[phase]
        else:
            assert phase == "test"
            if self.case_counter[phase] == -1:
                # for debugging purposes
                self.dynamic_human_num = 3
                self.humans = [Human(self.config, "humans") for _ in range(self.dynamic_human_num)]
                self.humans[0].set(0, -6, 0, 5, 0, 0, np.pi / 2)
                self.humans[1].set(-5, -5, -5, 5, 0, 0, np.pi / 2)
                self.humans[2].set(5, -5, 5, 5, 0, 0, np.pi / 2)
            else:
                raise NotImplementedError
        for agent in [self.robot] + self.humans:
            agent.time_step = self.time_step
            if agent.policy is not None:
                agent.policy.time_step = self.time_step
        
        ob = [human.get_observable_state() for human in self.humans]
        return ob

    def step(self):
        human_actions = []
        for human in self.humans:
            # observation for humans is always coordinates
            ob = [other_human.get_observable_state() for other_human in self.humans if other_human != human]
            # 计算出human的下一步动作
            human_actions.append(human.act(ob))
        for i, human_action in enumerate(human_actions):
            # only dynamic humans can move
            if i < self.dynamic_human_num:
                self.humans[i].step(human_action)
        for i in range(self.dynamic_human_num):
            human = self.humans[i]
        ob = [human.get_observable_state() for human in self.humans]
        return ob
    
    def generate_random_human_position(self, human_num, rule):
        """
        Generate human position according to certain rule
        Rule square_crossing: generate start/goal position at two sides of y-axis
        Rule circle_crossing: generate start position on a circle, goal position is at the opposite side

        :param human_num:
        :param rule:
        :return:
        """
        # initial min separation distance to avoid danger penalty at beginning
        if rule == "square_crossing":
            self.humans = []
            for i in range(human_num):
                self.humans.append(self.generate_square_crossing_human())
        elif rule == "circle_crossing":
            self.humans = []
            for i in range(human_num):
                self.humans.append(self.generate_circle_crossing_human())
        else:
            raise ValueError("Rule doesn't exist")

    def generate_circle_crossing_human(self):
        human = Human(self.config, "humans")
        if self.randomize_attributes:
            human.sample_random_attributes()
        while True:
            angle = np.random.random() * np.pi * 2
            # add some noise to simulate all the possible cases robot could meet with human
            px_noise = (np.random.random() - 0.5) * human.v_pref
            py_noise = (np.random.random() - 0.5) * human.v_pref
            px = self.circle_radius * np.cos(angle) + px_noise
            py = self.circle_radius * np.sin(angle) + py_noise
            collide = False
            for agent in [self.robot] + self.humans:
                min_dist = human.radius + agent.radius + self.discomfort_dist
                if norm((px - agent.px, py - agent.py)) < min_dist or norm((px - agent.gx, py - agent.gy)) < min_dist:
                    collide = True
                    break
            if not collide:
                break
        human.set(px, py, -px, -py, 0, 0, 0)
        return human

    def generate_square_crossing_human(self):
        human = Human(self.config, "humans")
        if self.randomize_attributes:
            human.sample_random_attributes()
        if np.random.random() > 0.5:
            sign = -1
        else:
            sign = 1
        while True:
            px = np.random.random() * self.square_width * 0.5 * sign
            py = (np.random.random() - 0.5) * self.square_width
            collide = False
            for agent in [self.robot] + self.humans:
                if norm((px - agent.px, py - agent.py)) < human.radius + agent.radius + self.discomfort_dist:
                    collide = True
                    break
            if not collide:
                break
        while True:
            gx = np.random.random() * self.square_width * 0.5 * -sign
            gy = (np.random.random() - 0.5) * self.square_width
            collide = False
            for agent in [self.robot] + self.humans:
                if norm((gx - agent.gx, gy - agent.gy)) < human.radius + agent.radius + self.discomfort_dist:
                    collide = True
                    break
            if not collide:
                break
        human.set(px, py, gx, gy, 0, 0, 0)
        return human
    
    def add_static_humans(self, num, humans):
        # randomly initialize static objects in a square of (width, height)
        width = 4
        height = 8
        for i in range(num):
            human = Human(self.config, "humans")
            if np.random.random() > 0.5:
                sign = -1
            else:
                sign = 1
            while True:
                px = np.random.random() * width * 0.5 * sign
                py = (np.random.random() - 0.5) * height
                collide = False
                # 检测是否会和其他human距离过小
                for agent in [self.robot] + humans:
                    if norm((px - agent.px, py - agent.py)) < human.radius + agent.radius + self.discomfort_dist:
                        collide = True
                        break
                if not collide:
                    break
            human.set(px, py, px, py, 0, 0, 0)
            humans.append(human)

if __name__ == "__main__":
    orca_simulator = ORCASimulator()
    rospy.spin()