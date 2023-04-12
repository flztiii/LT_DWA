#!/usr/bin/env python
#! -*- coding: utf-8 -*-

import rospy
from crowd_simulator.srv import PrepareORCASimulation, GetObstacles
from geometry_msgs.msg import Pose, Point
import numpy as np

if __name__ == "__main__":
    # 初始化ros
    rospy.init_node("test_orca_simulator_node")
    # 订阅服务
    prepare_service_name = "/crowd_simulator/prepare_orca_simulation"
    rospy.wait_for_service(prepare_service_name)
    prepare_service = rospy.ServiceProxy(prepare_service_name, PrepareORCASimulation)
    get_service_name = "/crowd_simulator/get_orca_simulation"
    rospy.wait_for_service(get_service_name)
    get_service = rospy.ServiceProxy(get_service_name, GetObstacles)
    # 开始测试
    robot_pose = Pose()
    robot_pose.position.y = -5
    robot_pose.orientation.z = np.sin(np.pi / 4)
    robot_pose.orientation.w = np.cos(np.pi / 4)
    target = Point()
    target.y = 5
    phase = "train"
    try:
        prepare_service.call(robot_pose, target, phase)
    except rospy.ServiceException:
        print("prepare failed")
        exit(0)
    while not rospy.is_shutdown():
        input("input to step")
        try:
            get_service.call()
        except rospy.ServiceException:
            print("get failed")
            exit(0)