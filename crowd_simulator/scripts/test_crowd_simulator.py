#!/usr/bin/env python
#! -*- coding: utf-8 -*-

import rospy
import roslib
import yaml
import cv2
from crowd_simulator.srv import PrepareCrowdSimulation, GetObstacles, PrepareCrowdSimulationResponse, PrepareCrowdSimulationRequest
import random
from costmap import GridMap

def loadScenarios():
        # 得到根目录
        root = roslib.packages.get_pkg_dir("crowd_simulator")
        # 准备解析场景信息
        scenarios_info_file = open(root + "/info/info.yaml", "r")
        scenarios_info_yaml = yaml.safe_load(scenarios_info_file)
        scenarios_info_file.close()
        scenario_names = scenarios_info_yaml["scenario_names"]
        time_durations = dict()
        for index in range(0, len(scenario_names)):
            time_durations[scenario_names[index]] = scenarios_info_yaml["time_durations"][index]
        # 加载全局地图
        scenario_maps = dict()
        for scenario_name in scenario_names:
            map_yaml_file = open(root + "/map/" + scenario_name + ".yaml", "r")
            yaml_data = yaml.safe_load(map_yaml_file)
            map_yaml_file.close()
            # 加载图片
            map_image = cv2.imread(root + "/map/" + yaml_data["image"], 0)
            cv2.flip(map_image, 0, map_image)
            height, width = map_image.shape
            map_image = map_image.flatten().tolist()
            # 构建全局地图
            global_costmap = GridMap(width=width, height=height, resolution=yaml_data["resolution"], root_x=yaml_data["origin"][0], root_y=yaml_data["origin"][1], root_theta=yaml_data["origin"][2], data=map_image)
            global_kdtree = global_costmap.toKDtree()
            scenario_maps[scenario_name] = (global_costmap, global_kdtree)
        return scenario_maps, scenario_names, time_durations

if __name__ == "__main__":
    # 初始化ros
    rospy.init_node("test_simulator_node")
    # 准备人群仿真服务
    prepare_simulation_service_name = "/crowd_simulator/prepare_crowd_simulation"
    rospy.wait_for_service(prepare_simulation_service_name)
    prepare_simulation_service = rospy.ServiceProxy(prepare_simulation_service_name, PrepareCrowdSimulation)
    # 障碍物服务
    obstacles_service_name = "/crowd_simulator/get_crowd_simulation"
    rospy.wait_for_service(obstacles_service_name)
    obstacles_service = rospy.ServiceProxy(obstacles_service_name, GetObstacles)

    # 进行仿真准备服务
    scenario_maps, scenario_names, time_durations = loadScenarios()
    currrent_scenario = random.choice(scenario_names)
    prepare_simulation_res = PrepareCrowdSimulationResponse()
    while not rospy.is_shutdown():
        prepare_simulation_req = PrepareCrowdSimulationRequest()
        prepare_simulation_req.crowd_name = currrent_scenario
        prepare_simulation_req.start_stamp = random.uniform(time_durations[currrent_scenario][0], time_durations[currrent_scenario][1])
        try:
            prepare_simulation_res = prepare_simulation_service.call(prepare_simulation_req)
            print("prepare simulation")
        except rospy.ServiceException:
            print("prepare simulation service failed")
            exit(0)
        # 判断仿真准备的结果
        if prepare_simulation_res.result == PrepareCrowdSimulationResponse().READY:
            # 仿真准备完成
            break
    
    # 获取障碍物信息
    while not rospy.is_shutdown():
        input()
        try:
            obstacles_service.call()
        except rospy.ServiceException:
            print("get obstacles failed")
            exit(0)