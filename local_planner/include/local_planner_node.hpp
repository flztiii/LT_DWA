/*
    Copyright [2022] Jian ZhiQiang
*/

#ifndef LOCAL_PLANNER_NODE_HPP_
#define LOCAL_PLANNER_NODE_HPP_

#include <map>
#include <vector>
#include <thread>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <crowd_simulator/PrepareORCASimulation.h>
#include <crowd_simulator/PrepareCrowdSimulation.h>
#include <crowd_simulator/GetObstacles.h>
#include <std_srvs/Empty.h>
#include <local_map_generation/GetLocalMap.h>
#include <obstacle_msgs/CircleObstacles.h>
#include <obstacle_msgs/CircleObstacle.h>
#include <navigation/AstarNavigation.h>
#include "util/robot.hpp"
#include "util/grid_map.hpp"
#include "util/kdtree.hpp"
#include "policy/seed_policy.hpp"

class LocalPlannerNode {
 public:
    LocalPlannerNode() {
        // 获取ros句柄
        this->nh_ = ros::NodeHandle("~");
        // 加载参赛
        this->loadParams();
        std::cout << "parameters loaded" << std::endl;
        // 初始化ros相关消息服务
        this->initRosConnection();
        std::cout << "ros initialized" << std::endl;
        // 开启ros线程
        std::thread ros_thread = std::thread(&LocalPlannerNode::rosConnector, this);
        ros_thread.detach();
        std::cout << "ros connected" << std::endl;
        // 初始化机器人
        this->robot_ptr_ = std::make_shared<Robot>(this->max_v_, this->min_v_, this->max_w_, this->max_acc_, this->max_angular_acc_);
        // 初始化策略
        if (this->policy_name_ == "seed") {
            this->policy_ptr_ = std::make_shared<SeedPolicy>(this->max_v_, this->min_v_, this->max_w_, this->max_acc_, this->max_angular_acc_, this->radius_, this->time_step_, this->local_map_service_);
        } else {
            std::cerr << "not implemented policy" << std::endl;
            exit(0);
        }
        std::cout << "robot policy is " << this->policy_name_ << std::endl;
    };

    ~LocalPlannerNode() {};

    // 进行一次规划
    int planOrcaOnce(int test_count=0) {
        // 进行环境的初始化
        std::cout << "start orca environment initialize" << std::endl;
        // 初始化空白地图
        double resolution = 0.05;
        double global_map_size = this->circle_radius_ * 1.4;
        GridMap global_map = GridMap(static_cast<int>(global_map_size * 2.0 / resolution), static_cast<int>(global_map_size * 2.0 / resolution), resolution, -global_map_size, -global_map_size, 0.0);
        this->static_map_pub_.publish(global_map.toRosMessage("odom"));
        // 进行初始状态的生成
        Pose init_pose = {0.0, -this->circle_radius_, M_PI * 0.5};
        Action init_action = {0.0, 0.0};
        // 设置终点位置
        Pose target_pose = {0.0, this->circle_radius_, 0.0};
        // 可视化终点
        this->visualizeTarget(target_pose);
        // 进行机器人设置
        this->robot_ptr_->init(init_pose, init_action);
        // 可视化机器人
        this->visualizeRobot(this->robot_ptr_);
        // 准备ORCA人群模拟
        crowd_simulator::PrepareORCASimulationRequest prepare_orca_simulation_req;
        crowd_simulator::PrepareORCASimulationResponse prepare_orca_simulation_res;
        prepare_orca_simulation_req.phase = "test";
        prepare_orca_simulation_req.robot_pose.position.x = this->robot_ptr_->getPose().x_;
        prepare_orca_simulation_req.robot_pose.position.y = this->robot_ptr_->getPose().y_;
        prepare_orca_simulation_req.target.x = target_pose.x_;
        prepare_orca_simulation_req.target.y = target_pose.y_;
        if(!this->prepare_simulation_service_.call(prepare_orca_simulation_req, prepare_orca_simulation_res)) {
            std::cerr << "call prepare orca simulation service failed" << std::endl;
        }
        // 获取信息
        obstacle_msgs::CircleObstacles obstacles_msg;
        std::map<int, Tools::FixedQueue<ObstacleInfo, OBSTACLE_INFO_LEN>> obstacles_info;
        GridMap gridmap;
        this->getInformation(this->robot_ptr_, obstacles_msg);
        this->processInfo(this->robot_ptr_, global_map, obstacles_msg, obstacles_info, gridmap);
        KDTree local_kdtree = KDTree(gridmap);
        // 生成导航路径
        std::vector<PathPose> navigation_path;
        this->generateNavigation(this->robot_ptr_, target_pose, global_map, navigation_path);
        // 进行机器人位置记录
        std::vector<Pose> robot_trajectory_recorder;
        robot_trajectory_recorder.push_back(this->robot_ptr_->getPose());
        // 进行轨迹可视化
        this->visualizeTrajectories(robot_trajectory_recorder, obstacles_info);
        std::cout << "planning test environment initialized" << std::endl;
        // 进行文件记录
        std::string record_data_file_path = ros::package::getPath("local_planner") + "/data/test_record_" + std::to_string(test_count) + ".csv";
        std::ofstream record_data_file(record_data_file_path, std::ios::out|std::ios::trunc);
        std::string vis_data_file_path = ros::package::getPath("local_planner") + "/vis/test_vis.csv";
        std::ofstream vis_data_file(vis_data_file_path, std::ios::out|std::ios::trunc);
        // 进行规划模拟
        int planning_result = -1;
        int current_step = 0;
        ros::Rate loop_rate(100);
        while (current_step < this->max_time_steps_) {
            // 获取行为
            Action action = {0.0, 0.0};
            auto start_planning_time = ros::WallTime::now();
            int forward_result = this->policy_ptr_->forward(*(this->robot_ptr_), target_pose, navigation_path, global_map, obstacles_info, action);
            if (forward_result < 0) {
                planning_result = 0;
                std::cerr << "policy failes to give an action" << std::endl;
                break;
            }
            auto end_planning_time = ros::WallTime::now();
            std::cout << "planning time consuming is " << (end_planning_time - start_planning_time).toNSec() / 1e6 << "ms" << std::endl;
            // getchar();
            // 更新机器人
            this->robot_ptr_->step(action, this->time_step_);
            // 可视化机器人
            this->visualizeRobot(this->robot_ptr_);
            // 更新信息
            this->getInformation(this->robot_ptr_, obstacles_msg);
            this->processInfo(this->robot_ptr_, global_map, obstacles_msg, obstacles_info, gridmap);
            local_kdtree = KDTree(gridmap);
            // 更新导航路径
            this->generateNavigation(this->robot_ptr_, target_pose, global_map, navigation_path);
            // 进行机器人位置记录
            robot_trajectory_recorder.push_back(this->robot_ptr_->getPose());
            // 进行轨迹可视化
            this->visualizeTrajectories(robot_trajectory_recorder, obstacles_info);
            // 计算离障碍物距离
            double dist_to_obs = distToObs(local_kdtree, this->radius_);
            // 进行记录
            record_data_file << action.v_ << "," << action.w_ << "," << dist_to_obs << "," << (end_planning_time - start_planning_time).toNSec() / 1e6 << "\n";
            // 进行可视化记录
            vis_data_file << this->robot_ptr_->getPose().x_ << "," << this->robot_ptr_->getPose().y_ << ";";
            for (auto obstacle: obstacles_msg.obstacles) {
                vis_data_file << obstacle.id << "," << obstacle.point.x << "," << obstacle.point.y << ";";
            }
            vis_data_file << "\n";
            // 判断是否到达终点
            double dist_to_goal = sqrt((this->robot_ptr_->getPose().x_ - target_pose.x_) * (this->robot_ptr_->getPose().x_ - target_pose.x_) + (this->robot_ptr_->getPose().y_ - target_pose.y_) * (this->robot_ptr_->getPose().y_ - target_pose.y_));
            if (Tools::isSmall(dist_to_goal, this->radius_)) {
                planning_result = 1;
                break;
            }
            // 判断是否发生碰撞
            if (this->collisionJudge(local_kdtree)) {
                planning_result = 0;
                break;
            }
            current_step++;
            loop_rate.sleep();
        }
        // 判断结果
        if (planning_result == 1) {
            std::cout << "planning success" << std::endl;
        } else if (planning_result == 0) {
            std::cout << "planning failed" << std::endl;
        } else {
            std::cout << "planning time out" << std::endl;
        }
        // 进行记录
        record_data_file << planning_result;
        record_data_file.close();
        vis_data_file.close();
        return planning_result;
    };

    // 进行一次规划
    int planCrowdOnce(int test_count=0) {
        // 进行环境的初始化
        std::cout << "start crowd environment initialize" << std::endl;
        // 加载测试场景
        std::string scenario_info_file_path = ros::package::getPath("crowd_simulator") + "/info/info.yaml";
        YAML::Node scenario_info_file = YAML::LoadFile(scenario_info_file_path);
        std::vector<std::string> scenario_names;
        for (auto scenario_name: scenario_info_file["scenario_names"]) {
            scenario_names.push_back(scenario_name.as<std::string>());
        }
        std::vector<std::pair<double, double>> scenario_durations;
        for (auto scenario_duration: scenario_info_file["time_durations"]) {
            scenario_durations.push_back(std::make_pair(scenario_duration[0].as<double>(), scenario_duration[1].as<double>()));
        }
        // 随机选择测试场景
        int choosed_scenario_index = rand() % scenario_names.size();
        std::string scenario_name = scenario_names[choosed_scenario_index];
        std::pair<double, double> scenario_duration = scenario_durations[choosed_scenario_index];
        std::cout << "choosed scenario: " << scenario_name << ", duration from " << scenario_duration.first << " to " << scenario_duration.second << std::endl;
        // 进行地图的加载
        std::string map_info_file_path = ros::package::getPath("crowd_simulator") + "/map/" + scenario_name + ".yaml";
        YAML::Node map_info_file = YAML::LoadFile(map_info_file_path);
        double resolution = map_info_file["resolution"].as<double>();
        double root_x = map_info_file["origin"][0].as<double>();
        double root_y = map_info_file["origin"][1].as<double>();
        double root_theta = map_info_file["origin"][2].as<double>();
        std::string map_file_path = ros::package::getPath("crowd_simulator") + "/map/" + map_info_file["image"].as<std::string>();
        cv::Mat map_image = cv::imread(map_file_path, 0);
        cv::bitwise_not(map_image, map_image);
        GridMap global_map = GridMap(map_image, resolution, root_x, root_y, root_theta);
        this->static_map_pub_.publish(global_map.toRosMessage("odom"));
        
        // 进行环境初始化
        obstacle_msgs::CircleObstacles obstacles_msg;
        std::map<int, Tools::FixedQueue<ObstacleInfo, OBSTACLE_INFO_LEN>> obstacles_info;
        GridMap gridmap;
        KDTree local_kdtree;
        Pose target_pose;
        std::mt19937 gen(test_count);
        double obs_time_gap;
        while (true) {
            // 准备Crowd人群模拟
            crowd_simulator::PrepareCrowdSimulationRequest prepare_crowd_simulation_req;
            crowd_simulator::PrepareCrowdSimulationResponse prepare_crowd_simulation_res;
            assert(scenario_duration.second - scenario_duration.first > 1.1 * static_cast<double>(this->max_time_steps_) * this->time_step_);
            std::uniform_real_distribution<double> distribution(scenario_duration.first, scenario_duration.second - 1.1 * static_cast<double>(this->max_time_steps_) * this->time_step_);
            prepare_crowd_simulation_req.start_stamp = distribution(gen);
            std::cout << "prepare_crowd_simulation_req.start_stamp " << prepare_crowd_simulation_req.start_stamp << std::endl;
            prepare_crowd_simulation_req.crowd_name = scenario_name;
            if(!this->prepare_simulation_service_.call(prepare_crowd_simulation_req, prepare_crowd_simulation_res)) {
                std::cerr << "call prepare crowd simulation service failed" << std::endl;
                exit(0);
            }
            if (prepare_crowd_simulation_res.result != crowd_simulator::PrepareCrowdSimulation::Response::READY) {
                std::cerr << "prepare crowd simulation service failed" << std::endl;
                continue;
            }
            // 得到障碍物更新时间间隔
            obs_time_gap = prepare_crowd_simulation_res.time_gap;
            // 得到障碍物信息
            for (auto start_obstacle: prepare_crowd_simulation_res.start_obstacles.obstacles) {
                // 创建信息
                obstacles_info.insert(std::make_pair(start_obstacle.id, Tools::FixedQueue<ObstacleInfo, OBSTACLE_INFO_LEN>()));
                obstacles_info[start_obstacle.id].push({start_obstacle.point.x, start_obstacle.point.y, start_obstacle.twist.linear.x, start_obstacle.twist.linear.y, start_obstacle.radius});
            }
            // 进行初始状态的生成
            Position init_position = Tools::calcOldCoordinationPosition({global_map.getRootX(), global_map.getRootY(), global_map.getRootTheta()}, {static_cast<double>(global_map.getWidth()) * 0.5 * global_map.getResolution(), this->radius_ * 2});
            Pose init_pose = {init_position.x_, init_position.y_, Tools::pi2Pi(M_PI * 0.5 + global_map.getRootTheta())};
            Action init_action = {0.0, 0.0};
            // 设置终点位置
            Position target_position = Tools::calcOldCoordinationPosition({global_map.getRootX(), global_map.getRootY(), global_map.getRootTheta()}, {static_cast<double>(global_map.getWidth()) * 0.5 * global_map.getResolution(), static_cast<double>(global_map.getHeight()) * global_map.getResolution() - this->radius_ * 2});
            target_pose = {target_position.x_, target_position.y_, 0.0};
            // 进行机器人设置
            this->robot_ptr_->init(init_pose, init_action);
            // 局部栅格地图得到
            local_map_generation::GetLocalMapRequest get_local_map_req;
            local_map_generation::GetLocalMapResponse get_local_map_res;
            get_local_map_req.pose.position.x = this->robot_ptr_->getPose().x_;
            get_local_map_req.pose.position.y = this->robot_ptr_->getPose().y_;
            get_local_map_req.pose.orientation.z = sin(this->robot_ptr_->getPose().theta_ * 0.5);
            get_local_map_req.pose.orientation.w = cos(this->robot_ptr_->getPose().theta_ * 0.5);
            get_local_map_req.obstacles = prepare_crowd_simulation_res.start_obstacles;
            get_local_map_req.global_map = global_map.toRosMessage("odom");
            get_local_map_req.publish = true;
            if (!this->local_map_service_.call(get_local_map_req, get_local_map_res)) {
                std::cerr << "call local map service failed" << std::endl;
                exit(0);
            }
            gridmap = GridMap(get_local_map_res.local_map);
            local_kdtree = KDTree(gridmap);
            // 判断当前是否发生碰撞
            if (!this->collisionJudge(local_kdtree)) {
                break;
            }
        }
        
        // 生成导航路径
        std::vector<PathPose> navigation_path;
        this->generateNavigation(this->robot_ptr_, target_pose, global_map, navigation_path);
        // 进行机器人位置记录
        std::vector<Pose> robot_trajectory_recorder;
        robot_trajectory_recorder.push_back(this->robot_ptr_->getPose());
        // 可视化终点
        this->visualizeTarget(target_pose);
        // 可视化机器人
        this->visualizeRobot(this->robot_ptr_);
        // 进行轨迹可视化
        this->visualizeTrajectories(robot_trajectory_recorder, obstacles_info);
        std::cout << "planning test environment initialized" << std::endl;
        // 进行文件记录
        std::string record_data_file_path = ros::package::getPath("local_planner") + "/data/test_record_" + std::to_string(test_count) + ".csv";
        std::ofstream record_data_file(record_data_file_path, std::ios::out|std::ios::trunc);
        std::string vis_data_file_path = ros::package::getPath("local_planner") + "/vis/test_vis.csv";
        std::ofstream vis_data_file(vis_data_file_path, std::ios::out|std::ios::trunc);
        // 进行规划模拟
        int planning_result = -1;
        int current_step = 0;
        ros::Rate loop_rate(100);
        while (current_step < this->max_time_steps_) {
            // 获取行为
            Action action = {0.0, 0.0};
            auto start_planning_time = ros::WallTime::now();
            int forward_result = this->policy_ptr_->forward(*(this->robot_ptr_), target_pose, navigation_path, global_map, obstacles_info, action);
            if (forward_result < 0) {
                planning_result = 0;
                std::cerr << "policy failes to give an action" << std::endl;
                break;
            }
            auto end_planning_time = ros::WallTime::now();
            std::cout << "planning time consuming is " << (end_planning_time - start_planning_time).toNSec() / 1e6 << "ms" << std::endl;
            // getchar();
            // 更新机器人
            this->robot_ptr_->step(action, this->time_step_);
            // 可视化机器人
            this->visualizeRobot(this->robot_ptr_);
            // 更新信息
            this->getInformation(this->robot_ptr_, obstacles_msg, (this->time_step_/ obs_time_gap));
            this->processInfo(this->robot_ptr_, global_map, obstacles_msg, obstacles_info, gridmap);
            local_kdtree = KDTree(gridmap);
            // 更新导航路径
            this->generateNavigation(this->robot_ptr_, target_pose, global_map, navigation_path);
            // 进行机器人位置记录
            robot_trajectory_recorder.push_back(this->robot_ptr_->getPose());
            // 进行轨迹可视化
            this->visualizeTrajectories(robot_trajectory_recorder, obstacles_info);
            // 计算离障碍物距离
            double dist_to_obs = distToObs(local_kdtree, this->radius_);
            record_data_file << action.v_ << "," << action.w_ << "," << dist_to_obs << "," << (end_planning_time - start_planning_time).toNSec() / 1e6 << "\n";
            // 进行可视化记录
            vis_data_file << this->robot_ptr_->getPose().x_ << "," << this->robot_ptr_->getPose().y_ << ";";
            for (auto obstacle: obstacles_msg.obstacles) {
                vis_data_file << obstacle.id << "," << obstacle.point.x << "," << obstacle.point.y << ";";
            }
            vis_data_file << "\n";
            // 判断是否到达终点
            double dist_to_goal = sqrt((this->robot_ptr_->getPose().x_ - target_pose.x_) * (this->robot_ptr_->getPose().x_ - target_pose.x_) + (this->robot_ptr_->getPose().y_ - target_pose.y_) * (this->robot_ptr_->getPose().y_ - target_pose.y_));
            if (Tools::isSmall(dist_to_goal, this->radius_)) {
                planning_result = 1;
                break;
            }
            // 判断是否发生碰撞
            if (this->collisionJudge(local_kdtree)) {
                planning_result = 0;
                break;
            }
            current_step++;
            loop_rate.sleep();
        }
        // 判断结果
        if (planning_result == 1) {
            std::cout << "planning success" << std::endl;
        } else if (planning_result == 0) {
            std::cout << "planning failed" << std::endl;
        } else {
            std::cout << "planning time out" << std::endl;
        }
        // 进行记录
        record_data_file << planning_result;
        record_data_file.close();
        vis_data_file.close();
        // getchar();
        return planning_result;
    };

    int planStaticOnce(int test_count=0) {
        // 进行环境的初始化
        std::cout << "start static environment initialize" << std::endl;
        // 加载测试场景
        std::string scenario_info_file_path = ros::package::getPath("static_map") + "/info/info.yaml";
        YAML::Node scenario_info_file = YAML::LoadFile(scenario_info_file_path);
        std::vector<std::string> scenario_names;
        for (auto scenario_name: scenario_info_file["scenario_names"]) {
            scenario_names.push_back(scenario_name.as<std::string>());
        }
        // 随机选择测试场景
        int choosed_scenario_index = rand() % scenario_names.size();
        std::string scenario_name = scenario_names[choosed_scenario_index];
        std::cout << "choosed scenario: " << scenario_name << std::endl;
        // 进行地图的加载
        std::string map_info_file_path = ros::package::getPath("static_map") + "/map/" + scenario_name + ".yaml";
        YAML::Node map_info_file = YAML::LoadFile(map_info_file_path);
        double resolution = map_info_file["resolution"].as<double>();
        double root_x = map_info_file["origin"][0].as<double>();
        double root_y = map_info_file["origin"][1].as<double>();
        double root_theta = map_info_file["origin"][2].as<double>();
        std::string map_file_path = ros::package::getPath("static_map") + "/map/" + map_info_file["image"].as<std::string>();
        cv::Mat map_image = cv::imread(map_file_path, 0);
        cv::bitwise_not(map_image, map_image);
        GridMap global_map = GridMap(map_image, resolution, root_x, root_y, root_theta);
        this->static_map_pub_.publish(global_map.toRosMessage("odom"));
        
        // 进行环境初始化
        obstacle_msgs::CircleObstacles obstacles_msg;
        std::map<int, Tools::FixedQueue<ObstacleInfo, OBSTACLE_INFO_LEN>> obstacles_info;
        GridMap gridmap;
        KDTree local_kdtree;
        Pose target_pose;
        std::vector<PathPose> navigation_path;
        std::mt19937 gen(test_count);
        double obs_time_gap;
        while (true) {
            // 进行初始状态的生成
            std::uniform_real_distribution<double> map_x_distr(this->radius_ * 2, static_cast<double>(global_map.getWidth()) * global_map.getResolution() - this->radius_ * 2);
            std::uniform_real_distribution<double> map_y_distr(this->radius_ * 2, static_cast<double>(global_map.getHeight()) * global_map.getResolution() - this->radius_ * 2);
            std::uniform_real_distribution<double> theta_distr(-M_PI, M_PI);
            Position init_position = Tools::calcOldCoordinationPosition({global_map.getRootX(), global_map.getRootY(), global_map.getRootTheta()}, {map_x_distr(gen), map_y_distr(gen)});
            Pose init_pose = {init_position.x_, init_position.y_, theta_distr(gen)};
            Action init_action = {0.0, 0.0};
            // 设置终点位置
            Position target_position = Tools::calcOldCoordinationPosition({global_map.getRootX(), global_map.getRootY(), global_map.getRootTheta()}, {map_x_distr(gen), map_y_distr(gen)});
            target_pose = {target_position.x_, target_position.y_, 0.0};
            // 进行机器人设置
            this->robot_ptr_->init(init_pose, init_action);
            // 生成导航路径
            int nav_result = this->generateNavigation(this->robot_ptr_, target_pose, global_map, navigation_path);
            if (!nav_result) {
                continue;
            }
            // 局部栅格地图得到
            local_map_generation::GetLocalMapRequest get_local_map_req;
            local_map_generation::GetLocalMapResponse get_local_map_res;
            get_local_map_req.pose.position.x = this->robot_ptr_->getPose().x_;
            get_local_map_req.pose.position.y = this->robot_ptr_->getPose().y_;
            get_local_map_req.pose.orientation.z = sin(this->robot_ptr_->getPose().theta_ * 0.5);
            get_local_map_req.pose.orientation.w = cos(this->robot_ptr_->getPose().theta_ * 0.5);
            get_local_map_req.global_map = global_map.toRosMessage("odom");
            get_local_map_req.publish = true;
            if (!this->local_map_service_.call(get_local_map_req, get_local_map_res)) {
                std::cerr << "call local map service failed" << std::endl;
                exit(0);
            }
            gridmap = GridMap(get_local_map_res.local_map);
            local_kdtree = KDTree(gridmap);
            // 判断当前距离障碍物是否大于阈值
            if (Tools::isLarge(this->distToObs(local_kdtree, this->radius_), 0.1)) {
                break;
            }
        }
        // 进行机器人位置记录
        std::vector<Pose> robot_trajectory_recorder;
        robot_trajectory_recorder.push_back(this->robot_ptr_->getPose());
        // 可视化终点
        this->visualizeTarget(target_pose);
        // 可视化机器人
        this->visualizeRobot(this->robot_ptr_);
        // 进行轨迹可视化
        this->visualizeTrajectories(robot_trajectory_recorder, obstacles_info);
        std::cout << "planning test environment initialized" << std::endl;
        // 进行文件记录
        std::string record_data_file_path = ros::package::getPath("local_planner") + "/data/test_record_" + std::to_string(test_count) + ".csv";
        std::ofstream record_data_file(record_data_file_path, std::ios::out|std::ios::trunc);
        std::string record_trajectory_file_path = ros::package::getPath("local_planner") + "/data/test_trajectory.csv";
        std::ofstream record_trajectory_file(record_trajectory_file_path, std::ios::out|std::ios::trunc);
        // 进行规划模拟
        int planning_result = -1;
        int current_step = 0;
        ros::Rate loop_rate(100);
        while (current_step < this->max_time_steps_) {
            // 获取行为
            Action action = {0.0, 0.0};
            auto start_planning_time = ros::WallTime::now();
            int forward_result = this->policy_ptr_->forward(*(this->robot_ptr_), target_pose, navigation_path, global_map, obstacles_info, action);
            if (forward_result < 0) {
                planning_result = 0;
                std::cerr << "policy failes to give an action" << std::endl;
                break;
            }
            auto end_planning_time = ros::WallTime::now();
            std::cout << "planning time consuming is " << (end_planning_time - start_planning_time).toNSec() / 1e6 << "ms" << std::endl;
            // getchar();
            // 更新机器人
            this->robot_ptr_->step(action, this->time_step_);
            // 可视化机器人
            this->visualizeRobot(this->robot_ptr_);
            // 更新信息
            this->getInformation(this->robot_ptr_, obstacles_msg, round(this->time_step_/ obs_time_gap));
            this->processInfo(this->robot_ptr_, global_map, obstacles_msg, obstacles_info, gridmap);
            local_kdtree = KDTree(gridmap);
            // 计算离障碍物距离
            double dist_to_obs = distToObs(local_kdtree, this->radius_);
            record_data_file << action.v_ << "," << action.w_ << "," << dist_to_obs << "," << (end_planning_time - start_planning_time).toNSec() / 1e6 << "\n";
            // 判断是否到达终点
            double dist_to_goal = sqrt((this->robot_ptr_->getPose().x_ - target_pose.x_) * (this->robot_ptr_->getPose().x_ - target_pose.x_) + (this->robot_ptr_->getPose().y_ - target_pose.y_) * (this->robot_ptr_->getPose().y_ - target_pose.y_));
            std::cout << "dist_to_goal: " << dist_to_goal << std::endl;
            if (Tools::isSmall(dist_to_goal, this->radius_)) {
                planning_result = 1;
                break;
            }
            // 更新导航路径
            int nav_result = this->generateNavigation(this->robot_ptr_, target_pose, global_map, navigation_path);
            // 进行机器人位置记录
            robot_trajectory_recorder.push_back(this->robot_ptr_->getPose());
            // 进行轨迹可视化
            this->visualizeTrajectories(robot_trajectory_recorder, obstacles_info);
            // 判断是否发生碰撞
            if (this->collisionJudge(local_kdtree)) {
                planning_result = 0;
                break;
            }
            // 判断导航是否失败
            if (!nav_result) {
                planning_result = 0;
                break;
            }
            current_step++;
            loop_rate.sleep();
        }
        // 判断结果
        if (planning_result == 1) {
            std::cout << "planning success" << std::endl;
        } else if (planning_result == 0) {
            std::cout << "planning failed" << std::endl;
        } else {
            std::cout << "planning time out" << std::endl;
        }
        // 进行记录
        record_data_file << planning_result;
        record_data_file.close();
        for (auto pose: robot_trajectory_recorder) {
            record_trajectory_file << pose.x_ << "," << pose.y_ << "," << pose.theta_ << "\n";
        }
        record_trajectory_file.close();
        return planning_result;
    }

    std::string getSimulationName() const {
        return this->simulator_;
    }

 private:
    // ros相关
    ros::NodeHandle nh_;
    ros::Publisher static_map_pub_;
    ros::Publisher robot_vis_pub_;
    ros::Publisher target_pose_pub_;
    ros::Publisher historical_poses_vis_pub_;
    ros::Publisher crowd_trajectory_vis_pub_;
    ros::ServiceClient prepare_simulation_service_;
    ros::ServiceClient obstacles_service_;
    ros::ServiceClient end_simulation_service_;
    ros::ServiceClient local_map_service_;
    ros::ServiceClient navigation_service_;
    // 规划相关
    double max_v_;
    double min_v_;
    double max_w_;
    double max_acc_;
    double max_angular_acc_;
    double radius_;
    double scan_radius_;
    std::string policy_name_;
    double circle_radius_;
    double time_step_;
    int max_time_steps_;
    std::string simulator_;
    std::string nav_method_;
    std::shared_ptr<Robot> robot_ptr_;
    // 策略
    std::shared_ptr<Policy> policy_ptr_;

    // 导航路径生成
    bool generateNavigation(const std::shared_ptr<Robot> &robot_ptr, const Pose &target_pose, const GridMap &global_map, std::vector<PathPose> &navigation_path) {
        std::vector<PathPose>().swap(navigation_path);
        if (this->nav_method_ == "astar") {
            // 调用服务
            navigation::AstarNavigationRequest req;
            req.start_point.x = robot_ptr->getPose().x_;
            req.start_point.y = robot_ptr->getPose().y_;
            req.goal_point.x = target_pose.x_;
            req.goal_point.y = target_pose.y_;
            req.cost_map = global_map.toRosMessage("odom");
            navigation::AstarNavigationResponse res;
            if (!this->navigation_service_.call(req, res)) {
                std::cerr << "call navigation service failed" << std::endl;
                exit(0);
            }
            // 判断导航是否成功
            if (res.navigation_path.poses.size() < 2) {
                return false;
            }
            // 进行格式转换
            // std::cout << "res.navigation_path.poses.size(): " << res.navigation_path.poses.size() << std::endl;
            double dist = 0.0;
            for (int i = 0; i < res.navigation_path.poses.size() - 1; i++) {
                double segment_dist = sqrt(std::pow(res.navigation_path.poses[i + 1].pose.position.x - res.navigation_path.poses[i].pose.position.x, 2) + std::pow(res.navigation_path.poses[i + 1].pose.position.y - res.navigation_path.poses[i].pose.position.y, 2));
                double dire = atan2(res.navigation_path.poses[i + 1].pose.position.y - res.navigation_path.poses[i].pose.position.y, res.navigation_path.poses[i + 1].pose.position.x - res.navigation_path.poses[i].pose.position.x);
                int sample_num = static_cast<int>(segment_dist / NAVIGATION_GAP);
                std::vector<double> samples = Tools::linspace<double>(0.0, segment_dist, sample_num);
                for (auto sample: samples) {
                    PathPose pose = {res.navigation_path.poses[i].pose.position.x + sample * cos(dire), res.navigation_path.poses[i].pose.position.y + sample * sin(dire), dire, dist + sample};
                    navigation_path.push_back(pose);
                }
                dist += segment_dist;
            }
            // 判断导航路径长度
            if (navigation_path.size() >= 2) {
                return true;
            } else {
                std::vector<PathPose>().swap(navigation_path);
            }
        }
        // 计算从起点到终点距离
        double dist_to_goal = sqrt((robot_ptr->getPose().x_ - target_pose.x_) * (robot_ptr->getPose().x_ - target_pose.x_) + (robot_ptr->getPose().y_ - target_pose.y_) * (robot_ptr->getPose().y_ - target_pose.y_));
        double dire = atan2(target_pose.y_ - robot_ptr->getPose().y_, target_pose.x_ - robot_ptr->getPose().x_);
        // 计算采样数量
        int sample_num = static_cast<int>(dist_to_goal / NAVIGATION_GAP);
        sample_num = std::max(sample_num, 2);
        std::vector<double> samples = Tools::linspace<double>(0.0, dist_to_goal, sample_num);
        for (auto sample: samples) {
            PathPose pose = {robot_ptr->getPose().x_ + sample * cos(dire), robot_ptr->getPose().y_ + sample * sin(dire), dire, sample};
            navigation_path.push_back(pose);
        }
        return true;
    }

    // 碰撞判断
    bool collisionJudge(const KDTree &kdtree) {
        std::vector<std::pair<float, float>> results;
        std::vector<float> sq_distances;
        if (kdtree.findRangeNeighbor(0.0, 0.0, &results, &sq_distances, this->radius_) == -1) {
            // 无碰撞
            return false;
        } else {
            return true;
        }
    }

    // 计算距离
    double distToObs(const KDTree &kdtree, double radius) {
        double min_dist = this->scan_radius_;
        std::vector<std::pair<float, float>> results;
        std::vector<float> sq_distances;
        if (kdtree.findKNeighbor(0.0, 0.0, &results, &sq_distances, 1) != -1) {
            double dist = std::max(0.0, sqrt(sq_distances.front()) - radius);
            if (Tools::isSmall(dist, min_dist)) {
                min_dist = dist;
            }
        }
        return min_dist;
    }

    // 处理障碍物信息
    void processInfo(const std::shared_ptr<Robot> &robot_ptr, const GridMap &global_map, const obstacle_msgs::CircleObstacles &current_obstacles, std::map<int, Tools::FixedQueue<ObstacleInfo, OBSTACLE_INFO_LEN>> &obstacles_info, GridMap &gridmap) {
        std::vector<obstacle_msgs::CircleObstacle> local_obstacles;
        // 对障碍物进行过滤
        for (auto crowd_obstacle: current_obstacles.obstacles) {
            // 计算障碍物到机器人距离
            // std::cout << "crowd_obstacle: " << crowd_obstacle.point.x << "," << crowd_obstacle.point.y << "," << crowd_obstacle.twist.linear.x << "," << crowd_obstacle.twist.linear.y << std::endl;
            // std::cout << "robot: " << robot_ptr->getPose().x_ << ", " << robot_ptr->getPose().y_ << std::endl;
            double dist = sqrt((crowd_obstacle.point.x - robot_ptr->getPose().x_) * (crowd_obstacle.point.x - robot_ptr->getPose().x_) + (crowd_obstacle.point.y - robot_ptr->getPose().y_) * (crowd_obstacle.point.y - robot_ptr->getPose().y_));
            // std::cout << "dist: " << dist << std::endl;
            if (Tools::isSmall(dist, this->scan_radius_)) {
                local_obstacles.push_back(crowd_obstacle);
            }
        }
        // 删除过时障碍物信息
        std::vector<int> local_obstacle_ids;
        for (auto local_obstacle: local_obstacles) {
            local_obstacle_ids.push_back(local_obstacle.id);
        }
        std::vector<int> none_exist_ids;
        std::vector<int> exist_ids;
        for (auto iter = obstacles_info.begin(); iter != obstacles_info.end(); iter++) {
            if (std::find(local_obstacle_ids.begin(), local_obstacle_ids.end(), iter->first) == local_obstacle_ids.end()) {
                none_exist_ids.push_back(iter->first);
            } else {
                exist_ids.push_back(iter->first);
            }
        }
        for (auto none_exist_id: none_exist_ids) {
            obstacles_info.erase(none_exist_id);
        }
        // 更新障碍物信息
        for (auto local_obstacle: local_obstacles) {
            if (std::find(exist_ids.begin(), exist_ids.end(), local_obstacle.id) != exist_ids.end()) {
                // 加入信息
                obstacles_info[local_obstacle.id].push({local_obstacle.point.x, local_obstacle.point.y, local_obstacle.twist.linear.x, local_obstacle.twist.linear.y, local_obstacle.radius});
            } else {
                // 创建信息
                obstacles_info.insert(std::make_pair(local_obstacle.id, Tools::FixedQueue<ObstacleInfo, OBSTACLE_INFO_LEN>()));
                obstacles_info[local_obstacle.id].push({local_obstacle.point.x, local_obstacle.point.y, local_obstacle.twist.linear.x, local_obstacle.twist.linear.y, local_obstacle.radius});
            }
        }
        // 获得局部地图信息
        obstacle_msgs::CircleObstacles local_obstacles_msg;
        local_obstacles_msg.header.frame_id = "odom";
        local_obstacles_msg.header.stamp = ros::Time::now();
        local_obstacles_msg.obstacles = local_obstacles;
        local_map_generation::GetLocalMapRequest get_local_map_req;
        local_map_generation::GetLocalMapResponse get_local_map_res;
        get_local_map_req.pose.position.x = robot_ptr->getPose().x_;
        get_local_map_req.pose.position.y = robot_ptr->getPose().y_;
        get_local_map_req.pose.orientation.z = sin(robot_ptr->getPose().theta_ * 0.5);
        get_local_map_req.pose.orientation.w = cos(robot_ptr->getPose().theta_ * 0.5);
        get_local_map_req.obstacles = local_obstacles_msg;
        get_local_map_req.global_map = global_map.toRosMessage("odom");
        get_local_map_req.publish = true;
        if (!this->local_map_service_.call(get_local_map_req, get_local_map_res)) {
            std::cerr << "call local map service failed" << std::endl;
            exit(0);
        }
        gridmap = GridMap(get_local_map_res.local_map);
    }

    // 获取信息
    void getInformation(const std::shared_ptr<Robot> &robot_ptr, obstacle_msgs::CircleObstacles &current_obstacles, int update_times=1) {
        if (this->simulator_ == "orca" || this->simulator_ == "crowd" || this->simulator_ == "hybrid") {
            // 获取障碍物信息
            crowd_simulator::GetObstaclesRequest get_obstacles_req;
            crowd_simulator::GetObstaclesResponse get_obstacles_res;
            for (int i = 0; i < update_times; i++) {
                if (!this->obstacles_service_.call(get_obstacles_req, get_obstacles_res)) {
                    std::cerr << "call obstacle service failed" << std::endl;
                    exit(0);
                }
            }
            current_obstacles = get_obstacles_res.obstacles;
        }
    }

    // 可视化轨迹
    void visualizeTrajectories(const std::vector<Pose> &robot_trajectory_recorder, const std::map<int, Tools::FixedQueue<ObstacleInfo, OBSTACLE_INFO_LEN>> &obstacles_info) {
        // 可视化机器人轨迹
        visualization_msgs::MarkerArray robot_marker_array;
        visualization_msgs::Marker robot_marker;
        robot_marker.header.frame_id = "odom";
        robot_marker.header.stamp = ros::Time::now();
        robot_marker.action = visualization_msgs::Marker::ADD;
        robot_marker.type = visualization_msgs::Marker::LINE_STRIP;
        robot_marker.id = 0;
        robot_marker.scale.x = 0.05;
        robot_marker.color.r = 0;
        robot_marker.color.g = 1;
        robot_marker.color.b = 0;
        robot_marker.color.a = 1;
        for (auto robot_pose: robot_trajectory_recorder) {
            geometry_msgs::Point point;
            point.x = robot_pose.x_;
            point.y = robot_pose.y_;
            robot_marker.points.push_back(point);
        }
        robot_marker_array.markers.push_back(robot_marker);
        this->historical_poses_vis_pub_.publish(robot_marker_array);

        // 可视化障碍物轨迹
        visualization_msgs::MarkerArray obstacle_marker_array, del_marker_array;
        // 删除之前的可视化
        visualization_msgs::Marker del_marker;
        del_marker.header.frame_id = "odom";
        del_marker.header.stamp = ros::Time::now();
        del_marker.action = visualization_msgs::Marker::DELETEALL;
        del_marker.id = 0;
        del_marker_array.markers.push_back(del_marker);
        this->crowd_trajectory_vis_pub_.publish(del_marker_array);
        // 进行可视化
        for (auto obstacle_info: obstacles_info) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = ros::Time::now();
            marker.action = visualization_msgs::Marker::ADD;
            marker.id = obstacle_info.first;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.color.r = 1;
            marker.color.g = 0;
            marker.color.b = 0;
            marker.color.a = 1;
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            for (auto iter = obstacle_info.second.begin(); iter != obstacle_info.second.end(); iter++) {
                geometry_msgs::Point point;
                point.x = iter->x_;
                point.y = iter->y_;
                marker.points.push_back(point);
            }
            obstacle_marker_array.markers.push_back(marker);
        }
        this->crowd_trajectory_vis_pub_.publish(obstacle_marker_array);
    }

    // 可视化机器人
    void visualizeRobot(const std::shared_ptr<Robot> &robot_ptr) {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = robot_ptr->getPose().x_;
        marker.pose.position.y = robot_ptr->getPose().y_;
        marker.scale.x = this->radius_ * 2.0;
        marker.scale.y = this->radius_ * 2.0;
        marker.scale.z = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;
        marker_array.markers.push_back(marker);
        visualization_msgs::Marker dire_marker;
        dire_marker.header = marker.header;
        dire_marker.id = 1;
        dire_marker.type = visualization_msgs::Marker::ARROW;
        dire_marker.action = visualization_msgs::Marker::ADD;
        dire_marker.pose.position.x = robot_ptr->getPose().x_;
        dire_marker.pose.position.y = robot_ptr->getPose().y_;
        dire_marker.pose.orientation.z = sin(robot_ptr->getPose().theta_ * 0.5);
        dire_marker.pose.orientation.w = cos(robot_ptr->getPose().theta_ * 0.5);
        dire_marker.scale.x = this->radius_ * 1.5;
        dire_marker.scale.y = 0.05;
        dire_marker.scale.z = 0.05;
        dire_marker.color.r = 1.0;
        dire_marker.color.g = 0.0;
        dire_marker.color.b = 0.0;
        dire_marker.color.a = 1.0;
        marker_array.markers.push_back(dire_marker);
        this->robot_vis_pub_.publish(marker_array);
    }

    // 可视化终点
    void visualizeTarget(const Pose &target_pose) {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = target_pose.x_;
        marker.pose.position.y = target_pose.y_;
        marker.scale.x = this->radius_ * 2.0;
        marker.scale.y = this->radius_ * 2.0;
        marker.scale.z = 0.05;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker_array.markers.push_back(marker);
        this->target_pose_pub_.publish(marker_array);
    }

    // ros线程
    void rosConnector() {
        ros::spin();
    };

    // 初始化ros相关消息和服务
    void initRosConnection() {
        // 静态地图消息
        std::string static_map_topic;
        this->nh_.getParam("static_map_topic", static_map_topic);
        this->static_map_pub_ = this->nh_.advertise<nav_msgs::OccupancyGrid> (static_map_topic, 10);
        // 机器人可视化
        std::string robot_vis_topic;
        this->nh_.getParam("robot_vis_topic", robot_vis_topic);
        this->robot_vis_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(robot_vis_topic, 10);
        // 目标点可视化
        std::string target_pose_topic;
        this->nh_.getParam("target_pose_topic", target_pose_topic);
        this->target_pose_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(target_pose_topic, 10);
        // 历史轨迹可视化
        std::string historical_poses_vis_topic;
        this->nh_.getParam("historical_poses_vis_topic", historical_poses_vis_topic);
        this->historical_poses_vis_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(historical_poses_vis_topic, 10);
        // 障碍物轨迹可视化
        std::string crowd_trajectory_vis_topic;
        this->nh_.getParam("crowd_trajectory_vis_topic", crowd_trajectory_vis_topic);
        this->crowd_trajectory_vis_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(crowd_trajectory_vis_topic, 10);

        // 准备ORCA障碍物模拟服务
        if (this->simulator_ == "orca") {
            std::string prepare_simulation_service_name;
            this->nh_.getParam("prepare_simulation_service_name", prepare_simulation_service_name);
            ros::service::waitForService(prepare_simulation_service_name);
            this->prepare_simulation_service_ = this->nh_.serviceClient<crowd_simulator::PrepareORCASimulation>(prepare_simulation_service_name);
            // 结束障碍物模拟服务
            std::string end_simulation_service_name;
            this->nh_.getParam("end_simulation_service_name", end_simulation_service_name);
            ros::service::waitForService(end_simulation_service_name);
            this->end_simulation_service_ = this->nh_.serviceClient<std_srvs::Empty>(end_simulation_service_name);
            // 障碍物服务
            std::string obstacles_service_name;
            this->nh_.getParam("obstacles_service_name", obstacles_service_name);
            ros::service::waitForService(obstacles_service_name);
            this->obstacles_service_ = this->nh_.serviceClient<crowd_simulator::GetObstacles>(obstacles_service_name, true);
        } else if (this->simulator_ == "crowd") {
            std::string prepare_simulation_service_name;
            this->nh_.getParam("prepare_simulation_service_name", prepare_simulation_service_name);
            ros::service::waitForService(prepare_simulation_service_name);
            this->prepare_simulation_service_ = this->nh_.serviceClient<crowd_simulator::PrepareCrowdSimulation>(prepare_simulation_service_name);
            // 障碍物服务
            std::string obstacles_service_name;
            this->nh_.getParam("obstacles_service_name", obstacles_service_name);
            ros::service::waitForService(obstacles_service_name);
            this->obstacles_service_ = this->nh_.serviceClient<crowd_simulator::GetObstacles>(obstacles_service_name, true);
        } else {
            
        }
        // 局部地图服务
        std::string local_map_service_name;
        this->nh_.getParam("local_map_service_name", local_map_service_name);
        ros::service::waitForService(local_map_service_name);
        this->local_map_service_ = this->nh_.serviceClient<local_map_generation:: GetLocalMap>(local_map_service_name, true);
        // 导航服务
        if (this->nav_method_ == "astar") {
            std::string navigation_service_name;
            this->nh_.getParam("navigation_service_name", navigation_service_name);
            ros::service::waitForService(navigation_service_name);
            this->navigation_service_ = this->nh_.serviceClient<navigation::AstarNavigation>(navigation_service_name, true);
        }
    };

    // 加载参数
    void loadParams() {
        // 读取文件路径
        std::string file_path = ros::package::getPath("local_planner") + "/config/planning.config";
        if (!std::filesystem::exists(file_path)) {
            std::cerr << "config file not exist" << std::endl;
            exit(0);
        }
        // 解析文件
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini(file_path, pt);
        this->max_v_ = pt.get<double>("robot.max_v");
        this->min_v_ = pt.get<double>("robot.min_v");
        this->max_w_ = pt.get<double>("robot.max_w");
        this->max_acc_ = pt.get<double>("robot.max_acc");
        this->max_angular_acc_ = pt.get<double>("robot.max_angular_acc");
        this->radius_ = pt.get<double>("robot.radius");
        this->scan_radius_ = pt.get<double>("robot.scan_radius");
        this->policy_name_ = pt.get<std::string>("robot.policy");
        this->nav_method_ = pt.get<std::string>("robot.nav_method");
        this->circle_radius_ = pt.get<double>("env.circle_radius");
        this->time_step_ = pt.get<double>("env.time_step");
        this->max_time_steps_ = pt.get<int>("env.max_time_steps");
        this->simulator_ = pt.get<std::string>("env.simulator");
    }
};

#endif  // LOCAL_PLANNER_NODE_HPP_
