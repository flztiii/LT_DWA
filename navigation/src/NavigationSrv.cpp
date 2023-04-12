#include "planner/Astar.hpp"
#include "planner/KeyVerticesObtain.hpp"
#include "navigation/AstarNavigation.h"
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <memory>

// planner
std::shared_ptr<BasePlanner> planner_ptr = nullptr;
// publisher
ros::Publisher navigation_pub_;
// robot_disc_size
double robot_disc_size_;

// 获取导航路径服务
bool getNavigationPath(navigation::AstarNavigation::Request &request, navigation::AstarNavigation::Response &response) {
    clock_t start_time = clock();
    // 解析输入信息
    PathPlanningUtilities::Point2f start_point, goal_point;
    start_point.x_ = request.start_point.x;
    start_point.y_ = request.start_point.y;
    goal_point.x_ = request.goal_point.x;
    goal_point.y_ = request.goal_point.y;
    GridMap grid_map = GridMap(request.cost_map);
    KDTree kdtree = KDTree(grid_map);
    // 进行规划
    PathPlanningUtilities::Path astar_path;
    int planning_result = planner_ptr->planning(start_point, goal_point, grid_map, kdtree, astar_path);
    // 判断是否规划失败
    if (planning_result < 0) {
        // 规划失败
        return true;
    }
    // 進行路徑簡化
    std::shared_ptr<DouglasPeuckerSimplify> simplifier_ptr = std::make_shared<DouglasPeuckerSimplify>(DouglasPeuckerSimplify());
    astar_path = simplifier_ptr->simplifiedPath(astar_path, 0.5 * robot_disc_size_);
    // 规划成功,转化数据格式
    nav_msgs::Path navigation_path;
    navigation_path.header.frame_id = "odom";
    navigation_path.header.stamp = ros::Time::now();
    for (size_t i = 0; i < astar_path.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.header = navigation_path.header;
        pose.pose.position.x = astar_path[i].x_;
        pose.pose.position.y = astar_path[i].y_;
        navigation_path.poses.push_back(pose);
    }
    response.navigation_path = navigation_path;
    navigation_pub_.publish(navigation_path);
    clock_t end_time = clock();
    ROS_INFO_STREAM("time consuming is " << static_cast<double>(end_time - start_time) * 1000.0 / CLOCKS_PER_SEC << " ms");
    return true;
}

int main(int argc, char** argv) {
    // 初始化ros
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh("~");
    // planner type
    std::string planner_type;
    nh.getParam("planner_type", planner_type);
    // 发布规划的路径
    std::string navigation_topic;
    nh.getParam("navigation_topic", navigation_topic);
    navigation_pub_ = nh.advertise<nav_msgs::Path>(navigation_topic, 1);
    // 构建规划器
    nh.getParam("robot_disc_size", robot_disc_size_);
    if (planner_type == "astar") {
        planner_ptr = std::make_shared<AstarPlanner>(0.5 * robot_disc_size_);
    } else {
        throw;
    }
    // 创建服务
    std::string navigation_service_name;
    nh.getParam("navigation_service_name", navigation_service_name);
    ros::ServiceServer navigation_service = nh.advertiseService(navigation_service_name, getNavigationPath);
    // 保持ros
    ros::spin();
    return 0;
}