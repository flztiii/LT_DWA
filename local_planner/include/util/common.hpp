/*
    Copyright [2022] Jian ZhiQiang
*/

#ifndef COMMON_HPP_
#define COMMON_HPP_

#define VISUALIZATION false
#define NAVIGATION_GAP 0.1
#define OBSTACLE_INFO_LEN 8
#define PREDICT_TIME_LEN 16
#define V_SAMPLE_NUM 8
#define W_SAMPLE_NUM 8
#define LAYER_MAX_SEED_NUM 300
#define X_SAMPLE_NUM 10
#define Y_SAMPLE_NUM 10
#define THETA_SAMPLE_NUM 10
#define DECLINE_RATE 0.9
#define EVALUATE_MIN_DIST_TO_BOUNDARY 0.3
#define EVALUATE_BOUNDARY_PENALTY_COFF 2e2
#define EVALUATE_MIN_DIST_TO_OBS 0.3
#define EVALUATE_OBS_VEL_COFF 1.0
#define EVALUATE_OBS_PENALTY_COFF 3e2
#define LON_OFFSET_PENALTY_COFF 5.0
#define LAT_OFFSET_PENALTY_COFF 1.0
#define DIRE_PENALTY_COFF 0.5
#define VEL_PENALTY_COFF 0.1
#define ANGULAR_VEL_PENALTY_COFF 0.1
#define ACC_VALUE_PENALTY 0.1
#define ANGULAR_ACC_VALUE_PENALTY 0.1
#define V_LIMIT_PENALTY_COFF 1e4
#define W_LIMIT_PENALTY_COFF 1e4
#define ACC_LIMIT_PENALTY_COFF 1e4
#define ANGULAR_ACC_LIMIT_PENALTY_COFF 1e4
#define KINEMATIC_LIMIT_PENALTY_COFF 1e5
#define OBS_LIMIT_PENALTY_COFF 1e5
#define SINGLE_OPT_ITER_NUM 100
#define MAX_OPT_TIMES 1
#define CANDIDATE_NUM 1
#define CONTOUR_DIM 180
#define CONTOUR_RANGE 3.5

#include <ros/ros.h>

struct Position {
    double x_;
    double y_;
};

struct Vector2d {
    double x_;
    double y_;
};

struct Pose {
    double x_;
    double y_;
    double theta_;
};

struct PathPose {
    double x_;
    double y_;
    double theta_;
    double dist_;
};

struct Action {
    double v_;
    double w_;
};

struct State {
    double x_;
    double y_;
    double theta_;
    double v_;
    double w_;
};

struct Acceleration {
    double la_;
    double aa_;
};

struct ObstacleInfo {
    double x_;
    double y_;
    double vx_;
    double vy_;
    double radius_;
};

struct CircleRegion {
    Position center_;
    double radius_;
};

struct Line2d {
    Vector2d norm_;
    double b_;
};

#endif  // COMMON_HPP_
