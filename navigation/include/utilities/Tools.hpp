#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include "common/Point.hpp"
#include "common/Path.hpp"
#include "utilities/PathGenerator.h"

#define PI 3.141592653589793238462643383279  // pi
#define EPS 1.0e-8  // 浮点数比较的精度

namespace Tools{
    // 判断一个double类型的数是否等于0
    bool isZero(double value);

    // 判断前一个double是否大于后一个double
    bool isLarge(double value_1, double value_2);

    // 判断前一个double是否小于后一个double
    bool isSmall(double value_1, double value_2);

    // 判断前一个double是否等于后一个double
    bool isEqual(double value_1, double value_2);

    // 计算acos
    double safeAcos(double x);

    // 采样函数
    template<typename T>
    std::vector<double> linspace(T start_in, T end_in, int num_in)
    {

        std::vector<double> linspaced;

        double start = static_cast<double>(start_in);
        double end = static_cast<double>(end_in);
        double num = static_cast<double>(num_in);

        if (num == 0) { return linspaced; }
        if (num == 1) {
            linspaced.push_back(start);
            return linspaced;
        }

        double delta = (end - start) / (num - 1);

        for(int i=0; i < num-1; ++i){
            linspaced.push_back(start + delta * i);
        }
        linspaced.push_back(end);
        return linspaced;
    }

    // 计算坐标系转换
    PathPlanningUtilities::Point2f calcNewCoordinationPosition(const PathPlanningUtilities::VehicleState &new_coordination_origin, const PathPlanningUtilities::Point2f &position);

    // 计算坐标系转换
    PathPlanningUtilities::Point2f calcNewCoordinationPosition(const PathPlanningUtilities::CurvePoint &new_coordination_origin, const PathPlanningUtilities::Point2f &position);

    // 计算坐标系转换
    PathPlanningUtilities::Point2f calcOldCoordinationPosition(const PathPlanningUtilities::CurvePoint &new_coordination_origin, const PathPlanningUtilities::Point2f &position);

    // 计算参考线离当前位置最近的点
    size_t findNearestPositionIndexInCoordination(const std::vector<PathPlanningUtilities::CoordinationPoint> &coordination_points, const PathPlanningUtilities::Point2f &current_position, size_t start_index = 0);

    // 计算路径离当前位置最近的点
    size_t findNearestPositionIndexInCurve(const PathPlanningUtilities::Curve &curve, const PathPlanningUtilities::Point2f &current_position, size_t start_index = 0);

    // 计算曲线的最大曲率点
    size_t calcMaxKappaIndexForCurve(const PathPlanningUtilities::Curve &curve);

    // 计算曲线的最大曲率
    double calcMaxKappaForCurve(const PathPlanningUtilities::Curve &curve);

    // 创建日志文件夹
    void resetLogFile(const std::string &file_path);

    //返回当前时刻
    std::string returnCurrentTimeAndDate();
};
