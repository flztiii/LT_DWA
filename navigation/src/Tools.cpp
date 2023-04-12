/*
    Copyright [2019] Jian ZhiQiang
*/
#include "utilities/Tools.hpp"

// 判断一个double类型的数是否等于0
bool Tools::isZero(double value) {
    if (std::fabs(value) <= EPS) {
        return true;
    } else {
        return false;
    }
}

// 判断前一个double是否大于后一个double
bool Tools::isLarge(double value_1, double value_2) {
    if (value_1 > value_2 && std::fabs(value_1 - value_2) > EPS) {
        return true;
    } else {
        return false;
    }
}

// 判断前一个double是否小于后一个double
bool Tools::isSmall(double value_1, double value_2) {
    if (value_1 < value_2 && std::fabs(value_1 - value_2) > EPS) {
        return true;
    } else {
        return false;
    }
}

// 判断前一个double是否等于后一个double
bool Tools::isEqual(double value_1, double value_2) {
    if (std::fabs(value_1 - value_2) <= EPS) {
        return true;
    } else {
        return false;
    }
}

// 计算acos
double Tools::safeAcos (double x) {
    if (Tools::isSmall(x, -0.999)) {
        x = -0.999;
    } else if (Tools::isLarge(x, 0.999)) {
        x = 0.999;
    }
    return acos(x) ;
}

// 计算坐标系转换
PathPlanningUtilities::Point2f Tools::calcNewCoordinationPosition(const PathPlanningUtilities::CurvePoint &new_coordination_origin, const PathPlanningUtilities::Point2f &position) {
    PathPlanningUtilities::Point2f new_position;
    new_position.x_ = (position.x_ - new_coordination_origin.position_.x_) * cos(new_coordination_origin.theta_) + (position.y_ - new_coordination_origin.position_.y_) * sin(new_coordination_origin.theta_);
    new_position.y_ = -(position.x_ - new_coordination_origin.position_.x_) * sin(new_coordination_origin.theta_) + (position.y_ - new_coordination_origin.position_.y_) * cos(new_coordination_origin.theta_);
    return new_position;
}

// 计算坐标系转换
PathPlanningUtilities::Point2f Tools::calcNewCoordinationPosition(const PathPlanningUtilities::VehicleState &new_coordination_origin, const PathPlanningUtilities::Point2f &position) {
    PathPlanningUtilities::Point2f new_position;
    new_position.x_ = (position.x_ - new_coordination_origin.position_.x_) * cos(new_coordination_origin.theta_) + (position.y_ - new_coordination_origin.position_.y_) * sin(new_coordination_origin.theta_);
    new_position.y_ = -(position.x_ - new_coordination_origin.position_.x_) * sin(new_coordination_origin.theta_) + (position.y_ - new_coordination_origin.position_.y_) * cos(new_coordination_origin.theta_);
    return new_position;
}

// 计算坐标系反转换
PathPlanningUtilities::Point2f Tools::calcOldCoordinationPosition(const PathPlanningUtilities::CurvePoint &new_coordination_origin, const PathPlanningUtilities::Point2f &position) {
    PathPlanningUtilities::Point2f old_position;
    old_position.x_ = new_coordination_origin.position_.x_ + position.x_ * cos(new_coordination_origin.theta_) - position.y_ * sin(new_coordination_origin.theta_);
    old_position.y_ = new_coordination_origin.position_.y_ + position.x_ * sin(new_coordination_origin.theta_) + position.y_ * cos(new_coordination_origin.theta_);
    return old_position;
}

// 计算参考线离当前位置最近的点
size_t Tools::findNearestPositionIndexInCoordination(const std::vector<PathPlanningUtilities::CoordinationPoint> &coordination_points, const PathPlanningUtilities::Point2f &current_position, size_t start_index) {
    //首先选出可能的下标
    size_t current_position_index = coordination_points.size() - 1;
    for (size_t i = start_index; i < coordination_points.size(); i++) {
        PathPlanningUtilities::Point2f local_position = calcNewCoordinationPosition(coordination_points[i].worldpos_, current_position);
        if (isSmall(local_position.x_, 0.0)) {
            // 当前位置在路径起始点后面
            current_position_index = i;
            break;
        }
    }
    return current_position_index;
}

size_t Tools::findNearestPositionIndexInCurve(const PathPlanningUtilities::Curve &curve, const PathPlanningUtilities::Point2f &current_position, size_t start_index)  {
    //首先选出可能的下标
    size_t current_position_index = curve.size() - 1;
    for (size_t i = start_index; i < curve.size(); i++) {
        PathPlanningUtilities::Point2f local_position = calcNewCoordinationPosition(curve[i], current_position);
        if (isSmall(local_position.x_, 0.0)) {
            // 当前位置在路径起始点后面
            current_position_index = i;
            break;
        }
    }
    return current_position_index;
}

// 计算曲线的最大曲率点
size_t Tools::calcMaxKappaIndexForCurve(const PathPlanningUtilities::Curve &curve) {
    double max_kappa = 0.0;
    size_t index = 0;
    for (size_t i = 0; i < curve.size(); i++) {
        double kappa = std::fabs(curve[i].kappa_);
        if (!isSmall(kappa, max_kappa)) {
            max_kappa = kappa;
            index = i;
        }
    }
    return index;
}

// 计算曲线的最大曲率
double Tools::calcMaxKappaForCurve(const PathPlanningUtilities::Curve &curve) {
    size_t index = calcMaxKappaIndexForCurve(curve);
    return curve[index].kappa_;
}

// 创建日志文件夹
void Tools::resetLogFile(const std::string &file_path) {
    if (access(file_path.c_str(), 0) == -1) {
        // 如果文件夹不存在则创建文件夹
        mkdir(file_path.c_str(), S_IRWXU|S_IRGRP|S_IXGRP|S_IROTH|S_IXOTH);
    }
}

//返回当前时刻
std::string Tools::returnCurrentTimeAndDate() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d%X");
    return ss.str();
}
