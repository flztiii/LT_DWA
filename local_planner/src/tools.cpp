#include "util/tools.hpp"

// 判断一个double类型的数是否等于0
bool Tools::isZero(double value) {
    if (std::abs(value) <= EPS) {
        return true;
    } else {
        return false;
    }
}

// 判断前一个double是否大于后一个double
bool Tools::isLarge(double value_1, double value_2) {
    if (value_1 > value_2 && std::abs(value_1 - value_2) > EPS) {
        return true;
    } else {
        return false;
    }
}

// 判断前一个double是否小于后一个double
bool Tools::isSmall(double value_1, double value_2) {
    if (value_1 < value_2 && std::abs(value_1 - value_2) > EPS) {
        return true;
    } else {
        return false;
    }
}

// 判断前一个double是否等于后一个double
bool Tools::isEqual(double value_1, double value_2) {
    if (std::abs(value_1 - value_2) <= EPS) {
        return true;
    } else {
        return false;
    }
}

// 角度范围
double Tools::pi2Pi(double angle) {
    while (isLarge(angle, M_PI)) {
        angle = angle - 2.0 * M_PI;
    }
    while (!isLarge(angle, - M_PI)) {
        angle = angle + 2.0 * M_PI;
    }
    return angle;
}

// 范围截断
double Tools::clip(double value, double min_value, double max_value) {
    if (isSmall(value, min_value)) {
        value = min_value;
    } else if (isLarge(value, max_value)) {
        value = max_value;
    }
    return value;
};

// 计算坐标系转换
Position Tools::calcNewCoordinationPosition(const Pose &new_coordination_origin, const Position &position) {
    Position new_position;
    new_position.x_ = (position.x_ - new_coordination_origin.x_) * cos(new_coordination_origin.theta_) + (position.y_ - new_coordination_origin.y_) * sin(new_coordination_origin.theta_);
    new_position.y_ = -(position.x_ - new_coordination_origin.x_) * sin(new_coordination_origin.theta_) + (position.y_ - new_coordination_origin.y_) * cos(new_coordination_origin.theta_);
    return new_position;
}

// 计算坐标系反转换
Position Tools::calcOldCoordinationPosition(const Pose &new_coordination_origin, const Position &position) {
    Position old_position;
    old_position.x_ = new_coordination_origin.x_ + position.x_ * cos(new_coordination_origin.theta_) - position.y_ * sin(new_coordination_origin.theta_);
    old_position.y_ = new_coordination_origin.y_ + position.x_ * sin(new_coordination_origin.theta_) + position.y_ * cos(new_coordination_origin.theta_);
    return old_position;
}
