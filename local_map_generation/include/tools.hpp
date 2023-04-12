/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef TOOLS_HPP_
#define TOOLS_HPP_

#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>

struct Point2f
{
    double x_;
    double y_;
};

struct CurvePoint
{   
    Point2f position_;		///< Position of point on curve, Unit = meter/centimeter/...
    double theta_;			///< Tengential orientation of point, rad 
    double kappa_;			///< Curvature of point, rad/Unit,positive for anticlockwise & negative for clockwise
};

struct StampedPoint
{
    CurvePoint pose_;
    double time_;
};

struct Circle {
    Point2f center_;
    double radius_;
};

// 占用区域基本单位——矩形
class Rectangle {
 public:
    double center_x_;  // 矩形中心点x
    double center_y_;  // 矩形中心点y
    double width_;  // 矩形宽度
    double length_;  // 矩形长度
    double rotation_;  // 矩形朝向
    std::vector<Point2f> points_;  // 矩形角点

    // 构造函数
    Rectangle(){};

    // 析构函数
    ~Rectangle(){};

    // 构造函数
    Rectangle(double center_x, double center_y, double rotation, double width, double length) {
        this->center_x_ = center_x;
        this->center_y_ = center_y;
        this->rotation_ = rotation;
        this->width_ = width;
        this->length_ = length;
        this->generatePoints();
    }

    // 求解矩形四个角点
    std::vector<Point2f> getRectanglePoints(double center_x, double center_y, double rotation, double width, double length) {
        std::vector<Point2f> points;
        points.resize(4);
        Point2f point_1, point_2, point_3, point_4;
        point_1.x_ = center_x + length*0.5*cos(rotation) - width*0.5*sin(rotation);
        point_1.y_ = center_y + length*0.5*sin(rotation) + width*0.5*cos(rotation);
        points[0] = point_1;
        point_2.x_ = center_x + length*0.5*cos(rotation) + width*0.5*sin(rotation);
        point_2.y_ = center_y + length*0.5*sin(rotation) - width*0.5*cos(rotation);
        points[1] = point_2;
        point_3.x_ = center_x - length*0.5*cos(rotation) + width*0.5*sin(rotation);
        point_3.y_ = center_y - length*0.5*sin(rotation) - width*0.5*cos(rotation);
        points[2] = point_3;
        point_4.x_ = center_x - length*0.5*cos(rotation) - width*0.5*sin(rotation);
        point_4.y_ = center_y - length*0.5*sin(rotation) + width*0.5*cos(rotation);
        points[3] = point_4;
        return points;
    }

 private:
    // 计算矩形角点
    void generatePoints() {
        this->points_ = this->getRectanglePoints(this->center_x_, this->center_y_, this->rotation_, this->width_, this->length_);
    }
};

// 计算坐标系转换
Point2f calcNewCoordinationPosition(const CurvePoint &new_coordination_origin, const Point2f &position) {
    Point2f new_position;
    new_position.x_ = (position.x_ - new_coordination_origin.position_.x_) * cos(new_coordination_origin.theta_) + (position.y_ - new_coordination_origin.position_.y_) * sin(new_coordination_origin.theta_);
    new_position.y_ = -(position.x_ - new_coordination_origin.position_.x_) * sin(new_coordination_origin.theta_) + (position.y_ - new_coordination_origin.position_.y_) * cos(new_coordination_origin.theta_);
    return new_position;
}

// 计算坐标系反转换
Point2f calcOldCoordinationPosition(const CurvePoint &new_coordination_origin, const Point2f &position) {
    Point2f old_position;
    old_position.x_ = new_coordination_origin.position_.x_ + position.x_ * cos(new_coordination_origin.theta_) - position.y_ * sin(new_coordination_origin.theta_);
    old_position.y_ = new_coordination_origin.position_.y_ + position.x_ * sin(new_coordination_origin.theta_) + position.y_ * cos(new_coordination_origin.theta_);
    return old_position;
}


/***************** Mat转vector **********************/
template<typename _Tp>
std::vector<_Tp> convertMat2Vector(const cv::Mat &mat)
{
	return (std::vector<_Tp>)(mat.reshape(1, 1));  //通道数不变，按行转为一行
}
 
/****************** vector转Mat *********************/
template<typename _Tp>
cv::Mat convertVector2Mat(std::vector<_Tp> v, int channels, int rows)
{
	cv::Mat mat = cv::Mat(v);  //将vector变成单列的mat
	cv::Mat dest = mat.reshape(channels, rows).clone();  //PS：必须clone()一份，否则返回出错
	return dest;
}

// 重置文件夹
void resetDirectory(const std::string &file_path) {
    if (access(file_path.c_str(), 0) == -1) {
        // 如果文件夹不存在则创建文件夹
        boost::filesystem::create_directories(file_path.c_str());
    } else {
        // 删除文件夹
        boost::filesystem::remove_all(file_path.c_str());
        boost::filesystem::create_directories(file_path.c_str());
    }
}

#endif
