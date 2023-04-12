/*
    Copyright [2022] Jian ZhiQiang
*/

#ifndef TOOLS_HPP_
#define TOOLS_HPP_

#include <cmath>
#include <vector>
#include <deque>
#include <opencv2/opencv.hpp>
#include "util/common.hpp"

#define EPS 1.0e-6  // 浮点数比较的精度

namespace Tools {

// 固定长度队列
template <typename T, int MaxLen>
class FixedQueue : public std::deque<T> {
public:
    void push(const T& value) {
        if (this->size() == MaxLen) {
           this->pop_front();
        }
        this->push_back(value);
    }
};

// 判断一个double类型的数是否等于0
bool isZero(double value);

// 判断前一个double是否大于后一个double
bool isLarge(double value_1, double value_2);

// 判断前一个double是否小于后一个double
bool isSmall(double value_1, double value_2);

// 判断前一个double是否等于后一个double
bool isEqual(double value_1, double value_2);

// 角度范围
double pi2Pi(double angle);

// 范围截断
double clip(double value, double min_value, double max_value);

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

// 计算坐标系转换
Position calcNewCoordinationPosition(const Pose &new_coordination_origin, const Position &position);

// 计算坐标系反转换
Position calcOldCoordinationPosition(const Pose &new_coordination_origin, const Position &position);

// 采样函数
template<typename T>
std::vector<double> linspace(T start_in, T end_in, int num_in) {

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

}; // namespace Tools

#endif  // TOOLS_HPP_
