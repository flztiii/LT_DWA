#pragma once

#include <algorithm>
#include "common/Point.hpp"
#include "utilities/Tools.hpp"

namespace PathPlanningUtilities
{
    // 线段类
    class LineSegment {
    public:
        // 构造函数
        LineSegment(const PathPlanningUtilities::Point2f &point_start, const PathPlanningUtilities::Point2f &point_end){
            this->point_start_ = point_start;
            this->point_end_ = point_end;
            this->vector_ = point_end - point_start;
        };

        LineSegment(){};

        // 析构函数
        ~LineSegment(){};

        // 获取线段起点
        PathPlanningUtilities::Point2f getStartPoint() const {
            return this->point_start_;
        }

        // 获取线段终点
        PathPlanningUtilities::Point2f getEndPoint() const {
            return this->point_end_;
        }

        // 获取方向向量
        PathPlanningUtilities::Vector2f getVector() const {
            return this->vector_;
        }

        // 获取线段长度
        double length() const {
            return PathPlanningUtilities::getLength(this->vector_);
        } 

        // 判断点是否在线段上
        bool isPointOnLineSegment(double point_x, double point_y) const {
            if (std::min(this->point_start_.x_, this->point_end_.x_) <= point_x && point_x <= std::max(this->point_start_.x_, this->point_end_.x_) && std::min(this->point_start_.y_, this->point_end_.y_) <= point_y && point_y <= std::max(this->point_start_.y_, this->point_start_.y_)) {
                PathPlanningUtilities::Vector2f sp = {point_x - this->point_start_.x_, point_y - this->point_start_.y_};
                if (Tools::isZero(PathPlanningUtilities::cross(sp, this->vector_))) {
                    return true;
                } else {
                    return false;
                }
            } else {
                return false;
            }
        }

        // 求点到直线最短距离
        double pointToLineSegmentDistance(double point_x, double point_y) const {
            PathPlanningUtilities::Vector2f sp = {point_x - this->point_start_.x_, point_y - this->point_start_.y_};
            PathPlanningUtilities::Vector2f ep = {point_x - this->point_end_.x_, point_y - this->point_end_.y_};

            double r = PathPlanningUtilities::dot(sp, this->vector_) / PathPlanningUtilities::dot(this->vector_, this->vector_);
            if (Tools::isSmall(r, 0.0) || Tools::isEqual(r, 0.0)) {
                return PathPlanningUtilities::norm(sp);
            } else if (Tools::isLarge(r, 1.0) || Tools::isEqual(r, 1.0)) {
                return PathPlanningUtilities::norm(ep);
            } else {
                return PathPlanningUtilities::norm(sp - this->vector_ * r);
            }
        }

        // 判断两个线段是否相交
        bool isInteract(const PathPlanningUtilities::LineSegment& line_segment) const {
            // 相交有两种情况，一种是端点在线段上，一种是成'X'形状
            // 首先判断第一种，是不是在线段上
            // if (this->isPointOnLineSegment(line_segment.getStartPoint().x_, line_segment.getStartPoint().y_)) {
            //     return true;
            // } else if (this->isPointOnLineSegment(line_segment.getEndPoint().x_, line_segment.getEndPoint().y_)) {
            //     return true;
            // } else if (line_segment.isPointOnLineSegment(this->point_start_.x_, this->point_start_.y_)) {
            //     return true;
            // } else if (line_segment.isPointOnLineSegment(this->point_end_.x_, this->point_end_.y_)) {
            //     return true;
            // } else {
            //     // 判断第二种情况
            //     PathPlanningUtilities::Vector2f p1q1 = line_segment.getStartPoint() - this->point_start_;
            //     PathPlanningUtilities::Vector2f p1q2 = line_segment.getEndPoint() - this->point_start_;

            //     PathPlanningUtilities::Vector2f q1p1 = this->point_start_ - line_segment.getStartPoint();
            //     PathPlanningUtilities::Vector2f q1p2 = this->point_end_ - line_segment.getStartPoint();

            //     if (Tools::isSmall(PathPlanningUtilities::cross(this->vector_, p1q1) * PathPlanningUtilities::cross(this->vector_, p1q2), 0.0) && Tools::isSmall(PathPlanningUtilities::cross(line_segment.getVector(), q1p1) * PathPlanningUtilities::cross(line_segment.getVector(), q1p2), 0.0)) {
            //         return true;
            //     } else {
            //         return false;
            //     }
            // }
            double delta = PathPlanningUtilities::determinant(this->point_end_.x_-this->point_start_.x_, line_segment.point_start_.x_-line_segment.point_end_.x_, this->point_end_.y_-this->point_start_.y_, line_segment.point_start_.y_-line_segment.point_end_.y_);
            if (Tools::isZero(delta)) {
                // delta=0，表示两线段重合或平行
                return false;
            }
            double namenda = PathPlanningUtilities::determinant(line_segment.point_start_.x_-this->point_start_.x_, line_segment.point_start_.x_-line_segment.point_end_.x_, line_segment.point_start_.y_-this->point_start_.y_, line_segment.point_start_.y_-line_segment.point_end_.y_) / delta;
            if (Tools::isLarge(namenda, 1.0) || Tools::isSmall(namenda, 0.0)) {
                return false;
            }
            double miu = PathPlanningUtilities::determinant(this->point_end_.x_-this->point_start_.x_, line_segment.point_start_.x_-this->point_start_.x_, this->point_end_.y_-this->point_start_.y_, line_segment.point_start_.y_-this->point_start_.y_) / delta;
            if (Tools::isLarge(miu, 1.0) || Tools::isSmall(miu, 0.0)) {
                return false;
            }
            return true;
        }

        // 计算两线段距离
        double lineSegmentDistance(const PathPlanningUtilities::LineSegment& line_segment, bool judge_interact=true) {
            // 是否需要先判断相交
            if (judge_interact) {
                // 需要判断
                if (this->isInteract(line_segment)) {
                    return 0.0;
                } else {
                    std::vector<double> distances;
                    distances.push_back(this->pointToLineSegmentDistance(line_segment.getStartPoint().x_, line_segment.getStartPoint().y_));
                    distances.push_back(this->pointToLineSegmentDistance(line_segment.getEndPoint().x_, line_segment.getEndPoint().y_));
                    distances.push_back(line_segment.pointToLineSegmentDistance(this->point_start_.x_, this->point_start_.y_));
                    distances.push_back(line_segment.pointToLineSegmentDistance(this->point_end_.x_, this->point_end_.y_));
                    auto smallest = std::min_element(std::begin(distances), std::end(distances));
                    return *smallest;
                }
            } else {
                // 不进行判断（效率更高，但一旦相交就会出错）
                std::vector<double> distances;
                distances.push_back(this->pointToLineSegmentDistance(line_segment.getStartPoint().x_, line_segment.getStartPoint().y_));
                distances.push_back(this->pointToLineSegmentDistance(line_segment.getEndPoint().x_, line_segment.getEndPoint().y_));
                distances.push_back(line_segment.pointToLineSegmentDistance(this->point_start_.x_, this->point_start_.y_));
                distances.push_back(line_segment.pointToLineSegmentDistance(this->point_end_.x_, this->point_end_.y_));
                auto smallest = std::min_element(std::begin(distances), std::end(distances));
                return *smallest;
            }
        }

    private:
        PathPlanningUtilities::Point2f point_start_;
        PathPlanningUtilities::Point2f point_end_;
        PathPlanningUtilities::Vector2f vector_;
    };

};
