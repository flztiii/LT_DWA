/*
    Copyright [2020] Jian ZhiQiang
*/

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include "utilities/Tools.hpp"
#include "common/Point.hpp"
#include "common/Path.hpp"
#include "utilities/PathGenerator.h"

namespace VisualizationMethods {

// 构建颜色
std_msgs::ColorRGBA color(double r, double g, double b, double a);

// 将路径转化为marker
visualization_msgs::Marker visualizeCurvesToMarker(const PathPlanningUtilities::Curve &curve, const std_msgs::ColorRGBA &color, int id, double fontsize = 0.1);

visualization_msgs::Marker visualizeCurvesToMarker(const PathPlanningUtilities::Path &curve, const std_msgs::ColorRGBA &color, int id, double fontsize = 0.1);

visualization_msgs::Marker visualizeCurvesToMarker(const std::vector<PathPlanningUtilities::CoordinationPoint> &curve, const std_msgs::ColorRGBA &color, int id, double fontsize = 0.1);

// 将矩形框转为marker
visualization_msgs::Marker visualizeRectToMarker(double position_x, double position_y, double theta, double width, double length, double center_scale, const std_msgs::ColorRGBA &color, int id, double fontsize = 0.1);

// 将文本转为为marker
visualization_msgs::Marker visualizeStringToMarker(const std::string &text, double position_x, double position_y, const std_msgs::ColorRGBA &color, int id, double fontsize = 0.1);

// 删除可视化marker
visualization_msgs::Marker visualizedeleteMarker(int id);

// 删除从起始id开始的全部可视化marker
visualization_msgs::Marker visualizedeleteAllMarker(int start_id);

// 在特定位置生成圆球marker
visualization_msgs::Marker visualizeSphere(double position_x, double position_y, double radius, std_msgs::ColorRGBA color, int id);

};

#endif