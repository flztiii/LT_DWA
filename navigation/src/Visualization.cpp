# include "utilities/Visualization.hpp"

// 构建颜色
std_msgs::ColorRGBA VisualizationMethods::color(double r, double g, double b, double a) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

// 将路径转化为marker
visualization_msgs::Marker VisualizationMethods::visualizeCurvesToMarker(const PathPlanningUtilities::Curve &curve, const std_msgs::ColorRGBA &color, int id, double fontsize) {
    visualization_msgs::Marker curve_marker;
    curve_marker.header.frame_id = "world";
    curve_marker.header.stamp = ros::Time::now();
    curve_marker.type = visualization_msgs::Marker().LINE_STRIP;
    curve_marker.color = color;
    curve_marker.id = id;
    geometry_msgs::Vector3 v3c;
    v3c.x = fontsize;
    curve_marker.scale = v3c;
    for (size_t i = 0; i < curve.size(); i++) {
        PathPlanningUtilities::CurvePoint curve_point = curve[i];
        geometry_msgs::Point point;
        point.x = curve_point.position_.x_;
        point.y = curve_point.position_.y_;
        curve_marker.points.push_back(point);
    }
    return curve_marker;
}

// 将路径转化为marker
visualization_msgs::Marker VisualizationMethods::visualizeCurvesToMarker(const PathPlanningUtilities::Path &curve, const std_msgs::ColorRGBA &color, int id, double fontsize) {
    visualization_msgs::Marker curve_marker;
    curve_marker.header.frame_id = "world";
    curve_marker.header.stamp = ros::Time::now();
    curve_marker.type = visualization_msgs::Marker().LINE_STRIP;
    curve_marker.color = color;
    curve_marker.id = id;
    geometry_msgs::Vector3 v3c;
    v3c.x = fontsize;
    curve_marker.scale = v3c;
    for (size_t i = 0; i < curve.size(); i++) {
        PathPlanningUtilities::Point2f curve_point = curve[i];
        geometry_msgs::Point point;
        point.x = curve_point.x_;
        point.y = curve_point.y_;
        curve_marker.points.push_back(point);
    }
    return curve_marker;
}

// 将路径转化为marker
visualization_msgs::Marker VisualizationMethods::visualizeCurvesToMarker(const std::vector<PathPlanningUtilities::CoordinationPoint> &curve, const std_msgs::ColorRGBA &color, int id, double fontsize) {
    visualization_msgs::Marker curve_marker;
    curve_marker.header.frame_id = "world";
    curve_marker.header.stamp = ros::Time::now();
    curve_marker.type = visualization_msgs::Marker().LINE_STRIP;
    curve_marker.color = color;
    curve_marker.id = id;
    geometry_msgs::Vector3 v3c;
    v3c.x = fontsize;
    curve_marker.scale = v3c;
    for (size_t i = 0; i < curve.size(); i++) {
        PathPlanningUtilities::CoordinationPoint curve_point = curve[i];
        geometry_msgs::Point point;
        point.x = curve_point.worldpos_.position_.x_;
        point.y = curve_point.worldpos_.position_.y_;
        curve_marker.points.push_back(point);
    }
    return curve_marker;
}

// 将矩形框转为marker
visualization_msgs::Marker VisualizationMethods::visualizeRectToMarker(double position_x, double position_y, double theta, double width, double length, double center_scale, const std_msgs::ColorRGBA &color, int id, double fontsize) {
    visualization_msgs::Marker rect_marker;
    rect_marker.header.frame_id = "world";
    rect_marker.header.stamp = ros::Time::now();
    rect_marker.type = visualization_msgs::Marker().LINE_STRIP;
    rect_marker.color = color;
    rect_marker.id = id;
    geometry_msgs::Vector3 v3c;
    v3c.x = fontsize;
    rect_marker.scale = v3c;
    geometry_msgs::Point point_1, point_2, point_3, point_4;
    point_1.x = position_x + length*center_scale*cos(theta) - width/2.0*sin(theta);
    point_1.y = position_y + length*center_scale*sin(theta) + width/2.0*cos(theta);
    rect_marker.points.push_back(point_1);
    point_2.x = position_x + length*center_scale*cos(theta) + width/2.0*sin(theta);
    point_2.y = position_y + length*center_scale*sin(theta) - width/2.0*cos(theta);
    rect_marker.points.push_back(point_2);
    point_3.x = position_x - length*(1.0 - center_scale)*cos(theta) + width/2.0*sin(theta);
    point_3.y = position_y - length*(1.0 - center_scale)*sin(theta) - width/2.0*cos(theta);
    rect_marker.points.push_back(point_3);
    point_4.x = position_x - length*(1.0 - center_scale)*cos(theta) - width/2.0*sin(theta);
    point_4.y = position_y - length*(1.0 - center_scale)*sin(theta) + width/2.0*cos(theta);
    rect_marker.points.push_back(point_4);
    rect_marker.points.push_back(point_1);
    return rect_marker;
}

// 将文本转为为marker
visualization_msgs::Marker VisualizationMethods::visualizeStringToMarker(const std::string &text, double position_x, double position_y, const std_msgs::ColorRGBA &color, int id, double fontsize) {
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "world";
    text_marker.header.stamp = ros::Time::now();
    text_marker.type = visualization_msgs::Marker().TEXT_VIEW_FACING;
    text_marker.color = color;
    text_marker.id = id;
    text_marker.pose.position.x = position_x;
    text_marker.pose.position.y = position_y;
    text_marker.pose.position.z = 0;
    text_marker.pose.orientation.w = 1;
    text_marker.scale.x = fontsize;
    text_marker.scale.y = fontsize;
    text_marker.scale.z = fontsize;
    text_marker.text = text;
    return text_marker;
}

// 删除可视化marker
visualization_msgs::Marker VisualizationMethods::visualizedeleteMarker(int id) {
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "world";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.action = visualization_msgs::Marker::DELETE;
    delete_marker.id = id;
    return delete_marker;
}

// 删除从起始id开始的全部可视化marker
visualization_msgs::Marker VisualizationMethods::visualizedeleteAllMarker(int start_id) {
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "world";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_marker.id = start_id;
    return delete_marker;
}

// 在特定位置生成圆球marker
visualization_msgs::Marker VisualizationMethods::visualizeSphere(double position_x, double position_y, double radius, std_msgs::ColorRGBA color, int id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.id = id;
    marker.color = color;
    marker.pose.position.x = position_x;
    marker.pose.position.y = position_y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = 0.01;
    return marker;
}
