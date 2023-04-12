#include "utilities/PathUtilities.h"
#include <sstream>
#include <algorithm>

namespace PathPlanningUtilities
{
	void PathUtilities::pathSmooth(const Path & raw_path, Path & smoothed_path, double& average_distance, const size_t iteration)
	{
		if (iteration == 0)
		{
			smoothed_path = raw_path;
			return;
		}
		size_t length = raw_path.size();
		std::vector<Point2f> iteration_data[2];
		std::vector<double> acc_distance;
		iteration_data[0] = raw_path;
		iteration_data[1].resize(length);
		acc_distance.resize(length);

		acc_distance[0] = 0;

		for (size_t iter = 0; iter < iteration; iter++)
		{
			const std::vector<Point2f>& src = iteration_data[iter % 2];
			std::vector<Point2f>& target = iteration_data[(iter + 1) % 2];
			for (size_t i = 1; i < length; i++)
				acc_distance[i] = acc_distance[i - 1] + calcDistance(src[i - 1], src[i]);
			average_distance = acc_distance[length - 1] / (length - 1);
			target[0] = src[0];
			target[length - 1] = src[length - 1];
			size_t i_src = 1;
			for (size_t i_target = 1; i_target < length - 1; i_target++)
			{
				double total_distance = average_distance * i_target;
				while (!((acc_distance[i_src - 1] <= total_distance) && (acc_distance[i_src] > total_distance)))
					i_src++;
				double eta = (total_distance - acc_distance[i_src - 1]) / (acc_distance[i_src] - acc_distance[i_src - 1]);
				target[i_target] = findMiddlePoint(src[i_src - 1], src[i_src], eta);
			}
		}
		smoothed_path = iteration_data[iteration % 2];
	}

	void PathUtilities::getCurve(const Path & raw_path, Curve & curve, const size_t window_size)
	{
		CurvePoint cp;
		Point2f p;
		curve.resize(raw_path.size());
		for (size_t i = 0; i < raw_path.size(); i++){
			p = raw_path[i];
			cp.position_.x_ = p.x_;
			cp.position_.y_ = p.y_;
			cp.theta_ = calcTheta(raw_path, i, window_size);
			curve[i]=cp;
		}
		for(size_t i = 0; i < raw_path.size(); i++){
			curve[i].kappa_ = calcKappa(curve, i, window_size);
		}
	}

	void PathUtilities::getBound(const BoundedCurve& bcurve, Path& left_bound, Path& right_bound)
	{
		left_bound.resize(0);
		right_bound.resize(0);
		for(size_t i=0; i<bcurve.size(); i++)
		{
			const BoundedCurvePoint &bp = bcurve[i];
			Point2f lp,rp;
			// if(bp.left_distance_ <= 0.01 || bp.right_distance_ <= 0.01) continue;
			
			double theta = bp.center_point_.theta_;
			double theta_left = theta +  M_PI / 2.0f;

			Vector2f unit_vec;
			unit_vec.x_ = cos(theta_left);
			unit_vec.y_ = sin(theta_left);

			lp = bp.center_point_.position_ + unit_vec * bp.left_distance_ ;
			rp = bp.center_point_.position_ + unit_vec * (-bp.right_distance_);
			left_bound.push_back(lp);
			right_bound.push_back(rp);
		}
	}

	
	size_t PathUtilities::FindNearestPoint(const Path & path, const Point2f & point)
	{
		double distance;
		double min_distance = calcDistance(path[0],point);
		size_t result=0;
		for (size_t i = 0; i < path.size(); i++){
			//std::cout<<"position:"<<point.x_<<" "<<point.y_<<std::endl;
			//std::cout<<"point:"<<path[i].x_<<" "<<path[i].y_<<std::endl;
			distance=calcDistance(point, path[i]);
			if (distance < min_distance){
				result = i;
				min_distance = distance;
			}
		}
		return result;
	}
	
	size_t PathUtilities::findNearestPoint(const Curve & curve, const Point2f & point, const double theta0)
	{
		double distance;
		double theta;
		double max_distance = 1000000.0;
		size_t result = 0;
		size_t flag = 0;
		std::vector<double> distance_set;
		std::vector<size_t> choose_index;
		distance_set.resize(curve.size());
		for (size_t i = 0; i < curve.size(); i++) {
			theta = fabs(theta0-curve[i].theta_);
			if (theta > M_PI) {
				theta = 2 * M_PI - theta;
			}
			if (theta<M_PI_4) {
				distance = calcDistance(point, curve[i].position_);
				distance_set[i] = distance;
			}
			else {
				distance_set[i] = max_distance;
			}
		}
		distance = *min_element(distance_set.begin(), distance_set.end());;
		distance = *min_element(distance_set.begin(), distance_set.end());
		for (size_t i = 0; i < distance_set.size(); i++) {
			if (distance_set[i] == distance) {
				choose_index.push_back(i);
			}
		}
		result = *max_element(choose_index.begin(), choose_index.end());
		flag = *min_element(choose_index.begin(), choose_index.end());
		if (flag == 0 && result == curve.size() - 1) {
			result = 0;
		}
		return result;
	}

	double PathUtilities::calcTheta(const Path& raw_path, const size_t index, const size_t window_size) {
		size_t center;
		double theta;
		std::vector<double> theta_set;
		if (raw_path.size() < window_size * 2 + 1)
			throw "Window size is too large";
		if (index < window_size) {
			center = window_size;
		}
		else if (index > raw_path.size() - 1 - window_size) {
			center = raw_path.size() - 1 - window_size;
		}
		else {
			center = index;
		}
		for (size_t i = window_size; i > 0; i--)
		{
			theta = calcIncludedAngle(raw_path[center + window_size] - raw_path[center - window_size], 
					raw_path[center] - raw_path[center - i]);
			theta_set.push_back(theta);
		}
		for (size_t i = 1; i <= window_size; i++)
		{
			theta = calcIncludedAngle(raw_path[center + window_size] - raw_path[center - window_size], 
					raw_path[center + i] - raw_path[center]);
			theta_set.push_back(theta);
		}
		std::sort(theta_set.begin(), theta_set.end());
		theta = (theta_set[window_size]+theta_set[window_size+1])/2.0f;
		theta += calcDeflection(raw_path[center - window_size], raw_path[center + window_size]);
		theta=rectifyAngle(theta);
		return theta;
	}
	
	double PathUtilities::calcKappa(const Curve& raw_path, const size_t index, const size_t window_size){
		size_t center;
		double kappa;
		std::vector<double> kappa_set;
		if (raw_path.size() < window_size * 2 + 1)
			throw "Window size is too large";
		if (index < window_size){
			center = window_size;
		}
		else if (index > raw_path.size() - 1 - window_size) {
			center = raw_path.size() - 1 - window_size;
		}
		else{
			center = index;
		}
		for (size_t i = window_size; i > 0; i--)
		{
			kappa = rectifyAngle(raw_path[center].theta_ - raw_path[center - i].theta_) / 
					calcDistance(raw_path[center - i].position_, raw_path[center].position_);
			kappa_set.push_back(kappa);
		}
		for (size_t i = 1; i <= window_size; i++)
		{
			kappa = rectifyAngle(raw_path[center + i].theta_ - raw_path[center].theta_) /
				calcDistance(raw_path[center].position_, raw_path[center + i].position_);
			kappa_set.push_back(kappa);
		}
		std::sort(kappa_set.begin(), kappa_set.end());
		kappa = (kappa_set[window_size]+ kappa_set[window_size+1])/2.0f;
		return kappa;
	}
}
