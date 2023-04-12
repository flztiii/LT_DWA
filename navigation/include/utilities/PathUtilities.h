#pragma once

#include <vector>
#include "common/Point.hpp"
#include "common/Path.hpp"

namespace PathPlanningUtilities
{
	class PathUtilities
	{
	public:
		/** @brief Pre-smoothing of global path

		Raw path points may not distribute evenly and gather as small groups, inconvinient for furture calculation
	    Realize the resampling of raw path with this function 
		Distance between two neighbor points of smoothed path is nearly the same
		Jerking in raw path all filtered
	  @param raw_path recored raw path
		@param smoothed_path path after smoothing
		@param average_distance average distance between two neighbor points  on smoothed path
		@param iteration number f iteration
		*/
		static void pathSmooth(const Path& raw_path, Path& smoothed_path, double& average_distance, const size_t iteration = 3);

		/** @brief Calculate information of each point
		
		Calculate the orientation and curvature of points on raw path via robust algorithm
    @param raw_path recored raw path
		@param curve calculated curve containing orientation and curvature of each point
		@param window_size selected size of window
		*/
		static void getCurve(const Path& raw_path, Curve& curve, const size_t window_size = 10);

		/** @brief Calculate the left and right bound of a bounded curve
	  
		Calculate each point on bound using the curvature of the center point and the distance
		@param bcurve bounded curve
		@param left_bound the left bound of the origin curve
		@param right_bound the right bound of the origin curve
		*/
		static void getBound(const BoundedCurve& bcurve, Path& left_bound, Path& right_bound);

		/** @brief Find  on path the nearest point to given point
		 
		@param path path
		@param point given point
		@return the nearest point 
		*/
		static size_t FindNearestPoint(const Path & path, const Point2f & point);

		/** @brief Find on path the nearest point to given point

		@param path path
		@param point given point
		@return the nearest point index
		*/
		static size_t findNearestPoint(const Curve & curve, const Point2f & point,const double theta0);

	private:
		/** @brief Calculate angle of a point on global path
		
		@param raw_path raw global path
		@param index index of point 
		@return calculated angle
		*/
		static double calcTheta(const Path& raw_path,const size_t index, const size_t window_size=10);
		/**@brief Calculate curvature of points 
		
		@param raw_path raw path with orientation
		@param index point index
		@param window_size selected size of window
		@return calculated curvature
		*/
		static double calcKappa(const Curve& raw_path, const size_t index, const size_t window_size);

	};

}