#pragma once
#include <vector>
#include <algorithm>
#include "common/Point.hpp"
#include "utilities/PathUtilities.h"

namespace PathPlanningUtilities
{
	/** @brief Struct of quntic spline function coefficients
	
	Struct of quintic spline function coefficients along aixs x and axis y
	Get the only solution of quntic spline parametric equation from the struct
	*/
	struct QParameters
	{
		double ax_, bx_, cx_, dx_, ex_, fx_;   ///< coefficients of x, order descent
		double ay_, by_, cy_, dy_, ey_, fy_;   ///< coefficients of y, order descent
	};

	/** @brief Struct of vehicle information

	Indicate present position and state of vehicle
	*/
	struct VehicleState
	{
		Point2f position_;				///< current position of vehicle
		double theta_;					///< heading of vehicle/longtitual velocity, rad
		double kappa_;					///< curvature of vehicle，equals to Tan[front wheel steering angle]/wheelbase，or angular_velocity/longtitual_velocity
	};

	/** @brief Struct of vehicle motion information
	
	Indicate present motion information of vehicle
	*/
	struct VehicleMovementState
	{
		double velocity_;					///< longtitual velocity of vehicle
		double acceleration_;				///< longtitual acceleration of vehicle
	};

	class QuinticSplinePlanning
	{
	public:
		/** @brief Get time-based path from quintic spline planning
		
		Calculate coefficients of quintic spline function from current positon and state of vehicle, global path and time limits
		Time is an independent variable here, genarated path points are evenly distributed in time
		@param begin_state vehicle state at start point
		@param end_state curvature and vehicle state at end point on path
		@param begin_movement_state vehicle motion state at start point
		@param end_movement_state vehicle motion state at end point
		@param time_constraint total time of path generation
		@param order order of the splines, 5 or 4 or 3
		@param params coefficients of generated quintic splines
		*/
		static void getParametersByTimeConstraint(
			const VehicleState& begin_state, 
			const CurvePoint& end_state,
			const VehicleMovementState& begin_movement_state,
			const VehicleMovementState& end_movement_state,
			const double time_constraint,
			const int order,
			QParameters& params);

		/** @brief Get mileage-based path from quintic spline planning
		
		Calculate coefficients of quintic spline function from current positon and state of vehicle, global path and mileage limits
		Mileage is an independent variable here, genarated path points are evenly distributed in mileage
		@param begin_state vehicle state at start point
        @param end_state curvature and vehicle state at end point on path
		@param mileage_constraint total mileage of path generation
		@param order order of the splines, 5 or 4 or 3
		@param params coefficients of generated quintic splines
		*/
		static void getParametersByMileageConstraint(
			const VehicleState& begin_state,
			const CurvePoint& end_state,
			const double mileage_constraint, 
			const int order,
			QParameters& params);
	
		/** @brief Get curvature and vehicle state on end points of path from current vehicle position and global path 
		
		@param global_path global path
		@param begin_state vehicle state at start point
		@param begin_movement_state vehicle motion state at start point
		@param follow_length threshold of vision field
		@param end_idx vehicle state index of end point on generated path
		@return whether goal point is final point of path，0 for False，1 for Ture
		*/
		static int getGoalPoint(
			const Curve& global_path,
			const VehicleState& begin_state,
			const double velocity,
			const double follow_length,
			const size_t& last_idx,
			size_t& end_idx);

		/** @brief Extend goal points to point assemble according to tangential direction
		
		@param goal goal point
		@param space distance between two neighbor points
		@param number number of points generated in each side
		@param goalpoints assemble of generated goal points，size of assemble = 2*number+1
		*/
		static void goalSetGenerate(
			const CurvePoint& goal,
			const double space,
			const size_t number,
			std::vector<CurvePoint>& goal_set);

		/** @brief Extend goal points to point assemble according to tangential direction, bounded
		
		@param goal goal point
		@param space distance between two neighbor points
		@param number number of points generated in each side
		@param goalpoints assemble of generated goal points，less than 2*number+1
		*/
		static void goalSetGenerate(
			const BoundedCurvePoint& goal,
			const double space,
			const size_t number,
			std::vector<CurvePoint>& goal_set);
	};

	/** @brief generator of quintic spline local paths

    Genetate local paths from coefficients of quintic spline
	*/
	class pathGenerator
	{
	public:
		/** @brief constructed function of generator
		
		@param points number of sampled points on local path, including start point and end point
		*/
		pathGenerator(const size_t points);
		~pathGenerator();

		/** @brief Generate paths from coefficients
		
		@param params coefficients of quintic splines
		@param path generated path
		*/
		void calcPath(const QParameters & params, Path& path) const;

		void calcCurve(const QParameters& params, Curve& curve) const;

		void calcCurveChangeRate(const QParameters& params, std::vector<double>& curve_change_rates) const;

	private:
		size_t points_;
		double(*eta_)[5];
	};
}