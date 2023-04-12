#include "utilities/PathGenerator.h"

namespace PathPlanningUtilities
{
	pathGenerator::pathGenerator(const size_t points)
	{
		this->points_ = points;
		this->eta_ = new double[points][5];
		for (size_t i = 0; i < points; i++){
			for (size_t j = 0; j < 5; j++){
				this->eta_[i][j] = powf(i*1.0f / (points - 1), j + 1.0f);
			}
		}
	}

	pathGenerator::~pathGenerator()
	{
		delete[](eta_);
	}

	void pathGenerator::calcPath(const QParameters& params, Path& path) const
	{
		Point2f p;
		path.resize(this->points_);
		for (size_t i = 0; i < this->points_; i++){
			p.x_ = 
				params.ax_*this->eta_[i][4] + 
				params.bx_*this->eta_[i][3] + 
				params.cx_*this->eta_[i][2] + 
				params.dx_*this->eta_[i][1] + 
				params.ex_*this->eta_[i][0] + 
				params.fx_;
			p.y_ = 
				params.ay_*this->eta_[i][4] + 
				params.by_*this->eta_[i][3] + 
				params.cy_*this->eta_[i][2] + 
				params.dy_*this->eta_[i][1] + 
				params.ey_*this->eta_[i][0] + 
				params.fy_;
			path[i]=p;
		}
	}

	void pathGenerator::calcCurve(const QParameters& params, Curve& curve) const
	{
		CurvePoint cp;
		curve.resize(this->points_);
		for (size_t i = 0; i < this->points_; i++){
			cp.position_.x_ = 
				params.ax_*this->eta_[i][4] + 
				params.bx_*this->eta_[i][3] + 
				params.cx_*this->eta_[i][2] + 
				params.dx_*this->eta_[i][1] + 
				params.ex_*this->eta_[i][0] + 
				params.fx_;
			cp.position_.y_ = 
				params.ay_*this->eta_[i][4] + 
				params.by_*this->eta_[i][3] + 
				params.cy_*this->eta_[i][2] + 
				params.dy_*this->eta_[i][1] + 
				params.ey_*this->eta_[i][0] + 
				params.fy_;
			double vx = 
			    params.ax_*this->eta_[i][3] * 5 + 
				params.bx_*this->eta_[i][2] * 4 + 
				params.cx_*this->eta_[i][1] * 3 + 
				params.dx_*this->eta_[i][0] * 2 + 
				params.ex_;
			double vy = 
			    params.ay_*this->eta_[i][3] * 5 + 
				params.by_*this->eta_[i][2] * 4 + 
				params.cy_*this->eta_[i][1] * 3 + 
				params.dy_*this->eta_[i][0] * 2 + 
				params.ey_;
			cp.theta_ = atan2(vy, vx);
			double ax = 
			    params.ax_*this->eta_[i][2] * 20 + 
				params.bx_*this->eta_[i][1] * 12 + 
				params.cx_*this->eta_[i][0] * 6 + 
				params.dx_ * 2;
			double ay = 
			    params.ay_*this->eta_[i][2] * 20 + 
				params.by_*this->eta_[i][1] * 12 + 
				params.cy_*this->eta_[i][0] * 6 + 
				params.dy_ * 2;

			cp.kappa_ = (ay*vx-ax*vy)/pow((vx*vx+vy*vy), 1.5);
			curve[i]=cp;
		}
	}

	void pathGenerator::calcCurveChangeRate(const QParameters& params, std::vector<double>& curve_change_rates) const
	{
		curve_change_rates.resize(this->points_);
		for (size_t i = 0; i < this->points_; i++){
			double dx = 
			    params.ax_*this->eta_[i][3] * 5 + 
				params.bx_*this->eta_[i][2] * 4 + 
				params.cx_*this->eta_[i][1] * 3 + 
				params.dx_*this->eta_[i][0] * 2 + 
				params.ex_;
			double dy = 
			    params.ay_*this->eta_[i][3] * 5 + 
				params.by_*this->eta_[i][2] * 4 + 
				params.cy_*this->eta_[i][1] * 3 + 
				params.dy_*this->eta_[i][0] * 2 + 
				params.ey_;
			double ddx = 
			    params.ax_*this->eta_[i][2] * 20 + 
				params.bx_*this->eta_[i][1] * 12 + 
				params.cx_*this->eta_[i][0] * 6 + 
				params.dx_ * 2;
			double ddy = 
			    params.ay_*this->eta_[i][2] * 20 + 
				params.by_*this->eta_[i][1] * 12 + 
				params.cy_*this->eta_[i][0] * 6 + 
				params.dy_ * 2;
			double dddx = 
				params.ax_*this->eta_[i][1] * 60 +
				params.bx_*this->eta_[i][0] * 24 + 
				params.cx_* 6;
			double dddy = 
				params.ay_*this->eta_[i][1] * 60 +
				params.by_*this->eta_[i][0] * 24 + 
				params.cy_* 6;
			double curvature_change_rate = (1.0 / (2.0 * pow(dx * dx + dy * dy, 2.5)) * (6.0 * (dy * ddx - dx * ddy) * (dx * ddx + dy * ddy) + 2.0 * (dx * dx + dy * dy) * (-dy * dddx + dx * dddy))) / sqrt(dx * dx + dy * dy);
			curve_change_rates[i] = curvature_change_rate;
		}
	}

	void QuinticSplinePlanning::getParametersByTimeConstraint(
		const VehicleState& begin_state,
		const CurvePoint& end_state,
		const VehicleMovementState& begin_movement_state,
		const VehicleMovementState& end_movement_state,
		const double time_constraint,
		const int order,
		QParameters& params)
	{
		double t = time_constraint;
		double vr0 = begin_movement_state.velocity_;
		double vr1 = end_movement_state.velocity_;
		double ar0 = begin_movement_state.acceleration_;
		double ar1 = end_movement_state.acceleration_;
		//Initilize parameters
		double p0x, p0y, v0x, v0y, a0x, a0y;
		p0x = begin_state.position_.x_;
		p0y = begin_state.position_.y_;
		v0x = vr0*cos(begin_state.theta_);
		v0y = vr0*sin(begin_state.theta_);
		//Calculate
		//  {Ax, Ay} 
		//      == D[{Vx[t], Vy[t]}, t]
		//      == D[{V[t]*Cos[theta_[t]], V[t]*Sin[theta_[t]]}, t]
		a0x = ar0*cos(begin_state.theta_) - begin_state.kappa_*sin(begin_state.theta_)*vr0*vr0;
		a0y = ar0*sin(begin_state.theta_) + begin_state.kappa_*cos(begin_state.theta_)*vr0*vr0;
		//Finish parameter initilization
		double p1x, p1y, v1x, v1y, a1x, a1y;
		p1x = end_state.position_.x_;
		p1y = end_state.position_.y_;
		v1x = vr1*cos(end_state.theta_);
		v1y = vr1*sin(end_state.theta_);
		a1x = ar1*cos(end_state.theta_) - end_state.kappa_*sin(end_state.theta_)*vr1*vr1;
		a1y = ar1*cos(end_state.theta_) + end_state.kappa_*cos(end_state.theta_)*vr1*vr1;

		//Generate parameters
		//Solve equation S[e*T]:={ax*(e*T)^5 + …… + fx, ay*(e*T)^5 + …… + fy}
		//  S[0]   == {P0x, P0y}
		//  S'[0]  == {V0x, V0y}
		//  S''[0] == {A0x, A0y}
		//  S[T]   == {P1x, P1y}
		//  S'[T]  == {V1x, V1y}  << onlv if order >= 4
		//  S''[T] == {A1x, A1y}  << only if order >= 5
		//Get {ax*T^5, ……, fx, ay*T^5, ……, fy}
			params.fx_ = +(                   p0x );
			params.ex_ = +(                   v0x ) * t;
			params.dx_ = +(                   a0x ) * t * t / 2.0f;
		if(order == 3)
		{
			params.cx_ = +( +      p1x -      p0x )
					     +(            -      v0x ) * t
						 +(            -      a0x ) * t * t / 2.0f;
			params.bx_ =                              0;
			params.ax_ =                              0;
		}
		else if(order == 4)
		{
			params.cx_ = +( +  4 * p1x -  4 * p0x )
					     +( -      v1x -  3 * v0x ) * t
					     +(            -  2 * a0x ) * t * t / 2.0f;
			params.bx_ = +( -  3 * p1x +  3 * p0x )
					     +( +      v1x +  2 * v0x ) * t
					     +(            +      a0x ) * t * t / 2.0f;
			params.ax_ =                              0;
		}
		else
		{
			params.cx_ = +( + 10 * p1x - 10 * p0x )
						 +( -  4 * v1x -  6 * v0x ) * t
						 +( +      a1x -  3 * a0x ) * t * t / 2.0f;
			params.bx_ = +( - 15 * p1x + 15 * p0x )
						 +( +  7 * v1x +  8 * v0x ) * t
						 +( -  2 * a1x +  3 * a0x ) * t * t / 2.0f;
			params.ax_ = +( +  6 * p1x -  6 * p0x )
						 +( -  3 * v1x -  3 * v0x ) * t
						 +( +      a1x -      a0x ) * t * t / 2.0f;
		}

			params.fy_ = +(                   p0y );
			params.ey_ = +(                   v0y ) * t;
			params.dy_ = +(                   a0y ) * t * t / 2.0f;
		if(order == 3)
		{
			params.cy_ = +( +      p1y -      p0y )
					     +(            -      v0y ) * t
						 +(            -      a0y ) * t * t / 2.0f;
			params.by_ =                              0;
			params.ay_ =                              0;
		}
		else if(order == 4)
		{
			params.cy_ = +( +  4 * p1y -  4 * p0y )
					     +( -      v1y -  3 * v0y ) * t
					     +(            -  2 * a0y ) * t * t / 2.0f;
			params.by_ = +( -  3 * p1y +  3 * p0y )
					     +( +      v1y +  2 * v0y ) * t
					     +(            +      a0y ) * t * t / 2.0f;
			params.ay_ =                              0;
		}
		else
		{
			params.cy_ = +( + 10 * p1y - 10 * p0y )
						 +( -  4 * v1y -  6 * v0y ) * t
						 +( +      a1y -  3 * a0y ) * t * t / 2.0f;
			params.by_ = +( - 15 * p1y + 15 * p0y )
						 +( +  7 * v1y +  8 * v0y ) * t
						 +( -  2 * a1y +  3 * a0y ) * t * t / 2.0f;
			params.ay_ = +( +  6 * p1y -  6 * p0y )
						 +( -  3 * v1y -  3 * v0y ) * t
						 +( +      a1y -      a0y ) * t * t / 2.0f;
		}
	}

	void QuinticSplinePlanning::getParametersByMileageConstraint(
		const VehicleState& begin_state,
		const CurvePoint& end_state,
		const double mileage_constraint,
		const int order,
		QParameters& params)
	{
		double  l= mileage_constraint;
		//Initialize parameters
		double p0x, p0y, t0x, t0y, k0x, k0y;
		p0x = begin_state.position_.x_;
		p0y = begin_state.position_.y_;
		t0x = cos(begin_state.theta_);
		t0y = sin(begin_state.theta_);
		k0x = -begin_state.kappa_*sin(begin_state.theta_);
		k0y = begin_state.kappa_*cos(begin_state.theta_);
		//Finish parameter initialization
		double p1x, p1y, t1x, t1y, k1x, k1y;
		p1x = end_state.position_.x_;
		p1y = end_state.position_.y_;
		t1x = cos(end_state.theta_);
		t1y = sin(end_state.theta_);
		k1x = -end_state.kappa_*sin(end_state.theta_);
		k1y = end_state.kappa_*cos(end_state.theta_);
		//Generate parameters
			params.fx_ = +(                   p0x );
			params.ex_ = +(                   t0x ) * l;
			params.dx_ = +(                   k0x ) * l * l / 2.0f;
		if(order == 3)
		{
			params.cx_ = +( +      p1x -      p0x )
					     +(            -      t0x ) * l
						 +(            -      k0x ) * l * l / 2.0f;
			params.bx_ =                              0;
			params.ax_ =                              0;
		}
		else if(order == 4)
		{
			params.cx_ = +( +  4 * p1x -  4 * p0x )
					     +( -      t1x -  3 * t0x ) * l
					     +(            -  2 * k0x ) * l * l / 2.0f;
			params.bx_ = +( -  3 * p1x +  3 * p0x )
					     +( +      t1x +  2 * t0x ) * l
					     +(            +      k0x ) * l * l / 2.0f;
			params.ax_ =                              0;
		}
		else
		{
			params.cx_ = +( + 10 * p1x - 10 * p0x )
						 +( -  4 * t1x -  6 * t0x ) * l
						 +( +      k1x -  3 * k0x ) * l * l / 2.0f;
			params.bx_ = +( - 15 * p1x + 15 * p0x )
						 +( +  7 * t1x +  8 * t0x ) * l
						 +( -  2 * k1x +  3 * k0x ) * l * l / 2.0f;
			params.ax_ = +( +  6 * p1x -  6 * p0x )
						 +( -  3 * t1x -  3 * t0x ) * l
						 +( +      k1x -      k0x ) * l * l / 2.0f;
		}

			params.fy_ = +(                   p0y );
			params.ey_ = +(                   t0y ) * l;
			params.dy_ = +(                   k0y ) * l * l / 2.0f;
		if(order == 3)
		{
			params.cy_ = +( +      p1y -      p0y )
					     +(            -      t0y ) * l
						 +(            -      k0y ) * l * l / 2.0f;
			params.by_ =                              0;
			params.ay_ =                              0;
		}
		else if(order == 4)
		{
			params.cy_ = +( +  4 * p1y -  4 * p0y )
					     +( -      t1y -  3 * t0y ) * l
					     +(            -  2 * k0y ) * l * l / 2.0f;
			params.by_ = +( -  3 * p1y +  3 * p0y )
					     +( +      t1y +  2 * t0y ) * l
					     +(            +      k0y ) * l * l / 2.0f;
			params.ay_ =                              0;
		}
		else
		{
			params.cy_ = +( + 10 * p1y - 10 * p0y )
						 +( -  4 * t1y -  6 * t0y ) * l
						 +( +      k1y -  3 * k0y ) * l * l / 2.0f;
			params.by_ = +( - 15 * p1y + 15 * p0y )
						 +( +  7 * t1y +  8 * t0y ) * l
						 +( -  2 * k1y +  3 * k0y ) * l * l / 2.0f;
			params.ay_ = +( +  6 * p1y -  6 * p0y )
						 +( -  3 * t1y -  3 * t0y ) * l
						 +( +      k1y -      k0y ) * l * l / 2.0f;
		}
	}

	int QuinticSplinePlanning::getGoalPoint(
		const Curve& global_path,
		const VehicleState& begin_state,
		const double velocity,
		const double follow_length,
		const size_t& last_idx,
		size_t& end_idx)
	{
		size_t nearest_idx;
		size_t index;
		size_t aim;
		double kappa = 0.0;
		double dis = 0.0;
		double theta0 = begin_state.theta_;
		double l0;
		int is_final = 0;
		double l;
		//Get information of starting point
		Point2f pc = begin_state.position_;
		double r;
		l = follow_length;
		nearest_idx = PathUtilities::findNearestPoint(global_path, pc, theta0);
		//std::cout << "Closest：" << nearest_idx << std::endl;
		index = nearest_idx;
		aim = nearest_idx;
		//Get expected goal point
		for (size_t i = nearest_idx; i < global_path.size(); i++)
		{
			if (i == global_path.size() - 1) {
				index = global_path.size() - 1;
				break;
			}
			if (dis >= l) {
				break;
			}
			l0 = calcDistance(global_path[i].position_, global_path[i + 1].position_);
			kappa+=fabs(global_path[i].kappa_);
			dis += l0;
			aim = i;
		}
		kappa=kappa/fabs(aim-nearest_idx+1);
		if(fabs(global_path[nearest_idx].kappa_)>kappa){
			kappa=0.7*fabs(global_path[nearest_idx].kappa_)+0.3*kappa;
		}
		//Get vision field
		if (kappa > 0.0001) {
			r = tanh(kappa*l) / kappa;
		}
		else {
			r = l - pow(l, 3)*kappa*kappa / 3.0 + 2 * pow(l, 5)*pow(kappa, 4) / 15.0;
		}
		dis = 0.0;
		for (size_t i = nearest_idx; i < global_path.size(); i++)
		{
			if (i == global_path.size() - 1) {
				index = global_path.size() - 1;
				break;
			}
			if (dis >= r) {
				break;
			}
			l0 = calcDistance(global_path[i].position_, global_path[i + 1].position_);
			kappa = fabs(global_path[i].kappa_);
			//param = exp(kappa_/0.035);
			if (kappa != 0) {
				//l = l0 / tanh(1/param/ kappa_ / l0);
				l = l0*exp(kappa*l0);
			}
			else {
				l = l0;
			}

			dis += l;
			index = i;

		}
		if(index < last_idx){
			index = last_idx;
		}
		end_idx = index;
		//If goal is reached
		if (end_idx == global_path.size() - 1) {
			is_final = 1;
		}
		return is_final;
	}

	void QuinticSplinePlanning::goalSetGenerate(
		const CurvePoint& goal,
		const double space,
		const size_t number,
		std::vector<CurvePoint>& goal_set)
	{
		//Direction from "right" to "left" 
		Vector2f unit_vec = Vector2f{ -sin(goal.theta_), cos(goal.theta_) };
		goal_set.resize(2 * number + 1);
		if(fabs(goal.kappa_) < 0.0001)
		{
			for (size_t i = 0; i < 2 * number + 1; i++)
			{
				int index = static_cast<int>(i - number);
				goal_set[i].position_ = goal.position_ + unit_vec * index * space;
				goal_set[i].theta_ = goal.theta_;
				goal_set[i].kappa_ = goal.kappa_;
			}
		}
		else
		{
			double goal_r = fabs(1 / goal.kappa_);
			double space_k = number * space * fabs(goal.kappa_);
			double kappa_sign = (goal.kappa_ >= 0) ? 1.0 : -1.0;
			for (size_t i = 0; i < 2 * number + 1; i++)
			{
				//index = -number : rightmost, smaller R for kappa < 0
				//index = 0       : middle
				//index = +number : leftmost, smaller R for kappa > 0
				int index = static_cast<int>(i - number);
				double current_r = goal_r * pow(1 + space_k, - kappa_sign * index / number);
				goal_set[i].position_ = goal.position_ + unit_vec * (- kappa_sign * (current_r - goal_r));
				goal_set[i].theta_ = goal.theta_;
				goal_set[i].kappa_ = kappa_sign / current_r;
			}
		}
	}

	void QuinticSplinePlanning::goalSetGenerate(
		const BoundedCurvePoint& goal,
		const double space,
		const size_t number,
		std::vector<CurvePoint>& goal_set)
	{
		if(number * space < std::min(goal.left_distance_, goal.right_distance_))
		{
			goalSetGenerate(goal.center_point_, space, number, goal_set);
		}
		else
		{
			std::vector<CurvePoint> tmp_goal_set;
			goalSetGenerate(goal.center_point_, space, number, tmp_goal_set);
			size_t right_cnt = 0;
			size_t left_cnt = 0;
			size_t i=0;
			for(i=0; i<number; i++)
			{
				double distance_r = calcDistance(tmp_goal_set[i].position_, goal.center_point_.position_);
				if(distance_r < goal.right_distance_)
				{
					right_cnt = i;
					break;
				}
			}
			if(i == number)
			    right_cnt = number;

			for(i=0; i<number; i++)
			{
				double distance_l = calcDistance(tmp_goal_set[2*number - i].position_, goal.center_point_.position_);
				if(distance_l < goal.left_distance_)
				{
					left_cnt = i;
					break;
				}
			}
			if(i == number)
			    left_cnt = number;

			size_t dual_count = std::max(left_cnt, right_cnt);
			goal_set.resize(2*(number-dual_count) + 1);
			for(size_t i=0; i<2*(number-dual_count) + 1; i++)
			{
				goal_set[i] = tmp_goal_set[dual_count + i];
			}

		}
		
	}
}