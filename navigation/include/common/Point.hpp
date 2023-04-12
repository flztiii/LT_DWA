#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include <sstream>
#include <iomanip>

namespace PathPlanningUtilities
{
	struct Point2f
	{
		double x_;
		double y_;
	};

	struct Vector2f
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

	struct BoundedCurvePoint
	{
		CurvePoint center_point_;
		double left_distance_;
		double right_distance_;
	};

	struct CoordinationPoint
	{
		CurvePoint worldpos_;
		double station_;
		double max_height_;
		double min_height_;
	};

	/// Get vector of two points
	inline Vector2f operator - (const Point2f& end, const Point2f& start)
	{
		Vector2f vec;
		vec.x_ = end.x_ - start.x_;
		vec.y_ = end.y_ - start.y_;
		return vec;
	}

	/// Get new point from adding a vector with a point
	inline Point2f operator + (const Point2f& base, const Vector2f& vec)
	{
		Point2f result;
		result.x_ = base.x_ + vec.x_;
		result.y_ = base.y_ + vec.y_;
		return result;
	}

	// Sub of two vectors
	inline Vector2f operator - (const Vector2f& vec_1, const Vector2f& vec_2) {
		Vector2f result;
		result.x_ = vec_1.x_ - vec_2.x_;
		result.y_ = vec_1.y_ - vec_2.y_;
		return result;
	} 

	/// Multiplication of two vectors
	inline Vector2f operator * (const Vector2f& base, const double factor)
	{
		Vector2f result;
		result.x_ = base.x_ * factor;
		result.y_ = base.y_ * factor;
		return result;
	}

	/// Whether two points are the same
	inline bool operator == (const Point2f& a, const Point2f& b)
	{
		return a.x_ == b.x_ && a.y_ == b.y_;
	}

	/// Whether two vectors are the same
	inline bool operator == (const Vector2f& a, const Vector2f& b)
	{
		return a.x_ == b.x_ && a.y_ == b.y_;
	}

	/// Whether two curve points are the same
	inline bool operator == (const CurvePoint& a, const CurvePoint& b)
	{
		return a.position_ == b.position_ && a.theta_ == b.theta_ && a.kappa_ == b.kappa_;
	}

	/// Get points from stream
	inline std::istringstream& operator >> (std::istringstream& stream, Point2f& pt)
	{
		stream >> pt.x_;
		if (stream.peek() == ',') stream.ignore();
		stream >> pt.y_;
		return stream;
	}

	/// Get vector from stream
	inline std::istringstream& operator >> (std::istringstream& stream, Vector2f& vec)
	{
		stream >> vec.x_;
		if (stream.peek() == ',') stream.ignore();
		stream >> vec.y_;
		return stream;
	}

	/// Get curve points from stream
	inline std::istringstream& operator >> (std::istringstream& stream, CurvePoint& cpt)
	{
		stream >> cpt.position_;
		if (stream.peek() == ',') stream.ignore();
		stream >> cpt.theta_;
		if (stream.peek() == ',') stream.ignore();
		stream >> cpt.kappa_;
		return stream;
	}

	/// Write points into stream
	inline std::ostringstream& operator << (std::ostringstream& stream, const Point2f& pt)
	{
		stream << std::setiosflags(std::ios::fixed) << std::setprecision(24) 
			   << pt.x_ << "," << pt.y_ << std::endl;
		return stream;
	}

	/// Write curve points into stream
	inline std::ostringstream& operator << (std::ostringstream& stream, const CurvePoint& cpt)
	{
		stream << std::setiosflags(std::ios::fixed) << std::setprecision(24) 
			   << cpt.position_.x_ << "," << cpt.position_.y_ << "," << cpt.theta_ << "," << cpt.kappa_ << std::endl;
		return stream;
	}
	
	/// Get module length of vector
	inline double getLength(const Vector2f& vec)
	{
		return sqrt(vec.x_*vec.x_ + vec.y_*vec.y_);
	}

	// 计算行列式
	inline double determinant(double v1, double v2, double v3, double v4) {
		return (v1 * v4 - v2 * v3);
	}

	// 计算两个向量之间的内积
	inline double dot(const Vector2f& vec1, const Vector2f &vec2) {
		return vec1.x_ * vec2.x_ + vec1.y_ * vec2.y_;
	}

	// 计算两个向量之间的外积
	inline double cross(const Vector2f& vec1, const Vector2f &vec2) {
		return vec1.x_ * vec2.y_ - vec2.x_ * vec1.y_;
	}

	// 求解向量的范数
	inline double norm(const Vector2f& vec) {
		return sqrt(vec.x_*vec.x_ + vec.y_*vec.y_);
	}

	/// Get angle of vector，-pi~pi，return 0 if angle is zero
	inline double getAngle(Vector2f& vec)
	{
		return atan2(vec.y_, vec.x_);
	}

	/// Get distance between two points
	inline double calcDistance(const Point2f& start, const Point2f& end)
	{
		Vector2f vec = end - start;
		return getLength(vec);
	};

	/// Get angle of line formed by two points ，-pi~pi，return 0 if points are the same
	inline double calcDeflection(const Point2f& start, const Point2f& end)
	{
		Vector2f vec = end - start;
		return getAngle(vec);
	};

	/// Calculate inner production of two vectors
	inline double calcInnerProduct(const Vector2f& vec1, const Vector2f& vec2)
	{
		return vec1.x_ * vec2.x_ + vec1.y_ * vec2.y_;
	}

	/// Calculate angle between two vectors
	/// 
	/// Rotate vector and axis, latter to axis x; calculate angle between vector and axis x
	inline double calcIncludedAngle(const Vector2f& axis, const Vector2f& vec)
	{
		Vector2f rot = { axis.x_ * vec.x_ + axis.y_ * vec.y_, axis.x_ * vec.y_ - axis.y_ * vec.x_ };
		return getAngle(rot);
	}

	/// Find middle point of vector
	inline Point2f findMiddlePoint(const Point2f& start, const Point2f& end, const double eta)
	{
		return start + (end - start) * eta;
	};

	/// Reflect angle between -pi~pi
	inline double rectifyAngle(double angle)
	{
		if (angle > M_PI)
		{
			angle -= floor(angle / M_PI / 2.0f) * M_PI * 2.0f;
			if (angle > M_PI)
				angle -= M_PI * 2.0f;
		}
		else if (angle <= -M_PI)
		{
			angle += floor(-angle / M_PI / 2.0f) * M_PI * 2.0f;
			if (angle <= -M_PI)
				angle += M_PI * 2.0f;
		}
		return angle;
	};
}

