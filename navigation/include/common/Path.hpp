#pragma once

#include <vector>

#include "common/Point.hpp"
#include <iostream>
#include <iomanip>

namespace PathPlanningUtilities
{
	typedef std::vector<Point2f> Path;
	typedef std::vector<CurvePoint> Curve;
	typedef std::vector<BoundedCurvePoint> BoundedCurve;

	/** @brief Get path from stream */
	inline std::istream& operator >> (std::istream& stream, Path& path)
	{
		Point2f p;
		for (std::string buf; std::getline(stream, buf);)
		{
			std::istringstream line(buf);
			line >> p;
			path.push_back(p);
		}
		return stream;
	}

	/** @brief Get curve from curve */
	inline std::istream& operator >> (std::istream& stream, Curve& curve)
	{
		CurvePoint cp;
		for (std::string buf; std::getline(stream, buf);)
		{
			std::istringstream line(buf);
			line >> cp;
			curve.push_back(cp);
		}
		return stream;
	}

	/** @brief Write path to stream */
	inline std::ostream& operator << (std::ostream& stream, Path& path)
	{
		for (const Point2f& pt : path)
		{
			std::ostringstream line;
			line << pt;
			stream << line.str();
		}
		return stream;
	}

	/** @brief Write curve to stream */
	inline std::ostream& operator << (std::ostream& stream, Curve& curve)
	{
		for (const CurvePoint& cpt : curve)
		{
			std::ostringstream line;
			line << cpt;
			stream << line.str();
		}
		return stream;
	}

	/** @brief Write position of curve to path
	
	@note size of path equals that of curve after treatement, original data overrided
	*/
	inline void operator >> (const Curve& curve, Path& path)
	{
		path.resize(curve.size());
		for (size_t i = 0; i < curve.size(); i++)
		{
			path[i] = curve[i].position_;
		}
	}

	/** @brief Write path  data into curve's postion
	
	@note  size of curve equals that of path after treatement, original data overrided
	Original orientation and curvature data saved, newly-added oriention and curvature are random values
	*/
	inline void operator >> (const Path& path, Curve& curve)
	{
		curve.resize(path.size());
		for (size_t i = 0; i < curve.size(); i++)
		{
			curve[i].position_ = path[i];
		}
	}

	/** @brief Write curve data into bounded curve
	
	@note the distance of each point will overrided to zero
	*/
	inline void operator >> (const Curve& curve, BoundedCurve& bcurve)
	{
		bcurve.resize(curve.size());
		for (size_t i = 0; i < curve.size(); i++)
		{
			bcurve[i].center_point_ = curve[i];
			bcurve[i].left_distance_ = 0;
			bcurve[i].right_distance_ = 0;
		}
	}

	/** @brief Write curve data into bounded curve
	
	@note the distance of each point will overrided to zero
	*/
	inline void operator >> (const BoundedCurve& bcurve, Curve& curve)
	{
		curve.resize(bcurve.size());
		for (size_t i = 0; i < curve.size(); i++)
		{
			curve[i] = bcurve[i].center_point_;
		}
	}
}