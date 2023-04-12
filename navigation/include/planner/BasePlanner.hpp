/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef BASE_PLANNER_HPP_
#define BASE_PLANNER_HPP_

#include "common/Config.hpp"
#include "common/Common.hpp"
#include "common/Point.hpp"
#include "common/Path.hpp"
#include "utilities/Tools.hpp"
#include "utilities/PathGenerator.h"
#include "utilities/LineSegment.hpp"
#include "utilities/Visualization.hpp"
#include "utilities/KDTree.hpp"

// base planner
class BasePlanner {
 public:
    virtual int planning(const PathPlanningUtilities::Point2f &start_point, const PathPlanningUtilities::Point2f &goal_point, const GridMap &grid_map, const KDTree &kd_tree, PathPlanningUtilities::Path &path)=0;
};

#endif // BASE_PLANNER_HPP_