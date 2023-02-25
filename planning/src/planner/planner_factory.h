//
// Created by luyifan on 19-11-6.
//

#ifndef PLANNING_PLANNER_FACTORY_H
#define PLANNING_PLANNER_FACTORY_H

// #include "bezier_planner.h"
// #include "dstar_rrt_planner.h"
#include "dubins_planner.h"
// #include "frenet_planner.h"
// #include "hybrid_a_star_planner.h"
// #include "lattice_planner.h"
// #include "reference_line_planner.h"
// #include "rrt_planner.h"
// #include "spline_planner.h"

namespace planning {
class PlannerFactory {
 public:
  Planner *Create(planner::PlannerType type) {
    if (type == planner::DUBINS) {
      return new DubinsPlanner("Dubins");
    }
  }

  Planner *GetPlanner(const planner::PlannerType type);

 private:
  Planner *dubins_planner = nullptr;
};
}  // namespace planning

#endif  // PLANNING_PLANNER_FACTORY_H
