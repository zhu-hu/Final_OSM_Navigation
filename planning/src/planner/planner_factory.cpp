//
// Created by luyifan on 19-11-6.
//

#include "planner_factory.h"

namespace planning {
Planner *PlannerFactory::GetPlanner(const planner::PlannerType type) {
  if (type == planner::DUBINS) {
    if (!dubins_planner) {
      dubins_planner = Create(type);
    }
    return dubins_planner;
  }
}
}  // namespace planning