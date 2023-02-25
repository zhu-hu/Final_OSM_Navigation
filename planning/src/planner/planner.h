//
// Created by luyifan on 19-11-5.
//

#ifndef PLANNING_PLANNER_H
#define PLANNING_PLANNER_H
#include "common/struct/frame.h"
#include "common/struct/reference_line.h"

#define FrenetTes

namespace planning {
namespace planner {
enum PlannerType {
  BEZIER,
  SPLINE,
  LATTICE,
  REFERENCE_LINE,
  DSTAR_RRT,
  FRENET,
  RRT,
  HYBRID_A_STAR,
  DUBINS
};
}

// template<class DerivedState>
class Planner {
 public:
  Planner() = default;
  virtual ~Planner() { AINFO << "[PLANNER] " << name() << " is destroyed!!"; };

  std::string name() { return name_; };
  virtual Trajectory *const Plan(Frame *frame) = 0;
  std::string name_;
};
}  // namespace planning

#endif  // PLANNING_PLANNER_H
