//
// Created by luyifan on 18-7-11.
//

#ifndef STATEMACHINE_PARAMETER_H
#define STATEMACHINE_PARAMETER_H

#include "BehaviorParam.h"
#include "GridMapParam.h"
#include "MapParam.h"
#include "SpeedGenParam.h"

namespace planning {
class Parameter {
 public:
  Parameter() {
    behavior_param_ = BehaviorParam();
    grid_map_param_ = GridMapParam();
    speed_gen_param_ = SpeedGenParam();
    map_param_ = MapParam();
  }

 public:
  BehaviorParam behavior_param_;
  GridMapParam grid_map_param_;
  SpeedGenParam speed_gen_param_;
  MapParam map_param_;
};
}  // namespace planning

#endif  // STATEMACHINE_PARAMETER_H
