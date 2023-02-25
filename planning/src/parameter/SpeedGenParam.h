//
// Created by luyifan on 18-7-12.
//

#ifndef STATEMACHINE_SPEEDGENPARAM_H
#define STATEMACHINE_SPEEDGENPARAM_H
#include <string.h>

namespace planning {
class SpeedGenParam {
 public:
  SpeedGenParam() {}

 public:
  double desire_speed_idling_;        //怠速 单位:m/s
  double desire_speed_slow_down_;     //特定路段减速慢行 单位:m/s
  double desire_speed_change_lane_;   //超车 单位:m/s
  double desire_speed_lane_;          //单位:m/s
  double desire_speed_intersection_;  //单位:m/s
  double desire_speed_intersection_straight_;  //单位:m/s
  double accelerate_up_;                       //单位:m/s2
  double accelerate_down_;                     //单位:m/s2
  double desire_low_speed_;
  double desire_speed_lane_change;  //绕障
  double maximum_speed;
  double desire_speed_uturn_;
};
}  // namespace planning

#endif  // STATEMACHINE_SPEEDGENPARAM_H
