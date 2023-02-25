//
// Created by luyifan on 18-7-10.
//

#ifndef STATEMACHINE_TOOL_H
#define STATEMACHINE_TOOL_H

#include "bezier.h"

namespace planning{
#define MY_INF std::numeric_limits<double>::infinity()
#define MY_PI 3.141592658

  inline double dist2Points(Bezier::Vec2 pt1, Bezier::Vec2 pt2){
    return sqrt(pow(pt1.x - pt2.x, 2.0) + pow(pt1.y - pt2.y, 2.0));
  }

  // 寻找bezier终止点在参考线中的位置
  inline int lower_bound(std::vector<Bezier::Vec2>nums, double target){
    int low = 0;
    int high = (int)nums.size() - 1;
    int pos = (int)nums.size();
    int mid;
    while(low < high){
      mid = (low+high)/2;
      if(nums[mid].x < target)
        low = mid + 1;
      else
        high = mid;
    }
    if(nums[low].x >= target - 0.00001)
      pos = low;
    if(pos == (int)nums.size())
      pos -= 1;

    return pos;
  }

  inline double normalize_angle(double angle){
    double tmp = (angle + M_PI) / (2 * M_PI);
    double tmp1 = (angle + M_PI) - (int)tmp * (2 * M_PI);
    if(tmp1 < 0.0){
      tmp1 += 2 * M_PI;
    }
    return tmp1 - M_PI;
  }
}
#endif //STATEMACHINE_TOOL_H
