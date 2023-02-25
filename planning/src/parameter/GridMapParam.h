//
// Created by luyifan on 18-7-11.
//

#ifndef STATEMACHINE_GRIDMAPPARAM_H
#define STATEMACHINE_GRIDMAPPARAM_H
#include <cstring>

namespace planning {
class GridMapParam {
 public:
  GridMapParam() {}

 public:
  double pixel_scale;  //栅格图的像素比例，Xm/像素
  double min_x;        //栅格图的最小X坐标值
  double max_x;        //栅格图的最大X坐标值
  double min_y;        //栅格图的最小Y坐标值
  double max_y;        //栅格图的最大Y坐标值
};
}  // namespace planning

#endif  // STATEMACHINE_GRIDMAPPARAM_H
