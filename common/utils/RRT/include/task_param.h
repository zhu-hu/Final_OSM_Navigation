//
// Created by huyao on 2018/6/11.
// Email: hooyao@sjtu.edu.cn
//

#ifndef PROJECT_TASK_PARAM_H
#define PROJECT_TASK_PARAM_H

#include <cmath>

namespace task {

    constexpr bool debug = true;

    constexpr float kMaxSpeed = 3;// 20/3.6;
    constexpr float kDistSmoothed = 1.0;

    constexpr float kDisTriggerEnd = 10.0;
    constexpr float kDisTriggerReverse = 10.0;

    constexpr float kDestinationDis = kDisTriggerEnd;
    constexpr float kStopDis = 1.0;
    constexpr int kRefDisListSize = 200;
    constexpr float kInitRefDis = 10.0;

    constexpr float kHeadingInverseThreshold = 15.0/180*M_PI;

    constexpr float kStopVel = 0.5;

    constexpr float kRemainDis = 10.0;
    constexpr float kVarThreshold = 100.0;
    constexpr float kKeepLength = 3.0;
    constexpr float kCutEdgeLength = 15.0;
    constexpr float kReverseThreshold = 7*M_PI/8;
    constexpr float kReverseBackToNormal = 4*M_PI/9;
    constexpr float kReversePathLength = 40.0;

    // Interpolation
    constexpr float kPointGap = 1.0;


}

#endif //PROJECT_TASK_PARAM_H
