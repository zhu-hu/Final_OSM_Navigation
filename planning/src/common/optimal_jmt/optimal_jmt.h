//
// Created by luyifan on 19-11-6.
//

#ifndef PLANNING_OPTIMAL_JMT_H
#define PLANNING_OPTIMAL_JMT_H

#include "jerk_mini_trajectory.h"

namespace planning{
  class OptimalJMTQuintic{
  public:
    OptimalJMTQuintic(const vector<double>& start, const vector<double>& end, double max_square_jerk);
    inline void SetmMxSquareJerk(double val){max_square_jerk_ = val;}
    std::pair<vector<double>, double> OptimalTrajectory();
    std::pair<vector<double>, double> OptimalTrajectory(double &jerk);
  private:
    vector<double> start_;
    vector<double> end_;
    double max_square_jerk_;
  };

  class OptimalJMTQuartic{
  public:
    OptimalJMTQuartic(const vector<double>& start, const vector<double>& end, double t, double max_square_jerk);
    inline void SetmMxSquareJerk(double val){max_square_jerk_ = val;}
    std::pair<vector<double>, double> OptimalTrajectory();
    std::pair<vector<double>, double> OptimalTrajectory(double &jerk);
  private:
    vector<double> start_;
    vector<double> end_;
    double max_square_jerk_;
    double t_;
  };
}

#endif //PLANNING_OPTIMAL_JMT_H
