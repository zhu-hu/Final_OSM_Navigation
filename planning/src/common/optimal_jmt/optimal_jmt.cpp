//
// Created by luyifan on 19-11-6.
//

#include "optimal_jmt.h"
#include "common/utils/log.h"

namespace planning
{
OptimalJMTQuintic::OptimalJMTQuintic(const vector<double> &start, const vector<double> &end, double max_square_jerk) : max_square_jerk_(max_square_jerk), start_(start), end_(end) {}

std::pair<vector<double>, double> OptimalJMTQuintic::OptimalTrajectory()
{
  double t = 3.0;
  vector<double> result;
  while (t <= 15.0)
  {
    JMTQuintic trajectory(start_, end_, t);
    if (trajectory.sum_jerk() < max_square_jerk_)
    {
      //AINFO<<"lat jerk : "<<trajectory.sum_jerk();
      return std::make_pair(trajectory.vals(), t);
    }
    result = trajectory.vals();
    t += 1.0;
  }
  //AINFO<<"Return the best one with time"<<t;
  // AINFO<<"No trajectory's jerk is under "<<max_square_jerk_<<". Return the best one with time "<<t<<".";
  return std::make_pair(result, t);
}

std::pair<vector<double>, double> OptimalJMTQuintic::OptimalTrajectory(double &jerk)
{
  double t = 3.0;
  vector<double> result;
  while (t <= 15.0)
  {
    JMTQuintic trajectory(start_, end_, t);
    if (trajectory.sum_jerk() < max_square_jerk_)
    {
      //AINFO<<"lat jerk : "<<trajectory.sum_jerk();
      jerk = trajectory.sum_jerk();
      return std::make_pair(trajectory.vals(), t);
    }
    result = trajectory.vals();
    jerk = trajectory.sum_jerk();
    t += 1.0;
  }
  //AINFO<<"Return the best one with time"<<t;
  // AINFO<<"No trajectory's jerk is under "<<max_square_jerk_<<". Return the best one with time "<<t<<".";
  return std::make_pair(result, t);
}

OptimalJMTQuartic::OptimalJMTQuartic(const vector<double> &start, const vector<double> &end, double t, double max_square_jerk) : max_square_jerk_(max_square_jerk), start_(start), end_(end), t_(t) {}

std::pair<vector<double>, double> OptimalJMTQuartic::OptimalTrajectory()
{
  vector<double> result;
  const double current_velocity = start_[0];
  const double target_velocity = end_[0];
  const double gap = (target_velocity - current_velocity) / 10.0;
  double velocity = target_velocity;
  for (int i = 0; i < 10; ++i)
  {
    end_[0] = velocity;
    JMTQuartic trajectory(start_, end_, t_);
    if (trajectory.sum_jerk() < max_square_jerk_)
    {
      //AINFO<<"lon jerk : "<<trajectory.sum_jerk();
      return std::make_pair(trajectory.vals(), t_);
    }
    result = trajectory.vals();
    velocity -= gap;
  }
  // AINFO<<"No trajectory's jerk is under "<<max_square_jerk_<<". Return the best one with velocity "<<end_[0]<<".";
  return std::make_pair(result, t_);
}

std::pair<vector<double>, double> OptimalJMTQuartic::OptimalTrajectory(double &jerk)
{
  vector<double> result;
  const double current_velocity = start_[0];
  const double target_velocity = end_[0];
  const double gap = (target_velocity - current_velocity) / 10.0;
  double velocity = target_velocity;
  for (int i = 0; i < 10; ++i)
  {
    end_[0] = velocity;
    JMTQuartic trajectory(start_, end_, t_);
    if (trajectory.sum_jerk() < max_square_jerk_)
    {
      //AINFO<<"lon jerk : "<<trajectory.sum_jerk();
      jerk = trajectory.sum_jerk();
      return std::make_pair(trajectory.vals(), t_);
    }
    result = trajectory.vals();
    jerk = trajectory.sum_jerk();
    velocity -= gap;
  }
  
  // AINFO<<"No trajectory's jerk is under "<<max_square_jerk_<<". Return the best one with velocity "<<end_[0]<<".";
  return std::make_pair(result, t_);
}

} // namespace planning
