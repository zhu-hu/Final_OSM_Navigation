//
// Created by luyifan on 19-11-10.
//

#ifndef PLANNING_SCENARIO_H
#define PLANNING_SCENARIO_H

namespace planning
{
enum ScenarioType
{
  LANEFOLLOW = 0,
  INTERSECTION = 1,
  COUNTRYROAD = 2,
  TAKEPASSENGER = 3,
  PARKING = 4,
  CREEP = 5,
  ROADCHECK = 6,
  UTURN = 7,
  RAMP = 8,
  UNPROTECTED = 9,
  SIDEWALK = 10,
  LEFTLANECHANGE = 11,
  RIGHTLANECHANGE = 12,
  CAUTIOUSZONE = 13
};

enum IntersectionType
{
  NOTURN = 0,
  LEFTTURN = 1,
  RIGHTTURN = 2
};

struct SignalLocation
{
  double utm_x;
  double utm_y;
  double height;
};

struct TaskEndPoint
{
  double utm_x;
  double utm_y;
  double heading;
};
struct Scenario
{
  ScenarioType type;
  double s;
  IntersectionType intersection_type;
  SignalLocation signal_location;
  double stop_time;
  double stop_sign_s;
  TaskEndPoint task_end_point;
  TaskEndPoint reverse_end_point;
  TaskEndPoint roadchek_end_point;
};
} // namespace planning

#endif //PLANNING_SCENARIO_H
