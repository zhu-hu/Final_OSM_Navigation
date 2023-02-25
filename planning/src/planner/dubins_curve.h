/*
 * Copyright (c) 2008-2018, Andrew Walker
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef DUBINS_CURVE_H
#define DUBINS_CURVE_H

// #include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <vector>

#define EDUBOK (0)        /* No error */
#define EDUBCOCONFIGS (1) /* Colocated configurations */
#define EDUBPARAM (2)     /* Path parameterisitation error */
#define EDUBBADRHO (3)    /* the rho value is invalid */
#define EDUBNOPATH                                           \
  (4) /* no connection between configurations with this word \
       */

#define EPSILON (10e-10)

namespace hybrid_a_star {

typedef enum {
  LSL = 0,
  LSR = 1,
  RSL = 2,
  RSR = 3,
  RLR = 4,
  LRL = 5
} DubinsPathType;

typedef struct {
  /* the initial configuration */
  double qi[3];
  /* the lengths of the three segments */
  double param[3];
  /* model forward velocity / model angular velocity */
  double rho;
  /* the path type described */
  DubinsPathType type;
} DubinsPath;

typedef enum { L_SEG = 0, S_SEG = 1, R_SEG = 2 } DubinsSegmentType;

/* The segment types for each of the Path types */
const DubinsSegmentType DIRDATA[6][3] = {
    {L_SEG, S_SEG, L_SEG}, {L_SEG, S_SEG, R_SEG}, {R_SEG, S_SEG, L_SEG},
    {R_SEG, S_SEG, R_SEG}, {R_SEG, L_SEG, R_SEG}, {L_SEG, R_SEG, L_SEG}};

typedef struct {
  double alpha;
  double beta;
  double d;
  double sa;
  double sb;
  double ca;
  double cb;
  double c_ab;
  double d_sq;
} DubinsIntermediateResults;
class DubinsCurve {
 public:
  DubinsCurve();
  ~DubinsCurve();

  /**
   * Callback function for path sampling
   *
   * @note the q parameter is a configuration
   * @note the t parameter is the distance along the path
   * @note the user_data parameter is forwarded from the caller
   * @note return non-zero to denote sampling should be stopped
   */
  typedef int (*DubinsPathSamplingCallback)(double q[3], double t,
                                            void* user_data);

  /**
   * Generate a path from an initial configuration to
   * a target configuration, with a specified maximum turning
   * radii
   *
   * A configuration is (x, y, theta), where theta is in radians, with zero
   * along the line x = 0, and counter-clockwise is positive
   *
   * @param path  - the resultant path
   * @param q0    - a configuration specified as an array of x, y, theta
   * @param q1    - a configuration specified as an array of x, y, theta
   * @param rho   - turning radius of the vehicle (forward velocity divided by
   * maximum angular velocity)
   * @return      - non-zero on error
   */
  int dubins_shortest_path(DubinsPath* path, double q0[3], double q1[3],
                           double rho);

  /**
   * Generate a path with a specified word from an initial configuration to
   * a target configuration, with a specified turning radius
   *
   * @param path     - the resultant path
   * @param q0       - a configuration specified as an array of x, y, theta
   * @param q1       - a configuration specified as an array of x, y, theta
   * @param rho      - turning radius of the vehicle (forward velocity divided
   * by maximum angular velocity)
   * @param pathType - the specific path type to use
   * @return         - non-zero on error
   */
  int dubins_path(DubinsPath* path, double q0[3], double q1[3], double rho,
                  DubinsPathType pathType);

  /**
   * Calculate the length of an initialised path
   *
   * @param path - the path to find the length of
   */
  double dubins_path_length(DubinsPath* path);

  /**
   * Return the length of a specific segment in an initialized path
   *
   * @param path - the path to find the length of
   * @param i    - the segment you to get the length of (0-2)
   */
  double dubins_segment_length(DubinsPath* path, int i);

  /**
   * Return the normalized length of a specific segment in an initialized path
   *
   * @param path - the path to find the length of
   * @param i    - the segment you to get the length of (0-2)
   */
  double dubins_segment_length_normalized(DubinsPath* path, int i);

  /**
   * Extract an integer that represents which path type was used
   *
   * @param path    - an initialised path
   * @return        - one of LSL, LSR, RSL, RSR, RLR or LRL
   */
  DubinsPathType dubins_path_type(DubinsPath* path);

  /**
   * Calculate the configuration along the path, using the parameter t
   *
   * @param path - an initialised path
   * @param t    - a length measure, where 0 <= t < dubins_path_length(path)
   * @param q    - the configuration result
   * @returns    - non-zero if 't' is not in the correct range
   */
  int dubins_path_sample(DubinsPath* path, double t, double q[3]);

  /**
   * Walk along the path at a fixed sampling interval, calling the
   * callback function at each interval
   *
   * The sampling process continues until the whole path is sampled, or the
   * callback returns a non-zero value
   *
   * @param path      - the path to sample
   * @param stepSize  - the distance along the path for subsequent samples
   *
   * @returns - zero on successful completion, or the result of the callback
   */
  int dubins_path_sample_many(DubinsPath* path, double stepSize);

  /**
   * Convenience function to identify the endpoint of a path
   *
   * @param path - an initialised path
   * @param q    - the configuration result
   */
  int dubins_path_endpoint(DubinsPath* path, double q[3]);

  /**
   * Convenience function to extract a subset of a path
   *
   * @param path    - an initialised path
   * @param t       - a length measure, where 0 < t < dubins_path_length(path)
   * @param newpath - the resultant path
   */
  int dubins_extract_subpath(DubinsPath* path, double t, DubinsPath* newpath);

  //将DubinsPath这种数据结构的路径改成点列的路径形式
  void GeneratePathFromDubinsPath(const DubinsPath* path);

  inline const std::vector<geometry_msgs::Pose> path_points() const {
    return path_points_;
  }

 private:
  std::vector<geometry_msgs::Pose> path_points_;

  double step_t_ = 0.05;

  int dubins_word(DubinsIntermediateResults* in, DubinsPathType pathType,
                  double out[3]);
  int dubins_intermediate_results(DubinsIntermediateResults* in, double q0[3],
                                  double q1[3], double rho);

  /**
   * Floating point modulus suitable for rings
   *
   * fmod doesn't behave correctly for angular quantities, this function does
   */
  double fmodr(double x, double y) { return x - y * floor(x / y); }

  double mod2pi(double theta) { return fmodr(theta, 2 * M_PI); }

  double NormalAngle(double angle);

  void dubins_segment(double t, double qi[3], double qt[3],
                      DubinsSegmentType type);

  int dubins_LSL(DubinsIntermediateResults* in, double out[3]);

  int dubins_RSR(DubinsIntermediateResults* in, double out[3]);

  int dubins_LSR(DubinsIntermediateResults* in, double out[3]);

  int dubins_RSL(DubinsIntermediateResults* in, double out[3]);

  int dubins_RLR(DubinsIntermediateResults* in, double out[3]);

  int dubins_LRL(DubinsIntermediateResults* in, double out[3]);

  void analyse_LSL_curve(const DubinsPath* path);

  void analyse_RSR_curve(const DubinsPath* path);

  void analyse_LSR_curve(const DubinsPath* path);

  void analyse_RSL_curve(const DubinsPath* path);

  void analyse_RLR_curve(const DubinsPath* path);

  void analyse_LRL_curve(const DubinsPath* path);
};

}  // namespace hybrid_a_star

#endif