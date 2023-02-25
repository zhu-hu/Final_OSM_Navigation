#include "dubins_curve.h"

namespace hybrid_a_star {
DubinsCurve::DubinsCurve() {}

DubinsCurve::~DubinsCurve() {}

int DubinsCurve::dubins_shortest_path(DubinsPath* path, double q0[3],
                                      double q1[3], double rho) {
  int i, errcode;
  DubinsIntermediateResults in;
  double params[3];
  double cost;
  double best_cost = INFINITY;
  int best_word = -1;
  errcode = dubins_intermediate_results(&in, q0, q1, rho);
  if (errcode != EDUBOK) {
    return errcode;
  }

  path->qi[0] = q0[0];
  path->qi[1] = q0[1];
  path->qi[2] = q0[2];
  path->rho = rho;

  for (i = 0; i < 6; i++) {
    DubinsPathType pathType = (DubinsPathType)i;
    errcode = dubins_word(&in, pathType, params);
    if (errcode == EDUBOK) {
      cost = params[0] + params[1] + params[2];
      if (cost < best_cost) {
        best_word = i;
        best_cost = cost;
        path->param[0] = params[0];
        path->param[1] = params[1];
        path->param[2] = params[2];
        path->type = pathType;
      }
    }
  }
  if (best_word == -1) {
    return EDUBNOPATH;
  }
  return EDUBOK;
}

int DubinsCurve::dubins_path(DubinsPath* path, double q0[3], double q1[3],
                             double rho, DubinsPathType pathType) {
  int errcode;
  DubinsIntermediateResults in;
  errcode = dubins_intermediate_results(&in, q0, q1, rho);
  if (errcode == EDUBOK) {
    double params[3];
    errcode = dubins_word(&in, pathType, params);
    if (errcode == EDUBOK) {
      path->param[0] = params[0];
      path->param[1] = params[1];
      path->param[2] = params[2];
      path->qi[0] = q0[0];
      path->qi[1] = q0[1];
      path->qi[2] = q0[2];
      path->rho = rho;
      path->type = pathType;
    }
  }
  return errcode;
}

double DubinsCurve::dubins_path_length(DubinsPath* path) {
  double length = 0.;
  length += path->param[0];
  length += path->param[1];
  length += path->param[2];
  length = length * path->rho;
  return length;
}

double DubinsCurve::dubins_segment_length(DubinsPath* path, int i) {
  if ((i < 0) || (i > 2)) {
    return INFINITY;
  }
  return path->param[i] * path->rho;
}

double DubinsCurve::dubins_segment_length_normalized(DubinsPath* path, int i) {
  if ((i < 0) || (i > 2)) {
    return INFINITY;
  }
  return path->param[i];
}

DubinsPathType DubinsCurve::dubins_path_type(DubinsPath* path) {
  return path->type;
}

void DubinsCurve::dubins_segment(double t, double qi[3], double qt[3],
                                 DubinsSegmentType type) {
  double st = sin(qi[2]);
  double ct = cos(qi[2]);
  if (type == DubinsSegmentType::L_SEG) {
    qt[0] = +sin(qi[2] + t) - st;
    qt[1] = -cos(qi[2] + t) + ct;
    qt[2] = t;
  } else if (type == DubinsSegmentType::R_SEG) {
    qt[0] = -sin(qi[2] - t) + st;
    qt[1] = +cos(qi[2] - t) - ct;
    qt[2] = -t;
  } else if (type == DubinsSegmentType::S_SEG) {
    qt[0] = ct * t;
    qt[1] = st * t;
    qt[2] = 0.0;
  }
  qt[0] += qi[0];
  qt[1] += qi[1];
  qt[2] += qi[2];
}

int DubinsCurve::dubins_path_sample(DubinsPath* path, double t, double q[3]) {
  /* tprime is the normalised variant of the parameter t */
  double tprime = t / path->rho;
  double qi[3]; /* The translated initial configuration */
  double q1[3]; /* end-of segment 1 */
  double q2[3]; /* end-of segment 2 */
  const DubinsSegmentType* types = DIRDATA[path->type];
  double p1, p2;

  if (t < 0 || t > dubins_path_length(path)) {
    return EDUBPARAM;
  }

  /* initial configuration */
  qi[0] = 0.0;
  qi[1] = 0.0;
  qi[2] = path->qi[2];

  /* generate the target configuration */
  p1 = path->param[0];
  p2 = path->param[1];
  dubins_segment(p1, qi, q1, types[0]);
  dubins_segment(p2, q1, q2, types[1]);
  if (tprime < p1) {
    dubins_segment(tprime, qi, q, types[0]);
  } else if (tprime < (p1 + p2)) {
    dubins_segment(tprime - p1, q1, q, types[1]);
  } else {
    dubins_segment(tprime - p1 - p2, q2, q, types[2]);
  }

  /* scale the target configuration, translate back to the original starting
   * point */
  q[0] = q[0] * path->rho + path->qi[0];
  q[1] = q[1] * path->rho + path->qi[1];
  q[2] = mod2pi(q[2]);

  return EDUBOK;
}

int DubinsCurve::dubins_path_sample_many(DubinsPath* path, double stepSize) {
  int retcode;
  double q[3];
  double x = 0.0;
  double length = dubins_path_length(path);
  path_points_.clear();
  geometry_msgs::Pose point;
  while (x < length) {
    dubins_path_sample(path, x, q);
    // retcode = cb(q, x, user_data);
    // if (retcode != 0) {
    //   return retcode;
    // }
    point.position.x = q[0];
    point.position.y = q[1];
    point.position.z = 0.0;
    auto tf_q = tf::createQuaternionFromYaw(q[2]);
    point.orientation.w = tf_q.getW();
    point.orientation.x = tf_q.getX();
    point.orientation.y = tf_q.getY();
    point.orientation.z = tf_q.getZ();
    path_points_.emplace_back(point);
    x += stepSize;
  }
  return 0;
}

int DubinsCurve::dubins_path_endpoint(DubinsPath* path, double q[3]) {
  return dubins_path_sample(path, dubins_path_length(path) - EPSILON, q);
}

int DubinsCurve::dubins_extract_subpath(DubinsPath* path, double t,
                                        DubinsPath* newpath) {
  /* calculate the true parameter */
  double tprime = t / path->rho;

  if ((t < 0) || (t > dubins_path_length(path))) {
    return EDUBPARAM;
  }

  /* copy most of the data */
  newpath->qi[0] = path->qi[0];
  newpath->qi[1] = path->qi[1];
  newpath->qi[2] = path->qi[2];
  newpath->rho = path->rho;
  newpath->type = path->type;
  /* fix the parameters */
  newpath->param[0] = fmin(path->param[0], tprime);
  newpath->param[1] = fmin(path->param[1], tprime - newpath->param[0]);
  newpath->param[2] =
      fmin(path->param[2], tprime - newpath->param[0] - newpath->param[1]);
  return 0;
}

int DubinsCurve::dubins_intermediate_results(DubinsIntermediateResults* in,
                                             double q0[3], double q1[3],
                                             double rho) {
  double dx, dy, D, d, theta, alpha, beta;
  if (rho <= 0.0) {
    return EDUBBADRHO;
  }

  dx = q1[0] - q0[0];
  dy = q1[1] - q0[1];
  D = sqrt(dx * dx + dy * dy);
  d = D / rho;
  theta = 0;

  /* test required to prevent domain errors if dx=0 and dy=0 */
  if (d > 0) {
    theta = mod2pi(atan2(dy, dx));
  }
  alpha = mod2pi(q0[2] - theta);
  beta = mod2pi(q1[2] - theta);

  in->alpha = alpha;
  in->beta = beta;
  in->d = d;
  in->sa = sin(alpha);
  in->sb = sin(beta);
  in->ca = cos(alpha);
  in->cb = cos(beta);
  in->c_ab = cos(alpha - beta);
  in->d_sq = d * d;

  return EDUBOK;
}

int DubinsCurve::dubins_LSL(DubinsIntermediateResults* in, double out[3]) {
  double tmp0, tmp1, p_sq;

  tmp0 = in->d + in->sa - in->sb;
  p_sq = 2 + in->d_sq - (2 * in->c_ab) + (2 * in->d * (in->sa - in->sb));

  if (p_sq >= 0) {
    tmp1 = atan2((in->cb - in->ca), tmp0);
    out[0] = mod2pi(tmp1 - in->alpha);
    out[1] = sqrt(p_sq);
    out[2] = mod2pi(in->beta - tmp1);
    return EDUBOK;
  }
  return EDUBNOPATH;
}

int DubinsCurve::dubins_RSR(DubinsIntermediateResults* in, double out[3]) {
  double tmp0 = in->d - in->sa + in->sb;
  double p_sq = 2 + in->d_sq - (2 * in->c_ab) + (2 * in->d * (in->sb - in->sa));
  if (p_sq >= 0) {
    double tmp1 = atan2((in->ca - in->cb), tmp0);
    out[0] = mod2pi(in->alpha - tmp1);
    out[1] = sqrt(p_sq);
    out[2] = mod2pi(tmp1 - in->beta);
    return EDUBOK;
  }
  return EDUBNOPATH;
}

int DubinsCurve::dubins_LSR(DubinsIntermediateResults* in, double out[3]) {
  double p_sq =
      -2 + (in->d_sq) + (2 * in->c_ab) + (2 * in->d * (in->sa + in->sb));
  if (p_sq >= 0) {
    double p = sqrt(p_sq);
    double tmp0 =
        atan2((-in->ca - in->cb), (in->d + in->sa + in->sb)) - atan2(-2.0, p);
    out[0] = mod2pi(tmp0 - in->alpha);
    out[1] = p;
    out[2] = mod2pi(tmp0 - mod2pi(in->beta));
    return EDUBOK;
  }
  return EDUBNOPATH;
}

int DubinsCurve::dubins_RSL(DubinsIntermediateResults* in, double out[3]) {
  double p_sq =
      -2 + in->d_sq + (2 * in->c_ab) - (2 * in->d * (in->sa + in->sb));
  if (p_sq >= 0) {
    double p = sqrt(p_sq);
    double tmp0 =
        atan2((in->ca + in->cb), (in->d - in->sa - in->sb)) - atan2(2.0, p);
    out[0] = mod2pi(in->alpha - tmp0);
    out[1] = p;
    out[2] = mod2pi(in->beta - tmp0);
    return EDUBOK;
  }
  return EDUBNOPATH;
}

int DubinsCurve::dubins_RLR(DubinsIntermediateResults* in, double out[3]) {
  double tmp0 =
      (6. - in->d_sq + 2 * in->c_ab + 2 * in->d * (in->sa - in->sb)) / 8.;
  double phi = atan2(in->ca - in->cb, in->d - in->sa + in->sb);
  if (fabs(tmp0) <= 1) {
    double p = mod2pi((2 * M_PI) - acos(tmp0));
    double t = mod2pi(in->alpha - phi + mod2pi(p / 2.));
    out[0] = t;
    out[1] = p;
    out[2] = mod2pi(in->alpha - in->beta - t + mod2pi(p));
    return EDUBOK;
  }
  return EDUBNOPATH;
}

int DubinsCurve::dubins_LRL(DubinsIntermediateResults* in, double out[3]) {
  double tmp0 =
      (6. - in->d_sq + 2 * in->c_ab + 2 * in->d * (in->sb - in->sa)) / 8.;
  double phi = atan2(in->ca - in->cb, in->d + in->sa - in->sb);
  if (fabs(tmp0) <= 1) {
    double p = mod2pi(2 * M_PI - acos(tmp0));
    double t = mod2pi(-in->alpha - phi + p / 2.);
    out[0] = t;
    out[1] = p;
    out[2] = mod2pi(mod2pi(in->beta) - in->alpha - t + mod2pi(p));
    return EDUBOK;
  }
  return EDUBNOPATH;
}

int DubinsCurve::dubins_word(DubinsIntermediateResults* in,
                             DubinsPathType pathType, double out[3]) {
  int result;
  switch (pathType) {
    case LSL:
      result = dubins_LSL(in, out);
      break;
    case RSL:
      result = dubins_RSL(in, out);
      break;
    case LSR:
      result = dubins_LSR(in, out);
      break;
    case RSR:
      result = dubins_RSR(in, out);
      break;
    case LRL:
      result = dubins_LRL(in, out);
      break;
    case RLR:
      result = dubins_RLR(in, out);
      break;
    default:
      result = EDUBNOPATH;
  }
  return result;
}

void DubinsCurve::GeneratePathFromDubinsPath(const DubinsPath* path) {
  DubinsPathType type = path->type;
  switch (type) {
    case LSL:
      ROS_INFO("LSL Analyse");
      analyse_LSL_curve(path);
      break;
    case RSL:
      ROS_INFO("RSL Analyse");
      analyse_RSL_curve(path);
      break;
    case LSR:
      ROS_INFO("LSR Analyse");
      analyse_LSR_curve(path);
      break;
    case RSR:
      ROS_INFO("RSR Analyse");
      analyse_RSR_curve(path);
      break;
    case LRL:
      ROS_INFO("LRL Analyse");
      analyse_LRL_curve(path);
      break;
    case RLR:
      ROS_INFO("RLR Analyse");
      analyse_RLR_curve(path);
      break;
    default:
      ROS_INFO("Type Invalid");
      break;
  }
}

/***********************************************************
 * 左转（L）
 * x = x_0 + R * (sin(theta_0 + t) - sin(theta_0))
 * y = y_0 + R * (cos(theta_0) - cos(theta_0 + t))
 * theta = theta_0 + t
 * 右转（R）
 * x = x_0 + R * (sin(theta_0) - sin(theta_0 - t))
 * y = y_0 + R * (cos(theta_0) + cos(theta_0 - t))
 * theta = theta_0 - t
 * 直行（S）
 * x = x_0 + R * t * cos(theta_0)
 * y = y_0 + R * t * sin(theta_0)
 * theta = theta_0
 * ********************************************************/

void DubinsCurve::analyse_LSL_curve(const DubinsPath* path) {
  double start_x = path->qi[0];
  double start_y = path->qi[1];
  double start_yaw = mod2pi(path->qi[2]);

  double R = path->rho;

  double first_length = path->param[0];
  double second_length = path->param[1];
  double third_length = path->param[2];

  // L
  geometry_msgs::Pose first_end_point;
  first_end_point.position.x =
      start_x +
      R * (sin(mod2pi(first_length + start_yaw)) - sin(mod2pi(start_yaw)));
  first_end_point.position.y =
      start_y +
      R * (cos(mod2pi(start_yaw)) - cos(mod2pi(first_length + start_yaw)));
  first_end_point.position.z = 0.0;
  double first_end_point_yaw = mod2pi(start_yaw + first_length);

  // S
  geometry_msgs::Pose second_end_point;
  second_end_point.position.x =
      first_end_point.position.x + R * second_length * cos(first_end_point_yaw);
  second_end_point.position.y =
      first_end_point.position.y + R * second_length * sin(first_end_point_yaw);
  second_end_point.position.z = 0.0;
  double second_end_point_yaw = first_end_point_yaw;

  // L
  geometry_msgs::Pose third_end_point;
  third_end_point.position.x =
      second_end_point.position.x +
      R * (sin(mod2pi(second_end_point_yaw + third_length)) -
           sin(second_end_point_yaw));
  third_end_point.position.y =
      second_end_point.position.y +
      R * (cos(second_end_point_yaw) -
           cos(mod2pi(second_end_point_yaw + third_length)));
  third_end_point.position.z = 0.0;
  double third_end_point_yaw = mod2pi(second_end_point_yaw + third_length);
  path_points_.clear();
  geometry_msgs::Pose point;
  point.position.x = start_x;
  point.position.y = start_y;
  point.position.z = 0.0;
  path_points_.emplace_back(point);

  // L
  double t = 0.0;
  while (t <= first_length) {
    point.position.x =
        start_x + R * (sin(mod2pi(t + start_yaw)) - sin(start_yaw));
    point.position.y =
        start_y + R * (cos(start_yaw) - cos(mod2pi(t + start_yaw)));
    path_points_.emplace_back(point);
    t += step_t_;
  }

  // S
  t = 0.0;
  while (t <= second_length) {
    point.position.x =
        first_end_point.position.x + R * t * cos(first_end_point_yaw);
    point.position.y =
        first_end_point.position.y + R * t * sin(first_end_point_yaw);
    path_points_.emplace_back(point);
    t += step_t_;
  }

  // L
  t = 0.0;
  while (t <= third_length) {
    point.position.x =
        second_end_point.position.x +
        R * (sin(mod2pi(second_end_point_yaw + t)) - sin(second_end_point_yaw));
    point.position.y =
        second_end_point.position.y +
        R * (cos(second_end_point_yaw) - cos(mod2pi(second_end_point_yaw + t)));
    path_points_.emplace_back(point);
    t += step_t_;
  }
}

void DubinsCurve::analyse_RSR_curve(const DubinsPath* path) {
  double start_x = path->qi[0];
  double start_y = path->qi[1];
  double start_yaw = mod2pi(path->qi[2]);

  double R = path->rho;

  double first_length = path->param[0];
  double second_length = path->param[1];
  double third_length = path->param[2];

  // R
  geometry_msgs::Pose first_end_point;
  first_end_point.position.x =
      start_x + R * (sin(start_yaw) - sin(mod2pi(start_yaw - first_length)));
  first_end_point.position.y =
      start_y + R * (cos(start_yaw) + cos(mod2pi(start_yaw - first_length)));
  first_end_point.position.z = 0.0;
  double first_end_point_yaw = mod2pi(start_yaw - first_length);

  // S
  geometry_msgs::Pose second_end_point;
  second_end_point.position.x =
      first_end_point.position.x + R * second_length * cos(first_end_point_yaw);
  second_end_point.position.y =
      first_end_point.position.y + R * second_length * sin(first_end_point_yaw);
  second_end_point.position.z = 0.0;
  double second_end_point_yaw = first_end_point_yaw;

  // R
  geometry_msgs::Pose third_end_point;
  third_end_point.position.x =
      second_end_point.position.x +
      R * (sin(second_end_point_yaw) -
           sin(mod2pi(second_end_point_yaw - third_length)));
  third_end_point.position.y =
      second_end_point.position.y +
      R * (cos(second_end_point_yaw) +
           cos(mod2pi(second_end_point_yaw - third_length)));
  third_end_point.position.z = 0.0;
  double third_end_point_yaw = mod2pi(second_end_point_yaw - third_length);
  path_points_.clear();
  geometry_msgs::Pose point;
  point.position.x = start_x;
  point.position.y = start_y;
  point.position.z = 0.0;
  path_points_.emplace_back(point);

  // R
  double t = 0.0;
  while (t <= first_length) {
    point.position.x =
        start_x + R * (sin(start_yaw) - sin(mod2pi(start_yaw - t)));
    point.position.y =
        start_y + R * (cos(start_yaw) + cos(mod2pi(start_yaw - t)));
    path_points_.emplace_back(point);
    t += step_t_;
  }

  // S
  t = 0.0;
  while (t <= second_length) {
    point.position.x =
        first_end_point.position.x + R * t * cos(first_end_point_yaw);
    point.position.y =
        first_end_point.position.y + R * t * sin(first_end_point_yaw);
    path_points_.emplace_back(point);
    t += step_t_;
  }

  // R
  t = 0.0;
  while (t <= third_length) {
    point.position.x =
        second_end_point.position.x +
        R * (sin(second_end_point_yaw) - sin(mod2pi(second_end_point_yaw - t)));
    point.position.y =
        second_end_point.position.y +
        R * (cos(second_end_point_yaw) + cos(mod2pi(second_end_point_yaw - t)));
    path_points_.emplace_back(point);
    t += step_t_;
  }
}

void DubinsCurve::analyse_LSR_curve(const DubinsPath* path) {
  double start_x = path->qi[0];
  double start_y = path->qi[1];
  double start_yaw = mod2pi(path->qi[2]);

  double R = path->rho;

  double first_length = path->param[0];
  double second_length = path->param[1];
  double third_length = path->param[2];

  // L
  geometry_msgs::Pose first_end_point;
  first_end_point.position.x =
      start_x + R * (sin(mod2pi(first_length + start_yaw)) - sin(start_yaw));
  first_end_point.position.y =
      start_y + R * (cos(start_yaw) - cos(mod2pi(first_length + start_yaw)));
  first_end_point.position.z = 0.0;
  double first_end_point_yaw = mod2pi(start_yaw + first_length);

  // S
  geometry_msgs::Pose second_end_point;
  second_end_point.position.x =
      first_end_point.position.x + R * second_length * cos(first_end_point_yaw);
  second_end_point.position.y =
      first_end_point.position.y + R * second_length * sin(first_end_point_yaw);
  second_end_point.position.z = 0.0;
  double second_end_point_yaw = first_end_point_yaw;

  // R
  geometry_msgs::Pose third_end_point;
  third_end_point.position.x =
      second_end_point.position.x +
      R * (sin(second_end_point_yaw) -
           sin(mod2pi(second_end_point_yaw - third_length)));
  third_end_point.position.y =
      second_end_point.position.y +
      R * (cos(second_end_point_yaw) +
           cos(mod2pi(second_end_point_yaw - third_length)));
  third_end_point.position.z = 0.0;
  double third_end_point_yaw = mod2pi(second_end_point_yaw - third_length);
  path_points_.clear();
  geometry_msgs::Pose point;
  point.position.x = start_x;
  point.position.y = start_y;
  point.position.z = 0.0;
  path_points_.emplace_back(point);

  // L
  double t = 0.0;
  while (t <= first_length) {
    point.position.x =
        start_x + R * (sin(mod2pi(t + start_yaw)) - sin(start_yaw));
    point.position.y =
        start_y + R * (cos(start_yaw) - cos(mod2pi(t + start_yaw)));
    path_points_.emplace_back(point);
    t += step_t_;
  }

  // S
  t = 0.0;
  while (t <= second_length) {
    point.position.x =
        first_end_point.position.x + R * t * cos(first_end_point_yaw);
    point.position.y =
        first_end_point.position.y + R * t * sin(first_end_point_yaw);
    path_points_.emplace_back(point);
    t += step_t_;
  }

  // R
  t = 0.0;
  while (t <= third_length) {
    point.position.x =
        second_end_point.position.x +
        R * (sin(second_end_point_yaw) - sin(mod2pi(second_end_point_yaw - t)));
    point.position.y =
        second_end_point.position.y +
        R * (cos(second_end_point_yaw) + cos(mod2pi(second_end_point_yaw - t)));
    path_points_.emplace_back(point);
    t += step_t_;
  }
}

void DubinsCurve::analyse_RSL_curve(const DubinsPath* path) {
  double start_x = path->qi[0];
  double start_y = path->qi[1];
  double start_yaw = mod2pi(path->qi[2]);

  double R = path->rho;

  double first_length = path->param[0];
  double second_length = path->param[1];
  double third_length = path->param[2];

  // R
  geometry_msgs::Pose first_end_point;
  first_end_point.position.x =
      start_x + R * (sin(start_yaw) - sin(mod2pi(start_yaw - first_length)));
  first_end_point.position.y =
      start_y + R * (cos(start_yaw) + cos(mod2pi(start_yaw - first_length)));
  first_end_point.position.z = 0.0;
  double first_end_point_yaw = mod2pi(start_yaw - first_length);

  // S
  geometry_msgs::Pose second_end_point;
  second_end_point.position.x =
      first_end_point.position.x + R * second_length * cos(first_end_point_yaw);
  second_end_point.position.y =
      first_end_point.position.y + R * second_length * sin(first_end_point_yaw);
  second_end_point.position.z = 0.0;
  double second_end_point_yaw = first_end_point_yaw;

  // L
  geometry_msgs::Pose third_end_point;
  third_end_point.position.x =
      second_end_point.position.x +
      R * (sin(mod2pi(second_end_point_yaw + third_length)) -
           sin(second_end_point_yaw));
  third_end_point.position.y =
      second_end_point.position.y +
      R * (cos(second_end_point_yaw) -
           cos(mod2pi(second_end_point_yaw + third_length)));
  third_end_point.position.z = 0.0;
  double third_end_point_yaw = mod2pi(second_end_point_yaw + third_length);
  path_points_.clear();
  geometry_msgs::Pose point;
  point.position.x = start_x;
  point.position.y = start_y;
  point.position.z = 0.0;
  path_points_.emplace_back(point);

  // R
  double t = 0.0;
  while (t <= first_length) {
    point.position.x =
        start_x + R * (sin(start_yaw) - sin(mod2pi(start_yaw - t)));
    point.position.y =
        start_y + R * (cos(start_yaw) + cos(mod2pi(start_yaw - t)));
    path_points_.emplace_back(point);
    t += step_t_;
  }

  // S
  t = 0.0;
  while (t <= second_length) {
    point.position.x =
        first_end_point.position.x + R * t * cos(first_end_point_yaw);
    point.position.y =
        first_end_point.position.y + R * t * sin(first_end_point_yaw);
    path_points_.emplace_back(point);
    t += step_t_;
  }

  // L
  t = 0.0;
  while (t <= third_length) {
    point.position.x =
        second_end_point.position.x +
        R * (mod2pi(sin(second_end_point_yaw + t)) - sin(second_end_point_yaw));
    point.position.y =
        second_end_point.position.y +
        R * (cos(second_end_point_yaw) - cos(mod2pi(second_end_point_yaw + t)));
    path_points_.emplace_back(point);
    t += step_t_;
  }
}

void DubinsCurve::analyse_RLR_curve(const DubinsPath* path) {
  double start_x = path->qi[0];
  double start_y = path->qi[1];
  double start_yaw = mod2pi(path->qi[2]);

  double R = path->rho;

  double first_length = path->param[0];
  double second_length = path->param[1];
  double third_length = path->param[2];

  // R
  geometry_msgs::Pose first_end_point;
  first_end_point.position.x =
      start_x + R * (sin(start_yaw) - sin(mod2pi(start_yaw - first_length)));
  first_end_point.position.y =
      start_y + R * (cos(start_yaw) + cos(mod2pi(start_yaw - first_length)));
  first_end_point.position.z = 0.0;
  double first_end_point_yaw = mod2pi(start_yaw - first_length);

  // L
  geometry_msgs::Pose second_end_point;
  second_end_point.position.x =
      first_end_point.position.x +
      R * (sin(mod2pi(first_end_point_yaw + second_length)) -
           sin(first_end_point_yaw));
  second_end_point.position.y =
      first_end_point.position.y +
      R * (cos(first_end_point_yaw) -
           cos(mod2pi(first_end_point_yaw + second_length)));
  second_end_point.position.z = 0.0;
  double second_end_point_yaw = mod2pi(first_end_point_yaw + second_length);

  // R
  geometry_msgs::Pose third_end_point;
  third_end_point.position.x =
      second_end_point.position.x +
      R * (sin(second_end_point_yaw) -
           sin(mod2pi(second_end_point_yaw - third_length)));
  third_end_point.position.y =
      second_end_point.position.y +
      R * (cos(second_end_point_yaw) +
           cos(mod2pi(second_end_point_yaw - third_length)));
  third_end_point.position.z = 0.0;
  double third_end_point_yaw = mod2pi(second_end_point_yaw - third_length);
  path_points_.clear();
  geometry_msgs::Pose point;
  point.position.x = start_x;
  point.position.y = start_y;
  point.position.z = 0.0;
  path_points_.emplace_back(point);

  // R
  double t = 0.0;
  while (t <= first_length) {
    point.position.x =
        start_x + R * (sin(start_yaw) - sin(mod2pi(start_yaw - t)));
    point.position.y =
        start_y + R * (cos(start_yaw) + cos(mod2pi(start_yaw - t)));
    path_points_.emplace_back(point);
    t += step_t_;
  }

  // L
  t = 0.0;
  while (t <= second_length) {
    point.position.x =
        first_end_point.position.x +
        R * (sin(mod2pi(first_end_point_yaw + t)) - sin(first_end_point_yaw));
    point.position.y =
        first_end_point.position.y +
        R * (cos(first_end_point_yaw) - cos(mod2pi(first_end_point_yaw + t)));
    path_points_.emplace_back(point);
    t += step_t_;
  }

  // R
  t = 0.0;
  while (t <= third_length) {
    point.position.x =
        second_end_point.position.x +
        R * (sin(second_end_point_yaw) - sin(mod2pi(second_end_point_yaw - t)));
    point.position.y =
        second_end_point.position.y +
        R * (cos(second_end_point_yaw) + cos(mod2pi(second_end_point_yaw - t)));
    path_points_.emplace_back(point);
    t += step_t_;
  }
}

void DubinsCurve::analyse_LRL_curve(const DubinsPath* path) {
  double start_x = path->qi[0];
  double start_y = path->qi[1];
  double start_yaw = mod2pi(path->qi[2]);

  double R = path->rho;

  double first_length = path->param[0];
  double second_length = path->param[1];
  double third_length = path->param[2];

  // L
  geometry_msgs::Pose first_end_point;
  first_end_point.position.x =
      start_x + R * (sin(mod2pi(first_length + start_yaw)) - sin(start_yaw));
  first_end_point.position.y =
      start_y + R * (cos(start_yaw) - cos(mod2pi(first_length + start_yaw)));
  first_end_point.position.z = 0.0;
  double first_end_point_yaw = mod2pi(start_yaw + first_length);

  // R
  geometry_msgs::Pose second_end_point;
  second_end_point.position.x =
      first_end_point.position.x +
      R * (sin(first_end_point_yaw) -
           sin(mod2pi(first_end_point_yaw - second_length)));
  second_end_point.position.y =
      first_end_point.position.y +
      R * (cos(first_end_point_yaw) +
           cos(mod2pi(first_end_point_yaw - second_length)));
  second_end_point.position.z = 0.0;
  double second_end_point_yaw = mod2pi(first_end_point_yaw - second_length);

  // L
  geometry_msgs::Pose third_end_point;
  third_end_point.position.x =
      second_end_point.position.x +
      R * (sin(mod2pi(second_end_point_yaw + third_length)) -
           sin(second_end_point_yaw));
  third_end_point.position.y =
      second_end_point.position.y +
      R * (cos(second_end_point_yaw) -
           cos(mod2pi(second_end_point_yaw + third_length)));
  third_end_point.position.z = 0.0;
  double third_end_point_yaw = mod2pi(second_end_point_yaw + third_length);
  path_points_.clear();
  geometry_msgs::Pose point;
  point.position.x = start_x;
  point.position.y = start_y;
  point.position.z = 0.0;
  path_points_.emplace_back(point);

  // L
  double t = 0.0;
  while (t <= first_length) {
    point.position.x =
        start_x + R * (sin(mod2pi(t + start_yaw)) - sin(start_yaw));
    point.position.y =
        start_y + R * (cos(start_yaw) - cos(mod2pi(t + start_yaw)));
    path_points_.emplace_back(point);
    t += step_t_;
  }

  // R
  t = 0.0;
  while (t <= second_length) {
    point.position.x =
        first_end_point.position.x +
        R * (sin(first_end_point_yaw) - sin(mod2pi(first_end_point_yaw - t)));
    point.position.y =
        first_end_point.position.y +
        R * (cos(first_end_point_yaw) + cos(mod2pi(first_end_point_yaw - t)));
    path_points_.emplace_back(point);
    t += step_t_;
  }

  // L
  t = 0.0;
  while (t <= third_length) {
    point.position.x =
        second_end_point.position.x +
        R * (sin(mod2pi(second_end_point_yaw + t)) - sin(second_end_point_yaw));
    point.position.y =
        second_end_point.position.y +
        R * (cos(second_end_point_yaw) - cos(mod2pi(second_end_point_yaw + t)));
    path_points_.emplace_back(point);
    t += step_t_;
  }
}

double DubinsCurve::NormalAngle(double angle) {
  double tmp = (angle + M_PI) / (2 * M_PI);
  double tmp1 = (angle + M_PI) - (int)tmp * (2 * M_PI);
  if (tmp1 < 0.0) {
    tmp1 += 2 * M_PI;
  }
  return tmp1 - M_PI;
}

}  // namespace hybrid_a_star
