/**
 *ekf_pose.h
 *brief:localization fusion of gps, slam and imu based on EKF
 *author:Chen Xiaofeng @ SJTU CyberC3 Lab
 *date:20191028
 **/

#ifndef ALL_EKF_POSE_H
#define ALL_EKF_POSE_H
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
using namespace std;

class All_EKF_Pose {
 public:
  All_EKF_Pose() {
    gps_init_flag = false;
    vel_init_flag = false;

    X = Eigen::VectorXd::Zero(5);
    P = Eigen::MatrixXd::Zero(5, 5);
  }
  ~All_EKF_Pose() {}

  /**
   * @brief Update the state vector by observation state from GPS.
   * @param Z_gps Pose from GPS (x, y, heading).
   * @param R_gps Covariance Matrix.
   * @param time current time in second.
   */
  void gpsStateUpdate(Eigen::Vector3d &Z_gps, const Eigen::Matrix3d &R_gps,
                      const double time);

  /**
   * @brief Update the state vector by observation state from SLAM.
   * @param Z_slam Pose from SLAM (x, y, heading).
   * @param R_slam Covariance Matrix.
   * @param time current time in second.
   */
  void slamStateUpdate(Eigen::Vector3d &Z_slam, const Eigen::Matrix3d &R_slam,
                       const double time);

  /**
   * @brief Update the state vector by observation state from odometry and imu.
   * @param Z_vel motion from odometry and imu (velocity, yaw angle rate).
   * @param R_vel Covariance Matrix.
   * @param time current time in second.
   */
  void velStateUpdate(const Eigen::Vector2d &Z_vel,
                      const Eigen::Matrix2d &R_vel, const double time);

  /**
   * @brief Get state vector.
   * @param time current time in second.
   * @param prediction State prediction required or not.
   * @return The state vector (x, y, heading, velocity, yaw angle rate).
   */
  Eigen::VectorXd readX(double time, bool prediction);

  /**
   * @brief Get Matrix P.
   * @param time current time in second.
   * @param prediction State prediction required or not.
   * @return Matrix P.
   */
  Eigen::MatrixXd readP(double time, bool prediction);

  /**
   * @brief Update time but not update state vector.
   * @param time current time in second.
   */
  void timeUpdate(double time);

  /**
   * @brief Reset the filter.
   */
  void Reset(double &init_x, double &init_y, double &init_theta);

 private:
  bool gps_init_flag;
  bool vel_init_flag;
  double time_now;
  Eigen::VectorXd X;
  Eigen::MatrixXd P;
  void statePrediction(double dt);
  void constrainRadian(double &x);
};

#endif
