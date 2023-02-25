/**
  *ekf_pose.cpp
  *brief:ekf fusion of gps and slam and imu
  *author:Chen Xiaofeng
  *date:20191028
  **/

#include "ekf_pose.h"

void All_EKF_Pose::gpsStateUpdate(Eigen::Vector3d &Z_gps, const Eigen::Matrix3d &R_gps, double time)
{
    if (!gps_init_flag || !vel_init_flag)
    {
        X.head(3) = Z_gps;
        P.topLeftCorner(3,3) = R_gps;
        time_now = time;
        gps_init_flag = true;
        return;
    }

    if(Z_gps(2) > X(2) + M_PI) Z_gps(2) -= 2 * M_PI;
    else if (Z_gps(2) < X(2) - M_PI) Z_gps(2) += 2 * M_PI;

    //prediction
    double dt = time - time_now;
    time_now = time;
    // std::cout<<"P="<<P<<std::endl;
    statePrediction(dt);
    // std::cout<<"P="<<P<<std::endl;
    //update
    Eigen::MatrixXd H(3,5);
    Eigen::MatrixXd I(5,5);
    Eigen::MatrixXd S(3,3);
    Eigen::MatrixXd K(5,3);
    Eigen::Vector3d Y;

    H << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0;
    I = Eigen::MatrixXd::Identity(5,5);

    if(Z_gps(2) > X(2) + M_PI) Z_gps(2) -= 2 * M_PI;
    else if (Z_gps(2) < X(2) - M_PI) Z_gps(2) += 2 * M_PI;

    Y = Z_gps - H * X;
    S = H * P * H.transpose() + R_gps;
    K = P * H.transpose() * S.inverse();
    X = X + K * Y;
    P = (I - K * H) * P;
    // std::cout<<"S="<<S<<std::endl;
    // std::cout<<"K="<<K<<std::endl;
    // std::cout<<"P="<<P<<std::endl;
    constrainRadian(X(2));
}

void All_EKF_Pose::gpsStateUpdate(Eigen::Vector2d &Z_gps, const Eigen::Matrix2d &R_gps, double time)
{
    if (!gps_init_flag || !vel_init_flag)
    {
        X.head(2) = Z_gps;
        P.topLeftCorner(2,2) = R_gps;
        time_now = time;
        gps_init_flag = true;
        return;
    }

    //prediction
    double dt = time - time_now;
    time_now = time;
    // std::cout<<"P="<<P<<std::endl;
    statePrediction(dt);
    // std::cout<<"P="<<P<<std::endl;
    //update
    Eigen::MatrixXd H(2,5);
    Eigen::MatrixXd I(5,5);
    Eigen::MatrixXd S(2,2);
    Eigen::MatrixXd K(5,2);
    Eigen::Vector2d Y;

    H << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0;
    I = Eigen::MatrixXd::Identity(5,5);

    // if(Z_gps(2) > X(2) + M_PI) Z_gps(2) -= 2 * M_PI;
    // else if (Z_gps(2) < X(2) - M_PI) Z_gps(2) += 2 * M_PI;

    Y = Z_gps - H * X;
    S = H * P * H.transpose() + R_gps;
    K = P * H.transpose() * S.inverse();
    X = X + K * Y;
    P = (I - K * H) * P;
    // std::cout<<"S="<<S<<std::endl;
    // std::cout<<"K="<<K<<std::endl;
    // std::cout<<"P="<<P<<std::endl;
    constrainRadian(X(2));
}

geometry_msgs::Pose2D All_EKF_Pose::slamStateUpdate(Eigen::Vector3d &Z_slam, const Eigen::Matrix3d &R_slam, const double time)
{
    geometry_msgs::Pose2D diff_pose;
    if (!gps_init_flag || !vel_init_flag)
    {
        X.head(3) = Z_slam;
        P.topLeftCorner(3,3) = R_slam;
        time_now = time;
        gps_init_flag = true;
        return diff_pose;
    }

    if(Z_slam(2) > X(2) + M_PI) Z_slam(2) -= 2 * M_PI;
    else if (Z_slam(2) < X(2) - M_PI) Z_slam(2) += 2 * M_PI;

    //prediction
    double dt = time - time_now;
    time_now = time;
    statePrediction(dt);

    //update
    Eigen::MatrixXd H(3,5);
    Eigen::MatrixXd I(5,5);
    Eigen::MatrixXd S(3,3);
    Eigen::MatrixXd K(5,3);
    Eigen::Vector3d Y;

    H << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0;
    I = Eigen::MatrixXd::Identity(5,5);
    if(Z_slam(2) > X(2) + M_PI) Z_slam(2) -= 2 * M_PI;
    else if (Z_slam(2) < X(2) - M_PI) Z_slam(2) += 2 * M_PI;

    diff_pose.x = Z_slam(0) - X(0);
    diff_pose.y = Z_slam(1) - X(1);
    diff_pose.theta = Z_slam(2) - X(2);
 
    Y = Z_slam - H * X;
    S = H * P * H.transpose() + R_slam;
    K = P * H.transpose() * S.inverse();
    X = X + K * Y;
    P = (I - K * H) * P;

    constrainRadian(X(2));

    return diff_pose;
}

void All_EKF_Pose::velStateUpdate(const Eigen::Vector2d &Z_vel, const Eigen::Matrix2d &R_vel, double time)
{
    if (!gps_init_flag || !vel_init_flag)
    {
        X.tail<2>() = Z_vel;
        P.bottomRightCorner(2,2) = R_vel;
        time_now = time;
        vel_init_flag = true;
        return;
    }

    //prediction
    double dt = time - time_now;
    time_now = time;
    statePrediction(dt);

    //update
    Eigen::MatrixXd H(2,5);
    Eigen::MatrixXd I(5,5);
    Eigen::MatrixXd S(2,2);
    Eigen::MatrixXd K(5,2);
    Eigen::Vector2d Y;

    H << 0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
    I = Eigen::MatrixXd::Identity(5,5);

    Y = Z_vel - H * X;
    S = H * P * H.transpose() + R_vel;
    K = P * H.transpose() * S.inverse();
    X = X + K * Y;
    P = (I - K * H) * P;

    constrainRadian(X(2));
}

Eigen::VectorXd All_EKF_Pose::readX(double time,bool prediction)
{
    //prediction
    double dt = time - time_now;
    time_now = time;
    if(prediction) statePrediction(dt);

    return X;
}

Eigen::MatrixXd All_EKF_Pose::readP(double time,bool prediction)
{
    //prediction
    double dt = time - time_now;
    time_now = time;
    if(prediction) statePrediction(dt);

    return P;
}


void All_EKF_Pose::timeUpdate(double time)
{
    X(3) = 0.0001;
    X(4) = 0.0001;
    time_now = time;
}

void All_EKF_Pose::statePrediction(double dt)
{
    double x = X(0);
    double y = X(1);
    double yaw = X(2);
    double v = X(3);
    double w = X(4);

    Eigen::MatrixXd F(5,5);
    Eigen::MatrixXd Q(5,5);
    Eigen::MatrixXd G(5,2);
    Eigen::MatrixXd E(2,2);

    X(0) = x + v/w*(sin(yaw+w*dt)-sin(yaw));
    X(1) = y + v/w*(cos(yaw)-cos(yaw+w*dt));
    X(2) = yaw + w*dt;
    X(3) = v;
    X(4) = w;

    F << 1, 0, (v*(cos(yaw + dt*w) - cos(yaw)))/w,  (sin(yaw + dt*w) - sin(yaw))/w, (dt*v*cos(yaw + dt*w))/w - (v*(sin(yaw + dt*w) - sin(yaw)))/pow(w,2),
            0, 1, (v*(sin(yaw + dt*w) - sin(yaw)))/w, -(cos(yaw + dt*w) - cos(yaw))/w, (v*(cos(yaw + dt*w) - cos(yaw)))/pow(w,2) + (dt*v*sin(yaw + dt*w))/w,
            0, 0,                                  1,                               0,                                                                   dt,
            0, 0,                                  0,                               1,                                                                    0,
            0, 0,                                  0,                               0,                                                                    1;
    G << cos(yaw)*pow(dt,2)/2, 0,
            sin(yaw)*pow(dt,2)/2, 0,
            0, pow(dt,2)/2,
            dt, 0,
            0, dt;
    E << pow(5,2), 0, //5
            0, pow(0.1,2);    //0.1 
    Q = G * E * G.transpose();
    P = F * P * F.transpose() + Q;

    constrainRadian(X(2));
}

void All_EKF_Pose::constrainRadian(double &x)
{
    if (x >= M_PI)
        x -= 2*M_PI;
    if (x < -M_PI)
        x += 2*M_PI;
}

void All_EKF_Pose::Reset()
{
    gps_init_flag = false;
    vel_init_flag = false;
    time_now = 0;

    X = Eigen::VectorXd::Zero(5);
    P = Eigen::MatrixXd::Zero(5,5);
}