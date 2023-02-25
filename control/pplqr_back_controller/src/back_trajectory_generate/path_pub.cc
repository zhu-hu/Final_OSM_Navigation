#include "path_pub.h"

#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>

#include <fstream>
#include <list>
#include <memory>
#include <unordered_set>
#include <vector>

#include "common/log.h"

namespace path_pub {

PathPub::PathPub(ros::NodeHandle *nh) : nh_(nh) {
  std::string default_package_path = std::string(
      "/home/perception/Projects/Logistic_Vehicle_Net_Center/src/control/"
      "pplqr_back_controller/data/");

  nh_->param("use_laser_result", use_laser_result_, false);
  nh_->param("package_path", package_path_, default_package_path);
  nh_->param("show_real_path", show_real_path_, false);

  sub_start_ = nh_->subscribe("/start_pub", 5, &PathPub::StartCallback, this);

  if (!use_laser_result_) {
    sub_loaclization_ = nh_->subscribe("/localization/estimation", 5,
                                       &PathPub::LocalizationCallback, this);
  } else {
    sub_localization_ =
        new message_filters::Subscriber<cyber_msgs::LocalizationEstimate>(
            *nh_, "/localization/estimation", 1);
    sub_link_angle_ = new message_filters::Subscriber<cyber_msgs::LinkAngle>(
        *nh_, "/fusion_angle_degree", 1);

    motion_sync_ = new message_filters::Synchronizer<MotionPolicy>(
        MotionPolicy(20), *sub_localization_, *sub_link_angle_);

    motion_sync_->registerCallback(
        boost::bind(&PathPub::MotionCallback, this, _1, _2));
  }
  pub_trajectory_ =
      nh_->advertise<cyber_msgs::LocalTrajList>("/reverse_trailer_path", 5);

  pub_finished_ = nh_->advertise<std_msgs::Bool>("/reverse_finished", 5);

  pub_stop_index_ = nh_->advertise<std_msgs::UInt64>("/path_stop_index", 5);

  pub_rviz_global_path_ =
      nh_->advertise<nav_msgs::Path>("/rviz_global_path", 5);

  pub_rviz_pose_ =
      nh_->advertise<visualization_msgs::MarkerArray>("/rviz_pose", 5);

  pub_all_trajectories_done_ =
      nh_->advertise<std_msgs::Bool>("/all_trajectories_done", 5);

  sub_one_trajectory_finished_ =
      nh_->subscribe("/one_trajectory_finished", 5,
                     &PathPub::OneTrajectoryFinishedCallback, this);

  if (show_real_path_) {
    pub_rviz_tractor_real_path_ =
        nh_->advertise<visualization_msgs::MarkerArray>(
            "/rviz_tractor_real_path", 5);

    pub_rviz_trailer_real_path_ =
        nh_->advertise<visualization_msgs::MarkerArray>(
            "/rviz_trailer_real_path", 5);
    pub_trailer_real_pose_ =
        nh_->advertise<geometry_msgs::Point>("/trailer_real_pose", 5);
    sub_reset_real_path_ = nh_->subscribe(
        "/reset_real_path", 5, &PathPub::ResetRealPathCallback, this);
  }
}

PathPub::~PathPub() {}

void PathPub::InitCommonTrajectory(const std::string &common_trajectory_path) {
  common_trajectory_.clear();
  std::stringstream ss;
  std::ifstream fin;

  AINFO << "Now opening file:" << common_trajectory_path;
  fin.open(common_trajectory_path.c_str(), std::ios_base::in);
  if (!fin.is_open()) {
    AERROR << "Couldn't open this file!";
    return;
  }
  Point2DS points;
  std::string str;
  cyber_msgs::LocalTrajList one_trajectory;
  while (std::getline(fin, str)) {
    if (str.length() > 1) {
      cyber_msgs::LocalTrajPoint trajectory_point;
      ss.str("");
      ss.clear();
      ss << str;
      ss >> trajectory_point.position.x >> trajectory_point.position.y >>
          trajectory_point.theta >> trajectory_point.velocity;

      if (one_trajectory.points.empty()) {
        trajectory_point.s = 0.0;
        one_trajectory.points.push_back(trajectory_point);
      } else {
        //轨迹出现了转折
        if (one_trajectory.points.back().velocity * trajectory_point.velocity <
            0.0) {
          //先把前一段轨迹存进list里
          common_trajectory_.push_back(one_trajectory);
          //清空one_trajectory
          one_trajectory.points.clear();
          //把这个转折处的点存入one_trajectory
          trajectory_point.s = 0.0;
          one_trajectory.points.push_back(trajectory_point);
        } else {  //轨迹没有出现转折
          trajectory_point.s =
              one_trajectory.points.back().s +
              std::hypot(trajectory_point.position.x -
                             one_trajectory.points.back().position.x,
                         trajectory_point.position.y -
                             one_trajectory.points.back().position.y);
          one_trajectory.points.push_back(trajectory_point);
        }
      }

      // if (common_trajectory_.points.empty())
      //   trajectory_point.s = 0.0;
      // else {
      //   trajectory_point.s =
      //       common_trajectory_.points.back().s +
      //       std::hypot(trajectory_point.position.x -
      //                      common_trajectory_.points.back().position.x,
      //                  trajectory_point.position.y -
      //                      common_trajectory_.points.back().position.y);
      // }
      // common_trajectory_.points.push_back(trajectory_point);
    }
  }
  common_trajectory_.push_back(one_trajectory);
  // common_trajectory_.mode = false;
  fin.close();

  AINFO << "Initialize trajectory: " << common_trajectory_path;
  AINFO << "trajectories nums : " << common_trajectory_.size();
}

void PathPub::SetTrajectory(
    std::list<cyber_msgs::LocalTrajList> *trajectories_in) {
  // if (trajectories_in->empty())
  //   return;
  // if (trajectories_in->size() == 1)
  // {
  //   std_msgs::Bool msg;
  //   msg.data = true;
  //   pub_all_trajectories_done_.publish(msg);
  // }
  // std::cout << "7" << std::endl;
  // now_trajectory_.trajectory = &(trajectories_in->front());
  // std::cout << "8" << std::endl;
  // now_trajectory_.mode = false;
  // now_trajectory_.nearest_index = 0;
  // std::cout << "9" << std::endl;
  // now_trajectory_.stop_index = trajectories_in->front().points.size() - 1;
  // std::cout << "10" << std::endl;
  // trajectories_in->pop_front();

  // std::cout << "11" << std::endl;
  // pub_trajectory_.publish(*now_trajectory_.trajectory);
  // std::cout << "12" << std::endl;
  // std::list<cyber_msgs::LocalTrajList> trajectories;
  // trajectories.push_back(*(now_trajectory_.trajectory));
  // // trajectories.emplace_back(*now_trajectory_.trajectory);
  // std::cout << "13" << std::endl;
  // visualization::PublishGlobalTrajectory(pub_rviz_global_path_,
  // trajectories); AINFO << "Trajectory published! Trajectory size:"
  //       << now_trajectory_.stop_index + 1;
  if (common_trajectory_.empty()) return;
  if (common_trajectory_.size() == 1) {
    std_msgs::Bool msg;
    msg.data = true;
    pub_all_trajectories_done_.publish(msg);
  }
  now_trajectory_.trajectory = &(common_trajectory_.front());
  now_trajectory_.mode = false;
  now_trajectory_.nearest_index = 0;
  now_trajectory_.stop_index = common_trajectory_.front().points.size() - 1;

  pub_trajectory_.publish(*now_trajectory_.trajectory);
  std::list<cyber_msgs::LocalTrajList> trajectories;
  trajectories.push_back(*(now_trajectory_.trajectory));
  // trajectories.emplace_back(*now_trajectory_.trajectory);
  visualization::PublishGlobalTrajectory(pub_rviz_global_path_, trajectories);
  common_trajectory_.pop_front();
  AINFO << "Trajectory published! Trajectory size:"
        << now_trajectory_.stop_index + 1;
}

void PathPub::ResetTrajectory() {
  now_trajectory_.trajectory = nullptr;
  now_trajectory_.nearest_index = 0;
  now_trajectory_.stop_index = std::numeric_limits<std::size_t>::max();

  cyber_msgs::LocalTrajList empty_trajectory;
  pub_trajectory_.publish(empty_trajectory);
  AWARN << "Publish empty trajectory!";
}

void PathPub::StartCallback(const std_msgs::Int8ConstPtr &start_in) {
  bplan_ = false;
  state_ = start_in->data;
  switch (state_) {
    case 100:  // 100
      InitCommonTrajectory(package_path_ + "path_demo1.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 101:  // 101
      InitCommonTrajectory(package_path_ + "path_demo2.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 102:  // 102
      InitCommonTrajectory(package_path_ + "path_demo3.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 103:  // 103
      InitCommonTrajectory(package_path_ + "path_demo4.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 104:  // 104
      InitCommonTrajectory(package_path_ + "path_demo5.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 105:  // 105
      InitCommonTrajectory(package_path_ + "path_demo6.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 106:  // 106
      InitCommonTrajectory(package_path_ + "path_demo7.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 107:  // 107
      InitCommonTrajectory(package_path_ + "path_demo8.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 108:  // 108
      InitCommonTrajectory(package_path_ + "path_demo9.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 109:  // 109
      InitCommonTrajectory(package_path_ + "path_demo10.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 110:  // 110
      InitCommonTrajectory(package_path_ + "path_demo11.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 111:  // 111
      InitCommonTrajectory(package_path_ + "path_demo12.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 112:  // 06.29.0
      InitCommonTrajectory(package_path_ + "path_demo290.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 113:  // 06.29.1
      InitCommonTrajectory(package_path_ + "path_demo291.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 114:  // 06.29.2
      InitCommonTrajectory(package_path_ + "path_demo292.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 115:  // 07.02.1,学校(J)
      InitCommonTrajectory(package_path_ + "path_demo721.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 116:  // 07.02.2,学校(U)
      InitCommonTrajectory(package_path_ + "path_demo722.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 117:  // 07.02.3
      InitCommonTrajectory(package_path_ + "path_demo723.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 118:  // 07.02.4
      InitCommonTrajectory(package_path_ + "path_demo724.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 119:  // 07.03.1
      InitCommonTrajectory(package_path_ + "path_demo731.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 120:  // 07.03.2,学校(S)
      InitCommonTrajectory(package_path_ + "path_demo732.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 80:  // yi型
      InitCommonTrajectory(package_path_ + "path_demo_yi.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 81:  // S型
      InitCommonTrajectory(package_path_ + "path_demo_S.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 82:  // L型
      InitCommonTrajectory(package_path_ + "path_demo_L.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 83:  // O型
      InitCommonTrajectory(package_path_ + "path_demo_O.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 84:  // T
      InitCommonTrajectory(package_path_ + "path_demo_T.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 85:  // T2,学校(T)
      InitCommonTrajectory(package_path_ + "path_demo_T2.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 60:  // T,学校(T),两段路,220927
      InitCommonTrajectory(package_path_ + "path_demo_2_T.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 61:  // path_demo_bk_3_duan,220927
      InitCommonTrajectory(package_path_ + "path_demo_bk_3_duan.txt");
      SetTrajectory(&common_trajectory_);
      break;
    case 62:  // path_demo_bk_3d,220927,2段倒车一段前进
      InitCommonTrajectory(package_path_ + "path_demo_bk_3d.txt");
      SetTrajectory(&common_trajectory_);
      break;
    default:
      state_ = -1;
      AERROR << "ERROR message from topic:/parking/start!";
      return;
  }
}

void PathPub::OneTrajectoryFinishedCallback(const std_msgs::Int8ConstPtr &msg) {
  if (msg->data == 1) {
    SetTrajectory(&common_trajectory_);
    AINFO << "Trajectory is set!!!";
  }

  AINFO << "one trajectory finished : " << msg->data;
}

void PathPub::LocalizationCallback(
    const cyber_msgs::LocalizationEstimateConstPtr &localization_in) {
  localization_flag_ = true;
  vehicle_pose_ = localization_in->pose;

  visualization::ShowVehiclePose(pub_rviz_pose_, localization_in);

  //获取当前牵引车的后轴中心位姿
  geometry_msgs::PoseStamped tractor_real_pose;
  tractor_real_pose.header.frame_id = "world";
  tractor_real_pose.header.stamp = ros::Time::now();
  tractor_real_pose.pose = localization_in->pose;
  if (tractor_path_.poses.size() != 0) {
    if (fabs(localization_in->pose.position.x -
             tractor_path_.poses.back().pose.position.x) > 0.7 ||
        fabs(localization_in->pose.position.y -
             tractor_path_.poses.back().pose.position.y) > 0.7 ||
        tractor_path_.poses.size() > 30000) {
      tractor_path_.poses.clear();
      return;
    }
  }
  if (show_real_path_) {
    tractor_path_.poses.emplace_back(tractor_real_pose);
    visualization::PublishRealPath(pub_rviz_tractor_real_path_, tractor_path_,
                                   "red");
  }
}

void PathPub::MotionCallback(
    const cyber_msgs::LocalizationEstimateConstPtr &localization_in,
    const cyber_msgs::LinkAngleConstPtr &link_angle_in) {
  //可视化当前的整个车体状态
  visualization::ShowVehiclePose(pub_rviz_pose_, localization_in,
                                 link_angle_in);

  //获取当前拖挂车和牵引车的后轴中心位姿
  geometry_msgs::PoseStamped tractor_real_pose;
  geometry_msgs::PoseStamped trailer_real_pose;

  tractor_real_pose.header.frame_id = "world";
  tractor_real_pose.header.stamp = ros::Time::now();
  tractor_real_pose.pose = localization_in->pose;

  trailer_real_pose.header.frame_id = "world";
  trailer_real_pose.header.stamp = ros::Time::now();

  double theta1 = tf::getYaw(localization_in->pose.orientation);
  double theta2 =
      NormalizeAngle(theta1 - link_angle_in->link_angle_degree * M_PI / 180.0);
  double link_point_x =
      localization_in->pose.position.x - vehicle_param::M1 * cos(theta1);
  double link_point_y =
      localization_in->pose.position.y - vehicle_param::M1 * sin(theta1);

  auto tf_q = tf::createQuaternionFromYaw(theta2);

  trailer_real_pose.pose.orientation.x = tf_q.getX();
  trailer_real_pose.pose.orientation.y = tf_q.getY();
  trailer_real_pose.pose.orientation.z = tf_q.getZ();
  trailer_real_pose.pose.orientation.w = tf_q.getW();
  trailer_real_pose.pose.position.x =
      link_point_x - vehicle_param::L2 * cos(theta2);
  trailer_real_pose.pose.position.y =
      link_point_y - vehicle_param::L2 * sin(theta2);
  trailer_real_pose.pose.position.z = 0.0;

  //更新tractor_path_和trailer_path_
  tractor_path_.header.stamp = ros::Time::now();
  tractor_path_.header.frame_id = "world";
  trailer_path_.header.stamp = ros::Time::now();
  trailer_path_.header.frame_id = "world";
  if (tractor_path_.poses.size() != 0) {
    if (fabs(localization_in->pose.position.x -
             tractor_path_.poses.back().pose.position.x) > 0.7 ||
        fabs(localization_in->pose.position.y -
             tractor_path_.poses.back().pose.position.y) > 0.7 ||
        tractor_path_.poses.size() > 30000) {
      tractor_path_.poses.clear();
      trailer_path_.poses.clear();
      return;
    }
  }

  if (show_real_path_) {
    tractor_path_.poses.emplace_back(tractor_real_pose);
    trailer_path_.poses.emplace_back(trailer_real_pose);

    geometry_msgs::Point trailer_real_point;
    trailer_real_point = trailer_real_pose.pose.position;

    pub_trailer_real_pose_.publish(trailer_real_point);

    visualization::PublishRealPath(pub_rviz_tractor_real_path_, tractor_path_,
                                   "red");
    visualization::PublishRealPath(pub_rviz_trailer_real_path_, trailer_path_,
                                   "yellow");
  }
}

void PathPub::ResetRealPathCallback(const std_msgs::BoolConstPtr &msg) {
  if (msg->data == true) {
    tractor_path_.poses.clear();
    trailer_path_.poses.clear();
  }
}

double PathPub::NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

}  // namespace path_pub

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_pub");

  // google::InitGoogleLogging("path_pub");
  // FLAGS_stderrthreshold = google::INFO;
  // FLAGS_colorlogtostderr = true;
  // FLAGS_log_prefix = true;
  // FLAGS_logbufsecs = 0;
  // FLAGS_max_log_size = 1024;
  // FLAGS_log_dir = expand_catkin_ws("/log/");
  // static_cast<std::string>(getenv("HOME")) + "/Projects/IVFC2020/log/";

  ros::NodeHandle nh("~");
  AINFO << "Path pub version : 1.10.4";
  path_pub::PathPub path_pub(&nh);
  ros::spin();

  return 0;
}