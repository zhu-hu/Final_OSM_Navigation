//
// Created by luyifan on 18-8-20.
//

#include "create_map/CreateMap.h"

#include <dstar_map/RoadCheckGoal.h>

#include <chrono>

#include "math_utils.h"

CreateMap::CreateMap(ros::NodeHandle *nh, tf::TransformListener *listener,
                     tf::TransformBroadcaster *broadcaster)
    : nh_(nh), listener_(listener), broadcaster_(broadcaster) {
  task_id_ = 0;
  global_map_ = new cv::Mat();
  kernelDilateL_ =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10));
  T_update_ = 5;
  bMapInit_ = false;
  bTFInit_ = false;
  bDStarInit_ = false;
  bWaitInit_ = false;

  PubMapPair pub_map;
  pub_map.first = "local_map";
  pub_map.second = nh_->advertise<nav_msgs::OccupancyGrid>(
      "local_map", 1); ///发布local map到rviz
  pub_occ_map_.emplace_back(pub_map);
  pub_map.first = "global_map";
  pub_map.second = nh_->advertise<nav_msgs::OccupancyGrid>(
      "global_map", 1); ///发布global map到rviz
  pub_occ_map_.emplace_back(pub_map);
  client_task_request_ =
      nh_->serviceClient<dstar_map::MapUpdate>("task_service");
  task_goal_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("task_goal", 1);
  cp_.pose.position.x = std::numeric_limits<double>::infinity();
  cp_.pose.position.y = std::numeric_limits<double>::infinity();
  cp_.pose.position.z = std::numeric_limits<double>::infinity();

  goal_.x = std::numeric_limits<double>::infinity();
  goal_.y = std::numeric_limits<double>::infinity();

  ///回调函数多线程 1
  nh_->setCallbackQueue(&queue_1_);
  sub_local_map_ = nh_->subscribe("/perception/local_grid_map/compressed", 1,
                                  &CreateMap::UpdateMap, this);
  sub_task_rviz_ = nh_->subscribe("/move_base_simple/goal", 1,
                                  &CreateMap::SetTaskRviz, this);
  sub_state_machine_task_ = nh_->subscribe(
      "/DSMap/path_task", 10, &CreateMap::SetTaskStateMachine, this);
  sub_cp_ = nh_->subscribe("/localization/estimation", 1,
                           &CreateMap::CpCallback, this);
  ///线程2: 发布transfrom world->global_map
  nh_->setCallbackQueue(&queue_2_);
  timer1_ =
      nh->createTimer(ros::Duration(0.02), &CreateMap::Timer1Callback, this);

  spinner_1_ = new ros::AsyncSpinner(1, &queue_1_);
  spinner_2_ = new ros::AsyncSpinner(1, &queue_2_);

  spinner_1_->start();
  spinner_2_->start();
}

CreateMap::~CreateMap() {}

bool CreateMap::TaskGoalCallback(dstar_map::RoadCheckGoal::Request &req,
                                 dstar_map::RoadCheckGoal::Response &res) {
  ROS_INFO("Get task goal request!!!");

  res.goal.pose.position.x = goal_.x;
  res.goal.pose.position.y = goal_.y;
  res.goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_.theta);

  return true;
}

void CreateMap::CpCallback(const cyber_msgs::LocalizationEstimate &pose) {
  cp_.pose = pose.pose;
  // if got task before cp
  if (bWaitInit_) {
    //        std::cout << "What the hell is here" << std::endl;
    start_.x = cp_.pose.position.x;
    start_.y = cp_.pose.position.y;
    start_.theta = tf::getYaw(cp_.pose.orientation);
    InitMap(start_, goal_, local_map_);
    bWaitInit_ = false;
  }
}

void CreateMap::Timer1Callback(const ros::TimerEvent &) {
  if (!bMapInit_)
    return;
  tf::Transform transform;
  tf::Quaternion q;
  {
    read_lock glock(global_map_mutex_);
    transform.setOrigin(tf::Vector3(global_map_origin_.position.x,
                                    global_map_origin_.position.y, 0.0));
    q.setX(global_map_origin_.orientation.x);
    q.setY(global_map_origin_.orientation.y);
    q.setZ(global_map_origin_.orientation.z);
    q.setW(global_map_origin_.orientation.w);
  }
  transform.setRotation(q);
  broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                   "/world", "/global_map"));
  bTFInit_ = true;
}

void CreateMap::SetTaskStateMachine(
    const geometry_msgs::PoseStampedConstPtr &goal) {
  ///约定 9 表示停止建图 --> 停止路径规划
  if (goal->pose.position.z == 9) {
    DeleteMap();
    return;
  }

  if (local_map_.cols == 0 || local_map_.rows == 0) {
    ROS_ERROR("NO LOCAL MAP, INIT FAILED!!");
    return;
  }

  if (goal_.x == goal->pose.position.x && goal_.y == goal->pose.position.y)
    return;

  goal_.x = goal->pose.position.x;
  goal_.y = goal->pose.position.y;
  goal_.theta = tf::getYaw(goal->pose.orientation);

  std::cout << "StateMachine goal set at (" << goal_.x << "," << goal_.y << ")"
            << std::endl;
  if (cp_.pose.position.x == std::numeric_limits<double>::infinity()) {
    bWaitInit_ = true;
    return;
  }

  start_.x = cp_.pose.position.x;
  start_.y = cp_.pose.position.y;
  start_.theta = tf::getYaw(cp_.pose.orientation);
  InitMap(start_, goal_, local_map_);
}

void CreateMap::SetTaskRviz(const geometry_msgs::PoseStampedConstPtr &click) {
  if (local_map_.cols == 0 || local_map_.rows == 0) {
    ROS_ERROR("NO LOCAL MAP, INIT FAILED!!");
    return;
  }

  goal_.x = click->pose.position.x;
  goal_.y = click->pose.position.y;
  goal_.theta = tf::getYaw(click->pose.orientation);

  std::cout << "Rviz goal location set at (" << goal_.x << "," << goal_.y << ")"
            << std::endl;
  std::cout << "Rviz goal location set at (" << goal_.x << "," << goal_.y << ")"
            << std::endl;
  std::cout << "Rviz goal location set at (" << goal_.x << "," << goal_.y << ")"
            << std::endl;
  std::cout << "Rviz goal location set at (" << goal_.x << "," << goal_.y << ")"
            << std::endl;
  std::cout << "Rviz goal location set at (" << goal_.x << "," << goal_.y << ")"
            << std::endl;
  std::cout << "Rviz goal location set at (" << goal_.x << "," << goal_.y << ")"
            << std::endl;
  std::cout << "Rviz goal location set at (" << goal_.x << "," << goal_.y << ")"
            << std::endl;

  if (cp_.pose.position.x == std::numeric_limits<double>::infinity()) {
    bWaitInit_ = true;
    return;
  }

  start_.x = cp_.pose.position.x;
  start_.y = cp_.pose.position.y;
  start_.theta = tf::getYaw(cp_.pose.orientation);

  InitMap(start_, goal_, local_map_);
}

void CreateMap::InitMap(dstar_map::WayPoint start, dstar_map::WayPoint goal,
                        cv::Mat &map_in) {
  if (bMapInit_)
    DeleteMap();
  geometry_msgs::PoseStamped current_goal;
  current_goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal.theta);
  current_goal.pose.position.x = goal.x;
  current_goal.pose.position.y = goal.y;
  current_goal.header.frame_id = "world";
  task_goal_pub_.publish(current_goal);

  ///初始化地图大小
  const double WORLDDETX = 50;
  const double WORLDDETY = 50;

  global_map_origin_.position.x =
      (start.x < goal.x ? start.x : goal.x) - WORLDDETX;
  global_map_origin_.position.y =
      (start.y < goal.y ? start.y : goal.y) - WORLDDETY;
  global_map_origin_.orientation = tf::createQuaternionMsgFromYaw(0.0);
  int widthW = (int)((2 * WORLDDETX + fabs(goal.x - start.x)) *
                     map_param::grid_map::kCellFactor) +
               1;
  int lengthW = (int)((2 * WORLDDETY + fabs(goal.y - start.y)) *
                      map_param::grid_map::kCellFactor) +
                1;
  int length = std::max(widthW, lengthW);
  cv::Mat global_map = cv::Mat::zeros(
      length, length,
      CV_8UC1); ///初始化一个大的栅格图,使得start和goal包含在栅格图内
  std::cout << "Init a new global map." << std::endl;

  *global_map_ = global_map;
  int goal_x = (int)((goal.x - global_map_origin_.position.x) *
                     map_param::grid_map::kCellFactor);
  int goal_y = (int)(length - ((goal.y - global_map_origin_.position.y) *
                               map_param::grid_map::kCellFactor));
  std::cout << "goal x: " << goal_x << " goal y: " << goal_y << std::endl;
  bMapInit_ = true;
  ros::Duration(1.0)
      .sleep(); ///等待TimerCallback发布global_map到base_link的变换

  dstar_map::MapUpdate task_srv;
  task_srv.request.init = 1; ///表示dstar那边需要重新初始化一张global map
  task_srv.request.goal_x = goal_x;
  task_srv.request.goal_y = goal_y;
  task_srv.request.goal_theta =
      goal_.theta; // this theta is in world coordinate
  task_srv.request.map_length = length;

  cv::Mat local_map;
  cv::dilate(map_in, local_map, kernelDilateL_);

  ///写入地图数据
  double ego_theta = tf::getYaw(cp_.pose.orientation);
  for (int i = 0; i < local_map.rows; i++) {
    const uchar *p = local_map.ptr<uchar>(i);
    for (int j = 0; j < local_map.cols; j++) { ///珊格图太长了，建图范围60-120米
      unsigned char cost = *(p + j);
      if (cost == 128) ///未知,不做任何操作
        continue;
      cost = (cost < 255) ? 0 : 255;

      ///计算该点在GlobalMap中的像素位置
      double x = (map_param::grid_map::kHeight -
                  map_param::grid_map::kCarCenterY - i - 0.5) *
                 map_param::grid_map::kCellSize;
      double y = (map_param::grid_map::kCarCenterX - j - 0.5) *
                 map_param::grid_map::kCellSize;
      double xg = cp_.pose.position.x - global_map_origin_.position.x +
                  x * cos(ego_theta) - y * sin(ego_theta);
      double yg = cp_.pose.position.y - global_map_origin_.position.y +
                  y * cos(ego_theta) + x * sin(ego_theta);
      auto col = (int)(xg * map_param::grid_map::kCellFactor);
      auto row = (int)(length - yg * map_param::grid_map::kCellFactor);

      if (col < 0 || col > global_map_->cols - 1 || row < 0 ||
          row > global_map_->rows - 1)
        continue; ///越界Global Map

      ///更新地图
      global_map_->at<uchar>(row, col) = cost;
      dstar_map::CellUpdate cell_update;
      cell_update.x = col;
      cell_update.y = row;
      cell_update.index = col + row * global_map_->cols;
      ;
      cell_update.cost = cost;
      task_srv.request.data.updates.emplace_back(cell_update);
    }
  }

  PubOccMap(global_map_, "global_map");

  if (task_srv.request.data.updates.empty()) {
    ROS_ERROR("No Local Map or global map wrong!");
    return;
  }

  if (client_task_request_.call(task_srv)) {
    ROS_INFO("Init succeed!");
  } else
    ROS_ERROR("Cannot request dstar!");
  bDStarInit_ = true;
}

void CreateMap::UpdateMap(const sensor_msgs::CompressedImageConstPtr &map_in) {
  {
    write_lock llock(local_map_mutex_);
    local_map_ = cv::imdecode(cv::Mat(map_in->data), CV_LOAD_IMAGE_UNCHANGED);
  }
  {
    read_lock llock(local_map_mutex_);
    PubOccMap(&local_map_, "local_map"); ///和visualization模块不在一台电脑上
  }
  if (!bTFInit_ || !bMapInit_ || !bDStarInit_)
    return;
  ///每收到5帧local_map才更新一次global_map

  T_update_++;

  if (T_update_ < 3)
    return;
  else
    T_update_ = 0;
  cv::Mat local_map;

  {
    read_lock llock(local_map_mutex_);
    cv::dilate(local_map_, local_map, kernelDilateL_);
    // local_map = local_map_;
  }

  dstar_map::MapUpdate task_srv;
  task_srv.request.init = 0;
  //  cv::imshow("dilate", local_map);
  //  cv::waitKey(1);

  auto start = std::chrono::system_clock::now();

  double ego_theta = tf::getYaw(cp_.pose.orientation);
  for (int i = local_map.rows / 2; i < local_map.rows; i++) {
    const uchar *p = local_map.ptr<uchar>(i);
    for (int j = 0; j < local_map.cols; j++) {
      unsigned char cost = *(p + j);
      if (cost == 128) ///未知
        continue;
      cost = (cost < 255) ? 0 : 255;

      ///计算该点在GlobalMap中的像素位置
      double x = (map_param::grid_map::kHeight -
                  map_param::grid_map::kCarCenterY - i - 0.5) *
                 map_param::grid_map::kCellSize;
      double y = (map_param::grid_map::kCarCenterX - j - 0.5) *
                 map_param::grid_map::kCellSize;
      double xg = cp_.pose.position.x - global_map_origin_.position.x +
                  x * cos(ego_theta) - y * sin(ego_theta);
      double yg = cp_.pose.position.y - global_map_origin_.position.y +
                  y * cos(ego_theta) + x * sin(ego_theta);
      auto col = (int)(xg * map_param::grid_map::kCellFactor);
      auto row =
          (int)(global_map_->rows - yg * map_param::grid_map::kCellFactor);

      if (col < 0 || col > global_map_->cols - 1 || row < 0 ||
          row > global_map_->rows - 1)
        continue; ///越界
      ///更新地图
      {
        write_lock glock(global_map_mutex_);
        global_map_->at<uchar>(row, col) = cost;
      }
      int index = col + row * global_map_->cols;
      dstar_map::CellUpdate cell_update;
      cell_update.x = col;
      cell_update.y = row;
      cell_update.index = index;
      cell_update.cost = cost;
      task_srv.request.data.updates.emplace_back(cell_update);
    }
  }
  auto end = std::chrono::system_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  double time_us = double(duration.count());
  // std::cout << "Loading map costs " << time_us << " us."<<std::endl;
  if (client_task_request_.call(task_srv)) {
    // ROS_INFO("Launch a request to generate path");
  } else {
    ROS_ERROR("Failed to connect to dstar!");
  }

  {
    read_lock glock(global_map_mutex_);
    PubOccMap(global_map_, "global_map");
  }

  // ROS_INFO("Updating...");
}

void CreateMap::DeleteMap() {
  std::cout << "Delete previous global map." << std::endl;
  bMapInit_ = false;
  bTFInit_ = false;
  bDStarInit_ = false;
  ros::Duration(1.0).sleep(); //等待其他线程里对global map的操作执行结束

  free(global_map_);
  global_map_ = new cv::Mat();

  goal_.x = std::numeric_limits<double>::infinity();
  goal_.y = std::numeric_limits<double>::infinity();
}

void CreateMap::PubOccMap(const cv::Mat *map_in, std::string frame_id) {
  {
    if (map_in->rows == 0 || map_in->cols == 0) {
      ROS_ERROR("NO MAP IN!!");
      return;
    }

    int newHeight = map_in->rows;
    int newWidth = map_in->cols;

    nav_msgs::OccupancyGrid::Ptr grid;
    grid.reset(new nav_msgs::OccupancyGrid);
    grid->header.frame_id = frame_id;
    grid->info.height = newHeight;
    grid->info.width = newWidth;
    grid->info.resolution = kCellSize;
    grid->header.stamp = ros::Time::now();

    for (int row = map_in->rows - 1; row >= 0; row--) {
      for (int col = 0; col < map_in->cols; col++) {
        if (map_in->at<uchar>(row, col) == 255)
          grid->data.emplace_back(100); ///障碍物
        else if (map_in->at<uchar>(row, col) == 0)
          grid->data.emplace_back(0); ///无障碍物
        else
          grid->data.emplace_back(50); ///未知
      }
    }

    for (size_t i = 0; i < pub_occ_map_.size(); i++) {
      if (pub_occ_map_[i].first == frame_id) {
        pub_occ_map_[i].second.publish(*grid);
        break;
      }
    }
  }
}
