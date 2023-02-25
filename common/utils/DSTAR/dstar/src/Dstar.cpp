#include "Dstar/Dstar.h"

using namespace std;

Dstar::Dstar(ros::NodeHandle *nh, int obs_thresh, double runtime,
             ros::Publisher *pub_path, ros::Publisher *pub_target)
    : nh_(nh), obs_thresh_(obs_thresh),
      allocated_time_secs_foreachplan_(runtime), pub_path_(pub_path),
      pub_target_(pub_target), map_(nullptr) {
  listener_ = new tf::TransformListener();
  goal_theta_ = std::numeric_limits<double>::infinity();
  ros::Duration(1).sleep();
  ROS_INFO("DStar working...");
}

void Dstar::InitMap(geometry_msgs::PoseStamped current_pose, int goal_x,
                    int goal_y, float goal_theta, int length,
                    dstar_map::CellUpdateList update_list) {
  DeletePreTask();
  width_ = length;
  height_ = length;
  goal_theta_ = goal_theta;
  goal_quaternion_ = tf::createQuaternionMsgFromYaw(goal_theta);

  int start_x, start_y;
  geometry_msgs::PointStamped current_pose_w, current_pose_g;
  current_pose_w.point.x = current_pose.pose.position.x;
  current_pose_w.point.y = current_pose.pose.position.y;
  TransformPoint(listener_, "world", "global_map", current_pose_w,
                 current_pose_g);
  // TODO: x=0; y=0

  TFXY2PixInGlobalMap(current_pose_g.point.x, current_pose_g.point.y, start_x,
                      start_y, length);

  // TODO:初始化地图
  map_ =
      (unsigned char *)calloc(size_t(length * length), sizeof(unsigned char));
  for (auto cell : update_list.updates) {
    map_[cell.index] = cell.cost;
  }

  if (!environmentNav2D_.SetEnvParameter("is16connected", 0)) {
    ROS_ERROR("ERROR: failed to set parameters");
  }
  std::cout << "width_=height_=" << length << "; start_x: " << start_x
            << "; start_y: " << start_y << "; goal_x: " << goal_x
            << "; goal_y: " << goal_y << "; obs_thresh: " << obs_thresh_
            << std::endl;
  if (!environmentNav2D_.InitializeEnv(width_, height_, map_, start_x, start_y,
                                       goal_x, goal_y, obs_thresh_)) {
    ROS_ERROR("ERROR: InitializeEnv failed");
  }
  // Initialize MDP Info
  if (!environmentNav2D_.InitializeMDPCfg(&mDPCfg_)) {
    ROS_ERROR("ERROR: InitializeMDPCfg failed");
  }

  ///初始化路径规划期
  bool bforwardsearch = true;
  planner_ = new ADPlanner(&environmentNav2D_, bforwardsearch);
  planner_->set_initialsolution_eps(2.0);
  ///设置搜索模式
  planner_->set_search_mode(false);
  ///设置起点终点
  if (planner_->set_start(mDPCfg_.startstateid) == 0) {
    ROS_ERROR("ERROR: failed to set start state");
  }
  if (planner_->set_goal(mDPCfg_.goalstateid) == 0) {
    ROS_ERROR("ERROR: failed to set goal state");
  }
}

void Dstar::UpdateMap(dstar_map::CellUpdateList update_list) {
  std::vector<nav2dcell_t> changedcellsV;
  std::vector<int> preds_of_changededgesIDV;
  geometry_msgs::PointStamped local_map_pt, global_map_pt;

  for (auto cell : update_list.updates) {
    write_lock(read_write_mutex_);
    map_[cell.index] = cell.cost;
    environmentNav2D_.UpdateCost(cell.x, cell.y, map_[cell.index]);
    /// store the changed cells
    nav2dcell_t nav2dcell;
    nav2dcell.x = cell.x;
    nav2dcell.y = cell.y;
    changedcellsV.push_back(nav2dcell);
  }
  // get the affected states
  environmentNav2D_.GetPredsofChangedEdges(&changedcellsV,
                                           &preds_of_changededgesIDV);
  // let know the incremental planner about them
  ((ADPlanner *)planner_)
      ->update_preds_of_changededges(&preds_of_changededgesIDV);
  std::cout << "RePlanning!" << std::endl;
}

bool Dstar::RePlan(geometry_msgs::PoseStamped current_pose,
                   cyber_msgs::LocalTrajList *path) {
  bool bPlanExists;

  // struct timeval start;
  // struct timeval end;

  // gettimeofday(&start,NULL);
  ///设置起点
  int start_x, start_y;
  geometry_msgs::PointStamped current_pose_w, current_pose_g;
  current_pose_w.point.x = current_pose.pose.position.x;
  current_pose_w.point.y = current_pose.pose.position.y;
  TransformPoint(listener_, "world", "global_map", current_pose_w,
                 current_pose_g);

  TFXY2PixInGlobalMap(current_pose_g.point.x, current_pose_g.point.y, start_x,
                      start_y, height_);
  environmentNav2D_.SetStart(start_x, start_y);
  if (!environmentNav2D_.InitializeMDPCfg(&mDPCfg_)) {
    ROS_ERROR("ERROR: InitializeMDPCfg failed");
  }
  if (planner_->set_start(mDPCfg_.startstateid) == 0) {
    ROS_ERROR("ERROR: failed to set start state");
  }

  ///规划路径
  std::vector<int> solution_stateIDs_V;
  bPlanExists = (planner_->replan(allocated_time_secs_foreachplan_,
                                  &solution_stateIDs_V) == 1);
  path->points.clear();
  geometry_msgs::PointStamped pt_g, pt_w;
  if (bPlanExists) {
    for (size_t i = 0; i < solution_stateIDs_V.size(); i++) {
      /// x, y是Global Map下的像素值
      int pix_x, pix_y;
      environmentNav2D_.GetCoordFromState(solution_stateIDs_V[i], pix_x, pix_y);
      TFPix2XYInGlobalMap(pt_g.point.x, pt_g.point.y, pix_x, pix_y, height_);
      TransformPoint(listener_, "global_map", "world", pt_g, pt_w);

      cyber_msgs::LocalTrajPoint ltpt;
      ltpt.position.x = pt_w.point.x;
      ltpt.position.y = pt_w.point.y;
      path->points.emplace_back(ltpt);
    }
  }
  // gettimeofday(&end,NULL);
  // std::cout << 1000000 * (end.tv_sec - start.tv_sec) + end.tv_usec -
  // start.tv_usec << std::endl;

  return bPlanExists;
}

bool Dstar::RePlan(geometry_msgs::PoseStamped current_pose,
                   cyber_msgs::LocalTrajList *path, const double time) {
  bool bPlanExists;

  struct timeval start;
  struct timeval end;

  gettimeofday(&start, NULL);

  ///设置起点
  int start_x, start_y;
  geometry_msgs::PointStamped current_pose_w, current_pose_g;
  current_pose_w.point.x = current_pose.pose.position.x;
  current_pose_w.point.y = current_pose.pose.position.y;
  TransformPoint(listener_, "world", "global_map", current_pose_w,
                 current_pose_g);
  TFXY2PixInGlobalMap(current_pose_g.point.x, current_pose_g.point.y, start_x,
                      start_y, height_);
  environmentNav2D_.SetStart(start_x, start_y);
  if (!environmentNav2D_.InitializeMDPCfg(&mDPCfg_)) {
    ROS_ERROR("ERROR: InitializeMDPCfg failed");
  }
  if (planner_->set_start(mDPCfg_.startstateid) == 0) {
    ROS_ERROR("ERROR: failed to set start state");
  }

  ///规划路径
  std::vector<int> solution_stateIDs_V;
  bPlanExists = (planner_->replan(time, &solution_stateIDs_V) == 1);
  (*path).points.clear();
  int pix_x, pix_y;
  geometry_msgs::PointStamped pt_g, pt_w;
  if (bPlanExists) {
    for (size_t i = 0; i < solution_stateIDs_V.size(); i++) {
      /// x, y是Global Map下的像素值
      environmentNav2D_.GetCoordFromState(solution_stateIDs_V[i], pix_x, pix_y);
      TFPix2XYInGlobalMap(pt_g.point.x, pt_g.point.y, pix_x, pix_y, height_);
      TransformPoint(listener_, "global_map", "world", pt_g, pt_w);
      cyber_msgs::LocalTrajPoint ltpt;
      ltpt.position.x = pt_w.point.x;
      ltpt.position.y = pt_w.point.y;
      (*path).points.emplace_back(ltpt);
    }
  }

  if (bPlanExists) {
    std::cout << "solution found" << std::endl;
  } else {
    std::cout << "no solution" << std::endl;
  }

  gettimeofday(&end, NULL);
  std::cout << 1000000 * (end.tv_sec - start.tv_sec) + end.tv_usec -
                   start.tv_usec
            << std::endl;

  return bPlanExists;
}

void Dstar::PubPath(cyber_msgs::LocalTrajList *path) {
  if (path->points.size() < 10) {
    ROS_WARN("No Path!!");
    return;
  }
  nav_msgs::Path path_show;
  path_show.header.frame_id = "world";
  for (const auto &point : (*path).points) {
    geometry_msgs::PoseStamped pose_show;
    pose_show.header.frame_id = "world";
    pose_show.header.stamp = ros::Time::now();
    pose_show.pose.position = point.position;
    path_show.poses.emplace_back(pose_show);
  }
  pub_path_->publish(path_show);
}

void Dstar::PubLocalTarget(geometry_msgs::PoseStamped current_pose,
                           cyber_msgs::LocalTrajList *path) {

  if (path->points.size() < 3) {
    ROS_INFO("NO PATH!!!!");
    return;
  }

  // std::cout << "size: " << path->points.size() << std::endl;

  path->points.front().s = 0;
  double s = 0;
  for (int i = 1; i < path->points.size(); i++) {
    double x_diff = path->points[i - 1].position.x - path->points[i].position.x;
    double y_diff = path->points[i - 1].position.y - path->points[i].position.y;
    double distance = sqrt(x_diff * x_diff + y_diff * y_diff);
    s = s + distance;
    path->points[i].s = s;
  }

  size_t id = 0;
  for (auto pt : path->points) {
    geometry_msgs::PointStamped pt_w, pt_l;
    pt_w.header.frame_id = "world";
    pt_w.point.x = pt.position.x;
    pt_w.point.y = pt.position.y;
    TransformPoint(listener_, "world", "base_link", pt_w, pt_l);
    int pix_x, pix_y;
    TFXY2PixInCar(pt_l.point.x, pt_l.point.y, pix_x, pix_y);

    if (pix_x < map_param::dstar::local_map_margin ||
        pix_y < map_param::dstar::local_map_margin_top ||
        pix_x >
            map_param::grid_map::kWidth - map_param::dstar::local_map_margin ||
        pt.s > 35) {
      //            pix_y >
      //            map_param::grid_map::kHeight-map_param::dstar::local_map_margin_top
      //            ){
      geometry_msgs::PoseStamped local_target;
      local_target.header.frame_id = "world";
      local_target.header.stamp = ros::Time::now();
      local_target.pose.position.x = pt.position.x;
      local_target.pose.position.y = pt.position.y;

      if (id > 5) {
        double yaw = atan2(pt.position.y - path->points[id - 5].position.y,
                           pt.position.x - path->points[id - 5].position.x);
        local_target.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      } else {
        double yaw = atan2(pt.position.y - current_pose.pose.position.y,
                           pt.position.x - current_pose.pose.position.x);
        local_target.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      }

      //            std::cout << "pix_x: " << pix_x << ", pix_y: "<< pix_y <<
      //            std::endl;
      //            std::cout << "id: " << id << std::endl;
      std::cout << "Pub local target." << std::endl;
      pub_target_->publish(local_target);
      return;
    }
    id++;
  }
  //    ROS_INFO("Pub last point on global path as local target!");
  geometry_msgs::PoseStamped local_target;
  local_target.header.frame_id = "world";
  local_target.header.stamp = ros::Time::now();
  local_target.pose.position.x = path->points.back().position.x;
  local_target.pose.position.y = path->points.back().position.y;

  std::cout << "goal_theta: " << goal_theta_ << std::endl;
  if (goal_theta_ < std::numeric_limits<double>::infinity()) {
    local_target.pose.orientation = goal_quaternion_;
  } else {
    if (path->points.size() > 5) {
      double yaw = atan2(path->points.back().position.y -
                             path->points[path->points.size() - 5].position.y,
                         path->points.back().position.x -
                             path->points[path->points.size() - 5].position.x);
      local_target.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    } else {
      double yaw =
          atan2(path->points.back().position.y - current_pose.pose.position.y,
                path->points.back().position.x - current_pose.pose.position.x);
      local_target.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    }
  }
  std::cout << "Pub local target." << std::endl;
  pub_target_->publish(local_target);
}

void Dstar::DeletePreTask() {
  if (map_ != nullptr)
    free(map_);
}
