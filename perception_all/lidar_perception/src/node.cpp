/*
 * Copyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.02.20
 */

#include "node.h"

// USB Key，只在生成发布版的时候才解注释
// #include "../../common/Pwd_8/SoftkeyPWD.h"

Node::Node() : nh_("~") {
  // topic input
  nh_.param<std::string>("in_point_cloud_topic", in_point_cloud_topic_,
                         "/driver/pandar/point_cloud");
  nh_.param<std::string>("in_localization_topic", in_localization_topic_,
                         "/localization/estimation");

  // topic output
  nh_.param<std::string>("out_local_grid_map_topic", out_local_grid_map_topic_,
                         "/perception/local_grid_map");
  nh_.param<std::string>("out_global_objects_topic", out_global_objects_topic_,
                         "/perception/global_objects");

  // debug params
  nh_.param<bool>("publish_local_points", publish_local_points_, false);
  nh_.param<std::string>("out_local_points_topic", out_local_points_topic_,
                         "/perception/local_points");
  nh_.param<bool>("publish_global_points", publish_global_points_, false);
  nh_.param<std::string>("out_global_points_topic", out_global_points_topic_,
                         "/perception/global_points");

  // lidar_preprocess params
  nh_.param("lidar_preprocess/near_roi/near_noise_x_min",
            lp_params_.near_noise_x_min, lp_params_.near_noise_x_min);
  nh_.param("lidar_preprocess/near_roi/near_noise_x_max",
            lp_params_.near_noise_x_max, lp_params_.near_noise_x_max);
  nh_.param("lidar_preprocess/near_roi/near_noise_y_min",
            lp_params_.near_noise_y_min, lp_params_.near_noise_y_min);
  nh_.param("lidar_preprocess/near_roi/near_noise_y_max",
            lp_params_.near_noise_y_max, lp_params_.near_noise_y_max);
  nh_.param("lidar_preprocess/far_roi/livox_roi_x_min",
            lp_params_.livox_roi_x_min, lp_params_.livox_roi_x_min);
  nh_.param("lidar_preprocess/far_roi/livox_roi_x_max",
            lp_params_.livox_roi_x_max, lp_params_.livox_roi_x_max);
  nh_.param("lidar_preprocess/far_roi/livox_roi_y_min",
            lp_params_.livox_roi_y_min, lp_params_.livox_roi_y_min);
  nh_.param("lidar_preprocess/far_roi/livox_roi_y_max",
            lp_params_.livox_roi_y_max, lp_params_.livox_roi_y_max);
  nh_.param("lidar_preprocess/z_roi/livox_roi_z_min",
            lp_params_.livox_roi_z_min, lp_params_.livox_roi_z_min);
  nh_.param("lidar_preprocess/z_roi/livox_roi_z_max",
            lp_params_.livox_roi_z_max, lp_params_.livox_roi_z_max);
  nh_.param("lidar_preprocess/near_noise_filter/livox_noise_flag",
            lp_params_.livox_noise_flag, lp_params_.livox_noise_flag);
  nh_.param("lidar_preprocess/near_noise_filter/roi/livox_noise_x_min",
            lp_params_.livox_noise_x_min, lp_params_.livox_noise_x_min);
  nh_.param("lidar_preprocess/near_noise_filter/roi/livox_noise_x_max",
            lp_params_.livox_noise_x_max, lp_params_.livox_noise_x_max);
  nh_.param("lidar_preprocess/near_noise_filter/roi/livox_noise_y_min",
            lp_params_.livox_noise_y_min, lp_params_.livox_noise_y_min);
  nh_.param("lidar_preprocess/near_noise_filter/roi/livox_noise_y_max",
            lp_params_.livox_noise_y_max, lp_params_.livox_noise_y_max);
  nh_.param("lidar_preprocess/near_noise_filter/rate/livox_noise_rate",
            lp_params_.livox_noise_rate, lp_params_.livox_noise_rate);
  nh_.param("lidar_preprocess/near_noise_filter/min_direct_remove_intensity",
            lp_params_.min_direct_remove_intensity,
            lp_params_.min_direct_remove_intensity);
  nh_.param("lidar_preprocess/trailer_noise/trailer_x_min",
            lp_params_.trailer_x_min, lp_params_.trailer_x_min);
  nh_.param("lidar_preprocess/trailer_noise/trailer_y_min",
            lp_params_.trailer_y_min, lp_params_.trailer_y_min);
  nh_.param("lidar_preprocess/trailer_noise/trailer_y_max",
            lp_params_.trailer_y_max, lp_params_.trailer_y_max);
  nh_.param("lidar_preprocess/outlier_remove/min_outlier_intensity",
            lp_params_.min_outlier_intensity, lp_params_.min_outlier_intensity);
  nh_.param("lidar_preprocess/outlier_remove/search_radius",
            lp_params_.search_radius, lp_params_.search_radius);
  nh_.param("lidar_preprocess/outlier_remove/min_neighbors",
            lp_params_.min_neighbors, lp_params_.min_neighbors);
  nh_.param("lidar_preprocess/frame_merging/frame_merging_count",
            lp_params_.frame_merging_count, lp_params_.frame_merging_count);

  // ground segmentation params
  nh_.param("ground_segmentation/n_bins", lp_params_.gs_params.n_bins,
            lp_params_.gs_params.n_bins);
  nh_.param("ground_segmentation/n_segments", lp_params_.gs_params.n_segments,
            lp_params_.gs_params.n_segments);
  nh_.param("ground_segmentation/gamma_rate", lp_params_.gs_params.gamma_rate,
            lp_params_.gs_params.gamma_rate);
  nh_.param("ground_segmentation/prior_ground_z",
            lp_params_.gs_params.prior_ground_z,
            lp_params_.gs_params.prior_ground_z);
  nh_.param("ground_segmentation/max_initial_slope",
            lp_params_.gs_params.max_initial_slope,
            lp_params_.gs_params.max_initial_slope);
  nh_.param("ground_segmentation/max_slope", lp_params_.gs_params.max_slope,
            lp_params_.gs_params.max_slope);
  nh_.param("ground_segmentation/line_search_segment_num",
            lp_params_.gs_params.line_search_segment_num,
            lp_params_.gs_params.line_search_segment_num);
  nh_.param("ground_segmentation/max_dist_to_line",
            lp_params_.gs_params.max_dist_to_line,
            lp_params_.gs_params.max_dist_to_line);

  // object segmentation params
  nh_.param("object_segmentation/length_max", os_params_.length_max,
            os_params_.length_max);
  nh_.param("object_segmentation/width_max", os_params_.width_max,
            os_params_.width_max);
  nh_.param("object_segmentation/area_max", os_params_.area_max,
            os_params_.area_max);

  // grid map params
  nh_.param("grid_map/roi_map/min_x", gm_params_.roi_params.min_x,
            gm_params_.roi_params.min_x);
  nh_.param("grid_map/roi_map/max_x", gm_params_.roi_params.max_x,
            gm_params_.roi_params.max_x);
  nh_.param("grid_map/roi_map/min_y", gm_params_.roi_params.min_y,
            gm_params_.roi_params.min_y);
  nh_.param("grid_map/roi_map/max_y", gm_params_.roi_params.max_y,
            gm_params_.roi_params.max_y);
  nh_.param("grid_map/roi_map/pixel_scale", gm_params_.roi_params.pixel_scale,
            gm_params_.roi_params.pixel_scale);
  nh_.param("grid_map/person_detection/size_max",
            gm_params_.person_params.size_max,
            gm_params_.person_params.size_max);
  nh_.param("grid_map/person_detection/size_min",
            gm_params_.person_params.size_min,
            gm_params_.person_params.size_min);
  nh_.param("grid_map/person_detection/area_max",
            gm_params_.person_params.area_max,
            gm_params_.person_params.area_max);
  nh_.param("grid_map/person_detection/height_min",
            gm_params_.person_params.height_min,
            gm_params_.person_params.height_min);
  nh_.param("grid_map/cone_detection/size_max", gm_params_.cone_params.size_max,
            gm_params_.cone_params.size_max);
  nh_.param("grid_map/cone_detection/research_dis",
            gm_params_.cone_params.research_dis,
            gm_params_.cone_params.research_dis);
  nh_.param("grid_map/cone_detection/height_max",
            gm_params_.cone_params.height_max,
            gm_params_.cone_params.height_max);
  nh_.param("grid_map/cone_detection/sur_time", gm_params_.cone_params.sur_time,
            gm_params_.cone_params.sur_time);
  nh_.param("grid_map/cone_detection/cone_roi/cone_x_min",
            gm_params_.cone_params.cone_x_min,
            gm_params_.cone_params.cone_x_min);
  nh_.param("grid_map/cone_detection/cone_roi/cone_x_max",
            gm_params_.cone_params.cone_x_max,
            gm_params_.cone_params.cone_x_max);
  nh_.param("grid_map/cone_detection/cone_roi/cone_y_min",
            gm_params_.cone_params.cone_y_min,
            gm_params_.cone_params.cone_y_min);
  nh_.param("grid_map/cone_detection/cone_roi/cone_y_max",
            gm_params_.cone_params.cone_y_max,
            gm_params_.cone_params.cone_y_max);
  nh_.param("grid_map/tracking/v_threshold",
            gm_params_.tracking_params.v_threshold,
            gm_params_.tracking_params.v_threshold);
  nh_.param("grid_map/tracking/sur_age", gm_params_.tracking_params.sur_age,
            gm_params_.tracking_params.sur_age);

  // emergency params
  nh_.param("near_safe_area/slow_down/x", gm_params_.safe_params.slow_down_x_,
            gm_params_.safe_params.slow_down_x_);
  nh_.param("near_safe_area/slow_down/y", gm_params_.safe_params.slow_down_y_,
            gm_params_.safe_params.slow_down_y_);
  nh_.param("near_safe_area/emergency_stop/x", gm_params_.safe_params.stop_x_,
            gm_params_.safe_params.stop_x_);
  nh_.param("near_safe_area/emergency_stop/y", gm_params_.safe_params.stop_y_,
            gm_params_.safe_params.stop_y_);
  nh_.param("near_safe_area/car_info/x", gm_params_.safe_params.car_x_,
            gm_params_.safe_params.car_x_);
  nh_.param("near_safe_area/car_info/y", gm_params_.safe_params.car_y_,
            gm_params_.safe_params.car_y_);

  // subscribe
  sub_point_cloud_ =
      nh_.subscribe(in_point_cloud_topic_, 2, &Node::LivoxCloudCallback, this);
  sub_localization_ = nh_.subscribe(in_localization_topic_, 2,
                                    &Node::LocalizatioinCallback, this);
  sub_mode_ = nh_.subscribe("/perception_mode", 10,
                            &Node::PerceptionModeCloudCallback, this);

  // publish
  if (publish_local_points_) {
    pub_local_points_ =
        nh_.advertise<PointTypeCloud>(out_local_points_topic_, 2);
  }
  image_transport::ImageTransport it(nh_);
  pub_local_grid_map_ = it.advertise(out_local_grid_map_topic_, 2);
  pub_global_objects_ =
      nh_.advertise<cyber_msgs::ObjectArray>(out_global_objects_topic_, 2);
  if (publish_global_points_) {
    pub_global_points_ =
        nh_.advertise<PointTypeCloud>(out_global_points_topic_, 2);
  }

  // lidar_preprocess
  lidar_preprocess_ = std::make_shared<LidarPreprocess>(lp_params_);

  // grid_map
  grid_map_ = std::make_shared<GridMap>(gm_params_);

  // new object segmenter
  os_params_.cone_params = grid_map_->params_.cone_params;
  object_segmenter_ = std::make_shared<ObjectSegmenter>(os_params_);

  // sensor_fusion
  sensor_fusion_ = std::make_shared<SensorFusion>();
}

void Node::LivoxCloudCallback(const PointTypeCloudConstPtr &in_cloud_ptr) {
  double top_receive_time = ros::Time::now().toSec();

  if (have_localization_) {
    geometry_msgs::Pose cur_pose = localization_.pose;
    mrpt::poses::CPose3D cur_pose_mrpt;
    common::GetMrptPose(cur_pose, cur_pose_mrpt);
    // lidar_preprocess
    PointTypeCloudPtr obstacle_cloud_ptr(new PointTypeCloud);
    bool send;
    lidar_preprocess_->LivoxPreprocess(*in_cloud_ptr, cur_pose_mrpt, send,
                                       *obstacle_cloud_ptr);
    cv::Mat roi_grid_map;
    roi_grid_map =
        cv::Mat(grid_map_->roi_map_height_, grid_map_->roi_map_width_, CV_8UC1,
                cv::Scalar::all(0));

    if (obstacle_cloud_ptr->size() > 0) {
      if (publish_local_points_) {
        obstacle_cloud_ptr->header = in_cloud_ptr->header;
        pub_local_points_.publish(obstacle_cloud_ptr);
      }

      // generate local roi_map
      cv::Mat max_height_map, min_height_map;
      grid_map_->GenerateGridMap(*obstacle_cloud_ptr, roi_grid_map,
                                 max_height_map, min_height_map);

      // // object segmentation
      // std::vector<LidarObject> small_objects, big_objects;
      // object_segmenter_->Segmenter(roi_grid_map, max_height_map,
      // min_height_map,
      //                              small_objects, big_objects);

      // // get global objects
      // cyber_msgs::ObjectArray global_big_objects;
      // grid_map_->GetGlobalBigObjects(cur_pose_mrpt, big_objects,
      //                                global_big_objects);

      // sensor_fusion
      // cyber_msgs::ObjectArray fusion_objects;
      // sensor_fusion_->ObjectCallback(global_big_objects,
      //                                ros::Time::now().toSec(),
      //                                fusion_objects);

      // // post process big objects
      // grid_map_->PostProcessBigObjects(fusion_objects, roi_grid_map,
      //                                  glocal_objects);

      // // get cone objects
      // cyber_msgs::ObjectArray global_cone_objects;
      // grid_map_->GetGlobalSmallObjects(cur_pose_mrpt, small_objects,
      //                                  glocal_objects, global_cone_objects);
      // glocal_objects.objects.insert(glocal_objects.objects.end(),
      //                               global_cone_objects.objects.begin(),
      //                               global_cone_objects.objects.end());

      // publish
      if (publish_global_points_) {
        common::TransformPointCloud(cur_pose_mrpt, *obstacle_cloud_ptr);
        obstacle_cloud_ptr->header = in_cloud_ptr->header;
        obstacle_cloud_ptr->header.frame_id = global_frame_id_;
        pub_global_points_.publish(obstacle_cloud_ptr);
      }

      ROS_INFO("porcess time: %f", ros::Time::now().toSec() - top_receive_time);
    }

    // publish
    if (send && pub_grid_map_flag) {
      PublishImage(roi_grid_map, in_cloud_ptr->header.stamp,
                   pub_local_grid_map_);
    }
  } else {
    ROS_WARN("[Node::LivoxCloudCallback]: Don't have localization results!");
  }
}

void Node::LocalizatioinCallback(
    const cyber_msgs::LocalizationEstimate::ConstPtr &in_localization_msg) {
  if (in_localization_msg->status > 0) {
    localization_ = *in_localization_msg;
    have_localization_ = true;
  }
}

void Node::PublishImage(const cv::Mat &image, const uint64_t &stamp,
                        const image_transport::Publisher &pub) {
  sensor_msgs::ImagePtr image_msg =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
  pcl_conversions::fromPCL(stamp, image_msg->header.stamp);
  image_msg->header.frame_id = local_frame_id_;
  pub.publish(image_msg);
}

void Node::PublishObjects(cyber_msgs::ObjectArray &objects,
                          const uint64_t &stamp, const ros::Publisher &pub) {
  pcl_conversions::fromPCL(stamp, objects.header.stamp);
  objects.header.frame_id = global_frame_id_;
  for (size_t i = 0; i < objects.objects.size(); i++) {
    objects.objects[i].header = objects.header;
  }
  pub.publish(objects);
}

void Node::PerceptionModeCloudCallback(const std_msgs::Int8ConstPtr &msg) {
  const int &m = msg->data;
  ROS_INFO("Recieved perception mode: %d", m);
  if (m == 2 || m == 3) {
    pub_grid_map_flag = true;
  } else {
    pub_grid_map_flag = false;
  }
}

int main(int argc, char **argv) {
  // 只在生成发布版的时候才解注释，否则在自己电脑上无法调试
  // if(!checkUSBKey()) return 0;

  ros::init(argc, argv, "lidar_percetion");
  Node node;

  ros::spin();

  return 0;
}
