/*
 * Copyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.02.21
 */

#include "grid_map/grid_map.h"

GridMap::GridMap(const GridMapParams& params) {
  params_ = params;
  // roi grid map
  roi_map_height_ =
      static_cast<int>((params_.roi_params.max_x - params_.roi_params.min_x) *
                       params_.roi_params.pixel_scale);
  roi_map_width_ =
      static_cast<int>((params_.roi_params.max_y - params_.roi_params.min_y) *
                       params_.roi_params.pixel_scale);
  roi_map_height_origin_ = static_cast<int>(params_.roi_params.max_x *
                                            params_.roi_params.pixel_scale);
  roi_map_width_origin_ = static_cast<int>(params_.roi_params.max_y *
                                           params_.roi_params.pixel_scale);
  LocalToPixel(params_.cone_params.cone_x_max, params_.cone_params.cone_y_max,
               params_.cone_params.cone_row_min,
               params_.cone_params.cone_col_min);
  LocalToPixel(params_.cone_params.cone_x_min, params_.cone_params.cone_y_min,
               params_.cone_params.cone_row_max,
               params_.cone_params.cone_col_max);
  // emergency
  LocalToPixel(params_.safe_params.car_x_ + params_.safe_params.slow_down_x_,
               params_.safe_params.car_y_ + params_.safe_params.slow_down_y_,
               params_.safe_params.slow_down_row_min,
               params_.safe_params.slow_down_col_min);
  LocalToPixel(params_.safe_params.car_x_,
               -params_.safe_params.car_y_ - params_.safe_params.slow_down_y_,
               params_.safe_params.slow_down_row_max,
               params_.safe_params.slow_down_col_max);
  LocalToPixel(params_.safe_params.car_x_ + params_.safe_params.stop_x_,
               params_.safe_params.car_y_ + params_.safe_params.stop_y_,
               params_.safe_params.stop_row_min,
               params_.safe_params.stop_col_min);
  LocalToPixel(params_.safe_params.car_x_,
               -params_.safe_params.car_y_ - params_.safe_params.stop_y_,
               params_.safe_params.stop_row_max,
               params_.safe_params.stop_col_max);
}

void GridMap::GenerateGridMap(const PointTypeCloud& cloud_in,
                              cv::Mat& roi_grid_map, cv::Mat& max_height_map,
                              cv::Mat& min_height_map) {
  roi_grid_map =
      cv::Mat(roi_map_height_, roi_map_width_, CV_8UC1, cv::Scalar::all(0));
  max_height_map = cv::Mat_<float>(roi_map_height_, roi_map_width_, -3.0);
  min_height_map = cv::Mat_<float>(roi_map_height_, roi_map_width_, 3.0);
  for (int i = 0; i < cloud_in.size(); ++i) {
    int col, row;
    const PointType& pt = cloud_in.points[i];
    LocalToPixel(pt.x, pt.y, row, col);
    if (row >= 0 && row < roi_map_height_ && col >= 0 && col < roi_map_width_) {
      roi_grid_map.at<uchar>(row, col) = (uchar)255;
      float max_height = max_height_map.at<float>(row, col);
      if (max_height < pt.z) {
        max_height_map.at<float>(row, col) = pt.z;
      }
      float min_height = min_height_map.at<float>(row, col);
      if (min_height > pt.z) {
        min_height_map.at<float>(row, col) = pt.z;
      }
    }
  }
}

void GridMap::GenerateGridMap(const PointTypeCloud& cloud_in,
                              cv::Mat& roi_grid_map, int& safe) {
  roi_grid_map =
      cv::Mat(roi_map_height_, roi_map_width_, CV_8UC1, cv::Scalar::all(0));
  cv::Mat count_map = cv::Mat_<int>(roi_map_height_, roi_map_width_, 0);
  safe = 0;
  for (int i = 0; i < cloud_in.size(); ++i) {
    int col, row;
    const PointType& pt = cloud_in.points[i];
    LocalToPixel(pt.x, pt.y, row, col);
    if (row >= params_.safe_params.slow_down_row_min &&
        row <= params_.safe_params.slow_down_row_max &&
        col >= params_.safe_params.slow_down_col_min &&
        col <= params_.safe_params.slow_down_col_max) {
      int& count = count_map.at<int>(row, col);
      count++;
      if (count >= 2) {
        roi_grid_map.at<uchar>(row, col) = (uchar)255;
        if (row >= params_.safe_params.stop_row_min &&
            row <= params_.safe_params.stop_row_max &&
            col <= params_.safe_params.stop_col_min &&
            col >= params_.safe_params.stop_col_max) {
          safe = 2;
        } else {
          if (safe == 0) {
            safe = 1;
          }
        }
      }
    }
  }
}

void GridMap::GetGlobalBigObjects(const mrpt::poses::CPose3D& pose,
                                  const std::vector<LidarObject>& lidar_objects,
                                  cyber_msgs::ObjectArray& global_objects) {
  global_objects.objects.resize(lidar_objects.size());
  for (size_t i = 0; i < lidar_objects.size(); i++) {
    const LidarObject& lidar_object = lidar_objects[i];
    cyber_msgs::Object& out_object = global_objects.objects[i];
    // get position
    const cv::Point2f& center = lidar_object.rRect.center;
    float pixel_row = center.y;
    float pixel_col = center.x;
    float local_px, local_py;
    PixelToLocal(pixel_row, pixel_col, local_px, local_py);
    float global_x, global_y;
    LocalToGlobal(pose, local_px, local_py, global_x, global_y);
    // get yaw and dimension
    float pixel_yaw = lidar_object.rRect.angle - 90;  // [Deg]
    float pixel_dx = lidar_object.rRect.size.width;
    float pixel_dy = lidar_object.rRect.size.height;
    pixel_dx = pixel_dx > 0 ? pixel_dx : 1;
    pixel_dy = pixel_dy > 0 ? pixel_dy : 1;
    float local_yaw, local_dx, local_dy;
    if (pixel_dy > pixel_dx) {
      local_dx = pixel_dy;
      local_dy = pixel_dx;
      local_yaw =
          common::ControlAngleRad((-pixel_yaw + 90) / 180 * M_PI);  // [Rad]
    } else {
      local_dx = pixel_dx;
      local_dy = pixel_dy;
      local_yaw = common::ControlAngleRad((-pixel_yaw) / 180 * M_PI);  // [Rad]
    }
    float global_yaw = common::ControlAngleRad(local_yaw + pose.yaw());
    out_object.header.stamp = ros::Time::now();
    out_object.sensor_type = 0;  // Lidar
    out_object.object_id = i;
    out_object.object_type = 0;  // Unknown
    out_object.object_score = 0.0;
    out_object.track_status = 0;  // Untracked
    out_object.track_age = 0;
    out_object.motion_status = 0;  // Unknown
    out_object.pose.position.x = global_x;
    out_object.pose.position.y = global_y;
    out_object.pose.position.z =
        (lidar_object.max_z + lidar_object.min_z) * 0.5;
    out_object.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, global_yaw);
    out_object.dimensions.x = local_dx / params_.roi_params.pixel_scale;
    out_object.dimensions.y = local_dy / params_.roi_params.pixel_scale;
    out_object.dimensions.z = fabs(lidar_object.max_z - lidar_object.min_z);
    out_object.dimensions.z =
        out_object.dimensions.z > 0.05 ? out_object.dimensions.z : 0.05;
    out_object.velocity.linear.x = 0.0;
    out_object.velocity.linear.y = 0.0;
    out_object.velocity.linear.z = 0.0;
    out_object.velocity.angular.x = 0.0;
    out_object.velocity.angular.y = 0.0;
    out_object.velocity.angular.z = 0.0;
    out_object.acceleration.linear.x = 0.0;
    out_object.acceleration.linear.y = 0.0;
    out_object.acceleration.linear.z = 0.0;
    out_object.acceleration.angular.x = 0.0;
    out_object.acceleration.angular.y = 0.0;
    out_object.acceleration.angular.z = 0.0;
    // get polygon
    TransPolygon(lidar_object, pose, out_object.polygon);
    // judge type
    float pixel_size_max = local_dx > local_dy ? local_dx : local_dy;
    float pixel_size_min = local_dx < local_dy ? local_dx : local_dy;
    if (pixel_size_max < params_.person_params.size_max &&
        pixel_size_min > params_.person_params.size_min &&
        (pixel_size_max * pixel_size_min < params_.person_params.area_max) &&
        (out_object.dimensions.z > params_.person_params.height_min)) {
      out_object.object_type = 1;  // Pedestrian
    } else if (pixel_size_max >= params_.person_params.size_max ||
               (pixel_size_max * pixel_size_min >=
                params_.person_params.area_max)) {
      out_object.object_type = 4;  // Car
    }
    // save hull points in image
    PointTypeCloudPtr hull_points(new PointTypeCloud);
    for (size_t hi = 0; hi < lidar_object.hull.size(); hi++) {
      const cv::Point& hull_pt = lidar_object.hull[hi];
      PointType pt;
      pt.x = hull_pt.x;
      pt.y = hull_pt.y;
      hull_points->push_back(pt);
    }
    pcl::toROSMsg(*hull_points, out_object.point_cloud);
  }
}

void GridMap::PostProcessBigObjects(const cyber_msgs::ObjectArray& in_objects,
                                    cv::Mat& roi_grid_map,
                                    cyber_msgs::ObjectArray& out_objects) {
  out_objects.objects.clear();
  for (size_t i = 0; i < in_objects.objects.size(); i++) {
    const cyber_msgs::Object& object = in_objects.objects[i];
    if (object.track_age > params_.tracking_params.sur_age ||
        object.object_type == 1) {
      float liner_v = sqrt(object.velocity.linear.x * object.velocity.linear.x +
                           object.velocity.linear.y * object.velocity.linear.y);
      uint32_t motion_status;
      if (liner_v > params_.tracking_params.v_threshold)
        motion_status = 2;  // Moving
      else
        motion_status = 1;  // Static
      if (motion_status == 2 || object.object_type == 1) {
        // remove points from roi_grid_map
        PointTypeCloudPtr hull_points(new PointTypeCloud);
        pcl::fromROSMsg(object.point_cloud, *hull_points);
        cv::Point image_points[1][hull_points->size()];
        for (size_t pi = 0; pi < hull_points->size(); pi++) {
          image_points[0][pi].x = hull_points->points[pi].x;
          image_points[0][pi].y = hull_points->points[pi].y;
        }
        const cv::Point* ppt[1] = {image_points[0]};
        int npt[] = {(int)hull_points->size()};
        cv::fillPoly(roi_grid_map, ppt, npt, 1, cv::Scalar(0));
        cyber_msgs::Object out_object = object;
        out_object.motion_status = motion_status;
        out_objects.objects.emplace_back(out_object);
      }
    }
  }
}

void GridMap::GetGlobalSmallObjects(
    const mrpt::poses::CPose3D& pose,
    const std::vector<LidarObject>& lidar_objects,
    const cyber_msgs::ObjectArray& big_objects,
    cyber_msgs::ObjectArray& global_objects) {
  global_objects.objects.clear();
  // get big_points
  pcl::PointCloud<pcl::PointXYZ>::Ptr big_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<std::vector<cv::Point2f>> polygon_vec;
  bool big_check = big_objects.objects.size() > 0;
  if (big_check) {
    big_cloud_ptr->resize(big_objects.objects.size());
    polygon_vec.resize(big_objects.objects.size());
    for (size_t i = 0; i < big_objects.objects.size(); i++) {
      const cyber_msgs::Object& object = big_objects.objects[i];
      pcl::PointXYZ& pt = big_cloud_ptr->points[i];
      pt.x = object.pose.position.x;
      pt.y = object.pose.position.y;
      pt.z = 0;
      std::vector<cv::Point2f>& polygon = polygon_vec[i];
      polygon.resize(object.polygon.points.size());
      for (size_t j = 0; j < object.polygon.points.size(); j++) {
        const geometry_msgs::Point32& src_pt = object.polygon.points[j];
        cv::Point2f& dst_pt = polygon[j];
        dst_pt.x = src_pt.x;
        dst_pt.y = src_pt.y;
      }
    }
  }
  // set big_kdtree for search
  pcl::KdTreeFLANN<pcl::PointXYZ> big_kdtree;
  if (big_check) {
    big_kdtree.setInputCloud(big_cloud_ptr);
  }

  double cur_time = ros::Time::now().toSec();
  // get new cone objects
  std::vector<std::pair<double, cyber_msgs::Object>> global_new_cone_objects;
  global_new_cone_objects.reserve(lidar_objects.size());
  pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  new_cloud_ptr->reserve(lidar_objects.size());
  for (size_t i = 0; i < lidar_objects.size(); i++) {
    const LidarObject& lidar_object = lidar_objects[i];
    const cv::Point2f& center = lidar_object.rRect.center;
    float pixel_row = center.y;
    float pixel_col = center.x;
    float local_px, local_py;
    PixelToLocal(pixel_row, pixel_col, local_px, local_py);
    float global_x, global_y;
    LocalToGlobal(pose, local_px, local_py, global_x, global_y);
    pcl::PointXYZ search_pt(global_x, global_y, 0);
    bool remain = true;
    if (big_check) {
      std::vector<int> pointIdxKNNSearch(1);
      std::vector<float> pointKNNSquaredDistance(1);
      if (big_kdtree.nearestKSearch(search_pt, 1, pointIdxKNNSearch,
                                    pointKNNSquaredDistance) > 0) {
        int index = pointIdxKNNSearch[0];
        cv::Point2f check_pt;
        check_pt.x = search_pt.x;
        check_pt.y = search_pt.y;
        double distance =
            cv::pointPolygonTest(polygon_vec[index], check_pt, true);
        if (distance > -params_.cone_params.research_dis) {
          remain = false;
        }
      }
    }
    float height = fabs(lidar_object.max_z - lidar_object.min_z);
    if (remain && height < params_.cone_params.height_max &&
        local_px < params_.cone_params.cone_x_max &&
        local_py < params_.cone_params.cone_y_max &&
        local_px > params_.cone_params.cone_x_min &&
        local_py > params_.cone_params.cone_y_min) {
      // cone objects
      cyber_msgs::Object out_object;
      out_object.header.stamp = ros::Time::now();
      out_object.sensor_type = 0;  // Lidar
      out_object.object_id = 0;
      out_object.object_type = 2;  // Cone
      out_object.object_score = 0.0;
      out_object.track_status = 0;  // Untracked
      out_object.track_age = 5;
      out_object.motion_status = 1;  // Static
      out_object.pose.position.x = global_x;
      out_object.pose.position.y = global_y;
      out_object.pose.position.z =
          (lidar_object.max_z + lidar_object.min_z) * 0.5;
      out_object.pose.orientation =
          tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
      out_object.dimensions.x =
          params_.cone_params.size_max / params_.roi_params.pixel_scale;
      out_object.dimensions.y = out_object.dimensions.x;
      out_object.dimensions.z = height > 0.05 ? height : 0.05;
      out_object.velocity.linear.x = 0.0;
      out_object.velocity.linear.y = 0.0;
      out_object.velocity.linear.z = 0.0;
      out_object.velocity.angular.x = 0.0;
      out_object.velocity.angular.y = 0.0;
      out_object.velocity.angular.z = 0.0;
      out_object.acceleration.linear.x = 0.0;
      out_object.acceleration.linear.y = 0.0;
      out_object.acceleration.linear.z = 0.0;
      out_object.acceleration.angular.x = 0.0;
      out_object.acceleration.angular.y = 0.0;
      out_object.acceleration.angular.z = 0.0;
      out_object.polygon.points.resize(4);
      float delta_size = 0.1;
      geometry_msgs::Point32& pt1 = out_object.polygon.points[0];
      pt1.x = global_x + delta_size;
      pt1.y = global_y + delta_size;
      pt1.z = 0.0;
      geometry_msgs::Point32& pt2 = out_object.polygon.points[1];
      pt2.x = global_x - delta_size;
      pt2.y = global_y + delta_size;
      pt2.z = 0.0;
      geometry_msgs::Point32& pt3 = out_object.polygon.points[2];
      pt3.x = global_x - delta_size;
      pt3.y = global_y - delta_size;
      pt3.z = 0.0;
      geometry_msgs::Point32& pt4 = out_object.polygon.points[3];
      pt4.x = global_x + delta_size;
      pt4.y = global_y - delta_size;
      pt4.z = 0.0;
      global_objects.objects.emplace_back(out_object);
      std::pair<double, cyber_msgs::Object> cone_object;
      cone_object.first = cur_time;
      cone_object.second = out_object;
      global_new_cone_objects.emplace_back(cone_object);
      pcl::PointXYZ pt;
      pt.x = global_x;
      pt.y = global_y;
      pt.z = 0;
      new_cloud_ptr->push_back(pt);
    }
  }
  // set big_kdtree for search
  pcl::KdTreeFLANN<pcl::PointXYZ> new_kdtree;
  bool new_check = new_cloud_ptr->size() > 0;
  if (new_check) {
    new_kdtree.setInputCloud(new_cloud_ptr);
  }

  // remove invalid old objects
  std::vector<std::pair<double, cyber_msgs::Object>> cone_objects_copy;
  cone_objects_copy.swap(global_cone_objects_);
  for (size_t i = 0; i < cone_objects_copy.size(); i++) {
    double cone_time = cone_objects_copy[i].first;
    if (cur_time - cone_time < params_.cone_params.sur_time) {
      bool remain = true;
      pcl::PointXYZ search_pt;
      search_pt.x = cone_objects_copy[i].second.pose.position.x;
      search_pt.y = cone_objects_copy[i].second.pose.position.y;
      search_pt.z = 0;
      if (big_check) {
        std::vector<int> pointIdxKNNSearch(1);
        std::vector<float> pointKNNSquaredDistance(1);
        if (big_kdtree.nearestKSearch(search_pt, 1, pointIdxKNNSearch,
                                      pointKNNSquaredDistance) > 0) {
          int index = pointIdxKNNSearch[0];
          cv::Point2f check_pt;
          check_pt.x = search_pt.x;
          check_pt.y = search_pt.y;
          double distance =
              cv::pointPolygonTest(polygon_vec[index], check_pt, true);
          if (distance > -params_.cone_params.research_dis) {
            remain = false;
          }
        }
      }
      if (remain && new_check) {
        std::vector<int> pointIdxKNNSearch(1);
        std::vector<float> pointKNNSquaredDistance(1);
        if (new_kdtree.nearestKSearch(search_pt, 1, pointIdxKNNSearch,
                                      pointKNNSquaredDistance) > 0) {
          int index = pointIdxKNNSearch[0];
          const pcl::PointXYZ& new_pt = new_cloud_ptr->at(index);
          double distance =
              pow(search_pt.x - new_pt.x, 2) + pow(search_pt.y - new_pt.y, 2);
          if (distance < 0.09) {  // 0.3m * 0.3m
            remain = false;
          }
        }
      }
      if (remain) {
        global_cone_objects_.emplace_back(cone_objects_copy[i]);
        global_objects.objects.emplace_back(cone_objects_copy[i].second);
      }
    }
  }

  // push new objects
  if (global_new_cone_objects.size() > 0) {
    global_cone_objects_.insert(global_cone_objects_.end(),
                                global_new_cone_objects.begin(),
                                global_new_cone_objects.end());
  }
}

void GridMap::TransPolygon(const LidarObject& object_in,
                           const mrpt::poses::CPose3D& pose,
                           geometry_msgs::Polygon& polygon_out) {
  if (object_in.hull.size() < 3) {
    polygon_out.points.resize(4);
    cv::Point2f vertices[4];
    object_in.rRect.points(vertices);
    for (int i = 0; i < 4; i++) {
      float pixel_row = vertices[i].y;
      float pixel_col = vertices[i].x;
      float local_x, local_y;
      PixelToLocal(pixel_row, pixel_col, local_x, local_y);
      float global_x, global_y;
      LocalToGlobal(pose, local_x, local_y, global_x, global_y);
      geometry_msgs::Point32& world_point = polygon_out.points[i];
      world_point.x = global_x;
      world_point.y = global_y;
      world_point.z = 0.0;
    }
  } else {
    const std::vector<cv::Point>& hull = object_in.hull;
    polygon_out.points.resize(hull.size());
    for (size_t hi = 0; hi < hull.size(); hi++) {
      const cv::Point& pt = hull[hi];
      // Transform convex hull points from pixel to base_link
      float pixel_row = pt.y;
      float pixel_col = pt.x;
      float local_x, local_y;
      PixelToLocal(pixel_row, pixel_col, local_x, local_y);
      float global_x, global_y;
      LocalToGlobal(pose, local_x, local_y, global_x, global_y);
      geometry_msgs::Point32& world_point = polygon_out.points[hi];
      world_point.x = global_x;
      world_point.y = global_y;
      world_point.z = 0.0;
    }
  }
}

void GridMap::LocalToPixel(const float& real_x, const float& real_y,
                           int& pixel_row, int& pixel_coln) {
  pixel_row = roi_map_height_origin_ -
              static_cast<int>(real_x * params_.roi_params.pixel_scale);
  pixel_coln = roi_map_width_origin_ -
               static_cast<int>(real_y * params_.roi_params.pixel_scale);
}

void GridMap::PixelToLocal(const float& pixel_row, const float& pixel_coln,
                           float& real_x, float& real_y) {
  real_x =
      (roi_map_height_origin_ - pixel_row) / params_.roi_params.pixel_scale;
  real_y =
      (roi_map_width_origin_ - pixel_coln) / params_.roi_params.pixel_scale;
}

void GridMap::LocalToGlobal(const mrpt::poses::CPose3D& pose,
                            const float& local_x, const float& local_y,
                            float& global_x, float& global_y) {
  float global_z;
  pose.composePoint(local_x, local_y, 0.0f, global_x, global_y, global_z);
}
