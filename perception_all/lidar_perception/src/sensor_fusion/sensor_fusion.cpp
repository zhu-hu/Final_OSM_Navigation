/*
 * Cobjectyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao
 * Tong University. All rights reserved. Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.03.10
 */

#include "sensor_fusion/sensor_fusion.h"

SensorFusion::SensorFusion() { track_objects_.clear(); }

void SensorFusion::ObjectCallback(const cyber_msgs::ObjectArray& meas_objects,
                                  double time_stamp,
                                  cyber_msgs::ObjectArray& out_objects) {
  int meas_size = meas_objects.objects.size();
  if (track_objects_.empty()) {
    track_objects_.resize(meas_size);
    for (size_t mi = 0; mi < meas_size; mi++) {
      TrackingObject& track_object = track_objects_[mi];
      track_object.id = cur_id_;
      cur_id_++;
      cur_id_ = cur_id_ < id_max_ ? cur_id_ : 0;
      track_object.sur_age = 1;
      track_object.dis_age = 0;
      track_object.pre_timestamp = time_stamp;
      track_object.meas_object = meas_objects.objects[mi];
      kf_.Init(track_object);
    }
  } else {
    // no measurements
    if (meas_size == 0) {
      track_objects_.clear();
      return;
    }
    // predict old tracking list
    for (int ti = 0; ti < track_objects_.size(); ti++) {
      kf_.Predict(time_stamp - track_objects_[ti].pre_timestamp,
                  track_objects_[ti]);
      track_objects_[ti].pre_timestamp = time_stamp;
    }

    // data association
    std::vector<std::pair<std::vector<int>, std::vector<int>>>
        association_list = data_association_.GreedyAlgorithmLidar(
            meas_objects, track_objects_);

    // update new vehicle list
    std::vector<TrackingObject> track_objects_copy;
    track_objects_copy.swap(track_objects_);
    for (int i = 0; i < association_list.size(); ++i) {
      std::vector<int> track_vec = association_list[i].first;
      std::vector<int> meas_vec = association_list[i].second;
      if (track_vec[0] == -1) {  // new object appear
        TrackingObject new_object;
        new_object.id = cur_id_;
        cur_id_++;
        cur_id_ = cur_id_ < id_max_ ? cur_id_ : 0;
        new_object.sur_age = 1;
        new_object.dis_age = 0;
        new_object.pre_timestamp = time_stamp;
        new_object.meas_object = meas_objects.objects[meas_vec[0]];
        kf_.Init(new_object);
        track_objects_.push_back(new_object);
      } else if (meas_vec[0] == -1) {  // old object disappear
      } else {                         // association success
        TrackingObject track_object = track_objects_copy[track_vec[0]];
        track_object.sur_age += 1;
        track_object.dis_age = 0;
        uint32_t pre_object_type = track_object.meas_object.object_type;
        track_object.meas_object = meas_objects.objects[meas_vec[0]];
        if (pre_object_type == 4) {
          track_object.meas_object.object_type = 4;
        }
        kf_.Update(track_object);
        track_objects_.push_back(track_object);
      }
    }
  }
  OutputObject(out_objects);
}
void SensorFusion::OutputObject(cyber_msgs::ObjectArray& out_objects) {
  out_objects.objects.resize(track_objects_.size());
  for (size_t i = 0; i < track_objects_.size(); i++) {
    const TrackingObject& track_object = track_objects_[i];
    cyber_msgs::Object& meas_object = out_objects.objects[i];
    meas_object = track_object.meas_object;
    float dx = track_object.x(2);
    float dy = track_object.x(3);
    float meas_angle = track_object.x(4);
    float v_angle =
        common::ControlAngleRad(atan2f(track_object.x(6), track_object.x(5)));
    bool need_trans;
    common::AlignAngleRad(v_angle, meas_angle, need_trans);
    if (need_trans) {
      float tmp = dx;
      dx = dy;
      dy = tmp;
    }
    // change value
    meas_object.object_id = track_object.id;
    if (track_object.sur_age == 1)
      meas_object.track_status = 1;
    else
      meas_object.track_status = 2;
    meas_object.track_age = track_object.sur_age;
    meas_object.pose.position.x = track_object.x(0);
    meas_object.pose.position.y = track_object.x(1);
    meas_object.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, meas_angle);
    meas_object.dimensions.x = dx;
    meas_object.dimensions.y = dy;
    meas_object.velocity.linear.x = track_object.x(5);
    meas_object.velocity.linear.y = track_object.x(6);
  }
}
