#include "global_visualize.h"

GlobalViewerNode::GlobalViewerNode() : nh_("~") {
  sub_object_array_ = nh_.subscribe(
      object_array_topic_, 2, &GlobalViewerNode::ObjectArrayCallback, this);
  sub_localization_ = nh_.subscribe(
      localization_topic_, 2, &GlobalViewerNode::LocalizationCallback, this);

  pub_bounding_box_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(
      bounding_box_topic_, 2);
  pub_orientation_ =
      nh_.advertise<visualization_msgs::MarkerArray>(orientation_topic_, 2);
  pub_polygon_ =
      nh_.advertise<jsk_recognition_msgs::PolygonArray>(polygon_topic_, 2);
  pub_id_and_velocity_ =
      nh_.advertise<visualization_msgs::MarkerArray>(id_and_velocity_topic_, 2);

  // TF broadcaster
  tf_br_ = new tf::TransformBroadcaster();
}

void GlobalViewerNode::ObjectArrayCallback(
    const cyber_msgs::ObjectArray::ConstPtr& object_array) {
  jsk_recognition_msgs::BoundingBoxArray bounding_box_array;
  jsk_recognition_msgs::PolygonArray polygon_array;

  // Delete marker from last frame
  if (arrow_array_.markers.size() > 0) {
    for (int i = 0; i < arrow_array_.markers.size(); i++) {
      arrow_array_.markers[i].header.stamp = ros::Time::now();
      arrow_array_.markers[i].action = visualization_msgs::Marker::DELETE;
    }
    pub_orientation_.publish(arrow_array_);
    arrow_array_.markers.clear();
  }
  if (id_array_.markers.size() > 0) {
    for (int i = 0; i < id_array_.markers.size(); i++) {
      id_array_.markers[i].header.stamp = ros::Time::now();
      id_array_.markers[i].action = visualization_msgs::Marker::DELETE;
    }
    pub_id_and_velocity_.publish(id_array_);
    id_array_.markers.clear();
  }

  // Arrow config
  visualization_msgs::Marker arrow, id_text;
  arrow.header.stamp = id_text.header.stamp = ros::Time::now();
  arrow.header.frame_id = id_text.header.frame_id = frame_id_;
  arrow.ns = "arrow";
  id_text.ns = "text";
  arrow.action = id_text.action = visualization_msgs::Marker::ADD;
  arrow.type = visualization_msgs::Marker::ARROW;
  id_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  arrow.scale.x = 0.2;
  arrow.scale.y = 1.0;
  arrow.scale.z = 0.5;
  arrow.color.b = arrow.color.a = 1.0;
  id_text.scale.x = 1;
  id_text.scale.y = 1;
  id_text.scale.z = 1;
  id_text.color.r = id_text.color.g = id_text.color.a = 1.0;

  for (size_t i = 0; i < object_array->objects.size(); ++i) {
    cyber_msgs::Object object = object_array->objects[i];
    // Bounding box
    jsk_recognition_msgs::BoundingBox bounding_box;
    // Bounding box label
    bounding_box.label = i;
    // Bounding box value
    bounding_box.value = object.track_age;
    // Bounding box position
    bounding_box.pose = object.pose;
    // Bounding box size
    bounding_box.dimensions = object.dimensions;
    // Bounding box header
    bounding_box.header = object_array->header;
    bounding_box.header.frame_id = frame_id_;
    bounding_box_array.boxes.push_back(bounding_box);

    // Polygon
    geometry_msgs::PolygonStamped polygon_stamped;
    polygon_stamped.header = object_array->header;
    polygon_stamped.header.frame_id = frame_id_;
    polygon_stamped.polygon = object.polygon;
    polygon_array.polygons.push_back(polygon_stamped);

    if (object.motion_status == 2) {
      // Arrow array(the arrow needs two points for each)
      arrow.id = i;
      arrow.points.clear();
      // Get the first point
      geometry_msgs::Point pt = object.pose.position;
      arrow.points.push_back(pt);
      // Get the second point
      double roll, pitch, yaw;
      tf::Matrix3x3(
          tf::Quaternion(object.pose.orientation.x, object.pose.orientation.y,
                         object.pose.orientation.z, object.pose.orientation.w))
          .getRPY(roll, pitch, yaw);
      pt.x += cos(yaw) * 1.5 * object.dimensions.x;
      pt.y += sin(yaw) * 1.5 * object.dimensions.x;
      arrow.points.push_back(pt);
      arrow_array_.markers.push_back(arrow);
    }

    // set text(object id and velocity)
    id_text.id = i;
    double liner_v = sqrt(object.velocity.linear.x * object.velocity.linear.x +
                          object.velocity.linear.y * object.velocity.linear.y);
    std::ostringstream strm;
    strm.precision(1);
    strm.setf(std::ios::fixed);
    strm << liner_v;
    if (object.object_type == 1) {
      id_text.text =
          std::to_string(object.object_id) + "_" + strm.str() + "_Pedestrian";
    } else if (object.object_type == 2) {
      id_text.text = "C";
    } else if (object.object_type == 4) {
      id_text.text =
          std::to_string(object.object_id) + "_" + strm.str() + "_Car";
    } else {
      id_text.text = std::to_string(object.object_id) + "_" + strm.str();
    }
    id_text.pose = object.pose;
    id_text.pose.position.z = object.pose.position.z + 4 * object.dimensions.z;
    id_text.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0, -0.5 * M_PI, 0);
    id_array_.markers.push_back(id_text);
  }

  // Publish bounding box
  bounding_box_array.header = object_array->header;
  bounding_box_array.header.frame_id = frame_id_;
  pub_bounding_box_.publish(bounding_box_array);

  // Publish polygon array
  polygon_array.header = object_array->header;
  polygon_array.header.frame_id = frame_id_;
  pub_polygon_.publish(polygon_array);

  // Publish orientation as arrow
  if (arrow_array_.markers.size() > 0) {
    pub_orientation_.publish(arrow_array_);
  }

  // Publish pictogram array
  if (id_array_.markers.size() > 0) {
    pub_id_and_velocity_.publish(id_array_);
  }
}

void GlobalViewerNode::LocalizationCallback(
    cyber_msgs::LocalizationEstimate localization_estimation) {
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(localization_estimation.pose.position.x,
                                  localization_estimation.pose.position.y,
                                  0.0));
  tf::Quaternion q;
  q.setX(localization_estimation.pose.orientation.x);
  q.setY(localization_estimation.pose.orientation.y);
  q.setZ(localization_estimation.pose.orientation.z);
  q.setW(localization_estimation.pose.orientation.w);
  transform.setRotation(q);
  tf_br_->sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_visualize_node");
  GlobalViewerNode global_visualize_node;
  ros::spin();

  return 0;
}
