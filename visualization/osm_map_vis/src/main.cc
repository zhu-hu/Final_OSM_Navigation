#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "cyber_msgs/GPGGA_MSG.h"
#include "cyber_msgs/Heading.h"
#include "cyber_msgs/LocalizationEstimate.h"
#include "osm_parser/src/coordinates_converter/wgs84_to_utm.h"
#include "osm_parser/src/osm_parser.h"

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::NavSatFix, cyber_msgs::Heading, cyber_msgs::GPGGA_MSG>
    GpsPolicy;
message_filters::Subscriber<sensor_msgs::NavSatFix> *sub_gps_fix = nullptr;
message_filters::Subscriber<cyber_msgs::Heading> *sub_gps_heading = nullptr;
message_filters::Subscriber<cyber_msgs::GPGGA_MSG> *sub_gps_status = nullptr;
message_filters::Synchronizer<GpsPolicy> *gps_sync = nullptr;

cyber_msgs::LocalizationEstimate adc_state;
double last_utm_x = 0.0;
double last_utm_y = 0.0;
ros::Publisher pub_loaclization_estimation;
ros::Timer publisher_timer;

void GPSCallback(const sensor_msgs::NavSatFixConstPtr &gps_in,
                 const cyber_msgs::HeadingConstPtr &heading_in,
                 const cyber_msgs::GPGGA_MSGConstPtr &status_in) {
  double lat = gps_in->latitude;
  double lon = gps_in->longitude;
  double utm_x = 0.0;
  double utm_y = 0.0;
  double utm_yaw = 0.0;
  LLtoUTM(lat, lon, utm_y, utm_x);
  utm_yaw = atan2(utm_y - last_utm_y, utm_x - last_utm_x);
  std::cout << "heading : " << utm_yaw << std::endl;

  adc_state.header.frame_id = "world";
  adc_state.header.stamp = ros::Time::now();
  adc_state.longitude = lon;
  adc_state.latitude = lat;
  adc_state.pose.position.x = utm_x;
  adc_state.pose.position.y = utm_y;
  adc_state.pose.position.z = 0.0;

  auto tf_q = tf::createQuaternionFromYaw(utm_yaw);

  adc_state.pose.orientation.x = tf_q.getX();
  adc_state.pose.orientation.y = tf_q.getY();
  adc_state.pose.orientation.z = tf_q.getZ();
  adc_state.pose.orientation.w = tf_q.getW();

  std::cout << "[" << tf_q.getX() << ", " << tf_q.getY() << ", " << tf_q.getZ()
            << ", " << tf_q.getW() << "]" << std::endl;
  last_utm_x = utm_x;
  last_utm_y = utm_y;

  pub_loaclization_estimation.publish(adc_state);
}

int main(int argc, char **argv) {
  // Initialize ROS node
  ros::init(argc, argv, "osm_map_rviz_node");
  ros::NodeHandle pnh("~");

  //   sub_gps_fix = new message_filters::Subscriber<sensor_msgs::NavSatFix>(
  //       pnh, "/strong/fix", 1);
  //   sub_gps_heading = new message_filters::Subscriber<cyber_msgs::Heading>(
  //       pnh, "/strong/heading", 1);
  //   sub_gps_status = new message_filters::Subscriber<cyber_msgs::GPGGA_MSG>(
  //       pnh, "/strong/raw_data", 1);
  //   gps_sync = new message_filters::Synchronizer<GpsPolicy>(
  //       GpsPolicy(10), *sub_gps_fix, *sub_gps_heading, *sub_gps_status);
  //   gps_sync->registerCallback(boost::bind(&GPSCallback, _1, _2, _3));

  //   pub_loaclization_estimation =
  //   pnh.advertise<cyber_msgs::LocalizationEstimate>(
  //       "/localization/estimation", 5);

  // publisher_timer = pnh.createTimer(ros::Duration(0.01), &TimerCallback);

  osm_parser::Parser *osm_map = new osm_parser::Parser;

  std::string osm_map_path = "";
  pnh.param<std::string>("osm_map_path", osm_map_path, "");
  std::cout << "osm map path : " << osm_map_path << std::endl;
  osm_map->setNewMap(osm_map_path);

  std::vector<std::string> ways_filter{"service"};
  //   pnh.param("osm_ways_filter", ways_filter);
  osm_map->setTypeOfWays(ways_filter);

  double interpolation_max_distance;
  pnh.param<double>("osm_interpolation_max_distance",
                    interpolation_max_distance, 10.0);
  std::cout << "osm max distance : " << interpolation_max_distance << std::endl;
  osm_map->setInterpolationMaxDistance(interpolation_max_distance);

  osm_map->parse();
  // osm_map->publishMapArray();
  ros::Rate loop_rate(0.1);  // 10s重新发布一次

  while (ros::ok()) {
    osm_map->publishMapArray();
    // osm_map->publishMapArrow();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}