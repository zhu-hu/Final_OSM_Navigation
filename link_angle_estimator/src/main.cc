#include "link_angle_estimator.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "link_angle_estimator");
  ros::NodeHandle pnh("~");
  LinkAngleEstimator estimator(pnh);
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
  return 0;
}