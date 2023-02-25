#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <fstream>
#include <limits>

#include "cyber_msgs/LocalizationEstimate.h"

double last_pose_x = std::numeric_limits<double>::max();
double last_pose_y = std::numeric_limits<double>::max();

void WriteFile(const double x, const double y, const double theta,
               double speed) {
  std::ofstream ofile;
  ofile.open("path_demo0.txt", std::ios_base::app);
  ofile << std::setiosflags(std::ios::fixed) << std::setprecision(14);
  if (speed >= 0.0)
    speed = 1.0;
  else
    speed = -1.0;
  ofile << x << " " << y << " " << theta << " " << speed << "\n";
  ofile.close();
}

void PoseCallback(const cyber_msgs::LocalizationEstimateConstPtr& pose_in) {
  double detx = pose_in->pose.position.x - last_pose_x;
  double dety = pose_in->pose.position.y - last_pose_y;
  double speed = pose_in->velocity.linear.x;
  if (std::hypot(detx, dety) > 0.1) {
    last_pose_x = pose_in->pose.position.x;
    last_pose_y = pose_in->pose.position.y;
    WriteFile(last_pose_x, last_pose_y, tf::getYaw(pose_in->pose.orientation),
              speed);
    printf("%lf, %lf\n", last_pose_x, last_pose_y);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_record");
  printf("waiting for topic:/localization/estimation\n");

  ros::NodeHandle nh("~");
  ros::Subscriber sub_pose_ =
      nh.subscribe("/localization/estimation", 5, &PoseCallback);

  ros::spin();
  return 0;
}
