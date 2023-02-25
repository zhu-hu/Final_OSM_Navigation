#include "pplqr_back_controller.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pplqr_back_controller");
  ros::NodeHandle pnh("~");
  controller::PPLqrBackController controller_obj(&pnh);

  return 0;
}