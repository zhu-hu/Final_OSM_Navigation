//
// Created by huyao on 18-1-15.
//
#include "planning.h"
#include "state/StPause.h"
// #include "state/StRun.h"
// #include "state/StStop.h"
#include "parameter/tiggo_model.h"
#include "ros/ros.h"

// USB Key，只在生成发布版的时候才解注释
// #include "../../../common/Pwd_8/SoftkeyPWD.h"

int main(int argc, char **argv) {
  // 只在生成发布版的时候才解注释，否则在自己电脑上无法调试
  // if(!checkUSBKey()) return 0;

  ros::init(argc, argv, "StateMachine");
  ros::NodeHandle pnh("~");
  planning::Model model_param(&pnh);
  planning::Stager *stager = planning::Stager::instance(&model_param);

  stager->initiate();

  ros::Rate loop_rate(30);  // 30hz的循环频率

  //等待初始化
  std::cout << "waiting to get current pose..." << std::endl;
  std::cout << "waiting to get grid map..." << std::endl;
  while (((stager->grid_map_manager_->grid_map().empty()) ||
          (stager->self_state_->GetLocalizationEstimation().pose.position.x ==
               MY_INF ||
           stager->self_state_->GetLocalizationEstimation().pose.position.y ==
               MY_INF)) &&
         ros::ok()) {
    if (stager->grid_map_manager_->grid_map().empty())
      stager->publisher_manager_->PublishText("grid map empty!!!", 1);
    bool not_localizaiton_avaliable =
        stager->self_state_->GetLocalizationEstimation().pose.position.x ==
            MY_INF ||
        stager->self_state_->GetLocalizationEstimation().pose.position.y ==
            MY_INF;
    if (not_localizaiton_avaliable)
      stager->publisher_manager_->PublishText("Localization unavaliable!!!", 1);
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "get current pose..." << std::endl;
  std::cout << "get grid map..." << std::endl;

  stager->process_event(planning::EvActivate());
  while (ros::ok()) {
    stager->process_event(planning::EvSysTick());
    ros::spinOnce();
    loop_rate.sleep();  // sleep 0.0333s
  }

  return 0;
}
