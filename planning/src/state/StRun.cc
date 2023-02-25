//
// Created by ShenQiyue on 19-11-10
//
#include "StRun.h"

namespace planning {
StRun::StRun(my_context ctx)
    : my_base(ctx), StBase<StRun>(std::string("StRun")) {}
StRun::~StRun() {}
sc::result StRun::react(const EvSysTick &evt) {
  Stager &stager = context<Stager>();
  const auto &grid_map_manager = stager.grid_map_manager_;
  const auto &grid_map = grid_map_manager->grid_map();
  const auto &self_state_manager = stager.self_state_;
  const auto &adc_state = self_state_manager->GetLocalizationEstimation();
  const auto param = stager.params_;

  emergency_detect(grid_map, _emergency_mode, _emergency_time_cnt,
                   _close_time_cnt, param);
  bool is_localization_healthy =
      (adc_state.status) &&
      self_state_manager->GetLocalizationValidity();  //正常为1,异常为0
  auto publisher = stager.publisher_manager_;

  //接收到紧急停车指令
  if (self_state_manager->GetEmergencyCommand()) {
    _emergency_mode = 2;
  }

  if (_emergency_mode == 0 && is_localization_healthy) {
    publisher->PublishText("No Emergency", 0);
    stager.CancelEmergency();
    self_state_manager->PublishEmergencyMode(0);
  } else if (_emergency_mode == 1) {
    stager.SetEmergencyMode(1);
    publisher->PublishText("Emergency Speed Down!!", 1);
    self_state_manager->PublishEmergencyMode(1);
  } else if (_emergency_mode == 2) {
    stager.SetEmergencyMode(2);
    publisher->PublishText("Emergency Stop!!", 1);
    self_state_manager->PublishEmergencyMode(2);
  } else if (!is_localization_healthy) {
    stager.SetEmergencyMode(2);
    // publisher->PublishText("Localization Error!!", 1);
    self_state_manager->PublishEmergencyMode(3);
  }
  return forward_event();
}
sc::result StRun::react(const sc::exception_thrown &evt) {
  return forward_event();
}
}  // namespace planning