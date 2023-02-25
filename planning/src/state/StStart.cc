//
// Created by ShenQiyue on 19-11-12
//
#include "StStart.h"

#include "StNavigation.h"
#include "StRun.h"

namespace planning {
StStart::StStart(my_context ctx)
    : my_base(ctx), StBase<StStart>(std::string("StStart")) {}
StStart::~StStart() {}
sc::result StStart::react(const EvSysTick &evt) {
  //获取当前自身状态
  Stager &stager = context<Stager>();
  const auto &self_state_manager = stager.self_state_;
  const auto &adc_state = self_state_manager->GetLocalizationEstimation();

  const auto &osm_route_manager = stager.osm_route_manager_;

  if (osm_route_manager->TargetIdUpdated()) {
    return transit<StNavigation>();
  }

  return forward_event();
}
sc::result StStart::react(const sc::exception_thrown &evt) {
  return forward_event();
}
}  // namespace planning