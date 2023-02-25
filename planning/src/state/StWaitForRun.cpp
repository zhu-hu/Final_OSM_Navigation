//
// Created by luyifan on 18-1-10.
//

#include "state/StWaitForRun.h"

#include "state/StRun.h"
#include "state/StStart.h"

namespace planning {
StWaitForRun::StWaitForRun(my_context ctx)
    : my_base(ctx), StBase<StWaitForRun>(std::string("StWaitForRun")) {
  // tWaiting_for_run.sec = tWaiting_for_run.now().sec;
}
StWaitForRun::~StWaitForRun() {}
sc::result StWaitForRun::react(const EvSysTick &evt) {
  //获取当前自身状态
  Stager &stager = context<Stager>();
  //生成发布器
  auto publisher = stager.publisher_manager_;

  publisher->PublishStationStatus(1);
  return transit<StRun>();
}
sc::result StWaitForRun::react(const sc::exception_thrown &evt) {
  return forward_event();
}
}  // namespace planning
