//
// Created by Huyao on 12/19/17.
//

#include "state/StPause.h"

namespace planning
{

StPause::StPause(my_context ctx) : my_base(ctx), StBase<StPause>(std::string("StPause"))
{
}

StPause::~StPause()
{
}

sc::result StPause::react(const EvSysTick &evt)
{
    //停车状态
    return forward_event();
}

sc::result StPause::react(const sc::exception_thrown &evt)
{
    return forward_event();
}
} // namespace planning