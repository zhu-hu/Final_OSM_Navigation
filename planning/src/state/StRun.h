//
// Created by ShenQiyue on 19-11-10
//

#ifndef STRUN_H
#define STRUN_H

#include "planning.h"
#include "state/StBase.h"
#include "state/StPause.h"
#include "common/utlis/emergency_detection.h"
#include "common/utlis/optimal_reference_line.h"

namespace planning
{
class StStart;
class StRun : public sc::state<StRun, Stager, StStart>, public StBase<StRun>
{
public:
    StRun(my_context ctx);
    ~StRun();
    sc::result react(const EvSysTick &evt);
    sc::result react(const sc::exception_thrown &evt);
    typedef mpl::list<
        sc::custom_reaction<EvSysTick>,
        sc::custom_reaction<sc::exception_thrown>,
        sc::transition<EvPause, StPause>>
        reactions;

private:
    uint32_t _emergency_mode;
    int _emergency_time_cnt = 0;
    int _close_time_cnt = 0;
};
} // namespace planning

#endif