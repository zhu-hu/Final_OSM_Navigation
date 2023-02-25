//
// Created by luyifan on 18-1-10.
//

#ifndef STATEMACHINE_STWAITFORRUN_H
#define STATEMACHINE_STWAITFORRUN_H

#include "planning.h"
#include "state/StBase.h"

namespace planning
{
class StWaitForRun : public sc::state<StWaitForRun, Stager>, public StBase<StWaitForRun>
{
public:
    StWaitForRun(my_context ctx);
    ~StWaitForRun();
    sc::result react(const EvSysTick &evt);
    sc::result react(const sc::exception_thrown &evt);
    typedef mpl::list<
        sc::custom_reaction<EvSysTick>,
        sc::custom_reaction<sc::exception_thrown>>
        reactions;
    ros::Time tWaiting_for_run;
};
} // namespace planning

#endif //STATEMACHINE_STWAITFORRUN_H
