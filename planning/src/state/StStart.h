//
// Created by ShenQiyue on 19-11-12
//

#ifndef STSTART_H
#define STSTART_H

#include "planning.h"
#include "state/StBase.h"
#include "state/StRun.h"
#include "state/StPause.h"

namespace planning
{
class StStart : public sc::state<StStart, StRun>, public StBase<StStart>
{
public:
    StStart(my_context ctx);
    ~StStart();
    sc::result react(const EvSysTick &evt);
    sc::result react(const sc::exception_thrown &evt);
    typedef mpl::list<
        sc::custom_reaction<EvSysTick>,
        sc::custom_reaction<sc::exception_thrown>>
        reactions;
};
} // namespace planning

#endif