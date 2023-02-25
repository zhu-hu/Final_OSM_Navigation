//
// Created by Huyao on 12/19/17.
//

#ifndef STATEMACHINE_STPAUSE_H
#define STATEMACHINE_STPAUSE_H

#include "planning.h"
#include "state/StBase.h"
#include "state/StWaitForRun.h"

namespace planning {

    class StPause : public sc::state<StPause, Stager>, public StBase<StPause> {
    public:
        StPause(my_context ctx);
        virtual ~StPause();
        sc::result react(const EvSysTick &evt);
        sc::result react(const sc::exception_thrown &evt);
        typedef mpl::list
        <
            sc::custom_reaction<EvSysTick>,
            sc::transition<EvActivate, StWaitForRun>,
            sc::custom_reaction<sc::exception_thrown>
        > reactions;

    };

}// namespace planning

#endif //STATEMACHINE_STPAUSE_H
