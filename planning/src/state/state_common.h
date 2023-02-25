//
// Created by cybernp on 12/27/17.
//

#ifndef STATEMACHINE_STATE_COMMON_H
#define STATEMACHINE_STATE_COMMON_H

#include <boost/mpl/list.hpp>
#include <boost/pool/pool_alloc.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/deep_history.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/exception_translator.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/transition.hpp>
#include <map>

#include "events.hpp"

namespace mpl = boost::mpl;
namespace sc = boost::statechart;

#endif  // STATEMACHINE_STATE_COMMON_H
