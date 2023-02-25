#ifndef __EVENTS_HPP__
#define __EVENTS_HPP__

#include <boost/statechart/event.hpp>

namespace sc = boost::statechart;

namespace planning {

class EvStop : public sc::event<EvStop> {};

class EvDrive : public sc::event<EvDrive> {};

class EvPause : public sc::event<EvPause> {};

class EvActivate : public sc::event<EvActivate> {};

class EvSysTick : public sc::event<EvSysTick> {};

class EvTemplate : public sc::event<EvTemplate> {};

}  // namespace planning

#endif
