//
// Created by Huyao on 12/20/17.
//

#ifndef STATEMACHINE_STBASE_H
#define STATEMACHINE_STBASE_H

#include <string>
#include <iostream>

#include "state/state_common.h"
#include "log.h"
namespace planning
{
enum TransitionEnum
{
    TRANSIT_DEFAULT,
    TRANSIT_TO_GOAL,
    TRANSIT_TO_U_TURN,
    TRANSIT_TO_INTERSECTION,
    TRANSIT_TO_LANECHANGE,
    TRANSIT_TO_STOPLINE,
    TRANSIT_TO_TUNNEL,
    TRANSIT_TO_CROSSWALK,
    TRANSIT_TO_ROADCHECK,
    TRANSIT_TO_PASSENGER,
    TRANSIT_TO_COUNTRYROAD,
    TRANSIT_TO_RAMP
};

template <class DerivedState>
class StBase
{
public:
    StBase(const std::string &n) : name_(n)
    {
        AINFO << "[Stage] Entering " << name();
    };
    virtual ~StBase()
    {
        AINFO << "[Stage] Exiting " << name();
    };
    std::string name() const { return name_; };
    std::string getStateName()
    {
        std::cout << name_ << std::endl;
    };

private:
    std::string name_;
};

} // namespace planning

#endif //STATEMACHINE_STBASE_H
