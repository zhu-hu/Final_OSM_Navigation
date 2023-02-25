//
// Created by localization on 11/5/19.
//

#ifndef SRC_DEAD_RECKONING_2D_H
#define SRC_DEAD_RECKONING_2D_H

#include <list>

#include "common/struct/Pose2DStamped.h"
#include "common/struct/TwistStamped.h"
#include "common/util/math_util.h"

namespace localization{
    class DeadReckoning2D{
    public:
        DeadReckoning2D();

        void AddLocalOdom(TwistStamped twi);

        void UpdatePose(Pose2DStamped *&src_pose,
                                         double current_timestamp);

    private:
        std::list<localization::TwistStamped> twist_list_;
        int list_max_length_;
    };
} // namespace localization

#endif //SRC_DEAD_RECKONING_2D_H
