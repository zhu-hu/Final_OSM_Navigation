//
// Created by localization on 11/5/19.
//

#include "dead_reckoning_2d.h"

namespace localization {

    DeadReckoning2D::DeadReckoning2D() {
        list_max_length_ = 100;
    }

    void DeadReckoning2D::AddLocalOdom(localization::TwistStamped twi) {
        twist_list_.push_back(twi);
        while (twist_list_.size() > list_max_length_) {
            twist_list_.pop_front();
        }
    }

    void DeadReckoning2D::UpdatePose(Pose2DStamped *&src_pose,
                                     double current_timestamp) {
        std::list<localization::TwistStamped> twist_list_copy(twist_list_);
        while (!twist_list_copy.empty() &&
               src_pose->timestamp - twist_list_copy.front().timestamp > 0) {
            twist_list_copy.pop_front();
        }
        while (twist_list_copy.size() > 1 &&
               current_timestamp - twist_list_copy.front().timestamp > 0) {
            double last_timestamp = twist_list_copy.front().timestamp;
            twist_list_copy.pop_front();
            double time_diff = twist_list_copy.front().timestamp - last_timestamp;
            src_pose->pose.x += twist_list_copy.front().twist.linear.x * time_diff;
            src_pose->pose.y += twist_list_copy.front().twist.linear.y * time_diff;
        }
        src_pose->timestamp = current_timestamp;

    }


}
