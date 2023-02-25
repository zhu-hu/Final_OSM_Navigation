//
// Created by localization on 11/5/19.
//

#ifndef SRC_POSE2D_H
#define SRC_POSE2D_H

namespace localization{
    struct Pose2D{
        double x;
        double y;
        double phi;

        Pose2D():x(0.0), y(0.0), phi(0.0){};
    };
} // namespace localization

#endif //SRC_POSE2D_H
