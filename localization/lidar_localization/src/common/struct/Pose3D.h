//
// Created by localization on 11/5/19.
//

#ifndef SRC_POSE3D_H
#define SRC_POSE3D_H

namespace localization{
    struct Pose3D{
        double x;
        double y;
        double z;
        double rx;
        double ry;
        double rz;

        Pose3D():x(0.0), y(0.0), z(0.0), rx(0.0), ry(0.0), rz(0.0){};
    };
} // namespace localization

#endif //SRC_POSE3D_H
