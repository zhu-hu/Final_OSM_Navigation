//
// Created by localization on 11/4/19.
//

#ifndef SRC_QUATERNION_H
#define SRC_QUATERNION_H

namespace localization{
    struct Quaternion{
        double x;
        double y;
        double z;
        double w;

        Quaternion():x(0.0), y(0.0), z(0.0), w(1.0){};
    };
} // namespace localization

#endif //SRC_QUATERNION_H
