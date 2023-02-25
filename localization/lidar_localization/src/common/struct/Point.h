//
// Created by localization on 11/4/19.
//

#ifndef SRC_POINT_H
#define SRC_POINT_H

namespace localization{
    struct Point{
        double x;
        double y;
        double z;

        Point():x(0.0), y(0.0), z(0.0){};
        Point(double a,double b,double c):x(a), y(b), z(c){};
    };
} // namespace localization

#endif //SRC_POINT_H
