#include "common/util/util.h"

#include <cmath>
#include <vector>

namespace cyberc3 {
namespace common {
namespace util {

PathPoint GetWeightedAverageOfTwoPathPoints(const PathPoint& p1,
                                            const PathPoint& p2,
                                            const double w1, const double w2) {
  PathPoint p;
  p.set_x(p1.x() * w1 + p2.x() * w2);
  p.set_y(p1.y() * w1 + p2.y() * w2);
  p.set_z(p1.z() * w1 + p2.z() * w2);
  p.set_theta(p1.theta() * w1 + p2.theta() * w2);
  p.set_kappa(p1.kappa() * w1 + p2.kappa() * w2);
  p.set_dkappa(p1.dkappa() * w1 + p2.dkappa() * w2);
  p.set_ddkappa(p1.ddkappa() * w1 + p2.ddkappa() * w2);
  p.set_s(p1.s() * w1 + p2.s() * w2);
  return p;
}

}  // namespace util
}  // namespace common
}  // namespace cyberc3
