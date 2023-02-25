#ifndef STNAVIGATION_H
#define STNAVIGATION_H

#include "../planner/dubins_curve.h"
#include "planning.h"
#include "state/StBase.h"
#include "state/StRun.h"

namespace planning {
class StNavigation : public sc::state<StNavigation, StRun>,
                     public StBase<StNavigation> {
 public:
  StNavigation(my_context ctx);
  ~StNavigation();

  sc::result react(const EvSysTick &evt);

  sc::result react(const sc::exception_thrown &evt);
  typedef mpl::list<sc::custom_reaction<EvSysTick>,
                    sc::custom_reaction<sc::exception_thrown>>
      reactions;

 private:
  hybrid_a_star::DubinsCurve dubins_curve_;
  //将自车坐标系下的(x, y)转换成局部栅格图里的(pix_x, pix_y),通过(pix_x,
  // pix_y)即可访问局部栅格图里对应的位置
  inline void TFXY2PixInLocalMap(const double x, const double y, int &pix_x,
                                 int &pix_y) {
    pix_x = 300 - (int)(x * 10.0);
    pix_y = 100 - (int)(y * 10.0);
  }

  //将局部栅格图里的(pix_x, pix_y)转换成自车坐标系下的(x,y)
  inline void TFPix2XYInLocalMap(const int pix_x, const int pix_y, double &x,
                                 double &y) {
    x = 30.0 - pix_x * 0.1;
    y = 10.0 - pix_y * 0.1;
  }
};
}  // namespace planning

#endif