//
// Created by luyifan on 19-11-6.
//

#ifndef PLANNING_JERK_MINI_TRAJECTORY_H
#define PLANNING_JERK_MINI_TRAJECTORY_H

#include "../math/quintic_polynomial.h"

namespace planning{
  class JMTQuintic{
  public:
    JMTQuintic(vector<double> start, vector<double> end, double T){
      //获取5次多项式系数
      coeffs_ = quintic_polynomial(start, end, T);
      //生成轨迹点
      vals_.clear();
      sum_jerk_ = 0;
      for (int i = 0; i < (int)(10 * T); i++){
        double t = 0.1 * i;
        double t2 = t*t;
        double t3 = t2*t;
        double t4 = t3*t;
        double t5 = t4*t;
        double val = coeffs_[0] + coeffs_[1] * t + coeffs_[2] * t2 +
                     coeffs_[3] * t3 + coeffs_[4] * t4 + coeffs_[5] * t5;
        vals_.emplace_back(val);

        double ddd = CalcThirdDerivative(t);
        sum_jerk_ += ddd * ddd;
      }
      sum_jerk_ /= (10 * T);
    }

    double CalcThirdDerivative(double t){
      return 6 * coeffs_[3] + 24 * coeffs_[4] * t + 60 * coeffs_[5]* t * t;
    }

  public:
    inline const vector<double>& vals(){ return vals_; }
    inline const double sum_jerk(){ return sum_jerk_; }
  private:
    vector<double> coeffs_;
    vector<double> vals_;
    double sum_jerk_;
  };

  class JMTQuartic{
  public:
    JMTQuartic(vector<double> start, vector<double> end, double T){
      //获取4次多项式系数
      coeffs_ = quartic_polynomial(start, end, T);
      //生成轨迹点
      vals_.clear();
      sum_jerk_ = 0;
      for (int i = 0; i < (int)(10 * T); i++){
        double t = 0.1 * i;
        double t2 = t*t;
        double t3 = t2*t;
        double t4 = t3*t;
        double val = coeffs_[0] + coeffs_[1] * t + coeffs_[2] * t2 +
                     coeffs_[3] * t3 + coeffs_[4] * t4;
        vals_.emplace_back(val);

        double ddd = CalcThirdDerivative(t);
        sum_jerk_ += ddd * ddd;
      }
      sum_jerk_ /= (10 * T);
    }

    double CalcThirdDerivative(double t){
      return 6 * coeffs_[3] + 24 * coeffs_[4] * t;
    }

  public:
    inline const vector<double>& vals(){ return vals_; }
    inline const double sum_jerk(){ return sum_jerk_; }
  private:
    vector<double> coeffs_;
    vector<double> vals_;
    double sum_jerk_;
  };
}

#endif //PLANNING_JERK_MINI_TRAJECTORY_H
