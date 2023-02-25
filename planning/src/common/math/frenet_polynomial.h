#ifndef FRENET_QUINTIC_POLYNOMIAL_H
#define FRENET_QUINTIC_POLYNOMIAL_H

#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

class QuinticPolynomial
{
private:
    double _a0;
    double _a1;
    double _a2;
    double _a3;
    double _a4;
    double _a5;

public:
    inline QuinticPolynomial(const double &p_start_, const double &v_start_, const double &a_start_, const double &p_end_, const double &v_end_, const double &a_end_, const double &time_)
    {
        _a0 = p_start_;
        _a1 = v_start_;
        _a2 = a_start_ / 2.0;
        MatrixXd A(3, 3);
        MatrixXd B(3, 1);
        MatrixXd X(3, 1);
        A << time_ * time_ * time_, time_ * time_ * time_ * time_, time_ * time_ * time_ * time_ * time_,
            3 * time_ * time_, 4 * time_ * time_ * time_, 5 * time_ * time_ * time_ * time_,
            6 * time_, 12 * time_ * time_, 20 * time_ * time_ * time_;
        B << p_end_ - _a0 - _a1 * time_ - _a2 * time_ * time_, v_end_ - _a1 - 2 * _a2 * time_, a_end_ - 2 * _a2;
        X = A.householderQr().solve(B);
        _a3 = X.data()[0];
        _a4 = X.data()[1];
        _a5 = X.data()[2];
    }
    ~QuinticPolynomial() = default;
    inline double CalcPoint(const double &t_)
    {
        return _a0 + _a1 * t_ + _a2 * pow(t_, 2) + _a3 * pow(t_, 3) + _a4 * pow(t_, 4) + _a5 * pow(t_, 5);
    }
    inline double CalcFirstDerivative(const double &t_)
    {
        return _a1 + 2 * _a2 * t_ + 3 * _a3 * pow(t_, 2) + 4 * _a4 * pow(t_, 3) + 5 * _a5 * pow(t_, 4);
    }
    inline double CalcSecondDerivative(const double &t_)
    {
        return 2 * _a2 + 6 * _a3 * t_ + 12 * _a4 * pow(t_, 2) + 20 * _a5 * pow(t_, 3);
    }
    inline double CalcThirdDerivative(const double &t_)
    {
        return 6 * _a3 + 24 * _a4 * t_ + 60 * _a5 * t_ * t_;
    }
};

class QuarticPolynomial
{
private:
    double _a0;
    double _a1;
    double _a2;
    double _a3;
    double _a4;

public:
    inline QuarticPolynomial(const double &p_start_, const double &v_start_, const double &a_start_, const double &v_end_, const double &a_end_, const double &time_) : _a0(p_start_), _a1(v_start_), _a2(a_start_ / 2.0)
    {
        MatrixXd A(2, 2);
        MatrixXd B(2, 1);
        MatrixXd X(2,1);
        A << 3 * time_ * time_, 4 * time_ * time_ * time_, 6 * time_, 12 * time_ * time_;
        B << v_end_ - _a1 - 2 * _a2 * time_, a_end_ - 2 * _a2;
        X = A.householderQr().solve(B);
        _a3 = X.data()[0];
        _a4 = X.data()[1];
    }
    ~QuarticPolynomial() = default;
    inline double CalcPoint(const double &t_)
    {
        return _a0 + _a1 * t_ + _a2 * pow(t_, 2) + _a3 * pow(t_, 3) + _a4 * pow(t_, 4) ;
    }
    inline double CalcFirstDerivative(const double &t_)
    {
        return _a1 + 2 * _a2 * t_ + 3 * _a3 * pow(t_, 2) + 4 * _a4 * pow(t_, 3) ;
    }
    inline double CalcSecondDerivative(const double &t_)
    {
        return 2 * _a2 + 6 * _a3 * t_ + 12 * _a4 * pow(t_, 2);
    }
    inline double CalcThirdDerivative(const double &t_)
    {
        return 6 * _a3 + 24 * _a4 * t_;
    }
};

#endif
