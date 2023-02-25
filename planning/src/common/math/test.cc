#include "frenet_polynomial.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test");
    QuinticPolynomial lat_qp(2.0, 0.0, 0.0, 4.0, 0.0, 0.0, 1.0);
    QuarticPolynomial lon_qp(0.0,10,0.0,20,0.0,5);
    double t = 0.1;
    double res = 0.0;
    res = lon_qp.CalcPoint(t); //n*1;
    std::cout << res << std::endl;
    std::cout << "----" << std::endl;
    res = lon_qp.CalcFirstDerivative(t); //n*1;
    std::cout << res << std::endl;
    std::cout << "----" << std::endl;
    res = lon_qp.CalcSecondDerivative(t); //n*1;
    std::cout << res << std::endl;
    std::cout << "----" << std::endl;
    res = lon_qp.CalcThirdDerivative(t); //n*1;
    std::cout << res << std::endl;
    std::cout << "----" << std::endl;
    return 0;
}