/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file vehicle_param.h
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief
 * @version IVFC20
 * @date 2020-11-03
 *
 * @maintainer
 */
#pragma once

namespace vehicle_param {
//牵引车宽度
constexpr double kWidth = 1.56;
//牵引车总长度
constexpr double kLength = 2.53;
//牵引车后轴中心到后平面的距离 = K1
constexpr double kRearToBack = 0.33;
//牵引车转弯半径，打死的转弯半径是４.3米
constexpr double kTurningRadius = 4.3;
//牵引车轴距，和L1一样
constexpr double kWheelBase = 1.62;

//牵引车后轴中心到牵引车后平面的距离
constexpr double K1 = 0.33;
//牵引车前轴中心到牵引车前平面的距离
constexpr double K2 = 0.58;
//拖挂车前轴中心到连接点处的距离
constexpr double K3 = 0.50;
//拖挂车后轴中心到拖挂车后平面的距离
constexpr double K4 = 0.14;

//牵引车后轴中心到拖挂连接处的距离
constexpr double M1 = 0.57;
//连接处到拖挂车后轴中心的距离
constexpr double L2 = 2.04;
//拖挂车的宽度
constexpr double W2 = 0.90;
//连接处到拖挂车前平面的距离
constexpr double ForwardLength = 0.5;
//拖挂车车体的长度 = L2 + K4 - ForwardLength
constexpr double Length2 = 1.64;

//栅格图的尺寸相关
constexpr double kMinX = -10.0;
constexpr double kMaxX = 30.0;
constexpr double kMinY = -10.0;
constexpr double kMaxY = 10.0;
constexpr int PixelScale = 10;
}  // namespace vehicle_param
