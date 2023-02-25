/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file sin_table.h
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief Exports the SIN_TABLE, used by the Angle class.
 * @version IVFC20
 * @date 2020-09-23
 *
 * @maintainer
 */

#pragma once

/**
 * @namespace cyberc3::common::math
 * @brief cyberc3::common::math
 */
namespace cyberc3 {
namespace common {
namespace math {

//! Used by Angle class to speed-up computation of trigonometric functions.
#define SIN_TABLE_SIZE 16385
extern const float SIN_TABLE[SIN_TABLE_SIZE];

}  // namespace math
}  // namespace common
}  // namespace cyberc3
