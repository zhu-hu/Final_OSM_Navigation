/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file string_util.h
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief Some string util functions.
 * @version IVFC20
 * @date 2020-09-23
 *
 * @maintainer
 */

#pragma once

#include <string>

#include "absl/strings/str_format.h"
#include "common/util/future.h"

#define FORMAT_TIMESTAMP(timestamp) \
  std::fixed << std::setprecision(9) << timestamp

/**
 * @namespace cyberc3::common::util
 * @brief cyberc3::common::util
 */
namespace cyberc3 {
namespace common {
namespace util {

using absl::StrFormat;

struct DebugStringFormatter {
  template <class T>
  void operator()(std::string* out, const T& t) const {
    out->append(t.DebugString());
  }
};

std::string EncodeBase64(std::string_view in);

}  // namespace util
}  // namespace common
}  // namespace cyberc3
