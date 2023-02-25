/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file string_util.cc
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief
 * @version IVFC20
 * @date 2020-09-23
 *
 * @maintainer
 */

#include "common/util/string_util.h"

#include <cmath>
#include <vector>

#include "absl/strings/str_cat.h"

namespace cyberc3 {
namespace common {
namespace util {
namespace {

static const char kBase64Array[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string Base64Piece(const char in0, const char in1, const char in2) {
  const int triplet = in0 << 16 | in1 << 8 | in2;
  std::string out(4, '=');
  out[0] = kBase64Array[(triplet >> 18) & 0x3f];
  out[1] = kBase64Array[(triplet >> 12) & 0x3f];
  if (in1) {
    out[2] = kBase64Array[(triplet >> 6) & 0x3f];
  }
  if (in2) {
    out[3] = kBase64Array[triplet & 0x3f];
  }
  return out;
}

}  // namespace

std::string EncodeBase64(std::string_view in) {
  std::string out;
  if (in.empty()) {
    return out;
  }

  const size_t in_size = in.length();
  out.reserve(((in_size - 1) / 3 + 1) * 4);
  for (size_t i = 0; i + 2 < in_size; i += 3) {
    absl::StrAppend(&out, Base64Piece(in[i], in[i + 1], in[i + 2]));
  }
  if (in_size % 3 == 1) {
    absl::StrAppend(&out, Base64Piece(in[in_size - 1], 0, 0));
  }
  if (in_size % 3 == 2) {
    absl::StrAppend(&out, Base64Piece(in[in_size - 2], in[in_size - 1], 0));
  }
  return out;
}

}  // namespace util
}  // namespace common
}  // namespace cyberc3
