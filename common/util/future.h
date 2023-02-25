/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file future.h
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief
 * @version IVFC20
 * @date 2020-09-23
 *
 * @maintainer
 */

#pragma once

#if __cplusplus == 201103L || __cplusplus == 201402L
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#endif

#if __cplusplus == 201103L
#include "absl/memory/memory.h"
#include "absl/utility/utility.h"
#endif

namespace std {
// Drop-in replacement for code compliant with future C++ versions.

#if __cplusplus == 201103L || __cplusplus == 201402L

// Borrow from C++ 17 (201703L)
using absl::optional;
using absl::string_view;

#endif

#if __cplusplus == 201103L
// Borrow from C++ 14.
using absl::make_integer_sequence;
using absl::make_unique;
#endif

}  // namespace std
