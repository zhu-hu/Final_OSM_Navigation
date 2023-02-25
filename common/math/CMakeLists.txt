cmake_minimum_required(VERSION 3.5)
project(math)

add_compile_options(-std=c++14)

find_package(absl REQUIRED NO_MODULE)
find_package(Eigen3 REQUIRED)

find_package(glog REQUIRED NO_MODULE)

include_directories(
  ${CMAKE_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(math_utils STATIC math_utils.cc)
target_link_libraries(math_utils  vec2d)

add_library(vec2d STATIC vec2d.cc)
target_link_libraries(vec2d absl::strings glog)

add_library(pose2d STATIC pose2d.cc)
target_link_libraries(pose2d  vec2d absl::strings glog)

add_library(geometry STATIC aabox2d.cc box2d.cc line_segment2d.cc polygon2d.cc)
target_link_libraries(geometry  math_utils string_util absl::strings glog)

add_library(sin_table STATIC sin_table.cc)

add_library(angle STATIC angle.cc)
target_link_libraries(angle sin_table)

add_library(matrix_operations STATIC matrix_operations.cc)
target_link_libraries(matrix_operations glog)

add_library(path_matcher STATIC path_matcher.cc)
target_link_libraries(path_matcher linear_interpolation)

add_library(search STATIC search.cc)

add_library(linear_interpolation STATIC linear_interpolation.cc)
target_link_libraries(linear_interpolation  glog math_utils)

add_library(integral STATIC integral.cc)
target_link_libraries(integral glog)

add_library(lqr STATIC linear_quadratic_regulator.cc)
target_link_libraries(lqr glog)

add_library(cartesian_frenet_conversion  STATIC cartesian_frenet_conversion.cc)
target_link_libraries(cartesian_frenet_conversion vec2d glog math_utils)

# set (SRCS filters/digital_filter_coefficients.cc
#           filters/digital_filter.cc
#           filters/mean_filter.cc
# )

# add_library(filter STATIC ${SRCS})
# target_link_libraries(filter glog)
