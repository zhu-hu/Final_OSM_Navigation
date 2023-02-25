cmake_minimum_required(VERSION 3.5)
project(util)

add_compile_options(-std=c++14)

find_package(absl REQUIRED NO_MODULE)

include_directories(${CMAKE_SOURCE_DIR})

catkin_package()

add_library(string_util STATIC string_util.cc)
target_link_libraries(string_util  absl::strings)
