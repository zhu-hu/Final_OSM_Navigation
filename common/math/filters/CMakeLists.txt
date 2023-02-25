find_package(glog REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})

set (SRCS digital_filter_coefficients.cc
          digital_filter.cc
          mean_filter.cc
)

add_library(filter STATIC ${SRCS})
target_link_libraries(filter glog)
