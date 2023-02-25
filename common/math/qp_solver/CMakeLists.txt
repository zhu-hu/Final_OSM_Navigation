find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}
                    ${EIGEN3_INCLUDE_DIR})

add_library(qp_solver STATIC qp_solver.cc)
