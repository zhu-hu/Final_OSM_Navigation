#ifndef COMMON_UTILS_POINT_TYPES_H_
#ifndef COMMON_UTILS_POINT_TYPES_H_

#include <pcl/point_types.h>

namespace pcl
{
struct PointXYZIR
{
  PCL_ADD_POINT4D;                    // preferred way of adding a XYZ+padding
  uint8_t intensity;                  // point intensity 
  uint8_t ring;                       // laser ring 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
} EIGEN_ALIGN16;                      // enforce SSE padding for correct memory alignment
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint8_t, intensity, intensity)
                                  (uint8_t, ring, ring))

#endif  // COMMON_UTILS_POINT_TYPES_H_
