/*
 * @Author: Liuyaqi99 283402121@qq.com
 * @Date: 2022-06-28 15:34:57
 * @LastEditors: Liuyaqi99 283402121@qq.com
 * @LastEditTime: 2022-06-28 15:44:25
 * @FilePath: /free_space_detect/src/livox_free_space/include/FreeSpace.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _FREESPACE_HPP
#define _FREESPACE_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <math.h>
#include <vector>
namespace perception {
using namespace std;

#define SELF_CALI_FRAMES 20

#define GND_IMG_NX 150
#define GND_IMG_NY 400
#define GND_IMG_DX 0.2
#define GND_IMG_DY 0.2
#define GND_IMG_OFFX 40
#define GND_IMG_OFFY 40

#define GND_IMG_NX1 24
#define GND_IMG_NY1 20
#define GND_IMG_DX1 4
#define GND_IMG_DY1 4
#define GND_IMG_OFFX1 40
#define GND_IMG_OFFY1 40

#define DN_SAMPLE_IMG_NX 500
#define DN_SAMPLE_IMG_NY 200
#define DN_SAMPLE_IMG_NZ 100
#define DN_SAMPLE_IMG_DX 0.4
#define DN_SAMPLE_IMG_DY 0.4
#define DN_SAMPLE_IMG_DZ 0.2
#define DN_SAMPLE_IMG_OFFX 50 // -50～150
#define DN_SAMPLE_IMG_OFFY 40 // -40～40
#define DN_SAMPLE_IMG_OFFZ 10 // -10～10

#define DENOISE_IMG_NX 200
#define DENOISE_IMG_NY 80
#define DENOISE_IMG_NZ 100
#define DENOISE_IMG_DX 1
#define DENOISE_IMG_DY 1
#define DENOISE_IMG_DZ 0.2

#define FREE_PI 3.14159265

class LivoxFreeSpace {
public:
  LivoxFreeSpace();
  // functions
  int GenerateFreeSpace(float *fPoints1, int pointNum,
                        std::vector<float> &free_space);
  void FreeSpace(float *fPoints, int n, float *free_space, int free_space_n);
  void FreeSpaceFilter(float *free_space_small, int n,
                       std::vector<float> &free_space);
  int GroundSegment(int *pLabel, float *fPoints, int pointNum,
                    float fSearchRadius);

  unsigned char *pVImg;

  ~LivoxFreeSpace();
};
} // namespace perception
#endif
