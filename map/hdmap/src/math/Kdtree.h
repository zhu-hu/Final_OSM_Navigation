#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include "common/utils/log.h"

#include "AABox2d.h"
#include "utlis.h"

namespace common{
namespace math{

/**
 * @class AABoxKDTreeParams
 * @brief Contains parameters of axis-aligned bounding box.
 */
struct AABoxKDTreeParams {
  /// The maximum depth of the kdtree.
  int max_depth = -1;
  /// The maximum number of items in one leaf node.
  int max_leaf_size = -1;
  /// The maximum dimension size of leaf node.
  double max_leaf_dimension = -1.0;
};

/**
 * @class AABoxKDTree2dNode
 * @brief The class of KD-tree node of axis-aligned bounding box.
 */
template <class ObjectType>
class AABoxKDTree2dNode {
 public:
  using ObjectPtr = const ObjectType *;
  /**
   * @brief Constructor which takes a vector of objects,
   *        parameters and depth of the node.
   * @param objects Objects to build the KD-tree node.
   * @param params Parameters to build the KD-tree.
   * @param depth Depth of the KD-tree node.
   */
  AABoxKDTree2dNode(const std::vector<ObjectPtr> &objects,
                    const AABoxKDTreeParams &params, int depth)
      : depth_(depth) {
    CHECK(!objects.empty());

    ComputeBoundary(objects);//计算objects的边界
    ComputePartition();//计算分隔点，进行2叉
    //如果还能2叉
    if (SplitToSubNodes(objects, params)) {
      std::vector<ObjectPtr> left_subnode_objects;
      std::vector<ObjectPtr> right_subnode_objects;
      PartitionObjects(objects, &left_subnode_objects, &right_subnode_objects);

      // 继续对左右子树进行二叉
      if (!left_subnode_objects.empty()) {
        left_subnode_.reset(new AABoxKDTree2dNode<ObjectType>(
            left_subnode_objects, params, depth + 1));
      }
      if (!right_subnode_objects.empty()) {
        right_subnode_.reset(new AABoxKDTree2dNode<ObjectType>(
            right_subnode_objects, params, depth + 1));
      }
    } else {
      InitObjects(objects);
    }
  }

  /**
   * @brief Get the nearest object to a target point by the KD-tree
   *        rooted at this node.
   * @param point The target point. Search it's nearest object.
   * @return The nearest object to the target point.
   */
  ObjectPtr GetNearestObject(const Vec2d &point) const {
    ObjectPtr nearest_object = nullptr;
    double min_distance_sqr = std::numeric_limits<double>::infinity();
    GetNearestObjectInternal(point, &min_distance_sqr, &nearest_object);
    return nearest_object;
  }

  /**
   * @brief Get objects within a distance to a point by the KD-tree
   *        rooted at this node.
   * @param point The center point of the range to search objects.
   * @param distance The radius of the range to search objects.
   * @return All objects within the specified distance to the specified point.
   */
  std::vector<ObjectPtr> GetObjects(const Vec2d &point,
                                    const double distance) const {
    std::vector<ObjectPtr> result_objects;
    GetObjectsInternal(point, distance, Square(distance), &result_objects);
    return result_objects;
  }

  /**
   * @brief Get the axis-aligned bounding box of the objects.
   * @return The axis-aligned bounding box of the objects.
   */
  AABox2d GetBoundingBox() const {
    return AABox2d({min_x_, min_y_}, {max_x_, max_y_});
  }

 private:
  void InitObjects(const std::vector<ObjectPtr> &objects) {
    num_objects_ = static_cast<int>(objects.size());
    objects_sorted_by_min_ = objects;
    objects_sorted_by_max_ = objects;
    //按照当前的2叉参考轴对元素从两方向进行排序
    std::sort(objects_sorted_by_min_.begin(), objects_sorted_by_min_.end(),
              [&](ObjectPtr obj1, ObjectPtr obj2) {
                return partition_ == PARTITION_X
                           ? obj1->aabox().min_x() < obj2->aabox().min_x()
                           : obj1->aabox().min_y() < obj2->aabox().min_y();
              });
    std::sort(objects_sorted_by_max_.begin(), objects_sorted_by_max_.end(),
              [&](ObjectPtr obj1, ObjectPtr obj2) {
                return partition_ == PARTITION_X
                           ? obj1->aabox().max_x() > obj2->aabox().max_x()
                           : obj1->aabox().max_y() > obj2->aabox().max_y();
              });
    //按照排序把元素用来排序的参考值pushback到vector中
    objects_sorted_by_min_bound_.reserve(num_objects_);
    for (ObjectPtr object : objects_sorted_by_min_) {
      objects_sorted_by_min_bound_.push_back(partition_ == PARTITION_X
                                                 ? object->aabox().min_x()
                                                 : object->aabox().min_y());
    }
    objects_sorted_by_max_bound_.reserve(num_objects_);
    for (ObjectPtr object : objects_sorted_by_max_) {
      objects_sorted_by_max_bound_.push_back(partition_ == PARTITION_X
                                                 ? object->aabox().max_x()
                                                 : object->aabox().max_y());
    }
  }

  /************
   * 判断是否能进行2叉操作
   * @param objects
   * @param params
   * @return
   */
  bool SplitToSubNodes(const std::vector<ObjectPtr> &objects,
                       const AABoxKDTreeParams &params) {
    if (params.max_depth >= 0 && depth_ >= params.max_depth) {
      return false;
    }
    if (static_cast<int>(objects.size()) <= std::max(1, params.max_leaf_size)) {
      return false;
    }
    if (params.max_leaf_dimension >= 0.0 &&
        std::max(max_x_ - min_x_, max_y_ - min_y_) <=
            params.max_leaf_dimension) {
      return false;
    }
    return true;
  }

  double LowerDistanceSquareToPoint(const Vec2d &point) const {
    double dx = 0.0;
    if (point.x() < min_x_) {
      dx = min_x_ - point.x();
    } else if (point.x() > max_x_) {
      dx = point.x() - max_x_;
    }
    double dy = 0.0;
    if (point.y() < min_y_) {
      dy = min_y_ - point.y();
    } else if (point.y() > max_y_) {
      dy = point.y() - max_y_;
    }
    return dx * dx + dy * dy;
  }

  double UpperDistanceSquareToPoint(const Vec2d &point) const {
    const double dx =
        (point.x() > mid_x_ ? (point.x() - min_x_) : (point.x() - max_x_));
    const double dy =
        (point.y() > mid_y_ ? (point.y() - min_y_) : (point.y() - max_y_));
    return dx * dx + dy * dy;
  }

  void GetAllObjects(std::vector<ObjectPtr> *const result_objects) const {
    result_objects->insert(result_objects->end(),
                           objects_sorted_by_min_.begin(),
                           objects_sorted_by_min_.end());
    if (left_subnode_ != nullptr) {
      left_subnode_->GetAllObjects(result_objects);
    }
    if (right_subnode_ != nullptr) {
      right_subnode_->GetAllObjects(result_objects);
    }
  }

  void GetObjectsInternal(const Vec2d &point, const double distance,
                          const double distance_sqr,
                          std::vector<ObjectPtr> *const result_objects) const {
    if (LowerDistanceSquareToPoint(point) > distance_sqr) {
      return;
    }
    if (UpperDistanceSquareToPoint(point) <= distance_sqr) {
      GetAllObjects(result_objects);
      return;
    }
    const double pvalue = (partition_ == PARTITION_X ? point.x() : point.y());
    if (pvalue < partition_position_) {
      const double limit = pvalue + distance;
      for (int i = 0; i < num_objects_; ++i) {
        if (objects_sorted_by_min_bound_[i] > limit) {
          break;
        }
        ObjectPtr object = objects_sorted_by_min_[i];
        if (object->DistanceSquareTo(point) <= distance_sqr) {
          result_objects->push_back(object);
        }
      }
    } else {
      const double limit = pvalue - distance;
      for (int i = 0; i < num_objects_; ++i) {
        if (objects_sorted_by_max_bound_[i] < limit) {
          break;
        }
        ObjectPtr object = objects_sorted_by_max_[i];
        if (object->DistanceSquareTo(point) <= distance_sqr) {
          result_objects->push_back(object);
        }
      }
    }
    if (left_subnode_ != nullptr) {
      left_subnode_->GetObjectsInternal(point, distance, distance_sqr,
                                        result_objects);
    }
    if (right_subnode_ != nullptr) {
      right_subnode_->GetObjectsInternal(point, distance, distance_sqr,
                                         result_objects);
    }
  }
  /*****************
   * 获取距离point最近的object
   * @param point
   * @param min_distance_sqr
   * @param nearest_object
   */
  void GetNearestObjectInternal(const Vec2d &point,
                                double *const min_distance_sqr,
                                ObjectPtr *const nearest_object) const {
    //这里的if一开始没用，后面找完一侧子树后再找另一侧子树时可以用这个条件节省无效查找
    if (LowerDistanceSquareToPoint(point) >= *min_distance_sqr - kMathEpsilon) {
      return;
    }
    //根据当前参考轴，确定先找左还是右子树
    const double pvalue = (partition_ == PARTITION_X ? point.x() : point.y());
    const bool search_left_first = (pvalue < partition_position_);
    if (search_left_first) {
      if (left_subnode_ != nullptr) {
        left_subnode_->GetNearestObjectInternal(point, min_distance_sqr,
                                                nearest_object);
      }
    } else {
      if (right_subnode_ != nullptr) {
        right_subnode_->GetNearestObjectInternal(point, min_distance_sqr,
                                                 nearest_object);
      }
    }
    if (*min_distance_sqr <= kMathEpsilon) {
      return;
    }

    //已经找到叶子节点了，直接根据排序找到最近的
    if (search_left_first) {
      for (int i = 0; i < num_objects_; ++i) {
        const double bound = objects_sorted_by_min_bound_[i];
        if (bound > pvalue && Square(bound - pvalue) > *min_distance_sqr) {
          break;
        }
        ObjectPtr object = objects_sorted_by_min_[i];
        const double distance_sqr = object->DistanceSquareTo(point);
        if (distance_sqr < *min_distance_sqr) {
          *min_distance_sqr = distance_sqr;
          *nearest_object = object;
        }
      }
    } else {
      for (int i = 0; i < num_objects_; ++i) {
        const double bound = objects_sorted_by_max_bound_[i];
        if (bound < pvalue && Square(bound - pvalue) > *min_distance_sqr) {
          break;
        }
        ObjectPtr object = objects_sorted_by_max_[i];
        const double distance_sqr = object->DistanceSquareTo(point);
        if (distance_sqr < *min_distance_sqr) {
          *min_distance_sqr = distance_sqr;
          *nearest_object = object;
        }
      }
    }
    //如果距离足够小直接返回
    if (*min_distance_sqr <= kMathEpsilon) {
      return;
    }
    //如果先找左子树，再找一遍右子树，可能存在更近的
    if (search_left_first) {
      if (right_subnode_ != nullptr) {
        right_subnode_->GetNearestObjectInternal(point, min_distance_sqr,
                                                 nearest_object);
      }
    } else {
      if (left_subnode_ != nullptr) {
        left_subnode_->GetNearestObjectInternal(point, min_distance_sqr,
                                                nearest_object);
      }
    }
  }

  /***************
   * 获取objects的边界（找一个大长方形把objects都包起来）
   * @param objects
   */
  void ComputeBoundary(const std::vector<ObjectPtr> &objects) {
    min_x_ = std::numeric_limits<double>::infinity();
    min_y_ = std::numeric_limits<double>::infinity();
    max_x_ = -std::numeric_limits<double>::infinity();
    max_y_ = -std::numeric_limits<double>::infinity();
    for (ObjectPtr object : objects) {
      min_x_ = std::fmin(min_x_, object->aabox().min_x());
      max_x_ = std::fmax(max_x_, object->aabox().max_x());
      min_y_ = std::fmin(min_y_, object->aabox().min_y());
      max_y_ = std::fmax(max_y_, object->aabox().max_y());
    }
    mid_x_ = (min_x_ + max_x_) / 2.0;
    mid_y_ = (min_y_ + max_y_) / 2.0;
    CHECK(!std::isinf(max_x_) && !std::isinf(max_y_) && !std::isinf(min_x_) &&
          !std::isinf(min_y_))
        << "the provided object box size is infinity";
  }

  /************
   * 根据大长方形x和y边的长度，从长的一边分隔（2叉），并确定分裂点
   */
  void ComputePartition() {
    if (max_x_ - min_x_ >= max_y_ - min_y_) {
      partition_ = PARTITION_X;
      partition_position_ = (min_x_ + max_x_) / 2.0;
    } else {
      partition_ = PARTITION_Y;
      partition_position_ = (min_y_ + max_y_) / 2.0;
    }
  }

  /***********************
   * 将objects分配到左右子树，进行2叉
   * @param objects
   * @param left_subnode_objects
   * @param right_subnode_objects
   */
  void PartitionObjects(const std::vector<ObjectPtr> &objects,
                        std::vector<ObjectPtr> *const left_subnode_objects,
                        std::vector<ObjectPtr> *const right_subnode_objects) {
    left_subnode_objects->clear();
    right_subnode_objects->clear();
    std::vector<ObjectPtr> other_objects;
    if (partition_ == PARTITION_X) {
      for (ObjectPtr object : objects) {
        if (object->aabox().max_x() <= partition_position_) {
          left_subnode_objects->push_back(object);
        } else if (object->aabox().min_x() >= partition_position_) {
          right_subnode_objects->push_back(object);
        } else {
          other_objects.push_back(object);
        }
      }
    } else {
      for (ObjectPtr object : objects) {
        if (object->aabox().max_y() <= partition_position_) {
          left_subnode_objects->push_back(object);
        } else if (object->aabox().min_y() >= partition_position_) {
          right_subnode_objects->push_back(object);
        } else {
          other_objects.push_back(object);
        }
      }
    }
    InitObjects(other_objects);
  }

 private:
  int num_objects_ = 0;
  std::vector<ObjectPtr> objects_sorted_by_min_;
  std::vector<ObjectPtr> objects_sorted_by_max_;
  std::vector<double> objects_sorted_by_min_bound_;
  std::vector<double> objects_sorted_by_max_bound_;
  int depth_ = 0;

  // Boundary
  double min_x_ = 0.0;
  double max_x_ = 0.0;
  double min_y_ = 0.0;
  double max_y_ = 0.0;
  double mid_x_ = 0.0;
  double mid_y_ = 0.0;

  enum Partition {
    PARTITION_X = 1,
    PARTITION_Y = 2,
  };
  Partition partition_ = PARTITION_X;
  double partition_position_ = 0.0;

  std::unique_ptr<AABoxKDTree2dNode<ObjectType>> left_subnode_ = nullptr;
  std::unique_ptr<AABoxKDTree2dNode<ObjectType>> right_subnode_ = nullptr;
};

/**
 * @class AABoxKDTree2d
 * @brief The class of KD-tree of Aligned Axis Bounding Box(AABox).
 */
template <class ObjectType>
class AABoxKDTree2d {
 public:
  using ObjectPtr = const ObjectType *;

  /**
   * @brief Contructor which takes a vector of objects and parameters.
   * @param params Parameters to build the KD-tree.
   */
  AABoxKDTree2d(const std::vector<ObjectType> &objects,
                const AABoxKDTreeParams &params) {
    if (!objects.empty()) {
      std::vector<ObjectPtr> object_ptrs;
      for (const auto &object : objects) {
        object_ptrs.push_back(&object);
      }
      root_.reset(new AABoxKDTree2dNode<ObjectType>(object_ptrs, params, 0));
    }
  }

  /**
   * @brief Get the nearest object to a target point.
   * @param point The target point. Search it's nearest object.
   * @return The nearest object to the target point.
   */
  ObjectPtr GetNearestObject(const Vec2d &point) const {
    return root_ == nullptr ? nullptr : root_->GetNearestObject(point);
  }

  /**
   * @brief Get objects within a distance to a point.
   * @param point The center point of the range to search objects.
   * @param distance The radius of the range to search objects.
   * @return All objects within the specified distance to the specified point.
   */
  std::vector<ObjectPtr> GetObjects(const Vec2d &point,
                                    const double distance) const {
    if (root_ == nullptr) {
      return {};
    }
    return root_->GetObjects(point, distance);
  }

  /**
   * @brief Get the axis-aligned bounding box of the objects.
   * @return The axis-aligned bounding box of the objects.
   */
  AABox2d GetBoundingBox() const {
    return root_ == nullptr ? AABox2d() : root_->GetBoundingBox();
  }

 private:
  std::unique_ptr<AABoxKDTree2dNode<ObjectType>> root_ = nullptr;
};

}
}