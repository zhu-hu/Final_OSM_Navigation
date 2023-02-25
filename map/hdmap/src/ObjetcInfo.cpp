//
// Created by luyifan on 19-6-18.
//

#include "ObjectInfo.h"
#include "Impl.h"

using common::math::Vec2d;
// Minimum distance to remove duplicated points.
const double kDuplicatedPointsEpsilon = 1e-7;
// Margin for comparation
const double kEpsilon = 0.1;

namespace hdmap{
  /*删除重复的点*/
  void RemoveDuplicates(std::vector<Vec2d> *points) {
    RETURN_IF_NULL(points);

    int count = 0;
    const double limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
    for (const auto &point : *points) {
      if (count == 0 || point.DistanceSquareTo((*points)[count - 1]) > limit) {
        (*points)[count++] = point;
      }
    }
    points->resize(count);
  }
  /*Curve转Points*/
  void PointsFromCurve(const Curve input_curve, std::vector<Vec2d> *points) {
    RETURN_IF_NULL(points);
    points->clear();

    for (const auto &curve : input_curve.segments) {
      if (curve.line != nullptr) {
        for (const auto &point : curve.line->points) {
          points->emplace_back(*point.x, *point.y);
        }
      } else {
        AERROR << "Can not handle curve type.";
      }
    }
    RemoveDuplicates(points);
  }
  /*Curve转Segments*/
  void SegmentsFromCurve(const Curve &curve, std::vector<common::math::LineSegment2d> *segments) {
    RETURN_IF_NULL(segments);

    std::vector<Vec2d> points;
    PointsFromCurve(curve, &points);
    for (size_t i = 0; i + 1 < points.size(); ++i) {
      segments->emplace_back(points[i], points[i + 1]);
    }
  }
  /*将polygon类型转换为polygon2d*/
  common::math::Polygon2d ConvertToPolygon2d(const Polygon &polygon) {
    std::vector<Vec2d> points;
    points.reserve(polygon.points.size());
    for (const auto &point : polygon.points) {
      points.emplace_back(*point.x, *point.y);
    }
    RemoveDuplicates(&points);
    while (points.size() >= 2 && points[0].DistanceTo(points.back()) <=
                                 common::math::kMathEpsilon) {
      points.pop_back();
    }
    return common::math::Polygon2d(points);
  }
  common::PointENU PointFromVec2d(const Vec2d &point) {
    common::PointENU pt;
    pt.x = point.x();
    pt.y = point.y();
    return pt;
  }
  /******************
   * 车道信息
   */
  LaneInfo::LaneInfo(const Lane &lane): lane_(lane) { Init(); }
  void LaneInfo::Init() {
    PointsFromCurve(*lane_.central_curve, &points_);
    CHECK_GE(points_.size(), 2);
    segments_.clear();
    accumulated_s_.clear();
    unit_directions_.clear();
    headings_.clear();

    double s = 0;
    for(size_t i=0; i+1<points_.size(); ++i){
      segments_.emplace_back(points_[i], points_[i+1]);
      accumulated_s_.emplace_back(s);
      unit_directions_.emplace_back(segments_.back().unit_direction());
      s += segments_.back().length();
    }

    accumulated_s_.emplace_back(s);
    total_length_ = s;
    CHECK(!unit_directions_.empty());
    unit_directions_.emplace_back(unit_directions_.back());
    for (const auto &direction : unit_directions_){
      headings_.emplace_back(direction.Angle());
    }
    
    for (const auto &overlap_id : lane_.overlap_ids){
      overlap_ids_.emplace_back(*overlap_id);
    }
    
    CHECK(!segments_.empty());

    sampled_left_width_.clear();
    sampled_right_width_.clear();
    for (const auto &sample : lane_.left_sample) {
      sampled_left_width_.emplace_back(*sample.s, *sample.width);
    }
    for (const auto &sample : lane_.right_sample) {
      sampled_right_width_.emplace_back(*sample.s, *sample.width);
    }

    if (lane_.type != nullptr) {
      if (*lane_.type == Lane::NORMAL_ROAD || *lane_.type == Lane::JUNCTION_ROAD || *lane_.type == Lane::COUNTRY_ROAD
       || *lane_.type == Lane::SLOW_ROAD || *lane_.type == Lane::PARKING_ROAD || *lane_.type == Lane::IDLING_ROAD || *lane_.type == Lane::HIGHWAY_ROAD || *lane_.type == Lane::STURN_ROAD) {
        const double kMinHalfWidth = 0.9; // width of CyberTiggo Car is 1.8m
        for (const auto &p : sampled_left_width_){
          if(p.second < kMinHalfWidth) AERROR<<"lane[id = "<<*lane_.id
                <<"]. sampled_left_width_["<<p.second
                <<"] is too small. It should be larger than half vehicle width["
                <<kMinHalfWidth<<"].";
        }
        for (const auto &p : sampled_right_width_){
          if(p.second < kMinHalfWidth) AERROR<<"lane[id = "<<*lane_.id
                <<"]. sampled_right_width_["<<p.second
                <<"] is too small. It should be larger than half vehicle width["
                <<kMinHalfWidth<<"].";
        }
      } else{
        AERROR << "lane_[id = " << *lane_.id << "type is NONE.";
      }
    } else {
      AERROR << "lane_[id = " << *lane_.id <<"has NO type.";
    }

    sampled_left_road_width_.clear();
    sampled_right_road_width_.clear();
//    for (const auto &sample : lane_.left_road_sample) {
//      sampled_left_road_width_.emplace_back(*sample.s, *sample.width);
//    }
//    for (const auto &sample : lane_.right_road_sample) {
//      sampled_right_road_width_.emplace_back(*sample.s, *sample.width);
//    }

    CreateKDTree();
  }
  void LaneInfo::GetWidth(const double s, double *left_width,
                          double *right_width) const {
    if (left_width != nullptr) {
      *left_width = GetWidthFromSample(sampled_left_width_, s);
    }
    if (right_width != nullptr) {
      *right_width = GetWidthFromSample(sampled_right_width_, s);
    }
  }
  double LaneInfo::Heading(const double s) const {
    const double kEpsilon = 0.2;
    double tmp = s;
    if (tmp + kEpsilon < accumulated_s_.front()) {
      AERROR << "s:" << s << " should be >= " << accumulated_s_.front();
      return 0.0;
    }
    if (tmp - kEpsilon > accumulated_s_.back()) {
      AERROR << "s:" << s << " should be <= " << accumulated_s_.back();
      return 0.0;
    }
    
    const double kEpsilon2 = 0.01;
    if (tmp + kEpsilon2 < accumulated_s_.front()) {
      tmp = accumulated_s_.front();
    }
    if (tmp - kEpsilon > accumulated_s_.back()) {
      tmp = accumulated_s_.back();
    }

    auto iter = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), tmp);
    int index = static_cast<int>(std::distance(accumulated_s_.begin(), iter));
    if (index == 0 || *iter - tmp <= common::math::kMathEpsilon) {
      return headings_[index];
    } else {
      return common::math::slerp(headings_[index - 1], accumulated_s_[index - 1],
                                 headings_[index], accumulated_s_[index], s);
    }
  }
  double LaneInfo::Curvature(const double s) const {
    double tmp =s;
    if (points_.size() < 2) {
      AERROR << "Not enough points to compute curvature.";
      return 0.0;
    }
    const double kEpsilon = 0.2;
    if (tmp + kEpsilon < accumulated_s_.front()) {
      AERROR << "s:" << s << " should be >= " << accumulated_s_.front();
      return 0.0;
    }
    if (tmp - kEpsilon > accumulated_s_.back()) {
      AERROR << "s:" << s << " should be <= " << accumulated_s_.back();
      return 0.0;
    }
    
    const double kEpsilon2 = 0.01;
    if (tmp + kEpsilon2 < accumulated_s_.front()) {
      tmp = accumulated_s_.front();
    }
    if (tmp - kEpsilon > accumulated_s_.back()) {
      tmp = accumulated_s_.back();
    }

    auto iter = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), tmp);
    if (iter == accumulated_s_.end()) {
      ADEBUG << "Reach the end of lane.";
      return 0.0;
    }
    int index = static_cast<int>(std::distance(accumulated_s_.begin(), iter));
    if (index == 0) {
      ADEBUG << "Reach the beginning of lane";
      return 0.0;
    } else {
      return (headings_[index] - headings_[index - 1]) /
             (accumulated_s_[index] - accumulated_s_[index - 1] + kEpsilon);
    }
  }
  double LaneInfo::GetWidth(const double s) const {
    double left_width = 0.0;
    double right_width = 0.0;
    GetWidth(s, &left_width, &right_width);
    return left_width + right_width;
  }
  double LaneInfo::GetEffectiveWidth(const double s) const {
    double left_width = 0.0;
    double right_width = 0.0;
    GetWidth(s, &left_width, &right_width);
    return 2 * std::min(left_width, right_width);
  }
  void LaneInfo::GetRoadWidth(const double s, double *left_width,
                              double *right_width) const {
    if (left_width != nullptr) {
      *left_width = GetWidthFromSample(sampled_left_road_width_, s);
    }
    if (right_width != nullptr) {
      *right_width = GetWidthFromSample(sampled_right_road_width_, s);
    }
  }
  double LaneInfo::GetRoadWidth(const double s) const {
    double left_width = 0.0;
    double right_width = 0.0;
    GetRoadWidth(s, &left_width, &right_width);
    return left_width + right_width;
  }
  double LaneInfo::GetWidthFromSample(
      const std::vector<LaneInfo::SampledWidth> &samples, const double s) const {
    if (samples.empty()) {
      return 0.0;
    }
    if (s <= samples[0].first) {
      return samples[0].second;
    }
    if (s >= samples.back().first) {
      return samples.back().second;
    }
    int low = 0;
    int high = static_cast<int>(samples.size());
    while (low + 1 < high) {
      const int mid = (low + high) / 2;
      if (samples[mid].first <= s) {
        low = mid;
      } else {
        high = mid;
      }
    }
    const LaneInfo::SampledWidth &sample1 = samples[low];
    const LaneInfo::SampledWidth &sample2 = samples[high];
    const double ratio = (sample2.first - s) / (sample2.first - sample1.first);
    return sample1.second * ratio + sample2.second * (1.0 - ratio);
  }
  bool LaneInfo::IsOnLane(const Vec2d &point) const {
    double accumulate_s = 0.0;
    double lateral = 0.0;
    if (!GetProjection(point, &accumulate_s, &lateral)) {
      return false;
    }

    if (accumulate_s > (total_length() + kEpsilon) ||
        (accumulate_s + kEpsilon) < 0.0) {
      return false;
    }

    double left_width = 0.0;
    double right_width = 0.0;
    GetWidth(accumulate_s, &left_width, &right_width);
    if (lateral < left_width && lateral > -right_width) {
      return true;
    }
    return false;
  }
  bool LaneInfo::IsOnLane(const common::math::Box2d &box) const {
    std::vector<Vec2d> corners;
    box.GetAllCorners(&corners);
    for (const auto &corner : corners) {
      if (!IsOnLane(corner)) {
        return false;
      }
    }
    return true;
  }
  //获取s处的世界坐标系坐标(l=0)
  common::PointENU LaneInfo::GetSmoothPoint(double s) const {
    common::PointENU point;
    RETURN_VAL_IF(points_.size() < 2, point);
    if (s <= 0.0) {
      return PointFromVec2d(points_[0]);
    }

    if (s >= total_length()) {
      return PointFromVec2d(points_.back());
    }
    const auto low_itr =
        std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
    RETURN_VAL_IF(low_itr == accumulated_s_.end(), point);
    size_t index = low_itr - accumulated_s_.begin();
    double delta_s = *low_itr - s;
    if (delta_s < common::math::kMathEpsilon) {
      return PointFromVec2d(points_[index]);
    }
    auto smooth_point = points_[index] - unit_directions_[index - 1] * delta_s;
    return PointFromVec2d(smooth_point);
  }
  double LaneInfo::DistanceTo(const Vec2d &point) const {
    const auto segment_box = lane_segment_kdtree_->GetNearestObject(point);
    RETURN_VAL_IF_NULL(segment_box, 0.0);
    return segment_box->DistanceTo(point);
  }
  double LaneInfo::DistanceTo(const Vec2d &point, Vec2d *map_point,
                              double *s_offset, int *s_offset_index) const {
    RETURN_VAL_IF_NULL(map_point, 0.0);
    RETURN_VAL_IF_NULL(s_offset, 0.0);
    RETURN_VAL_IF_NULL(s_offset_index, 0.0);

    const auto segment_box = lane_segment_kdtree_->GetNearestObject(point);
    RETURN_VAL_IF_NULL(segment_box, 0.0);
    int index = segment_box->id();
    double distance = segments_[index].DistanceTo(point, map_point);
    *s_offset_index = index;
    *s_offset =
        accumulated_s_[index] + segments_[index].start().DistanceTo(*map_point);
    return distance;
  }
  //获取point在center_line上的投影点
  common::PointENU LaneInfo::GetNearestPoint(const Vec2d &point, double *distance) const {
    common::PointENU empty_point;
    RETURN_VAL_IF_NULL(distance, empty_point);

    const auto segment_box = lane_segment_kdtree_->GetNearestObject(point);
    RETURN_VAL_IF_NULL(segment_box, empty_point);
    int index = segment_box->id();
    Vec2d nearest_point;
    *distance = segments_[index].DistanceTo(point, &nearest_point);

    return PointFromVec2d(nearest_point);
  }

  bool LaneInfo::GetProjectionWithHeading(const Vec2d &point, double heading, double *accumulate_s,
                               double *lateral) const {
    RETURN_VAL_IF_NULL(accumulate_s, false);
    RETURN_VAL_IF_NULL(lateral, false);

    if (segments_.empty()) {
      return false;
    }
    double min_dist = std::numeric_limits<double>::infinity();
    int seg_num = static_cast<int>(segments_.size());
    int min_index = 0;
    for (int i = 0; i < seg_num; ++i) {
      const double distance = segments_[i].DistanceSquareTo(point) + 
                              pow(fabs(common::math::AngleDiff(segments_[i].heading(), heading)), 2.0); // coeff需要调整
      if (distance < min_dist) {
        min_index = i;
        min_dist = distance;
      }
    }
    min_dist = std::sqrt(min_dist);
    const auto &nearest_seg = segments_[min_index];
    const auto prod = nearest_seg.ProductOntoUnit(point);
    const auto proj = nearest_seg.ProjectOntoUnit(point);
    if (min_index == 0) {
      *accumulate_s = std::min(proj, nearest_seg.length());
      if (proj < 0) {
        *lateral = prod;
      } else {
        *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
      }
    } else if (min_index == seg_num - 1) {
      *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
      if (proj > 0) {
        *lateral = prod;
      } else {
        *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
      }
    } else {
      *accumulate_s = accumulated_s_[min_index] +
                      std::max(0.0, std::min(proj, nearest_seg.length()));
      *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
    }
    return true;
  }

  bool LaneInfo::GetProjection(const Vec2d &point, double *accumulate_s,
                               double *lateral) const {
    RETURN_VAL_IF_NULL(accumulate_s, false);
    RETURN_VAL_IF_NULL(lateral, false);

    if (segments_.empty()) {
      return false;
    }
    double min_dist = std::numeric_limits<double>::infinity();
    int seg_num = static_cast<int>(segments_.size());
    int min_index = 0;
    for (int i = 0; i < seg_num; ++i) {
      const double distance = segments_[i].DistanceSquareTo(point);
      if (distance < min_dist) {
        min_index = i;
        min_dist = distance;
      }
    }
    min_dist = std::sqrt(min_dist);
    const auto &nearest_seg = segments_[min_index];
    const auto prod = nearest_seg.ProductOntoUnit(point);
    const auto proj = nearest_seg.ProjectOntoUnit(point);
    if (min_index == 0) {
      *accumulate_s = std::min(proj, nearest_seg.length());
      if (proj < 0) {
        *lateral = prod;
      } else {
        *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
      }
    } else if (min_index == seg_num - 1) {
      *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
      if (proj > 0) {
        *lateral = prod;
      } else {
        *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
      }
    } else {
      *accumulate_s = accumulated_s_[min_index] +
                      std::max(0.0, std::min(proj, nearest_seg.length()));
      *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
    }
    return true;
  }

  bool LaneInfo::GetUTM(const double &accumulate_s, const double &lateral,
                        double *utm_x, double *utm_y) const{
    double s_temp = accumulate_s;
    common::PointENU point = GetSmoothPoint(s_temp);
    double heading = Heading(accumulate_s);
    *utm_x = *point.x - sin(heading) * lateral;
    *utm_y = *point.y + cos(heading) * lateral;
    return true;
  }
  void LaneInfo::CreateKDTree() {
    common::math::AABoxKDTreeParams params;
    params.max_leaf_dimension = 5.0;  // meters.
    params.max_leaf_size = 16;

    segment_box_list_.clear();
    for (size_t id = 0; id < segments_.size(); ++id) {
      const auto &segment = segments_[id];
      segment_box_list_.emplace_back(
          common::math::AABox2d(segment.start(), segment.end()), this,
          &segment, id);
    }
    lane_segment_kdtree_.reset(new LaneSegmentKDTree(segment_box_list_, params));
  }
  /******************
   * 红绿灯信息
   * @param signal
   */
  SignalInfo::SignalInfo(const Signal &signal): signal_(signal) {
    Init();
  }
  void SignalInfo::Init() {
    // for (const auto &curve : signal_.stop_line){
    //   SegmentsFromCurve(curve, &segments_);
    // }
    // CHECK(!segments_.empty());
    // std::vector<Vec2d> points;//这里的points的意义？
    // for (const auto &segment : segments_) {
    //   points.emplace_back(segment.start());
    //   points.emplace_back(segment.end());
    // }
    // CHECK_GT(points.size(), 0);
  }

  /******************
   * 停止牌信息
   */
  StopSignInfo::StopSignInfo(const StopSign &stop_sign): stop_sign_(stop_sign) {
    Init();
  }
  void StopSignInfo::Init() {
    for (const auto &curve : stop_sign_.stop_line) {
      SegmentsFromCurve(curve, &segments_);
    }
    CHECK(!segments_.empty());

   for (const auto &overlap_id : stop_sign_.overlap_ids) {
     overlap_ids_.emplace_back(*overlap_id);
   }
  }
  /*
  void StopSignInfo::PostProcess(const HDMapImpl &map_instance) {
    UpdateOverlaps(map_instance);
  }

  void StopSignInfo::UpdateOverlaps(const HDMapImpl &map_instance) {
    for (const auto &overlap_id : overlap_ids_) {
      const auto &overlap_ptr = map_instance.GetOverlapById(overlap_id);
      if (overlap_ptr == nullptr) {
        continue;
      }

      for (const auto &object : overlap_ptr->overlap().objects_overlap) {
        const auto &object_id = *(object->id);
        if (object_id == id()) {
          continue;
        }

        if (*(object->type) == ObjectOverlapInfo::Junction) {
          overlap_junction_ids_.push_back(*(object->id));
        } else if (*(object->type) == ObjectOverlapInfo::Lane) {
          overlap_lane_ids_.push_back(*(object->id));
        }
      }
    }
    if (overlap_junction_ids_.size() <= 0) {
      AWARN << "stop sign " << id() << "has no overlap with any junction.";
    }
  }
  */

  /****************
   * 路口信息
   */
  JunctionInfo::JunctionInfo(const Junction &junction): junction_(junction) {
    Init();
  }

  void JunctionInfo::Init() {
    polygon_ = ConvertToPolygon2d(*(junction_.polygon));
    CHECK_GT(polygon_.num_points(), 2);

    for (const auto &overlap_id : junction_.overlap_ids) {
      overlap_ids_.emplace_back(*overlap_id);
    }
  }
  /*
  void JunctionInfo::PostProcess(const HDMapImpl &map_instance) {
    UpdateOverlaps(map_instance);
  }


  void JunctionInfo::UpdateOverlaps(const HDMapImpl &map_instance) {
    for (const auto &overlap_id : overlap_ids_) {
      const auto &overlap_ptr = map_instance.GetOverlapById(overlap_id);
      if (overlap_ptr == nullptr) {
        continue;
      }

      for (const auto &object : overlap_ptr->overlap().objects_overlap) {
        const auto &object_id = *(object->id);
        if (object_id == id()) {
          continue;
        }

        if (*(object->type) == ObjectOverlapInfo::StopSign) {
          overlap_stop_sign_ids_.push_back(*(object->id));
        }
      }
    }
  }
  */

  /******************
   * 重叠区域信息
   * @param overlap
   */
  OverlapInfo::OverlapInfo(const Overlap &overlap): overlap_(overlap) {
    Init();
  }
  void OverlapInfo::Init() {
    
  }
}
