//
// Created by luyifan on 19-6-18.
//

#ifndef HDMAP_OBJECTINFO_H
#define HDMAP_OBJECTINFO_H

#include "math/LineSegment2d.h"
#include "math/AABox2d.h"
#include "math/Kdtree.h"
#include "math/utlis.h"
#include "math/Polygon.h"
#include "math/vec2d.h"
#include "math/linear_interpolation.h"

#include "struct/Map.h"

namespace hdmap
{
  //模板类：{aabox, Object, GeoObject}捆绑在一起
  template <class Object, class GeoObject>
  class ObjectWithAABox
  {
  public:
    ObjectWithAABox(const common::math::AABox2d &aabox,
                    const Object *object, const GeoObject *geo_object,
                    const int id)
        : aabox_(aabox), object_(object), geo_object_(geo_object), id_(id) {}
    ~ObjectWithAABox() {}

    const common::math::AABox2d &aabox() const { return aabox_; }
    //该对象距离某点的距离
    double DistanceTo(const common::math::Vec2d &point) const
    {
      return geo_object_->DistanceTo(point);
    }
    double DistanceSquareTo(const common::math::Vec2d &point) const
    {
      return geo_object_->DistanceSquareTo(point);
    }
    const Object *object() const { return object_; }
    const GeoObject *geo_object() const { return geo_object_; }
    int id() const { return id_; }

  private:
    common::math::AABox2d aabox_;
    const Object *object_;
    const GeoObject *geo_object_;
    int id_;
  };

  class LaneInfo;
  class SignalInfo;
  class StopSignInfo;
  class JunctionInfo;
  class OverlapInfo;
  class HDMapImpl;

  using LaneInfoConstPtr = std::shared_ptr<const LaneInfo>;
  using SignalInfoConstPtr = std::shared_ptr<const SignalInfo>;
  using StopSignInfoConstPtr = std::shared_ptr<const StopSignInfo>;
  using JunctionInfoConstPtr = std::shared_ptr<const JunctionInfo>;
  using OverlapInfoConstPtr = std::shared_ptr<const OverlapInfo>;
  /**********************
   * 车道
   */
  class LaneInfo
  {
  public:
    explicit LaneInfo(const Lane &lane);

    const string &id() const { return *lane_.id; }
    const string &road_id() const { return road_id_; }
    const string &section_id() const { return section_id_; }
    const Lane &lane() const { return lane_; }
    const std::vector<common::math::Vec2d> &points() const
    {
      return points_;
    }
    const std::vector<common::math::Vec2d> &unit_directions() const
    {
      return unit_directions_;
    }
    double Heading(const double s) const;
    double Curvature(const double s) const;
    const std::vector<double> &headings() const { return headings_; }
    const std::vector<common::math::LineSegment2d> &segments() const
    {
      return segments_;
    }
    const std::vector<double> &accumulate_s() const { return accumulated_s_; }
    double total_length() const { return total_length_; }

    //获取车道宽度
    using SampledWidth = std::pair<double, double>;
    const std::vector<SampledWidth> &sampled_left_width() const
    {
      return sampled_left_width_;
    }
    const std::vector<SampledWidth> &sampled_right_width() const
    {
      return sampled_right_width_;
    }
    void GetWidth(const double s, double *left_width, double *right_width) const;
    double GetWidth(const double s) const;
    double GetEffectiveWidth(const double s) const;

    //获取道路宽度
    const std::vector<SampledWidth> &sampled_left_road_width() const
    {
      return sampled_left_road_width_;
    }
    const std::vector<SampledWidth> &sampled_right_road_width() const
    {
      return sampled_right_road_width_;
    }
    void GetRoadWidth(const double s, double *left_width,
                      double *right_width) const;
    double GetRoadWidth(const double s) const;

    bool IsOnLane(const common::math::Vec2d &point) const;
    bool IsOnLane(const common::math::Box2d &box) const;
    common::PointENU GetSmoothPoint(double s) const;
    double DistanceTo(const common::math::Vec2d &point) const;
    double DistanceTo(const common::math::Vec2d &point,
                      common::math::Vec2d *map_point, double *s_offset,
                      int *s_offset_index) const;
    common::PointENU GetNearestPoint(
        const common::math::Vec2d &point, double *distance) const;
    bool GetProjectionWithHeading(const common::math::Vec2d &point, double heading,
                                  double *accumulate_s, double *lateral) const;
    bool GetProjection(const common::math::Vec2d &point,
                       double *accumulate_s, double *lateral) const;
    bool GetUTM(const double &accumulate_s, const double &lateral,
                double *utm_x, double *utm_y) const;

    // 获取重叠区域
    const std::vector<string> &overlap_ids() const
    {
      return overlap_ids_;
    }

  private:
    friend class HDMapImpl;
    friend class RoadInfo;
    void Init();
    //void PostProcess(const HDMapImpl &map_instance);
    //void UpdateOverlaps(const HDMapImpl &map_instance);
    double GetWidthFromSample(const std::vector<LaneInfo::SampledWidth> &samples,
                              const double s) const;
    void CreateKDTree();
    void set_road_id(const string &road_id) { road_id_ = road_id; }
    void set_section_id(const string &section_id) { section_id_ = section_id; }

  private:
    const Lane &lane_;
    std::vector<common::math::Vec2d> points_; //center_line的路点
    std::vector<common::math::Vec2d> unit_directions_;
    std::vector<double> headings_;                      //center_line上每个路点的方向
    std::vector<common::math::LineSegment2d> segments_; //center_line每两个point组成一个segment
    std::vector<double> accumulated_s_;                 //center_line上每个点对应一个累计的s值，第一个点s=0
    double total_length_ = 0.0;
    std::vector<SampledWidth> sampled_left_width_; //左侧车道边线
    std::vector<SampledWidth> sampled_right_width_;
    std::vector<SampledWidth> sampled_left_road_width_; //左侧道路边线
    std::vector<SampledWidth> sampled_right_road_width_;
    std::vector<string> overlap_ids_; //重叠区域

    using LaneSegmentBox = ObjectWithAABox<LaneInfo, common::math::LineSegment2d>;
    using LaneSegmentKDTree = common::math::AABoxKDTree2d<LaneSegmentBox>;
    std::vector<LaneSegmentBox> segment_box_list_; //lane的segments_kdtree的节点
    std::unique_ptr<LaneSegmentKDTree> lane_segment_kdtree_;

    string road_id_;
    string section_id_;
  };

  /**********************
   * 红绿灯
   */
  class SignalInfo
  {
  public:
    explicit SignalInfo(const Signal &signal);

    const string &id() const { return *signal_.id; }
    const Signal &signal() const { return signal_; }
    const std::vector<common::math::LineSegment2d> &segments() const
    {
      return segments_;
    }

  private:
    void Init();

  private:
    const Signal &signal_;
    std::vector<common::math::LineSegment2d> segments_; //停止线
  };

  using SignalSegmentBox = ObjectWithAABox<SignalInfo, common::math::LineSegment2d>;
  using SignalSegmentKDTree = common::math::AABoxKDTree2d<SignalSegmentBox>;

  /*********************
   * 停止牌
   */
  class StopSignInfo
  {
  public:
    explicit StopSignInfo(const StopSign &stop_sign);

    const string &id() const { return *stop_sign_.id; }
    const StopSign &stop_sign() const { return stop_sign_; }
    const std::vector<common::math::LineSegment2d> &segments() const
    {
      return segments_;
    }
    const std::vector<string> &OverlapLaneIds() const { return overlap_lane_ids_; }
    const std::vector<string> &OverlapJunctionIds() const
    {
      return overlap_junction_ids_;
    }

  private:
    friend class HDMapImpl;
    void Init();
    void PostProcess(const HDMapImpl &map_instance);
    void UpdateOverlaps(const HDMapImpl &map_instance);

  private:
    const StopSign &stop_sign_;
    std::vector<common::math::LineSegment2d> segments_;

    std::vector<string> overlap_lane_ids_;
    std::vector<string> overlap_junction_ids_;
    std::vector<string> overlap_ids_;
  };
  using StopSignSegmentBox = ObjectWithAABox<StopSignInfo, common::math::LineSegment2d>;
  using StopSignSegmentKDTree = common::math::AABoxKDTree2d<StopSignSegmentBox>;

  /*********************
   * 路口
   */
  class JunctionInfo
  {
  public:
    explicit JunctionInfo(const Junction &junction);

    const string &id() const { return *(junction_.id); }
    const Junction &junction() const { return junction_; }
    const common::math::Polygon2d &polygon() const { return polygon_; }

    const std::vector<std::string> &OverlapStopSignIds() const
    {
      return overlap_stop_sign_ids_;
    }

  private:
    friend class HDMapImpl;
    void Init();
    void PostProcess(const HDMapImpl &map_instance);
    void UpdateOverlaps(const HDMapImpl &map_instance);

  private:
    const Junction &junction_;
    common::math::Polygon2d polygon_;

    std::vector<std::string> overlap_stop_sign_ids_;
    std::vector<std::string> overlap_ids_;
  };
  using JunctionPolygonBox = ObjectWithAABox<JunctionInfo, common::math::Polygon2d>;
  using JunctionPolygonKDTree = common::math::AABoxKDTree2d<JunctionPolygonBox>;

  struct JunctionBoundary
  {
    JunctionInfoConstPtr junction_info;
  };
  using JunctionBoundaryPtr = std::shared_ptr<JunctionBoundary>;

  /**********************
   * 重叠区域
   */
  class OverlapInfo
  {
  public:
    explicit OverlapInfo(const Overlap &overlap);

    const string &id() const { return *overlap_.id; }
    const Overlap &overlap() const { return overlap_; }
    const string &object1_id() const { return *overlap_.object1_id; }
    const string &object2_id() const { return *overlap_.object2_id; }
    const Overlap::Type &object1_type() const { return *overlap_.object1_type; }
    const Overlap::Type &object2_type() const { return *overlap_.object2_type; }
    const double &start_s() const { return *overlap_.start_s; }
    const double &end_s() const { return *overlap_.end_s; }

  private:
    void Init();

  private:
    const Overlap &overlap_;
  };
}

#endif //HDMAP_OBJECTINFO_H
