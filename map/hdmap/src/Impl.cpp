//
// Created by luyifan on 19-6-18.
//

#include "Impl.h"

using namespace hdmap;

using common::PointENU;
using common::math::AABoxKDTreeParams;
using common::math::Vec2d;

// default lanes search radius in GetForwardNearestSignalsOnLane
constexpr double kLanesSearchRange = 10.0;
// backward search distance in GetForwardNearestSignalsOnLane
constexpr int kBackwardDistance = 4;

bool EndWith(const std::string &ori, const std::string &pat)
{
  return std::equal(pat.rbegin(), pat.rend(), ori.rbegin());
}
/*****************
 * load map from xodr file
 * @param filename
 * @return
 */
int HDMapImpl::LoadMap(const std::string &filename,
                       const double& utm_origin_x,
                       const double& utm_origin_y)
{
  Clear();
  if (EndWith(filename, ".json"))
  {
    adapter::GLOBAL_ZERO_X = utm_origin_x;
    adapter::GLOBAL_ZERO_Y = utm_origin_y;
    if (!adapter::JsonAdapter::LoadData(filename, &map_))
    {
      return -1;
    }
  }
  else
  {
    AERROR << "map file format is invalid!!!";
    return -1;
  }

  AINFO << "file load data finished.";

  if (map_.roads.empty())
  {
    AERROR << "no roads data!!!";
    return -1;
  }

  // if(map_.stop_signs.empty()){
  //   AERROR<<"no stop signs data!!!";
  //   return -1;
  // }

  // if(map_.signals.empty()){
  //   AERROR<<"no signals data!!!";
  //   return -1;
  // }

  // if(map_.junctions.empty()){
  //   AERROR<<"no junctions data!!!";
  //   return -1;
  // }

  // if(map_.overlaps.empty()){
  //   AERROR<<"no overlaps data!!!";
  //   return -1;
  // }

  LoadElements(map_);
  AINFO << "map file load elements finished.";

  int lane_num = 0;
  for (auto road : map_.roads)
  {
    for (auto lane : road.lanes)
    {
      for(auto successor : lane.successor_ids){
        AINFO << "------------lane " << *lane.id << " suceesor : " << *successor <<"  -----------";
      }
      
      if (lane.speed_expect != nullptr)
        std::cout << *lane.id << " : " << *lane.speed_expect << " km/h" << std::endl;
      if (*lane.type == Lane::LaneType::NORMAL_ROAD)
        AINFO << "------------lane " << *lane.id << "  NORMAL_ROAD-----------";
      else if (*lane.type == Lane::LaneType::JUNCTION_ROAD)
        AINFO << "------------lane " << *lane.id << "  JUNCTION_ROAD-----------";
      else if (*lane.type == Lane::LaneType::SLOW_ROAD)
        AINFO << "------------lane " << *lane.id << "  SLOW_ROAD-----------";
      else if (*lane.type == Lane::LaneType::HIGHWAY_ROAD)
        AINFO << "------------lane " << *lane.id << "  HIGHWAY_ROAD-----------";
      else if (*lane.type == Lane::LaneType::STURN_ROAD)
        AINFO << "------------lane " << *lane.id << "  STURN_ROAD-----------";
      else if (*lane.type == Lane::LaneType::PARKING_ROAD)
        AINFO << "------------lane " << *lane.id << "  PARKING_ROAD-----------";
      else if (*lane.type == Lane::LaneType::IDLING_ROAD)
        AINFO << "------------lane " << *lane.id << "  IDLING_ROAD-----------";
      else
        AERROR << "------------lane " << *lane.id << "  WRONG_ROAD_TYPE-----------";
      if (*lane.type != Lane::LaneType::NORMAL_ROAD && *lane.type != Lane::LaneType::JUNCTION_ROAD && *lane.type != Lane::LaneType::SLOW_ROAD &&
          *lane.type != Lane::LaneType::HIGHWAY_ROAD && *lane.type != Lane::LaneType::STURN_ROAD && *lane.type != Lane::LaneType::PARKING_ROAD &&
          *lane.type != Lane::LaneType::IDLING_ROAD && *lane.type != Lane::LaneType::COUNTRY_ROAD)
        AERROR << "------------lane " << *lane.id << "  -----------";

      d_double x_temp = lane.central_curve->segments.front().line->points.front().x;
      d_double y_temp = lane.central_curve->segments.front().line->points.front().y;
      d_double distance_temp = lane.length;
      AINFO << "start point"
            << " on utm x: " << *x_temp << " y: " << *y_temp << " length: " << *distance_temp;
      // for(auto success : lane.successor_ids) AINFO << "success lane: " << *success;
      lane_num += 1;
    }
  }
  AINFO << "There are " << lane_num << " roads in total.";

  // int junction_num = 0;
  // for(auto junction : map_.junctions){
  //   AINFO << "------------junction " << *junction.id << "-----------";
  //   for(auto overlap : junction.overlap_ids) AINFO << "overlap lane: " << *overlap;
  //   junction_num += 1;
  // }
  // AINFO << "There are " << junction_num << " junctions in total." ;

  // int stop_sign_num = 0;
  // for(auto stop_sign : map_.stop_signs){
  //   AINFO << "------------stop sign " << *stop_sign.id << "-----------";
  //   for(auto overlap : stop_sign.overlap_ids) AINFO << "overlaps : " << *overlap;
  //   stop_sign_num += 1;
  // }
  // AINFO << "There are " << stop_sign_num << " stop signs in total." ;

  // int overlap_num = 0;
  // for(auto overlap : map_.overlaps){
  //   AINFO << "----overlap " << *overlap.id << "--object1--" << *overlap.object1_id << "--object2--" << *overlap.object2_id << "--------";
  //   overlap_num += 1;
  // }
  // AINFO << "There are " << overlap_num << " overlaps in total." ;

  // int signal_num = 0;
  // for(auto signal : map_.signals){
  //   AINFO << "----signal " << *signal.id << "--latitude--" << *signal.latitude << "--longitude--" << *signal.longitude << "--height--" << *signal.height << "--";
  //   signal_num += 1;
  // }
  // AINFO << "There are " << signal_num << " signals in total." ;

  return true;
}

/*****************
 * load elements from OpenDrivePtr map and build kdtrees
 * @param map
 * @return
 */
int HDMapImpl::LoadElements(const Map &map)
{
  if (&map != &map_)
  { // avoid an unnecessary copy
    Clear();
    map_ = map;
  }
  for (auto &road : map_.roads)
  {
    for (const auto &lane : road.lanes)
    {
      lane_table_[*lane.id].reset(new LaneInfo(lane));
    }
    for (const auto &signal : road.signals)
    {
      signal_table_[*signal.id].reset(new SignalInfo(signal));
    }
  }
  for (const auto &stop_sign : map_.stop_signs)
  {
    stop_sign_table_[*stop_sign.id].reset(new StopSignInfo(stop_sign));
  }
  for (const auto &junction : map_.junctions)
  {
    junction_table_[*junction.id].reset(new JunctionInfo(junction));
  }
  for (const auto &overlap : map_.overlaps)
  {
    overlap_table_[*overlap.id].reset(new OverlapInfo(overlap));
  }
  for (const auto &signal : map_.signals)
  {
    signal_table_[*signal.id].reset(new SignalInfo(signal));
  }

  BuildLaneSegmentKDTree();
  BuildJunctionPolygonKDTree();
  // BuildSignalSegmentKDTree();
  BuildStopSignSegmentKDTree();
}

/********************
 * Get object by id
 * @param id
 * @return
 */
LaneInfoConstPtr HDMapImpl::GetLaneById(const string &id) const
{
  LaneTable::const_iterator it = lane_table_.find(id);
  return it != lane_table_.end() ? it->second : nullptr;
}
JunctionInfoConstPtr HDMapImpl::GetJunctionById(const std::string &id) const
{
  JunctionTable::const_iterator it = junction_table_.find(id);
  return it != junction_table_.end() ? it->second : nullptr;
}
SignalInfoConstPtr HDMapImpl::GetSignalById(const std::string &id) const
{
  SignalTable::const_iterator it = signal_table_.find(id);
  return it != signal_table_.end() ? it->second : nullptr;
}
StopSignInfoConstPtr HDMapImpl::GetStopSignById(const std::string &id) const
{
  StopSignTable::const_iterator it = stop_sign_table_.find(id);
  return it != stop_sign_table_.end() ? it->second : nullptr;
}
OverlapInfoConstPtr HDMapImpl::GetOverlapById(const std::string &id) const
{
  OverlapTable::const_iterator it = overlap_table_.find(id);
  return it != overlap_table_.end() ? it->second : nullptr;
}

/********************
 * Search objects from kdtrees
 * @param point
 * @param distance
 * @param signals
 * @return
 */
/**  Lane  **/
int HDMapImpl::GetLanes(const PointENU &point, double distance,
                        std::vector<LaneInfoConstPtr> *lanes) const
{
  return GetLanes({*point.x, *point.y}, distance, lanes);
}
int HDMapImpl::GetLanes(const Vec2d &point, double distance,
                        std::vector<LaneInfoConstPtr> *lanes) const
{
  if (lanes == nullptr || lane_segment_kdtree_ == nullptr)
  {
    return -1;
  }
  lanes->clear();
  std::vector<string> ids;
  const int status =
      SearchObjects(point, distance, *lane_segment_kdtree_, &ids);
  if (status < 0)
  {
    return status;
  }
  for (const auto &id : ids)
  {
    lanes->emplace_back(GetLaneById(id));
  }
  return 0;
}
int HDMapImpl::GetNearestLane(const PointENU &point,
                              LaneInfoConstPtr *nearest_lane, double *nearest_s,
                              double *nearest_l) const
{
  return GetNearestLane({*(point.x), *(point.y)}, nearest_lane, nearest_s,
                        nearest_l);
}
int HDMapImpl::GetNearestLane(const Vec2d &point,
                              LaneInfoConstPtr *nearest_lane, double *nearest_s,
                              double *nearest_l) const
{
  CHECK_NOTNULL(nearest_lane);
  CHECK_NOTNULL(nearest_s);
  CHECK_NOTNULL(nearest_l);
  const auto *segment_object = lane_segment_kdtree_->GetNearestObject(point);
  if (segment_object == nullptr)
  {
    return -1;
  }
  const string &lane_id = segment_object->object()->id();
  *nearest_lane = GetLaneById(lane_id);
  CHECK(*nearest_lane);
  const int id = segment_object->id();
  const auto &segment = (*nearest_lane)->segments()[id];
  Vec2d nearest_pt;
  segment.DistanceTo(point, &nearest_pt);
  *nearest_s = (*nearest_lane)->accumulate_s()[id] +
               nearest_pt.DistanceTo(segment.start());
  *nearest_l = segment.unit_direction().CrossProd(point - segment.start());

  return 0;
}
int HDMapImpl::GetNearestLaneWithHeading(
    const PointENU &point, const double distance, const double central_heading,
    const double max_heading_difference, LaneInfoConstPtr *nearest_lane,
    double *nearest_s, double *nearest_l) const
{
  return GetNearestLaneWithHeading({*(point.x), *(point.y)}, distance,
                                   central_heading, max_heading_difference,
                                   nearest_lane, nearest_s, nearest_l);
}
int HDMapImpl::GetNearestLaneWithHeading(
    const Vec2d &point, const double distance, const double central_heading,
    const double max_heading_difference, LaneInfoConstPtr *nearest_lane,
    double *nearest_s, double *nearest_l) const
{
  CHECK_NOTNULL(nearest_lane);
  CHECK_NOTNULL(nearest_s);
  CHECK_NOTNULL(nearest_l);

  std::vector<LaneInfoConstPtr> lanes;
  if (GetLanesWithHeading(point, distance, central_heading,
                          max_heading_difference, &lanes) != 0)
  {
    return -1;
  }

  double s = 0;
  size_t s_index = 0;
  Vec2d map_point;
  double min_distance = distance;
  for (const auto &lane : lanes)
  {
    double s_offset = 0.0;
    int s_offset_index = 0;
    double distance =
        lane->DistanceTo(point, &map_point, &s_offset, &s_offset_index);
    if (distance < min_distance)
    {
      min_distance = distance;
      *nearest_lane = lane;
      s = s_offset;
      s_index = s_offset_index;
    }
  }

  if (*nearest_lane == nullptr)
  {
    return -1;
  }

  *nearest_s = s;
  int segment_index = static_cast<int>(
      std::min(s_index, (*nearest_lane)->segments().size() - 1));
  const auto &segment_2d = (*nearest_lane)->segments()[segment_index];
  *nearest_l =
      segment_2d.unit_direction().CrossProd(point - segment_2d.start());

  return 0;
}
int HDMapImpl::GetLanesWithHeading(const PointENU &point, const double distance,
                                   const double central_heading,
                                   const double max_heading_difference,
                                   std::vector<LaneInfoConstPtr> *lanes) const
{
  return GetLanesWithHeading({*(point.x), *(point.y)}, distance, central_heading,
                             max_heading_difference, lanes);
}
int HDMapImpl::GetLanesWithHeading(const Vec2d &point, const double distance,
                                   const double central_heading,
                                   const double max_heading_difference,
                                   std::vector<LaneInfoConstPtr> *lanes) const
{
  CHECK_NOTNULL(lanes);
  std::vector<LaneInfoConstPtr> all_lanes;
  const int status = GetLanes(point, distance, &all_lanes);
  if (status < 0 || all_lanes.size() <= 0)
  {
    return -1;
  }

  lanes->clear();
  for (auto &lane : all_lanes)
  {
    Vec2d proj_pt(0.0, 0.0);
    double s_offset = 0.0;
    int s_offset_index = 0;
    double dis = lane->DistanceTo(point, &proj_pt, &s_offset, &s_offset_index);
    if (dis <= distance)
    {
      double heading_diff =
          fabs(lane->headings()[s_offset_index] - central_heading);
      if (fabs(common::math::NormalizeAngle(heading_diff)) <=
          max_heading_difference)
      {
        lanes->push_back(lane);
      }
    }
  }

  return 0;
}
/**  Junction  **/
int HDMapImpl::GetJunctions(
    const PointENU &point, double distance,
    std::vector<JunctionInfoConstPtr> *junctions) const
{
  return GetJunctions({*point.x, *point.y}, distance, junctions);
}
int HDMapImpl::GetJunctions(
    const Vec2d &point, double distance,
    std::vector<JunctionInfoConstPtr> *junctions) const
{
  if (junctions == nullptr || junction_polygon_kdtree_ == nullptr)
  {
    return -1;
  }
  junctions->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *junction_polygon_kdtree_, &ids);
  if (status < 0)
  {
    return status;
  }
  for (const auto &id : ids)
  {
    junctions->emplace_back(GetJunctionById(id));
  }
  return 0;
}
/**  Signal  **/
int HDMapImpl::GetSignals(const PointENU &point, double distance,
                          std::vector<SignalInfoConstPtr> *signals) const
{
  return GetSignals({*point.x, *point.y}, distance, signals);
}
int HDMapImpl::GetSignals(const Vec2d &point, double distance,
                          std::vector<SignalInfoConstPtr> *signals) const
{
  if (signals == nullptr || signal_segment_kdtree_ == nullptr)
  {
    return -1;
  }
  signals->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *signal_segment_kdtree_, &ids);
  if (status < 0)
  {
    return status;
  }
  for (const auto &id : ids)
  {
    signals->emplace_back(GetSignalById(id));
  }
  return 0;
}
/**  StopSign  **/
int HDMapImpl::GetStopSigns(
    const PointENU &point, double distance,
    std::vector<StopSignInfoConstPtr> *stop_signs) const
{
  return GetStopSigns({*(point.x), *(point.y)}, distance, stop_signs);
}
int HDMapImpl::GetStopSigns(
    const Vec2d &point, double distance,
    std::vector<StopSignInfoConstPtr> *stop_signs) const
{
  if (stop_signs == nullptr || stop_sign_segment_kdtree_ == nullptr)
  {
    return -1;
  }
  stop_signs->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *stop_sign_segment_kdtree_, &ids);
  if (status < 0)
  {
    return status;
  }
  for (const auto &id : ids)
  {
    stop_signs->emplace_back(GetStopSignById(id));
  }
  return 0;
}

/******************
 * Build kdtrees
 */
void HDMapImpl::BuildLaneSegmentKDTree()
{
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0; // meters.
  params.max_leaf_size = 16;
  BuildSegmentKDTree(lane_table_, params, &lane_segment_boxes_,
                     &lane_segment_kdtree_);
}
void HDMapImpl::BuildJunctionPolygonKDTree()
{
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0; // meters.
  params.max_leaf_size = 1;
  BuildPolygonKDTree(junction_table_, params, &junction_polygon_boxes_,
                     &junction_polygon_kdtree_);
}
void HDMapImpl::BuildSignalSegmentKDTree()
{
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0; // meters.
  params.max_leaf_size = 4;
  BuildSegmentKDTree(signal_table_, params, &signal_segment_boxes_,
                     &signal_segment_kdtree_);
}
void HDMapImpl::BuildStopSignSegmentKDTree()
{
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0; // meters.
  params.max_leaf_size = 4;
  BuildSegmentKDTree(stop_sign_table_, params, &stop_sign_segment_boxes_,
                     &stop_sign_segment_kdtree_);
}

/************
 * KDTree构建模板，总共两种类型
 */
template <class Table, class BoxTable, class KDTree>
void HDMapImpl::BuildSegmentKDTree(const Table &table,
                                   const AABoxKDTreeParams &params,
                                   BoxTable *const box_table,
                                   std::unique_ptr<KDTree> *const kdtree)
{
  box_table->clear();
  for (const auto &info_with_id : table)
  {
    const auto *info = info_with_id.second.get();
    for (size_t id = 0; id < info->segments().size(); ++id)
    {
      const auto &segment = info->segments()[id];
      box_table->emplace_back(
          common::math::AABox2d(segment.start(), segment.end()), info,
          &segment, id);
    }
  }
  kdtree->reset(new KDTree(*box_table, params));
}

template <class Table, class BoxTable, class KDTree>
void HDMapImpl::BuildPolygonKDTree(const Table &table,
                                   const AABoxKDTreeParams &params,
                                   BoxTable *const box_table,
                                   std::unique_ptr<KDTree> *const kdtree)
{
  box_table->clear();
  for (const auto &info_with_id : table)
  {
    const auto *info = info_with_id.second.get();
    const auto &polygon = info->polygon();
    box_table->emplace_back(polygon.AABoundingBox(), info, &polygon, 0);
  }
  kdtree->reset(new KDTree(*box_table, params));
}

/************
 * KDTree查找模板
 */
template <class KDTree>
int HDMapImpl::SearchObjects(const Vec2d &center, const double radius,
                             const KDTree &kdtree,
                             std::vector<std::string> *const results)
{
  if (results == nullptr)
  {
    return -1;
  }
  auto objects = kdtree.GetObjects(center, radius);
  std::unordered_set<std::string> result_ids;
  result_ids.reserve(objects.size());
  for (const auto *object_ptr : objects)
  {
    result_ids.insert(object_ptr->object()->id());
  }

  results->reserve(result_ids.size());
  results->assign(result_ids.begin(), result_ids.end());
  return 0;
}

void HDMapImpl::Clear()
{
  //TODO: map_.clear();
  lane_table_.clear();
  lane_segment_boxes_.clear();
  lane_segment_kdtree_.reset(nullptr);
  junction_table_.clear();
  junction_polygon_boxes_.clear();
  junction_polygon_kdtree_.reset(nullptr);
  stop_sign_table_.clear();
  stop_sign_segment_boxes_.clear();
  stop_sign_segment_kdtree_.reset(nullptr);
  signal_table_.clear();
  signal_segment_boxes_.clear();
  signal_segment_kdtree_.reset(nullptr);
}