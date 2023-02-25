//
// Created by luyifan on 19-6-18.
//

#ifndef HDMAP_IMPL_H
#define HDMAP_IMPL_H

#include "ObjectInfo.h"
#include "common/utils/find_path.h"
#include "json_adapter/json_adapter.h"

namespace hdmap
{
    /**
     * @class HDMapImpl
     *
     * @brief High-precision map loader implement.
     */
    class HDMapImpl
    {
    public:
        using LaneTable =
            std::unordered_map<std::string, std::shared_ptr<LaneInfo>>;
        using SignalTable =
            std::unordered_map<std::string, std::shared_ptr<SignalInfo>>;
        using StopSignTable =
            std::unordered_map<std::string, std::shared_ptr<StopSignInfo>>;
        using JunctionTable =
            std::unordered_map<std::string, std::shared_ptr<JunctionInfo>>;
        using OverlapTable =
            std::unordered_map<std::string, std::shared_ptr<OverlapInfo>>;

    public:
        int LoadMap(const std::string &filename,
                    const double& utm_origin_x,
                    const double& utm_origin_y);

        LaneInfoConstPtr GetLaneById(const string &id) const;
        JunctionInfoConstPtr GetJunctionById(const std::string &id) const;
        SignalInfoConstPtr GetSignalById(const std::string &id) const;
        StopSignInfoConstPtr GetStopSignById(const std::string &id) const;
        OverlapInfoConstPtr GetOverlapById(const std::string &id) const;

        int GetLanes(const common::PointENU &point, double distance,
                    std::vector<LaneInfoConstPtr> *lanes) const;
        int GetNearestLane(const common::PointENU &point,
                        LaneInfoConstPtr *nearest_lane, double *nearest_s,
                        double *nearest_l) const;
        int GetNearestLaneWithHeading(const common::PointENU &point,
                                    const double distance,
                                    const double central_heading,
                                    const double max_heading_difference,
                                    LaneInfoConstPtr *nearest_lane,
                                    double *nearest_s, double *nearest_l) const;
        int GetLanesWithHeading(const common::PointENU &point,
                                const double distance, const double central_heading,
                                const double max_heading_difference,
                                std::vector<LaneInfoConstPtr> *lanes) const;
        int GetJunctions(const common::PointENU &point, double distance,
                        std::vector<JunctionInfoConstPtr> *junctions) const;
        int GetSignals(const common::PointENU &point, double distance,
                    std::vector<SignalInfoConstPtr> *signals) const;
        int GetStopSigns(const common::PointENU &point, double distance,
                        std::vector<StopSignInfoConstPtr> *stop_signs) const;

        //Get tables function
        inline const LaneTable &GetLaneTable() { return lane_table_; };
        inline const JunctionTable &GetJunctionTable() { return junction_table_; };
        inline const SignalTable &GetSignalTable() { return signal_table_; };
        inline const StopSignTable &GetStopSignTable() { return stop_sign_table_; };
        inline const OverlapTable &GetOverlapTable() { return overlap_table_; };

        int LoadElements(const Map &map);
        int GetLanes(const common::math::Vec2d &point, double distance,
                    std::vector<LaneInfoConstPtr> *lanes) const;
        int GetNearestLane(const common::math::Vec2d &point,
                        LaneInfoConstPtr *nearest_lane, double *nearest_s,
                        double *nearest_l) const;
        int GetNearestLaneWithHeading(const common::math::Vec2d &point,
                                    const double distance,
                                    const double central_heading,
                                    const double max_heading_difference,
                                    LaneInfoConstPtr *nearest_lane,
                                    double *nearest_s, double *nearest_l) const;
        int GetLanesWithHeading(const common::math::Vec2d &point,
                                const double distance, const double central_heading,
                                const double max_heading_difference,
                                std::vector<LaneInfoConstPtr> *lanes) const;
        int GetJunctions(const common::math::Vec2d &point, double distance,
                        std::vector<JunctionInfoConstPtr> *junctions) const;
        int GetSignals(const common::math::Vec2d &point, double distance,
                    std::vector<SignalInfoConstPtr> *signals) const;
        int GetStopSigns(const common::math::Vec2d &point, double distance,
                        std::vector<StopSignInfoConstPtr> *stop_signs) const;

    private:
        template <class Table, class BoxTable, class KDTree>
        static void BuildSegmentKDTree(
            const Table &table, const common::math::AABoxKDTreeParams &params,
            BoxTable *const box_table, std::unique_ptr<KDTree> *const kdtree);

        template <class Table, class BoxTable, class KDTree>
        static void BuildPolygonKDTree(
            const Table &table, const common::math::AABoxKDTreeParams &params,
            BoxTable *const box_table, std::unique_ptr<KDTree> *const kdtree);

        void BuildLaneSegmentKDTree();
        void BuildJunctionPolygonKDTree();
        void BuildSignalSegmentKDTree();
        void BuildStopSignSegmentKDTree();

        template <class KDTree>
        static int SearchObjects(const common::math::Vec2d &center,
                                const double radius, const KDTree &kdtree,
                                std::vector<std::string> *const results);

        void Clear();

    private:
        Map map_;
        LaneTable lane_table_;
        JunctionTable junction_table_;
        SignalTable signal_table_;
        StopSignTable stop_sign_table_;
        OverlapTable overlap_table_;

        std::vector<LaneInfo::LaneSegmentBox> lane_segment_boxes_;
        std::unique_ptr<LaneInfo::LaneSegmentKDTree> lane_segment_kdtree_;

        std::vector<JunctionPolygonBox> junction_polygon_boxes_;
        std::unique_ptr<JunctionPolygonKDTree> junction_polygon_kdtree_;

        std::vector<SignalSegmentBox> signal_segment_boxes_;
        std::unique_ptr<SignalSegmentKDTree> signal_segment_kdtree_;

        std::vector<StopSignSegmentBox> stop_sign_segment_boxes_;
        std::unique_ptr<StopSignSegmentKDTree> stop_sign_segment_kdtree_;
    };
} // namespace hdmap

#endif // HDMAP_IMPL_H
