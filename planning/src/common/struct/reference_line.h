//
// Created by Chen Xiaofeng on 19-11-1.
//

#ifndef STATEMACHINE_REFERENCELINE_H
#define STATEMACHINE_REFERENCELINE_H
#include "trajectory.h"
#include "tool.h"
#include "common/struct/scenario.h"
#include "common/utils/spline.h"

namespace planning
{
    class ReferenceLine
    {
    private:
        Trajectory central_line_;
        Trajectory smoother_central_line_;
        hdmap::Lane lane_;
        std::shared_ptr<hdmap::LaneInfo> lane_info_;
        std::vector<Trajectory> derived_trajectories_;

        int reference_line_id_;
        std::vector<std::string> lane_ids_;
        int priority_ = 0;                // 当前优先级，值越大优先级越高
        double lane_width_ = 100;         // 当前最小车宽
        double expected_speed_ = 6 / 3.6; // 当前期望速度
        double localization_offset_ = 0;  // 参考线跟当前定位的横向距离，左边为正，右边为负
        bool left_virtual_ = false;       // 左边是否是虚线，是虚线为true
        bool right_virtual_ = false;      // 右边是否是虚线，是虚线为true
        Scenario scenario_;               // 下一个最近的场景
        bool vehicle_on_ = false;         // 是否是车辆当前所在的参考线，是的话为true
        double length_ = 0;               // 参考线长度

        double distance2signal = MY_INF;
        double distance2stopsign = MY_INF;

        void BuildLane();
        void SpineByFourPoints(std::vector<TrajectoryPoint> &raw_four_points_, std::vector<TrajectoryPoint> &smoother_four_points_);

    public:
        ReferenceLine(int reference_line_id);
        ReferenceLine() = default;
        ~ReferenceLine();
        const inline int reference_line_id() const { return reference_line_id_; }
        inline void SetPriority(int priority) { priority_ = priority; }
        const inline int priority() const { return priority_; }
        inline void SetLaneWidth(double lane_width) { lane_width_ = lane_width; }
        inline double lane_width() const { return lane_width_; }
        inline void SetExpectedSpeed(double expected_speed) { expected_speed_ = expected_speed; }
        inline double expected_speed() const { return expected_speed_; }
        inline void SetLocalizationOffset(double localization_offset) { localization_offset_ = localization_offset; }
        inline double localization_offset() const { return localization_offset_; }
        inline void SetLeftVirtual(bool left_virtual) { left_virtual_ = left_virtual; }
        inline bool left_virtual() const { return left_virtual_; }
        inline void SetRightVirtual(bool right_virtual) { right_virtual_ = right_virtual; }
        inline bool right_virtual() const { return right_virtual_; }
        const inline Scenario scenario() const { return scenario_; }
        inline void set_scenario(Scenario scenario) { scenario_ = scenario; }
        inline void set_scenario_task_end_point(TaskEndPoint task_end_point) { scenario_.task_end_point = task_end_point; }
        inline void set_scenario_reverse_end_point(TaskEndPoint reverse_end_point) { scenario_.reverse_end_point = reverse_end_point; }
        inline void SetVehicleOn(bool vehicle_on) { vehicle_on_ = vehicle_on; }
        inline bool vehicle_on() const { return vehicle_on_; }
        inline void SetLength(double length) { length_ = length; }
        inline double length() const { return length_; }
        void GetSmoother();

        inline std::vector<std::string> &lane_ids() { return lane_ids_; }
        inline const Trajectory &central_line() const { return central_line_; }
        inline const Trajectory &smoother_central_line() const { return smoother_central_line_; }
        inline std::vector<Trajectory> &mutable_derived_trajectories() { return derived_trajectories_; }
        inline const std::vector<Trajectory> &derived_trajectories() const { return derived_trajectories_; }
        // bool PlanOnReferenceCentralLine(const TrajectoryPoint & localization_point, double velocity, double lane_width);
        bool CreateCentralTrajectory(const std::vector<TrajectoryPoint> &trajectory, const double referenceline_speed_);
        bool RecordLaneIds(const std::vector<std::string> &lane_ids);
        void GetUTM(const double &accumulate_s, const double &lateral, double *utm_x, double *utm_y);
        void GetFrenet(const double &utm_x, const double &utm_y, const double &heading, double *accumulate_s, double *lateral);
        void GetFrenet(const double &utm_x, const double &utm_y, double *accumulate_s, double *lateral);
        double Heading(const double s);
        inline double length() { return lane_info_->total_length(); }
        inline double GetMaxS()
        {
            double number_of_points = central_line_.points().size();
            if (number_of_points != 0)
            {
                return (central_line_.points()[number_of_points - 1]).s;
            }
        };
        inline const std::shared_ptr<hdmap::LaneInfo> &lane_info() const { return lane_info_; }
        const double Distance2Task(const double &utm_x, const double &utm_y);
    };
} // namespace planning

#endif //STATEMACHINE_REFERENCELINE_H
