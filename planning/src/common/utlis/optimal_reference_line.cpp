#include "optimal_reference_line.h"

namespace planning
{
    struct Element
    {
        double distance_to_colliion;
        double adc_l;
        int priority;
        const ReferenceLine *line;
    };
    std::vector<ReferenceLine> OptimalReferenceLine(std::vector<ReferenceLine> &reference_lines,
                                                    const std::vector<PredictionObstacle> &obstacles,
                                                    const cyber_msgs::LocalizationEstimate &adc_state)
    {

        ReferenceLine current_reference_line = reference_lines.front();
        int max_priority = reference_lines.front().priority();
        for (auto &reference_line : reference_lines)
        {
            if (reference_line.priority() > max_priority)
            {
                current_reference_line = reference_line;
                max_priority = reference_line.priority();
            }
        }
        return std::vector<ReferenceLine>{current_reference_line};
    }
    // std::vector<ReferenceLine> OptimalReferenceLine(std::vector<ReferenceLine>& reference_lines,
    //                                                 const std::vector<PredictionObstacle>& obstacles,
    //                                                 const cyber_msgs::LocalizationEstimate& adc_state ){

    //     ReferenceLine current_reference_line;
    //     //先找可行域最长的
    //     std::vector<Element> lines_feasible_region;
    //     int max_tmp_priority = 0;
    //     for (auto& reference_line : reference_lines){
    //         double adc_s, adc_l;
    //         double adc_heading = normalize_angle(tf::getYaw(adc_state.pose.orientation));
    //         reference_line.GetFrenet(adc_state.pose.position.x, adc_state.pose.position.y, adc_heading, &adc_s, &adc_l);
    //         if (reference_line.vehicle_on()){
    //             if(reference_line.priority()>=max_tmp_priority){
    //                 current_reference_line = reference_line;
    //                 max_tmp_priority = reference_line.priority();
    //             }
    //         }
    //         double distance_to_collision = std::numeric_limits<double>::max();
    //         for(const auto& obs : obstacles){
    //             //不考虑动态障碍物
    //             if(obs.motion_status == DYNAMIC)
    //                 continue;
    //             //不考虑后方的障碍物
    //             double obs_s, obs_l;
    //             reference_line.GetFrenet(obs.position.x, obs.position.y, obs.theta, &obs_s, &obs_l);
    //             if(obs_s < adc_s)
    //                 continue;
    //             //不考虑ReferenceLine以外的障碍物
    //             if(fabs(obs_l) > reference_line.lane_width() / 3.0)
    //                 continue;
    //             //不考虑50米前的静态障碍物
    //             if(obs_s - 50.0 > adc_s)
    //                 continue;
    //             distance_to_collision = obs_s - adc_s;
    //             // AINFO<<"obs: "<<obs.id<<" vel:"<<obs.velocity<<" age: "<<obs.age<<" l: "<<fabs(obs_l)<<" adc_l: "<<fabs(adc_l);
    //         }
    //         //AINFO<<"distance_to_collision: "<<distance_to_collision;
    //         if(lines_feasible_region.empty()){
    //             Element e;
    //             e.distance_to_colliion = distance_to_collision;
    //             e.adc_l = adc_l;
    //             e.priority = reference_line.priority();
    //             e.line = &reference_line;
    //             lines_feasible_region.emplace_back(e);
    //         }
    //         else if(lines_feasible_region.front().distance_to_colliion < distance_to_collision){
    //             lines_feasible_region.clear();
    //             Element e;
    //             e.distance_to_colliion = distance_to_collision;
    //             e.adc_l = adc_l;
    //             e.priority = reference_line.priority();
    //             e.line = &reference_line;
    //             lines_feasible_region.emplace_back(e);
    //         }
    //         else if(lines_feasible_region.front().distance_to_colliion == distance_to_collision){
    //             Element e;
    //             e.distance_to_colliion = distance_to_collision;
    //             e.adc_l = adc_l;
    //             e.priority = reference_line.priority();
    //             e.line = &reference_line;
    //             lines_feasible_region.emplace_back(e);
    //         }
    //     }
    //     if(lines_feasible_region.size() == 1){
    //         // AINFO<<"1";
    //         return std::vector<ReferenceLine>{current_reference_line, *lines_feasible_region.front().line};
    //     }
    //     //再找优先级高的
    //     std::vector<Element> lines_priority;
    //     // int i=0;
    //     for(const auto& reference_line : lines_feasible_region){
    //         // AINFO<<"referenceLine num: "<<i<<" priority: "<<reference_line.priority;
    //         if(lines_priority.empty()){
    //             lines_priority.emplace_back(reference_line);
    //         }
    //         else if(reference_line.priority > lines_priority.front().priority){
    //             lines_priority.clear();
    //             lines_priority.emplace_back(reference_line);
    //         }
    //         else if(reference_line.priority == lines_priority.front().priority){
    //             lines_priority.emplace_back(reference_line);
    //         }
    //     }
    //     if(lines_priority.size() == 1){
    //         // AINFO<<"2: current priority: "<<current_reference_line.priority()<<" back's priority: "<<lines_priority.front().priority;
    //         return std::vector<ReferenceLine>{current_reference_line, *lines_priority.front().line};
    //     }
    //     //再找最靠近我的
    //     std::vector<Element> lines_closed;
    //     for(const auto& reference_line : lines_priority){
    //         if(lines_closed.empty()){
    //             lines_closed.emplace_back(reference_line);
    //         }
    //         else if(fabs(reference_line.adc_l) < fabs(lines_closed.front().adc_l)){
    //             lines_closed.clear();
    //             lines_closed.emplace_back(reference_line);
    //         }
    //         else if(fabs(reference_line.adc_l) == fabs(lines_closed.front().adc_l)){
    //             lines_closed.emplace_back(reference_line);
    //         }
    //     }
    //     if(lines_closed.size() == 1){
    //         // AINFO<<"3";
    //         return std::vector<ReferenceLine>{current_reference_line, *lines_closed.front().line};
    //     }
    //     //最后选左边的
    //     if(lines_closed.size() > 2)
    //         AERROR<<"Candidate lines should be <= 2. here is "<<lines_closed.size();
    //     else{
    //         for(const auto& reference_line : lines_closed){
    //             if(reference_line.adc_l > 0){
    //                 return std::vector<ReferenceLine>{current_reference_line, *reference_line.line};
    //             }
    //         }
    //     }
    //     //最后返回两条一样的
    //     return std::vector<ReferenceLine>{current_reference_line, current_reference_line};
    // }
} // namespace planning