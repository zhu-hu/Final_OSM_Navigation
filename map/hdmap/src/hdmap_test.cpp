//
// Created by Chen Xiaofeng on 19-10-20.
//

#include "Impl.h"
#include <chrono>

using namespace hdmap;

int main(int argc, char **argv)
{
    std::string log_file_path = expand_catkin_ws("/LOG/");
    google::InitGoogleLogging("HDMap");
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_dir = log_file_path;
    AINFO << "Log initiate success! ";

    // -------------------------- example for using hdmap ----------------------- //
    AINFO << "HDmap test begin! ";
    std::string map_file_path = expand_catkin_ws("/src/map/hdmap/data/BJ13/BJ13.json");
    HDMapImpl *map_json = new HDMapImpl();
    auto start = std::chrono::system_clock::now();
    if (map_json->LoadMap(map_file_path) != 1)
    {
        AERROR << "error when load hdmap!!!";
        return 0;
    }
    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    double time_us = double(duration.count());
    AINFO << "Loading map costs " << time_us << " us.";

    // // set test point longitude and latitude, lane 2_-2 last point
    // double longitude = 120.77441754162794;
    // double latitude = 31.592835693947897;
    // char zone;
    // // get utm x and y of test point
    // double x_temp, y_temp;
    // LLtoUTM(latitude,longitude,y_temp,x_temp,&zone);
    // common::PointENU point;
    // point.x = x_temp;
    // point.y = y_temp;
    // point.z = 0;

    // // get lanes by distance
    // std::vector<LaneInfoConstPtr> lanes;
    // start = std::chrono::system_clock::now();
    // map_json->GetLanes(point, 10, &lanes);
    // end = std::chrono::system_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // time_us = double(duration.count());
    // AINFO << "Getting lanes by distance costs " << time_us << " us.";
    // for(auto lane : lanes)
    // {
    //     AINFO << "Get lanes: " << lane->id() << " within 10 meters of last point of lane 2_-2.";
    // }

    // // get lanes by distance with heading, east:x ; north:y
    // lanes.clear();
    // start = std::chrono::system_clock::now();
    // map_json->GetLanesWithHeading(point, 10, M_PI/4, M_PI/3, &lanes);
    // end = std::chrono::system_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // time_us = double(duration.count());
    // AINFO << "Getting lanes by distance with heading costs " << time_us << " us.";
    // for(auto lane : lanes)
    // {
    //     AINFO << "Get lanes: " << lane->id() << "within 10 meters of last point of lane 2_-2 with heading.";
    // }

    // // get nearest lane
    // LaneInfoConstPtr nearest_lane;
    // double nearest_s = 0;
    // double nearest_l = 0;
    // start = std::chrono::system_clock::now();
    // map_json->GetNearestLane(point, &nearest_lane, &nearest_s, &nearest_l);
    // end = std::chrono::system_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // time_us = double(duration.count());
    // AINFO << "Getting nearest lane costs " << time_us << " us.";
    // AINFO << "last point of lane 2_-2 " << "Nearest lane: " << nearest_lane->id() << " with s: " << nearest_s << " and l: " << nearest_l;
    // // get stop_sign by distance by overlaps
    // std::vector<string> overlap_ids = nearest_lane->overlap_ids();
    // for(auto overlap_id : overlap_ids)
    // {
    //     OverlapInfoConstPtr overlap = map_json->GetOverlapById(overlap_id);
    //     AINFO << "overlap id: " << overlap->id() << " object2: " << overlap->object2_id() << " start: " << overlap->start_s() << " end: " << overlap->end_s();
    // }

    // // get nearest lane with heading
    // nearest_s = 0;
    // nearest_l = 0;
    // start = std::chrono::system_clock::now();
    // map_json->GetNearestLaneWithHeading(point, 10, M_PI*5/4, M_PI/3, &nearest_lane, &nearest_s, &nearest_l);
    // end = std::chrono::system_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // time_us = double(duration.count());
    // AINFO << "Getting nearest lane with heading costs " << time_us << " us.";
    // AINFO << "last point of lane 2_-2 with reverse heading " << "Nearest lane: " << nearest_lane->id() << " with s: " << nearest_s << " and l: " << nearest_l;

    // // get junctions by distance
    // std::vector<JunctionInfoConstPtr> junctions;
    // start = std::chrono::system_clock::now();
    // map_json->GetJunctions(point, 10, &junctions);
    // end = std::chrono::system_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // time_us = double(duration.count());
    // AINFO << "Getting junctions by distance costs " << time_us << " us.";
    // for(auto junction : junctions)
    // {
    //     AINFO << "Get junctions: " << junction->id() << " within 10 meters of last point of lane 2_-2.";
    // }

    // // get stop_sign by distance
    // std::vector<StopSignInfoConstPtr> stop_signs;
    // start = std::chrono::system_clock::now();
    // map_json->GetStopSigns(point, 10, &stop_signs);
    // end = std::chrono::system_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // time_us = double(duration.count());
    // AINFO << "Getting stop signs by distance costs " << time_us << " us.";
    // for(auto stop_sign : stop_signs)
    // {
    //     AINFO << "Get stop signs: " << stop_sign->id() << " within 10 meters of last point of lane 2_-2.";
    // }

    AINFO << "HDmap test finished!";
}