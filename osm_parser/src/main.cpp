#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "osm_parser.h"
#include "path_finder_algorithm/dijkstra.h"

std::vector<int> path_ids;
ros::Publisher pub_path;
ros::Publisher pub_point;

void publishPlannerPath(
    const std::vector<osm_parser::Parser::OSM_NODE>& nodes) {
  visualization_msgs::MarkerArray point_markers;
  visualization_msgs::Marker point_marker;
  int id_count = 0;
  point_marker.header.frame_id = "world";
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "path_points";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.type = visualization_msgs::Marker::ARROW;
  point_marker.lifetime = ros::Duration(0);
  // Scale
  point_marker.scale.x = 3.8;
  point_marker.scale.y = 1.8;
  point_marker.scale.z = 1.8;
  // Color
  point_marker.color.r = 0.0;
  point_marker.color.g = 0.0;
  point_marker.color.b = 1.0;

  for (int i = 0; i < path_ids.size() - 1; i++) {
    point_marker.id = id_count++;
    // point_marker.color.a = 1.0 * (i + 1) / (path_ids.size() + 1);
    point_marker.color.a = 1.0;
    point_marker.pose.position.x = nodes[path_ids[i]].utm_x;
    point_marker.pose.position.y = nodes[path_ids[i]].utm_y;
    point_marker.pose.position.z = 0.0;
    double delta_x = nodes[path_ids[i + 1]].utm_x - nodes[path_ids[i]].utm_x;
    double delta_y = nodes[path_ids[i + 1]].utm_y - nodes[path_ids[i]].utm_y;
    double yaw = atan2(delta_y, delta_x);
    auto tf_q = tf::createQuaternionFromYaw(yaw);
    point_marker.pose.orientation.x = tf_q.getX();
    point_marker.pose.orientation.y = tf_q.getY();
    point_marker.pose.orientation.z = tf_q.getZ();
    point_marker.pose.orientation.w = tf_q.getW();
    point_markers.markers.emplace_back(point_marker);
    std::cout << "(" << path_ids[i] << ")" << std::endl;
  }
  point_marker.id = id_count++;
  point_marker.color.a = 1.0;
  point_marker.pose.position.x = nodes[path_ids.back()].utm_x;
  point_marker.pose.position.y = nodes[path_ids.back()].utm_y;
  point_marker.pose.position.z = 0.0;
  point_marker.pose.orientation = point_markers.markers.back().pose.orientation;
  point_markers.markers.emplace_back(point_marker);
  std::cout << "(" << path_ids[path_ids.size() - 1] << ")" << std::endl;

  pub_path.publish(point_markers);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_osm_parser");

  ros::NodeHandle nh("~");

  pub_path = nh.advertise<visualization_msgs::MarkerArray>("/planner_path", 5);

  //   OsmPlannerNode osm_planner;
  std::shared_ptr<osm_parser::Parser> map;

  osm_parser::path_finder_algorithm::Dijkstra dijkstra_planner;

  bool path_found = false;

  map = std::make_shared<osm_parser::Parser>();

  std::string file = "skuska.osm";
  nh.getParam("osm_map_path", file);
  map->setNewMap(file);

  std::vector<std::string> types_of_ways{"service"};
  nh.getParam("osm_parser/filter_of_ways", types_of_ways);
  map->setTypeOfWays(types_of_ways);

  // // Set the density of points
  double interpolation_max_distance;
  nh.param<double>("osm_parser/interpolation_max_distance",
                   interpolation_max_distance, 10.0);
  map->setInterpolationMaxDistance(interpolation_max_distance);

  map->parse();

  int source_id = 0;
  nh.param<int>("osm_parser/source_id", source_id, 0);
  int target_id = map->osm_nodes().size();
  nh.param<int>("osm_parser/target_id", target_id, map->osm_nodes().size());
  double direction = 0.0;
  nh.param<double>("osm_parser/direction", direction, 0.0);
  if (target_id >= map->osm_nodes().size()) {
    target_id = map->osm_nodes().size() - 1;
  }
  if (source_id >= map->osm_nodes().size()) {
    source_id = map->osm_nodes().size() - 1;
  }

  std::cout << "source_id : " << source_id << std::endl;
  std::cout << "target_id : " << target_id << std::endl;

  map->deleteWrongDirectionEdgeOnGraph(source_id, direction);

  try {
    double start_time = ros::Time::now().toSec();
    path_ids = dijkstra_planner.findShortestPath(map->getGraphOfVertex(),
                                                 source_id, target_id);
    path_found = true;

    ROS_INFO("OSM planner: Time of planning : %f  ms",
             1000.0 * (ros::Time::now().toSec() - start_time));

  } catch (osm_parser::path_finder_algorithm::PathFinderException& e) {
    if (e.getErrId() ==
        osm_parser::path_finder_algorithm::PathFinderException::NO_PATH_FOUND) {
      ROS_ERROR("OSM planner: Make plan failed...");
    } else
      ROS_ERROR("OSM planner: Undefined error");
  }

  ros::Rate rate(1);

  while (ros::ok()) {
    // osm_planner.update();
    // map->publishRouteNetwork();
    if (path_found) {
      publishPlannerPath(map->osm_nodes());
    }
    // map->publishMapPoint();
    // map->publishMapArrow();
    map->publishMapArray();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}