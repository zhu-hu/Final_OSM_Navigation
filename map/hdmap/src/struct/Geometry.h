//
// Created by luyifan on 19-6-15.
//

#ifndef HDMAP_GEOMETRY_H
#define HDMAP_GEOMETRY_H

#include "common/utils/find_path.h"
#include "Attribute.h"
#include "../math/Polygon.h"
typedef xsd::d_double d_double;
typedef xsd::d_int d_int;
typedef xsd::d_float d_float;
typedef xsd::d_string d_string;
typedef xsd::d_bool d_bool;
using std::string;
using xsd::Attribute;


namespace common {
  struct PointENU {
    d_double x;// East from the origin, in meters.
    d_double y;// North from the origin, in meters.
    d_double z;// Up from the WGS-84 ellipsoid, in meters.
  };

  // A point in the global reference frame. Similar to PointENU, PointLLH allows
  // omitting the height field for representing a 2D location.
  struct PointLLH {
    d_double lon;// Longitude in degrees, ranging from -180 to 180.
    d_double lat;// Latitude in degrees, ranging from -90 to 90.
    d_double height;// WGS-84 ellipsoid height in meters.
  };

  // A general 2D point. Its meaning and units depend on context, and must be
  // explained in comments.
  struct Point2D {
    d_double x;
    d_double y;
  };

  // A general 3D point. Its meaning and units depend on context, and must be
  // explained in comments.
  struct Point3D {
    d_double x;
    d_double y;
    d_double z;
  };

  // A unit quaternion that represents a spatial rotation. See the link below for details.
  // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
  // The scalar part qw can be omitted. In this case, qw should be calculated by
  // qw = sqrt(1 - qx * qx - qy * qy - qz * qz).
  struct Quaternion {
    d_double qx;
    d_double qy;
    d_double qz;
    d_double qw;
  };

  struct Polygon {
    std::vector <Point3D> points;
  };
}

namespace hdmap{

  struct ROIAttribute {
    enum class PolygonType {
      JUNCTION_POLYGON = 0,
      PARKINGSPACE_POLYGON = 1,
      ROAD_HOLE_POLYGON = 2,
    };
    xsd::Attribute<PolygonType> type;
    d_string id;
  };

  struct PolygonROI {
    xsd::Attribute<common::math::Polygon2d> polygon;
    xsd::Attribute<ROIAttribute> attribute;
  };

  struct Polygon{
    std::vector<common::PointENU> points;
  };

  struct LineSegment{
    std::vector<common::PointENU> points;
  };

  struct CurveSegment{
    xsd::Attribute<LineSegment> line;
    d_double start_s;// start position (s-coordinate)
    xsd::Attribute<common::PointENU> start_position;
    d_double start_heading;// start orientation
    d_double length;
  };

  struct Curve{
    std::vector<CurveSegment> segments;
  };

  struct LineBoundary {
    std::vector<common::PointENU> line_points;
  };

  struct PolygonBoundary {
    std::vector<common::PointENU> polygon_points;
  };
}

#endif //HDMAP_GEOMETRY_H
