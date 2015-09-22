#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/exceptions.h>
#include <tf/transform_datatypes.h>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <algorithm>
#include <ros/assert.h>

namespace occupancy_grid_utils
{

namespace nm=nav_msgs;
namespace gm=geometry_msgs;
namespace sm=sensor_msgs;

using boost::bind;
using boost::ref;
using boost::optional;

RayTraceIterRange rayTrace (const Cell& c1, const Cell& c2)
{
  return RayTraceIterRange(RayTraceIterator(c1,c2), RayTraceIterator(c2,c2,true));
}

inline
bool cellWithinBounds (const nm::MapMetaData& info, const Cell& c)
{
  return withinBounds(info, c);
}

optional<Cell> rayTraceOntoGrid (const nm::MapMetaData& info, const Cell& c1, const Cell& c2)
{
  RayTraceIterRange r = rayTrace(c2, c1);
  RayTraceIterator pos = std::find_if (r.first, r.second, bind(cellWithinBounds, boost::ref(info), _1));
  optional<Cell> c;
  if (pos!=r.second)
    c = *pos;
  return c;
}

RayTraceIterRange rayTrace (const nm::MapMetaData& info, const gm::Point& p1, const gm::Point& p2,
                            bool project_onto_grid, bool project_source_onto_grid, 
                            float max_range)
{

  gm::Point np2 = p2;
  if (max_range > 0) {
    double distance = euclideanDistance(p1,p2);
    if (distance > max_range) {
      np2.x = ((p2.x - p1.x) * max_range/distance) + p1.x;
      np2.y = ((p2.y - p1.y) * max_range/distance) + p1.y;
    }
  }

  Cell c1 = pointCell(info, p1);
  Cell c2 = pointCell(info, np2);
  ROS_DEBUG_STREAM_NAMED ("ray_trace", "Ray tracing between " << c1.x <<
                          ", " << c1.y << " and " << c2.x << ", " << c2.y);
  const RayTraceIterator done(c1, c1, true);
  const RayTraceIterRange empty_range(done, done);
  if (!withinBounds(info, c1)) {
    if (project_source_onto_grid) {
      const optional<Cell> c = rayTraceOntoGrid(info, c2, c1);
      if (c)
        c1 = *c;
      else
        return empty_range;
    }
    else
      throw PointOutOfBoundsException(p1);
  }
  
  if (!withinBounds(info, np2)) {
    if (project_onto_grid) {
      const optional<Cell> c = rayTraceOntoGrid(info, c1, c2);
      if (c)
        c2 = *c;
      else
        return empty_range;
    }
    else {
      throw PointOutOfBoundsException(np2);
    }
  }

  ROS_DEBUG_STREAM_NAMED ("ray_trace", "Projected ray trace endpoints to " << c1.x <<
                          ", " << c1.y << " and " << c2.x << ", " << c2.y);
  return rayTrace(c1, c2);
}


gm::Point rayEndPoint (const gm::Point& p0, const double theta, const double d)
{
  gm::Point p;
  p.x = p0.x + cos(theta)*d;
  p.y = p0.y + sin(theta)*d;
  return p;
}                       

sm::LaserScan::Ptr
simulateRangeScan (const nm::OccupancyGrid& grid, const gm::Pose& sensor_pose,
                   const sm::LaserScan& scanner_info, const bool unknown_obstacles)
{
  sm::LaserScan::Ptr result(new sm::LaserScan(scanner_info));

  const double angle_range = scanner_info.angle_max - scanner_info.angle_min;
  const unsigned n =
    (unsigned) round(1+angle_range/scanner_info.angle_increment);
  const gm::Point& p0 = sensor_pose.position;
  const Cell c0 = pointCell(grid.info, p0);
  const double theta0 = tf::getYaw(sensor_pose.orientation);
  result->ranges.resize(n);

  for (unsigned i=0; i<n; i++)
  {
    const double theta = scanner_info.angle_min+i*scanner_info.angle_increment;
    const gm::Point scan_max =
      rayEndPoint(p0, theta0 + theta, scanner_info.range_max+1);
    
    result->ranges[i] = scanner_info.range_max+1; // Default if loop terminates
    BOOST_FOREACH (const Cell& c, rayTrace(grid.info, p0, scan_max, true))
    {
      const gm::Point p = cellCenter(grid.info, c);
      const double d = sqrt(pow(p.x-p0.x, 2) + pow(p.y-p0.y, 2));
      char data = grid.data[cellIndex(grid.info, c)];
      if (d > scanner_info.range_max)
        break;
      else if (data == OCCUPIED && !(c==c0))
      {
        result->ranges[i] = d;
        break;
      }
      else if (data == UNKNOWN && !(c==c0))
      {
        result->ranges[i] = unknown_obstacles ? d : scanner_info.range_max+1;
        break;
      }
    }
  }

  return result;
}


} // namespace occupancy_grid_utils
