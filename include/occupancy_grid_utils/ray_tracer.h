#ifndef OCCUPANCY_GRID_UTILS_RAY_TRACER_H
#define OCCUPANCY_GRID_UTILS_RAY_TRACER_H

#include <occupancy_grid_utils/impl/ray_trace_iterator.h>
#include <occupancy_grid_utils/LocalizedCloud.h>
#include <occupancy_grid_utils/OverlayClouds.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/optional.hpp>
#include <string>

namespace occupancy_grid_utils
{

/************************************************************
 * Basic ray tracing
 ************************************************************/

typedef std::pair<RayTraceIterator, RayTraceIterator> RayTraceIterRange;

inline double euclideanDistance (const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
  const double dx=p1.x-p2.x;
  const double dy=p1.y-p2.y;
  const double dz=p1.z-p2.z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

/// \brief Returns an iterator range over the cells on the 
/// line segment between two points (inclusive).  
/// \param project_target_onto_grid If true, \a p2 may be off the grid, in which case the ray
/// stops right before falling off the grid
/// \param project_source_onto_grid If true, \a p1 may be off the grid, in which case the
/// ray starts at the point where it enters the grid
/// \param max_range The maximum range to raycast a point out to
/// \throws PointOutOfBoundsException if \a p1 is off grid and project_source_onto_grid is false
/// \throws PointOutOfBoundsException if \a p2 is off grid and project_target_onto_grid is false
/// \retval range models
/// <a href="http://www.boost.org/doc/libs/1_42_0/libs/range/doc/range.html#forward_range">Forward range</a> 
/// with reference type const Cell&, i.e., it's a pair of iterators that you can use 
/// in for loops, std::for_each, etc.
RayTraceIterRange rayTrace (const nav_msgs::MapMetaData& info, const geometry_msgs::Point& p1, 
                            const geometry_msgs::Point& p2, bool project_target_onto_grid=false,
                            bool project_source_onto_grid=false, float max_range = -1);


/************************************************************
 * Simulating scans
 ***********************************************************/

/// \brief Simulate a planar laser range scan
///
/// \param grid Occupancy grid 
/// \param sensor_pose Assumed to lie on the grid and point along the grid
/// \param scanner_info Only the angle_{min,max,increment} and range_max
/// fields of this are used.
/// \param unknown_cells_are_obstacles If true, rays that hit unknown space
/// are considered to have hit an obstacle rather than being max-range
/// \retval Simulated laser range scan from the given pose on the grid
sensor_msgs::LaserScan::Ptr
simulateRangeScan (const nav_msgs::OccupancyGrid& grid, const geometry_msgs::Pose& sensor_pose,
                   const sensor_msgs::LaserScan& scanner_info,
                   bool unknown_cells_are_obstacles=false);

/************************************************************
 * Overlaying point clouds
 ************************************************************/


/// Default occupancy threshold for OverlayClouds
const double DEFAULT_OCCUPANCY_THRESHOLD=0.1;

/// Default max_distance for OverlayClouds
const double DEFAULT_MAX_DISTANCE=10.0;

/// Default min_pass_through for OverlayClouds 
const double DEFAULT_MIN_PASS_THROUGH=2;

/// \brief Create a cloud overlay object to which clouds can then be added
/// The returned (ros message) object should only be accessed using the api below
///
/// \param grid Contains the geometry of the underlying grid. Additionally, iff the data field of this object
/// is nonempty, rays will not be allowed to pass through obstacles in the grid.
/// \param frame_id The frame name for the overlaid grid.  Added clouds must have this
/// frame_id
/// \param occupancy_threshold what fraction of rays through the cell must end there
/// for it to be considered an obstacle
/// \param max_distance hits beyond this distance from the source are ignored
/// \param min_pass_through cells with fewer than this many rays through them
/// are marked UNKNOWN
OverlayClouds createCloudOverlay (const nav_msgs::OccupancyGrid& grid, const std::string& frame_id,
                                  double occupancy_threshold=DEFAULT_OCCUPANCY_THRESHOLD,
                                  double max_distance=DEFAULT_MAX_DISTANCE,
                                  double min_pass_through=DEFAULT_MIN_PASS_THROUGH);


/// \brief Raytrace a cloud onto grid in \a overlay
void addCloud (OverlayClouds* overlay, LocalizedCloud::ConstPtr cloud);

/// \brief Effectively subtract a cloud (which was presumably previously added), by
/// subtracting rather than adding counts, in \a overlay
void removeCloud (OverlayClouds* overlay, LocalizedCloud::ConstPtr cloud);

/// \brief Assert that a square centered at this point with side 2*r contains no obstacles
void addKnownFreePoint (OverlayClouds* overlay, const geometry_msgs::Point& p,
                        double r);


/// \brief Get the current grid.  It's fine to modify the returned object.
nav_msgs::OccupancyGrid::Ptr getGrid (const OverlayClouds& overlay);

// \brief Reset all counts to initial state in \a overlay
void resetCounts (OverlayClouds* overlay);

// \brief Saturate hit counts to the closest value to the threshold
void saturateCounts (OverlayClouds& overlay);

// \brief Saturate hit counts to the closest (by a given number of steps/observation) value to the threshold
void saturateCounts (OverlayClouds& overlay, const size_t steps);

// \brief Get the occupancy grid dimensions.
nav_msgs::MapMetaData gridInfo (const OverlayClouds& overlay);
  

} // namespace occupancy_grid_utils

#endif // include guard
