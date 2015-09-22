/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * 
 * Implements overlayClouds
 *
 * \author Bhaskara Marthi
 */

#include <occupancy_grid_utils/ray_tracer.h>
#include <ros/assert.h>
#include <tf/transform_datatypes.h>
#include <boost/foreach.hpp>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/optional.hpp>
#include "gcc_version.h"

namespace occupancy_grid_utils
{

namespace gm=geometry_msgs;
namespace nm=nav_msgs;

using boost::bind;
using boost::ref;
using std::vector;

typedef boost::shared_ptr<nm::OccupancyGrid> GridPtr;
typedef boost::shared_ptr<nm::OccupancyGrid const> GridConstPtr;


inline gm::Point transformPt (const tf::Pose& trans, const gm::Point32& p)
{
  const tf::Vector3 pt(p.x, p.y, p.z);
  gm::Point transformed;
  tf::pointTFToMsg(trans*pt, transformed);
  return transformed;
}


// This is our policy for computing the occupancy of a cell based on hit and pass through counts
inline int8_t determineOccupancy (const unsigned hit_count, const unsigned pass_through_count, 
                                  const double occupancy_threshold, const double min_pass_through)
{
  int8_t ret;
  if (pass_through_count < min_pass_through)
    ret=UNKNOWN;
  else if (hit_count > pass_through_count*occupancy_threshold)
    ret=OCCUPIED;
  else 
    ret=UNOCCUPIED;
  ROS_DEBUG_NAMED ("overlay_get_grid", " Hit count is %u, pass through count is %u, occupancy is %d",
                   hit_count, pass_through_count, ret);
  return ret;
}

OverlayClouds createCloudOverlay (const nm::OccupancyGrid& grid, const std::string& frame_id,
                                  double occupancy_threshold,
                                  double max_distance, double min_pass_through)
{
  OverlayClouds overlay;
  overlay.grid = grid;
  overlay.frame_id = frame_id;
  overlay.occupancy_threshold = occupancy_threshold;
  overlay.max_distance = max_distance;
  overlay.min_pass_through = min_pass_through;
  overlay.hit_counts.resize(grid.info.height*grid.info.width);
  overlay.pass_through_counts.resize(grid.info.height*grid.info.width);
  ROS_ASSERT(min_pass_through > 0);
  return overlay;
}

void addKnownFreePoint (OverlayClouds* overlay, const gm::Point& p, const double r)
{
  const nm::MapMetaData& geom = overlay->grid.info;
  const int cell_radius = floor(r/geom.resolution);
  const Cell c = pointCell(geom, p);
  for (int x= c.x-cell_radius; x<=c.x+cell_radius; x++)
  {
    for (int y=c.y-cell_radius; y<=c.y+cell_radius; y++)
    {
      const Cell c2(x, y);
      if (withinBounds(geom, c2))
      {
        const index_t ind = cellIndex(geom, c2);
        overlay->hit_counts[ind] = 0;
        overlay->pass_through_counts[ind] = overlay->min_pass_through+1;
      }
    }
  }
}


void addCloud (OverlayClouds* overlay, LocalizedCloud::ConstPtr cloud, const int inc)
{
  
  ROS_ASSERT_MSG(overlay->frame_id==cloud->header.frame_id,
                 "Frame id %s of overlayed cloud didn't match existing one %s",
                 cloud->header.frame_id.c_str(), overlay->frame_id.c_str());
  const index_t num_cells = overlay->grid.info.width*overlay->grid.info.height;
  ROS_ASSERT(num_cells>0);

  const gm::Point& sensor_pos = cloud->sensor_pose.position;
  ROS_DEBUG_NAMED ("overlay", "Ray tracing from %.2f, %.2f", sensor_pos.x, sensor_pos.y);

  // Transform points to world frame
  tf::Pose sensor_to_world;
  tf::poseMsgToTF(cloud->sensor_pose, sensor_to_world);
  vector<gm::Point> transformed_points(cloud->cloud.points.size());
  transform(cloud->cloud.points.begin(), 
            cloud->cloud.points.end(), 
            transformed_points.begin(),
            bind(transformPt, ref(sensor_to_world), _1));

  // Iterate over points in this cloud
  BOOST_FOREACH (const gm::Point& p, transformed_points) {
  
    ROS_DEBUG_NAMED ("overlay_counts", " Ray tracing to point %.2f, %.2f", p.x, p.y);
    boost::optional<index_t> last_ind;

    const bool have_existing_grid = !overlay->grid.data.empty();
    
    // Inner loop: raytrace along the line and update counts
    // We allow both the sensor pose and the target to be off the grid
    BOOST_FOREACH (const Cell& c, rayTrace(overlay->grid.info, sensor_pos, p, true, true, overlay->max_distance)) {
      last_ind = cellIndex(overlay->grid.info, c);
      overlay->pass_through_counts[*last_ind] += inc;
      if (have_existing_grid && overlay->grid.data[*last_ind] == OCCUPIED)
          break;
      ROS_ASSERT(overlay->pass_through_counts[*last_ind]>=0);
      ROS_DEBUG_NAMED ("overlay_counts", "  Pass-through counts for %d, %d are now %u", c.x, c.y, 
                       overlay->pass_through_counts[*last_ind]);
    }

    if (last_ind) {
      // If the last cell equals the point (i.e., point is not off the grid), update hit counts
      const Cell last_cell = indexCell(overlay->grid.info, *last_ind);
      if (last_cell == pointCell(overlay->grid.info, p)) {
        
#ifdef GRID_UTILS_GCC_46
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
#endif
        
        overlay->hit_counts[*last_ind] += inc;
        
#ifdef GRID_UTILS_GCC_46
#pragma GCC diagnostic pop
#endif
        
        ROS_ASSERT(overlay->hit_counts[*last_ind]>=0);
        ROS_DEBUG_NAMED ("overlay_counts", "  Hit counts for %d, %d are now %u", last_cell.x, last_cell.y, 
                         overlay->hit_counts[*last_ind]);
      }
    }
  }
  ROS_DEBUG_NAMED ("overlay", "Done ray tracing from %.2f, %.2f", sensor_pos.x, sensor_pos.y);
}


void addCloud (OverlayClouds* overlay, LocalizedCloud::ConstPtr cloud)
{
  addCloud(overlay, cloud, 1);
}

void removeCloud (OverlayClouds* overlay, LocalizedCloud::ConstPtr cloud)
{
  addCloud(overlay, cloud, -1);
}



GridPtr getGrid (const OverlayClouds& overlay) 
{
  GridPtr grid(new nm::OccupancyGrid());
  grid->info = overlay.grid.info;
  grid->header.frame_id = overlay.frame_id;
  ROS_DEBUG_STREAM_NAMED ("overlay_get_grid", "Getting overlaid grid with geometry " << overlay.grid.info);
  grid->data.resize(overlay.grid.info.width*overlay.grid.info.height);
  transform(overlay.hit_counts.begin(), overlay.hit_counts.end(), overlay.pass_through_counts.begin(), 
            grid->data.begin(), bind(determineOccupancy, _1, _2, overlay.occupancy_threshold,
                                     overlay.min_pass_through));
  return grid;
}

void resetCounts (OverlayClouds* overlay)
{
  fill(overlay->hit_counts.begin(), overlay->hit_counts.end(), 0);
  fill(overlay->pass_through_counts.begin(), overlay->pass_through_counts.end(), 0);
}

nm::MapMetaData gridInfo (const OverlayClouds& overlay) 
{
  return overlay.grid.info;
}

} // namespace occupancy_grid_utils
