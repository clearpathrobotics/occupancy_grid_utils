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
 *
 */


/**
 * \file
 *
 * Implementation for combine_grids.h
 *
 * \author Bhaskara Marthi
 */

#include <occupancy_grid_utils/combine_grids.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <ros/assert.h>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <set>
#include "gcc_version.h"


namespace occupancy_grid_utils
{

namespace nm=nav_msgs;
namespace gm=geometry_msgs;

using boost::bind;
using boost::ref;
using std::vector;
using std::multiset;
using std::set;
using std::min;
using std::max;

typedef boost::shared_ptr<nm::OccupancyGrid> GridPtr;
typedef boost::shared_ptr<nm::OccupancyGrid const> GridConstPtr;

inline Cell point32Cell (const nm::MapMetaData& info, const gm::Point32& p)
{
  gm::Point pt;
  pt.x = p.x;
  pt.y = p.y;
  pt.z = p.z;
  return pointCell(info, pt);
}

// Does this cell contain a vertex of this polygon?
inline bool containsVertex (const nm::MapMetaData& info, const Cell& c, const gm::Polygon& poly)
{
  BOOST_FOREACH (const gm::Point32& p, poly.points) {
    if (point32Cell(info, p)==c)
      return true;
  }
  return false;    
}


// Do the two cells (on different grids) intersect?
inline bool cellsIntersect (const nm::MapMetaData& info1, const Cell& c1, const nm::MapMetaData& info2, const Cell& c2)
{
  const gm::Polygon p1=cellPolygon(info1, c1);
  const gm::Polygon p2=cellPolygon(info2, c2);
  return containsVertex(info1, c1, p2) || containsVertex(info2, c2, p1);
}

inline gm::Polygon expandPolygon(const gm::Polygon& p, const double r)
{
  double sx=0;
  double sy=0;
  double sz=0;
  const size_t n = p.points.size();
  for (unsigned i=0; i<n; i++) {
    sx += p.points[i].x;
    sy += p.points[i].y;
    sz += p.points[i].z;
  }
  sx /= n;
  sy /= n;
  sz /= n;
  gm::Polygon p2;
  p2.points.resize(n);
  for (unsigned i=0; i<n; i++) {
    p2.points[i].x = sx + r*(p.points[i].x-sx);
    p2.points[i].y = sy + r*(p.points[i].y-sy);
    p2.points[i].z = sz + r*(p.points[i].z-sz);
  }
  return p2;
}

// Return set of intersecting cells in grid info, of cell cell2 in info2
set<Cell> intersectingCells (const nm::MapMetaData& info, const nm::MapMetaData& info2, const Cell& cell2)
{
  // The expansion is to avoid weird effects due to rounding when intersecting parallel grids
  const gm::Polygon poly=expandPolygon(cellPolygon(info2, cell2), 1.0001);

  // Figure out the candidates
  vector<Cell> corners(4);
  transform(poly.points.begin(), poly.points.end(), corners.begin(), 
            bind(point32Cell, boost::ref(info), _1));
  const coord_t min_x=min(corners[0].x, min(corners[1].x, min(corners[2].x, corners[3].x)));
  const coord_t min_y=min(corners[0].y, min(corners[1].y, min(corners[2].y, corners[3].y)));
  const coord_t max_x=max(corners[0].x, max(corners[1].x, max(corners[2].x, corners[3].x)));
  const coord_t max_y=max(corners[0].y, max(corners[1].y, max(corners[2].y, corners[3].y)));
  
  set<Cell> cells;
  for (coord_t x=min_x; x<=max_x; x++) {
    for (coord_t y=min_y; y<=max_y; y++) {
      const Cell candidate(x, y);
      if (withinBounds(info, candidate) &&
          cellsIntersect(info, candidate, info2, cell2))
        cells.insert(candidate);
    }
  }

  return cells;
}

double minX (const nm::MapMetaData& info)
{
  const gm::Polygon p=gridPolygon(info);
  return min(p.points[0].x, min(p.points[1].x, min(p.points[2].x, p.points[3].x)));
}

double maxX (const nm::MapMetaData& info)
{
  const gm::Polygon p=gridPolygon(info);
  return max(p.points[0].x, max(p.points[1].x, max(p.points[2].x, p.points[3].x)));
}

double minY (const nm::MapMetaData& info)
{
  const gm::Polygon p=gridPolygon(info);
  return min(p.points[0].y, min(p.points[1].y, min(p.points[2].y, p.points[3].y)));
}

double maxY (const nm::MapMetaData& info)
{
  const gm::Polygon p=gridPolygon(info);
  return max(p.points[0].y, max(p.points[1].y, max(p.points[2].y, p.points[3].y)));
}

gm::Pose transformPose (const tf::Pose trans, const gm::Pose p)
{
  tf::Pose pose;
  tf::poseMsgToTF(p, pose);
  gm::Pose transformed;
  tf::poseTFToMsg(trans*pose, transformed);
  return transformed;
}

// Get the dimensions of a combined grid
nm::MapMetaData getCombinedGridInfo (const vector<GridConstPtr>& grids, const double resolution)
{
  ROS_ASSERT (!grids.empty());
  nm::MapMetaData info;
  info.resolution = resolution;
  tf::Pose trans;
  tf::poseMsgToTF(grids[0]->info.origin, trans);
  

#ifdef GRID_UTILS_GCC_46
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
#endif

  boost::optional<double> min_x, max_x, min_y, max_y;
  BOOST_FOREACH (const GridConstPtr& g, grids) {
    nm::MapMetaData grid_info = g->info;
    grid_info.origin = transformPose(trans.inverse(), g->info.origin);
    if (!(min_x && *min_x < minX(grid_info)))
      min_x = minX(grid_info);
    if (!(min_y && *min_y < minY(grid_info)))
      min_y = minY(grid_info);
    if (!(max_x && *max_x > maxX(grid_info)))
      max_x = maxX(grid_info);
    if (!(max_y && *max_y > maxY(grid_info)))
      max_y = maxY(grid_info);
  }
  ROS_ASSERT(min_x && max_x && min_y && max_y);

  const double dx = *max_x - *min_x;
  const double dy = *max_y - *min_y;
  ROS_ASSERT ((dx > 0) && (dy > 0));
  
#ifdef GRID_UTILS_GCC_46
#pragma GCC diagnostic pop
#endif

  gm::Pose pose_in_grid_frame;
  pose_in_grid_frame.position.x = *min_x;
  pose_in_grid_frame.position.y = *min_y;
  pose_in_grid_frame.orientation.w = 1.0;
  info.origin = transformPose(trans, pose_in_grid_frame);
  info.height = ceil(dy/info.resolution);
  info.width = ceil(dx/info.resolution);

  return info;
}


// Main function
GridPtr combineGrids (const vector<GridConstPtr>& grids, const double resolution)
{
  GridPtr combined_grid(new nm::OccupancyGrid());
  combined_grid->info = getCombinedGridInfo(grids, resolution);
  combined_grid->data.resize(combined_grid->info.width*combined_grid->info.height);
  fill(combined_grid->data.begin(), combined_grid->data.end(), -1);
  ROS_DEBUG_NAMED ("combine_grids", "Combining %zu grids", grids.size());

  BOOST_FOREACH (const GridConstPtr& grid, grids) {
    for (coord_t x=0; x<(int)grid->info.width; x++) {
      for (coord_t y=0; y<(int)grid->info.height; y++) {
        const Cell cell(x, y);
        const signed char value=grid->data[cellIndex(grid->info, cell)];

        // Only proceed if the value is not unknown 
        if ((value>=0) && (value<=100)) {
          BOOST_FOREACH (const Cell& intersecting_cell, 
                         intersectingCells(combined_grid->info, grid->info, cell)) {
            const index_t ind = cellIndex(combined_grid->info, intersecting_cell);
            combined_grid->data[ind] = max(combined_grid->data[ind], value);
          }
        }
      }
    }
  }

  ROS_DEBUG_NAMED ("combine_grids", "Done combining grids");
  return combined_grid;
}


GridPtr combineGrids (const vector<GridConstPtr>& grids)
{
  ROS_ASSERT (!grids.empty());
  return combineGrids(grids, grids[0]->info.resolution);
}


} // namespace occupancy_grid_utils




