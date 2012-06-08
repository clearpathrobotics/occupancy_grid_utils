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
 * Implementation of geometry.h
 *
 * \author Bhaskara Marthi
 */

#include <occupancy_grid_utils/geometry.h>
#include <queue>
#include <set>
#include <boost/foreach.hpp>

namespace occupancy_grid_utils
{

namespace gm=geometry_msgs;
namespace nm=nav_msgs;
using std::vector;
typedef std::set<Cell> Cells;

struct Line
{
  Line (const gm::Point32& p1, const gm::Point32& p2)
  {
    const float dx = p2.x-p1.x;
    const float dy = p2.y-p1.y;
    if (fabs(dx) < 1e-3 && fabs(dy) < 1e-3)
    {
      boost::format e("Points (%.2f, %.2f) and (%.2f, %.2f) are too close");
      throw GridUtilsException(e % p1.x % p1.y % p2.x % p2.y);
    }      
    a = dy;
    b = -dx;
    c = p1.y*dx - p1.x*dy;
  }
  
  // Line intersects a convex polygon if two of the vertices have opposite signs
  bool intersects (const gm::Polygon& poly) const
  {
    bool seen_nonnegative = false;
    bool seen_nonpositive = false;
    BOOST_FOREACH (const gm::Point32& p, poly.points) 
    {
      const float d = a*p.x+b*p.y+c;
      if (d>=0)
        seen_nonnegative = true;
      if (d<=0)
        seen_nonpositive = true;
      if (seen_nonnegative && seen_nonpositive)
        return true;
    }
    return false;
  }

  // Coefficients of line equation ax+by+c=0
  float a;
  float b;
  float c;
};

// Visitor for the flood fill
struct CellsInPolygon
{
  CellsInPolygon (const nm::MapMetaData& info, const gm::Polygon& poly) :
    info(info)
  {
    const size_t n = poly.points.size();
    for (size_t i=0; i<n; i++)
    {
      const size_t j = i>0 ? i-1 : n-1;
      sides.push_back(Line(poly.points[i], poly.points[j]));
    }
  }

  // Add cell to the set of visited cells.  If cell doesn't intersect the
  // boundary of the polygon, return true (continue propagating), else false.
  bool operator() (const Cell& c)
  {
    cells.insert(c);
    const gm::Polygon cell_poly = cellPolygon(info, c);
    BOOST_FOREACH (const Line& s, sides)
    {
      if (s.intersects(cell_poly))
        return false;
    }
    return true;
  }

  Cells cells;
  const nm::MapMetaData& info;
  std::vector<Line> sides;
};



// Generic flood fill
template <class Visitor>
void flood_fill (const nm::MapMetaData& info, const std::set<Cell>& start,
                 Visitor& vis)
{
  Cells seen;
  std::queue<Cell> q;
  BOOST_FOREACH (const Cell& c, start) 
    q.push(c);
  while (!q.empty())
  {
    const Cell c = q.front();
    q.pop();
    ROS_DEBUG_STREAM_NAMED ("flood_fill", "Cell " << c);
    if (seen.find(c)==seen.end())
    {
      seen.insert(c);
      ROS_DEBUG_NAMED ("flood_fill", "  Visiting");
      if (vis(c))
      {
        ROS_DEBUG_NAMED ("flood_fill", "  Adding neighbors");
        for (int dy=-1; dy<=1; dy++)
        {
          for (int dx=-1; dx<=1; dx++)
          {
            if (dx!=0 || dy!=0)
            {
              const Cell c2(c.x+dx, c.y+dy);
              if (withinBounds(info, c2))
              {
                q.push(c2);
                ROS_DEBUG_STREAM_NAMED ("flood_fill", "    Added " << c2);
              }
            }
          }
        }
      }
    }
  }
}

// Generic flood fill 
template <class Visitor>
void flood_fill (const nm::MapMetaData& info, const Cell& start,
                 Visitor& vis)
{
  std::set<Cell> s;
  s.insert(start);
  flood_fill(info, s, vis);
}


Cell center(const nm::MapMetaData& info,
            const gm::Polygon& poly)
{
  float sx=0;
  float sy=0;
  BOOST_FOREACH (const gm::Point32& p, poly.points) 
  {
    sx += p.x;
    sy += p.y;
  }
  gm::Point p;
  p.x = sx/poly.points.size();
  p.y = sy/poly.points.size();
  return pointCell(info, p);
}

// We do this by starting at the center and flood-filling outwards till we
// reach the boundary
Cells cellsInConvexPolygon (const nm::MapMetaData& info,
                            const gm::Polygon& poly)
{
  CellsInPolygon visitor(info, poly);
  flood_fill(info, center(info, poly), visitor);
  return visitor.cells;
}

struct DistanceQueueItem
{
  DistanceQueueItem (const Cell& c, const float distance) :
    c(c), distance(distance)
  {}

  inline
  bool operator< (const DistanceQueueItem& other) const
  {
    // Higher priority means lower distance
    return (distance > other.distance);
  }
  
  Cell c;
  float distance;
};

DistanceField distanceField (const nav_msgs::OccupancyGrid& m,
                             const float max_dist)
{
  // Initialize
  vector<unsigned> dims;
  dims.push_back(m.info.width);
  dims.push_back(m.info.height);
  DistanceField::ArrayPtr distances(new DistanceField::Array(dims));
  for (unsigned x=0; x<m.info.width; x++)
    for (unsigned y=0; y<m.info.height; y++)
      (*distances)[x][y] = max_dist;
  std::set<Cell> seen; // Set of cells that have already been added to distance field
  std::priority_queue<DistanceQueueItem> q;

  // Sweep over the grid and add all obstacles to the priority queue
  for (index_t i=0; i<m.info.width*m.info.height; i++)
  {
    if (m.data[i]!=UNOCCUPIED && m.data[i]!=UNKNOWN)
    {
      DistanceQueueItem item(indexCell(m.info, i), 0);
      q.push(item);
    }
  }

  // Iteration guaranteed to be in nondecreasing order of distance
  while (!q.empty())
  {
    DistanceQueueItem item = q.top();
    ROS_DEBUG_THROTTLE_NAMED (1.0, "distance_field",
                              "Distance %.4f, %zu seen",
                              item.distance, seen.size());
    q.pop();

    // If not already seen, add this cell to distance field, and 
    // add its neighbors to the queue
    if (seen.find(item.c)==seen.end())
    {
      seen.insert(item.c);
      const Cell& c = item.c;
      (*distances)[c.x][c.y] = item.distance;
      if (max_dist>0 && item.distance>max_dist)
        continue;
      for (char vertical=0; vertical<2; vertical++)
      {
        for (char d=-1; d<=1; d+=2)
        {
          const char dx = vertical ? 0 : d;
          const char dy = vertical ? d : 0;
          const Cell neighbor(c.x+dx, c.y+dy);
          if (withinBounds(m.info, neighbor))
          {
            const float dist = item.distance+m.info.resolution;
            q.push(DistanceQueueItem(neighbor, dist));
          }
        }
      }
    }
  }
  
  return DistanceField(distances);
}


} // namespace
