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
 * Implementation of shortest_path.h
 *
 * \author Bhaskara Marthi
 */

#include "shortest_path_result.h"
#include <occupancy_grid_utils/shortest_path.h>
#include <ros/assert.h>
#include <queue>

namespace occupancy_grid_utils
{

namespace nm=nav_msgs;
namespace gm=geometry_msgs;

using std::vector;
using boost::optional;
using std::max;

TerminationCondition::TerminationCondition () 
{}

TerminationCondition::TerminationCondition (const double max_distance,
                                            const bool use_cells) :
  max_distance_(max_distance), use_cells_(use_cells)
{
  if (use_cells_)
    ROS_WARN ("Deprecated usage of version of TerminationCondition that uses"
              " distance in cells rather than meters.  This API will change"
              " in a hard-to-debug way in future!");
}

TerminationCondition::TerminationCondition (const Cells& goals) :
  goals_(goals)
{
}

TerminationCondition::TerminationCondition (const Cells& goals, const double max_distance,
                                            const bool use_cells) :
  max_distance_(max_distance), goals_(goals), use_cells_(use_cells)
{
  if (use_cells_)
    ROS_WARN ("Deprecated usage of version of TerminationCondition that uses"
              " distance in cells rather than meters.  This API will change"
              " in a hard-to-debug way in future!");
}


typedef vector<Cell> Path;
typedef boost::shared_ptr<nm::OccupancyGrid> GridPtr;

boost::optional<double> distance(ResultPtr res, const Cell& dest)
{
  ROS_WARN ("Using deprecated function distance.  Use distanceTo instead.");
  return res->potential[cellIndex(res->info, dest)];
}

boost::optional<double> distanceTo(ResultPtr res, const Cell& dest)
{
  boost::optional<double> d = res->potential[cellIndex(res->info, dest)];
  if (d)
    return *d * res->info.resolution;
  else
    return d;
}


boost::optional<Path> extractPath(ResultPtr res, const Cell& dest)
{
  boost::optional<Path> p;
  index_t current=cellIndex(res->info, dest);
  if (!res->back_pointers[current])
    return p; // No path exists

  index_t max=res->back_pointers.size();
  index_t n=0;
  p = Path();

  do {
    if (n++ > max) {
      ROS_FATAL("Cycle in extractPath");
      ROS_BREAK();
    }
    p->push_back(indexCell(res->info, current));
    ROS_ASSERT_MSG(res->back_pointers[current], "Back pointer not found for %u", current);
    current=*(res->back_pointers[current]);
  } while (current!=res->src_ind);
  p->push_back(indexCell(res->info, current));
  reverse(p->begin(), p->end());
  return p;
}

    

struct QueueItem
{
  index_t ind;
  double potential;
  QueueItem (const index_t ind, const double potential) :
    ind(ind), potential(potential) {}
};

bool operator< (const QueueItem& i1, const QueueItem& i2)
{
  return i1.potential > i2.potential;
}


ResultPtr singleSourceShortestPaths (const nm::OccupancyGrid& g, const Cell& src,
                                     const bool manhattan)
{
  return singleSourceShortestPaths(g, src, TerminationCondition(), manhattan);
}

ResultPtr singleSourceShortestPaths (const nm::OccupancyGrid& g, const Cell& src,
                                     const TerminationCondition& t,
                                     const bool manhattan)
{
  verifyDataSize(g);
  ShortestPathResult* res = new ShortestPathResult();
  ResultPtr result(res);
  res->info = g.info;
  res->src_ind = cellIndex(g.info, src);
  res->back_pointers.resize(g.data.size());
  res->potential.resize(g.data.size());

  std::priority_queue<QueueItem> q;
  q.push(QueueItem(res->src_ind, 0.0));
  res->potential[res->src_ind] = 0.0;
  ROS_DEBUG_NAMED ("shortest_path", "Computing single source shortest paths from %d, %d", src.x, src.y);

  Cells remaining_goals;
  if (t.goals_)
    remaining_goals = *t.goals_;

  while (!q.empty()) {
    const QueueItem i=q.top();
    q.pop();

    const Cell cell = indexCell(g.info, i.ind);
    const double dist = i.potential;
    const double dist_in_meters = dist*g.info.resolution;
    if (t.max_distance_ && 
        (*t.max_distance_ < (t.use_cells_ ? dist : dist_in_meters)))
        break;
    if (t.goals_) {
      remaining_goals.erase(cell);
      if (remaining_goals.empty())
        break;
    }
    
    for (int dx=-1; dx<=1; dx++) {
      for (int dy=-1; dy<=1; dy++) {
        if ((dx==0) && (dy==0))
          continue;
        if ((dx!=0) && (dy!=0) && manhattan)
          continue;
        const double d = ((dx==0) || (dy==0)) ? 1 : sqrt(2);
        const Cell neighbor(cell.x+dx, cell.y+dy);
        if (withinBounds(g.info, neighbor)) {
          const index_t ind=cellIndex(g.info, neighbor);
          const double new_dist = dist+d;
          if (g.data[ind]==UNOCCUPIED &&
              (!res->potential[ind] || *(res->potential[ind]) > new_dist) &&
              (!t.max_distance_ ||
               (t.use_cells_ && *t.max_distance_ >= new_dist) ||
               (!t.use_cells_ && *t.max_distance_ >= new_dist*g.info.resolution)))
          {
            ROS_DEBUG_NAMED ("shortest_path_propagation", "Adding cell %d, %d, at distance %.2f",
                             neighbor.x, neighbor.y, new_dist);
            res->potential[ind] = new_dist;
            res->back_pointers[ind] = i.ind;
            q.push(QueueItem(ind, new_dist));
          }
        }
      }
    }
  }
  ROS_DEBUG_NAMED ("shortest_path", "Done computing single source shortest paths from %d, %d", src.x, src.y);
  return result;
}


inline
bool myGt (const signed char x, const signed char y)
{
  return (x==-1 && y==0) || x>y;
}

struct InflationQueueItem
{
  InflationQueueItem (const Cell& cell, const signed char cost) :
    cell(cell), cost(cost)
  {}
  Cell cell;
  signed char cost;
};

GridPtr inflateObstacles (const nm::OccupancyGrid& g, const double r,
                          const bool allow_unknown)
{
  // Set up optimized 'priority queue'
  ROS_ASSERT (r>0);
  const int radius = 1+ceil(r/g.info.resolution);
  typedef vector<InflationQueueItem> QueueVec;
  vector<QueueVec> queues(radius);
  GridPtr g2(new nm::OccupancyGrid(g));
  const nm::MapMetaData& geom=g.info;

  // Add initial obstacles
  for (coord_t x=0; x<(int)geom.width; x++) {
    for (coord_t y=0; y<(int)geom.height; y++) {
      const Cell cell(x, y);
      const signed char val=g.data[cellIndex(geom, cell)];
      if ((allow_unknown && val>=1) || (!allow_unknown && val!=0))
        queues[0].push_back(InflationQueueItem(cell, val));
    }
  }

  while (true)
  {
    int ind=-1;
    for (int i=0; i<radius; i++)
    {
      if (queues[i].size()>0)
      {
        ind=i;
        break;
      }
    }
    if (ind<0)
      break;

    const InflationQueueItem& q=queues[ind].back();
    const index_t index = cellIndex(geom, q.cell);
    if (myGt(q.cost, g2->data[index]) || ind==0)
    {
      g2->data[index] = q.cost;
      if (ind<radius-1)
      {
        for (int vert=0; vert<2; vert ++)
        {
          for (int d=-1; d<=1; d += 2)
          {
            const int dx = vert ? 0 : d;
            const int dy = vert ? d : 0;
            const Cell c2(q.cell.x+dx, q.cell.y+dy);
            if (withinBounds(geom, c2))
            {
              queues[ind+1].push_back(InflationQueueItem(c2, q.cost));
            }
          }
        }
      }
    }

    queues[ind].pop_back();
  }
  return g2;
}

/************************************************************
 * A*
 ***********************************************************/

typedef std::pair<Path, double> AStarResult;
struct PQItem
{
  index_t ind;
  double g_cost;
  double h_cost;
  index_t parent_ind;
  PQItem (index_t ind, double g_cost, double h_cost, index_t parent_ind) :
    ind(ind), g_cost(g_cost), h_cost(h_cost), parent_ind(parent_ind) {}

  bool operator< (const PQItem& i2) const
  {
    return ((g_cost + h_cost) > (i2.g_cost + i2.h_cost));
  }
};


inline double manhattanHeuristic (const Cell& c1, const Cell& c2)
{
  return fabs(c1.x-c2.x) + fabs(c1.y-c2.y);
}


optional<AStarResult> shortestPathAStar(const nm::OccupancyGrid& g, const Cell& src, const Cell& dest)
{
  typedef std::map<index_t, index_t> ParentMap;

  const double resolution = g.info.resolution;
  std::priority_queue<PQItem> open_list;
  const unsigned num_cells = g.info.height*g.info.width;
  std::vector<bool> seen(num_cells); // Default initialized to all false
  const index_t dest_ind = cellIndex(g.info, dest);
  const index_t src_ind = cellIndex(g.info, src);
  open_list.push(PQItem(src_ind, 0, resolution*manhattanHeuristic(src, dest), src_ind));
  ParentMap parent;

  optional<AStarResult> res;
  ROS_DEBUG_STREAM_NAMED ("shortest_path", "Computing shortest path from " << src << " to " << dest);
  
  while (!open_list.empty()) {
    const PQItem current = open_list.top();
    open_list.pop();
    const Cell c = indexCell(g.info, current.ind);
    if (seen[current.ind])
      continue;
    parent[current.ind] = current.parent_ind;
    seen[current.ind] = true;
    ROS_DEBUG_STREAM_NAMED ("shortest_path_internal", "  Considering " << c << " with cost " <<
                            current.g_cost << " + " << current.h_cost);
    if (current.ind == dest_ind) {
      res = AStarResult();
      res->second = current.g_cost;
      break;
    }
      
    for (int d=-1; d<=1; d+=2) {
      for (int vertical=0; vertical<2; vertical++) {
        const int cx = c.x + d*(1-vertical);
        const int cy = c.y + d*vertical;
        if (cx>=0 && cy>=0) {
          const Cell c2((coord_t) cx, (coord_t) cy);
          if (withinBounds(g.info, c2)) {
            const index_t ind = cellIndex(g.info, c2);
            if (g.data[ind]==UNOCCUPIED && !seen[ind]) {
              open_list.push(PQItem(ind, current.g_cost + resolution,
                                    resolution*manhattanHeuristic(c2, dest),
                                    current.ind));
            }
            ROS_DEBUG_STREAM_COND_NAMED (g.data[ind]!=UNOCCUPIED, "shortest_path_internal",
                                         "  Skipping cell " << indexCell(g.info, ind) <<
                                         " with cost " << (unsigned) g.data[ind]);
          }
        }
      }
    }
  }

  // Extract path if found
  if (res)
  {
    vector<index_t> path;
    path.push_back(dest_ind);
    const index_t src_ind = cellIndex(g.info, src);
    while (true)
    {
      index_t last = *(--path.end());
      if (last == src_ind)
        break;
      ParentMap::const_iterator it = parent.find(last);
      ROS_ASSERT (it!=parent.end());
      index_t parent = it->second;
      ROS_ASSERT (parent!=last);
      path.push_back(parent);
    }
    for (int i=path.size()-1; i>=0; i--)
      res->first.push_back(indexCell(g.info, path[i]));
  }

  ROS_DEBUG_STREAM_NAMED ("shortest_path", "Computed shortest path.  Found = " << (bool) res.is_initialized());
  return res;
}


optional<AStarResult> shortestPath(const nm::OccupancyGrid& g, const Cell& src, const Cell& dest)
{
  ROS_WARN ("Using deprecated function shortestPath (use shortestPathAStar)");
  std::priority_queue<PQItem> open_list;
  const unsigned num_cells = g.info.height*g.info.width;
  std::vector<bool> seen(num_cells); // Default initialized to all false
  index_t src_ind = cellIndex(g.info, src);
  open_list.push(PQItem(src_ind, 0, manhattanHeuristic(src, dest), src_ind));
  std::vector<index_t> parent(num_cells);
  const index_t dest_ind = cellIndex(g.info, dest);

  optional<AStarResult> res;
  ROS_DEBUG_STREAM_NAMED ("shortest_path", "Computing shortest path from " << src << " to " << dest);
  
  while (!open_list.empty()) {
    const PQItem current = open_list.top();
    open_list.pop();
    const Cell c = indexCell(g.info, current.ind);
    if (seen[current.ind])
      continue;
    seen[current.ind] = true;
    ROS_DEBUG_STREAM_NAMED ("shortest_path_internal", "  Considering " << c << " with cost " <<
                            current.g_cost << " + " << current.h_cost);
    if (current.ind == dest_ind) {
      res = AStarResult();
      res->second = current.g_cost;
      // Todo fill in path
      break;
    }
      
    for (int d=-1; d<=1; d+=2) {
      for (int vertical=0; vertical<2; vertical++) {
        const int cx = c.x + d*(1-vertical);
        const int cy = c.y + d*vertical;
        if (cx>=0 && cy>=0) {
          const Cell c2((coord_t) cx, (coord_t) cy);
          if (withinBounds(g.info, c2)) {
            const index_t ind = cellIndex(g.info, c2);
            if (g.data[ind]==UNOCCUPIED && !seen[ind]) {
              open_list.push(PQItem(ind, current.g_cost + 1, manhattanHeuristic(c2, dest),
                                    current.ind));
              parent[ind] = current.ind;
            }
            ROS_DEBUG_STREAM_COND_NAMED (g.data[ind]!=UNOCCUPIED, "shortest_path_internal",
                                         "  Skipping cell " << indexCell(g.info, ind) <<
                                         " with cost " << (unsigned) g.data[ind]);
          }
        }
      }
    }
  }

  ROS_DEBUG_STREAM_NAMED ("shortest_path", "Computed shortest path.  Found = " << (bool) res.is_initialized());
  return res;
}





/************************************************************
 * Ros message conversion
 ************************************************************/

/// \brief Convert a shortest path result from a ros message
ResultPtr shortestPathResultFromMessage (const NavigationFunction& msg)
{
  ShortestPathResult* res = new ShortestPathResult();
  ResultPtr retval(res); // retval is a pointer to const, so we modify via res
  res->info = msg.info;
  res->src_ind = msg.source;

  const index_t num_cells = msg.valid.size();
  ROS_ASSERT (num_cells == msg.back_pointers.size());
  ROS_ASSERT (num_cells == msg.potential.size());
  res->back_pointers.resize(num_cells);
  res->potential.resize(num_cells);

  for (index_t i=0; i<num_cells; i++) {
    if (i==msg.source) {
      // Sanity check
      ROS_ASSERT (msg.back_pointers[i] == 123456);
      ROS_ASSERT (msg.potential[i] == -1.0);
      res->potential[i] = 0.0;
    }
    else if (msg.valid[i]) {
      res->back_pointers[i] = msg.back_pointers[i];
      res->potential[i] = msg.potential[i];
    }
    else {
      // Sanity check
      ROS_ASSERT (msg.back_pointers[i] == 234567);
      ROS_ASSERT (msg.potential[i] == -2.0);
    }
  }

  return retval;
}

/// \brief Convert a shortest path result to a ros message
NavigationFunction shortestPathResultToMessage (ResultPtr res)
{
  NavigationFunction msg;
  msg.info = res->info;
  msg.source = res->src_ind;
  
  const index_t num_cells=res->back_pointers.size();
  ROS_ASSERT (num_cells == res->potential.size());
  msg.valid.resize(num_cells);
  msg.back_pointers.resize(num_cells);
  msg.potential.resize(num_cells);

  for (index_t i=0; i<num_cells; i++) {
    if (i==msg.source) {
      msg.valid[i] = false;
      msg.potential[i] = -1.0;
      msg.back_pointers[i] = 123456;
    }
    else if (res->potential[i]) {
      msg.valid[i] = true;
      msg.potential[i] = *res->potential[i];
      msg.back_pointers[i] = *res->back_pointers[i];
    }
    else {
      msg.valid[i] = false;
      msg.potential[i] = -2.0;
      msg.back_pointers[i] = 234567;
    }
  }
  
  return msg;
}


  
} // namespace occupancy_grid_utils
