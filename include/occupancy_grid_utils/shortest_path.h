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
 * Shortest paths in occupancy grids
 *
 * \author Bhaskara Marthi
 */

#ifndef OCCUPANCY_GRID_UTILS_SHORTEST_PATH_H
#define OCCUPANCY_GRID_UTILS_SHORTEST_PATH_H

#include <occupancy_grid_utils/coordinate_conversions.h>
#include <occupancy_grid_utils/NavigationFunction.h>
#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>
#include <set>

namespace occupancy_grid_utils
{

/************************************************************
 * Obstacle inflation
 ************************************************************/

/// \brief Inflate obstacles in a grid
///
/// Obstacles are cell values that are not 0, unless allow_unknown is true,
/// in which case -1 is not considered an obstacle.
/// If there exist obstacles within radius r meters of a cell, its value is
/// replaced by the max of their values (-1 is considered to be the same as 1
/// for this).
/// Rounds up when converting from meters to cells
nav_msgs::OccupancyGrid::Ptr inflateObstacles (const nav_msgs::OccupancyGrid& g,
                                               double r,
                                               bool allow_unknown=false);


/************************************************************
 * Shortest paths
 ************************************************************/

struct ShortestPathResult;
typedef boost::shared_ptr<ShortestPathResult> ResultPtr;
typedef std::set<Cell> Cells;
typedef std::vector<Cell> Path;
typedef std::pair<Path, double> AStarResult;


/// \brief Termination condition for shortest path search
/// \a max_dist is a distance such that as soon as we see a cell with greater than this distance, we stop.
/// Note that, confusingly, max_dist is in cells though shortest path returns meters by default.  Currently,
/// we warn if use_cells is set to true below.  In future (H-turtle), this api will actually change to
/// only allow meters.
/// \a goals is a set such that once we've expanded all the cells in this set, we stop
struct TerminationCondition
{
  TerminationCondition ();
  TerminationCondition (double max_distance, bool use_cells=true);
  TerminationCondition (const Cells& goals);
  TerminationCondition (const Cells& goals, const double max_distance,
                        bool use_cells = true);

  boost::optional<double> max_distance_;
  boost::optional<std::set<Cell> > goals_;
  bool use_cells_;
};

/// \brief Single source Dijkstra's algorithm
/// \retval Structure from which paths can be extracted or distances computed
/// \param manhattan If false, Euclidean
/// 
/// Each cell is connected to its horizontal and vertical neighbors, with cost
/// 1, and diagonal neighbors with cost sqrt(2).  Only cells with value UNOCCUPIED
/// are considered.
ResultPtr singleSourceShortestPaths (const nav_msgs::OccupancyGrid& g, const Cell& src,
                                     bool manhattan=false);


/// \brief Single source Dijkstra's algorithm
/// \retval Structure from which paths can be extracted or distances computed
/// \param term allows stopping early when a distance bound is reached or when a 
/// a set of goal cells have been found
/// \param manhattan If false, Euclidean
/// 
/// Each cell is connected to its horizontal and vertical neighbors, with cost
/// 1, and diagonal neighbors with cost sqrt(2).  Only cells with value UNOCCUPIED
/// are considered.
ResultPtr singleSourceShortestPaths (const nav_msgs::OccupancyGrid& g, const Cell& src,
                                     const TerminationCondition& term,
                                     bool manhattan=false);

/// \brief Extract a path from the result of single-source shortest paths
/// \retval path If path exists, vector of cells where the source is first and \a dest 
/// is last; if not, uninitialized
boost::optional<Path> extractPath (ResultPtr shortest_path_result, const Cell& dest);

/// \brief From result of single-source shortest paths, extract distance to some destination
/// \retval Distance if path exists, uninitialized otherwise
boost::optional<double> distance(ResultPtr shortest_path_result, const Cell& dest);

/// \brief From result of single-source shortest paths, extract distance to some destination.
/// \retval Distance if path exists, uninitialized otherwise.  In meters, not cells.
boost::optional<double> distanceTo(ResultPtr shortest_path_result, const Cell& dest);


/// \brief deprecated
boost::optional<double> distance(ResultPtr shortest_path_result, const Cell& dest);

/// \brief Convert a shortest path result from a ros message
ResultPtr shortestPathResultFromMessage (const NavigationFunction& msg);

/// \brief Convert a shortest path result to a ros message
NavigationFunction shortestPathResultToMessage (ResultPtr res);


/// \brief A* search that returns distance in cells.
/// Deprecated; use shortestPathAStar instead.using Manhattan distance cost and heuristic, with only horizontal and
/// vertical neighbors
/// \return If a path is not found, an uninitialized optional.  Else,
/// the path and the cost.  Currently, though, the path is not returned, just the cost.
boost::optional<AStarResult> shortestPath(const nav_msgs::OccupancyGrid& g,
                                          const Cell& src, const Cell& dest);

/// \brief A* search using Manhattan distance cost and heuristic, with
/// only horizontal and/// vertical neighbors
/// \return If a path is not found, an uninitialized optional.  Else, the path
/// and the cost (meters).  Currently, though, the path is not returned, just
/// the cost.
boost::optional<AStarResult> shortestPathAStar(const nav_msgs::OccupancyGrid& g,
                                               const Cell& src, const Cell& dest);


} // namespace

#endif // include guard



