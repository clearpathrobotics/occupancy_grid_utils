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
 * Geometric operations
 *
 * \author Bhaskara Marthi
 */

#ifndef OCCUPANCY_GRID_UTILS_GEOMETRY_H
#define OCCUPANCY_GRID_UTILS_GEOMETRY_H

#include "coordinate_conversions.h"
#include <set>
#include <boost/multi_array.hpp>

namespace occupancy_grid_utils
{


/// \retval Set of cells in the region bounded by convex polygon \a p
///
/// Cells that are partly in the polygon (because they intersect the
/// boundary) are included.  Cells that intersect the polygon only along
/// an edge or corner may or may not be included.
std::set<Cell> cellsInConvexPolygon (const nav_msgs::MapMetaData& info,
                                     const geometry_msgs::Polygon& p);


/// \retval Locally maximal set of cells which are at least \a d apart in
/// Euclidean distance, and all of which satisfy \a pred
/// \tparam pred Defines operator(), a boolean predicate on Cell objects
template <typename Pred>
std::set<Cell> tileCells (const nav_msgs::MapMetaData& info, float d,
                          const Pred& p);


/// Return value of distanceField function, that maps cells
/// to their Euclidean distance to nearest obstacle
class DistanceField
{
public:

  typedef boost::multi_array<float, 2> Array;
  typedef boost::shared_ptr<Array> ArrayPtr;

  DistanceField (ArrayPtr distances) : distances_(distances) {}
  
  inline
  float operator[] (const Cell& c) const
  {
    return (*distances_)[c.x][c.y];
  };
  
private:

  ArrayPtr distances_;
};

  

/// \retval Distance field d, such that d[c] is the Manhattan distance
/// (in meters) from cell c to an obstacle cell.
/// \param max_dist Distances will be thresholded at this value if
/// it's positive
DistanceField distanceField (const nav_msgs::OccupancyGrid& m,
                             float max_dist=-42);

} // namespace

#include "impl/geometry.hpp"

#endif // include guard
