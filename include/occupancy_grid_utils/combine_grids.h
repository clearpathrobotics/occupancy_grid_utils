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
 * Combining overlapping grids
 *
 * \author Bhaskara Marthi
 */

#ifndef OCCUPANCY_GRID_UTILS_COMBINE_GRIDS_H
#define OCCUPANCY_GRID_UTILS_COMBINE_GRIDS_H

#include <nav_msgs/OccupancyGrid.h>

namespace occupancy_grid_utils
{

/// \brief Combines a set of grids
///
/// The resulting grid's origin will be a translated version of grid 1's origin, with resolution \a resolution
///
/// The policy for combination is that for each cell, we look at each grid cell that 
/// intersects it and consider their values.  Map these to integers, where everything above
/// OCCUPIED (100) goes to -1.  Then take the max.  If there are no intersecting cells, value is -1. 
///
/// Assumes all grids lie on the xy plane, and will fail in weird ways if that's not true
nav_msgs::OccupancyGrid::Ptr combineGrids(const std::vector<nav_msgs::OccupancyGrid::ConstPtr>& grids, double resolution);

/// Version of combineGrids that uses the resolution of the first grid.
nav_msgs::OccupancyGrid::Ptr combineGrids(const std::vector<nav_msgs::OccupancyGrid::ConstPtr>& grids);



} // namespace occupancy_grid_utils

#endif // include guard
