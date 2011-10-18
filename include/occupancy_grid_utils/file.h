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
 * Loading/saving occ grids
 *
 * \author Bhaskara Marthi
 */

#ifndef OCCUPANCY_GRID_UTILS_FILE_H
#define OCCUPANCY_GRID_UTILS_FILE_H

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>

namespace occupancy_grid_utils
{

inline
geometry_msgs::Pose identityPose ()
{
  geometry_msgs::Pose p;
  p.orientation.w = 1.0;
  return p;
}

// Taken from map_server
const double DEFAULT_OCC_THRESHOLD = 0.65;
const double DEFAULT_FREE_THRESHOLD = 0.196;

/// \brief Load an occ grid from an image file
/// \param fname Filename
/// \param resolution in m/cell
/// \param origin Lower-left corner of image (only 2d-components are used)
///
/// Pixel values are considered occupied if they're above DEFAULT_OCC_THRESHOLD,
/// free if below DEFAULT_FREE_THRESHOLD, unknown otherwise
nav_msgs::OccupancyGrid::Ptr loadGrid (const std::string& fname,
                                       double resolution=1.0,
                                       const geometry_msgs::Pose& origin =
                                       identityPose());
                                   

} // namespace

#endif // include guard
