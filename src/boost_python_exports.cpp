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
 * Boost python exports for occupancy_grid_utils library
 *
 * \author Bhaskara Marthi
 */

#include <occupancy_grid_utils/coordinate_conversions.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/file.h>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

namespace occupancy_grid_utils
{

namespace nm=nav_msgs;
namespace gm=geometry_msgs;
namespace sm=sensor_msgs;
using std::string;


void allocateGrid (nav_msgs::OccupancyGrid& grid)
{
  grid.data.resize(grid.info.height*grid.info.width);
}

void setCell (nav_msgs::OccupancyGrid& grid, const Cell& c, const int x)
{
  grid.data[cellIndex(grid.info, c)] = x;
}

int getCell (const nav_msgs::OccupancyGrid& grid, const Cell& c)
{
  return grid.data[cellIndex(grid.info, c)];
}

nm::OccupancyGrid::Ptr loadGrid1 (const string& fname)
{
  return loadGrid(fname, 1.0, identityPose());
}

nm::OccupancyGrid::Ptr loadGrid2 (const string& fname, const double resolution)
{
  return loadGrid(fname, resolution, identityPose());
}

nm::OccupancyGrid::Ptr loadGrid3 (const string& fname, const double res,
                                  const gm::Pose& p)
{
  return loadGrid(fname, res, p);
}

sm::LaserScan::Ptr
simulateRangeScan3 (const nm::OccupancyGrid& grid, const gm::Pose& sensor_pose,
                    const sm::LaserScan& scanner_info)
{
  return simulateRangeScan(grid, sensor_pose, scanner_info);
}

sm::LaserScan::Ptr
simulateRangeScan4 (const nm::OccupancyGrid& grid, const gm::Pose& sensor_pose,
                    const sm::LaserScan& scanner_info, const bool unknown_obstacles)
{
  return simulateRangeScan(grid, sensor_pose, scanner_info, unknown_obstacles);
}

bool
withinBoundsCell (const nm::MapMetaData& info, const Cell& c)
{
  return withinBounds(info, c);
}

bool
withinBoundsPoint (const nm::MapMetaData& info, const gm::Point& p)
{
  return withinBounds(info, p);
}



BOOST_PYTHON_MODULE(occupancy_grid_utils)
{
  using namespace boost::python;

  /****************************************
   * Constants
   ****************************************/
  
  scope().attr("OCCUPIED") = OCCUPIED;
  scope().attr("UNOCCUPIED") = UNOCCUPIED;

  /****************************************
   * Types
   ****************************************/

  class_<Cell>("Cell", init<>())
    .def(init<int, int>())
    .def_readwrite("x", &Cell::x)
    .def_readwrite("y", &Cell::y)
    ;


  /****************************************
   * Operations
   ****************************************/

  def("cell_index", cellIndex);
  def("index_cell", indexCell);
  def("point_cell", pointCell);
  def("cell_center", cellCenter);
  def("point_index", pointIndex);
  def("within_bounds", withinBoundsCell);
  def("within_bounds", withinBoundsPoint);
  def("get_cell", getCell);
  def("set_cell", &setCell);
  def("allocate_grid", &allocateGrid);
  def("simulate_range_scan", &simulateRangeScan3);
  def("simulate_range_scan", &simulateRangeScan4);
  def("load_grid", loadGrid3);
  def("load_grid", loadGrid1);
  def("load_grid", loadGrid2);
}

} // namespace
