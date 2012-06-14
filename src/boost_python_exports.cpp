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

#include "shortest_path_result.h"
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/file.h>
#include <occupancy_grid_utils/geometry.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

// Macro for boost python's convoluted syntax for exposing vector<T> to python
#define BOOST_PYTHON_VECTOR(t, name) boost::python::class_<std::vector<t> >(name) \
  .def(boost::python::vector_indexing_suite<std::vector<t> >())

// Macro due to boost python requiring operator== for any T for which you want vector<T>
#define DEFINE_DUMMY_EQUALITY(ns, t) namespace ns {                     \
                                     bool operator== (const t& x1, const t& x2) { return false; } \
  } // namespace 

DEFINE_DUMMY_EQUALITY(geometry_msgs, Point32)

namespace occupancy_grid_utils
{

namespace nm=nav_msgs;
namespace gm=geometry_msgs;
namespace sm=sensor_msgs;
using std::string;
using std::vector;
using std::set;

// STL exports
void exportSTL ()
{
  using namespace boost::python;
  BOOST_PYTHON_VECTOR(int8_t, "Int8Vec");
}

// Ros message exports
// Should eventually be done via standard genmsg mechanisms
void exportRosMessages()
{
  using namespace boost::python;
  class_<std_msgs::Header>("Header")
    .def_readwrite("stamp", &std_msgs::Header::stamp)
    .def_readwrite("frame_id", &std_msgs::Header::frame_id);
  
  class_<ros::Time>("Time")
    .def_readwrite("sec", &ros::Time::sec)
    .def_readwrite("nsec", &ros::Time::nsec);

  class_<gm::Point>("Point")
    .def_readwrite("x", &gm::Point::x)
    .def_readwrite("y", &gm::Point::y)
    .def_readwrite("z", &gm::Point::z);
  
  class_<gm::Point32>("Point32")
    .def_readwrite("x", &gm::Point32::x)
    .def_readwrite("y", &gm::Point32::y)
    .def_readwrite("z", &gm::Point32::z);
  
  class_<gm::Quaternion>("Quaternion")
    .def_readwrite("x", &gm::Quaternion::x)
    .def_readwrite("y", &gm::Quaternion::y)
    .def_readwrite("z", &gm::Quaternion::z)
    .def_readwrite("w", &gm::Quaternion::w);
    
  class_<gm::Pose>("Pose")
    .def_readwrite("position", &gm::Pose::position)
    .def_readwrite("orientation", &gm::Pose::orientation);

  class_<nm::MapMetaData>("MapMetaData")
    .def_readwrite("resolution", &nm::MapMetaData::resolution)
    .def_readwrite("width", &nm::MapMetaData::width)
    .def_readwrite("height", &nm::MapMetaData::height)
    .def_readwrite("origin", &nm::MapMetaData::origin);
  
  BOOST_PYTHON_VECTOR(gm::Point32, "Point32Vec");
  
  class_<gm::Polygon>("Polygon")
    .def_readwrite("points", &gm::Polygon::points);

  class_<nm::OccupancyGrid, nm::OccupancyGrid::Ptr>("OccupancyGrid")
    .def_readwrite("header", &nm::OccupancyGrid::header)
    .def_readwrite("info", &nm::OccupancyGrid::info)
    .def_readwrite("data", &nm::OccupancyGrid::data);
}

/************************************************************
 * Helpers
 ************************************************************/

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

ResultPtr sssp1 (const nav_msgs::OccupancyGrid& g, const Cell& start,
                 const double max_dist)
{
  TerminationCondition term(max_dist);
  return singleSourceShortestPaths(g, start, term, false);
}

double ssspDistance (ResultPtr res, const Cell& dest)
{
  boost::optional<double> dist = distanceTo(res, dest);
  if (dist)
    return *dist;
  else
    return -1;
}

// Wrap the library function with one that returns a vector, so we
// can export to python
vector<Cell> cellVectorInConvexPolygon (const nm::MapMetaData& info,
                                        const gm::Polygon& p)
{
  set<Cell> cells = cellsInConvexPolygon(info, p);
  vector<Cell> cell_vec(cells.begin(), cells.end());
  return cell_vec;
}

BOOST_PYTHON_MODULE(occupancy_grid_utils)
{
  using namespace boost::python;

  exportSTL();
  exportRosMessages();

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
  
  BOOST_PYTHON_VECTOR(Cell, "CellVec");
  
  class_<ShortestPathResult, ResultPtr >
    ("ShortestPathResult")
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
  def("inflate_obstacles", inflateObstacles);
  def("nav_fn", sssp1);
  def("sssp_distance_internal", ssspDistance);
  def("cells_in_convex_polygon", cellVectorInConvexPolygon);
}




} // namespace
