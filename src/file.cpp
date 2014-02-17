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
 * Implementation of file ops
 *
 * \author Bhaskara Marthi
 */

#include <occupancy_grid_utils/file.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/GetMap.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace occupancy_grid_utils
{

namespace nm=nav_msgs;
namespace gm=geometry_msgs;

/************************************************************
 * Implementation
 * This is lifted from map_server, which unfortunately
 * does not export the library in its manifest
 ***********************************************************/


#include <cstring>
#include <stdexcept>

#include <stdlib.h>
#include <stdio.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "LinearMath/btMatrix3x3.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

void
loadMapFromFile(nav_msgs::GetMap::Response* resp,
                const char* fname, double res, bool negate,
                double occ_th, double free_th, double* origin)
{
  cv::Mat img;

  unsigned int i,j;
  double occ;
  double color_avg;

  // Load the image using OpenCV.  If we get NULL data back, the image load failed.
  cv::Mat imgColor = cv::imread( fname );
  if(!imgColor.data)
  {
      std::string errmsg = std::string("failed to open image file \"") +
              fname + std::string("\"");
      throw std::runtime_error(errmsg);
  }
  cvtColor(imgColor, img, CV_BGR2GRAY);

  // Copy the image data into the map structure
  resp->map.info.width = img.rows;
  resp->map.info.height = img.cols;
  resp->map.info.resolution = res;
  resp->map.info.origin.position.x = *(origin);
  resp->map.info.origin.position.y = *(origin+1);
  resp->map.info.origin.position.z = 0.0;
  btQuaternion q;
  q.setEuler(*(origin + 2), 0, 0);
  resp->map.info.origin.orientation.x = q.x();
  resp->map.info.origin.orientation.y = q.y();
  resp->map.info.origin.orientation.z = q.z();
  resp->map.info.origin.orientation.w = q.w();

  // Allocate space to hold the data
  resp->map.data.resize(resp->map.info.width * resp->map.info.height);

  // Copy pixel data into the map structure
  for(j = 0; j < resp->map.info.height; j++)
  {
    for (i = 0; i < resp->map.info.width; i++)
    {
      // Compute mean of RGB for this pixel
      color_avg = (double)(img.at<uint8_t>(i, j));

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels free.  Otherwise, it's vice versa.
      if(negate)
        occ = color_avg / 255.0;
      else
        occ = (255 - color_avg) / 255.0;
      
      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.
      if(occ > occ_th)
        resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = +100;
      else if(occ < free_th)
        resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = 0;
      else
        resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = -1;
    }
  }
}


/************************************************************
 * Interface
 ***********************************************************/

nm::OccupancyGrid::Ptr loadGrid (const std::string& fname,
                                 const double resolution,
                                 const gm::Pose& origin_pose)
{
  double origin[3];
  origin[0] = origin_pose.position.x;
  origin[1] = origin_pose.position.y;
  origin[2] = tf::getYaw(origin_pose.orientation);

  nm::GetMap::Response resp;
  loadMapFromFile(&resp, fname.c_str(), resolution, false,
                  DEFAULT_OCC_THRESHOLD, DEFAULT_FREE_THRESHOLD,
                  origin);
  nm::OccupancyGrid::Ptr grid(new nm::OccupancyGrid(resp.map));
  return grid;
}



} // namespace
