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
 * Template functions from geometry.h
 *
 * \author Bhaskara Marthi
 */


namespace occupancy_grid_utils
{

namespace nm=nav_msgs;
typedef std::set<Cell> Cells;

template <typename Pred>
std::set<Cell> tileCells (const nm::MapMetaData& info, const float d,
                          const Pred& pred)
{
  ROS_DEBUG_NAMED ("tile", "Tiling %ux%u map", info.height, info.width);
  Cells cells;
  Cells forbidden;
  int rad = ceil(d/info.resolution);
  for (size_t x=0; x<info.width; x++)
  {
    for (size_t y=0; y<info.height; y++)
    {
      const Cell c(x, y);
      if (!pred(c))
        continue;
      ROS_DEBUG_STREAM_NAMED("tile", "Cell " << c << " satisfies condition");
      if (forbidden.find(c)!=forbidden.end())
        continue;
      ROS_DEBUG_STREAM_NAMED ("tile", "  Sufficiently far");
      cells.insert(c);
      ROS_DEBUG_STREAM_NAMED ("tile", "  Inserted");
      for (int dx=0; dx<=rad; dx++)
      {
        for (int dy=-rad; dy<=rad; dy++)
        {
          const Cell c2(int(x)+dx, int(y)+dy);
          if (dx*dx+dy*dy <= rad*rad && withinBounds(info, c2))
          {
            ROS_DEBUG_STREAM_NAMED ("tile", "  Blocking " << c2);
            forbidden.insert(c2);
          }
        }
      }
    }
  }
  ROS_DEBUG_NAMED("tile", "Done tiling");
  return cells;
}


} // namespace
