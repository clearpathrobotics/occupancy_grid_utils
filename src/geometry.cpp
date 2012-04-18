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

namespace occupancy_grid_utils
{

typedef std::set<Cell> Cells;

bool lineSegmentIntersectsCell (const nm::MapMetaData& info,
                                const gm::Point32& p1,
                                const gm::Point32& p2,
                                const Cell& c)
{


}

bool boundaryIntersectsCell (const nm::MapMetaData& info,
                             const gm::Polygon& poly,
                             const Cell& c)
{
  for (size_t i=0; i<poly.points.size(); i++)
  {
    const size_t j = i>0 ? i-1 : poly.points.size()-1;
    if (lineSegmentIntersectsCell(info, poly.points[i],
                                  poly.points[j], c))
      return true;
  }
  return false;
}

Cells neighbors (const nm::MapMetaData& info,
                 const Cell& c)
{
  Cells neighbors;
  
}

Cells cellsInConvexPolygon (const nm::MapMetaData& info,
                            const gm::Polygon& poly)
{
  Cells cells;
  Queue q;
  q.push_back(center(poly));
  while (!q.empty())
  {
    const Cell c = q.front();
    q.pop();
    cells.insert(c);
    if (!boundaryIntersectsCell(info, poly, c))
    {
      BOOST_FOREACH (const Cell& n, neighbors(c))
      {
        if (visited.find(n)==visited.end())
        {
          q.push_back(n);
        }
      }
    }
  
  }
  return cells;
}

} // namespace
