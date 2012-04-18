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
#include <queue>
#include <set>

namespace occupancy_grid_utils
{

typedef std::set<Cell> Cells;

bool lineIntersectsCell (const nm::MapMetaData& info, const gm::Point32& p1,
                         const gm::Point32& p2, const Cell& c)
{
  const float& x1 = p1.x;
  const float& x2 = p1.y;
  const float& y1 = p1.y;
  const float& y2 = p2.y;
  const float dx = x2-x1;
  const float dy = y2-y1;
  
  const float a = dx/
}

bool boundaryIntersectsCell (const nm::MapMetaData& info,
                             const gm::Polygon& poly,
                             const Cell& c)
{
  for (size_t i=0; i<poly.points.size(); i++)
  {
    const size_t j = i>0 ? i-1 : poly.points.size()-1;
    if (lineIntersectsCell(info, poly.points[i],
                                  poly.points[j], c))
      return true;
  }
  return false;
}

// Visitor for the flood fill
struct CellsInPolygon
{
  CellsInPolygon (const nm::MapMetaData& info, const gm::Polygon& poly) :
    info(info), poly(poly)
  {}

  bool operator() (const Cell& c)
  {
    cells.insert(c);
    return !boundaryIntersectsCell(info, poly, c);
  }

  Cells cells;
  const nm::MapMetaData& info;
  const gm::Polygon& poly;
};

// Generic flood fill
template <class Visitor>
void flood_fill (const nm::MapMetaData& info, const Cell& start,
                 Visitor& vis)
{
  Cells seen;
  std::queue<Cell> q;
  q.insert(start);
  while (!q.empty())
  {
    const Cell c = q.front();
    q.pop();
    seen.insert(c);
    if (vis(c))
    {
      for (int vertical=0; vertical<2; vertical++)
      {
        for (int d=-1; d<=1; d+=2)
        {
          const int dx = vertical ? 0 : d;
          const int dy = vertical ? d : 0;
          const Cell c2(c.x+dx, c.y+dy);
          if (withinBounds(info, c2))
            q.push_back(c2);
        }
      }
    }
  }
}



// We do this by starting at the center and flood-filling outwards till we
// reach the boundary
Cells cellsInConvexPolygon (const nm::MapMetaData& info,
                            const gm::Polygon& poly)
{
  CellsInPolygon visitor(info, poly);
  flood_fill(info, center(info, poly), visitor);
  return visitor.cells;
}


} // namespace
