# Script to set up test environment for the Python bindings

import roslib
roslib.load_manifest('occupancy_grid_utils')
import occupancy_grid_utils_python as gu
import math

map_info = gu.MapMetaData()
map_info.resolution = 0.2
map_info.origin.position.x = 1
map_info.origin.position.y = 1
map_info.origin.orientation.w = 1
map_info.height = 20
map_info.width = 20

grid = gu.OccupancyGrid()
grid.info = map_info
gu.allocate_grid(grid)

def occ(i, j):
    gu.set_cell(grid, gu.Cell(i, j), gu.OCCUPIED)

occ(1, 6)
occ(3, 4)
occ(3, 7)
occ(4, 4)
occ(4, 6)
occ(5, 4)
occ(7, 8)

scan_info = gu.LaserScan()
scan_info.angle_min = -math.pi/2
scan_info.angle_max = math.pi/2
scan_info.angle_increment = math.pi/4
scan_info.range_max = 1.0

sp = gu.make_pose(1.7, 1.9, math.pi/2)


