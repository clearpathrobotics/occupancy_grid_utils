from occupancy_grid_utils import *
import math

def cell_str (c):
    return str((c.x, c.y))

def header_str (h):
    return "[Frame: {0}, Stamp: {1}]".format(h.frame_id, h.stamp)

def time_str (t):
    return str(t.sec + t.nsec*1e-9)

def info_str (i):
    return "[{0}x{1} map at {2} m/cell]".format(i.width, i.height, i.resolution)

def grid_str (g):
    if len(g.data)==g.info.height*g.info.width:
        return "[Grid with dims: {0}]".format(g.info)
    elif len(g.data)==0:
        return "[Info: {0}, data uninitialized]".format(s)
    else:
        return "[Info: {0}, data size {1} instead of {2}]".format(g.info, len(g.data), g.info.height*g.info.width)

def scan_str (s):
    lr = len(s.ranges)
    nz = list(s.ranges).count(0)
    if nz<lr:
        return "[Scan with header {3}, bounds [{0}, {1}] and {2} readings]".\
               format(s.angle_min, s.angle_max, len(s.ranges), s.header)
    else:
        return "[Scan with header {2}, bounds [{0}, {1}].  Uninitialized.]".\
               format(s.angle_min, s.angle_max, s.header)

assume_2d = True

def pt_str(p):
    if assume_2d:
        return str((p.x, p.y))
    else:
        return str((p.x, p.y, p.z))

def quaternion_str(q):
    if assume_2d:
        return str(2*math.acos(q.w))
    else:
        return str((q.x, q.y, q.z, q.w))

def pose_str(p):
    return "({0}, {1})".format(p.position, p.orientation)

def polygon_str(p):
    s = ', '.join(pt_str(p) for p in p.points)
    return "[Polygon with points {0}]".format(s)


Cell.__str__ = cell_str
Header.__str__ = header_str
Time.__str__ = time_str
MapMetaData.__str__ = info_str
OccupancyGrid.__str__ = grid_str
# LaserScan.__str__ = scan_str
Pose.__str__ = pose_str
Point32.__str__ = pt_str
Point.__str__ = pt_str
Polygon.__str__ = polygon_str


use_repr = True
# These are technically incorrect, but make life easier in a shell

if use_repr:
    Cell.__repr__ = cell_str
    Header.__repr__ = header_str
    Time.__repr__ = time_str
    MapMetaData.__repr__ = info_str
    OccupancyGrid.__repr__ = grid_str
    # LaserScan.__repr__ = scan_str
    Pose.__repr__ = pose_str
    Point.__repr__ = pt_str
    Point32.__repr__ = pt_str
    Polygon.__repr__ = polygon_str
    Quaternion.__repr__ = quaternion_str

