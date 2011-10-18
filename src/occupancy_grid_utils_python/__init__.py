import roslib
roslib.load_manifest('occupancy_grid_utils')
import rospy
import nav_msgs.msg as nm
import geometry_msgs.msg as gm
import sensor_msgs.msg as sm

from occupancy_grid_utils import *
import math
from .str import *

def make_pose(x, y, th):
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.orientation.w = math.cos(th/2)
    p.orientation.z = math.sin(th/2)
    return p

############################################################
# Conversion to ROS message types so we can publish them
############################################################

def time_to_ros(t):
    return rospy.Time(t.sec, t.nsec)

Time.to_ros = time_to_ros

def header_to_ros(h):
    return rospy.Header(stamp=h.stamp.to_ros(), frame_id=h.frame_id)

Header.to_ros = header_to_ros

def point_to_ros(p):
    return gm.Point(x=p.x, y=p.y, z=p.z)

def point_from_ros(m):
    p = Point()
    p.x = m.x
    p.y = m.y
    p.z = m.z
    return p

Point.to_ros = point_to_ros

def quaternion_to_ros(q):
    return gm.Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)

def quaternion_from_ros(m):
    q = Quaternion()
    q.x = m.x
    q.y = m.y
    q.z = m.z
    q.w = m.w
    return q

Quaternion.to_ros = quaternion_to_ros

def pose_to_ros(p):
    return gm.Pose(position=p.position.to_ros(),
                   orientation=p.orientation.to_ros())

def pose_from_ros(m):
    p = Pose()
    p.position = point_from_ros(m.position)
    p.orientation = quaternion_from_ros(m.orientation)
    return p

Pose.to_ros = pose_to_ros

def metadata_to_ros(m):
    return nm.MapMetaData(resolution=m.resolution, width=m.width,
                          height=m.height, origin=m.origin.to_ros())

MapMetaData.to_ros = metadata_to_ros


def grid_to_ros(g):
    return nm.OccupancyGrid(data=g.data, header=g.header.to_ros(),
                            info=g.info.to_ros())

OccupancyGrid.to_ros = grid_to_ros

def scan_to_ros(s):
    return sm.LaserScan(header=s.header.to_ros(), angle_min=s.angle_min,
                        angle_max=s.angle_max,
                        angle_increment=s.angle_increment,
                        # time_increment=s.time_increment, scan_time=s.scan_time,
                        range_min=s.range_min, range_max=s.range_max,
                        ranges=s.ranges,
                        #intensities=s.intensities
                        )

LaserScan.to_ros = scan_to_ros

