# Represents a point cloud (in a sensor frame) together with the pose of the sensor in
# reference frame header.frame_id
# The header of the cloud is ignored
Header header
geometry_msgs/Pose sensor_pose
sensor_msgs/PointCloud cloud