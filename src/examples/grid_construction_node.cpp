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
 * Ros node that uses grid construction from ray_tracer.h
 *
 * \author Bhaskara Marthi
 */

#include <occupancy_grid_utils/ray_tracer.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>

namespace grid_construction_node
{

namespace gm=geometry_msgs;
namespace sm=sensor_msgs;
namespace nm=nav_msgs;
namespace gu=occupancy_grid_utils;

using std::vector;
using std::string;

typedef boost::mutex::scoped_lock Lock;
typedef boost::shared_ptr<gu::LocalizedCloud> CloudPtr;
typedef boost::shared_ptr<gu::LocalizedCloud const> CloudConstPtr;
typedef boost::circular_buffer<CloudConstPtr> CloudBuffer;

class GridConstructionNode
{
public:
  GridConstructionNode ();

private:

  void scanCallback (const sm::LaserScan& scan);
  void buildGrid (const ros::WallTimerEvent& e);

  /****************************************
   * Params
   ****************************************/

  ros::NodeHandle nh_;
  const unsigned history_length_;
  const double resolution_;
  const string fixed_frame_;
  const string sensor_frame_;
  const double grid_construction_interval_;
  const double local_grid_size_;

  /****************************************
   * Associated objects
   ****************************************/

  tf::TransformListener tf_;
  ros::Subscriber scan_sub_;
  ros::Publisher grid_pub_;
  ros::WallTimer build_grid_timer_;
  boost::mutex mutex_;
  
  /****************************************
   * State
   ****************************************/
  
  CloudBuffer clouds_;
  CloudConstPtr last_cloud_;

};
  

template <class T>
T getPrivateParam(const string& name, const T& default_value)
{
  ros::NodeHandle nh("~");
  T value;
  nh.param(name, value, default_value);
  return value;
}


GridConstructionNode::GridConstructionNode () :
  history_length_(getPrivateParam("history_length", 100)), resolution_(getPrivateParam("resolution", 0.1)),
  fixed_frame_("map"), sensor_frame_("base_laser_link"), 
  grid_construction_interval_(getPrivateParam("grid_construction_interval", 0.3)),
  local_grid_size_(getPrivateParam("local_grid_size", 15.0)),
  scan_sub_(nh_.subscribe("base_scan", 1, &GridConstructionNode::scanCallback, this)),
  grid_pub_(nh_.advertise<nm::OccupancyGrid>("local_grid", 1)),
  build_grid_timer_(nh_.createWallTimer(ros::WallDuration(grid_construction_interval_), 
                                        &GridConstructionNode::buildGrid, this)),
  clouds_(history_length_)
{
}


void GridConstructionNode::scanCallback (const sm::LaserScan& scan)
{
  try {

    // We'll need the transform between sensor and fixed frames at the time when the scan was taken
    if (!tf_.waitForTransform(fixed_frame_, sensor_frame_, scan.header.stamp, ros::Duration(1.0)))
    {
      ROS_WARN_STREAM ("Timed out waiting for transform from " << sensor_frame_ << " to "
                       << fixed_frame_ << " at " << scan.header.stamp.toSec());
      return;
    }

    // Figure out current sensor position
    tf::Pose identity(tf::createIdentityQuaternion(), tf::Vector3(0, 0, 0));
    tf::Stamped<tf::Pose> odom_pose;
    tf_.transformPose(fixed_frame_, tf::Stamped<tf::Pose> (identity, ros::Time(), sensor_frame_), odom_pose);

    // Project scan from sensor frame (which varies over time) to odom (which doesn't)
    sm::PointCloud fixed_frame_cloud;
    laser_geometry::LaserProjection projector_;
    projector_.transformLaserScanToPointCloud (fixed_frame_, scan, fixed_frame_cloud, tf_);
    
    // Now transform back into sensor frame at a single time point
    sm::PointCloud sensor_frame_cloud;
    tf_.transformPointCloud (sensor_frame_, scan.header.stamp, fixed_frame_cloud, fixed_frame_, 
                             sensor_frame_cloud);

    // Construct and save LocalizedCloud
    CloudPtr loc_cloud(new gu::LocalizedCloud());
    loc_cloud->cloud.points = sensor_frame_cloud.points;
    tf::poseTFToMsg(odom_pose, loc_cloud->sensor_pose);
    loc_cloud->header.frame_id = fixed_frame_;
    Lock lock(mutex_);
    last_cloud_=loc_cloud;
  }
  catch (tf::TransformException& e) {
    ROS_INFO ("Not saving scan due to tf lookup exception: %s",
              e.what());
  }
}


void GridConstructionNode::buildGrid (const ros::WallTimerEvent& scan)
{
  if (last_cloud_) {
    {
      Lock lock(mutex_);
      clouds_.push_back(last_cloud_);
      last_cloud_.reset();
    }

    ROS_DEBUG_NAMED ("build_grid", "Building grid with %zu scans", clouds_.size());
    
    // Figure out current position
    gm::PoseStamped identity, odom_pose;
    identity.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    identity.header.frame_id = sensor_frame_;
    identity.header.stamp = ros::Time();
    tf_.transformPose(fixed_frame_, identity, odom_pose);

    // Set up map dimensions
    nm::MapMetaData info;
    info.origin.position.x = odom_pose.pose.position.x-local_grid_size_/2;
    info.origin.position.y = odom_pose.pose.position.y-local_grid_size_/2;
    info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
    info.resolution = resolution_;
    info.width = local_grid_size_/resolution_;
    info.height = local_grid_size_/resolution_;

    nm::OccupancyGrid fake_grid;
    fake_grid.info = info;
    gu::OverlayClouds overlay = gu::createCloudOverlay(fake_grid, fixed_frame_, 0.1, 10, 2);
    vector<CloudConstPtr> clouds(clouds_.begin(), clouds_.end());
    BOOST_FOREACH  (CloudConstPtr cloud, clouds_)
      gu::addCloud(&overlay, cloud);
    nm::OccupancyGrid::ConstPtr grid = gu::getGrid(overlay);

    ROS_DEBUG_NAMED ("build_grid", "Done building grid");
    
    grid_pub_.publish(grid);
  }
}

} // namespace grid_construction_node

int main (int argc, char** argv)
{
  ros::init(argc, argv, "grid_construction_node");
  grid_construction_node::GridConstructionNode node;
  ros::spin();
  return 0;
}
