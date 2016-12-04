/*
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the O3M151 3D LIDARs
 */

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include "driver.h"

namespace o3m151_driver
{

O3M151Driver::O3M151Driver(ros::NodeHandle node,
                               ros::NodeHandle private_nh)
{
  // use private node handle to get parameters
  private_nh.param("frame_id", frame_id_, std::string("o3m151"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  frame_id_ = tf::resolve(tf_prefix, frame_id_);

  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));

  // initialize diagnostics
  diagnostics_.setHardwareID(std::string("O3M151"));
  const double diag_freq = 25;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic("o3m151_packets", diagnostics_,
                                        FrequencyStatusParam(&diag_min_freq_,
                                                             &diag_max_freq_,
                                                             0.1, 10),
                                        TimeStampStatusParam()));

  // open O3M151 input device or file
  if (dump_file != "")
    input_.reset(new o3m151_driver::InputPCAP(private_nh,
                                                  diag_freq*19,   // Needs 19 packets to obtain a scan (cf wireshark)
                                                  dump_file));
  else
    input_.reset(new o3m151_driver::InputSocket(private_nh));

  // raw data output topic
  output_ = node.advertise<sensor_msgs::PointCloud2>("o3m151_points", 10);
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool O3M151Driver::poll(void)
{
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  //sensor_msgs::PointCloud2Ptr pc(new sensor_msgs::PointCloud2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());

  // Since the o3m151 delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  pc->points.clear();
  while (true)
  {
    // keep reading until full packet received
    int rc = input_->getPacket(*pc);
    if (rc == 0) break;       // got a full packet?
    if (rc < 0) return false; // end of file reached?
  }

  // publish message using time of last packet read
  ROS_DEBUG("Publishing a full O3M151 scan with %d points", pc->points.size());
  ros::Time now = ros::Time::now();
  /// Do not use pcl_conversions::toPCL() function for ROS hydro compatiblity !
  pc->header.stamp = now.toNSec() / 1e3;
  pc->header.frame_id = frame_id_;
  pc->height = 1;
  pc->width = pc->points.size();
  output_.publish(pc);

  // notify diagnostics that a message has been published, updating
  // its status
  diag_topic_->tick(now);
  diagnostics_.update();

  return true;
}

} // namespace o3m151_driver
