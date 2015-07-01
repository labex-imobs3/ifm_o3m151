/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 * 
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
  private_nh.param("frame_id", config_.frame_id, std::string("o3m151"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  // get model name, validate string, determine packet rate
  private_nh.param("model", config_.model, std::string("o3m151"));
  std::string deviceName(std::string("O3M151"));

  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = 10;
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
    {
      input_.reset(new o3m151_driver::InputPCAP(private_nh,
                                                  diag_freq,
                                                  dump_file));
    }
  else
    {
      input_.reset(new o3m151_driver::InputSocket(private_nh));
    }

  // raw data output topic
  output_ = node.advertise<sensor_msgs::PointCloud2>("o3m151_points", 10);

//  pcl::visualization::PCLVisualizer * pcl_viewer = new pcl::visualization::PCLVisualizer ("3D Viewer");
//  viewer_.reset(pcl_viewer);
//  viewer_->setBackgroundColor (0, 0, 0);
  pcl::PointCloud<pcl::PointXYZ>* pc_cart_packet = new pcl::PointCloud<pcl::PointXYZ>;
  cart_packet_.reset(pc_cart_packet);
//  viewer_->addPointCloud<pcl::PointXYZ> (cart_packet_, "cartesian cloud");
//  viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cartesian cloud");
//  viewer_->addCoordinateSystem (1.0);
//  viewer_->initCameraParameters ();
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool O3M151Driver::poll(void)
{
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  //sensor_msgs::PointCloud2Ptr pc(new sensor_msgs::PointCloud2);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());

  // Since the o3m151 delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  cart_packet_->points.clear();
  while (true)
    {
      // keep reading until full packet received
      int rc = input_->getPacket(*cart_packet_);
      if (rc == 0) break;       // got a full packet?
      if (rc < 0) return false; // end of file reached?
    }

  // publish message using time of last packet read
  ROS_DEBUG("Publishing a full O3M151 scan. %d", cart_packet_->points.size());
  //ROS_DEBUG_STREAM(cart_packet_->points.at(0));
  ros::Time now = ros::Time::now();
  //pcl_conversions::toPCL()
  cart_packet_->header.stamp = now.toNSec() / 1e3;
  cart_packet_->header.frame_id = config_.frame_id;
  cart_packet_->height = 1;
  cart_packet_->width = cart_packet_->points.size();
//  viewer_->updatePointCloud(cart_packet_, "cartesian cloud");
//  viewer_->spinOnce();
  output_.publish(cart_packet_);

  // notify diagnostics that a message has been published, updating
  // its status
  diag_topic_->tick(now);
  diagnostics_.update();

  return true;
}

} // namespace o3m151_driver
