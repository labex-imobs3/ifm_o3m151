/* -*- mode: C++ -*- */
/*
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver interface for the O3M151 3D LIDARs
 */

#ifndef _O3M151_DRIVER_H_
#define _O3M151_DRIVER_H_ 1

#include <string>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <pcl/visualization/cloud_viewer.h>

#include <o3m151_driver/input.h>

namespace o3m151_driver
{

class O3M151Driver
{
public:

  O3M151Driver(ros::NodeHandle node,
                 ros::NodeHandle private_nh);
  ~O3M151Driver() {}

  bool poll(void);

private:

  // configuration parameters
  std::string frame_id_;            ///< tf frame ID

  boost::shared_ptr<Input> input_;
  ros::Publisher output_;

  /** diagnostics updater */
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
};

} // namespace o3m151_driver

#endif // _O3M151_DRIVER_H_
