/*
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver node for the O3M151 3D LIDARs.
 */

#include <ros/ros.h>
#include "driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "o3m151_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  // start the driver
  o3m151_driver::O3M151Driver dvr(node, private_nh);

  // loop until shut down or end of file
  while(ros::ok() && dvr.poll())
  {
    ros::spinOnce();
  }

  return 0;
}
