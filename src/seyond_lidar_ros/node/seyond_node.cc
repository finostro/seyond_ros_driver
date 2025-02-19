/*
 *  Copyright (C) 2024 Seyond Inc.
 *
 *  License: Apache License
 *
 *  $Id$
 */

#include <signal.h>
#include <memory>
#ifdef ROS_FOUND
#include <ros/ros.h>
#include <ros/package.h>
#include "src/driver/ros1_driver_adapter.hpp"
#elif ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include "src/driver/ros2_driver_adapter.hpp"
#endif

static void shutdown_callback(int sig) {
#ifdef ROS_FOUND
  ros::shutdown();
#elif ROS2_FOUND
  rclcpp::shutdown();
#endif
}


int main(int argc, char *argv[]) {
#ifdef ROS_FOUND
  ros::init(argc, argv, "seyond", ros::init_options::NoSigintHandler);
#elif ROS2_FOUND
  rclcpp::init(argc, argv);
#endif

  signal(SIGINT, shutdown_callback);

  std::shared_ptr<ROSNode> ros_driver_ptr = std::make_shared<ROSNode>();
  ros_driver_ptr->init();
  ros_driver_ptr->start();
  ros_driver_ptr->spin();
  return 0;
}
