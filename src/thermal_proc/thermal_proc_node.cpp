/*
 * thermal_proc_node.cpp
 *  _   _             _           _____         _
 * | \ | | ___  _   _| | ____ _  |_   _|__  ___| |__
 * |  \| |/ _ \| | | | |/ / _` |   | |/ _ \/ __| '_ \
 * | |\  | (_) | |_| |   < (_| |   | |  __/ (__| | | |
 * |_| \_|\___/ \__,_|_|\_\__,_|   |_|\___|\___|_| |_|
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of flir_gige.
 *
 *	Created on: 21/08/2014
 */

#include "flir_gige/thermal_proc/thermal_proc.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "thermal_proc");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    flir_gige::ThermalProc thermal_proc(nh, pnh);
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR_STREAM("flir_gige: thermal_proc: " << e.what());
    return -1;
  }

  return 0;
}
