//
// Created by kevinlad on 2020/4/8.
//

#include "ros/ros.h"
#include "radar.h"
#include <iostream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "radar_driver_node");
  ros::NodeHandle n;

//  Radar radar_test_parse("test_parse");
  Radar radar("normal");

  while (ros::ok()){
    ros::spinOnce();
  }


}