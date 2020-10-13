
#include "ros/ros.h"
#include <iostream>

#include "radar.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "radar_driver_node");
  ros::NodeHandle n;

  radar::Radar radar;

  while (ros::ok()){
    ros::spinOnce();
  }

}