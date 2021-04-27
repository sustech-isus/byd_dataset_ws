
#include <ros/ros.h>
#include "byd_sdk.h"
#include <iostream>

int main(int argc, char** argv)
{
  GLogWrapper glog_wrapper("byd_sdk_node");
  ros::init(argc,argv, "byd_sdk_node");
  ros::NodeHandle nh;
  auto byd_sdk_ptr = std::make_unique<BydSDK>();
  int status = byd_sdk_ptr->Init(nh);
  while (status == ERR_BYD_AUTO_OK && ros::ok())
  {
    ros::spinOnce();
  }

}