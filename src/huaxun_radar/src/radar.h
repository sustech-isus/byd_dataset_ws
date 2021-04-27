#ifndef SRC_RADAR_H
#define SRC_RADAR_H

#include <thread>
#include <memory>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

// Kvaser CANLIB
#include <canlib.h>

#include "protocol.h"

namespace radar{


class Device;

struct CanMsg;

class Radar {
 public:

  Radar();
  ~Radar();
 private:

  int CanInit();

  canHandle InitChannel(int channel_);
  int CloseChannel(canHandle can_handle_);

  void ReceiveThread(int channel_);

  void PrintFrameInfo(int can_index, unsigned int can_frame_id,int data_len, const unsigned char *data);

  //void PublishMsg();

  // template<class POINTS>
  // void PublishPointCloud2Msg(POINTS points, ros::Publisher pub);

 private:

  //void procDelphiMsg(CanMsg *msg);

  std::shared_ptr<std::thread> receive_thread_ptr_0;
  std::shared_ptr<std::thread> receive_thread_ptr_1;
  std::shared_ptr<std::thread> receive_thread_ptr_2;
  std::shared_ptr<std::thread> receive_thread_ptr_3;
  //std::string radar_frame_ = "radar";
  //std::string radar_targets_topic_ = "/radar_targets";
  //std::string radar_pointcloud_raw_topic_ = "/radar_pointcloud_raw";
  //std::string radar_pointcloud_topic_ = "/radar_pointcloud";

  bool print_frame = false;

  ros::NodeHandle nh_;

  std::map<int, Device*> devices;
};

}

#endif //SRC_RADAR_H
