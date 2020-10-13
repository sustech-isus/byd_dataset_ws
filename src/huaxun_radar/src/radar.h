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

struct CanMsg{
  long id = 0;
  unsigned char msg[8];
  unsigned int dlc = 0;
  unsigned int flag = 0;
  unsigned long time = 0;
};

class Radar {
 public:

  Radar();

 private:

  int CanInit();

  int CanOff() const;

  void ReceiveThread();

  void PrintFrameInfo(int can_index, unsigned int can_frame_id,int data_len, const unsigned char *data);

  void PublishMsg();

 private:

  std::shared_ptr<std::thread> receive_thread_ptr_;

  std::string radar_frame_ = "radar";
  std::string radar_targets_topic_ = "/radar_targets";
  std::string radar_pointcloud_raw_topic_ = "/radar_pointcloud_raw";
  std::string radar_pointcloud_topic_ = "/radar_pointcloud";

  bool print_frame = false;

  ros::NodeHandle nh_;
  ros::Publisher radar_tracks_pub_;
  ros::Publisher radar_points_pub_;

  canHandle can_handle_;
  canStatus can_status_;
  int channel_ = 0;

  ros::Time timestamp_;
  pcl::PointCloud<PclRadarPointType> pcl_radar_points_;
  pcl::PointCloud<PclRadarTrackType> pcl_radar_tracks_;



};

}

#endif //SRC_RADAR_H
