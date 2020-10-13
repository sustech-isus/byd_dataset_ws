
#include "radar.h"

namespace radar{

Radar::Radar() {

  radar_frame_ = "radar";

  // ROS Initialization
  radar_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar_points", 1);
  radar_tracks_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar_tracks", 1);

  if(CanInit() != 0){
    std::cerr << "CAN Init failed!" << std::endl;
    return;
  }

  std::cout << "Starting receiving thread..." << std::endl;
  receive_thread_ptr_ = std::make_shared<std::thread>(std::bind(&Radar::ReceiveThread, this));

}

//Radar::~Radar() {
//  if(receive_thread_ptr_ != nullptr){
//    receive_thread_ptr_->join();
//  }
//
//}


int Radar::CanInit() {
  std::cout << "CanInit: Reading messages on channel " << channel_ << std::endl;
  canInitializeLibrary();
  can_handle_ = canOpenChannel(channel_, canOPEN_EXCLUSIVE | canOPEN_REQUIRE_EXTENDED | canOPEN_ACCEPT_VIRTUAL);
  if(can_handle_ < 0){
    std::cerr << "CanInit: canOpenChannel failed! Channel: " << channel_ << std::endl;
  }

  can_status_ = canSetBusParams(can_handle_, canBITRATE_500K, 0, 0, 0, 0, 0);
  if (can_status_ != canOK) {
    std::cerr << "CanInit: canSetBusParams failed!" << std::endl;
    return -1;
  }

  can_status_ = canAccept(can_handle_, 0x0F0L, canFILTER_SET_MASK_STD);
  if (can_status_ < 0) {
    std::cerr << "canAccept set mask failed!" << std::endl;
    return -1;
  }
  can_status_ = canAccept(can_handle_, 0x060L, canFILTER_SET_CODE_STD);
  if (can_status_ < 0) {
    std::cerr << "canAccept set code failed!" << std::endl;
    return -1;
  }

  can_status_ = canBusOn(can_handle_);
  if (can_status_ != canOK) {
    std::cerr << "CanInit: canBusOn failed!" << std::endl;
    return -1;
  }
  return 0;
}

int Radar::CanOff() const {
  canBusOff(can_handle_);
  canClose(can_handle_);
  canUnloadLibrary();
  std::cout << "CanOff: Success!" << std::endl;
  return 0;
}

void Radar::ReceiveThread() {

  bool head = false;
  bool first_head = false;

  auto can_msg_ptr = std::make_shared<CanMsg>();

  while(!ros::isShuttingDown())
  {
      can_status_ = canReadWait(can_handle_,
                            &can_msg_ptr->id,
                            &can_msg_ptr->msg,
                            &can_msg_ptr->dlc,
                            &can_msg_ptr->flag,
                            &can_msg_ptr->time,
                            30);

      if (can_status_ == canOK) {
        auto current_rec_time = ros::Time::now();
        if (print_frame) {
          PrintFrameInfo(channel_, can_msg_ptr->id, can_msg_ptr->dlc, can_msg_ptr->msg);
        }

        switch (can_msg_ptr->id) {
          case 0x70A:head = true;
            first_head = true;
            timestamp_ = current_rec_time;
            break;
          case 0x70C:
            if (head && first_head) {
              pcl_radar_points_.push_back(CanMsgToRadarPointMsg(can_msg_ptr->msg));
            }
            break;
          case 0x70E:
            if (head && first_head) {
              pcl_radar_tracks_.push_back(CanMsgToRadarTrackMsg(can_msg_ptr->msg));
            }
            break;
          case 0x70F:
            auto duration = ros::Time::now() - timestamp_;
            if (head && first_head) {
              head = false;
              if(!pcl_radar_points_.empty() || !pcl_radar_tracks_.empty()){
                PublishMsg();
                pcl_radar_points_.clear();
                pcl_radar_tracks_.clear();
              }
            }
            break;
        };
    }
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(10ms);
  }
  CanOff();
}


void Radar::PrintFrameInfo(int can_index, unsigned int can_frame_id, int data_len, const unsigned char *data) {
  std::cout << "CAN" << can_index+1 << " RX | ID: 0x" << std::hex << can_frame_id;
  for(int i = 0; i < data_len; i++)
  {
    std::cout << " ";
    std::cout.width(2);
    std::cout.fill('0');
    std::cout << std::hex << (unsigned int)data[i];
  }
  std::cout << std::endl;
}

void Radar::PublishMsg() {

  pcl_radar_points_.width = pcl_radar_points_.size();
  sensor_msgs::PointCloud2 points_msg;
  pcl::toROSMsg(pcl_radar_points_, points_msg);
  points_msg.header.frame_id = radar_frame_;
  points_msg.header.stamp = timestamp_;
  radar_points_pub_.publish(points_msg);

  pcl_radar_tracks_.width = pcl_radar_tracks_.size();
  sensor_msgs::PointCloud2 tracks_msg;
  pcl::toROSMsg(pcl_radar_tracks_, tracks_msg);
  tracks_msg.header.frame_id = radar_frame_;
  tracks_msg.header.stamp = timestamp_;
  radar_tracks_pub_.publish(tracks_msg);
}



}