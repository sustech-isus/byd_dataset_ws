
#include "radar.h"
// Kvaser CANLIB


namespace radar{


struct CanMsg{
  long id = 0;
  unsigned char msg[8];
  unsigned int dlc = 0;
  unsigned int flag = 0;
  unsigned long time = 0;
};



class Device
{
public:
    Device(ros::NodeHandle nh_, int device_id): nh_(nh_), device_id(device_id)
    {
        std::stringstream ss;
        ss << "/radar_points" << "/" << device_id;
        radar_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(ss.str(), 1);

        ss.str("");
        ss << "/radar_tracks" << "/" << device_id;
        radar_tracks_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(ss.str(), 1);

        ss.str("");
        ss << "radar_" << device_id;
        radar_frame_ = ss.str();

        state = FRAME_ENDED;
    }

    virtual ~Device()
    {

    }

    int get_device_id()const {return device_id; }

    void handle_msg(const CanMsg *can_msg_ptr)
    {

      switch (can_msg_ptr->id) {
          case 0x70A:
            if (state != FRAME_ENDED)
            {
                std::cerr << "device "<<device_id<< "received frame-start in state " << state << std::endl;
            }
            state = FRAME_STARTED;
            timestamp_ = ros::Time::now();
            break;
          case 0x70C:
            if (state == FRAME_STARTED) {
              pcl_radar_points_.push_back(CanMsgToRadarPointMsg(can_msg_ptr->msg));
            }
            else
            {
              std::cerr << "device "<<device_id << "received points in state " << state << std::endl;
            }
            
            break;
          case 0x70E:
            if (state == FRAME_STARTED) {
              pcl_radar_tracks_.push_back(CanMsgToRadarTrackMsg(can_msg_ptr->msg));
            }
            else{
              std::cerr << "device "<<device_id << "received tracks in state " << state << std::endl;
            }
            
            break;
          case 0x70F:
            {
              //auto duration = ros::Time::now() - timestamp_;
              if (state == FRAME_STARTED) {
                state = FRAME_ENDED;

                if (!pcl_radar_points_.empty() || !pcl_radar_tracks_.empty())
                {
                  //PublishMsg();
                  PublishPointCloud2Msg(pcl_radar_points_, radar_points_pub_);
                  PublishPointCloud2Msg(pcl_radar_tracks_, radar_tracks_pub_);
                  pcl_radar_points_.clear();
                  pcl_radar_tracks_.clear();
                }
              }
              else
              {
                std::cerr << "device "<<device_id << "received frame_end in state " << state << std::endl;
              }
              
            }
            break;
          default:
            std::cerr << "device "<<device_id  << "unknown msg id" << can_msg_ptr->id << std::endl;
            break;
        };
    }
private:

  template<class POINTS>
  void PublishPointCloud2Msg(POINTS points, ros::Publisher pub) {
    points.width = points.size();
    sensor_msgs::PointCloud2 points_msg;
    pcl::toROSMsg(points, points_msg);
    points_msg.header.frame_id = radar_frame_;
    points_msg.header.stamp = timestamp_;
    pub.publish(points_msg);

    if (0)
      std::cout<<"published pointcloud "<< radar_frame_ <<" "<<points.width<<std::endl;
  }

private:
  int device_id;
  std::string radar_frame_;
  ros::NodeHandle nh_;
  ros::Publisher radar_tracks_pub_;
  ros::Publisher radar_points_pub_;
  
  ros::Time timestamp_;
  pcl::PointCloud<PclRadarPointType> pcl_radar_points_;
  pcl::PointCloud<PclRadarTrackType> pcl_radar_tracks_;


  enum State{
    FRAME_ENDED,  
    FRAME_STARTED,
    
  } state;

};




Radar::Radar() {

  //radar_frame_ = "radar";

  // ROS Initialization
  // radar_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar_points", 1);
  // radar_tracks_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar_tracks", 1);

  devices.insert(std::pair<int, Device*>(0x0, new Device(nh_, 0x0)));
  devices.insert(std::pair<int, Device*>(0x11, new Device(nh_, 0x11)));
  devices.insert(std::pair<int, Device*>(0x12, new Device(nh_, 0x12)));
  devices.insert(std::pair<int, Device*>(0x13, new Device(nh_, 0x13)));
  devices.insert(std::pair<int, Device*>(0x14, new Device(nh_, 0x14)));
  devices.insert(std::pair<int, Device*>(0x22, new Device(nh_, 0x22)));
  
  if(CanInit() != 0){
    std::cerr << "CAN Init failed!" << std::endl;
    return;
  }

  std::cout << "Starting receiving thread..." << std::endl;
  receive_thread_ptr_0 = std::make_shared<std::thread>(std::bind(&Radar::ReceiveThread, this, 0));
  receive_thread_ptr_1 = std::make_shared<std::thread>(std::bind(&Radar::ReceiveThread, this, 1));
  receive_thread_ptr_2 = std::make_shared<std::thread>(std::bind(&Radar::ReceiveThread, this, 2));
  receive_thread_ptr_3 = std::make_shared<std::thread>(std::bind(&Radar::ReceiveThread, this, 3));


}

Radar::~Radar() {
  canUnloadLibrary();
}


canHandle Radar::InitChannel(int channel_)
{
  canHandle can_handle_;
  canStatus can_status_;

  can_handle_ = canOpenChannel(channel_, canOPEN_EXCLUSIVE | canOPEN_REQUIRE_EXTENDED | canOPEN_ACCEPT_VIRTUAL);
  if(can_handle_ < 0){
    std::cerr << "CanInit: canOpenChannel failed! Channel: " << channel_ << std::endl;
  }

  can_status_ = canSetBusParams(can_handle_, canBITRATE_500K, 0, 0, 0, 0, 0);
  if (can_status_ != canOK) {
    std::cerr << "CanInit: canSetBusParams failed!" << std::endl;
    return -1;
  }

  // can_status_ = canAccept(can_handle_, 0x0F0L, canFILTER_SET_MASK_STD);
  // if (can_status_ < 0) {
  //   std::cerr << "canAccept set mask failed!" << std::endl;
  //   return -1;
  // }
  // can_status_ = canAccept(can_handle_, 0x060L, canFILTER_SET_CODE_STD);
  // if (can_status_ < 0) {
  //   std::cerr <<  "canAccept set code failed!" << std::endl;
  //   return -1;
  // }

  can_status_ = canBusOn(can_handle_);
  if (can_status_ != canOK) {
    std::cerr << "CanInit: canBusOn failed!" << std::endl;
    return -1;
  }

  return can_handle_;
}


int Radar::CanInit() {
  
  canInitializeLibrary();
 
  return 0;
}

int Radar::CloseChannel(canHandle can_handle_) {
  canBusOff(can_handle_);
  canClose(can_handle_);
  std::cout << "CloseChannel: Success!" << std::endl;
  return 0;
}



void Radar::ReceiveThread(int channel_) {
  canStatus can_status_;
  canHandle can_handle_ = InitChannel(channel_);

  if (can_handle_ < 0)
  {
    std::cerr << "CanInit: open channel failed!" << channel_ << std::endl;
  }

  CanMsg msg;
  auto can_msg_ptr = &msg;
  
  while(!ros::isShuttingDown())
  {
      can_status_ = canReadWait(can_handle_,
                            &can_msg_ptr->id,
                            &can_msg_ptr->msg,
                            &can_msg_ptr->dlc,
                            &can_msg_ptr->flag,
                            &can_msg_ptr->time,
                            0xFFFFFFFF); //30);

      if (can_status_ == canOK) {
        auto current_rec_time = ros::Time::now();
        if (print_frame) {
          PrintFrameInfo(channel_, can_msg_ptr->id, can_msg_ptr->dlc, can_msg_ptr->msg);
        }

        /*
          0x709
        0x70911
          ^^^------ message id
             ^^---- device id
        0x70912
        0x70913
        ...
        */
        int device_id = 0x00;
        if ((can_msg_ptr->id & 0x0ff000) != 0)
        {
            device_id = can_msg_ptr->id & 0xff;
            can_msg_ptr->id >>= 8;
        }
        
        auto device_it = devices.find(device_id);

        if (device_it == devices.end())
        {
          std::cerr << "unknonw device id" << device_id << std::endl;
        }
        else
        {
            device_it->second->handle_msg(can_msg_ptr);
        }
    }
    else{
      std::cout<<"can status "<< can_status_ << std::endl;
    }

    //using namespace std::chrono_literals;
    //std::this_thread::sleep_for(10ms);
  }

  CloseChannel(can_handle_);
}



// // tricky impl
// inline std::uint64_t readBits(std::uint64_t d, int start, int length)
// {
//   int byte = start/8;
//   int bit = start%8;
//   int startInU64 = 8 * (7-byte) + bit - length + 1;

//   uint64_t ret = d << (64- startInU64 - length);
//   ret = ret >> (64-length);
//   return ret;
// }

// inline std::uint64_t makeUint64(const unsigned char *data, int length)
// {
//   std::uint64_t d = 0;
//   for (int i=0; i < 8 && i < length; i++)
//   {
//     d = d*0x100 + data[i];
//   }
//   return d;
// }

// /*
//  SG_ CAN_TX_TRACK_GROUPING_CHANGED : 1|1@0+ (1,0) [0|0] "" Vector__XXX
//  SG_ CAN_TX_TRACK_ONCOMING : 0|1@0+ (1,0) [0|0] "" Vector__XXX
//  SG_ CAN_TX_TRACK_LAT_RATE : 7|6@0- (0.25,0) [-8|7.75] "" Vector__XXX
//  SG_ CAN_TX_TRACK_BRIDGE_OBJECT : 39|1@0+ (1,0) [0|0] "" Vector__XXX
//  SG_ CAN_TX_TRACK_WIDTH : 37|4@0+ (0.5,0) [0|7.5] "m" Vector__XXX
//  SG_ CAN_TX_TRACK_STATUS : 15|3@0+ (1,0) [0|7] "" Vector__XXX
//  SG_ CAN_TX_TRACK_ROLLING_COUNT : 38|1@0+ (1,0) [0|1] "" Vector__XXX
//  SG_ CAN_TX_TRACK_RANGE_RATE : 53|14@0- (0.01,0) [-81.92|81.91] "m/s" Vector__XXX
//  SG_ CAN_TX_TRACK_RANGE_ACCEL : 33|10@0- (0.05,0) [-25.6|25.55] "m/s/s" Vector__XXX
//  SG_ CAN_TX_TRACK_RANGE : 18|11@0+ (0.1,0) [0|204.7] "m" Vector__XXX
//  SG_ CAN_TX_TRACK_MED_RANGE_MODE : 55|2@0+ (1,0) [0|3] "" Vector__XXX
//  SG_ CAN_TX_TRACK_ANGLE : 12|10@0- (0.1,0) [-51.2|51.1] "deg" Vector__XXX
//  */
// inline PclDelphiRadarTrackType CanMsgToDelphiRadarTrackMsg(const unsigned char *data) {

//   std::uint64_t  d = makeUint64(data, 8);

//   if (d != 0)
//   {
//     std::cout<<"track data " << std::hex << d << std::endl;
//   }

//   PclDelphiRadarTrackType track;
  
//   track.oncoming =          readBits(d, 0,  1);
//   track.grouping_changed =  readBits(d, 1,  1);
//   track.lat_rate =          readBits(d, 7,  6);
//   track.bridge_object =     readBits(d, 39, 1);
//   track.width =             readBits(d, 37, 4);
//   track.status =            readBits(d, 15, 3);
//   track.rolling_count =     readBits(d, 38, 1);
//   track.range_rate =        readBits(d, 53, 14);
//   track.range_accel =       readBits(d, 33, 10);
//   track.range =             readBits(d, 18, 11);
//   track.med_range_mode =    readBits(d, 55, 2);
//   track.angle =             readBits(d, 12, 10);

//   float angle = track.angle * 0.1 - 51.2;
//   float range = track.range * 0.1;

//   track.x  = range * std::sin(angle / 180.0 * M_PI);
//   track.y  = range * std::cos(angle / 180.0 * M_PI);
  
//   return track;
// }


// void Radar::procDelphiMsg(CanMsg *msg)
// {
//   if (msg->id >= 0x500 && msg->id <= 0x53f)
//   {
//     // front radar, left/right
//     auto track = CanMsgToDelphiRadarTrackMsg(msg->msg);

//     if (track.x != 0)
//     {
//       PrintFrameInfo(channel_, msg->id, msg->dlc, msg->msg);
//     }
//     track.id = msg->id - 0x500;
//     pcl_delphi_radar_tracks_front_.push_back(track);
    
//     if (track.id == 0x3f) //last one
//     {
//       //build one point cloud 
//       PublishPointCloud2Msg(pcl_delphi_radar_tracks_front_, delphi_front_radar_tracks_pub_);
//       pcl_delphi_radar_tracks_front_.clear();
//     }
//   }
//   else if (msg->id >= 0x600 && msg->id <= 0x63f)
//   {
//     // rear radar, left/right
//     auto track = CanMsgToDelphiRadarTrackMsg(msg->msg);
//     track.id = msg->id - 0x600;
//     pcl_delphi_radar_tracks_rear_.push_back(track);

//     if (track.x != 0)
//     {
//       PrintFrameInfo(channel_, msg->id, msg->dlc, msg->msg);
//     }

//     if (track.id == 0x3f) //last one
//     {
//       //build one point cloud 
//       PublishPointCloud2Msg(pcl_delphi_radar_tracks_rear_, delphi_rear_radar_tracks_pub_);
//       pcl_delphi_radar_tracks_rear_.clear();
//     }
//   }
// }


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



// template<class POINTS>
// void Radar::PublishPointCloud2Msg(POINTS points, ros::Publisher pub) {
//   points.width = points.size();
//   sensor_msgs::PointCloud2 points_msg;
//   pcl::toROSMsg(points, points_msg);
//   points_msg.header.frame_id = radar_frame_;
//   points_msg.header.stamp = timestamp_;
//   pub.publish(points_msg);
// }

// void Radar::PublishMsg() {
//   PublishPointCloud2Msg(pcl_radar_points_, radar_points_pub_);
//   PublishPointCloud2Msg(pcl_radar_tracks_, radar_tracks_pub_);
// }

}