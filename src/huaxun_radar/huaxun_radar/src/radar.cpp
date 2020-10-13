//
// Created by kevinlad on 2020/4/8.
//

#include "radar.h"



Radar::Radar(char* mode) {
  std::string test_parse = "test_parse";
  std::string test_can = "test_can";
  std::string mode_str = mode;
  if(!mode_str.compare(test_parse)){
    TestParse();
  } else if (!mode_str.compare(test_can)){
    TestSendRec();
  } else {
    Init();
  }
}

Radar::~Radar() {
  receive_thread_ptr_->join();
  std::cout << "Close device!" << std::endl;
  usleep(100000); // delay100ms
  VCI_ResetCAN(VCI_USBCAN2, 0, 0); //Reset CAN1
  usleep(100000); // delay100ms
  VCI_ResetCAN(VCI_USBCAN2, 0, 1); //Reset CAN2
  usleep(100000); // delay100ms
  VCI_CloseDevice(VCI_USBCAN2,0) ;// Close Device
  std::cout << "Close success!" << std::endl;
}

void Radar::Init() {

  // ROS Initialization

  radar_pub_ = nh_.advertise<TargetArrayMsg>("/radar_targets", 10);
  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/radar_pointcloud", 1);
  pointcloud_raw_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/radar_pointcloud_raw", 1);

  // USB-CAN Initialization

  num_ = VCI_FindUsbDevice2(pInfo1);
  std::cout << ">>Find " << num_ << " USB-CAN devices" << std::endl;

  if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//Open Device
  {
    std::cout << ">>Open deivce success!" << std::endl;
  }else
  {
    std::cerr << ">>Open deivce error!" << std::endl;
    exit(1);
    //TODO: Safe exit
  }

  auto read_status = VCI_ReadBoardInfo(VCI_USBCAN2, 0, &pInfo);
  if(read_status == 1){
    // TODO: print device info
  } else{
    std::cerr << ">>Get VCI_ReadBoardInfo error!" << std::endl;
    exit(1);
  }

  //TODO: rosparam support
  config_.AccCode=0;
  config_.AccMask=0xFFFFFFFF;
  config_.Filter=1;// Receive All frames
  config_.Timing0=0x00;  // 500 kbps
  config_.Timing1=0x1C;
  config_.Mode=0;// Normal mode

  if(VCI_InitCAN(VCI_USBCAN2,0,0,&config_)!=1)
  {
    std::cerr << ">>Init CAN1 error"<< std::endl;
    VCI_CloseDevice(VCI_USBCAN2,0);
  }

  if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
  {
    std::cerr << ">>Start CAN1 error" << std::endl;
    VCI_CloseDevice(VCI_USBCAN2,0);

  }

  if(VCI_InitCAN(VCI_USBCAN2,0,1,&config_)!=1)
  {
    std::cerr << ">>Init can2 error" << std::endl;
    VCI_CloseDevice(VCI_USBCAN2,0);

  }
  if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
  {
    std::cerr << ">>Start can2 error" << std::endl;
    VCI_CloseDevice(VCI_USBCAN2,0);
  }

  receive_thread_ptr_ = std::make_shared<std::thread>(std::bind(&Radar::ReceiveThread, this));

}

void Radar::ReceiveThread() {
  int reclen=0;
//  VCI_CAN_OBJ rec_buffer[3000];//接收缓存，设为3000为佳。
  unsigned int i,j;
  int ind=0;
  bool head = false;
  bool first_head = false;
  bool tail = false;

  while(!ros::isShuttingDown())
  {
    if((reclen = VCI_Receive(VCI_USBCAN2,0,ind,rec_buffer_,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
    {

      auto current_rec_time = ros::Time::now();

      for(j=0; j<reclen; j++)
      {
        if(print_frame) {
          PrintFrameInfo(ind, rec_buffer_[j]);
        }

        switch (rec_buffer_[j].ID){
          case 0x70A:
            head = true;
            first_head = true;
            radar_target_array_msg_ = std::make_shared<TargetArrayMsg>();
            radar_target_array_msg_->header.stamp = current_rec_time;
            break;
          case 0x70C:
            if(head && first_head) {
              auto point_msg = ParsePointFrame(rec_buffer_[j].Data);
              radar_target_array_msg_->radar_point_array.push_back(point_msg);
            }
            break;
          case 0x70E:
            if(head && first_head) {
              auto track_msg = ParseTrackFrame(rec_buffer_[j].Data);
              radar_target_array_msg_->radar_track_array.push_back(ParseTrackFrame(rec_buffer_[j].Data));
            }
            break;
          case 0x70F:
            tail = true;
            head = false;
            PublishMsg();
            //reset
            last_radar_target_array_msg_ = radar_target_array_msg_;
            radar_target_array_msg_.reset();
            break;
        };
      }
    }
    ind=!ind;//变换通道号，以便下次读取另一通道，交替读取。
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(30ms);
  }

//  usleep(100000);//延时100ms。
//  VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
//  usleep(100000);//延时100ms。
//  VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
//  usleep(100000);//延时100ms。
//  VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
//  printf("run thread exit\n");//退出接收线程

}

Radar::PointMsg Radar::ParsePointFrame(const BYTE *data) {
  auto point_msg = Radar::PointMsg();
  point_msg.target_id       = (uint8_t)(data[0]);
  point_msg.target_peak_val = (uint16_t)(data[1] + ((data[2] & 0x1f) << 8));
  auto raw_target_distance  = (float)((data[2] >> 5) + (data[3] << 3));
  point_msg.car_speed       = (uint8_t)data[4];
  auto raw_target_angle     = (float)(data[5] + ((data[6] & 0x0f) << 8));
  auto raw_target_speed     = (float)((data[6] >> 4) + (data[7] << 4));

  point_msg.target_distance = raw_target_distance / 10.0f;
  point_msg.target_angle    = (raw_target_angle * 0.1f) - 180.0f;
  point_msg.target_speed    = (raw_target_speed * 0.025f) - 50.0f;
  return point_msg;
}

Radar::TrackMsg Radar::ParseTrackFrame(const BYTE *data) {
  auto track_msg = Radar::TrackMsg();
  track_msg.target_id = (uint8_t)(data[0] & 0x3f);
  auto raw_v_x = (float)(((data[2] & 0x07) << 10) + (data[1] << 2) + (data[0] >> 6));
  auto raw_v_y = (float)((data[3] << 5) + (data[2] >> 3));
  auto raw_r_x = (float)((data[5] << 8) + data[4]);
  auto raw_r_y = (float)((data[7] << 8) + data[6]);

  track_msg.x_velocity = raw_v_x / 50.0f - 50.0f;
  track_msg.y_velocity = raw_v_y / 50.0f - 50.0f;
  track_msg.x_distance = raw_r_x / 50.0f - 200.0f;
  track_msg.y_distance = raw_r_y / 50.0f - 10.0f;

  return track_msg;
}

void Radar::PrintFrameInfo(int can_index, const VCI_CAN_OBJ &buffer_frame) {

//  std::cout << "CAN" << can_index+1 << " RX ID: " << std::hex << buffer_frame.ID;
//  if(buffer_frame.ExternFlag==0)
//    std::cout <<" Standard ";//帧格式：标准帧
//  if(buffer_frame.ExternFlag==1)
//    std::cout <<" Extend   ";//帧格式：扩展帧
//
//  std::cout << std::endl;
//
//  printf("DLC:0x%02X",buffer_frame.DataLen);//帧长度
//  printf(" data:0x");	//数据

  std::cout << "CAN" << can_index+1 << " RX | ID: 0x" << std::hex << buffer_frame.ID;

  for(int i = 0; i < buffer_frame.DataLen; i++)
  {
    std::cout << " ";
    std::cout.width(2);
    std::cout.fill('0');
    std::cout << std::hex << (unsigned int)buffer_frame.Data[i];
  }
  std::cout << std::endl;
}

void Radar::PublishMsg() {
  radar_pub_.publish(*radar_target_array_msg_);
  auto msg_timestamp = radar_target_array_msg_->header.stamp;

  //publish pointcloud for visualization
  sensor_msgs::PointCloud pointcloud_raw_msg;
  pointcloud_raw_msg.header.frame_id = radar_frame_;
  pointcloud_raw_msg.header.stamp = msg_timestamp;
  pointcloud_raw_msg.points.resize(radar_target_array_msg_->radar_point_array.size());
  for(unsigned int m = 0; m < radar_target_array_msg_->radar_point_array.size(); m++){
    auto p = radar_target_array_msg_->radar_point_array[m];
    pointcloud_raw_msg.points[m].x = std::sin(p.target_angle / 180.0 * M_PI) * p.target_distance;
    pointcloud_raw_msg.points[m].y = std::cos(p.target_angle / 180.0 * M_PI) * p.target_distance;
    pointcloud_raw_msg.points[m].z = 0;
  }
  pointcloud_raw_pub_.publish(pointcloud_raw_msg);

  sensor_msgs::PointCloud pointcloud_msg;
  pointcloud_msg.header.frame_id = radar_frame_;
  pointcloud_msg.header.stamp = msg_timestamp;
  auto size = radar_target_array_msg_->radar_track_array.size();
  pointcloud_msg.points.resize(size);
  for(unsigned int m = 0; m < size; m++){
    auto p = radar_target_array_msg_->radar_track_array[m];
    pointcloud_msg.points[m].x = p.x_distance;
    pointcloud_msg.points[m].y = p.y_distance;
    pointcloud_msg.points[m].z = 0;
  }
  pointcloud_pub_.publish(pointcloud_msg);
}

bool Radar::TestParse() {

  std::cout << "TestParse!" << std::endl;
  std::cout << "*****************" << std::endl;


  // Test Parse Point frame
  uint8_t data[8];
  data[7] = 0x7D;
  data[6] = 0x06;
  data[5] = 0xFA;
  data[4] = 0x00;
  data[3] = 0x25;
  data[2] = 0xB4;
  data[1] = 0xDB;
  data[0] = 0x01;
  auto point_msg = ParsePointFrame(data);


  std::cout << "Point msg: " << std::endl;
  std::cout << "id: " << (unsigned int)point_msg.target_id << std::endl;
  std::cout << "peak_val " << std::dec << (unsigned int)point_msg.target_peak_val << std::endl;
  std::cout << "target_distance: " << point_msg.target_distance << std::endl;
  std::cout << "Car Speed: " << (unsigned int)point_msg.car_speed << std::endl;
  std::cout << "target angle: " << point_msg.target_angle << std::endl;
  std::cout << "target velocity: " << point_msg.target_speed << std::endl;


  // Test Parse Track msg
  uint8_t data_track[8];
  data_track[7] = 0x01;
  data_track[6] = 0x0c;
  data_track[5] = 0x27;
  data_track[4] = 0x32;
  data_track[3] = 0x4e;
  data_track[2] = 0x22;
  data_track[1] = 0x70;
  data_track[0] = 0x42;

  auto track_msg = ParseTrackFrame(data_track);
  std::cout << "*****************" << std::endl;
  std::cout << "Track msg:" << std::endl;
  std::cout << "id: " << (unsigned int)track_msg.target_id << std::endl;
  std::cout << "v_x: " << track_msg.x_velocity << std::endl;
  std::cout << "v_y: " << track_msg.y_velocity << std::endl;
  std::cout << "r_x: " << track_msg.x_distance << std::endl;
  std::cout << "r_y: " << track_msg.y_distance << std::endl;


}

bool Radar::TestSendRec(){

};
