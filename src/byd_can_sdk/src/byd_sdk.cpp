
#include "byd_sdk.h"
#include <unistd.h>

int BydSDK::Init(const ros::NodeHandle& nh)
{
  nh_ = nh;
  vehicle_odometry_pub_ = nh_.advertise<byd_sdk::VehicleOdometry>("/vehicle/odometry",1);
  control_info_pub_ = nh_.advertise<byd_sdk::ControlInfo>("/vehicle/controlinfo",1);
  vehicle_status_pub_ = nh_.advertise<byd_sdk::VehicleStatus>("/vehicle/status",1);



  LOG_INFO << "Start Byd Auto Api Data Initialization ...";
  CanDataInit();
  byd_sdk_status_ = BydAutoApiDataInit(&gDataBuf);
 // std::cout << byd_sdk_status_ << std::endl;
  switch (byd_sdk_status_) {
    case ERR_BYD_AUTO_MEM_MALLOC:
      LOG_ERROR << "BYD CAN Memory Malloc Error";
      break;
    case ERR_BYD_AUTO_OK:
      LOG_INFO << "BydAutoApiDataInit SUCCESS...";
      break;
    case ERR_BYD_AUTO_CAN_ENABLE:
    default:
      LOG_ERROR << "BYD CAN Enable Error";
      break;
  }

  LOG_INFO << "Start Byd Auto Api Enable ..." << std::endl;
  byd_sdk_status_ = BydAutoApiEnable();
  switch (byd_sdk_status_) {
    case ERR_BYD_AUTO_MEM_MALLOC:
      LOG_ERROR << "BYD CAN Memory Malloc Error";
      break;
    case ERR_BYD_AUTO_OK:
      LOG_INFO << "AutoApi SUCCESS...";
      break;
    case ERR_BYD_AUTO_CAN_ENABLE:
    default:
      LOG_ERROR << "BYD CAN Enable Error";
      break;
  }

  if(byd_sdk_status_ == ERR_BYD_AUTO_OK) {
    LOG_INFO << "Start Receive Thread ...";
    receive_thread_ptr_ = std::make_unique<std::thread>(std::bind(&BydSDK::ReceiveThread, this));
  }

  return byd_sdk_status_;

}



void BydSDK::ReceiveThread()
{
  while (ros::ok()){
    usleep(10000);// sleep 10 ms 10ms = 10000 us
    if (byd_sdk_status_ == ERR_BYD_AUTO_OK) {

      PublishVehicleOdometry();

      PublishControlInfo();

      PublishVehicleStatus();

    }
  }
  LOG_INFO << "Exit ReceiveThread...";
  BydAutoApiDisable();
  LOG_INFO << "BydAutoApiDisable Success";
}

int BydSDK::PublishVehicleOdometry() {
  auto current_stamp = ros::Time::now();

  byd_sdk::VehicleOdometry msg;
  msg.header.stamp = current_stamp;
  msg.header.frame_id = "vehicle";
  int sdk_status = ERR_BYD_AUTO_OK;

  //*********************************************
  // Publish AckermannDrive Msg
  // Get vehicle speed return int32_t
  sdk_status = GetVelocityInfo(&gData.m_nVelocity, &gData.m_nTimeStamp);
  switch (sdk_status) {
    case ERR_BYD_AUTO_OK: {
      constexpr double factor = 0.06875 / 3.6;  // PH = INT * 0.06875km/h, Factor = PH / 3.6
      msg.speed = gData.m_nVelocity * factor; // m/s
      std::cout << "the vechile speed is: " << gData.m_nVelocity * factor << std::endl;
      break;
    }
    case ERR_BYD_AUTO_INVALID_PARAM:
      LOG_ERROR << "GetVelocityInfo have the wrong param";
      break;
    case ERR_BYD_AUTO_READ:
    default:
      LOG_ERROR << "CAN read wrong";
      break;
  }
  // Get angular of the Steering wheel
  sdk_status = GetAngularInfo(&gData.m_nAngular, &gData.m_nTimeStamp);
  switch (sdk_status) {
    case ERR_BYD_AUTO_OK:
      if (gData.m_nAngular != 0xFFFF) {
        double angle = gData.m_nAngular * (-0.1) + 780; // Degree
        msg.steering_angle = angle / 180.0 * M_PI; // Radian
        std::cout << "the angular of the steering wheel is: " << angle / 180.0 * M_PI << std::endl;
      }
      break;
    case ERR_BYD_AUTO_INVALID_PARAM:
      LOG_ERROR << "GetAngularInfo have the wrong param";
      break;
    case ERR_BYD_AUTO_READ:
    default:
      LOG_ERROR << "CAN read wrong";
      break;
  }
  // Get the rotation speed info of the Steering wheel
  sdk_status = GetRotationSpeedInfo(&gData.m_nRotationSpeed, &gData.m_nTimeStamp);
  switch (sdk_status) {
    case ERR_BYD_AUTO_OK:
      msg.steering_angle_velocity = gData.m_nRotationSpeed * 4 / 180.0 * M_PI; // radian per sec
      std::cout << "the rotation speed info of the Steering wheel is: " <<  gData.m_nRotationSpeed * 4 / 180.0 * M_PI << std::endl;
      break;
    case ERR_BYD_AUTO_INVALID_PARAM:
      LOG_ERROR << "GetRotationSpeedInfo have the wrong param";
      break;
    case ERR_BYD_AUTO_READ:
    default:
      LOG_ERROR << "CAN read wrong";
      break;
  }
  // get Acceleration of XInfo
  sdk_status = GetAccelerationXInfo(&gData.m_nAX, &gData.m_nAXOffset, &gData.m_nTimeStamp);
  switch (sdk_status) {
    case ERR_BYD_AUTO_OK: {
      double ax = gData.m_nAX * 0.027126736 - 21.593;
      double ax_offset = gData.m_nAXOffset * 0.027126736 - 21.593;
      msg.acceleration_x = ax - ax_offset; // m/s2
      std::cout << "the acceleration of x is: " <<  ax - ax_offset << std::endl;
      break;
    }
    case ERR_BYD_AUTO_INVALID_PARAM:
      LOG_ERROR << "GetAccelerationXInfo have the wrong param";
      break;
    case ERR_BYD_AUTO_READ:
    default:
      LOG_ERROR << "CAN read wrong";
      break;
  }

  sdk_status = GetAccelerationYInfo(&gData.m_nAY, &gData.m_nAXOffset, &gData.m_nTimeStamp);
  switch (sdk_status) {
    case ERR_BYD_AUTO_OK: {
      double ay = gData.m_nAY * 0.027126736 - 21.593;
      double ay_offset = gData.m_nAYOffset * 0.027126736 - 21.593;
      msg.acceleration_x = ay - ay_offset; // m/s2
      std::cout << "the acceleration of y is: " <<  ay - ay_offset << std::endl;
      break;
    }
    case ERR_BYD_AUTO_INVALID_PARAM:
      LOG_ERROR << "GetAccelerationYInfo have the wrong param";
      break;
    case ERR_BYD_AUTO_READ:
    default:
      LOG_ERROR << "CAN read wrong";
      break;
  }

  sdk_status = GetYawRateInfo(&gData.m_nYawRate, &gData.m_nYawRateOffset, &gData.m_nTimeStamp);
  switch (sdk_status) {
    case ERR_BYD_AUTO_OK: {
      double yaw_rate_offset = gData.m_nYawRateOffset * 0.002132603 - 0.13;
      double yaw_rate = gData.m_nYawRate * 0.002132603 - 2.0943;
      msg.yaw_rate = yaw_rate - yaw_rate_offset;
      std::cout << "the YawRateInfo is: " << yaw_rate - yaw_rate_offset << std::endl;
      break;
    }
    case ERR_BYD_AUTO_INVALID_PARAM:
      LOG_ERROR << "GetYawRateInfo have the wrong param";
      break;
    case ERR_BYD_AUTO_READ:
    default:
      LOG_ERROR << "CAN read wrong";
      break;
  }

  vehicle_odometry_pub_.publish(msg);
  last_vegicle_odometry_msg_ = msg;

  return sdk_status;
}

int BydSDK::PublishControlInfo() {
  auto current_stamp = ros::Time::now();
  byd_sdk::ControlInfo msg;
  msg.header.stamp = current_stamp;
  int sdk_status = ERR_BYD_AUTO_OK;

  // Get deepness of the Accelerate deepness
  sdk_status = GetAccelerateDeepnessInfo(&gData.m_nAccelerateDeepness, &gData.m_nTimeStamp);
  switch(sdk_status)
  {
    case ERR_BYD_AUTO_OK:
      msg.accelerate_deepness = gData.m_nAccelerateDeepness * 1; //%
      std::cout << "the deepness of the Accelerate deepness is: " << gData.m_nAccelerateDeepness * 1 << std::endl;
      break;
    case ERR_BYD_AUTO_INVALID_PARAM:
      LOG_ERROR << "GetAccelerateDeepnessInfo have the wrong param";
      return sdk_status;
    case ERR_BYD_AUTO_READ:
      LOG_ERROR << "CAN read wrong";
      return sdk_status;
    default:
      return sdk_status;
  }
  // Get brake deepness info
  sdk_status = GetBrakeDeepnessInfo(&gData.m_nBrakeDeepness, &gData.m_nTimeStamp);
  switch(sdk_status)
  {
    case ERR_BYD_AUTO_OK:
      msg.brake_deepness = gData.m_nBrakeDeepness * 1; //%
      std::cout << "the deepness of the Accelerate deepness is: " << gData.m_nBrakeDeepness * 1 << std::endl;
      break;
    case ERR_BYD_AUTO_INVALID_PARAM:
      LOG_ERROR << "GetBrakeDeepnessInfo have the wrong param";
      return sdk_status;
    case ERR_BYD_AUTO_READ:
      LOG_ERROR << "CAN read wrong";
      return sdk_status;
    default:
      return sdk_status;
  }

  // Get gear info
  sdk_status = GetCarGearInfo(&gData.m_nCarGear, &gData.m_nTimeStamp);
  switch(sdk_status)
  {
    case ERR_BYD_AUTO_OK:
      msg.gear_info = gData.m_nCarGear ;
      std::cout << "the car gear is: " << gData.m_nCarGear << std::endl;
      break;
    case ERR_BYD_AUTO_INVALID_PARAM:
      LOG_ERROR << "GetCarGearInfo have the wrong param";
      return sdk_status;
    case ERR_BYD_AUTO_READ:
      LOG_ERROR << "CAN read wrong";
      return sdk_status;
    default:
      return sdk_status;
  }

  sdk_status = GetAngularInfo(&gData.m_nAngular, &gData.m_nTimeStamp);
  switch (sdk_status) {
    case ERR_BYD_AUTO_OK:
      if (gData.m_nAngular != 0xFFFF) {
        double angle = gData.m_nAngular * (-0.1) + 780; // Degree
        msg.angular_info = angle / 180.0 * M_PI; // Radian
      } else{
        return sdk_status;
      }
      break;
    case ERR_BYD_AUTO_INVALID_PARAM:
      LOG_ERROR << "GetAngularInfo have the wrong param";
      break;
    case ERR_BYD_AUTO_READ:
      LOG_ERROR << "CAN read wrong";
      return sdk_status;
    default:
      return sdk_status;
  }

  sdk_status = GetBrakePedalSignalInfo(&gData.m_nBrakePedalSignal, &gData.m_nTimeStamp);
  switch (sdk_status) {
    case ERR_BYD_AUTO_OK:
      msg.brake_pedal_signal = gData.m_nBrakePedalSignal;
      std ::cout << "the BrakePedalSignalInfo is: " << gData.m_nBrakePedalSignal << std::endl;
      break;
    case ERR_BYD_AUTO_INVALID_PARAM:
      LOG_ERROR << "GetBrakePedalSignalInfo have the wrong param";
      break;
    case ERR_BYD_AUTO_READ:
      LOG_ERROR << "CAN read wrong";
      return sdk_status;
    default:
      return sdk_status;
  }

  control_info_pub_.publish(msg);
  last_control_info_msg_ = msg;

  return sdk_status;
}


int BydSDK::PublishVehicleStatus() {

  int sdk_status = ERR_BYD_AUTO_OK;

  auto current_stamp = ros::Time::now();
  byd_sdk::VehicleStatus msg;
  msg.header.stamp = current_stamp;

  // get wheel drive direction
  byd_sdk_status_ = GetWheelDriveDirectionInfo(&gData.m_nWheelDriveDirectionRR, &gData.m_nWheelDriveDirectionRL, &gData.m_nWheelDriveDirectionFR, &gData.m_nWheelDriveDirectionFL, &gData.m_nTimeStamp);
  switch(byd_sdk_status_)
  {
    case ERR_BYD_AUTO_OK:
      msg.WheelDirection[0] = gData.m_nWheelDriveDirectionFR;
      std::cout << "the m_nWheelDriveDirectionFR is: " << gData.m_nWheelDriveDirectionFR << std::endl;
      msg.WheelDirection[1] = gData.m_nWheelDriveDirectionFL;
      std::cout << "the m_nWheelDriveDirectionFL is: " << gData.m_nWheelDriveDirectionFL<< std::endl;
      msg.WheelDirection[2] = gData.m_nWheelDriveDirectionRL;
      std::cout << "the m_nWheelDriveDirectionRL is: " << gData.m_nWheelDriveDirectionRL<< std::endl;
      msg.WheelDirection[3] = gData.m_nWheelDriveDirectionRR;
      std::cout << "the m_nWheelDriveDirectionRR is: " << gData.m_nWheelDriveDirectionRR<< std::endl;
      break;
    case ERR_BYD_AUTO_INVALID_PARAM:
      LOG_ERROR << "GetWheelDriveDirectionInfo have the wrong param";
      break;
    case ERR_BYD_AUTO_READ:
      LOG_ERROR << "CAN read wrong";
      return sdk_status;
    default:
      return sdk_status;
  }


  // get vehicle wheel speed
  sdk_status = GetWheelSpeedInfo(&gData.m_nWheelSpeedFL, &gData.m_nWheelSpeedFR, &gData.m_nWheelSpeedRL, &gData.m_nWheelSpeedRR, &gData.m_nTimeStamp);
  switch(sdk_status)
  {
    case ERR_BYD_AUTO_OK:
      msg.WheelSpeed[0] = gData.m_nWheelSpeedFR * 0.06875 ; //km/h
      std::cout << "the m_nWheelDriveDirectionRR is: " << gData.m_nWheelDriveDirectionRR<< std::endl;
      msg.WheelSpeed[1] = gData.m_nWheelSpeedFL * 0.06875 ; //km/h
      std::cout << "the m_nWheelSpeedFL is: " << gData.m_nWheelSpeedFL<< std::endl;
      msg.WheelSpeed[2] = gData.m_nWheelSpeedRL * 0.06875 ; //km/h
      std::cout << "the m_nWheelSpeedRL is: " << gData.m_nWheelSpeedRL<< std::endl;
      msg.WheelSpeed[3] = gData.m_nWheelSpeedRR * 0.06875 ; //km/h
      std::cout << "the m_nWheelSpeedRR is: " << gData.m_nWheelSpeedRR<< std::endl;
      break;
    case ERR_BYD_AUTO_INVALID_PARAM:
      LOG_ERROR << "GetWheelSpeedInfo have the wrong param";
      break;
    case ERR_BYD_AUTO_READ:
      LOG_ERROR << "CAN read wrong";
      return sdk_status;
    default:
      return sdk_status;
  }

  // sdk_status = GetLampInfo(&gData.m_nLampSmall, &gData.m_nLampNear, &gData.m_nLampFar, &gData.m_nTimeStamp);
  // switch(sdk_status) {
  //   case ERR_BYD_AUTO_OK:
  //     msg.LampInfo[0] = gData.m_nLampSmall;
  //     msg.LampInfo[1] = gData.m_nLampNear;
  //     msg.LampInfo[2] = gData.m_nLampFar;
  //   case ERR_BYD_AUTO_INVALID_PARAM:
  //     LOG_ERROR << "GetLampInfo have the wrong param";
  //     break;
  //   case ERR_BYD_AUTO_READ:
  //     LOG_ERROR << "CAN read wrong";
  //     return sdk_status;
  //   default:return sdk_status;
  // }

  // sdk_status = GetLampTurnInfo(&gData.m_nLampLeft, &gData.m_nLampRight, &gData.m_nTimeStamp);
  // switch (sdk_status) {
  //   case ERR_BYD_AUTO_OK:
  //     msg.LampTurnInfo[0] = gData.m_nLampLeft;
  //     msg.LampTurnInfo[1] = gData.m_nLampRight;
  //   case ERR_BYD_AUTO_INVALID_PARAM:
  //     LOG_ERROR << "GetLampTurnInfo have the wrong param";
  //     break;
  //   case ERR_BYD_AUTO_READ:
  //     LOG_ERROR << "CAN read wrong";
  //     return sdk_status;
  //   default:
  //     return sdk_status;
  // }

  vehicle_status_pub_.publish(msg);
  last_vehicle_status_msg_ = msg;
  return sdk_status;
}

void BydSDK::CanDataInit()
{
  gDataBuf.m_nModeRequest = 0;
  gDataBuf.m_nAutoLampSwitch = 0;
  gDataBuf.m_nLeftLampSwitch = 0;
  gDataBuf.m_nRightLampSwitch = 0;
  gDataBuf.m_nBrakeLampSwitch = 0;
  gDataBuf.m_nAngular = 0x1E78;
  gDataBuf.m_nEpbState = 0;
  gDataBuf.m_nRainWiper = 1;
  gDataBuf.m_nCarGear = 3;
  gDataBuf.m_nDriveModeFeedback = 1;
  gDataBuf.m_nAngularVelocity = 0;
  gDataBuf.m_nAcceleratedVelocity = 0x64;
  gDataBuf.m_nDriveAutoState = 0;
  gnCount = 0;
  memset(&gData, 0, sizeof(Data));
}
