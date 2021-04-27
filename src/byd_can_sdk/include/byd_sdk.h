#ifndef  _CANBUSROS_H
#define  _CANBUSROS_H

#include <ros/ros.h>

#include <byd_sdk/VehicleOdometry.h>
#include <byd_sdk/ControlInfo.h>
#include <byd_sdk/VehicleStatus.h>

#include "log.h"
#include "AutoDataProgramInterface.h"

#include <thread>

#define TEST_ERR_LOG                printf
#define TEST_DEBUG_LOG              printf
#define ERR_TEST_OK                 0
#define ERR_TEST_THREAD_CREATE      -1
#define ERR_TEST_TIMER_CREATE       -2
#define ERR_TEST_TIMER_EXIST        -3
#define ERR_TEST_THREAD_JOIN        -4
#define TIMER_ID_1                  1
#define CLASSINITSUCESS             1
#define CLASSINITFAIL               0




class BydSDK
{
 public:
  typedef struct
  {
    uint16_t	m_nVelocity;//車速
    uint16_t	m_nWheelSpeedFL;//前左輪的輪速
    uint16_t	m_nWheelSpeedFR;//前右輪的輪速
    uint16_t	m_nWheelSpeedRL;//後左輪的輪速
    uint16_t	m_nWheelSpeedRR;//後右輪的輪速
    uint8_t     m_nLampSmall;//小燈
    uint8_t     m_nLampNear;//近光燈
    uint8_t     m_nLampFar;//遠光燈
    uint8_t     m_nLampLeft;//左轉向
    uint8_t     m_nLampRight;//右轉向
    uint8_t     m_nLampFront;//前燈
    uint8_t     m_nLampRear;//後燈
    uint8_t     m_nRainWiper;//雨刮器
    uint8_t     m_nSwitchDriveModeFeedback;//組合開關智能駕駛模式
    uint8_t     m_nLampAuto;//自動燈光
    uint8_t     m_nDoorFL;//左前門
    uint8_t     m_nDoorFR;//右前門
    uint8_t     m_nDoorRL;//左後門
    uint8_t     m_nDoorRR;//右後門
    uint8_t     m_nDoorTrunk;//後備箱
    uint8_t     m_nWheelDriveDirectionRR;//右後輪胎方向
    uint8_t     m_nWheelDriveDirectionRL;//左後輪胎方向
    uint8_t     m_nWheelDriveDirectionFR;//右前輪胎方向
    uint8_t     m_nWheelDriveDirectionFL;//左前輪胎方向
    uint8_t     m_nESPDriveModeFeedback;//ESP當前駕駛模式
    uint8_t     m_nActive;//EPS激活
    uint8_t     m_nAlive;//EPS狀態
    uint16_t    m_nYawRate;//？
    uint16_t    m_nYawRateOffset;// ？
    uint16_t    m_nAX;//縱向
    uint16_t    m_nAXOffset;// 縱向偏移
    uint16_t    m_nAY;// 橫向
    uint16_t    m_nAYOffset;// 橫向偏移
    uint16_t    m_nWheelPulseFL;// 左前輪輪速脈衝
    uint16_t    m_nWheelPulseFR;// 右前輪輪速脈衝
    uint16_t    m_nWheelPulseRL;// 左後輪輪速脈衝
    uint16_t    m_nWheelPulseRR;// 右後輪輪速脈衝
    uint8_t     m_nAccelerateDeepness;// 油門深度
    uint8_t	    m_nBrakeDeepness; // 制動踏板深度
    uint8_t     m_nCarGear; // 檔位
    uint8_t	    m_nEleBrake;//電子手剎
    uint16_t    m_nAngular;//方向盤角度
    uint8_t		m_nEPSDriveModeFeedback;//EPS駕駛模式反饋
    uint8_t		m_nDriveAutoPermission;//駕駛模式權限
    uint8_t		m_nRotationSpeed;//方向盤旋轉速度
    uint8_t		m_nMediumKey;//多媒體按鍵信息
    uint16_t	m_nMediumCoordinateX;//多媒體座標
    uint16_t	m_nMediumCoordinateY;
    uint16_t	m_nMediumResolutionW;
    uint16_t	m_nMediumResolutionH;
    uint8_t		m_nMediumBehavior;
    uint16_t	m_nMediumMovementY;
    uint16_t	m_nMediumMovementX;
    uint8_t		m_nPASDistanceFL;//障礙物
    uint8_t		m_nPASDistanceFR;
    uint8_t		m_nPASDistanceRL;
    uint8_t		m_nPASDistanceRR;
    uint8_t		m_nPASDistanceL;
    uint8_t		m_nPASDistanceR;
    uint8_t		m_nReverseHint;
    uint8_t		m_nPower;
    uint16_t	m_nMotorTorque;//電機扭矩
    uint8_t		m_nMCDriveModeFeedback;//電機控制駕駛模式
    uint16_t	m_nPressure;//主缸壓力信息
    uint8_t		m_nPressureState;
    uint16_t	m_nPressureOffset;
    uint8_t		m_nPressureOffsetState;
    uint8_t		m_nTractionControl;
    uint8_t		m_nBodyDynamic;
    uint8_t		m_nBrakePedalSignal;
    uint8_t		m_nGearDriveModeFeedback;
    uint64_t	m_nTimeStamp;
  }Data;
 public:
  BydSDK() = default;

  int Init(const ros::NodeHandle& nh);

 private:

  void CanDataInit();

  void ReceiveThread();

  int PublishVehicleOdometry();

  int PublishControlInfo();

  int PublishVehicleStatus();

 private:
  ros::NodeHandle nh_;
  ros::Publisher vehicle_odometry_pub_;
  ros::Publisher control_info_pub_;
  ros::Publisher vehicle_status_pub_;
  std::unique_ptr<std::thread> receive_thread_ptr_;

  byd_sdk::VehicleOdometry last_vegicle_odometry_msg_;
  byd_sdk::ControlInfo last_control_info_msg_;
  byd_sdk::VehicleStatus last_vehicle_status_msg_;


 public:
  int gnCount = 0;
  Data gData;
  DataBuf gDataBuf;
  int byd_sdk_status_ = ERR_TEST_OK;


};

#endif