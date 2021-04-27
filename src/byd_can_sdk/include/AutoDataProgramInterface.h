#ifndef  _AUTODATAPROGRAMINTERFACE_H
#define  _AUTODATAPROGRAMINTERFACE_H

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {               // 告诉编译器下列代码要以C链接约定的模式进行链接
#endif


//·µ»ØÖµ±êÊ¶Î»
#define ERR_BYD_AUTO_OK                                0           //各種API初始化狀態成功   
#define ERR_BYD_AUTO_CAN_ENABLE                        -1          //API使能初始化時CAN使能錯誤 
#define ERR_BYD_AUTO_MEM_MALLOC                        -2          //內存申請錯誤  
#define ERR_BYD_AUTO_INVALID_PARAM                     -3          //輸入參數錯誤 
#define ERR_BYD_AUTO_DISENABLE                         -4          //API取消使能裝填失敗  
#define ERR_BYD_AUTO_WRITE                             -5          //向CAN網絡寫失敗
#define ERR_BYD_AUTO_READ                              -6          //
#define ERR_BYD_AUTO_REPEATE_CALL                      -7          //API初始化時多次重複調用（不能重複調用，取消是能後才能再次調用）

//³µÃÅ×ŽÌ¬	
#define BYD_AUTO_DOOR_OFF                              0          //¹Ø±Õ
#define BYD_AUTO_DOOR_ON                               1          //Žò¿ª

//ÐÐÀîÏä×ŽÌ¬
#define BYD_AUTO_TRUNK_OFF                             0          //¹Ø±Õ
#define BYD_AUTO_TRUNK_ON                              1          //Žò¿ª

//¶àÃœÌåŽ¥ÃþÆÁ¶¯×÷	
#define BYD_AUTO_MEDIUM_BEHAVIOR_INVALID               0          //ÎÞÐ§
#define BYD_AUTO_MEDIUM_BEHAVIOR_DOWN                  1          //°ŽÏÂ
#define BYD_AUTO_MEDIUM_BEHAVIOR_UP                    2          //Ì§Æð
#define BYD_AUTO_MEDIUM_BEHAVIOR_MOVE                  3          //ÒÆ¶¯

//×ÝÏòÐÐÎª	
#define BYD_AUTO_MEDIUM_MOVEENT_Y_INVALID              0          //ÎÞÐ§
#define BYD_AUTO_MEDIUM_MOVEENT_Y_UP                   1          //ÉÏ
#define BYD_AUTO_MEDIUM_MOVEENT_Y_DOWN                 2          //ÏÂ

//ºáÏòÐÐÎª	
#define BYD_AUTO_MEDIUM_MOVEENT_X_INVALID              0          //ÎÞÐ§
#define BYD_AUTO_MEDIUM_MOVEENT_X_LEFT                 1          //×ó
#define BYD_AUTO_MEDIUM_MOVEENT_X_RIGHT                2          //ÓÒ

//µ¹³µµµÎ»ÌáÊŸ
#define BYD_AUTO_REVERSE_HINT_INVALID                  0	      //ÎÞÐ§
#define BYD_AUTO_REVERSE_HINT                          1	      //µ¹³µµµÎ»ÌáÊŸ

//µçÔŽ¿ª¹Ø×ŽÌ¬
#define BYD_AUTO_PAS_POWER_FAULT                       0	      //ŽíÎó
#define BYD_AUTO_PAS_POWER_OPEN                        1	      //µçÔŽ¿ª¹ØŽò¿ª

//Ä£¿é³õÊŒ»¯
/*---------------------------------------------------------------------------------------------*/
int BydAutoApiEnable();//智能駕駛API使能
int BydAutoApiDisable();//智能駕駛API取消使能
/*---------------------------------------------------------------------------------------------*/
typedef struct
{
	uint8_t	m_nModeRequest;//駕駛模式 BYD_AUTO_DRIVE_MODE_HOLDING 0 保持當前駕駛模式   BYD_AUTO_DRIVE_MODE_AUTO_OPEN 1 開啓只能駕駛模式 BYD_AUTO_DRIVE_MODE_AUTO_CLOSE 2 關閉只能駕駛模式
	uint8_t	m_nAutoLampSwitch;//燈光AUTO檔開關 BYD_AUTO_LAMP_AUTO_ON  1  燈光AUTO檔 BYD_AUTO_LAMP_AUTO_OFF 2 燈光非AUTO檔
	uint8_t	m_nLeftLampSwitch;//左轉向燈開關  BYD_AUTO_LAMP_OFF 0 轉向燈關閉 BYD_AUTO_LAMP_ON  1 轉向燈打開
	uint8_t	m_nRightLampSwitch;//右轉向燈開關 BYD_AUTO_LAMP_OFF 0 轉向燈關閉 BYD_AUTO_LAMP_ON  1 轉向燈打開
	uint8_t	m_nBrakeLampSwitch;//制動燈開關 BYD_AUTO_LAMP_OFF 0 轉向燈關閉 BYD_AUTO_LAMP_ON  1 轉向燈打開
	uint16_t	m_nAngular;//目標方向盤角度
	uint8_t	m_nEpbState;//目標EPB狀態 BYD_AUTO_EPB_SET_INVALID 0 無效 BYD_AUTO_EPB_SET_APPLYING 1 拉起 BYD_AUTO_EPB_SET_RELEASED 2 釋放
	uint8_t	m_nRainWiper;//目標雨刮檔位 具體
	uint8_t	m_nCarGear;//目標檔位 BYD_AUTO_CAR_GEAR_P 1 P BYD_AUTO_CAR_GEAR_R 2 R BYD_AUTO_CAR_GEAR_N  3 N BYD_AUTO_CAR_GEAR_D 4 D
	uint8_t	m_nDriveModeFeedback;//駕駛模式反饋 BYD_AUTO_DRIVE_MODE_AUTO_ABNORMAL_1 0 智能駕駛異常1 BYD_AUTO_DRIVE_MODE_AUTO 1 智能駕駛模式 BYD_AUTO_DRIVE_MODE_NORMAL 2 正常駕駛模式 BYD_AUTO_DRIVE_MODE_AUTO_ABNORMAL_2 3	智能駕駛模式2
	uint16_t	m_nAngularVelocity;//目標方向盤角速度
	uint8_t	m_nAcceleratedVelocity;//目標加速度
	uint8_t	m_nDriveAutoState;//智能駕駛縱向控制 BYD_AUTO_DRIVE_MODE_AUTO_STATE_CLOSE 0 縱向控制關閉狀態 BYD_AUTO_DRIVE_MODE_AUTO_STATE_OPEN 3 總想控制開啓狀態
}DataBuf; //輸入值結構體
int BydAutoApiDataInit(DataBuf* pDataBuf);//智能駕駛API數據初始化接口定義 返回值 int_32 初始化狀態 以上部分宏定義的值
/*---------------------------------------------------------------------------------------------*/
int SendAliveCount(uint8_t nCount);//智能API滾動計數以及控制發送時序（週期20ms,超時200ms） 輸入[0,15]  返回 ERR_BYD_AUTO_OK或者ERR_BYD_AUTO_INVALID_PARAM或者ERR_BYD_AUTO_WRITE
/*---------------------------------------------------------------------------------------------*/
//駕駛模式
#define BYD_AUTO_DRIVE_MODE_HOLDING                    0          //保持當前駕駛模式
#define BYD_AUTO_DRIVE_MODE_AUTO_OPEN                  1          //開啓只能駕駛模式
#define BYD_AUTO_DRIVE_MODE_AUTO_CLOSE                 2          //關閉只能駕駛模式
int SendAutoDriveModeRequest(uint8_t nModeRequest);//向各功能模塊發送駕駛模式請求 返回值 int_32 初始化狀態 以上部分宏定義的值
/*---------------------------------------------------------------------------------------------*/
//AUTO燈光操作
#define BYD_AUTO_LAMP_AUTO_INVALID                     0          //
#define BYD_AUTO_LAMP_AUTO_ON                          1          //燈光AUTO檔
#define BYD_AUTO_LAMP_AUTO_OFF                         2          //燈光非AUTO檔
int SetLampAutoSwitch(uint8_t nAutoSwitch);//燈光AUTO檔開關設置 輸入爲上三行中的任意一個 返回值爲ERR_BYD_AUTO_OK或者ERR_BYD_AUTO_INVALID_PARAM
/*---------------------------------------------------------------------------------------------*/
//燈光操作	
#define BYD_AUTO_LAMP_OFF                              0          //燈光關閉
#define BYD_AUTO_LAMP_ON                               1          //燈光打開
int SetLampTurnSwitch(uint8_t nleft, uint8_t nRight);//轉向燈設置 輸入爲兩個方向等的關或閉 即上三行中的任意一個 返回值爲ERR_BYD_AUTO_OK或者ERR_BYD_AUTO_INVALID_PARAM
/*---------------------------------------------------------------------------------------------*/
int SetLampBrakeSwitch(uint8_t nBreakSwitch);//制動燈設置 輸入也是燈管相關的兩個宏定義 返回值爲ERR_BYD_AUTO_OK或者ERR_BYD_AUTO_INVALID_PARAM
/*---------------------------------------------------------------------------------------------*/
//方向盤
int SetAngular(uint16_t nAngular);//方向盤角度設置（左轉爲正數，右轉爲負數，限制角度正負550實際範圍[8FC,33F4]） 返回值 ERR_BYD_AUTO_OK或者ERR_BYD_AUTO_INVALID_PARAM
/*---------------------------------------------------------------------------------------------*/
//EPB狀態
#define BYD_AUTO_EPB_SET_INVALID                       0          //無效
#define BYD_AUTO_EPB_SET_APPLYING                      1          //拉起
#define BYD_AUTO_EPB_SET_RELEASED                      2          //釋放
int SetEpbState(uint8_t nEpbState);//輸入爲上述三個狀態之一 返回值  ERR_BYD_AUTO_OK或者ERR_BYD_AUTO_INVALID_PARAM
/*---------------------------------------------------------------------------------------------*/
//雨刮
#define BYD_AUTO_RAIN_WIPER_OFF                        1          //OFF檔
#define BYD_AUTO_RAIN_WIPER_SHORT_PRESS                2          //點刮短按
#define BYD_AUTO_RAIN_WIPER_LONG_PRESS                 3          //點刮長按
#define BYD_AUTO_RAIN_WIPER_INTERVAL_1                 4          //間隙1檔
#define BYD_AUTO_RAIN_WIPER_INTERVAL_2                 5          //間隙2檔
#define BYD_AUTO_RAIN_WIPER_INTERVAL_3                 6          //間隙3檔
#define BYD_AUTO_RAIN_WIPER_INTERVAL_4                 7          //間隙4檔
#define BYD_AUTO_RAIN_WIPER_SLOW                       8          //慢刮
#define BYD_AUTO_RAIN_WIPER_QUICK                      9          //快刮
#define BYD_AUTO_RAIN_WIPER_FAULT                      15         //ÎÞÐ§
int SetRainWiper(uint8_t nRainWiper);//目標雨刮檔位設置 輸入爲上述宏定義中的任意一個 返回值 ERR_BYD_AUTO_OK或者ERR_BYD_AUTO_INVALID_PARAM
/*---------------------------------------------------------------------------------------------*/
//檔位操作 
#define BYD_AUTO_CAR_GEAR_INVALID                      0          //
#define BYD_AUTO_CAR_GEAR_P                            1          //P
#define BYD_AUTO_CAR_GEAR_R                            2          //R
#define BYD_AUTO_CAR_GEAR_N                            3          //N
#define BYD_AUTO_CAR_GEAR_D                            4          //D
#define BYD_AUTO_CAR_GEAR_FAULT                        7          //ŽíÎó  
int SetCarGear(uint8_t nCarGear);//目標檔位設置 輸入爲上述宏定義中的任意一個 返回值 ERR_BYD_AUTO_OK或者ERR_BYD_AUTO_INVALID_PARAM
/*---------------------------------------------------------------------------------------------*/
//駕駛模式
#define BYD_AUTO_DRIVE_MODE_AUTO_ABNORMAL_1            0	      //智能駕駛異常1
#define BYD_AUTO_DRIVE_MODE_AUTO                       1          //智能駕駛模式
#define BYD_AUTO_DRIVE_MODE_NORMAL                     2          //正常駕駛模式
#define BYD_AUTO_DRIVE_MODE_AUTO_ABNORMAL_2            3	      //智能駕駛模式2
int SetDriveModeFeedback(uint8_t nDriveModeFeedback);//駕駛模式反饋設置 輸入爲上述宏定義中的任意一個 返回值 ERR_BYD_AUTO_OK或者ERR_BYD_AUTO_INVALID_PARAM
/*---------------------------------------------------------------------------------------------*/
//
int SetAngularVelocity(uint16_t nAngularVelocity);//目標方向盤角速度設置 [0x0,0x320] 返回值 ERR_BYD_AUTO_OK或者ERR_BYD_AUTO_INVALID_PARAM
/*---------------------------------------------------------------------------------------------*/
//
int SetAcceleratedVelocity(uint8_t nAcceleratedVelocity);//目標加速度設置 [0x00,0xC8] 返回值 ERR_BYD_AUTO_OK或者ERR_BYD_AUTO_INVALID_PARAM
/*---------------------------------------------------------------------------------------------*/
//縱向狀態
#define BYD_AUTO_DRIVE_MODE_AUTO_STATE_CLOSE           0          //縱向控制關閉狀態
#define BYD_AUTO_DRIVE_MODE_AUTO_STATE_OPEN            3          //總想控制開啓狀態
int SetDriveAutoState(uint8_t nDriveAutoState);//智能駕駛縱向控制狀態設置 輸入爲上述宏定義中的任意一個 返回值 ERR_BYD_AUTO_OK或者ERR_BYD_AUTO_INVALID_PARAM
/*---------------------------------------------------------------------------------------------*/

//智能駕駛模塊接受信息
/*---------------------------------------------------------------------------------------------*/
//獲取車速信息（誤差小於2% 20ms一個週期） pnVelocity 車速地址 pnTimeStamp 定義的最後一次接收數據的時間變量地址 輸出車速[0x000,0xFFE] 時間計數[0X00,0XFFFFFFF] (超過週期間隔的2次接收時間相同代表對應模塊掉線)
int GetVelocityInfo(uint16_t* pnVelocity, uint64_t* pnTimeStamp);//	
/*---------------------------------------------------------------------------------------------*/
//獲取車輪實時輪速（誤差1% ） pnFL 定義的左前輪實時輪速變量地址 pnFR 定義的右前輪實時輪速變量地址 pnRL 定義的左後輪實時輪速變量地址 pnRR 定義的右後輪實時輪速變量地址
int GetWheelSpeedInfo(uint16_t* pnFL, uint16_t* pnFR, uint16_t* pnRL, uint16_t* pnRR, uint64_t* pnTimeStamp);//
/*---------------------------------------------------------------------------------------------*/
//獲取燈光信息	
//pnSmall 小燈開關狀態地址 pnNear 近光燈開關狀態變量地址 pnFar 遠光燈開關狀態變量地址 
int GetLampInfo(uint8_t* pnSmall, uint8_t* pnNear, uint8_t* pnFar, uint64_t* pnTimeStamp);//
/*---------------------------------------------------------------------------------------------*/
int GetLampTurnInfo(uint8_t* pnLeft, uint8_t* pnRigh, uint64_t* pnTimeStampt);//獲取轉向燈信息 pnLeft 左轉向燈開關狀態變量 pnRigh 右轉向燈開關狀態變量
/*---------------------------------------------------------------------------------------------*/
int GetLampFogInfo(uint8_t* pnFront, uint8_t* pnRear, uint64_t* pnTimeStamp);//獲取霧燈信息 pnFront 前霧燈開關狀態百年來那個的地址 pnRear 後霧燈開關狀態變量的地址
/*---------------------------------------------------------------------------------------------*/
//
//#define BYD_AUTO_RAIN_WIPER_OFF                        1          //OFF
//#define BYD_AUTO_RAIN_WIPER_SHORT_PRESS                2          //
//#define BYD_AUTO_RAIN_WIPER_LONG_PRESS                 3          //
//#define BYD_AUTO_RAIN_WIPER_INTERVAL_1                 4          //
//#define BYD_AUTO_RAIN_WIPER_INTERVAL_2                 5          //
//#define BYD_AUTO_RAIN_WIPER_INTERVAL_3                 6          //
//#define BYD_AUTO_RAIN_WIPER_INTERVAL_4                 7          //
//#define BYD_AUTO_RAIN_WIPER_SLOW                       8          //
//#define BYD_AUTO_RAIN_WIPER_QUICK                      9          //
//#define BYD_AUTO_RAIN_WIPER_FAULT                      15         //
int GetRainWiperInfo(uint8_t* pnRainWiper, uint64_t* pnTimeStamp);// 獲取雨刮檔位的信息藉口（100ms週期）
/*---------------------------------------------------------------------------------------------*/
//	
#define BYD_AUTO_DRIVE_MODE_SWITCH_FEEDBACK_INVALID    0          //組合開關智能駕駛模式反饋 無效
#define BYD_AUTO_DRIVE_MODE_SWITCH_FEEDBACK_NORMAL     1          //組合開關智能駕駛模式反饋 正常駕駛模式
#define BYD_AUTO_DRIVE_MODE_SWITCH_FEEDBACK_AUTO       2          //組合開關智能駕駛模式反饋 智能駕駛模式
int GetSwitchDriveModeFeedbackInfo(uint8_t* pnSwitchDriveModeFeedback, uint64_t* pnTimeStamp);//獲取組合開關智能駕駛模式反饋信息（100ms週期） 輸出以上三個 返回
/*---------------------------------------------------------------------------------------------*/
//
//#define BYD_AUTO_LAMP_AUTO_INVALID                     0          //
//#define BYD_AUTO_LAMP_AUTO_ON                          1          //
//#define BYD_AUTO_LAMP_AUTO_OFF                         2          //
int GetLampAutoInfo(uint8_t* pnLampAuto, uint64_t* pnTimeStamp);//	獲取燈光AUTO檔開關信息
/*---------------------------------------------------------------------------------------------*/
//³µÂÖÐÐÊ»·œÏò	
#define BYD_AUTO_DRIVE_DIRECTION_UNDEFINE              0          //車輪行駛方向 未定義
#define BYD_AUTO_DRIVE_DIRECTION_FORWARD               1          //車輪行駛方向 向前
#define BYD_AUTO_DRIVE_DIRECTION_BACKWARD              2          //車輪行駛方向 向後
#define BYD_AUTO_DRIVE_DIRECTION_STOP                  3          //車輪行駛方向 停止
int GetWheelDriveDirectionInfo(uint8_t* pnRR, uint8_t* pnRL, uint8_t* pnFR, uint8_t* pnFL, uint64_t* pnTimeStamp);//獲取車輪行駛方向
/*---------------------------------------------------------------------------------------------*/
//獲取ESP信息 	
#define BYD_AUTO_DRIVE_MODE_ESP_FEEDBACK_CLOSE         0          //縱向控制關閉
#define BYD_AUTO_DRIVE_MODE_ESP_FEEDBACK_OPEN          1          //縱向控制開啓
int GetESPDriveModeFeedbackInfo(uint8_t* pnESPDriveModeFeedback, uint64_t* pnTimeStamp);//獲取ESP當前駕駛模式信息
/*---------------------------------------------------------------------------------------------*/
//
#define BYD_AUTO_DRIVE_MODE_ESP_NO_ACTIVE              0	      //ESP激活狀態  0 縱向控制未激活
#define BYD_AUTO_DRIVE_MODE_ESP_ACTIVE                 1	      //ESP激活狀態  1 縱向控制已激活
int GetESPActiveInfo(uint8_t* pnActive, uint64_t* pnTimeStamp);//獲取ESP控制激活狀態信息（是否正在執行加速/制動）
/*---------------------------------------------------------------------------------------------*/
//
int GetESPAliveInfo(uint8_t* pnAlive, uint64_t* pnTimeStamp);//獲取ESP滾動計數信息
/*---------------------------------------------------------------------------------------------*/
//
int GetYawRateInfo(uint16_t* pnYawRate, uint16_t* pnYawRateOffset, uint64_t* pnTimeStamp);//獲取橫擺角度信息
/*---------------------------------------------------------------------------------------------*/
//
int GetAccelerationXInfo(uint16_t* pnAX, uint16_t* pnAXOffset, uint64_t* pnTimeStamp);//獲取車輛縱向加速度（若要獲得實際的車輛縱向加速度值需進行計算：AX-AXoffset）
/*---------------------------------------------------------------------------------------------*/
//
int GetAccelerationYInfo(uint16_t* pnAY, uint16_t* pnAYOffset, uint64_t* pnTimeStamp);//獲取車輛橫向加速度（若要獲得實際的車輛橫向加速度值需進行計算：AY-AYoffset）
/*---------------------------------------------------------------------------------------------*/
//
int GetWheelPulseCounterInfo(uint16_t* pnFL, uint16_t* pnFR, uint16_t* pnRL, uint16_t* pnRR, uint64_t* pnTimeStamp);//獲取輪速脈衝信息
/*---------------------------------------------------------------------------------------------*/
//
int GetAccelerateDeepnessInfo(uint8_t* pnAccelerateDeepness, uint64_t* pnTimeStamp);//獲取油門深度信息
/*---------------------------------------------------------------------------------------------*/
//
int GetBrakeDeepnessInfo(uint8_t* pnBrakeDeepness, uint64_t* pnTimeStamp);//獲取制動深度信息
/*---------------------------------------------------------------------------------------------*/
//#define BYD_AUTO_CAR_GEAR_INVALID                      0          //
//#define BYD_AUTO_CAR_GEAR_P                            1          //P
//#define BYD_AUTO_CAR_GEAR_R                            2          //R
//#define BYD_AUTO_CAR_GEAR_N                            3          //N
//#define BYD_AUTO_CAR_GEAR_D                            4          //D
//#define BYD_AUTO_CAR_GEAR_FAULT                        7          //
int GetCarGearInfo(uint8_t* pnCarGear, uint64_t* pnTimeStamp);//獲取檔位信息
/*---------------------------------------------------------------------------------------------*/
//
#define BYD_AUTO_EPB_GET_RELEASING                     0          //電子手剎狀態  正在釋放
#define BYD_AUTO_EPB_GET_RELEASED                      1          //電子手剎狀態  釋放
#define BYD_AUTO_EPB_GET_APPLYING                      2          //電子手剎狀態  正在拉起
#define BYD_AUTO_EPB_GET_APPLIED                       3          //電子手剎狀態  拉起
#define BYD_AUTO_EPB_GET_BRAKE_FAULT                   4          //錯誤
int GetEleBrakeInfo(uint8_t* pnEleBrake, uint64_t* pnTimeStamp);//獲取電子手剎狀態信息
/*---------------------------------------------------------------------------------------------*/
//獲取方向盤角度信息
int GetAngularInfo(uint16_t* pnAngular, uint64_t* pnTimeStamp);//
/*---------------------------------------------------------------------------------------------*/
//EPS	
#define BYD_AUTO_DRIVE_MODE_EPS_FEEDBACK_INVALID       0          //EPS當前駕駛模式 無效
#define BYD_AUTO_DRIVE_MODE_EPS_FEEDBACK_NORMAL        1          //EPS當前駕駛模式 正常駕駛模式
#define BYD_AUTO_DRIVE_MODE_EPS_FEEDBACK_AUTO          2          //EPS當前駕駛模式 智能駕駛模式
int GetEPSDriveModeFeedbackInfo(uint8_t* pnEPSDriveModeFeedback, uint64_t* pnTimeStamp);//獲取EPS當前駕駛模式信息
/*---------------------------------------------------------------------------------------------*/
//
#define BYD_AUTO_EPS_PERMISSION_INVALID                0          //EPS進入智能駕駛允許 無效
#define BYD_AUTO_EPS_PERMISSION_OK                     1          //EPS進入智能駕駛允許 允許進入智能駕駛
#define BYD_AUTO_EPS_NOT_PERMISSION                    2          //EPS進入智能駕駛允許 不允許進入智能駕駛
int GetDriveAutoPermissionInfo(uint8_t* pnDriveAutoPermission, uint64_t* pnTimeStamp);//獲取EPS進入智能駕駛允許信息（EPS故障，發不允許，ESP無故障，發允許進入）
/*---------------------------------------------------------------------------------------------*/
//
int GetRotationSpeedInfo(uint8_t* pnRotationSpeed, uint64_t* pnTimeStamp);// 獲取方向盤旋轉速度信息
/*---------------------------------------------------------------------------------------------*/
//多媒體按鍵	
#define BYD_AUTO_MEDIUM_KEY_INVALID                    0          //按鍵信息 無效
#define BYD_AUTO_MEDIUM_KEY_STOP_SWITCH                2          //按鍵信息 緊急停止開關
#define BYD_AUTO_MEDIUM_KEY_DRIVE_AUTO_SWITCH          5          //按鍵信息 智能駕駛主開關
int GetMediumKeyInfo(uint8_t* pnMediumKey, uint64_t* pnTimeStamp);//獲取多媒體相關按鍵信息
/*---------------------------------------------------------------------------------------------*/
//
int GetMotorTorqueInfo(uint16_t* pnMotorTorque, uint64_t* pnTimeStamp);//獲取電機扭矩信息
/*---------------------------------------------------------------------------------------------*/
//電機控制駕駛模式反饋 
#define BYD_AUTO_DRIVE_MODE_MC_FEEDBACK_NORMAL         0          //電機控制駕駛模式反饋 正常駕駛模式
#define BYD_AUTO_DRIVE_MODE_MC_FEEDBACK_AUTO           1          //電機控制駕駛模式反饋 智能駕駛模式
int GetMCDriveModeFeedbackInfo(uint8_t* pnMCDriveModeFeedback, uint64_t* pnTimeStamp);//獲取電機控制駕駛模式反饋
/*---------------------------------------------------------------------------------------------*/
//主缸壓力
#define BYD_AUTO_DRIVE_MODE_MC_P_VALID                 0          //MC壓力狀態 有效
#define BYD_AUTO_DRIVE_MODE_MC_P_INVALID               1          //MC壓力狀態 無效
//獲取主缸壓力值信息
int GetMCPressureInfo(uint16_t* pnPressure, uint8_t* pnPressureState, uint8_t* pnPressureOffsetState, uint16_t* pnPressureOffset,  uint64_t* pnTimeStamp);
/*---------------------------------------------------------------------------------------------*/
//ESP故障
#define BYD_AUTO_ESP_ERROR_NO_FAILURE                  0          //無故障
#define BYD_AUTO_ESP_ERROR_FAILURE                     1	      //故障
int GetESPErrInfo(uint8_t* pnTractionControl, uint8_t* pnBodyDynamic, uint64_t* pnTimeStamp);//獲取ESP故障信息
/*---------------------------------------------------------------------------------------------*/
//制動踏板信息
#define BYD_AUTO_BRAKE_PEDAL_SIGNAL_NO_PRESSED         0          //制動踏板信號 未踩下
#define BYD_AUTO_BRAKE_PEDAL_SIGNAL_PRESSED            1          //制動踏板信號 踩下
#define BYD_AUTO_BRAKE_PEDAL_SIGNAL_ERR                3          //信號錯誤
int GetBrakePedalSignalInfo(uint8_t* pnBrakePedalSignal, uint64_t* pnTimeStamp);//獲取制動踏板信號信息
/*---------------------------------------------------------------------------------------------*/
//檔位駕駛	
#define BYD_AUTO_DRIVE_MODE_GEAR_FEEDBACK_NORMAL       0          //檔位駕駛模式反饋 正常駕駛模式
#define BYD_AUTO_DRIVE_MODE_GEAR_FEEDBACK_AUTO         1          //檔位駕駛模式反饋 智能駕駛模式
int GetGearDriveModeFeedbackInfo(uint8_t* pnGearDriveModeFeedback, uint64_t* pnTimeStamp);//獲取檔位駕駛模式反饋信息
/*---------------------------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif


#endif
