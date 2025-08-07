/** @file
 *  @brief TTAlink comm protocol generated from common.xml
 *  @see http://ttalink.org
 */
#pragma once
#ifndef TTALINK_COMMON_H
#define TTALINK_COMMON_H

#ifndef TTALINK_H
    #error Wrong include order: TTALINK_COMMON.H MUST NOT BE DIRECTLY USED. Include ttalink.h from the same directory instead or set ALL AND EVERY defines from ttalink.h manually accordingly, including the #define TTALINK_H call.
#endif

#undef TTALINK_THIS_XML_IDX
#define TTALINK_THIS_XML_IDX 0

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef TTALINK_MESSAGE_LENGTHS
#define TTALINK_MESSAGE_LENGTHS {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 50, 0, 0, 0, 137, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#endif

#ifndef TTALINK_MESSAGE_CRCS
//#define TTALINK_MESSAGE_CRCS {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 168, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 155, 0, 0, 0, 37, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#endif

#include "../protocol.h"

#define TTALINK_ENABLED_COMMON

// ENUM DEFINITIONS


/** @brief ttalink address ids. */
#ifndef HAVE_ENUM_TTALINK_ADDRESS_ID
#define HAVE_ENUM_TTALINK_ADDRESS_ID
typedef enum TTALINK_ADDRESS_ID
{
   TTALINK_FC_ADDRESS=1, /* This is fc address. | */
   TTALINK_EXAM_ADDRESS=2, /* This is Examination system address. | */
   TTALINK_SH_ADDRESS=3, /* This is sh address. | */
   TTALINK_TC_ADDRESS=4, /* This is subprocessor  address //subprocessor . | */
   TTALINK_IMAGE_ADDRESS=5, /* This is image  address //Intelligent image transmission . | */
   TTALINK_RTK_MODULE_ADDRESS=6, /* rtk模块地址，用于升级. | */
   TTALINK_RC_ADDRESS=7, /* 智能遥控器. | */
   TTALINK_FENCE_ADDRESS=8, /* 禁飞区用于升级. | */
   TTALINK_FOC_ADDRESS=10, /* This foc address. | */
   TTALINK_FOC1_ADDRESS=11, /* This foc address. | */
   TTALINK_FOC2_ADDRESS=12, /* This foc address. | */
   TTALINK_FOC3_ADDRESS=13, /* This foc address. | */
   TTALINK_FOC4_ADDRESS=14, /* This foc address. | */
   TTALINK_FOC5_ADDRESS=15, /* This foc address. | */
   TTALINK_FOC6_ADDRESS=16, /* This foc address. | */
   TTALINK_FOC7_ADDRESS=17, /* This foc address. | */
   TTALINK_FOC8_ADDRESS=18, /* This foc address. | */
   TTALINK_CAN1_DEVICE_ADDRESS=20, /* This can1 address. | */
   TTALINK_GIMBAL_ADDRESS=50, /* This Common GIMBAL address. | */
   TTALINK_PTZ_ADDRESS=51, /* This PTZ address. | */
   TTALINK_PTZ_RESERVE_ADDRESS=52, /* This PTZ Reserve address. | */
   TTALINK_VISIBLE_LIGHT_CAMERA_ADDRESS=53, /* This visible_light Camera address. | */
   TTALINK_INFRARED_CAMERA_ADDRESS=54, /* This infrared camera address. | */
   TTALINK_DUAL_LIGHT_CAMERA_ADDRESS=55, /* This dual_light Camera address. | */
   TTALINK_TRI_LIGHT_CAMERA_ADDRESS=56, /* This tri_light camera address. | */
   TTALINK_FIVE_LIGHT_CAMERA_ADDRESS=57, /* This five_light camera address. | */
   TTALINK_MULTI_SPECTRAL_CAMERA_ADDRESS=58, /* This PTZ mulit_camera address. | */
   TTALINK_CAMERA_RESERVE_ADDRESS=59, /* This camera Reserve address. | */
   TTALINK_PTZ_TRACKER_ADDRESS=60, /* This ptz tracker address. | */
   TTALINK_HOOK_ADDRESS=61, /* 抛钩广播地址. | */
   TTALINK_HOOK1_ADDRESS=62, /* 抛钩1地址. | */
   TTALINK_HOOK2_ADDRESS=63, /* 抛钩2地址. | */
   TTALINK_MULTI_HOOK_ADDRESS=64, /* 多重抛钩地址. | */
   TTALINK_PARACHUTE_ADDRESS=65, /* 降落伞地址. | */
   TTALINK_SEARCH_LIGHT_ADDRESS=66, /* 探照灯地址. | */
   TTALINK_BASE_RTK_ADDRESS=100, /* This bms address. | */
   TTALINK_LB_RADAR_ADDRESS=140, /* This Radar address. | */
   TTALINK_SPREADER_ADDRESS=141, /* 播撒机地址. | */
   TTALINK_MIST_SPRAYER_ADDRESS=142, /* 播撒机地址. | */
   TTALINK_LOUDSPEAKER_ADDRESS=143, /* loudspeaker | */
   TTALINK_VISION_ADDRESS=145, /* 对地视觉. | */
   TTALINK_BMS_ADDRESS=150, /* This bms address. | */
   TTALINK_BMS1_ADDRESS=151, /* This bms address. | */
   TTALINK_BMS2_ADDRESS=152, /* This bms address. | */
   TTALINK_BMS3_ADDRESS=153, /* This bms address. | */
   TTALINK_BMS4_ADDRESS=154, /* This bms address. | */
   TTALINK_CHARGER_ADDRESS=155, /* This bms address. | */
   TTALINK_HK_ADDRESS=175, /* 火控地址. | */
   TTALINK_IMU_DK13_ADDRESS=176, /* This foc address. | */
   TTALINK_WEATHER_STATION_ADDRESS=190, /* 气象站 weather station. | */
   TTALINK_MASTER_ADDRESS=200, /* This bat address. | */
   TTALINK_SLAVE_ADDRESS=201, /* This bat address. | */
   TTALINK_MULTICASTING_1_ADDRESS=210, /* group of top points, online server/offline server/apps  | */
   TTALINK_PROJECT_APP_ADDRESS=240, /* 项目APP地址 | */
   TTALINK_EDUCATION_APP_ADDRESS=241, /* 教育APP地址 | */
   TTALINK_AGRICULTURE_APP_ADDRESS=242, /* 农业APP地址 | */
   TTALINK_REGULATE_APP_ADDRESS=243, /* 监管平台APP地址 | */
   TTALINK_EXTRA_LINK_ADDRESS=244, /* 额外遥控器地址 | */
   TTALINK_TX2_ADDRESS=249, /* This TX2 address. | */
   TTALINK_VR_ADDRESS=251, /* This vr address. | */
   TTALINK_PSV_ADDRESS=252, /* 便携式离线服务器地址. | */
   TTALINK_EMBE_ADDRESS=253, /* This embe address. | */
   TTALINK_SV_ADDRESS=254, /* This sv address. | */
   TTALINK_0xff_ADDRESS=255, /* This ground station address. | */
   TTALINK_ADDRESS_ID_ENUM_END=256, /*  | */
} TTALINK_ADDRESS_ID;
#endif

/** @brief successfail */
#ifndef HAVE_ENUM_ACK_BOOL
#define HAVE_ENUM_ACK_BOOL
typedef enum ACK_BOOL
{
   ACK_SUCCESS=0, /*  .  | */
   ACK_FAIL=1, /*  .  | */
   ACK_BOOL_ENUM_END=2, /*  | */
} ACK_BOOL;
#endif

/** @brief  */
#ifndef HAVE_ENUM_ACK_TYPE
#define HAVE_ENUM_ACK_TYPE
typedef enum ACK_TYPE
{
   ACK_OK=0, /*  .  | */
   ACK_ERR=1, /*  .  | */
   ACK_ROCKER_ACTION=15, /*  执行航线失败：遥控器未在中位.  | */
   ACK_STATUS_ACTION=16, /*  执行航线失败：无人机飞行状态错误.  | */
   ACK_WP_SD_ERR=17, /*  执行航线失败：航线文件异常. | */
   ACK_WP_SELF_CHECK=18, /* 执行航线失败：无人机自检未通过. | */
   ACK_WP_SYNC_CPU_ERR=19, /*  执行航线失败：同步CPU错误. | */
   ACK_WP_SYNC_CPU_QUEUE=20, /*  执行航线失败：规划航线未回复. | */
   ACK_WP_SYNC_CREAT_HEADK=21, /* 执行航线失败：初始规划航线时错误. | */
   ACK_GCS_SYNC_MASTER_TEST=22, /*  执行航线失败：切换主机模式异常. | */
   ACK_GCS_SYNC_SLAVE_TEST=23, /*  执行航线失败：切换从机模式异常. | */
   ACK_WP_ATTI_MODE=24, /* 执行航线失败:未处于定点模式. | */
   ACK_WP_HOVER_CONDITION=27, /*  . | */
   ACK_WP_ADD_ITEM_INDEX_ERROR=30, /*  .  | */
   ACK_WP_ADD_ITEM_INDEX_OVER=31, /*  .  | */
   ACK_WP_ADD_ITEM_CREAT_ERROR=32, /*  .  | */
   ACK_WP_ADD_ITEM_CREAT_TIEMOUT=33, /*  .  | */
   ACK_WP_ADD_ITEM_WRITE_ERROR=34, /*  .  | */
   ACK_WP_ADD_ITEM_WRITE_TIEMOUT=35, /*  .  | */
   ACK_WP_ACTION_STATUS_ERR=40, /*  .  | */
   ACK_WP_ACTION_ANCHOR_LAND=41, /*  .  | */
   ACK_WP_ACTION_ANCHOR_DEFAULT=42, /*  .  | */
   ACK_AB_SET_MODE_ERROR=50, /*  记录AB点未处于定点模式.  | */
   ACK_AB_FLIGHT_MODE_ERROR=51, /*  未满足定点飞行要求.  | */
   ACK_AB_CLEAR_MODE_ERROR=52, /*  清除AB点未处于定点模式.  | */
   ACK_AB_SET_A_ERR=55, /*  记录A点失败.  | */
   ACK_AB_SET_B_ERR=56, /*  记录B点失败.  | */
   ACK_AB_SET_DIRECTION_ERR=57, /*  设置触发方向失败.  | */
   ACK_WP_READ_NULL=60, /*  读取航线断点失败.  | */
   ACK_WP_READ_OVERTIME=61, /*  读取航线断点超时.  | */
   ACK_GENERAL_DEFAULT=100, /*  通用协议默认错误.  | */
   ACK_SV_ERR=200, /*  General error in transmission between servers  | */
   ACK_SV_TRANSMIT_POINT_TIMEOUT=201, /*  Server transmission waypoint timeout  | */
   ACK_TYPE_ENUM_END=202, /*  | */
} ACK_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_FILE_READ_CMD
#define HAVE_ENUM_FILE_READ_CMD
typedef enum FILE_READ_CMD
{
   TTALINK_FILE_READ_COUNTS=0, /*  Reserved  | */
   TTALINK_FILE_READ_INTERVAL_COUNTS=1, /*  Reserved  | */
   TTALINK_FILE_READ_ORDER_COUNTS=2, /*  Reserved  | */
   TTALINK_FTP_TYPE_END=3, /*  Reserved  | */
   FILE_READ_CMD_ENUM_END=4, /*  | */
} FILE_READ_CMD;
#endif

/** @brief  */
#ifndef HAVE_ENUM_SWITCH_CMD
#define HAVE_ENUM_SWITCH_CMD
typedef enum SWITCH_CMD
{
   TTALINK_LOG_SWITCH_NULL=0, /*  无动作.  | */
   TTALINK_LOG_SWITCH_START=1, /*  .  | */
   TTALINK_LOG_SWITCH_END=2, /*  .  | */
   TTALINK_LOG_SIMULINK_START=3, /*  .  | */
   TTALINK_LOG_SIMULINK_END=4, /*  .  | */
   TTALINK_FUCNTION_RESTART=5, /* .  | */
   TTALINK_RTK_PPK_START=10, /*  set ppk.  | */
   TTALINK_RTK_PPK_END=11, /*  set ppk.  | */
   TTALINK_RTK_MOVING=12, /*  set moving rtk.  | */
   TTALINK_RTK_BASE=13, /*  set base rtk.  | */
   TTALINK_RTK_FRESET=14, /*  set moving rtk freset.  | */
   TTALINK_RTK_CONFIG=15, /*  set moving rtk config.  | */
   TTALINK_WP_DEFAULT_TURN=20, /*  航线规划航策略  | */
   TTALINK_WP_FIXED_TURN=21, /*  航线不规划航策略.  | */
   TTALINK_LAND_GEAR_OPEN=24, /*  .  | */
   TTALINK_LAND_GEAR_TRUN_OFF=25, /*  .  | */
   TTALINK_LAND_NEED_UPDATE=26, /*  .  | */
   TTALINK_LAND_SETUP_DONE=27, /*  .  | */
   TTALINK_USER_EX_IO_HIGH=50, /*  用户控制IO高电平触发.  | */
   TTALINK_USER_EX_IO_EDGE=51, /*  用户控制IO边沿触发.  | */
   SWITCH_CMD_ENUM_END=52, /*  | */
} SWITCH_CMD;
#endif

/** @brief  */
#ifndef HAVE_ENUM_REQUEST_ACTIVATE_TYPE
#define HAVE_ENUM_REQUEST_ACTIVATE_TYPE
typedef enum REQUEST_ACTIVATE_TYPE
{
   SHARE_REQUEST_TYPE_NULL=0, /*  .  | */
   SHARE_REQUEST_TYPE_CHEAK=1, /*  .  | */
   SHARE_REQUEST_TYPE_CTRL=2, /*  .  | */
   SHARE_REQUEST_TYPE_NAV=3, /*  .  | */
   SHARE_REQUEST_TYPE_WP_A=4, /*  .  | */
   SHARE_REQUEST_TYPE_WP_B=5, /*  .  | */
   REQUEST_ACTIVATE_TYPE_ENUM_END=6, /*  | */
} REQUEST_ACTIVATE_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_SYNC_CTRL_TYPE
#define HAVE_ENUM_SYNC_CTRL_TYPE
typedef enum SYNC_CTRL_TYPE
{
   SYNC_CTRL_TYPE_NULL=0, /*  .  | */
   SYNC_CTRL_TYPE_AUTO_CHECK=1, /*  .  | */
   SYNC_CTRL_TYPE_STICK_CHECK=2, /*  .  | */
   SYNC_CTRL_TYPE_RESET_MOTOR=3, /*  .  | */
   SYNC_CTRL_TYPE_RESET_FLIGHT=4, /*  .  | */
   SYNC_CTRL_TYPE_ENUM_END=5, /*  | */
} SYNC_CTRL_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_FUNCTION_CMD_TYPE
#define HAVE_ENUM_FUNCTION_CMD_TYPE
typedef enum FUNCTION_CMD_TYPE
{
   TTALINK_TAKEOFF=1, /*  起飞.  | */
   TTALINK_LAND=2, /*  降落.  | */
   TTALINK_UNLOCK_MOTOR=3, /*  解锁马达.  | */
   TTALINK_LOCK_MOTOR=4, /*  上锁马达.  | */
   TTALINK_MISSION_START=10, /*  自动航线.  | */
   TTALINK_MISSION_STOP=11, /*  .  | */
   TTALINK_MISSION_AGAIN=12, /*  .  | */
   TTALINK_MISSION_QUIT=13, /*  .  | */
   TTALINK_MISSION_VEL_LOW=14, /*  .  | */
   TTALINK_MISSION_VEL_MID=15, /*  .  | */
   TTALINK_MISSION_VEL_HIGHT=16, /*  .  | */
   TTALINK_MISSION_REQUEST_LAND=17, /*  .  | */
   TTALINK_RETURN_HOME=18, /*  .  | */
   TTALINK_MISSION_HIGHT_LOW=19, /*  .  | */
   TTALINK_MISSION_HIGHT_MID=20, /*  .  | */
   TTALINK_MISSION_HIGHT_HIGHT=21, /*  .  | */
   TTALINK_MISSION_VEL_AUTO_SUB=22, /*  .  | */
   TTALINK_MISSION_VEL_AUTO_ADD=23, /*  .  | */
   TTALINK_MISSION_VEL_AUTO_NOR=24, /*  .  | */
   TTALINK_MISSION_SET_VEL=25, /*  .  | */
   TTALINK_MISSION_SET_ALT=26, /*  .  | */
   TTALINK_WP_RETURN_HOME=27, /*  .  | */
   TTALINK_EMERGENCY_PADDLE=28, /*  .  | */
   TTALINK_EMERGENCY_UMBRELLA=29, /*  .  | */
   TTALINK_DEBUG_RC=30, /*  .  | */
   TTALINK_DEBUG_TEST=31, /*  .  | */
   TTALINK_BACK_DOOR=32, /*  .  | */
   TTALINK_GCS_LOG_SWITCH=33, /*  .  | */
   TTALINK_GET_EMBE_IP_MSG=34, /*  .  | */
   TTALINK_CONFIG_VIDEO_STREAM=35, /*  .  | */
   TTALINK_MISSION_GO_TO_POINT=50, /*  .  | */
   TTALINK_MISSION_GO_TO_LAND=51, /*  .  | */
   TTALINK_MISSION_START_AB=59, /*  .  | */
   TTALINK_MISSION_DELETE_WP_BREAK=60, /*  .  | */
   TTALINK_MISSION_DELETE_AB_BREAK=61, /*  .  | */
   FUNCTION_CMD_TYPE_ENUM_END=62, /*  | */
} FUNCTION_CMD_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_PARAM_PROTECT_ENUM
#define HAVE_ENUM_PARAM_PROTECT_ENUM
typedef enum PARAM_PROTECT_ENUM
{
   PARAM_PROTECT_NULL=0, /*  无动作.  | */
   PARAM_PROTECT_ALARM=1, /*  声光报警.  | */
   PARAM_PROTECT_HOVER=2, /*  悬停.  | */
   PARAM_PROTECT_RTH=3, /*  直接返航.  | */
   PARAM_PROTECT_LAND=4, /*  降落.  | */
   PARAM_PROTECT_GTH=5, /*  原路返航.  | */
   PARAM_PROTECT_ENUM_ENUM_END=6, /*  | */
} PARAM_PROTECT_ENUM;
#endif

/** @brief  */
#ifndef HAVE_ENUM_PARAM_ID_TYPE
#define HAVE_ENUM_PARAM_ID_TYPE
typedef enum PARAM_ID_TYPE
{
   TTALINK_PARAM_NULL=0, /*  .  | */
   TTALINK_PARAM_SENS_RC=1, /*  .  | */
   TTALINK_PARAM_SENS_ATTI=2, /*  .  | */
   TTALINK_PARAM_GYRO_PITCH_KP=3, /*  .  | */
   TTALINK_PARAM_GYRO_PITCH_KI=4, /*  .  | */
   TTALINK_PARAM_GYRO_PITCH_KD=5, /*  .  | */
   TTALINK_PARAM_GYRO_ROLL_KP=6, /*  .  | */
   TTALINK_PARAM_GYRO_ROLL_KI=7, /*  .  | */
   TTALINK_PARAM_GYRO_ROLL_KD=8, /*  .  | */
   TTALINK_PARAM_GYRO_YAW_KP=9, /*  .  | */
   TTALINK_PARAM_GYRO_YAW_KI=10, /*  .  | */
   TTALINK_PARAM_GYRO_YAW_KD=11, /*  .  | */
   TTALINK_PARAM_ANGLE_PITCH_KP=12, /*  .  | */
   TTALINK_PARAM_ANGLE_PITCH_KI=13, /*  .  | */
   TTALINK_PARAM_ANGLE_PITCH_KD=14, /*  .  | */
   TTALINK_PARAM_ANGLE_ROLL_KP=15, /*  .  | */
   TTALINK_PARAM_ANGLE_ROLL_KI=16, /*  .  | */
   TTALINK_PARAM_ANGLE_ROLL_KD=17, /*  .  | */
   TTALINK_PARAM_ANGLE_YAW_KP=18, /*  .  | */
   TTALINK_PARAM_ANGLE_YAW_KI=19, /*  .  | */
   TTALINK_PARAM_ANGLE_YAW_KD=20, /*  .  | */
   TTALINK_PARAM_ALT_ACC_KP=21, /*  .  | */
   TTALINK_PARAM_ALT_ACC_KI=22, /*  .  | */
   TTALINK_PARAM_ALT_ACC_KD=23, /*  .  | */
   TTALINK_PARAM_ALT_VEL_KP=24, /*  .  | */
   TTALINK_PARAM_ALT_HIGH_KP=25, /*  .  | */
   TTALINK_PARAM_POS_LOC_KP=26, /*  .  | */
   TTALINK_PARAM_POS_VEL_KP=27, /*  .  | */
   TTALINK_PARAM_POS_VEL_KI=28, /*  .  | */
   TTALINK_PARAM_POS_VEL_KD=29, /*  .  | */
   TTALINK_PARAM_POS_ACC_KP=30, /*  .  | */
   TTALINK_PARAM_POS_ACC_KI=31, /*  .  | */
   TTALINK_PARAM_POS_ACC_HOVER=32, /*  .  | */
   TTALINK_PARAM_FUZZY_E_ZO=33, /*  .  | */
   TTALINK_PARAM_FUZZY_E_PS=34, /*  .  | */
   TTALINK_PARAM_FUZZY_E_PM=35, /*  .  | */
   TTALINK_PARAM_FUZZY_E_PB=36, /*  .  | */
   TTALINK_PARAM_FUZZY_EC_ZO=37, /*  .  | */
   TTALINK_PARAM_FUZZY_EC_PS=38, /*  .  | */
   TTALINK_PARAM_FUZZY_EC_PM=39, /*  .  | */
   TTALINK_PARAM_FUZZY_EC_PB=40, /*  .  | */
   TTALINK_PARAM_FUZZY_P_ZO=41, /*  .  | */
   TTALINK_PARAM_FUZZY_P_PS=42, /*  .  | */
   TTALINK_PARAM_FUZZY_P_PM=43, /*  .  | */
   TTALINK_PARAM_FUZZY_P_PB=44, /*  .  | */
   TTALINK_PARAM_VERSION=48, /*  .  | */
   TTALINK_PARAM_FLIGHT_END=49, /*  .  | */
   TTALINK_PARAM_SYS_FRAME=50, /*  机架类型.  | */
   TTALINK_PARAM_SYS_CONFIG=51, /*  用户配置.  | */
   TTALINK_PARAM_SYS_SIMULATION_MODE=52, /*  模拟类型. 0=模拟仿真 2=飞行能力评估模块  | */
   TTALINK_PARAM_SYS_PARAM1=53, /*  系统预留.  | */
   TTALINK_PARAM_SYS_AUTHORIZE_FLAG=54, /*  授权标志位 0=未授权 1=已授权 0xff=后门授权.  | */
   TTALINK_PARAM_SYS_AUTHORIZE_CHANGE=55, /*  授权更改标志 原则上三个月更新一次.  | */
   TTALINK_PARAM_SYS_AUTHORIZE_CHANGE_DATE=56, /*  授权更改日期.  | */
   TTALINK_PARAM_SYS_AUTHORIZE_DATE=57, /*  授权日期.  | */
   TTALINK_PARAM_SYS_FC_LIMIT=58, /*  无人机飞控受限.  | */
   TTALINK_PARAM_SYS_GRAVITY_X=80, /*  重心X.  | */
   TTALINK_PARAM_SYS_GRAVITY_Y=81, /*  重心Y.  | */
   TTALINK_PARAM_SYS_GRAVITY_Z=82, /*  重心Z.  | */
   TTALINK_PARAM_SYS_MOI_X=83, /*  转动惯量X （Moment of Inertia）.  | */
   TTALINK_PARAM_SYS_MOI_Y=84, /*  转动惯量Y （Moment of Inertia）.  | */
   TTALINK_PARAM_SYS_MOI_Z=85, /*  转动惯量Z （Moment of Inertia）.  | */
   TTALINK_PARAM_SYS_WEIGHT=86, /*  重量.  | */
   TTALINK_PARAM_SYS_THRUST_A_COEF=87, /*  推力系数A.  | */
   TTALINK_PARAM_SYS_THRUST_B_COEF=88, /*  推力系数B.  | */
   TTALINK_PARAM_SYS_THRUST_C_COEF=89, /*  推力系数C.  | */
   TTALINK_PARAM_SYS_TORQUE_COEF=90, /*  扭力系数.  | */
   TTALINK_PARAM_SYS_THRUST_S_COEF=91, /*  推力系数S.  | */
   TTALINK_PARAM_SYS_THRUST_SS_COEF=92, /*  推力系数SS.  | */
   TTALINK_PARAM_SYS_THRUST_SSS_COEF=93, /*  推力系数SSS.  | */
   TTALINK_PARAM_SYS_MAX_MOTOR_THRUST=95, /*  单个马达最大推力.  | */
   TTALINK_PARAM_SYS_POWER_VERSION=157, /*  马达动力参数列表 版本号.  | */
   TTALINK_PARAM_SYSTEM_VERSION=158, /*  System_Config版本号.  | */
   TTALINK_PARAM_SYSTEM_END=159, /*  系统结束.  | */
   TTALINK_PARAM_UAV_JERK_MAX=160, /*  加加速度最大值.  | */
   TTALINK_PARAM_UAV_ACC_MIN=161, /*  加速度最小值.  | */
   TTALINK_PARAM_UAV_ACC_MAX=162, /*  加速度最大值.  | */
   TTALINK_PARAM_UAV_VEL_MAX=163, /*  速度最大值.  | */
   TTALINK_PARAM_UAV_RC_VEL_MAX=164, /*  遥控器打杆速度最大值.  | */
   TTALINK_PARAM_UAV_VEL_FILTER_CUTOFF=165, /*  速度滤波截止频率.  | */
   TTALINK_PARAM_UAV_AB_VEL_MAX=166, /*  AB点航线速度最大值.  | */
   TTALINK_PARAM_UAV_AB_WIDTH=167, /*  AB点航线喷福.  | */
   TTALINK_PARAM_UAV_ANGLE_MAX=168, /*  遥控器姿态最大角度.  | */
   TTALINK_PARAM_UAV_ACC_STEP_FLAG=169, /*  加速度阶梯标志.  | */
   TTALINK_PARAM_UAV_ACC_CTRL_FLAG=170, /*  加速度闭环可用标志.  | */
   TTALINK_PARAM_UAV_ACC_SMOOTH_TIME=171, /*  加速度闭环平滑时间.  | */
   TTALINK_PARAM_UAV_ACC_ROOT_VAR_MAX=172, /*  加速度矢量和方差最大值.  | */
   TTALINK_PARAM_UAV_WP_ACC_MIN=173, /*  航线加速度最小值.  | */
   TTALINK_PARAM_UAV_WP_ACC_MAX=174, /*  航线加速度最大值.  | */
   TTALINK_PARAM_UAV_FUZZY_CTRL_FLAG=175, /*  模糊控制可用标志.  | */
   TTALINK_PARAM_UAV_STEERING_GAIN=176, /*  辅助转向增益.  | */
   TTALINK_PARAM_UAV_NAVI_ATTI_LAND_ABLE=177, /*  导航手动标志字自动降落可用标志.  | */
   TTALINK_PARAM_UAV_GCS_LOST_ACTION=179, /*  GCS失控保护动作(0.无动作  1.声光报警 2.悬停 3.直接返航 4.降落 5.原路返航).  | */
   TTALINK_PARAM_UAV_RC_LOST_ACTION=180, /*  遥控器失控保护动作(0.无动作  1.声光报警 2.悬停 3.直接返航 4.降落 5.原路返航).  | */
   TTALINK_PARAM_UAV_VOLATGE_LEVEL_B_VALUE=181, /*  低电量B级报警电量值.  | */
   TTALINK_PARAM_UAV_VOLATGE_LEVEL_B_ACTION=182, /*  低电量B级报警动作(0.无动作  1.声光报警 2.悬停 3.直接返航 4.降落 5.原路返航) .  | */
   TTALINK_PARAM_UAV_VOLATGE_LEVEL_A_VALUE=183, /*  低电量A级报警电量值.  | */
   TTALINK_PARAM_UAV_VOLATGE_LEVEL_A_ACTION=184, /*  低电量A级报警动作(0.无动作  1.声光报警 2.悬停 3.直接返航 4.降落 5.原路返航).  | */
   TTALINK_PARAM_UAV_ADC_BATTERY_CELL=185, /*  非智能电池节数.  | */
   TTALINK_PARAM_UAV_PESTICIDE_EMPTY_ACTION=186, /*  断药保护动作(0.无动作  1.声光报警 2.悬停 3.直接返航 4.降落 5.原路返航).  | */
   TTALINK_PARAM_UAV_ALTIT_MAX_LIMIT=190, /*  最大限飞高度.  | */
   TTALINK_PARAM_UAV_DISTANCE_MAX_LIMIT=191, /*  最大限飞距离.  | */
   TTALINK_PARAM_UAV_RTH_ALTIT_LIMIT=192, /*  返航最大高度.  | */
   TTALINK_PARAM_UAV_RTH_SPEED_LIMIT=193, /*  返航最大速度.  | */
   TTALINK_PARAM_UAV_YAW_CTRL_ABLE=210, /*  航向可用标志.  | */
   TTALINK_PARAM_UAV_REMOTE_CONTROL_SWITCH=211, /*  遥控器模式 0 美国手 1 日本手.  | */
   TTALINK_PARAM_UAV_REMOTE_CONTROL_DEVICE=212, /*  遥控器设备 0：T12/FUATBA  1:H12..  | */
   TTALINK_PARAM_UAV_REMOTE_MODE_STABILE_ENABLE_SW6_0=213, /*  增稳模式开关.  | */
   TTALINK_PARAM_UAV_REMOTE_MODE_PARACHUTE_CTRL_SW7_2=214, /*  降落伞开关.  | */
   TTALINK_PARAM_UAV_FOLLOWING_RADAR_ABLE=220, /*  仿地雷达可用标志.（0：失效  1：启用）  | */
   TTALINK_PARAM_UAV_FRONT_AVOIDANCE_RADAR_ABLE=221, /*  前避障雷达可用标志 （0：失效  1：启用）.  | */
   TTALINK_PARAM_UAV_BACK_AVOIDANCE_RADAR_ABLE=222, /*  后避障雷达可用标志 （0：失效  1：启用）.  | */
   TTALINK_PARAM_UAV_AVOIDANCE_RADAR_PARAM=223, /*  仿地雷达灵敏度.  | */
   TTALINK_PARAM_UAV_AVOIDANCE_RADAR_DISTANCE=224, /*  避障雷达悬停的最小距离.  | */
   TTALINK_PARAM_UAV_SPRAY_ABLE=225, /*  喷雾可用标志 （0：失效  1：启用 2：自动）.  | */
   TTALINK_PARAM_UAV_SPRAY_BRAKE_ABLE=226, /*  断点续喷可用标志（0：失效  1：启用）.  | */
   TTALINK_PARAM_UAV_SPRAY_PARAM=227, /*  喷雾流量（0：失效  1：启用 2：自动）.  | */
   TTALINK_PARAM_UAV_SPRAY_WIDTH=228, /*  喷雾喷幅（无人机轴距长度，默认配置）.  | */
   TTALINK_PARAM_UAV_SPRAY_CALI_PARAM=229, /*  喷洒标定因数.  | */
   TTALINK_PARAM_UAV_SPRAY_SPACE=230, /*  喷洒规划航线间距.  | */
   TTALINK_PARAM_UAV_SPRAY_FLOW_PARAM=231, /*  喷洒参数：流速数值 L/min.  | */
   TTALINK_PARAM_UAV_SPRAY_MU_PARAM=232, /*  喷洒参数：亩喷量数值 L/MU.  | */
   TTALINK_PARAM_UAV_SPREAD_WIDTH=233, /*  播撒规划喷幅.  | */
   TTALINK_PARAM_UAV_SPREAD_FLOW_PARAM=234, /*  播撒参数：流速数值 .  | */
   TTALINK_PARAM_UAV_SPREAD_MU_PARAM=235, /*  播撒参数：亩喷量数值 .  | */
   TTALINK_PARAM_UAV_AG_DEVICE_SWITCH=239, /*  农业外设选择 .  | */
   TTALINK_PARAM_UAV_PARAM_END=240, /*  .  | */
   TTALINK_PARAM_UAV_STATIC_PARAM_1=245, /*  静态参数1.  | */
   TTALINK_PARAM_UAV_STATIC_PARAM_2=246, /*  静态参数2.  | */
   TTALINK_PARAM_UAV_STATIC_PARAM_3=247, /*  静态参数3.  | */
   TTALINK_PARAM_UAV_STATIC_PARAM_4=248, /*  静态参数4.  | */
   TTALINK_PARAM_UAV_STATIC_PARAM_5=249, /*  静态参数5.  | */
   TTALINK_PARAM_UAV_PLAN_LIMIT_ANGLE=250, /*  弧线向心角度限制.  | */
   TTALINK_PARAM_UAV_PLAN_MAX_ACC=251, /*  最大规划向心加速度.  | */
   TTALINK_PARAM_UAV_PLAN_MAX_CLIMB_VLU=252, /*  最大规划爬升速度.  | */
   TTALINK_PARAM_UAV_PLAN_MAX_YAW_GYRO=253, /*  最大规划航向角速度.  | */
   TTALINK_PARAM_UAV_PLAN_ACC_ACC=254, /*  最大规划加加速度.  | */
   TTALINK_PARAM_UAV_PLAN_MAX_CLIMB_VLD=255, /*  最大规划下降速度.  | */
   TTALINK_PARAM_UAV_VERSION=269, /*  Uav_Config版本号.  | */
   TTALINK_PARAM_UAV_CONFIG_END=270, /*  .  | */
   TTALINK_PARAM_UAV_RECORD_END=280, /*  .  | */
   TTALINK_PARAM_USER_MONITOR_ALL_SHIELD=281, /*  监控器全屏蔽.  | */
   TTALINK_PARAM_USER_MONITOR_MAG1_SHIELD=282, /*  监控器屏蔽罗盘1.  | */
   TTALINK_PARAM_USER_MONITOR_MAG2_SHIELD=283, /*  监控器屏蔽罗盘2.  | */
   TTALINK_PARAM_USER_MONITOR_GPS_SHIELD=284, /*  监控器屏蔽GPS.  | */
   TTALINK_PARAM_USER_MONITOR_RTK_SHIELD=285, /*  监控器屏蔽RTK.  | */
   TTALINK_PARAM_USER_MONITOR_SD_SHIELD=286, /*  监控器屏蔽SD卡.  | */
   TTALINK_PARAM_USER_MONITOR_BATTERY_SHIELD=287, /*  监控器屏蔽电池电压.  | */
   TTALINK_PARAM_USER_MONITOR_LEVEL2_SHIELD=288, /*  监控器屏蔽LEVEL2.  | */
   TTALINK_PARAM_USER_MONITOR_LEVEL1_SHIELD=289, /*  监控器屏蔽LEVEL1.  | */
   TTALINK_PARAM_USER_MONITOR_PARACHUTE_SHIELD=290, /*  监控器屏蔽PARACHUTE.  | */
   TTALINK_PARAM_USER_TASK_HEART_TIME=300, /*  心跳任务更新时间.  | */
   TTALINK_PARAM_USER_TASK_TAHRS_IME=301, /*  姿态任务更新时间.  | */
   TTALINK_PARAM_USER_TASK_LOCATION_TIME=302, /*  位置信息任务更新时间.  | */
   TTALINK_PARAM_USER_TASK_DEVICE_TIME=303, /*  外设数据任务更新时间.  | */
   TTALINK_PARAM_USER_TASK_BATTERY_TIME=304, /*  电池信息任务更新时间.  | */
   TTALINK_PARAM_USER_TASK_WP_CODE_TIME=305, /*  航线码任务更新时间.  | */
   TTALINK_PARAM_USER_TASK_MONITOR_TIME=306, /*  监控器码任务更新时间.  | */
   TTALINK_PARAM_USER_TASK_RC__TIME=307, /*  遥控器数据任务更新时间.  | */
   TTALINK_PARAM_USER_TASK_SENSOR_IMU_MAG_TIME=308, /*  传感器 IMU/气压计 任务更新时间.  | */
   TTALINK_PARAM_USER_TASK_SENSOR_BARO_GPS_RTK_TIME=309, /*  传感器 罗盘 GPS,RTK 任务更新时间.  | */
   TTALINK_PARAM_USER_DEBUG_LONGI=310, /*  模拟航线测试经度.  | */
   TTALINK_PARAM_USER_DEBUG_LATIT=311, /*  模拟航线测试纬度.  | */
   TTALINK_PARAM_USER_CHANGE_HIGH_VEL=315, /*  航线变高最大速度.  | */
   TTALINK_PARAM_USER_CHANGE_HIGH_ACC=316, /*  航线变高最大加速度.  | */
   TTALINK_PARAM_USER_CHANGE_HIGH_ACCACC=317, /*  航线变高最大加加速度.  | */
   TTALINK_PARAM_USER_VERSION=319, /*  Uav_Config版本号.  | */
   TTALINK_PARAM_USER_END=320, /*  .  | */
   TTALINK_PARAM_ELCE_FENCE_NUM=330, /*  电子围栏编号.  | */
   TTALINK_PARAM_ELCE_FENCE_TYPE=331, /*  电子围栏类型 0=默认限飞区域 1=圆形电子围栏. 2=多边形电子围栏 | */
   TTALINK_PARAM_ELEC_FENCE_DATA=332, /*  默认电子围栏数据.  | */
   TTALINK_PARAM_ELEC_FENCE_SET=333, /*  设置电子围栏数据.  | */
   TTALINK_PARAM_FENCE_NOFLY_NEAR_DIS=340, /*  禁飞区临界区距离限制.  | */
   TTALINK_PARAM_FENCE_NOFLY_LIMIT_DIS=341, /*  禁飞区限制区距离限制.  | */
   TTALINK_PARAM_FENCE_ELEC_NEAR_DIS=342, /*  电子围栏临界区距离限制.  | */
   TTALINK_PARAM_FENCE_ELEC_LIMIT_DIS=343, /*  电子围栏限制区距离限制.  | */
   TTALINK_PARAM_ELEC_FENCE_END=344, /*  电子围栏结束.  | */
   TTALINK_PARAM_TEMP_NOFLY_FENCE_SET=345, /*  设置 临时禁飞区 数据.  | */
   TTALINK_PARAM_TEMP_NOFLY_FENCE_READ=346, /*  读取 临时禁飞区 数据.  | */
   TTALINK_PARAM_TEMP_NOFLY_FENCE_END=350, /*  临时禁飞区结束.  | */
   TTALINK_PARAM_RC_CHANNEL_6_0=351, /*  遥控器 6通道-0-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_6_1=352, /*  遥控器 6通道-1-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_6_2=353, /*  遥控器 6通道-2-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_7_0=354, /*  遥控器 7通道-0-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_7_1=355, /*  遥控器 7通道-1-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_7_2=356, /*  遥控器 7通道-2-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_8_0=357, /*  遥控器 8通道-0-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_8_1=358, /*  遥控器 8通道-1-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_8_2=359, /*  遥控器 8通道-2-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_9_0=360, /*  遥控器 9通道-0-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_9_1=361, /*  遥控器 9通道-1-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_9_2=362, /*  遥控器 9通道-2-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_10_0=363, /*  遥控器 10通道-0-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_10_1=364, /*  遥控器 10通道-1-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_10_2=365, /*  遥控器 10通道-2-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_11_0=366, /*  遥控器 11通道-0-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_11_1=367, /*  遥控器 11通道-1-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_11_2=368, /*  遥控器 11通道-2-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_12_0=369, /*  遥控器 12通道-0-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_12_1=370, /*  遥控器 12通道-1-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_12_2=371, /*  遥控器 12通道-2-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_13_0=372, /*  遥控器 13通道-0-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_13_1=373, /*  遥控器 13通道-1-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_13_2=374, /*  遥控器 13通道-2-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_14_0=375, /*  遥控器 14通道-0-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_14_1=376, /*  遥控器 14通道-1-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_14_2=377, /*  遥控器 14通道-2-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_15_0=378, /*  遥控器 15通道-0-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_15_1=379, /*  遥控器 15通道-1-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_15_2=380, /*  遥控器 15通道-2-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_16_0=381, /*  遥控器 16通道-0-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_16_1=382, /*  遥控器 16通道-1-动作.  | */
   TTALINK_PARAM_RC_CHANNEL_16_2=383, /*  遥控器 16通道-2-动作.  | */
   TTALINK_PARAM_RC_ACTION_EVENT_0=384, /*  遥控器 通道-0-动作.  | */
   TTALINK_PARAM_RC_ACTION_EVENT_1=385, /*  遥控器 通道-1-动作.  | */
   TTALINK_PARAM_RC_ACTION_EVENT_2=386, /*  遥控器 通道-2-动作.  | */
   TTALINK_PARAM_RC_ACTION_VALUE_0=387, /*  遥控器 通道-0-动作值.  | */
   TTALINK_PARAM_RC_ACTION_VALUE_1=388, /*  遥控器 通道-1-动作值.  | */
   TTALINK_PARAM_RC_ACTION_VALUE_2=389, /*  遥控器 通道-2-动作值.  | */
   TTALINK_PARAM_RC_ACTION_VALUE_MIN=390, /*  遥控器 通道-0-动作最大值.  | */
   TTALINK_PARAM_RC_ACTION_VALUE_MAX=391, /*  遥控器 通道-2-动作最小值.  | */
   TTALINK_PARAM_RC_CHANNEL_TYPE=395, /*  遥控器通道类型.  | */
   TTALINK_PARAM_RC_CHANNEL_CONT=396, /*  遥控器通道开关个数.  | */
   TTALINK_PARAM_RC_ACTION_VERSION=398, /*  遥控器动作版本号.  | */
   TTALINK_PARAM_RC_REVERSE_BIT=399, /*  遥控器反向位域.  | */
   TTALINK_PARAM_RC_ACTION_END=400, /*  遥控器动作结束.  | */
   TTALINK_PARAM_FC_END=512, /*  .  | */
   TTALINK_PARAM_EDU_FIX_ANG_PITCH=513, /*   安装角度误差Pitch.  | */
   TTALINK_PARAM_EDU_FIX_ANG_ROLL=514, /*   安装角度误差Roll.  | */
   TTALINK_PARAM_EDU_FIX_ANG_YAW=515, /*   安装角度误差Yaw.  | */
   TTALINK_PARAM_EDU_FIX_POS_X=516, /*   安装位置误差X.  | */
   TTALINK_PARAM_EDU_FIX_POS_Y=517, /*   安装位置误差Y.  | */
   TTALINK_PARAM_EDU_FIX_POS_Z=518, /*   安装位置误差Z.  | */
   TTALINK_PARAM_BATT_CALI_K=520, /*   电压校准数据.  | */
   TTALINK_PARAM_RESERVE_1=521, /*   教育模块预留参数1.  | */
   TTALINK_PARAM_RESERVE_2=522, /*   教育模块预留参数2.  | */
   TTALINK_PARAM_RESERVE_3=523, /*   教育模块预留参数3.  | */
   TTALINK_PARAM_RESERVE_4=524, /*   教育模块预留参数4.  | */
   TTALINK_PARAM_RESERVE_5=525, /*   教育模块预留参数5.  | */
   TTALINK_PARAM_RESERVE_6=526, /*   教育模块预留参数6.  | */
   TTALINK_PARAM_RESERVE_7=527, /*   教育模块预留参数7.  | */
   TTALINK_PARAM_RESERVE_8=528, /*   教育模块预留参数8.  | */
   TTALINK_PARAM_RESERVE_9=529, /*   教育模块预留参数9.  | */
   TTALINK_PARAM_RESERVE_10=530, /*   教育模块预留参数10.  | */
   TTALINK_PARAM_EDU_SYS_END=550, /*  .  | */
   TTALINK_PARAM_EDU_END=630, /*  .  | */
   TTALINK_PARAM_ROS_START=1024, /*  .  | */
   TTALINK_PARAM_ROS_FLIGHT_START=1025, /*  .  | */
   TTALINK_PARAM_ROS_SENS_RC=1026, /*  .  | */
   TTALINK_PARAM_ROS_SENS_ATTI=1027, /*  .  | */
   TTALINK_PARAM_ATC_RAT_X_KP_DEFAULT=1028, /*  .  | */
   TTALINK_PARAM_ATC_RAT_X_KI_DEFAULT=1029, /*  .  | */
   TTALINK_PARAM_ATC_RAT_X_KD_DEFAULT=1030, /*  .  | */
   TTALINK_PARAM_ATC_RAT_X_IMAX_DEFAULT=1031, /*  .  | */
   TTALINK_PARAM_ATC_RAT_X_LIMIT_DEFAULT=1032, /*  .  | */
   TTALINK_PARAM_ATC_RAT_X_HZ_DEFAULT=1033, /*  .  | */
   TTALINK_PARAM_ATC_RAT_Y_KP_DEFAULT=1034, /*  .  | */
   TTALINK_PARAM_ATC_RAT_Y_KI_DEFAULT=1035, /*  .  | */
   TTALINK_PARAM_ATC_RAT_Y_KD_DEFAULT=1036, /*  .  | */
   TTALINK_PARAM_ATC_RAT_Y_IMAX_DEFAULT=1037, /*  .  | */
   TTALINK_PARAM_ATC_RAT_Y_LIMIT_DEFAULT=1038, /*  .  | */
   TTALINK_PARAM_ATC_RAT_Y_HZ_DEFAULT=1039, /*  .  | */
   TTALINK_PARAM_ATC_RAT_Z_KP_DEFAULT=1040, /*  .  | */
   TTALINK_PARAM_ATC_RAT_Z_KI_DEFAULT=1041, /*  .  | */
   TTALINK_PARAM_ATC_RAT_Z_KD_DEFAULT=1042, /*  .  | */
   TTALINK_PARAM_ATC_RAT_Z_IMAX_DEFAULT=1043, /*  .  | */
   TTALINK_PARAM_ATC_RAT_Z_LIMIT_DEFAULT=1044, /*  .  | */
   TTALINK_PARAM_ATC_RAT_Z_HZ_DEFAULT=1045, /*  .  | */
   TTALINK_PARAM_ATC_ANG_X_KP_DEFAULT=1050, /*  .  | */
   TTALINK_PARAM_ATC_ANG_X_KI_DEFAULT=1051, /*  .  | */
   TTALINK_PARAM_ATC_ANG_X_KD_DEFAULT=1052, /*  .  | */
   TTALINK_PARAM_ATC_ANG_X_IMAX_DEFAULT=1053, /*  .  | */
   TTALINK_PARAM_ATC_ANG_X_LIMIT_DEFAULT=1054, /*  .  | */
   TTALINK_PARAM_ATC_ANG_X_HZ_DEFAULT=1055, /*  .  | */
   TTALINK_PARAM_ATC_ANG_Y_KP_DEFAULT=1056, /*  .  | */
   TTALINK_PARAM_ATC_ANG_Y_KI_DEFAULT=1057, /*  .  | */
   TTALINK_PARAM_ATC_ANG_Y_KD_DEFAULT=1058, /*  .  | */
   TTALINK_PARAM_ATC_ANG_Y_IMAX_DEFAULT=1059, /*  .  | */
   TTALINK_PARAM_ATC_ANG_Y_LIMIT_DEFAULT=1060, /*  .  | */
   TTALINK_PARAM_ATC_ANG_Y_HZ_DEFAULT=1061, /*  .  | */
   TTALINK_PARAM_ATC_ANG_Z_KP_DEFAULT=1062, /*  .  | */
   TTALINK_PARAM_ATC_ANG_Z_KI_DEFAULT=1063, /*  .  | */
   TTALINK_PARAM_ATC_ANG_Z_KD_DEFAULT=1064, /*  .  | */
   TTALINK_PARAM_ATC_ANG_Z_IMAX_DEFAULT=1065, /*  .  | */
   TTALINK_PARAM_ATC_ANG_Z_LIMIT_DEFAULT=1066, /*  .  | */
   TTALINK_PARAM_ATC_ANG_Z_HZ_DEFAULT=1067, /*  .  | */
   TTALINK_PARAM_POS_KP_DEFAULT=1072, /*  .  | */
   TTALINK_PARAM_POS_KI_DEFAULT=1073, /*  .  | */
   TTALINK_PARAM_POS_KD_DEFAULT=1074, /*  .  | */
   TTALINK_PARAM_POS_IMAX_DEFAULT=1075, /*  .  | */
   TTALINK_PARAM_POS_LIMIT_DEFAULT=1076, /*  .  | */
   TTALINK_PARAM_POS_HZ_DEFAULT=1077, /*  .  | */
   TTALINK_PARAM_POS_VEL_KP_DEFAULT=1080, /*  .  | */
   TTALINK_PARAM_POS_VEL_KI_DEFAULT=1081, /*  .  | */
   TTALINK_PARAM_POS_VEL_KD_DEFAULT=1082, /*  .  | */
   TTALINK_PARAM_POS_VEL_IMAX_DEFAULT=1083, /*  .  | */
   TTALINK_PARAM_POS_VEL_LIMIT_DEFAULT=1084, /*  .  | */
   TTALINK_PARAM_POS_VEL_HZ_DEFAULT=1085, /*  .  | */
   TTALINK_PARAM_ALT_HIGH_KP_DEFAULT=1090, /*  .  | */
   TTALINK_PARAM_ALT_HIGH_KI_DEFAULT=1091, /*  .  | */
   TTALINK_PARAM_ALT_HIGH_KD_DEFAULT=1092, /*  .  | */
   TTALINK_PARAM_ALT_HIGH_IMAX_DEFAULT=1093, /*  .  | */
   TTALINK_PARAM_ALT_HIGH_LIMIT_DEFAULT=1094, /*  .  | */
   TTALINK_PARAM_ALT_HIGH_HZ_DEFAULT=1095, /*  .  | */
   TTALINK_PARAM_ALT_VEL_KP_DEFAULT=1100, /*  .  | */
   TTALINK_PARAM_ALT_VEL_KI_DEFAULT=1101, /*  .  | */
   TTALINK_PARAM_ALT_VEL_KD_DEFAULT=1102, /*  .  | */
   TTALINK_PARAM_ALT_VEL_IMAX_DEFAULT=1103, /*  .  | */
   TTALINK_PARAM_ALT_VEL_LIMIT_DEFAULT=1104, /*  .  | */
   TTALINK_PARAM_ALT_VEL_HZ_DEFAULT=1105, /*  .  | */
   TTALINK_PARAM_ALT_ACC_KP_DEFAULT=1110, /*  .  | */
   TTALINK_PARAM_ALT_ACC_KI_DEFAULT=1111, /*  .  | */
   TTALINK_PARAM_ALT_ACC_KD_DEFAULT=1112, /*  .  | */
   TTALINK_PARAM_ALT_ACC_IMAX_DEFAULT=1113, /*  .  | */
   TTALINK_PARAM_ALT_ACC_LIMIT_DEFAULT=1114, /*  .  | */
   TTALINK_PARAM_ALT_ACC_HZ_DEFAULT=1115, /*  .  | */
   TTALINK_PARAM_ROS_FLIGHT_VERSION=1149, /*  .  | */
   TTALINK_PARAM_ROS_FLIGHT_END=1150, /*  .  | */
   TTALINK_PARAM_ROS_SENSOR_START=1151, /*  .  | */
   TTALINK_PARAM_CALI_ACC_OFFSET0=1152, /*  .  | */
   TTALINK_PARAM_CALI_ACC_OFFSET1=1153, /*  .  | */
   TTALINK_PARAM_CALI_ACC_OFFSET2=1154, /*  .  | */
   TTALINK_PARAM_CALI_ACC_GAIN0=1155, /*  .  | */
   TTALINK_PARAM_CALI_ACC_GAIN1=1156, /*  .  | */
   TTALINK_PARAM_CALI_ACC_GAIN2=1157, /*  .  | */
   TTALINK_PARAM_CALI_MAG_OFFSET0=1160, /*  .  | */
   TTALINK_PARAM_CALI_MAG_OFFSET1=1161, /*  .  | */
   TTALINK_PARAM_CALI_MAG_OFFSET2=1162, /*  .  | */
   TTALINK_PARAM_CALI_MAG_GAIN00=1163, /*  .  | */
   TTALINK_PARAM_CALI_MAG_GAIN01=1164, /*  .  | */
   TTALINK_PARAM_CALI_MAG_GAIN02=1165, /*  .  | */
   TTALINK_PARAM_CALI_MAG_GAIN10=1166, /*  .  | */
   TTALINK_PARAM_CALI_MAG_GAIN11=1167, /*  .  | */
   TTALINK_PARAM_CALI_MAG_GAIN12=1168, /*  .  | */
   TTALINK_PARAM_CALI_MAG_GAIN20=1169, /*  .  | */
   TTALINK_PARAM_CALI_MAG_GAIN21=1170, /*  .  | */
   TTALINK_PARAM_CALI_MAG_GAIN22=1171, /*  .  | */
   TTALINK_PARAM_ANGLE_MAX=1180, /*  .  | */
   TTALINK_PARAM_GYRO_MAX=1181, /*  .  | */
   TTALINK_PARAM_GYRO_ACC_MAX=1182, /*  .  | */
   TTALINK_PARAM_ATTI_CF_HZ=1183, /*  .  | */
   TTALINK_PARAM_GYRO_CF_HZ=1184, /*  .  | */
   TTALINK_PARAM_GYRO_COFF=1185, /*  .  | */
   TTALINK_PARAM_ROS_SENSOR_END=1300, /*  .  | */
   TTALINK_PARAM_ROS_END=2048, /*  .  | */
   PARAM_ID_TYPE_ENUM_END=2049, /*  | */
} PARAM_ID_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GENERAL_PARAMS_TYPE
#define HAVE_ENUM_GENERAL_PARAMS_TYPE
typedef enum GENERAL_PARAMS_TYPE
{
   PARAM_TYPE_ERR=0, /*  数据类型错误.  | */
   PARAM_TYPE_UINT8=1, /*  .  | */
   PARAM_TYPE_INT8=2, /*   .   | */
   PARAM_TYPE_UINT16=3, /*   .   | */
   PARAM_TYPE_INT16=4, /*   .   | */
   PARAM_TYPE_UINT32=5, /*   .   | */
   PARAM_TYPE_INT32=6, /*   .   | */
   PARAM_TYPE_UINT64=7, /*  .  | */
   PARAM_TYPE_INT64=8, /*   .   | */
   PARAM_TYPE_FLOAT=9, /*   .   | */
   PARAM_TYPE_DOUBLE=10, /*   .   | */
   GENERAL_MSG_VALUE_TYPE_INT_ARRAY=11, /*  .  | */
   GENERAL_MSG_VALUE_TYPE_BYTE_ARRAY=12, /*   .   | */
   GENERAL_MSG_VALUE_TYPE_SHORT_ARRAY=13, /*   .   | */
   GENERAL_MSG_VALUE_TYPE_FLOAT_ARRAY=14, /*   .   | */
   GENERAL_MSG_VALUE_TYPE_DOUBLE_ARRAY=15, /*   .   | */
   GENERAL_MSG_VALUE_TYPE_STRING=16, /*   .   | */
   GENERAL_PARAMS_TYPE_ENUM_END=17, /*  | */
} GENERAL_PARAMS_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_FLIGHT_MODE
#define HAVE_ENUM_FLIGHT_MODE
typedef enum FLIGHT_MODE
{
   FLIGHT_MODE_NO_FLY=1, /*  未飞行.  | */
   FLIGHT_MODE_TAKEOFF=2, /*  起飞模式.  | */
   FLIGHT_MODE_LAND=3, /*  降落模式.  | */
   FLIGHT_MODE_RTH=5, /*  返航模式.  | */
   FLIGHT_MODE_GTH=6, /*  原路返航模式.  | */
   FLIGHT_MODE_RTHL=7, /*  归航模式.  | */
   FLIGHT_MODE_PTF=8, /*  指点模式.  | */
   FLIGHT_MODE_CIRCLE=9, /*  环绕模式.  | */
   FLIGHT_MODE_ATTI_STABLE=10, /*  姿态模式.  | */
   FLIGHT_MODE_ATTI_MANUAL=11, /*  定高模式.  | */
   FLIGHT_MODE_GPS_MANUAL=12, /*  定点模式.  | */
   FLIGHT_MODE_AUTO_PILOT=13, /*  自动航线.  | */
   FLIGHT_MODE_AB_POINT=14, /*  AB点模式.  | */
   FLIGHT_MODE_ENUM_END=15, /*  | */
} FLIGHT_MODE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_FLIGHT_MODE_STATUS
#define HAVE_ENUM_FLIGHT_MODE_STATUS
typedef enum FLIGHT_MODE_STATUS
{
   FLIGHT_STATUS_TAKEOFF_NO_FLY=20, /*  起飞模式未飞行.  | */
   FLIGHT_STATUS_TAKEOFF_FAIL=21, /*  执行起飞失败.  | */
   FLIGHT_STATUS_TAKEOFF_PAUSE=22, /*  暂停起飞.  | */
   FLIGHT_STATUS_TAKEOFF_ING=23, /*  起飞中.  | */
   FLIGHT_STATUS_TAKEOFF_DONE=29, /*  起飞完成.  | */
   FLIGHT_STATUS_LAND_NO_FLY=30, /*  降落模式未飞行.  | */
   FLIGHT_STATUS_LAND_FAIL=31, /*  执行降落失败.  | */
   FLIGHT_STATUS_LAND_PAUSE=32, /*  暂停降落.  | */
   FLIGHT_STATUS_LAND_ING=34, /*  降落中.  | */
   FLIGHT_STATUS_LAND_EMER=38, /*  紧急状态降落中.  | */
   FLIGHT_STATUS_LAND_DONE=39, /*  降落完成.  | */
   FLIGHT_STATUS_RTH_FAIL=51, /*  执行返航失败.  | */
   FLIGHT_STATUS_RTH_PAUSE=52, /*  暂停返航.  | */
   FLIGHT_STATUS_RTH_LAND_ING=54, /*  返航降落中.  | */
   FLIGHT_STATUS_RTH_ING=55, /*  返航中.  | */
   FLIGHT_STATUS_RTH_POINT=57, /*  返航到起飞点.  | */
   FLIGHT_STATUS_RTH_DONE=59, /*  返航完成.  | */
   FLIGHT_STATUS_GTH_FAIL=61, /*  执行原路返航失败.  | */
   FLIGHT_STATUS_GTH_PAUSE=62, /*  暂停原路返航.  | */
   FLIGHT_STATUS_GTH_LAND_ING=64, /*  原路返航降落中.  | */
   FLIGHT_STATUS_GTH_ING=65, /*  原路返航中.  | */
   FLIGHT_STATUS_GTH_POINT=67, /*  原路返航到起飞点.  | */
   FLIGHT_STATUS_GTH_DONE=69, /*  原路返航完成.  | */
   FLIGHT_STATUS_RTHL_FAIL=71, /*  执行归航失败.  | */
   FLIGHT_STATUS_RTHL_PAUSE=72, /*  暂停归航.  | */
   FLIGHT_STATUS_RTHL_LAND_ING=74, /*  归航降落中.  | */
   FLIGHT_STATUS_RTHL_ING=75, /*  归航中.  | */
   FLIGHT_STATUS_RTHL_POINT=77, /*  飞到归航点.  | */
   FLIGHT_STATUS_RTHL_DONE=79, /*  归航完成.  | */
   FLIGHT_STATUS_PTF_NO_FLY=80, /*  指点未飞行.  | */
   FLIGHT_STATUS_PTF_FAIL=81, /*  执行指点飞行失败.  | */
   FLIGHT_STATUS_PTF_PAUSE=82, /*  暂停指点飞行.  | */
   FLIGHT_STATUS_PTF_TAKE_OFF=83, /*  指点飞行起飞中.  | */
   FLIGHT_STATUS_PTF_LAND_ING=84, /*  指点飞行降落中.  | */
   FLIGHT_STATUS_PTF_ING=85, /*  指点飞行中.  | */
   FLIGHT_STATUS_PTF_HOVER=86, /*  悬停过程.  | */
   FLIGHT_STATUS_PTF_POINT=87, /*  飞到指定点.  | */
   FLIGHT_STATUS_PTF_ACTIVE=88, /*  执行预设动作.  | */
   FLIGHT_STATUS_PTF_DONE=89, /*  指点飞行完成.  | */
   FLIGHT_STATUS_CIRCLE_NO_FLY=90, /*  环绕未飞行.  | */
   FLIGHT_STATUS_CIRCLE_FAIL=91, /*  执行环绕飞行失败.  | */
   FLIGHT_STATUS_CIRCLE_PAUSE=92, /*  暂停环绕飞行.  | */
   FLIGHT_STATUS_CIRCLE_TAKE_OFF=93, /*  环绕飞行起飞中.  | */
   FLIGHT_STATUS_CIRCLE_LAND_ING=94, /*  环绕飞行降落中.  | */
   FLIGHT_STATUS_CIRCLE_ING=95, /*  环绕飞行中.  | */
   FLIGHT_STATUS_CIRCLE_HOVER=96, /*  悬停过程.  | */
   FLIGHT_STATUS_CIRCLE_POINT=97, /*  飞到环绕点.  | */
   FLIGHT_STATUS_CIRCLE_ACTIVE=98, /*  执行预设动作.  | */
   FLIGHT_STATUS_CIRCLE_DONE=99, /*  环绕飞行完成.  | */
   FLIGHT_STATUS_ATTI_STABLE_NO_FLY=100, /*  增稳模式未飞行.  | */
   FLIGHT_STATUS_ATTI_STABLE_UNLOCK_FAIL=101, /*  增稳模式失败.  | */
   FLIGHT_STATUS_ATTI_STABLE_RC=105, /*  增稳模式遥控器控制.  | */
   FLIGHT_STATUS_ATTI_MANUAL_NO_FLY=110, /*  姿态模式未飞行.  | */
   FLIGHT_STATUS_ATTI_MANUAL_UNLOCK_FAIL=111, /*  姿态模式失败.  | */
   FLIGHT_STATUS_ATTI_MANUAL_RC=115, /*  姿态模式遥控器控制.  | */
   FLIGHT_STATUS_GPS_MANUAL_NO_FLY=120, /*  定点模式未飞行.  | */
   FLIGHT_STATUS_GPS_MANUAL_UNLOCK_FAIL=121, /*  定点模式失败.  | */
   FLIGHT_STATUS_GPS_MANUAL_RC=125, /*  定点模式遥控器控制.  | */
   FLIGHT_STATUS_GPS_MANUAL_POS=127, /*  定点悬停.  | */
   FLIGHT_STATUS_GPS_MANUAL_WP=129, /*  航线临时切换定点.  | */
   FLIGHT_STATUS_AUTO_PILOT_NO_FLY=130, /*  自动航线未飞行.  | */
   FLIGHT_STATUS_AUTO_PILOT_FAIL=131, /*  执行自动航线失败.  | */
   FLIGHT_STATUS_AUTO_PILOT_PAUSE=132, /*  暂停自动航线.  | */
   FLIGHT_STATUS_AUTO_PILOT_TAKE_OFF=133, /*  自动航线起飞中.  | */
   FLIGHT_STATUS_AUTO_PILOT_LAND_ING=134, /*  自动航线降落中.  | */
   FLIGHT_STATUS_AUTO_PILOT_ING=135, /*  自动航线中.  | */
   FLIGHT_STATUS_AUTO_PILOT_HOVER=136, /*  悬停过程.  | */
   FLIGHT_STATUS_AUTO_PILOT_POINT=137, /*  飞到航点.  | */
   FLIGHT_STATUS_AUTO_PILOT_ACTIVE=138, /*  执行预设动作.  | */
   FLIGHT_STATUS_AUTO_PILOT_DONE=139, /*  自动航线完成.  | */
   FLIGHT_STATUS_ABPOINT_NO_FLY=140, /*  执行AB点航线未飞行.  | */
   FLIGHT_STATUS_AB_POINT_FAIL=141, /*  执行AB点航线失败.  | */
   FLIGHT_STATUS_AB_POINT_PAUSE=142, /*  暂停AB点航线航线.  | */
   FLIGHT_STATUS_AB_POINT_A_B=143, /*  AB点航线:A->B.  | */
   FLIGHT_STATUS_AB_POINT_B_B=144, /*  AB点航线:B->B.  | */
   FLIGHT_STATUS_AB_POINT_B_A=145, /*  AB点航线:B->A.  | */
   FLIGHT_STATUS_AB_POINT_A_A=146, /*  AB点航线:A->A.  | */
   FLIGHT_STATUS_AB_POINT_MODIFY=147, /*  AB点航线变更.  | */
   FLIGHT_STATUS_AB_POINTOASTACLE_AVOID=148, /*  AB点航线手动避障.  | */
   FLIGHT_STATUS_AB_POINT_ACTION_BREAK=149, /*  AB点航线执行断点.  | */
   FLIGHT_MODE_STATUS_ENUM_END=150, /*  | */
} FLIGHT_MODE_STATUS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_CONTROL_RIGHT_STATUS
#define HAVE_ENUM_CONTROL_RIGHT_STATUS
typedef enum CONTROL_RIGHT_STATUS
{
   CONTROL_STATUS_RC=100, /*  遥控器模式.  | */
   CONTROL_STATUS_CLOUD=101, /*  云控模式.  | */
   CONTROL_STATUS_CLOUD_FAIL=102, /*  云控模式失败.  | */
   CONTROL_RIGHT_STATUS_ENUM_END=103, /*  | */
} CONTROL_RIGHT_STATUS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_COM_SHELL_MODE
#define HAVE_ENUM_COM_SHELL_MODE
typedef enum COM_SHELL_MODE
{
   COM_DEBUG_UART_MODE=2, /*  .  | */
   COM_DEBUG_GCS_MODE=3, /*  .  | */
   COM_SHELL_MODE_ENUM_END=4, /*  | */
} COM_SHELL_MODE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_AUTO_ADJUST_TYPE
#define HAVE_ENUM_AUTO_ADJUST_TYPE
typedef enum AUTO_ADJUST_TYPE
{
   AUTO_ADJUST_SPEED=1, /*  .  | */
   AUTO_ADJUST_HEIGHT=2, /*  .  | */
   AUTO_ADJUST_TYPE_ENUM_END=3, /*  | */
} AUTO_ADJUST_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_AUTO_ADJUST_ACK
#define HAVE_ENUM_AUTO_ADJUST_ACK
typedef enum AUTO_ADJUST_ACK
{
   AUTO_ADJUST_READY=0, /*  .  | */
   AUTO_ADJUST_NO_READY=1, /*  .  | */
   AUTO_ADJUST_ACK_ENUM_END=2, /*  | */
} AUTO_ADJUST_ACK;
#endif

/** @brief  */
#ifndef HAVE_ENUM_ITEM_CMD_TYPE
#define HAVE_ENUM_ITEM_CMD_TYPE
typedef enum ITEM_CMD_TYPE
{
   TTALINK_ITEM_CMD_ADD=1, /*  增加航点，.  | */
   TTALINK_ITEM_CMD_REPLACE=2, /*  替代航点.  | */
   TTALINK_ITEM_CMD_DEL=3, /*  删除航点.  | */
   TTALINK_ITEM_CMD_CLEAR=4, /*  清空航点.  | */
   TTALINK_ITEM_CMD_ADD_AB=5, /*  增加AB航点.  | */
   TTALINK_ITEM_CMD_UPDEATE_WP=6, /*  上传普通航线.  | */
   TTALINK_ITEM_CMD_UPDEATE_AB=7, /*  上传AB点航线.  | */
   TTALINK_MISSION_ITEM=10, /*  .  | */
   ITEM_CMD_TYPE_ENUM_END=11, /*  | */
} ITEM_CMD_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_CALIBRATION_TYPE
#define HAVE_ENUM_CALIBRATION_TYPE
typedef enum CALIBRATION_TYPE
{
   TTALINK_CALIBRATION_TYPE_ACC_VECTOR=1, /*  .  | */
   TTALINK_CALIBRATION_TYPE_HORIZONTAL=2, /*  .  | */
   TTALINK_CALIBRATION_TYPE_VERTICAL=3, /*  .  | */
   TTALINK_CALIBRATION_TYPE_MAG=4, /*  .  | */
   TTALINK_CALIBRATION_TYPE_REMOTE_CONTROL=5, /*  .  | */
   TTALINK_CALIBRATION_TYPE_INSTALLATION_ERROR=6, /*  .  | */
   TTALINK_CALIBRATION_TYPE_STOP=7, /*  .  | */
   CALIBRATION_TYPE_ENUM_END=8, /*  | */
} CALIBRATION_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_CALIBRATION_COM_ACK_TYPE
#define HAVE_ENUM_CALIBRATION_COM_ACK_TYPE
typedef enum CALIBRATION_COM_ACK_TYPE
{
   TTALINK_CALIBRATION_NORMAL_START=100, /*  正常开始.  | */
   TTALINK_CALIBRATION_START_ERROR=110, /*  马达开启，不能进入校准.  | */
   TTALINK_CALIBRATION_BUSY_ERROR=111, /*  已在校准状态，请先停止.  | */
   TTALINK_CALIBRATION_PARAM_ERROR=112, /*  校准类型错误.  | */
   TTALINK_CALIBRATION_FAULT_ERROR_STOP=113, /*  异常错误，中断校准.  | */
   TTALINK_CALIBRATION_MANUAL_STOP_SUCCESS=114, /*  主动停止校准成功.  | */
   CALIBRATION_COM_ACK_TYPE_ENUM_END=115, /*  | */
} CALIBRATION_COM_ACK_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_CALIBRATION_STOP_ACK_TYPE
#define HAVE_ENUM_CALIBRATION_STOP_ACK_TYPE
typedef enum CALIBRATION_STOP_ACK_TYPE
{
   CALIBRATION_STOP_SUCCESS=0, /*  .  | */
   CALIBRATION_STOP_ERROR=1, /*  .  | */
   CALIBRATION_STOP_ACK_TYPE_ENUM_END=2, /*  | */
} CALIBRATION_STOP_ACK_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_CALIBRATION_ACC_VECTOR_ACK_TYPE
#define HAVE_ENUM_CALIBRATION_ACC_VECTOR_ACK_TYPE
typedef enum CALIBRATION_ACC_VECTOR_ACK_TYPE
{
   CALIBRATION_ACC_VECTOR_ACK_TYPE_SUCCESS=2, /*  | */
   CALIBRATION_ACC_VECTOR_ACK_TYPE_SAVE_DATA_ERROR=3, /* Save data error. | */
   CALIBRATION_ACC_VECTOR_ACK_TYPE_LARGE_ERROR=4, /* 零偏过大,请重新校准. | */
   CALIBRATION_ACC_VECTOR_ACK_TYPE_TIMEOUT_ERROR=10, /* 加速度计校准超时. | */
   CALIBRATION_ACC_VECTOR_ACK_TYPE_BEHIND=11, /* 后面校验完成. | */
   CALIBRATION_ACC_VECTOR_ACK_TYPE_FRONT=12, /* 前面校验完成.. | */
   CALIBRATION_ACC_VECTOR_ACK_TYPE_LEFT=13, /* 左面校验完成. | */
   CALIBRATION_ACC_VECTOR_ACK_TYPE_RIGHT=14, /* 右面校验完成.. | */
   CALIBRATION_ACC_VECTOR_ACK_TYPE_BACK=15, /* 反面校验完成. | */
   CALIBRATION_ACC_VECTOR_ACK_TYPE_OBVERSE=16, /* 正面校验完成. | */
   CALIBRATION_ACC_VECTOR_ACK_TYPE_ENUM_END=17, /*  | */
} CALIBRATION_ACC_VECTOR_ACK_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_CALIBRATION_HORIZONTAL_ACK_TYPE
#define HAVE_ENUM_CALIBRATION_HORIZONTAL_ACK_TYPE
typedef enum CALIBRATION_HORIZONTAL_ACK_TYPE
{
   CALIBRATION_HORIZONTAL_ACK_TYPE_SUCCESS=2, /*  | */
   CALIBRATION_HORIZONTAL_ACK_TYPE_SAVE_DATA_ERROR=3, /* Save data error. | */
   CALIBRATION_HORIZONTAL_ACK_TYPE_LARGE_ERROR=4, /* 零偏过大,请重新校准. | */
   CALIBRATION_HORIZONTAL_ACK_TYPE_TIMEOUT_ERROR=10, /* 水平校准超时. | */
   CALIBRATION_HORIZONTAL_ACK_TYPE_PROGRESS=11, /* 校准中 | */
   CALIBRATION_HORIZONTAL_ACK_TYPE_ENUM_END=12, /*  | */
} CALIBRATION_HORIZONTAL_ACK_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_CALIBRATION_VERTICAL_ACK_TYPE
#define HAVE_ENUM_CALIBRATION_VERTICAL_ACK_TYPE
typedef enum CALIBRATION_VERTICAL_ACK_TYPE
{
   CALIBRATION_VERTICAL_ACK_TYPE_SUCCESS=2, /*  | */
   CALIBRATION_VERTICAL_ACK_TYPE_SAVE_DATA_ERROR=3, /* Save data error. | */
   CALIBRATION_VERTICAL_ACK_TYPE_LARGE_ERROR=4, /* 零偏过大,请重新校准. | */
   CALIBRATION_VERTICAL_ACK_TYPE_TIMEOUT_ERROR=10, /* 垂直校准超时. | */
   CALIBRATION_VERTICAL_ACK_TYPE_PROGRESS=11, /* 校准中 | */
   CALIBRATION_VERTICAL_ACK_TYPE_ENUM_END=12, /*  | */
} CALIBRATION_VERTICAL_ACK_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_CALIBRATION_MAG_ACK_TYPE
#define HAVE_ENUM_CALIBRATION_MAG_ACK_TYPE
typedef enum CALIBRATION_MAG_ACK_TYPE
{
   CALIBRATION_MAG_ACK_TYPE_SUCCESS=2, /*  | */
   CALIBRATION_MAG_ACK_TYPE_SAVE_DATA_ERROR=3, /* Save data error. | */
   CALIBRATION_MAG_ACK_TYPE_LARGE_ERROR=4, /* 零偏过大,请重新校准. | */
   CALIBRATION_MAG_ACK_TYPE_MAG_UPDATE_ERROR=5, /* 双罗盘数据更新异常. | */
   CALIBRATION_MAG_ACK_TYPE_CALCULATE_ERROR=6, /* 罗盘校准计算错误. | */
   CALIBRATION_MAG_ACK_TYPE_MAG1_UPDATE_ERROR=7, /* 罗盘1数据更新异常. | */
   CALIBRATION_MAG_ACK_TYPE_MAG2_UPDATE_ERROR=8, /* 罗盘2数据更新异常. | */
   CALIBRATION_MAG_ACK_TYPE_ATTI_UPDATE_ERROR=9, /* 姿态更新异常. | */
   CALIBRATION_MAG_ACK_TYPE_TIMEOUT_ERROR=10, /* 罗盘校准超时. | */
   CALIBRATION_MAG_ACK_TYPE_Z=11, /* 开始罗盘Z轴校准. | */
   CALIBRATION_MAG_ACK_TYPE_Y=12, /* 开始罗盘Y轴校准. | */
   CALIBRATION_MAG_ACK_TYPE_X=13, /* 开始罗盘X轴校准. | */
   CALIBRATION_MAG_ACK_TYPE_COLLECT_COMPLETE=14, /* The mag collect complete. | */
   CALIBRATION_MAG_ACK_TYPE_ENUM_END=15, /*  | */
} CALIBRATION_MAG_ACK_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_CALIBRATION_REMOTE_CONTROL_ACK_TYPE
#define HAVE_ENUM_CALIBRATION_REMOTE_CONTROL_ACK_TYPE
typedef enum CALIBRATION_REMOTE_CONTROL_ACK_TYPE
{
   CALIBRATION_REMOTE_CONTROL_ACK_TYPE_SUCCESS=2, /*  | */
   CALIBRATION_REMOTE_CONTROL_ACK_TYPE_SAVE_DATA_ERROR=3, /* Save data error. | */
   CALIBRATION_REMOTE_CONTROL_ACK_TYPE_DATA_ERROR=4, /* Remote control data error. | */
   CALIBRATION_REMOTE_CONTROL_ACK_TYPE_TIMEOUT_ERROR=10, /* 遥控器校准超时. | */
   CALIBRATION_REMOTE_ACK_TYPE_ROCKER=11, /* 正在采样摇杆行程最大值,请旋转遥控器2个摇杆. | */
   CALIBRATION_REMOTE_ACK_TYPE_MEDIAN=12, /* 正在采样摇杆中点,请把所有摇杆放回中位. | */
   CALIBRATION_REMOTE_CONTROL_ACK_TYPE_ENUM_END=13, /*  | */
} CALIBRATION_REMOTE_CONTROL_ACK_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_CALIBRATION_INSTALLATION_ERROR_ACK_TYPE
#define HAVE_ENUM_CALIBRATION_INSTALLATION_ERROR_ACK_TYPE
typedef enum CALIBRATION_INSTALLATION_ERROR_ACK_TYPE
{
   CALIBRATION_INSTALLATION_ERROR_ACK_TYPE_SUCCESS=2, /*  | */
   CALIBRATION_INSTALLATION_ACK_TYPE_SAVE_DATA_ERROR=3, /* Save data error. | */
   CALIBRATION_INSTALLATION_ACK_TYPE_LARGE_ERROR=4, /* 零偏过大,请重新校准. | */
   CALIBRATION_INSTALLATION_ERROR_ACK_TYPE_X=11, /* The +X axial collect complete. | */
   CALIBRATION_INSTALLATION_ERROR_ACK_TYPE_X_=12, /* The -X axial collect complete. | */
   CALIBRATION_INSTALLATION_ERROR_ACK_TYPE_Y=13, /* The +Y axial collect complete. | */
   CALIBRATION_INSTALLATION_ERROR_ACK_TYPE_Y_=14, /* The -Y axial collect complete. | */
   CALIBRATION_INSTALLATION_ERROR_ACK_TYPE_Z=15, /* The +Z axial collect complete. | */
   CALIBRATION_INSTALLATION_ERROR_ACK_TYPE_Z_=16, /* The -Z axial collect complete. | */
   CALIBRATION_INSTALLATION_ERROR_ACK_TYPE_ENUM_END=17, /*  | */
} CALIBRATION_INSTALLATION_ERROR_ACK_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_TTALINK_FTP_ACK_TYPE
#define HAVE_ENUM_TTALINK_FTP_ACK_TYPE
typedef enum TTALINK_FTP_ACK_TYPE
{
   TTALINK_FTP_ACK_OK=0, /*  .  | */
   TTALINK_FTP_ACK_DISK_ERR=1, /*  .  | */
   TTALINK_FTP_ACK_INT_ERR=2, /*  .  | */
   TTALINK_FTP_ACK_NOT_READY=3, /*  .  | */
   TTALINK_FTP_ACK_NO_FILE=4, /*  .  | */
   TTALINK_FTP_ACK_NO_PATH=5, /*  .  | */
   TTALINK_FTP_ACK_INVALID_NAME=6, /*  .  | */
   TTALINK_FTP_ACK_DENIED=7, /*  .  | */
   TTALINK_FTP_ACK_EXIST=8, /*  .  | */
   TTALINK_FTP_ACK_INVALID_OBJECT=9, /*  .  | */
   TTALINK_FTP_ACK_WRITE_PROTECTED=10, /*  .  | */
   TTALINK_FTP_ACK_INVALID_DRIVE=11, /*  .  | */
   TTALINK_FTP_ACK_NOT_ENABLED=12, /*  .  | */
   TTALINK_FTP_ACK_NO_FILESYSTEM=13, /*  .  | */
   TTALINK_FTP_ACK_MKFS_ABORTED=14, /*  .  | */
   TTALINK_FTP_ACK_TIMEOUT=15, /*  .  | */
   TTALINK_FTP_ACK_LOCKED=16, /*  .  | */
   TTALINK_FTP_ACK_NOT_ENOUGH_CORE=17, /*  .  | */
   TTALINK_FTP_ACK_TOO_MANY_OPEN_FILES=18, /*  .  | */
   TTALINK_FTP_ACK_INVALID_PARAMETER=19, /*  .  | */
   TTALINK_FTP_ACK_NOT_OPEN=20, /*  .  | */
   TTALINK_FTP_ACK_MISS_PACKET=21, /*  .  | */
   TTALINK_FTP_ACK_WRITE_FINISH=22, /*  .  | */
   TTALINK_FTP_ACK_ERR=255, /*  .  | */
   TTALINK_FTP_ACK_TYPE_ENUM_END=256, /*  | */
} TTALINK_FTP_ACK_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_TTALINK_FTP_TYPE
#define HAVE_ENUM_TTALINK_FTP_TYPE
typedef enum TTALINK_FTP_TYPE
{
   TTALINK_FTP_FILE_MOUNT=0, /*  .  | */
   TTALINK_FTP_FILE_OPEN=1, /*  .  | */
   TTALINK_FTP_FILE_CLOSE=2, /*  .  | */
   TTALINK_FTP_FILE_READ=3, /*  .  | */
   TTALINK_FTP_FILE_WRITE=4, /*  .  | */
   TTALINK_FTP_FILE_LSEEK=5, /*  .  | */
   TTALINK_FTP_FILE_LIST_DIR=6, /*  .  | */
   TTALINK_FTP_REMOVE_DIR=7, /*  .  | */
   TTALINK_FTP_REMOVE_FILE=8, /*  .  | */
   TTALINK_FTP_FILE_CLOSE_DIR=9, /*  .  | */
   TTALINK_FTP_FILE_CREATE_FILE=10, /*  .  | */
   TTALINK_FTP_FILE_RENAME_FILE=11, /*  .  | */
   TTALINK_FTP_FILE_REPLACE_FILE=12, /*  .  | */
   TTALINK_FTP_FILE_UNMOUNT=13, /*  .  | */
   TTALINK_FTP_FILE_MD5_CALI=14, /*  .  | */
   TTALINK_FTP_FILE_OPENDIR=15, /*  .  | */
   TTALINK_FTP_FILE_STOP=16, /*  .  | */
   TTALINK_FTP_TYPE_ENUM_END=17, /*  | */
} TTALINK_FTP_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_SENSOR_DEV_INDEX
#define HAVE_ENUM_SENSOR_DEV_INDEX
typedef enum SENSOR_DEV_INDEX
{
   SENSORS_IMU1=1, /*  .  | */
   SENSORS_IMU2=2, /*  .  | */
   SENSORS_IMU3=3, /*  .  | */
   SENSOR_VISION=10, /*  .  | */
   SENSOR_RADAR=11, /*  .  | */
   SENSOR_BARO1=12, /*  .  | */
   SENSOR_BARO2=13, /*  .  | */
   SENSOR_MAG1=14, /*  .  | */
   SENSOR_MAG2=15, /*  .  | */
   SENSOR_GPS1=16, /*  .  | */
   SENSOR_GPS2=17, /*  .  | */
   SENSOR_RTK=18, /*  .  | */
   SENSOR_DEV_INDEX_ENUM_END=19, /*  | */
} SENSOR_DEV_INDEX;
#endif

/** @brief  */
#ifndef HAVE_ENUM_TTALINK_ITEM_MODE
#define HAVE_ENUM_TTALINK_ITEM_MODE
typedef enum TTALINK_ITEM_MODE
{
   ITEM_MODE_CYCLE_TIME=1, /*  .  | */
   ITEM_MODE_TAKEOFF=2, /*  .  | */
   ITEM_MODE_LAND=3, /*  .  | */
   ITEM_MODE_TIME=4, /*  .  | */
   ITEM_MODE_HOVER=5, /*  .  | */
   ITEM_MODE_ANCHOR=6, /*  .  | */
   ITEM_MODE_LOCATION=7, /*  .  | */
   ITEM_MODE_CHECK=8, /*  .  | */
   ITEM_MODE_RETURN=9, /*  .  | */
   TTALINK_ITEM_MODE_PLAY_ROUTE=13, /*  .  | */
   TTALINK_ITEM_MODE_WAITING=14, /*  .  | */
   ITEM_MODE_FLY_POINT=20, /*  .  | */
   ITEM_MODE_RETURN_LAND=21, /*  .  | */
   ITEM_MODE_CYCLE=22, /*  .  | */
   TTALINK_ITEM_MODE_ENUM_END=23, /*  | */
} TTALINK_ITEM_MODE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_TTALINK_MSG_TYPE
#define HAVE_ENUM_TTALINK_MSG_TYPE
typedef enum TTALINK_MSG_TYPE
{
   TTALINK_MSG_TYPE_SATRT=0, /*  .  | */
   TTALINK_MSG_ACK=1, /*  .  | */
   TTALINK_MSG_REQUEST=2, /*   .   | */
   TTALINK_MSG_MESSAGE=3, /*   .   | */
   TTALINK_MSG_TYPE_ENUM_END=4, /*  | */
} TTALINK_MSG_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_TTALINK_PASSTHROUGH_TYPE
#define HAVE_ENUM_TTALINK_PASSTHROUGH_TYPE
typedef enum TTALINK_PASSTHROUGH_TYPE
{
   PASS_DOWNLOAD=1, /*   .   | */
   PASS_RTK=2, /*   .  | */
   PASS_BAT=3, /*   .   | */
   PASS_COMSHELL=4, /*  .  | */
   PASS_TEST=5, /*   .   | */
   TTALINK_PASSTHROUGH_TYPE_ENUM_END=6, /*  | */
} TTALINK_PASSTHROUGH_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_TTALINK_VERSION_MODULE_TYPE
#define HAVE_ENUM_TTALINK_VERSION_MODULE_TYPE
typedef enum TTALINK_VERSION_MODULE_TYPE
{
   TTALINK_MODULE_ALL=0, /*  .  | */
   TTALINK_MODULE_INDEX_ACK=1, /*  .  | */
   TTALINK_MODULE_FC=10, /* 飞控升级 .  | */
   TTALINK_MODULE_SH=11, /*  传感器板.  | */
   TTALINK_MODULE_TC=12, /*  协议转发板卡.  | */
   TTALINK_MODULE_EMBE=13, /*  嵌入式板卡.  | */
   TTALINK_MODULE_BMS=14, /*  智能电池版本.  | */
   TTALINK_MODULE_FOC=15, /*  FOC版本.  | */
   TTALINK_MODULE_BASE_RTK=16, /*  base-rtk. | */
   TTALINK_MODULE_NANO=17, /*  nano-pi. | */
   TTALINK_MODULE_HISILICON=18, /*  HI3559-A. | */
   TTALINK_MODULE_UM482_RTK=19, /*  RTK-UM482模块升级. | */
   TTALINK_MODULE_SPREAD=20, /*  播撒机升级. | */
   TTALINK_VERSION_MODULE_TYPE_ENUM_END=21, /*  | */
} TTALINK_VERSION_MODULE_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_STEAM_TYPE
#define HAVE_ENUM_STEAM_TYPE
typedef enum STEAM_TYPE
{
   POS_LOCATION=10, /*    | */
   PWM_OUTPUT=11, /*    | */
   ITEM_STATE=12, /*    | */
   ITEM_INFO=13, /*    | */
   EXTERN_MESSAGE=14, /*    | */
   EXTERN_MESSAGE2=15, /*    | */
   EXTERN_MESSAGE3=16, /*    | */
   STREAM_BARO=17, /*    | */
   STREAM_IMU=18, /*    | */
   STREAM_MAG=19, /*    | */
   STREAM_DRONE_STATE=20, /*    | */
   STREAM_DRONE_SWITCH=21, /*    | */
   STEAM_TYPE_ENUM_END=22, /*  | */
} STEAM_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_UPLOAD_IMG_ACK_TYPE
#define HAVE_ENUM_UPLOAD_IMG_ACK_TYPE
typedef enum UPLOAD_IMG_ACK_TYPE
{
   UPLOAD_IMG_PROCESS=0, /*  .  | */
   UPLOAD_IMG_OK=1, /*  .  | */
   UPLOAD_IMG_TIMEOUT=20, /*  .  | */
   UPLOAD_IMG_FINISH=21, /*  .  | */
   UPLOAD_IMG_MISS_COUNT=22, /*  .  | */
   UPLOAD_IMG_ERROR_TYPE=23, /*  .  | */
   UPLOAD_IMG_ERROR_COUNT=24, /*  .  | */
   UPLOAD_IMG_OPEN_FILE_ERROR=25, /*  .  | */
   UPLOAD_IMG_WRITE_FILE_ERROR=26, /*  .  | */
   UPLOAD_IMG_ERROR_LOADING=27, /*  .  | */
   UPLOAD_IMG_ERROR_WRITING=28, /*  .  | */
   UPLOAD_IMG_ACK_TYPE_ENUM_END=29, /*  | */
} UPLOAD_IMG_ACK_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_VIDEO_STREAM
#define HAVE_ENUM_VIDEO_STREAM
typedef enum VIDEO_STREAM
{
   CLOSE_ALL_STREAM=0, /*  .  | */
   CLOSE_VIDEO_STREAM1=1, /*  .  | */
   CLOSE_VIDEO_STREAM2=2, /*  .  | */
   OPEN_VIDEO_STREAM1=3, /*  .  | */
   OPEN_VIDEO_STREAM2=4, /*  .  | */
   VIDEO_STREAM1_HB=5, /*  .  | */
   VIDEO_STREAM2_HB=6, /*  .  | */
   VIDEO_DEV_STATUS=7, /*  .  | */
   VIDEO_STREAMS_STATUS=8, /*  .  | */
   VIDEO_STREAM_ENUM_END=9, /*  | */
} VIDEO_STREAM;
#endif

/** @brief  */
#ifndef HAVE_ENUM_STREAM_GPS_TYPE
#define HAVE_ENUM_STREAM_GPS_TYPE
typedef enum STREAM_GPS_TYPE
{
   STREAM_GPS1=1, /*  .  | */
   STREAM_GPS2=2, /*  .  | */
   STREAM_RTK=3, /*  .  | */
   STREAM_GPS_TYPE_ENUM_END=4, /*  | */
} STREAM_GPS_TYPE;
#endif

/** @brief  Just powered on . / Offset calibration . / Throttle self test. / Electric commissioning preparation status, can receive speed torque command    . / Motor parameter identification  . / Speed tracking, only in this state when it is just started  . / Normal operation status . / Shutdown command received, in process of shutdown. / Fault status. / Idle state. / . / */
#ifndef HAVE_ENUM_ESC_FOC_STATE
#define HAVE_ENUM_ESC_FOC_STATE
typedef enum ESC_FOC_STATE
{
   ST_FOC_STATE_IDLE=0, /*  | */
   ST_FOC_STATE_IDLE_ALIGNMENT=1, /*  | */
   ST_FOC_STATE_ALIGNMENT=2, /*  | */
   ST_FOC_STATE_IDLE_START=3, /*  | */
   ST_FOC_STATE_START=4, /*  | */
   ST_FOC_STATE_START_RUN=5, /*  | */
   ST_FOC_STATE_RUN=6, /*  | */
   ST_FOC_STATE_ANY_STOP=7, /*  | */
   ST_FOC_STATE_STOP=8, /*  | */
   ST_FOC_STATE_STOP_IDLE=9, /*  | */
   ST_FOC_STATE_FAULT_NOW=10, /*  | */
   ST_FOC_STATE_FAULT_OVER=11, /*  | */
   ST_FOC_STATE_ICLWAIT=12, /*  | */
   ST_FOC_STATE_ALIGN_CHARGE_BOOT_CAP=13, /*  | */
   ST_FOC_STATE_ALIGN_OFFSET_CALIB=14, /*  | */
   ST_FOC_STATE_ALIGN_CLEAR=15, /*  | */
   ST_FOC_STATE_CHARGE_BOOT_CAP=16, /*  | */
   ST_FOC_STATE_OFFSET_CALIB=17, /*  | */
   ST_FOC_STATE_CLEAR=18, /*  | */
   TI_CTRL_Status_PowerOn=19, /*  | */
   TI_CTRL_Status_OffsetCal=20, /*  | */
   TI_CTRL_Status_SelfCheck=21, /*  | */
   TI_CTRL_Status_Ready=22, /*  | */
   TI_CTRL_Status_Identing=23, /*  | */
   TI_CTRL_Status_SpeedTrack=24, /*  | */
   TI_CTRL_Status_Running=25, /*  | */
   TI_CTRL_Status_Stopping=26, /*  | */
   TI_CTRL_Status_Error=27, /*  | */
   TI_CTRL_Status_Idle=28, /*  | */
   TI_CTRL_Status_NULL=29, /*  | */
   ESC_FOC_STATE_ENUM_END=30, /*  | */
} ESC_FOC_STATE;
#endif

/** @brief No faultFOC calculation time outBus overvoltageBus undervoltageOver temperatureStart up failureSpeed feedback is notreliableHardware break inSoftware fault */
#ifndef HAVE_ENUM_ESC_FOC_ERROR_CODE
#define HAVE_ENUM_ESC_FOC_ERROR_CODE
typedef enum ESC_FOC_ERROR_CODE
{
   ST_FOC_NO_ERROR=0, /*  | */
   ST_FOC_FOC_DURATION=1, /*  | */
   ST_FOC_OVER_VOLT=2, /*  | */
   ST_FOC_NDER_VOLT=4, /*  | */
   ST_FOC_OVER_TEMP=8, /*  | */
   ST_FOC_START_UP=16, /*  | */
   ST_FOC_SPEED_FDBK=32, /*  | */
   ST_FOC_BREAK_IN=64, /*  | */
   ST_FOC_SW_ERROR=128, /*  | */
   ESC_FOC_ERROR_CODE_ENUM_END=129, /*  | */
} ESC_FOC_ERROR_CODE;
#endif

/** @brief No commandstart motorstop motorspeed ctrltorque ctrlhybrid controlvoltage closed-loop controlvoltage open-loop controlclean up mistakes */
#ifndef HAVE_ENUM_ESC_FOC_CMD
#define HAVE_ENUM_ESC_FOC_CMD
typedef enum ESC_FOC_CMD
{
   ESC_FOC_CMD_NULL=0, /*  | */
   ESC_FOC_START_MOTOR=1, /*  | */
   ESC_FOC_STOP_MOTOR=2, /*  | */
   ESC_FOC_SPEED_CTRL=3, /*  | */
   ESC_FOC_TORQUE=4, /*  | */
   ESC_FOC_MEXED=5, /*  | */
   ESC_FOC_VSMAG_CLOSE=6, /*  | */
   ESC_FOC_VSMAG_LINE=7, /*  | */
   ESC_FOC_FAULT_ACK=8, /*  | */
   ESC_FOC_CMD_ENUM_END=9, /*  | */
} ESC_FOC_CMD;
#endif

/** @brief Speed loop kpSpeed loop default kpSpeed loop kiSpeed loop default kiSpeed loop kdSpeed loop default kdIt set a new value for speed loop lower integral term limitIt set a new value for speed loop upper integral term limitIt set a new value for speed loop lower output limitIt set a new value for speed loop upper output limitIq loop kpIq loop default kpIq loop kiIq loop default kiIq loop kdIq loop default kdIt set a new value for Iq loop lower integral term limitIt set a new value for Iq loop upper integral term limitIt set a new value for Iq loop lower output limitIt set a new value for Iq loop upper output limitId loop kpId loop default kpId loop kiId loop default kiId loop kdId loop default kdIt set a new value for Id loop lower integral term limitIt set a new value for Id loop upper integral term limitIt set a new value for Id loop lower output limitIt set a new value for Id loop upper output limitFluxWeakening loop kpFluxWeakening loop default kpFluxWeakening loop kiFluxWeakening loop default kiFluxWeakening loop kdFluxWeakening loop default kdIt set a new value for FluxWeakening loop lower integral term limitIt set a new value for FluxWeakening loop upper integral term limitIt set a new value for FluxWeakening loop lower output limitIt set a new value for FluxWeakening loop upper output limit........ */
#ifndef HAVE_ENUM_ESC_FOC_PARA
#define HAVE_ENUM_ESC_FOC_PARA
typedef enum ESC_FOC_PARA
{
   ST_FOC_SPEED_KP=0, /*  | */
   ST_FOC_SPEED_DEFAULT_KP=1, /*  | */
   ST_FOC_SPEED_KI=2, /*  | */
   ST_FOC_SPEED_DEFAULT_KI=3, /*  | */
   ST_FOC_SPEED_KD=4, /*  | */
   ST_FOC_SPEED_DEFAULT_KD=5, /*  | */
   ST_FOC_SPEED_LOWER_INTEGRAL_TERM_LIMIT=6, /*  | */
   ST_FOC_SPEED_UPPER_INTEGRAL_TERM_LIMIT=7, /*  | */
   ST_FOC_SPEED_LOWER_OUTPUT_LIMIT=8, /*  | */
   ST_FOC_SPEED_UPPER_OUTPUT_LIMIT=9, /*  | */
   ST_FOC_Iq_KP=10, /*  | */
   ST_FOC_Iq_DEFAULT_KP=11, /*  | */
   ST_FOC_Iq_KI=12, /*  | */
   ST_FOC_Iq_DEFAULT_KI=13, /*  | */
   ST_FOC_Iq_KD=14, /*  | */
   ST_FOC_Iq_DEFAULT_KD=15, /*  | */
   ST_FOC_Iq_LOWER_INTEGRAL_TERM_LIMIT=16, /*  | */
   ST_FOC_Iq_UPPER_INTEGRAL_TERM_LIMIT=17, /*  | */
   ST_FOC_Iq_LOWER_OUTPUT_LIMIT=18, /*  | */
   ST_FOC_Iq_UPPER_OUTPUT_LIMIT=19, /*  | */
   ST_FOC_Id_KP=20, /*  | */
   ST_FOC_Id_DEFAULT_KP=21, /*  | */
   ST_FOC_Id_KI=22, /*  | */
   ST_FOC_Id_DEFAULT_KI=23, /*  | */
   ST_FOC_Id_KD=24, /*  | */
   ST_FOC_Id_DEFAULT_KD=25, /*  | */
   ST_FOC_Id_LOWER_INTEGRAL_TERM_LIMIT=26, /*  | */
   ST_FOC_Id_UPPER_INTEGRAL_TERM_LIMIT=27, /*  | */
   ST_FOC_Id_LOWER_OUTPUT_LIMIT=28, /*  | */
   ST_FOC_Id_UPPER_OUTPUT_LIMIT=29, /*  | */
   ST_FOC_FluxWeakening_KP=30, /*  | */
   ST_FOC_FluxWeakening_DEFAULT_KP=31, /*  | */
   ST_FOC_FluxWeakening_KI=32, /*  | */
   ST_FOC_FluxWeakening_DEFAULT_KI=33, /*  | */
   ST_FOC_FluxWeakening_KD=34, /*  | */
   ST_FOC_FluxWeakening_DEFAULT_KD=35, /*  | */
   ST_FOC_FluxWeakening_LOWER_INTEGRAL_TERM_LIMIT=36, /*  | */
   ST_FOC_FluxWeakening_UPPER_INTEGRAL_TERM_LIMIT=37, /*  | */
   ST_FOC_FluxWeakening_LOWER_OUTPUT_LIMIT=38, /*  | */
   ST_FOC_FluxWeakening_UPPER_OUTPUT_LIMIT=39, /*  | */
   ESC_FOC_MAX_SPEED=40, /*  | */
   ECS_FOC_MAX_TORQUE=41, /*  | */
   TI_FOC_SPEED_KP=42, /*  | */
   TI_FOC_SPEED_KI=43, /*  | */
   TI_FOC_TORQUE_KP=44, /*  | */
   TI_FOC_TORQUE_KI=45, /*  | */
   TI_FOC_TURN_FORWARD=50, /*  | */
    TI_FOC_TURN_REVERSAL=51, /*  | */
    TI_FOC_SLOW_START=52, /*  | */
    TI_FOC_BRAKE=53, /*  | */
    TI_FOC_BEEP=60, /*  | */
   ESC_FOC_PARA_ENUM_END=61, /*  | */
} ESC_FOC_PARA;
#endif

/** @brief ack okCommand does not supportData beyond rangeSetup failure */
#ifndef HAVE_ENUM_ESC_FOC_PARA_ACK
#define HAVE_ENUM_ESC_FOC_PARA_ACK
typedef enum ESC_FOC_PARA_ACK
{
   ST_FOC_PARA_CORRECT=0, /*  | */
   ST_FOC_PARA_NOT_SUPPORT=1, /*  | */
   ST_FOC_PARA_BEYOND_RANGE=2, /*  | */
   ST_FOC_PARA_SETUP_FAILURE=3, /*  | */
   ESC_FOC_PARA_ACK_ENUM_END=4, /*  | */
} ESC_FOC_PARA_ACK;
#endif

/** @brief  0-10 功能码  10-15 仿真 15-40 特殊预留  40-60 四轴无人机 60-80 六轴无人机（70-80 ZT系列） 80-100 八轴无人机  */
#ifndef HAVE_ENUM_TTA_FRAME_TYPE
#define HAVE_ENUM_TTA_FRAME_TYPE
typedef enum TTA_FRAME_TYPE
{
   TTA_FRAME_CONFIG_FC=0, /*  默认:飞控6轴模块 . | */
   TTA_FRAME_CONFIG_FC_EX=1, /*  默认:飞控6轴扩展模块. | */
   TTA_FRAME_CONFIG_HUB=2, /*  默认:教育模块. | */
   TTA_FRAME_CONFIG_HUB_EX=3, /*  默认:扩展模块. | */
   TTA_FRAME_ROTOR_4X=4, /*   四旋翼 . | */
   TTA_FRAME_ROTOR_6X=6, /*   六旋翼. | */
   TTA_FRAME_ROTOR_8X=8, /*   八旋翼. | */
   TTA_FRAME_SIMULINK_FC=10, /*  仿真通用. | */
   TTA_FRAME_SIMULINK_WP=11, /*  航线仿真. | */
   TTA_FRAME_SIMULINK_VR=12, /*  VR仿真. | */
   TTA_FRAME_SIMULINK_HIL=13, /*  Mavlink半实物仿真. | */
   TTA_FRAME_SIMULINK_TEST=15, /*  仿真的测试. | */
   TTA_FRAME_EDU_EXAM_HUB=16, /*  教育考试模块. | */
   TTA_FRAME_EDU_SIMULATOR=17, /*  教育飞行模拟器. | */
   TTA_FRAME_HADR_TEST=19, /*  硬件测试工装. | */
   TTA_FRAME_SH_COMMON=20, /*  COMM：M6E、ZT6 . | */
   TTA_FRAME_SH_G200P_COMMON=21, /*  G200P . | */
   TTA_FRAME_TC_COMMON=30, /*  COMM：M6E、ZT6 . | */
   TTA_FRAME_TC_G200P_COMMON=31, /*  COMM：G200P . | */
   TTA_FRAME_TC_ZT6_COMMON=32, /*  COMM：ZT6 . | */
   TTA_FRAME_POD_HOOK=33, /*  抛钩 . | */
   TTA_FRAME_POD_PARACHUTE=34, /*  降落伞 . | */
   TTA_FRAME_4X_FC=50, /*  默认：4X 飞控模块 . | */
   TTA_FRAME_4X_COMMON=51, /*  4X_通用机架. | */
   TTA_FRAME_4X_MINI_EDU=52, /*  4X_轴距550教育四轴. | */
   TTA_FRAME_4X_MINI_550=53, /*  4X_轴距四轴550标准版. | */
   TTA_FRAME_6X_FC=60, /*  默认：6X 飞控模块 . | */
   TTA_FRAME_6X_COMMON=61, /*  6X_通用机架. | */
   TTA_FRAME_6X_M6E=62, /*  M6E机架. | */
   TTA_FRAME_6X_M6E_EX=63, /*  M6E机架扩展. | */
   TTA_FRAME_6X_G200_COMMON=65, /*   6X_G200. | */
   TTA_FRAME_6X_G200P_COMMON=66, /*  M6E机架扩展. | */
   TTA_FRAME_6X_G300_COMMON=67, /*   6X_G300. | */
   TTA_FRAME_6X_GXXX_COMMON=68, /*   6X_GXXX  | */
   TTA_FRAME_6X_ZT_COMMON=69, /*   ZT系列通用 . | */
   TTA_FRAME_6X_ZT_EDU=70, /*   ZT6系列教育通用. | */
   TTA_FRAME_6X_ZT_AG=71, /*   ZT6系列农业通用. | */
   TTA_FRAME_6X_ZT_PROJ=72, /*   ZT系列项目通用. | */
   TTA_FRAME_6X_ZT5_COMMON=73, /*   ZT5通用 . | */
   TTA_FRAME_6X_ZT5_COMMON_EX=74, /*   ZT5扩展通用 . | */
   TTA_FRAME_6X_ZT6_COMMON=75, /*   ZT6通用 . | */
   TTA_FRAME_6X_ZT6_COMMON_EX=76, /*   ZT6扩展通用 . | */
   TTA_FRAME_6X_ZT7_COMMON=77, /*   ZT7通用 . | */
   TTA_FRAME_6X_ZT7_COMMON_EX=78, /*   ZT7扩展通用 . | */
   TTA_FRAME_6X_ZT6M_COMMON_EX=79, /*   ZE6-MIN6版本 . | */
   TTA_FRAME_6X_M6FA=80, /*   M6FA版本 . | */
   TTA_FRAME_6X_M6FA_PLUS=81, /*   M6FA版本 . | */
   TTA_FRAME_6X_M6FC=82, /*   M6FC版本 . | */
   TTA_FRAME_6X_M6FC_T6C=83, /*   M6FC-版本（8017HD-2kv120好盈100A电调） . | */
   TTA_FRAME_6X_M6FA_PLUS1=84, /*   M6FA-版本（M10-缩短机臂 1400：1400：1580） MC-M6A-039 . | */
   TTA_FRAME_6X_M10_040=85, /*   M10多用途版本 MC-M10-040 . | */
   TTA_FRAME_6X_LITE_M10_042=86, /*   M10 Lite 多用途版本 MC-M10-042 . | */
   TTA_FRAME_6X_COMMON_EX=99, /*   6X-通用扩展 结束标志. | */
   TTA_FRAME_8X_FC=100, /*  默认：8X 飞控模块 . | */
   TTA_FRAME_8X_MINI=101, /*  8X_轴距1500以下机架. | */
   TTA_FRAME_8X_NORMAL=102, /*  8X_轴距1500以上机架. | */
   TTA_FRAME_8X_COMMON=103, /*  8X_通用机架. | */
   TTA_FRAME_8X_M8FA=104, /*  8X_M8FA机架(通用). | */
   TTA_FRAME_8X_M8FA_PLUS=105, /*  8X_M8FA机架(18寸碳桨DJ6210电机 好盈80A电调动力套). | */
   TTA_FRAME_8X_M8FA_2=106, /*  8X_M8FA机架(). | */
   TTA_FRAME_8X_M8FD=110, /*  8X_M8FD机架(通用). | */
   TTA_FRAME_8X_M8FD_1=111, /*  8X_M8FD机架(808017HDkv105 好盈100A电调3010). | */
   TTA_FRAME_8X_M8FD_2=112, /*  8X_M8FD机架(). | */
   TTA_FRAME_8X_M50_054=115, /*    8X_M50机架(一体化机架) . | */
   TTA_FRAME_8X_M8E=120, /*  8X_M8E机架(通用). | */
   TTA_FRAME_TTA_MODULE=150, /*  默认：功能模块 . | */
   TTA_FRAME_TTA_MODULE_FW_EVA=151, /*  TTA载人机评估模块 FW-EVA-022  . | */
   TTA_FRAME_TTA_MODULE_FC_EVA=152, /*  TTA载人机评估模块飞控板 FC-EVA-023 . | */
   TTA_FRAME_TTA_MODULE_EDU_EVA=153, /*  TTA飞行能力评估模块 MC-EVA-001 . | */
   TTA_FRAME_TITU_MODULE_EDU_EVA=154, /*  TITU飞行能力评估模块 MC-EVA-022 . | */
   TTA_FRAME_DEVICE_MODULE_SPREAD=512, /*  外设-播撒机 . | */
   TTA_FRAME_TYPE_ENUM_END=513, /*  | */
} TTA_FRAME_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_TTA_DEVICE_TYPE
#define HAVE_ENUM_TTA_DEVICE_TYPE
typedef enum TTA_DEVICE_TYPE
{
   TTA_DEVICE_TYPE_GENERIC=0, /* Generic micro air vehicle. | */
   TTA_DEVICE_TYPE_TC=32, /* tc | */
   TTA_DEVICE_TYPE_FC=33, /* fc | */
   TTA_DEVICE_TYPE_BASE_RTK=34, /* rtk基站 | */
   TTA_DEVICE_TYPE_GW=35, /* gate way | */
   TTA_DEVICE_TYPE_SERVER=36, /* server | */
   TTA_DEVICE_TYPE_OFFLINE_GCS=37, /* Offline ground station | */
   TTA_DEVICE_TYPE_SH=38, /* sh | */
   TTA_DEVICE_TYPE_BMS=39, /* bms | */
   TTA_DEVICE_TYPE_SIM=40, /* simulation | */
   TTA_DEVICE_TYPE_UM482_RTK=41, /* rtk-UM482模块 | */
   TTA_DEVICE_TYPE_SPREAD=42, /* 播撒机升级 | */
   TTA_DEVICE_TYPE_MODULE_FENCE=50, /* 禁飞区模拟模块升级 | */
   TTA_DEVICE_TYPE_MC_G2P_008=61, /*  | */
   TTA_DEVICE_TYPE_MC_M6E_005=62, /*  | */
   TTA_DEVICE_TYPE_RTK_BAS_002=63, /*  | */
   TTA_DEVICE_TYPE_RTK_BAS_006=64, /*  | */
   TTA_DEVICE_TYPE_MC_ZT6_001=65, /*  | */
   TTA_DEVICE_TYPE_MC_EMU_011=66, /*  | */
   TTA_DEVICE_TYPE_FW_EVA_022=67, /*  | */
   TTA_DEVICE_TYPE_WB_TTA_028=68, /*  | */
   TTA_DEVICE_TYPE_VTL_WLT_030=69, /*  | */
   TTA_DEVICE_TYPE_VTL_ZLT_031=70, /*  | */
   TTA_DEVICE_TYPE_MC_M8A_032=71, /*  | */
   TTA_DEVICE_TYPE_MC_G2P_009=72, /*  | */
   TTA_DEVICE_TYPE_MC_M8D_016=73, /*  | */
   TTA_DEVICE_TYPE_MC_T20_020=74, /*  | */
   TTA_DEVICE_TYPE_MC_T6C_033=75, /*  | */
   TTA_DEVICE_TYPE_MC_M4E_034=76, /*  | */
   TTA_DEVICE_TYPE_MC_300_035=77, /*  | */
   TTA_DEVICE_TYPE_MC_M6A_039=78, /*  | */
   TTA_DEVICE_TYPE_MC_M10_040=79, /*  | */
   TTA_DEVICE_TYPE_MC_M10_042=80, /*  | */
   POD_DEVICE_TYPE_PTZ_COMMOM=100, /*  | */
   POD_DEVICE_TYPE_PTZ_CTRL=109, /*  | */
   POD_DEVICE_TYPE_PTZ_RESERVE=110, /*  | */
   POD_DEVICE_TYPE_VISIBLE_LIGHT_CAMERA=120, /*  | */
   POD_DEVICE_TYPE_INFRARED_CAMERA=130, /*  | */
   POD_DEVICE_TYPE_DUAL_LIGHT_CAMERA=140, /*  | */
   POD_DEVICE_TYPE_TRI_LIGHT_CAMERA=150, /*  | */
   POD_DEVICE_TYPE_FIVE_LIGHT_CAMERA=160, /*  | */
   POD_DEVICE_TYPE_MULTI_SPECTRAL_CAMERA=170, /*  | */
   POD_DEVICE_TYPE_RESERVE=180, /*  | */
   TTA_DEVICE_TYPE_MC_EVA_MC=200, /*  | */
   TTA_DEVICE_TYPE_MC_EVA_FW=201, /*  | */
   POD_DEVICE_TYPE_HK=210, /* 火控的外设 | */
   TTA_DEVICE_TYPE_ENUM_END=211, /*  | */
} TTA_DEVICE_TYPE;
#endif

/** @brief power_offpower_on */
#ifndef HAVE_ENUM_BMS_POWER_CMD
#define HAVE_ENUM_BMS_POWER_CMD
typedef enum BMS_POWER_CMD
{
   BMS_POWER_OFF=0, /*  | */
   BMS_POWER_ON=1, /*  | */
   BMS_POWER_CMD_ENUM_END=2, /*  | */
} BMS_POWER_CMD;
#endif

/** @brief  .  .  .  .  .  .  .  .  .  */
#ifndef HAVE_ENUM_BMS_DEVICE
#define HAVE_ENUM_BMS_DEVICE
typedef enum BMS_DEVICE
{
   BMS_DIVER_NULL=0, /*  | */
   BMS_DEVICE_LICO2=1, /*  | */
   BMS_DEVICE_LIB=2, /*  | */
   BMS_DEVICEY_PLB=3, /*  | */
   BMS_DEVICE_NiCoMn=4, /*  | */
   BMS_DEVICE_NIH=5, /*  | */
   BMS_DEVICEY_LIFE=6, /*  | */
   BMS_DEVICE_18650=7, /*  | */
   BMS_DEVICE_26650=8, /*  | */
   BMS_DEVICE_ENUM_END=9, /*  | */
} BMS_DEVICE;
#endif

/** @brief  No need to charge.  Precharge .  Balance charging.  Unavailable.  .  */
#ifndef HAVE_ENUM_BMS_CHARGE_STATE
#define HAVE_ENUM_BMS_CHARGE_STATE
typedef enum BMS_CHARGE_STATE
{
   BMS_CHARGE_NULL=0, /*  | */
   BMS_CHARGE_NEED=1, /*  | */
   BMS_CHARGE_BALANCE=2, /*  | */
   BMS_CHARGE_UNAVAILABLE=3, /*  | */
   BMS_CHARGE_STATE_ENUM_END=4, /*  | */
} BMS_CHARGE_STATE;
#endif

/** @brief  .  .  .  .  .  .  .  .  .  .  .  单电芯电压过低.  单电芯电压过高.  电芯电压均衡度.  内部故障.  放电过流.  充电过流.  总电压欠压.  总电压过压.  温度过低.  温度过高.  低电量二级告警.  低电量一级告警.  电压差值过大.  电量差值过大.  容量差值过大.  温度差值过大.  电池启动异常， 两块电池只启动一块.  飞行中，电池关电.  */
#ifndef HAVE_ENUM_BMS_ERROR
#define HAVE_ENUM_BMS_ERROR
typedef enum BMS_ERROR
{
   BMS_NULL=0, /*  | */
   BMS_DISCHARGE_CURRENT_ERR=1, /*  | */
   BMS_DISCHARGE_VOLTAGE_ERR=2, /*  | */
   BMS_DISCHARGE_TEMPERATURE_ERR=3, /*  | */
   BMS_CHARGE_CURRENT_ERR=4, /*  | */
   BMS_CHARGE_VOLTAGE_ERR=5, /*  | */
   BMS_CHARGE_TEMPERATURE_ERR=6, /*  | */
   BMS_CELL_DETECTED_ERR=7, /*  | */
   BMS_DISCHARGE_LOW_TEMPERATURE_ERR=8, /*  | */
   BMS_CHARGE_LOW_TEMPERATURE_ERR=9, /*  | */
   BMS_WARNING_BATTERY_LIFE=10, /*  | */
   BMS_CELL_VOLTAGE_LOW_ERR=20, /*  | */
   BMS_CELL_VOLTAGE_HIGH_ERR=21, /*  | */
   BMS_CELL_VOLTAGE_BALANCE_ERR=22, /*  | */
   BMS_INTERNAL_FAULT_ERR=23, /*  | */
   BMS_DISCHARGE_OVER_CURRENT_ERR=24, /*  | */
   BMS_CHARGE_OVER_CURRENT_ERR=25, /*  | */
   BMS_TOTAL_VOLTAGE_UNDER_ERR=26, /*  | */
   BMS_TOTAL_VOLTAGE_OVER_ERR=27, /*  | */
   BMS_TEMPERATURE_LOW_ERR=28, /*  | */
   BMS_TEMPERATURE_HIGH_ERR=29, /*  | */
   BMS_LOW_POWER_LEVEL2_ERR=30, /*  | */
   BMS_LOW_POWER_LEVEL1_ERR=31, /*  | */
   BMS_DIFF_VOLTAGE_ERR=50, /*  | */
   BMS_DIFF_ELEC_CAPACITY_ERR=51, /*  | */
   BMS_DIFF_CAPACITY_ERR=52, /*  | */
   BMS_DIFF_TEMPERATURE_ERR=53, /*  | */
   BMS_ONE_START_ERR=54, /*  | */
   BMS_FLIGHT_SHUTDOWN_ERR=55, /*  | */
   BMS_ERROR_ENUM_END=56, /*  | */
} BMS_ERROR;
#endif

/** @brief  */
#ifndef HAVE_ENUM_PTZ_CMD
#define HAVE_ENUM_PTZ_CMD
typedef enum PTZ_CMD
{
   CMD_GYRO_CTRL_PTZ=0, /*  .  | */
   CMD_ATTI_CTRL_PTZ=1, /*  .  | */
   CMD_POINT_CTRL_PTZ=2, /*  .  | */
   CMD_FOLLOW_CTRL_PTZ=3, /*  .  | */
   CMD_FUNCTION_DOWN_PTZ=4, /* One key down | */
   CMD_FUNCTION_RECOVERY_PTZ=5, /* One key recovery | */
   CMD_FUNCTION_PHOTO_CAMERA=6, /* Photo | */
   CMD_FUNCTION_PHOTO_CAMERA_2=7, /* Photo | */
   CMD_FUNCTION_POINT_CAMERA=8, /*  .  | */
   CMD_FUNCTION_ZOOM_IN_CAMERA=9, /* zoom in | */
   CMD_FUNCTION_ZOOM_OUT_CAMERA=10, /* zoom out | */
   CMD_FUNCTION_ZOOM_CAMERA=11, /* zoom param | */
   CMD_FUNCTION_ZOOM_QUERY=12, /* zoom query | */
   CMD_FUNCTION_VIDIO_START=15, /* Vidio | */
   CMD_FUNCTION_VIDIO_END=16, /* Vidio | */
   PTZ_CMD_ENUM_END=17, /*  | */
} PTZ_CMD;
#endif

/** @brief  */
#ifndef HAVE_ENUM_PTZ_ACK
#define HAVE_ENUM_PTZ_ACK
typedef enum PTZ_ACK
{
   PTZ_ACK_NULL=0, /*  .  | */
   PTZ_ACK_FALSE=1, /*  .  | */
   PTZ_ACK_ENUM_END=2, /*  | */
} PTZ_ACK;
#endif

/** @brief  */
#ifndef HAVE_ENUM_YAW_CTRL_CMD
#define HAVE_ENUM_YAW_CTRL_CMD
typedef enum YAW_CTRL_CMD
{
   AUTO_WP_YAW=0, /*  随航线.  | */
   SPECIFIED_YAW=1, /*  指向特定角度.  | */
   TARGET_PONIT_YAW=2, /*  指向特定经纬度点.  | */
   TARGET_HOME_YAW=4, /*  指向HOME点.  | */
   NORMAL_YAW=5, /*  不控制.  | */
   YAW_CTRL_CMD_ENUM_END=6, /*  | */
} YAW_CTRL_CMD;
#endif

/** @brief  */
#ifndef HAVE_ENUM_PTZ_ERROR
#define HAVE_ENUM_PTZ_ERROR
typedef enum PTZ_ERROR
{
   PTZ_ERROR_NULL=0, /*  .  | */
   PTZ_ERROR_ENUM_END=1, /*  | */
} PTZ_ERROR;
#endif

/** @brief  */
#ifndef HAVE_ENUM_FLIGHT_MODE_EXAMINATION
#define HAVE_ENUM_FLIGHT_MODE_EXAMINATION
typedef enum FLIGHT_MODE_EXAMINATION
{
   EXAM_MODE_MANUAL=0, /*   | */
   EXAM_MODE_ATTI=1, /*   | */
   EXAM_MODE_GPS=2, /*   | */
   EXAM_MODE_WP=3, /*   | */
   EXAM_MODE_RETURN=4, /*   | */
   EXAM_MODE_LAND=5, /*   | */
   EXAM_MODE_COMPETITIVE=6, /*   | */
   EXAM_MODE_BOOT=7, /*   | */
   EXAM_MODE_INTEREST=8, /*   | */
   EXAM_MODE_PARAMETER=9, /*   | */
   EXAM_MODE_AB=10, /*   | */
   EXAM_MODE_NO_SUPPORT=11, /*   | */
   FLIGHT_MODE_EXAMINATION_ENUM_END=12, /*  | */
} FLIGHT_MODE_EXAMINATION;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GPS_FIX_TPYE
#define HAVE_ENUM_GPS_FIX_TPYE
typedef enum GPS_FIX_TPYE
{
   FIX_NO_GPS=0, /*  无定位. | */
   FIX_BAD_GPS=1, /*  GPS定位差.  | */
   FIX_3D=2, /*  RTK单点解定位.  | */
   FIX_3D_RTD=3, /*  RTK差分解定位.  | */
   FIX_3D_RTK_FLOAT=4, /* RTK浮点解定位 .  | */
   FIX_3D_RTK_FIX=5, /*  RTK固定解定位.  | */
   FIX_GPS=6, /*  Gps定位.  | */
   GPS_FIX_TPYE_ENUM_END=7, /*  | */
} GPS_FIX_TPYE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_SWITCH_GPS_RTK
#define HAVE_ENUM_SWITCH_GPS_RTK
typedef enum SWITCH_GPS_RTK
{
   SWITCH_NULL=0, /* NULL | */
   SWITCH_GPS1=1, /* GPS1 | */
   SWITCH_GPS2=2, /* GPS2 | */
   SWITCH_RTK=3, /* RTK | */
   SWITCH_GPS_RTK_ENUM_END=4, /*  | */
} SWITCH_GPS_RTK;
#endif

/** @brief  */
#ifndef HAVE_ENUM_VR_SETING_STATUS
#define HAVE_ENUM_VR_SETING_STATUS
typedef enum VR_SETING_STATUS
{
   VR_STATUS_NULL=0, /* NULL | */
   FC_HEART_BEAT=1, /* 飞控心跳 1HZ 发送  | */
   VR_HEART_BEAT=2, /*  VR心跳 1HZ 发送 | */
   SET_INIT_POSITION=3, /*  初始位置界面点击后设置 | */
   VR_SETING_STATUS_ENUM_END=4, /*  | */
} VR_SETING_STATUS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_DEVICE_ACTION
#define HAVE_ENUM_DEVICE_ACTION
typedef enum DEVICE_ACTION
{
   TTALINK_NULL=0, /* NULL | */
   TTALINK_CAMERA_SHOOT_ONCE=1, /*  相机----拍照功能----单次拍照 | */
   TTALINK_CAMERA_SHOOT_EQUITIME=2, /* 相机----拍照功能----等时拍照  | */
   TTALINK_CAMERA_SHOOT_STOP=3, /*  相机----拍照功能----停止拍照 | */
   TTALINK_CAMERA_REC_START=4, /*  相机----录像功能----开始录像 | */
   TTALINK_CAMERA_REC_STOP=5, /*  相机----录像功能----结束录像 | */
   TTALINK_CAMERA_ZOOM_IN=6, /*  相机----变焦功能----图像放大 | */
   TTALINK_CAMERA_ZOOM_OUT=7, /*  相机----变焦功能----图像缩小 | */
   TTALINK_CAMERA_SHOOT_EQUIDISTANT=8, /* 相机----拍照功能----等距拍照 | */
   TTALINK_CAMERA_ZOOM_VALUE=9, /* 相机----变焦功能----变焦倍率 | */
   TTALINK_GIMBAL_YAW=10, /* 云台----角度功能----偏航角度  | */
   TTALINK_GIMBAL_PTICH=11, /* 云台----角度功能----俯仰角度  | */
   TTALINK_GIMBAL_ROLL=12, /* 云台----角度功能----滚转角度  | */
   TTALINK_GIMBAL_DOWN=13, /* 云台----云台模式----云台向下  | */
   TTALINK_GIMBAL_RESET=14, /* 云台----云台模式----云台复位  | */
   TTALINK_GIMBAL_FOLLOW=15, /* 云台----云台模式----跟随模式  | */
   TTALINK_GIMBAL_LOCK=16, /* 云台----云台模式----航向锁定  | */
   TTALINK_GIMBAL_FREE=17, /* 云台----云台模式----自由模式  | */
   TTALINK_GIMBAL_ATTITUDE=18, /* 云台----角度功能----姿态角度（偏航、俯仰、滚转）  | */
   TTALINK_SPRAY_MANUAL=20, /* 水泵----水泵模式----手动喷洒  | */
   TTALINK_SPRAY_FLOW=21, /* 水泵----流量功能----水泵流量  | */
   TTALINK_SPRAY_START=22, /* 水泵----水泵模式--- 自动喷洒开  | */
   TTALINK_SPRAY_END=23, /* 水泵----水泵模式----水泵关  | */
   TTALINK_SPRAY_MANUAL_VALUE=24, /* 水泵----水泵模式----水泵手动数值  | */
   TTALINK_CAMERA_FIVE_WUPIN=30, /* 五拼相机----拍照功能----单次拍照  | */
   TTALINK_ACTION_ANCHOR=40, /*  停泊----交互  | */
   TTALINK_LIGHT_ON=55, /* 探照灯----探照灯开关----探照灯开  | */
   TTALINK_LIGHT_OFF=56, /* 探照灯----探照灯开关----探照灯关  | */
   TTALINK_LIGHT_ON_VALUE=57, /* 探照灯----探照灯开关----探照灯开数值  | */
   TTALINK_MIC_ON=60, /* 喊话器----喊话器开关----喊话器开  | */
   TTALINK_MIC_OFF=61, /* 喊话器----喊话器开关----喊话器关  | */
   TTALINK_MIC_ON_VALUE=62, /* 抛放器----抛放器开关----抛放器数值  | */
   TTALINK_ACTION_THROWER_ON=65, /* 抛放器----抛放器开关----抛放器开  | */
   TTALINK_ACTION_THROWER_OFF=66, /* 抛放器----抛放器开关----抛放器关  | */
   TTALINK_ACTION_THROWER_VALUE=67, /* 抛放器----抛放器开关----抛放器数值  | */
   TTALINK_ACTION_LANDING_GEAR_DOWN=70, /* 起落架----起落架开关----打开  | */
   TTALINK_ACTION_LANDING_GEAR_UP=71, /* 起落架----起落架开关----关闭  | */
   TTALINK_ACTION_LANDING_GEAR_VALUE=72, /* 抛放器----抛放器开关----抛放器关  | */
   TTALINK_ACTION_USER_ON=80, /*   | */
   TTALINK_ACTION_USER_OFF=81, /*   | */
   TTALINK_ACTION_USER_ON_VALUE=82, /*   | */
   DEVICE_ACTION_ENUM_END=83, /*  | */
} DEVICE_ACTION;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GENERAL_CMD
#define HAVE_ENUM_GENERAL_CMD
typedef enum GENERAL_CMD
{
   GENERAL_CMD_NULL=0, /* NULL | */
   GENERAL_CMD_MOTOR_TEST=10, /* 马达测试 Param1 = 0 关闭  1 开启 | */
   GENERAL_CMD_UART1_TEST=20, /* 串口1数据测试 0关闭 1 开启发送一包数据FFFFFFFF | */
   GENERAL_CMD_SENSOR_FIX_ANG=30, /* 传感器校准-安装角度偏差 | */
   GENERAL_CMD_SENSOR_FIX_POS=31, /* 传感器校准-安装位置偏差 | */
   GENERAL_CMD_SENSOR_BATT_CALI=32, /* 电池校准 | */
   GENERAL_CMD_SENSOR_HORIZONTAL=33, /* 教育模块水平校准 | */
   GENERAL_CMD_SENSOR_RTK_FRESET=40, /* 传感器板 SH ：复位RTK | */
   GENERAL_CMD_SENSOR_RTK_RESET=41, /* 传感器板 SH ：重新配置RTK | */
   GENERAL_CMD_SENSOR_RTK_ECHO_SWITCH=45, /* 传感器板 SH ：RTK模块升级回传： 0 关闭 1 开启 | */
   GENERAL_CMD_PUMP1=50, /* 水泵1测试 0=关 1=开 | */
   GENERAL_CMD_PUMP2=51, /* 水泵2测试0=关 1=开 | */
   GENERAL_CMD_SPEED=52, /* 设置水泵流速 mL/min | */
   GENERAL_CMD_LED=53, /* G200探照灯测试 0=关 1=开 | */
   GENERAL_CMD_VOLUME=54, /* 设置水泵喷洒量 | */
   GENERAL_CMD_SET_SN=55, /* 设置SN码 | */
   GENERAL_CMD_LOG_CMD=56, /* log控制：param1 0=关闭log（如果开启的话）1=开启log param2 0=log-box 1=log-rtcm 2=log-rtk-raw | */
   GENERAL_CMD_AUTHORIZE_STATUS=60, /* 设置授权 param1 0=不授权 1=授权 255=后门  param2=year param3=month param4=date param5=hour param6=min  param7=sec | */
   GENERAL_CMD_REQUEST_AUTHORIZE=61, /* 请求飞控授权日期 | */
   GENERAL_CMD_FC_LIMIT=65, /* 设置飞控受限 | */
   GENERAL_CMD_REQUEST_FC_LIMIT=66, /* 请求飞控受限状态 | */
   GENERAL_CMD_RADAR_CLEAR=70, /* 清除雷达坐标点 | */
   GENERAL_CMD_FANCE_ELEC=71, /*  清除电子围栏数据.  | */
   GENERAL_CMD_SIMULATE_ERR=90, /*  模拟故障.  | */
   GENERAL_CMD_RESET_SYSTEM=100, /* 重启命令 0=NULL 2019=重启命令生效 | */
   GENERAL_CMD_RESET_PARAM=110, /* 0:NULL 1=cmd; param1=复位飞控飞行参数(PID)  param2=复位系统参数(SYS) param3=复位无人机飞行参数(UAV) param4=复位用户飞行参数(User) | */
   GENERAL_CMD_EVENT_ACTION=200, /* 
			
			param1 = enmu DEVICE_ACTION;			param2 = value；
			
			 | */
   GENERAL_CMD_EVENT_TEST=201, /* 测试事件触发 与 GENERAL_CMD_EVENT_ACTION 功能相同  | */
   GENERAL_CMD_EVENT_RC=202, /* 遥控器事件触发 与 GENERAL_CMD_EVENT_ACTION 功能相同  | */
   GENERAL_CMD_END=255, /* 通用命令结束标准 | */
   GENERAL_STATUS_NOTIFY_SET_WP_BREAK_POINT=513, /* 飞控通知：设置植保航线断点 | */
   GENERAL_STATUS_NOTIFY_SET_AB_BREAK_POINT=514, /* 飞控通知：设置AB点航线断点 | */
   GENERAL_STATUS_NOTIFY_CLEAR_WP_BREAK_POINT=515, /* 飞控通知：清除植保航线断点 | */
   GENERAL_STATUS_NOTIFY_CLEAR_AB_BREAK_POINT=516, /* 飞控通知：清除AB点航线断点 | */
   GENERAL_STATUS_EVENT_AB_ACTION=520, /* 执行AB点航线 | */
   GENERAL_STATUS_EVENT_AB_SAVE_BREAK=521, /* 保存AB点断点 | */
   GENERAL_STATUS_EVENT_AB_READ_BREAK=522, /* 读取AB点断点 | */
   GENERAL_STATUS_MONITOR_CHECK_ERR=590, /* 飞控通知：自检不通过 | */
   GENERAL_STATUS_MONITOR_CHECK_OK=591, /* 飞控通知：自检通过 | */
   GENERAL_STATUS_WP_SUSPEND=600, /* 飞控通知：航线任务中断 | */
   GENERAL_STATUS_WP_TASK_EMPTY=601, /* 飞控通知：航线任务断药事件 | */
   GENERAL_STATUS_WP_TASK_PAUSE=602, /* 飞控通知：航线任务暂停 | */
   GENERAL_STATUS_WP_TASK_AGAIN=603, /* 飞控通知：航线任务继续 | */
   GENERAL_STATUS_WP_TASK_POWER_B=604, /* 飞控通知：航线任务-低电量B级报警 | */
   GENERAL_STATUS_WP_TASK_POWER_A=605, /* 飞控通知：航线任务-低电量A级报警 | */
   GENERAL_STATUS_WP_EVENT_NULL=620, /* 飞控通知：航线任务-事件无动作 | */
   GENERAL_STATUS_WP_EVENT_WARN=621, /* 飞控通知：航线任务-事件告警 | */
   GENERAL_STATUS_WP_EVENT_PAUSE=622, /* 飞控通知：航线任务-事件暂停 | */
   GENERAL_STATUS_WP_EVENT_RTH=623, /* 飞控通知：航线任务-事件返航 | */
   GENERAL_STATUS_WP_EVENT_LAND=624, /* 飞控通知：航线任务-事件降落 | */
   GENERAL_STATUS_ALT_LIMIT=630, /* 飞控通知：无人机高度限制 | */
   GENERAL_STATUS_DIS_LIMIT=631, /* 飞控通知：无人机距离限制 | */
   GENERAL_STATUS_FENCE_NOFLY_LIMIT=638, /* 飞控通知：进入禁飞区减速区 | */
   GENERAL_STATUS_FENCE_NOFLY=639, /* 飞控通知：进入禁飞区警告 | */
   GENERAL_STATUS_FENCE_NOFLY_NEAR=640, /* 飞控通知：禁飞区附近警告 | */
   GENERAL_STATUS_FENCE_ELEC=641, /* 飞控通知：飞出电子围栏警告 | */
   GENERAL_STATUS_FENCE_ELEC_NEAR=642, /* 飞控通知：电子围栏附近警告 | */
   GENERAL_STATUS_SYSTEM_NAVI_ATTI_MODE=650, /* 飞控通知：导航手动标字异常 | */
   GENERAL_STATUS_PLAN_WP_ERR=651, /* 飞控通知：航线规划异常 | */
   GENERAL_CMD_SET_FIX_ANG=1010, /* 旧版教育模块校准安装角度 | */
   GENERAL_CMD_SET_FIX_ALT=1011, /* 模块高度重置 | */
   GENERAL_CMD_EXAM_START=1012, /*   | */
   GENERAL_CMD_EXAM_STOP=1013, /*   | */
   GENERAL_CMD_CHANGE_UPDATE_URL=1014, /*   | */
   GENERAL_CMD_CHANGE_ACCESS_WAY=1015, /*  教育模块TCPUDP选择命令 | */
   GENERAL_CMD_SET_FIX_POS=1016, /*  教育模块杆臂补偿值设置命令 | */
   GENERAL_CMD_CHANGE_EXAM_PLANE_TYPE=1017, /*  教育模块选择考试机型（多旋翼/固定翼） | */
   GENERAL_CMD_ENUM_END=1018, /*  | */
} GENERAL_CMD;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GENERAL_BROADCAST_LIST
#define HAVE_ENUM_GENERAL_BROADCAST_LIST
typedef enum GENERAL_BROADCAST_LIST
{
   GENERAL_VB_NULL=0, /* NULL | */
   GENERAL_VB_RC_RTH=1, /* 遥控器返航 | */
   GENERAL_VB_RC_LOST=2, /* 遥控器失联 | */
   GENERAL_VB_GCS_LOST=3, /* 地面站失联 | */
   GENERAL_VB_AB_EMPTY=4, /* AB点断药保护 | */
   GENERAL_VB_WP_EMPTY=5, /* 航线断药保护 | */
   GENERAL_VB_FRONT_RADAR=6, /* 前雷达避障 | */
   GENERAL_VB_BACK_RADAR=7, /* 后雷达避障 | */
   GENERAL_VB_LOW_POWER_B=8, /* 低电量一级告警 | */
   GENERAL_VB_LOW_POWER_A=9, /* 低电量二级告警 | */
   GENERAL_VB_FLY_ZONE_WARNING=10, /* 禁飞区预警 | */
   GENERAL_VB_FENCE_WARNING=11, /* 电子围栏预警 | */
   GENERAL_VB_ALTIT_LIMIT=12, /* 高度限制预警 | */
   GENERAL_VB_DISTANCE_LIMIT=13, /* 距离限制预警 | */
   GENERAL_BROADCAST_LIST_ENUM_END=14, /*  | */
} GENERAL_BROADCAST_LIST;
#endif

/** @brief  */
#ifndef HAVE_ENUM_RC_AUXSW_FUNCTION
#define HAVE_ENUM_RC_AUXSW_FUNCTION
typedef enum RC_AUXSW_FUNCTION
{
   RC_SW_FUNC_NULL=0, /* NULL（模式控制只能在6通道选择，0：可选模式  1：NULL 2：可选模式） | */
   RC_SW_FUNC_MODE_STABILIZE_ENABLE=1, /*  手动模式，只带增稳控制  | */
   RC_SW_FUNC_MODE_AB_ENABLE=3, /*  AB点模式，锁定航向  | */
   RC_SW_FUNC_MODE_GCS_DISABLE=4, /*  地面站失效控制 （失效命令） | */
   RC_SW_FUNC_MODE_GCS_RC_ENABLE=5, /*  地面站遥控器控制 （远程遥控器功能）  | */
   RC_SW_FUNC_CHAN_NULL=10, /* NULL（通道功能可在 7，8，9，10 通道选择，按照以下说明设置） | */
   RC_SW_FUNC_CHAN_RECORD_AB=11, /*  4秒内3次连续打杆，清除AB点 0~1：记录A点  1~2：记录B点  | */
   RC_SW_FUNC_CHAN_PUMP_CTRL=12, /*  0:AUTO 1:CLOSE 2:OPEN | */
   RC_SW_FUNC_CHAN_PARACHUTE_CTRL=13, /*  0：NULL  1:NULL  2:开伞 | */
   RC_SW_FUNC_CHAN_OBSTACLE_CTRL=14, /*  0：NULL  1:NULL  2:开启避障 | */
   RC_SW_FUNC_CHAN_FOLLOW_CTRL=15, /*  0：NULL  1:NULL  2:开启仿地 | */
   RC_SW_FUNC_CHAN_LAND_GEAR_CTRL=16, /*  0：NULL  1:NULL  2:开启起落架 | */
   RC_AUXSW_FUNC_CHAN_NULL=20, /* NULL（通道功能可在 11，12 通道选择，按照以下说明设置） | */
   RC_AUXSW_FUNC_CHAN_FLOW_RATE_CTRL=21, /*  0：减小 1：不变 2：增加 | */
   RC_AUXSW_FUNC_CHAN_PWM_CTRL=22, /*   | */
   RC_SW_FUNC_CHAN_POD_NULL=30, /* NULL（通道功能可在 13，14，15，16 通道选择，按照以下说明设置） | */
   RC_SW_FUNC_CHAN_POD_PTZ_PITCH=31, /*  0: -Rate_MIN 1: Rate_0  2:Rate_MAX | */
   RC_SW_FUNC_CHAN_POD_PTZ_ROLL=32, /*  0: -Rate_MIN 1: Rate_0  2:Rate_MAX | */
   RC_SW_FUNC_CHAN_POD_PTZ_YAW=33, /*  0: -Rate_MIN 1: Rate_0  2:Rate_MAX | */
   RC_AUXSW_FUNCTION_ENUM_END=34, /*  | */
} RC_AUXSW_FUNCTION;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GENERAL_POINTS_COMMAND_TYPE
#define HAVE_ENUM_GENERAL_POINTS_COMMAND_TYPE
typedef enum GENERAL_POINTS_COMMAND_TYPE
{
   TYPE_FARM_FIELD_POINTS=1, /*  | */
   GENERAL_POINTS_COMMAND_TYPE_ENUM_END=2, /*  | */
} GENERAL_POINTS_COMMAND_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GENERAL_ENTITY_TYPE
#define HAVE_ENUM_GENERAL_ENTITY_TYPE
typedef enum GENERAL_ENTITY_TYPE
{
   TYPE_FARM_TASK=1, /*  | */
   TYPE_FARM_FIELD=2, /*  | */
   GENERAL_ENTITY_TYPE_ENUM_END=3, /*  | */
} GENERAL_ENTITY_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GENERAL_POINTS_COMMAND_CMD
#define HAVE_ENUM_GENERAL_POINTS_COMMAND_CMD
typedef enum GENERAL_POINTS_COMMAND_CMD
{
   CMD_GET_COUNT=0, /*  | */
   CMD_GET_ITEM=1, /*  | */
   CMD_GET_COUNT_ACK=1000, /*  | */
   CMD_GET_ITEM_ACK=1001, /*  | */
   GENERAL_POINTS_COMMAND_CMD_ENUM_END=1002, /*  | */
} GENERAL_POINTS_COMMAND_CMD;
#endif

/** @brief  */
#ifndef HAVE_ENUM_FLIGHT_CTRL_STATUS_E
#define HAVE_ENUM_FLIGHT_CTRL_STATUS_E
typedef enum FLIGHT_CTRL_STATUS_E
{
   ROS_F_BASE_NC=0, /*  | */
   ROS_F_BASE_TAKE_OFF=1, /*  | */
   ROS_F_BASE_ATTI_AUTO_ALTI=2, /*  | */
   ROS_F_BASE_ATTI_AUTO_ALTI_VEL=3, /*  | */
   ROS_F_BASE_ATTI_WITHOUT_AUTOGAS=4, /*  | */
   ROS_F_BASE_ATTI_WITHOUT_AUTOGAS_LOOP=5, /*  | */
   ROS_F_GPS_POS_TAKE_OFF=6, /*  | */
   ROS_F_GPS_POS_VEL=7, /*  | */
   ROS_F_GPS_POS_VEL_ALTI_VEL=8, /*  | */
   ROS_F_GPS_HOVER_ACC=9, /*  | */
   ROS_F_GPS_HOVER=10, /*  | */
   ROS_F_GPS_HOVER_ALTI_VEL=11, /*  | */
   ROS_F_GPS_NAVGATION=12, /*  | */
   ROS_F_GPS_NAVGATION_ALTI_VEL=13, /*  | */
   ROS_F_GPS_AUTO_TAKE_OFF=14, /*  | */
   ROS_F_GPS_AUTO_LAND=15, /*  | */
   ROS_F_MANUAL_LAND=16, /*  | */
   ROS_F_EMERGENCY_LAND=17, /*  | */
   ROS_F_BASE_LOCK=18, /*  | */
   ROS_F_BASE_UNLOCK=19, /*  | */
   ROS_F_BASE_IDLING=20, /*  | */
   FLIGHT_CTRL_STATUS_E_ENUM_END=21, /*  | */
} FLIGHT_CTRL_STATUS_E;
#endif

// MESSAGE DEFINITIONS
#include "./ttalink_heartbeat.h"
#include "./ttalink_general_message.h"
#include "./ttalink_com_ack.h"
#include "./ttalink_general_command.h"
#include "./ttalink_general_command_ack.h"
#include "./ttalink_general_param_request_read.h"
#include "./ttalink_general_param_request_list.h"
#include "./ttalink_general_param_value.h"
#include "./ttalink_general_param_set.h"
#include "./ttalink_general_status.h"
#include "./ttalink_general_request_version.h"
#include "./ttalink_general_request_version_ack.h"
#include "./ttalink_general_request_upload_img.h"
#include "./ttalink_general_request_upload_img_ack.h"
#include "./ttalink_general_param_set_ack.h"
#include "./ttalink_general_entity.h"
#include "./ttalink_general_points_command.h"
#include "./ttalink_rosuav_imu_raw.h"
#include "./ttalink_rosuav_baro_raw.h"
#include "./ttalink_rosuav_p20_atti.h"
#include "./ttalink_rosuav_p20_navi.h"
#include "./ttalink_rosuav_p20_raw.h"
#include "./ttalink_rosuav_ctrl_loop_input.h"
#include "./ttalink_rosuav_ctrl_feed_back.h"
#include "./ttalink_rosuav_ctrl_target_data.h"

// base include


#undef TTALINK_THIS_XML_IDX
#define TTALINK_THIS_XML_IDX 0

#if TTALINK_THIS_XML_IDX == TTALINK_PRIMARY_XML_IDX
# define TTALINK_MESSAGE_INFO {{"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, TTALINK_MESSAGE_INFO_MAV_HIL_SIM_STATE, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, TTALINK_MESSAGE_INFO_EXAM_STATE, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, TTALINK_MESSAGE_INFO_EXAM_DATA, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}, {"EMPTY",0,{{"","",TTALINK_TYPE_CHAR,0,0,0}}}}
# define TTALINK_MESSAGE_NAMES {{ "AHRS_INFO", 2003 }, { "BARO_SENSORS_DATA", 2170 }, { "BATTERY_DATA", 2005 }, { "BMS_DATA", 2052 }, { "BMS_ERR_INFO", 2055 }, { "BMS_GET_CHARGE_SN", 2058 }, { "BMS_GET_CHARGE_SN_ACK", 2059 }, { "BMS_GET_ERROR_INFO", 2061 }, { "BMS_GET_ERROR_INFO_ACK", 2062 }, { "BMS_INFO", 2054 }, { "BMS_INFO_CHECK", 2053 }, { "BMS_POWER_CONTROL", 2050 }, { "BMS_POWER_CONTROL_ACK", 2051 }, { "BMS_REMAIN_TIME", 2060 }, { "BMS_SET_BALANCE_CHARGE", 2063 }, { "BMS_SET_BALANCE_CHARGE_ACK", 2064 }, { "CALIBRATE_DATA", 2127 }, { "CALIBRATE_DATA_ACK", 2128 }, { "CHANNEL_FUNCTION_CONFIG", 2258 }, { "CHANNEL_FUNCTION_CONFIG_ACK", 2261 }, { "CHANNEL_FUNCTION_CONFIG_CMD", 2260 }, { "COM_ACK", 2252 }, { "COM_SHELL_SWITCH", 2208 }, { "COM_SHELL_SWITCH_ACK", 2209 }, { "CONFIG_STREAM", 2140 }, { "CONFIG_STREAM_ACK", 2141 }, { "DEVICE_DATA", 2009 }, { "ELEC_FENCE_ITEMS", 2216 }, { "EMBE_CONFIG_ITEMS", 2082 }, { "EMBE_CONFIG_ITEMS_ACK", 2083 }, { "EMBE_RSSI", 2089 }, { "ERROR_CODE", 2254 }, { "EXAM_DATA", 194 }, { "EXAM_STATE", 190 }, { "EXTERN_MESSAGE", 2253 }, { "FILES_OPS", 2251 }, { "FOC_ARM_STATE", 2016 }, { "FOC_ARM_STATE_ACK", 2017 }, { "FOC_PARAMETERS", 2014 }, { "FOC_PARAMETERS_ACK", 2015 }, { "FOC_REPORT_BASIC_INFO", 2011 }, { "FOC_SEND_CMD", 2010 }, { "FOC_SET_PRIVATE_ADDR", 2012 }, { "FOC_SET_PRIVATE_ADDR_ACK", 2013 }, { "FOC_VERSION_INFOMATION", 2018 }, { "FOC_VERSION_INFOMATION_ACK", 2019 }, { "FUNCTION_MODE", 2204 }, { "FUNCTION_MODE_ACK", 2205 }, { "GCS_CONFIG_ITEMS", 2080 }, { "GCS_CONFIG_ITEMS_ACK", 2081 }, { "GENERAL_COMMAND", 4097 }, { "GENERAL_COMMAND_ACK", 4098 }, { "GENERAL_ENTITY", 4111 }, { "GENERAL_MESSAGE", 2240 }, { "GENERAL_PARAM_REQUEST_LIST", 4100 }, { "GENERAL_PARAM_REQUEST_READ", 4099 }, { "GENERAL_PARAM_SET", 4102 }, { "GENERAL_PARAM_SET_ACK", 4110 }, { "GENERAL_PARAM_VALUE", 4101 }, { "GENERAL_POINTS_COMMAND", 4112 }, { "GENERAL_REQUEST_UPLOAD_IMG", 4106 }, { "GENERAL_REQUEST_UPLOAD_IMG_ACK", 4107 }, { "GENERAL_REQUEST_VERSION", 4104 }, { "GENERAL_REQUEST_VERSION_ACK", 4105 }, { "GENERAL_STATUS", 4103 }, { "GENERAL_UPLOAD_IMG", 4108 }, { "GENERAL_UPLOAD_IMG_ACK", 4109 }, { "GENERAL_VOICE_BROADCAST", 4115 }, { "GET_GENERAL_MESSAGE", 2241 }, { "GPS_SENSORS_DATA", 2171 }, { "HEARTBEAT", 2001 }, { "IMU_SENSORS_AUGMENT_DATA", 2179 }, { "IMU_SENSORS_DATA", 2172 }, { "IRRIGATE_CTRL", 2160 }, { "IRRIGATE_CTRL_ACK", 2161 }, { "IRRIGATE_DATA", 2163 }, { "IRRIGATE_PARAM", 2164 }, { "IRRIGATE_PARAM_ACK", 2165 }, { "MAG_SENSORS_DATA", 2173 }, { "MAV_HIL_COMMAND_LONG", 8276 }, { "MAV_HIL_GPS", 8210 }, { "MAV_HIL_MOTOR", 8220 }, { "MAV_HIL_SENSOR", 8211 }, { "MAV_HIL_SIM_STATE", 108 }, { "MAV_HIL_STATE_QUATERNION", 8212 }, { "MAV_HIL_SYSTIME", 8209 }, { "MULTI_THROW_HOOK", 2270 }, { "MULTI_THROW_HOOK_ACK", 2271 }, { "MULTI_THROW_HOOK_STATUS", 2272 }, { "PASSTHROUGH", 2250 }, { "PASSTHROUGH_VARIABLE", 2244 }, { "PASS_COM_SHELL", 2245 }, { "PASS_JSON", 2248 }, { "PASS_PAYLOAD", 2249 }, { "PASS_RTK", 2246 }, { "PASS_RTK_LOW_BUFF", 2247 }, { "PASS_TC_DEVICE_LOG", 2243 }, { "PASS_URL", 2257 }, { "POD_1025_SET_CAMERA_MODE", 11025 }, { "POD_1026_GET_CAMERA_MODE", 11026 }, { "POD_1027_TAKE_PHOTO", 11027 }, { "POD_1028_GET_PHOTO_STATE", 11028 }, { "POD_1029_SET_PHOTO_MODE", 11029 }, { "POD_1030_GET_PHOTO_MODE", 11030 }, { "POD_1031_SET_RECORD_VIDEO", 11031 }, { "POD_1032_GET_RECORD_VIDEO", 11032 }, { "POD_1033_GET_SD_PARAM", 11033 }, { "POD_1034_FORMAT_SD_CARD", 11034 }, { "POD_1056_SET_MEASURE_MODE", 11056 }, { "POD_1057_GET_MEASURE_MODE", 11057 }, { "POD_1058_SET_MEASURE_AREA", 11058 }, { "POD_1059_GET_MEASURE_MODE", 11059 }, { "POD_1072_SET_FOCUS_MODE", 11072 }, { "POD_1073_GET_MEASURE_MODE", 11073 }, { "POD_1074_SET_FOCUS_AREA", 11074 }, { "POD_1075_GET_MEASURE_AREA", 11075 }, { "POD_1076_SET_DIGITAL_ZOOM_FACTOR", 11076 }, { "POD_1077_GET_DIGITAL_ZOOM_FACTOR", 11077 }, { "POD_1078_GET_OPTICAL_ZOOM_SPECS", 11078 }, { "POD_1079_SET_OPTICAL_ZOOM_FOCAL", 11079 }, { "POD_1080_GET_OPTICAL_ZOOM_FOCAL", 11080 }, { "POD_1081_GET_OPTICAL_ZOOM_FACTOR", 11081 }, { "POD_1088_START_OPTICAL_ZOOM", 11088 }, { "POD_1089_STOP_OPTICAL_ZOOM", 11089 }, { "POD_1090_SET_FOCUS_ASSIST_SETTING", 11090 }, { "POD_1091_GET_FOCUS_ASSIST_SETTING", 11091 }, { "POD_1092_GET_FOCUS_RING_MAX", 11092 }, { "POD_1093_SET_FOCUS_RING_MAX", 11093 }, { "POD_1094_SET_FOCUS_RING_MAX", 11094 }, { "POD_1095_GET_DIGITAL_ZOOM_SPECS", 11095 }, { "POD_1279_GET_CAMERA_SUPPORT_STATUS", 11279 }, { "POD_1281_GET_PTZ_STATUS", 11281 }, { "POD_1282_SET_PTZ_MODE", 11282 }, { "POD_1283_PTZ_HEAD_BACK", 11283 }, { "POD_1284_CTRL_FRAME_GYRO", 11284 }, { "POD_1285_CTRL_FRAME_ANGLE", 11285 }, { "POD_1286_CTRL_PTZ_ANGLE", 11286 }, { "POD_1287_SET_PTZ_CALI", 11287 }, { "POD_1288_UPDATE_PTZ_CALI_PROGRESS", 11288 }, { "POD_1359_UPDATE_PTZ_TTA_NAVI_DATA", 11359 }, { "POD_1360_UPDATE_PTZ_NAVI_DATA", 11360 }, { "POD_1361_UPDATE_PTZ_MOTOR_DATA", 11361 }, { "POD_1376_SET_PTZ_PID_PARA", 11376 }, { "POD_1377_UPDATE_PTZ_PID_PARA", 11377 }, { "POD_1378_Get_PTZ_PID_PARA", 11378 }, { "POD_1534_UPDATE_PTZ_ERROR", 11534 }, { "POD_1535_GET_PTZ_SUPPORT_STATE", 11535 }, { "POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD", 11537 }, { "POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT", 11538 }, { "POD_1649_SPEAKER_VOLUME_CTRL", 11649 }, { "POD_1650_SEARCHLIGHT_LIGHT_CTRL", 11650 }, { "POD_1651_SEARCHLIGHT_MODE_CTRL", 11651 }, { "POD_1652_SEARCHLIGHT_MORSE_CTRL", 11652 }, { "POD_1654_SET_HOOK_POINT_CTRL", 11654 }, { "POD_1655_UPDATE_HOOK_POINT_STATE", 11655 }, { "POD_1657_SET_SLOW_DOWN_CTRL", 11657 }, { "POD_1658_UPDATE_SLOW_DOWN_STATE", 11658 }, { "POD_257_IDENTITY_VERIFICATION", 10257 }, { "POD_258_GET_VERSION", 10258 }, { "POD_259_HAND_COMM_LINK", 10259 }, { "POD_260_GET_PRODUCT_INFO", 10260 }, { "POD_261_UPDATE_ADAPTER", 10261 }, { "POD_262_GET_PRODUCT_ALIAS", 10262 }, { "POD_513_PASS_GCS2LOAD", 10513 }, { "POD_PTZ_9010_NAVI_DATA", 9010 }, { "POD_PTZ_9013_RTK", 9013 }, { "POD_PTZ_9014_GPS", 9014 }, { "POINT_AB_ITEMS", 2188 }, { "POINT_AB_ITEMS_ACK", 2189 }, { "POINT_GOTO_ITEMS", 2190 }, { "POINT_GOTO_ITEMS_ACK", 2191 }, { "POINT_HOME_ITEMS", 2194 }, { "POINT_HOME_ITEMS_ACK", 2195 }, { "POINT_LAND_ITEMS", 2192 }, { "POINT_LAND_ITEMS_ACK", 2193 }, { "POS_LOCATION", 2002 }, { "PROJECT_DK13_IMU_RAW", 5100 }, { "PTZ_ATTI", 2300 }, { "PTZ_ATTI_CTRL", 2303 }, { "PTZ_ERROR_CODE", 2307 }, { "PTZ_FUNCTION", 2305 }, { "PTZ_GYRO_CTRL", 2302 }, { "PTZ_POINT_CTRL", 2304 }, { "PTZ_POSITION", 2301 }, { "PTZ_STATE", 2310 }, { "RADAR_SENSORS_DATA", 2174 }, { "RC_INPUT", 2229 }, { "RC_STATUS", 2232 }, { "RC_STATUS_ACK", 2233 }, { "REQUEST_LAND_LOCATION", 2184 }, { "REQUEST_LAND_LOCATION_ACK", 2185 }, { "REQUEST_MODULE_VERSION", 2220 }, { "REQUEST_MODULE_VERSION_ACK", 2221 }, { "REQUEST_POINT_ITEMS", 2186 }, { "REQUEST_POINT_ITEMS_ACK", 2187 }, { "REQUEST_SHARE_ACTIVATE_DATA", 2112 }, { "REQUEST_SHARE_ACTIVATE_DATA_ACK", 2113 }, { "REQUEST_SPRAY_STATUS", 2166 }, { "REQUEST_SPRAY_STATUS_ACK", 2167 }, { "REQUEST_UPLOAD_IMG", 2222 }, { "REQUEST_UPLOAD_IMG_ACK", 2223 }, { "REQUEST_WAYPOINT_ITEMS_FILE", 2212 }, { "REQUEST_WAYPOINT_ITEMS_FILE_ACK", 2213 }, { "REQUEST_WAYPOINT_LIST_FILE", 2210 }, { "REQUEST_WAYPOINT_LIST_FILE_ACK", 2211 }, { "REQUEST_WAYPOINT_MISSION_FILE_ACK", 2215 }, { "REQUEST_WAYPOINT_NOTE_FILE_ACK", 2214 }, { "ROSUAV_BARO_RAW", 8194 }, { "ROSUAV_CTRL_FEED_BACK", 8201 }, { "ROSUAV_CTRL_LOOP_INPUT", 8200 }, { "ROSUAV_CTRL_TARGET_DATA", 8202 }, { "ROSUAV_IMU_RAW", 8193 }, { "ROSUAV_P20_ATTI", 8195 }, { "ROSUAV_P20_NAVI", 8196 }, { "ROSUAV_P20_RAW", 8197 }, { "ROUTE_CONFIG_ITEMS", 2198 }, { "ROUTE_CONFIG_ITEMS_ACK", 2199 }, { "RTK_SENSORS_AUGMENT_DATA", 2177 }, { "RTK_SENSORS_DATA", 2176 }, { "RTK_SENSORS_P20_DATA", 2178 }, { "SEND_POINT_ITEMS", 2218 }, { "SENSORS_DEVICE_DATA", 2175 }, { "SENSOR_CALIBRATION", 2150 }, { "SENSOR_CALIBRATION_ACK", 2151 }, { "SENSOR_INFO", 2129 }, { "SENSOR_MONITOR", 2182 }, { "SENSOR_MONITOR_ACK", 2183 }, { "SENSOR_STATES", 2180 }, { "SENSOR_STATES_ACK", 2181 }, { "SET_DEVICE_SN", 2006 }, { "SET_DEVICE_SN_ACK", 2007 }, { "SET_EX_PWM", 2236 }, { "SET_EX_PWM_ACK", 2237 }, { "SET_WAYPOINT_ITEMS", 2200 }, { "SET_WAYPOINT_ITEMS_ACK", 2201 }, { "SET_WAYPOINT_ITEMS_NOTE", 2206 }, { "SET_WAYPOINT_ITEMS_NOTE_ACK", 2207 }, { "SET_WAYPOINT_MISSION", 2196 }, { "SET_WAYPOINT_MISSION_ACK", 2197 }, { "SINGLE_CHANNEL_FUNCTION_CONFIG", 2259 }, { "STREAM_BARO", 2146 }, { "STREAM_DRONE_STATES", 2143 }, { "STREAM_GPS", 2144 }, { "STREAM_IMU", 2145 }, { "STREAM_MAG", 2142 }, { "STREAM_RC", 2230 }, { "SWITCH_CTRL", 2202 }, { "SWITCH_CTRL_ACK", 2256 }, { "SYNC_ACK", 2130 }, { "SYNC_AUTO_ADJUST_ACTION", 2108 }, { "SYNC_AUTO_ADJUST_INFO", 2107 }, { "SYNC_CALI_ACC", 2120 }, { "SYNC_CALI_FIX", 2125 }, { "SYNC_CALI_HOR", 2123 }, { "SYNC_CALI_MAG", 2121 }, { "SYNC_CALI_RC", 2122 }, { "SYNC_CALI_VER", 2124 }, { "SYNC_CALI_YAW", 2126 }, { "SYNC_CHECK_MODE", 2114 }, { "SYNC_CPUID_INFO", 2100 }, { "SYNC_CTRL_MODE", 2110 }, { "SYNC_CTRL_STATUS_INFO", 2105 }, { "SYNC_FATFS_INFO", 2104 }, { "SYNC_FAULT_INFO", 2103 }, { "SYNC_INTEGRAL_INFO", 2106 }, { "SYNC_MONITOR_INFO", 2102 }, { "SYNC_NAV_ACTIVATE_CHECK_DATA", 2111 }, { "SYNC_TIME", 2004 }, { "SYNC_VERSION_INFO", 2101 }, { "TC_9000_HEART_BEAT", 9000 }, { "TC_9001_LED_STATE", 9001 }, { "TC_9002_NAVI_DATA", 9002 }, { "TC_9003_UAV_STATUS", 9003 }, { "TC_9020_WAY_POINT_DATA", 9020 }, { "TC_9021_SEND_OBSTACLE_DATA", 9021 }, { "TC_9022_MASS_SENSOR_DATA", 9022 }, { "TC_9030_UTC_TIME_DATA", 9030 }, { "TC_9035_PARACHUTE_STATUS", 9035 }, { "TC_9037_SEARCH_LIGHT_STATUS", 9037 }, { "TEST_PWM_OUTPUT", 2262 }, { "UAV_CONFIG_ITEMS", 2084 }, { "UAV_CONFIG_ITEMS_ACK", 2085 }, { "UAV_STATUS_DATA", 2008 }, { "UPDATE_VR_FROM_FC", 2227 }, { "UPDATE_VR_SET_STATUS", 2226 }, { "UPDATE_VR_TO_FC", 2228 }, { "UPLOAD_IMG", 2224 }, { "UPLOAD_IMG_ACK", 2225 }, { "WEATHER_STATION_SENSORS_DATA", 2169 }, { "WP_EVENT_CTRL", 2158 }, { "WP_EVENT_CTRL_ACK", 2159 }, { "WP_PROCESS_INDEX", 2231 }, { "YAW_CMD", 2070 }}
# if TTALINK_COMMAND_24BIT
#  include "../ttalink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // TTALINK_COMMON_H
