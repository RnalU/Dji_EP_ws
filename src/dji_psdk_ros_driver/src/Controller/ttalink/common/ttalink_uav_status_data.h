#pragma once
// MESSAGE UAV_STATUS_DATA PACKING

#define TTALINK_MSG_ID_UAV_STATUS_DATA 2008

TTAPACKED(
typedef struct __ttalink_uav_status_data_t {
 uint32_t update_time; /*<  更新时间.*/
 int64_t sn; /*< 设备序列号.*/
 float uav_home_point_distance; /*<  无人机距离home点距离.*/
 float uav_home_point_angle; /*<  无人机距离home点角度.*/
 float uav_home_land_distance; /*<  无人机距离归航点距离.*/
 float uav_home_land_angle; /*<  无人机距离归航点角度.*/
 float uav_flight_all_distance; /*<  无人机总飞行距离.*/
 float uav_flight_current_distance; /*<  无人机当前飞行距离.*/
 float uav_flight_last_distance; /*<  无人机前一次飞行距离.*/
 float uav_flight_all_time; /*<  无人机总飞行时间.*/
 float uav_flight_current_time; /*<  无人机当前飞行时间.*/
 float uav_flight_last_time; /*<  无人机前一次飞行时间.*/
 uint8_t parachute_check_state; /*<  降落伞 自检状态. 0=通过  0xff=自检中 >1：自检错误码（位域）
			 +------------------------------------------------------------------+
			 | 7 | 6 | 5 | 4 |    3   |       2      |       1      |     0     |
			 +------------------------------------------------------------------+
			 |   |   |   |   |  无伞  | pwm信号异常  | can通信异常  | 供电异常 |
			 +------------------------------------------------------------------+
			*/
 uint8_t parachute_lock_state; /*< 降落伞 开锁状态. 0=自检锁定 1=解锁 2=锁定 0xff=开伞*/
 uint8_t parachute_shield_able; /*<  0=不屏蔽错误，1=屏蔽错误 */
 uint8_t temp_param2; /*<  */
 float param2; /*<  预留.*/
 float param3; /*<  预留.*/
 float param4; /*<  预留.*/
}) ttalink_uav_status_data_t;

#define TTALINK_MSG_ID_UAV_STATUS_DATA_LEN 68
#define TTALINK_MSG_ID_UAV_STATUS_DATA_MIN_LEN 68
#define TTALINK_MSG_ID_2008_LEN 68
#define TTALINK_MSG_ID_2008_MIN_LEN 68

#define TTALINK_MSG_ID_UAV_STATUS_DATA_CRC 33
#define TTALINK_MSG_ID_2008_CRC 33



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_UAV_STATUS_DATA { \
    2008, \
    "UAV_STATUS_DATA", \
    19, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_uav_status_data_t, update_time) }, \
         { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 4, offsetof(ttalink_uav_status_data_t, sn) }, \
         { "uav_home_point_distance", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_uav_status_data_t, uav_home_point_distance) }, \
         { "uav_home_point_angle", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_uav_status_data_t, uav_home_point_angle) }, \
         { "uav_home_land_distance", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_uav_status_data_t, uav_home_land_distance) }, \
         { "uav_home_land_angle", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_uav_status_data_t, uav_home_land_angle) }, \
         { "uav_flight_all_distance", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_uav_status_data_t, uav_flight_all_distance) }, \
         { "uav_flight_current_distance", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_uav_status_data_t, uav_flight_current_distance) }, \
         { "uav_flight_last_distance", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_uav_status_data_t, uav_flight_last_distance) }, \
         { "uav_flight_all_time", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_uav_status_data_t, uav_flight_all_time) }, \
         { "uav_flight_current_time", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_uav_status_data_t, uav_flight_current_time) }, \
         { "uav_flight_last_time", NULL, TTALINK_TYPE_FLOAT, 0, 48, offsetof(ttalink_uav_status_data_t, uav_flight_last_time) }, \
         { "parachute_check_state", NULL, TTALINK_TYPE_UINT8_T, 0, 52, offsetof(ttalink_uav_status_data_t, parachute_check_state) }, \
         { "parachute_lock_state", NULL, TTALINK_TYPE_UINT8_T, 0, 53, offsetof(ttalink_uav_status_data_t, parachute_lock_state) }, \
         { "parachute_shield_able", NULL, TTALINK_TYPE_UINT8_T, 0, 54, offsetof(ttalink_uav_status_data_t, parachute_shield_able) }, \
         { "temp_param2", NULL, TTALINK_TYPE_UINT8_T, 0, 55, offsetof(ttalink_uav_status_data_t, temp_param2) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 56, offsetof(ttalink_uav_status_data_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 60, offsetof(ttalink_uav_status_data_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 64, offsetof(ttalink_uav_status_data_t, param4) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_UAV_STATUS_DATA { \
    "UAV_STATUS_DATA", \
    19, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_uav_status_data_t, update_time) }, \
         { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 4, offsetof(ttalink_uav_status_data_t, sn) }, \
         { "uav_home_point_distance", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_uav_status_data_t, uav_home_point_distance) }, \
         { "uav_home_point_angle", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_uav_status_data_t, uav_home_point_angle) }, \
         { "uav_home_land_distance", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_uav_status_data_t, uav_home_land_distance) }, \
         { "uav_home_land_angle", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_uav_status_data_t, uav_home_land_angle) }, \
         { "uav_flight_all_distance", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_uav_status_data_t, uav_flight_all_distance) }, \
         { "uav_flight_current_distance", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_uav_status_data_t, uav_flight_current_distance) }, \
         { "uav_flight_last_distance", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_uav_status_data_t, uav_flight_last_distance) }, \
         { "uav_flight_all_time", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_uav_status_data_t, uav_flight_all_time) }, \
         { "uav_flight_current_time", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_uav_status_data_t, uav_flight_current_time) }, \
         { "uav_flight_last_time", NULL, TTALINK_TYPE_FLOAT, 0, 48, offsetof(ttalink_uav_status_data_t, uav_flight_last_time) }, \
         { "parachute_check_state", NULL, TTALINK_TYPE_UINT8_T, 0, 52, offsetof(ttalink_uav_status_data_t, parachute_check_state) }, \
         { "parachute_lock_state", NULL, TTALINK_TYPE_UINT8_T, 0, 53, offsetof(ttalink_uav_status_data_t, parachute_lock_state) }, \
         { "parachute_shield_able", NULL, TTALINK_TYPE_UINT8_T, 0, 54, offsetof(ttalink_uav_status_data_t, parachute_shield_able) }, \
         { "temp_param2", NULL, TTALINK_TYPE_UINT8_T, 0, 55, offsetof(ttalink_uav_status_data_t, temp_param2) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 56, offsetof(ttalink_uav_status_data_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 60, offsetof(ttalink_uav_status_data_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 64, offsetof(ttalink_uav_status_data_t, param4) }, \
         } \
}
#endif


static inline uint16_t _ttalink_uav_status_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, int64_t sn, float uav_home_point_distance, float uav_home_point_angle, float uav_home_land_distance, float uav_home_land_angle, float uav_flight_all_distance, float uav_flight_current_distance, float uav_flight_last_distance, float uav_flight_all_time, float uav_flight_current_time, float uav_flight_last_time, uint8_t parachute_check_state, uint8_t parachute_lock_state, uint8_t parachute_shield_able, uint8_t temp_param2, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_UAV_STATUS_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int64_t(buf, 4, sn);
    _tta_put_float(buf, 12, uav_home_point_distance);
    _tta_put_float(buf, 16, uav_home_point_angle);
    _tta_put_float(buf, 20, uav_home_land_distance);
    _tta_put_float(buf, 24, uav_home_land_angle);
    _tta_put_float(buf, 28, uav_flight_all_distance);
    _tta_put_float(buf, 32, uav_flight_current_distance);
    _tta_put_float(buf, 36, uav_flight_last_distance);
    _tta_put_float(buf, 40, uav_flight_all_time);
    _tta_put_float(buf, 44, uav_flight_current_time);
    _tta_put_float(buf, 48, uav_flight_last_time);
    _tta_put_uint8_t(buf, 52, parachute_check_state);
    _tta_put_uint8_t(buf, 53, parachute_lock_state);
    _tta_put_uint8_t(buf, 54, parachute_shield_able);
    _tta_put_uint8_t(buf, 55, temp_param2);
    _tta_put_float(buf, 56, param2);
    _tta_put_float(buf, 60, param3);
    _tta_put_float(buf, 64, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_UAV_STATUS_DATA_LEN);
#else
    ttalink_uav_status_data_t packet;
    packet.update_time = update_time;
    packet.sn = sn;
    packet.uav_home_point_distance = uav_home_point_distance;
    packet.uav_home_point_angle = uav_home_point_angle;
    packet.uav_home_land_distance = uav_home_land_distance;
    packet.uav_home_land_angle = uav_home_land_angle;
    packet.uav_flight_all_distance = uav_flight_all_distance;
    packet.uav_flight_current_distance = uav_flight_current_distance;
    packet.uav_flight_last_distance = uav_flight_last_distance;
    packet.uav_flight_all_time = uav_flight_all_time;
    packet.uav_flight_current_time = uav_flight_current_time;
    packet.uav_flight_last_time = uav_flight_last_time;
    packet.parachute_check_state = parachute_check_state;
    packet.parachute_lock_state = parachute_lock_state;
    packet.parachute_shield_able = parachute_shield_able;
    packet.temp_param2 = temp_param2;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_UAV_STATUS_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_UAV_STATUS_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_UAV_STATUS_DATA_MIN_LEN, TTALINK_MSG_ID_UAV_STATUS_DATA_LEN, TTALINK_MSG_ID_UAV_STATUS_DATA_CRC, nocrc);
}

/**
 * @brief Pack a uav_status_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  更新时间.
 * @param sn 设备序列号.
 * @param uav_home_point_distance  无人机距离home点距离.
 * @param uav_home_point_angle  无人机距离home点角度.
 * @param uav_home_land_distance  无人机距离归航点距离.
 * @param uav_home_land_angle  无人机距离归航点角度.
 * @param uav_flight_all_distance  无人机总飞行距离.
 * @param uav_flight_current_distance  无人机当前飞行距离.
 * @param uav_flight_last_distance  无人机前一次飞行距离.
 * @param uav_flight_all_time  无人机总飞行时间.
 * @param uav_flight_current_time  无人机当前飞行时间.
 * @param uav_flight_last_time  无人机前一次飞行时间.
 * @param parachute_check_state  降落伞 自检状态. 0=通过  0xff=自检中 >1：自检错误码（位域）
			 +------------------------------------------------------------------+
			 | 7 | 6 | 5 | 4 |    3   |       2      |       1      |     0     |
			 +------------------------------------------------------------------+
			 |   |   |   |   |  无伞  | pwm信号异常  | can通信异常  | 供电异常 |
			 +------------------------------------------------------------------+
			
 * @param parachute_lock_state 降落伞 开锁状态. 0=自检锁定 1=解锁 2=锁定 0xff=开伞
 * @param parachute_shield_able  0=不屏蔽错误，1=屏蔽错误 
 * @param temp_param2  
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_uav_status_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, int64_t sn, float uav_home_point_distance, float uav_home_point_angle, float uav_home_land_distance, float uav_home_land_angle, float uav_flight_all_distance, float uav_flight_current_distance, float uav_flight_last_distance, float uav_flight_all_time, float uav_flight_current_time, float uav_flight_last_time, uint8_t parachute_check_state, uint8_t parachute_lock_state, uint8_t parachute_shield_able, uint8_t temp_param2, float param2, float param3, float param4)
{
    return _ttalink_uav_status_data_pack(dst_addr, src_addr, msg,  update_time, sn, uav_home_point_distance, uav_home_point_angle, uav_home_land_distance, uav_home_land_angle, uav_flight_all_distance, uav_flight_current_distance, uav_flight_last_distance, uav_flight_all_time, uav_flight_current_time, uav_flight_last_time, parachute_check_state, parachute_lock_state, parachute_shield_able, temp_param2, param2, param3, param4, false);
}

/**
 * @brief Pack a uav_status_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  更新时间.
 * @param sn 设备序列号.
 * @param uav_home_point_distance  无人机距离home点距离.
 * @param uav_home_point_angle  无人机距离home点角度.
 * @param uav_home_land_distance  无人机距离归航点距离.
 * @param uav_home_land_angle  无人机距离归航点角度.
 * @param uav_flight_all_distance  无人机总飞行距离.
 * @param uav_flight_current_distance  无人机当前飞行距离.
 * @param uav_flight_last_distance  无人机前一次飞行距离.
 * @param uav_flight_all_time  无人机总飞行时间.
 * @param uav_flight_current_time  无人机当前飞行时间.
 * @param uav_flight_last_time  无人机前一次飞行时间.
 * @param parachute_check_state  降落伞 自检状态. 0=通过  0xff=自检中 >1：自检错误码（位域）
			 +------------------------------------------------------------------+
			 | 7 | 6 | 5 | 4 |    3   |       2      |       1      |     0     |
			 +------------------------------------------------------------------+
			 |   |   |   |   |  无伞  | pwm信号异常  | can通信异常  | 供电异常 |
			 +------------------------------------------------------------------+
			
 * @param parachute_lock_state 降落伞 开锁状态. 0=自检锁定 1=解锁 2=锁定 0xff=开伞
 * @param parachute_shield_able  0=不屏蔽错误，1=屏蔽错误 
 * @param temp_param2  
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_uav_status_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, int64_t sn, float uav_home_point_distance, float uav_home_point_angle, float uav_home_land_distance, float uav_home_land_angle, float uav_flight_all_distance, float uav_flight_current_distance, float uav_flight_last_distance, float uav_flight_all_time, float uav_flight_current_time, float uav_flight_last_time, uint8_t parachute_check_state, uint8_t parachute_lock_state, uint8_t parachute_shield_able, uint8_t temp_param2, float param2, float param3, float param4)
{
    return _ttalink_uav_status_data_pack(dst_addr, src_addr, msg,  update_time, sn, uav_home_point_distance, uav_home_point_angle, uav_home_land_distance, uav_home_land_angle, uav_flight_all_distance, uav_flight_current_distance, uav_flight_last_distance, uav_flight_all_time, uav_flight_current_time, uav_flight_last_time, parachute_check_state, parachute_lock_state, parachute_shield_able, temp_param2, param2, param3, param4, true);
}


static inline uint16_t _ttalink_uav_status_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,int64_t sn,float uav_home_point_distance,float uav_home_point_angle,float uav_home_land_distance,float uav_home_land_angle,float uav_flight_all_distance,float uav_flight_current_distance,float uav_flight_last_distance,float uav_flight_all_time,float uav_flight_current_time,float uav_flight_last_time,uint8_t parachute_check_state,uint8_t parachute_lock_state,uint8_t parachute_shield_able,uint8_t temp_param2,float param2,float param3,float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_UAV_STATUS_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int64_t(buf, 4, sn);
    _tta_put_float(buf, 12, uav_home_point_distance);
    _tta_put_float(buf, 16, uav_home_point_angle);
    _tta_put_float(buf, 20, uav_home_land_distance);
    _tta_put_float(buf, 24, uav_home_land_angle);
    _tta_put_float(buf, 28, uav_flight_all_distance);
    _tta_put_float(buf, 32, uav_flight_current_distance);
    _tta_put_float(buf, 36, uav_flight_last_distance);
    _tta_put_float(buf, 40, uav_flight_all_time);
    _tta_put_float(buf, 44, uav_flight_current_time);
    _tta_put_float(buf, 48, uav_flight_last_time);
    _tta_put_uint8_t(buf, 52, parachute_check_state);
    _tta_put_uint8_t(buf, 53, parachute_lock_state);
    _tta_put_uint8_t(buf, 54, parachute_shield_able);
    _tta_put_uint8_t(buf, 55, temp_param2);
    _tta_put_float(buf, 56, param2);
    _tta_put_float(buf, 60, param3);
    _tta_put_float(buf, 64, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_UAV_STATUS_DATA_LEN);
#else
    ttalink_uav_status_data_t packet;
    packet.update_time = update_time;
    packet.sn = sn;
    packet.uav_home_point_distance = uav_home_point_distance;
    packet.uav_home_point_angle = uav_home_point_angle;
    packet.uav_home_land_distance = uav_home_land_distance;
    packet.uav_home_land_angle = uav_home_land_angle;
    packet.uav_flight_all_distance = uav_flight_all_distance;
    packet.uav_flight_current_distance = uav_flight_current_distance;
    packet.uav_flight_last_distance = uav_flight_last_distance;
    packet.uav_flight_all_time = uav_flight_all_time;
    packet.uav_flight_current_time = uav_flight_current_time;
    packet.uav_flight_last_time = uav_flight_last_time;
    packet.parachute_check_state = parachute_check_state;
    packet.parachute_lock_state = parachute_lock_state;
    packet.parachute_shield_able = parachute_shield_able;
    packet.temp_param2 = temp_param2;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_UAV_STATUS_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_UAV_STATUS_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_UAV_STATUS_DATA_MIN_LEN, TTALINK_MSG_ID_UAV_STATUS_DATA_LEN, TTALINK_MSG_ID_UAV_STATUS_DATA_CRC, nocrc);
}

/**
 * @brief Pack a uav_status_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  更新时间.
 * @param sn 设备序列号.
 * @param uav_home_point_distance  无人机距离home点距离.
 * @param uav_home_point_angle  无人机距离home点角度.
 * @param uav_home_land_distance  无人机距离归航点距离.
 * @param uav_home_land_angle  无人机距离归航点角度.
 * @param uav_flight_all_distance  无人机总飞行距离.
 * @param uav_flight_current_distance  无人机当前飞行距离.
 * @param uav_flight_last_distance  无人机前一次飞行距离.
 * @param uav_flight_all_time  无人机总飞行时间.
 * @param uav_flight_current_time  无人机当前飞行时间.
 * @param uav_flight_last_time  无人机前一次飞行时间.
 * @param parachute_check_state  降落伞 自检状态. 0=通过  0xff=自检中 >1：自检错误码（位域）
			 +------------------------------------------------------------------+
			 | 7 | 6 | 5 | 4 |    3   |       2      |       1      |     0     |
			 +------------------------------------------------------------------+
			 |   |   |   |   |  无伞  | pwm信号异常  | can通信异常  | 供电异常 |
			 +------------------------------------------------------------------+
			
 * @param parachute_lock_state 降落伞 开锁状态. 0=自检锁定 1=解锁 2=锁定 0xff=开伞
 * @param parachute_shield_able  0=不屏蔽错误，1=屏蔽错误 
 * @param temp_param2  
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_uav_status_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,int64_t sn,float uav_home_point_distance,float uav_home_point_angle,float uav_home_land_distance,float uav_home_land_angle,float uav_flight_all_distance,float uav_flight_current_distance,float uav_flight_last_distance,float uav_flight_all_time,float uav_flight_current_time,float uav_flight_last_time,uint8_t parachute_check_state,uint8_t parachute_lock_state,uint8_t parachute_shield_able,uint8_t temp_param2,float param2,float param3,float param4)
{
    return _ttalink_uav_status_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, sn, uav_home_point_distance, uav_home_point_angle, uav_home_land_distance, uav_home_land_angle, uav_flight_all_distance, uav_flight_current_distance, uav_flight_last_distance, uav_flight_all_time, uav_flight_current_time, uav_flight_last_time, parachute_check_state, parachute_lock_state, parachute_shield_able, temp_param2, param2, param3, param4, false);
}

/**
 * @brief Pack a uav_status_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  更新时间.
 * @param sn 设备序列号.
 * @param uav_home_point_distance  无人机距离home点距离.
 * @param uav_home_point_angle  无人机距离home点角度.
 * @param uav_home_land_distance  无人机距离归航点距离.
 * @param uav_home_land_angle  无人机距离归航点角度.
 * @param uav_flight_all_distance  无人机总飞行距离.
 * @param uav_flight_current_distance  无人机当前飞行距离.
 * @param uav_flight_last_distance  无人机前一次飞行距离.
 * @param uav_flight_all_time  无人机总飞行时间.
 * @param uav_flight_current_time  无人机当前飞行时间.
 * @param uav_flight_last_time  无人机前一次飞行时间.
 * @param parachute_check_state  降落伞 自检状态. 0=通过  0xff=自检中 >1：自检错误码（位域）
			 +------------------------------------------------------------------+
			 | 7 | 6 | 5 | 4 |    3   |       2      |       1      |     0     |
			 +------------------------------------------------------------------+
			 |   |   |   |   |  无伞  | pwm信号异常  | can通信异常  | 供电异常 |
			 +------------------------------------------------------------------+
			
 * @param parachute_lock_state 降落伞 开锁状态. 0=自检锁定 1=解锁 2=锁定 0xff=开伞
 * @param parachute_shield_able  0=不屏蔽错误，1=屏蔽错误 
 * @param temp_param2  
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_uav_status_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,int64_t sn,float uav_home_point_distance,float uav_home_point_angle,float uav_home_land_distance,float uav_home_land_angle,float uav_flight_all_distance,float uav_flight_current_distance,float uav_flight_last_distance,float uav_flight_all_time,float uav_flight_current_time,float uav_flight_last_time,uint8_t parachute_check_state,uint8_t parachute_lock_state,uint8_t parachute_shield_able,uint8_t temp_param2,float param2,float param3,float param4)
{
    return _ttalink_uav_status_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, sn, uav_home_point_distance, uav_home_point_angle, uav_home_land_distance, uav_home_land_angle, uav_flight_all_distance, uav_flight_current_distance, uav_flight_last_distance, uav_flight_all_time, uav_flight_current_time, uav_flight_last_time, parachute_check_state, parachute_lock_state, parachute_shield_able, temp_param2, param2, param3, param4, true);
}


static inline uint16_t _ttalink_uav_status_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_uav_status_data_t* uav_status_data, bool nocrc)
{
    if(nocrc){
        return ttalink_uav_status_data_pack_nocrc(dst_addr, src_addr, msg, uav_status_data->update_time, uav_status_data->sn, uav_status_data->uav_home_point_distance, uav_status_data->uav_home_point_angle, uav_status_data->uav_home_land_distance, uav_status_data->uav_home_land_angle, uav_status_data->uav_flight_all_distance, uav_status_data->uav_flight_current_distance, uav_status_data->uav_flight_last_distance, uav_status_data->uav_flight_all_time, uav_status_data->uav_flight_current_time, uav_status_data->uav_flight_last_time, uav_status_data->parachute_check_state, uav_status_data->parachute_lock_state, uav_status_data->parachute_shield_able, uav_status_data->temp_param2, uav_status_data->param2, uav_status_data->param3, uav_status_data->param4);
    }else{
        return ttalink_uav_status_data_pack(dst_addr, src_addr, msg, uav_status_data->update_time, uav_status_data->sn, uav_status_data->uav_home_point_distance, uav_status_data->uav_home_point_angle, uav_status_data->uav_home_land_distance, uav_status_data->uav_home_land_angle, uav_status_data->uav_flight_all_distance, uav_status_data->uav_flight_current_distance, uav_status_data->uav_flight_last_distance, uav_status_data->uav_flight_all_time, uav_status_data->uav_flight_current_time, uav_status_data->uav_flight_last_time, uav_status_data->parachute_check_state, uav_status_data->parachute_lock_state, uav_status_data->parachute_shield_able, uav_status_data->temp_param2, uav_status_data->param2, uav_status_data->param3, uav_status_data->param4);
    }
    
}

/**
 * @brief Encode a uav_status_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param uav_status_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_uav_status_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_uav_status_data_t* uav_status_data)
{
    return _ttalink_uav_status_data_encode(dst_addr, src_addr, msg, uav_status_data, false);
}

/**
 * @brief Encode a uav_status_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param uav_status_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_uav_status_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_uav_status_data_t* uav_status_data)
{
    return _ttalink_uav_status_data_encode(dst_addr, src_addr, msg, uav_status_data, true);
}


static inline uint16_t _ttalink_uav_status_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_uav_status_data_t* uav_status_data, bool nocrc)
{
    if(nocrc){
        return ttalink_uav_status_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, uav_status_data->update_time, uav_status_data->sn, uav_status_data->uav_home_point_distance, uav_status_data->uav_home_point_angle, uav_status_data->uav_home_land_distance, uav_status_data->uav_home_land_angle, uav_status_data->uav_flight_all_distance, uav_status_data->uav_flight_current_distance, uav_status_data->uav_flight_last_distance, uav_status_data->uav_flight_all_time, uav_status_data->uav_flight_current_time, uav_status_data->uav_flight_last_time, uav_status_data->parachute_check_state, uav_status_data->parachute_lock_state, uav_status_data->parachute_shield_able, uav_status_data->temp_param2, uav_status_data->param2, uav_status_data->param3, uav_status_data->param4);
    }else{
        return ttalink_uav_status_data_pack_chan(dst_addr, src_addr, chan, msg, uav_status_data->update_time, uav_status_data->sn, uav_status_data->uav_home_point_distance, uav_status_data->uav_home_point_angle, uav_status_data->uav_home_land_distance, uav_status_data->uav_home_land_angle, uav_status_data->uav_flight_all_distance, uav_status_data->uav_flight_current_distance, uav_status_data->uav_flight_last_distance, uav_status_data->uav_flight_all_time, uav_status_data->uav_flight_current_time, uav_status_data->uav_flight_last_time, uav_status_data->parachute_check_state, uav_status_data->parachute_lock_state, uav_status_data->parachute_shield_able, uav_status_data->temp_param2, uav_status_data->param2, uav_status_data->param3, uav_status_data->param4);
    }
}

/**
 * @brief Encode a uav_status_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param uav_status_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_uav_status_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_uav_status_data_t* uav_status_data)
{
    return _ttalink_uav_status_data_encode_chan(dst_addr, src_addr, chan, msg, uav_status_data, false);
}

/**
 * @brief Encode a uav_status_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param uav_status_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_uav_status_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_uav_status_data_t* uav_status_data)
{
    return _ttalink_uav_status_data_encode_chan(dst_addr, src_addr, chan, msg, uav_status_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_uav_status_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, int64_t sn, float uav_home_point_distance, float uav_home_point_angle, float uav_home_land_distance, float uav_home_land_angle, float uav_flight_all_distance, float uav_flight_current_distance, float uav_flight_last_distance, float uav_flight_all_time, float uav_flight_current_time, float uav_flight_last_time, uint8_t parachute_check_state, uint8_t parachute_lock_state, uint8_t parachute_shield_able, uint8_t temp_param2, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_UAV_STATUS_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int64_t(buf, 4, sn);
    _tta_put_float(buf, 12, uav_home_point_distance);
    _tta_put_float(buf, 16, uav_home_point_angle);
    _tta_put_float(buf, 20, uav_home_land_distance);
    _tta_put_float(buf, 24, uav_home_land_angle);
    _tta_put_float(buf, 28, uav_flight_all_distance);
    _tta_put_float(buf, 32, uav_flight_current_distance);
    _tta_put_float(buf, 36, uav_flight_last_distance);
    _tta_put_float(buf, 40, uav_flight_all_time);
    _tta_put_float(buf, 44, uav_flight_current_time);
    _tta_put_float(buf, 48, uav_flight_last_time);
    _tta_put_uint8_t(buf, 52, parachute_check_state);
    _tta_put_uint8_t(buf, 53, parachute_lock_state);
    _tta_put_uint8_t(buf, 54, parachute_shield_able);
    _tta_put_uint8_t(buf, 55, temp_param2);
    _tta_put_float(buf, 56, param2);
    _tta_put_float(buf, 60, param3);
    _tta_put_float(buf, 64, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UAV_STATUS_DATA, buf, TTALINK_MSG_ID_UAV_STATUS_DATA_MIN_LEN, TTALINK_MSG_ID_UAV_STATUS_DATA_LEN, TTALINK_MSG_ID_UAV_STATUS_DATA_CRC, nocrc);
#else
    ttalink_uav_status_data_t packet;
    packet.update_time = update_time;
    packet.sn = sn;
    packet.uav_home_point_distance = uav_home_point_distance;
    packet.uav_home_point_angle = uav_home_point_angle;
    packet.uav_home_land_distance = uav_home_land_distance;
    packet.uav_home_land_angle = uav_home_land_angle;
    packet.uav_flight_all_distance = uav_flight_all_distance;
    packet.uav_flight_current_distance = uav_flight_current_distance;
    packet.uav_flight_last_distance = uav_flight_last_distance;
    packet.uav_flight_all_time = uav_flight_all_time;
    packet.uav_flight_current_time = uav_flight_current_time;
    packet.uav_flight_last_time = uav_flight_last_time;
    packet.parachute_check_state = parachute_check_state;
    packet.parachute_lock_state = parachute_lock_state;
    packet.parachute_shield_able = parachute_shield_able;
    packet.temp_param2 = temp_param2;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UAV_STATUS_DATA, (const char *)&packet, TTALINK_MSG_ID_UAV_STATUS_DATA_MIN_LEN, TTALINK_MSG_ID_UAV_STATUS_DATA_LEN, TTALINK_MSG_ID_UAV_STATUS_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a uav_status_data message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  更新时间.
 * @param sn 设备序列号.
 * @param uav_home_point_distance  无人机距离home点距离.
 * @param uav_home_point_angle  无人机距离home点角度.
 * @param uav_home_land_distance  无人机距离归航点距离.
 * @param uav_home_land_angle  无人机距离归航点角度.
 * @param uav_flight_all_distance  无人机总飞行距离.
 * @param uav_flight_current_distance  无人机当前飞行距离.
 * @param uav_flight_last_distance  无人机前一次飞行距离.
 * @param uav_flight_all_time  无人机总飞行时间.
 * @param uav_flight_current_time  无人机当前飞行时间.
 * @param uav_flight_last_time  无人机前一次飞行时间.
 * @param parachute_check_state  降落伞 自检状态. 0=通过  0xff=自检中 >1：自检错误码（位域）
			 +------------------------------------------------------------------+
			 | 7 | 6 | 5 | 4 |    3   |       2      |       1      |     0     |
			 +------------------------------------------------------------------+
			 |   |   |   |   |  无伞  | pwm信号异常  | can通信异常  | 供电异常 |
			 +------------------------------------------------------------------+
			
 * @param parachute_lock_state 降落伞 开锁状态. 0=自检锁定 1=解锁 2=锁定 0xff=开伞
 * @param parachute_shield_able  0=不屏蔽错误，1=屏蔽错误 
 * @param temp_param2  
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 */
static inline void ttalink_uav_status_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, int64_t sn, float uav_home_point_distance, float uav_home_point_angle, float uav_home_land_distance, float uav_home_land_angle, float uav_flight_all_distance, float uav_flight_current_distance, float uav_flight_last_distance, float uav_flight_all_time, float uav_flight_current_time, float uav_flight_last_time, uint8_t parachute_check_state, uint8_t parachute_lock_state, uint8_t parachute_shield_able, uint8_t temp_param2, float param2, float param3, float param4)
{
    _ttalink_uav_status_data_send(dst_addr, src_addr, chan, update_time, sn, uav_home_point_distance, uav_home_point_angle, uav_home_land_distance, uav_home_land_angle, uav_flight_all_distance, uav_flight_current_distance, uav_flight_last_distance, uav_flight_all_time, uav_flight_current_time, uav_flight_last_time, parachute_check_state, parachute_lock_state, parachute_shield_able, temp_param2, param2, param3, param4, false);
}

/**
 * @brief Send a uav_status_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  更新时间.
 * @param sn 设备序列号.
 * @param uav_home_point_distance  无人机距离home点距离.
 * @param uav_home_point_angle  无人机距离home点角度.
 * @param uav_home_land_distance  无人机距离归航点距离.
 * @param uav_home_land_angle  无人机距离归航点角度.
 * @param uav_flight_all_distance  无人机总飞行距离.
 * @param uav_flight_current_distance  无人机当前飞行距离.
 * @param uav_flight_last_distance  无人机前一次飞行距离.
 * @param uav_flight_all_time  无人机总飞行时间.
 * @param uav_flight_current_time  无人机当前飞行时间.
 * @param uav_flight_last_time  无人机前一次飞行时间.
 * @param parachute_check_state  降落伞 自检状态. 0=通过  0xff=自检中 >1：自检错误码（位域）
			 +------------------------------------------------------------------+
			 | 7 | 6 | 5 | 4 |    3   |       2      |       1      |     0     |
			 +------------------------------------------------------------------+
			 |   |   |   |   |  无伞  | pwm信号异常  | can通信异常  | 供电异常 |
			 +------------------------------------------------------------------+
			
 * @param parachute_lock_state 降落伞 开锁状态. 0=自检锁定 1=解锁 2=锁定 0xff=开伞
 * @param parachute_shield_able  0=不屏蔽错误，1=屏蔽错误 
 * @param temp_param2  
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 */
static inline void ttalink_uav_status_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, int64_t sn, float uav_home_point_distance, float uav_home_point_angle, float uav_home_land_distance, float uav_home_land_angle, float uav_flight_all_distance, float uav_flight_current_distance, float uav_flight_last_distance, float uav_flight_all_time, float uav_flight_current_time, float uav_flight_last_time, uint8_t parachute_check_state, uint8_t parachute_lock_state, uint8_t parachute_shield_able, uint8_t temp_param2, float param2, float param3, float param4)
{
    _ttalink_uav_status_data_send(dst_addr, src_addr, chan, update_time, sn, uav_home_point_distance, uav_home_point_angle, uav_home_land_distance, uav_home_land_angle, uav_flight_all_distance, uav_flight_current_distance, uav_flight_last_distance, uav_flight_all_time, uav_flight_current_time, uav_flight_last_time, parachute_check_state, parachute_lock_state, parachute_shield_able, temp_param2, param2, param3, param4, true);
}


static inline void _ttalink_uav_status_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_uav_status_data_t* uav_status_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_uav_status_data_send_nocrc(dst_addr, src_addr, chan, uav_status_data->update_time, uav_status_data->sn, uav_status_data->uav_home_point_distance, uav_status_data->uav_home_point_angle, uav_status_data->uav_home_land_distance, uav_status_data->uav_home_land_angle, uav_status_data->uav_flight_all_distance, uav_status_data->uav_flight_current_distance, uav_status_data->uav_flight_last_distance, uav_status_data->uav_flight_all_time, uav_status_data->uav_flight_current_time, uav_status_data->uav_flight_last_time, uav_status_data->parachute_check_state, uav_status_data->parachute_lock_state, uav_status_data->parachute_shield_able, uav_status_data->temp_param2, uav_status_data->param2, uav_status_data->param3, uav_status_data->param4);
    }else{
        ttalink_uav_status_data_send(dst_addr, src_addr, chan, uav_status_data->update_time, uav_status_data->sn, uav_status_data->uav_home_point_distance, uav_status_data->uav_home_point_angle, uav_status_data->uav_home_land_distance, uav_status_data->uav_home_land_angle, uav_status_data->uav_flight_all_distance, uav_status_data->uav_flight_current_distance, uav_status_data->uav_flight_last_distance, uav_status_data->uav_flight_all_time, uav_status_data->uav_flight_current_time, uav_status_data->uav_flight_last_time, uav_status_data->parachute_check_state, uav_status_data->parachute_lock_state, uav_status_data->parachute_shield_able, uav_status_data->temp_param2, uav_status_data->param2, uav_status_data->param3, uav_status_data->param4);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UAV_STATUS_DATA, (const char *)uav_status_data, TTALINK_MSG_ID_UAV_STATUS_DATA_MIN_LEN, TTALINK_MSG_ID_UAV_STATUS_DATA_LEN, TTALINK_MSG_ID_UAV_STATUS_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a uav_status_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_uav_status_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_uav_status_data_t* uav_status_data)
{
    _ttalink_uav_status_data_send_struct(dst_addr, src_addr, chan, uav_status_data, false);
}

/**
 * @brief Send a uav_status_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_uav_status_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_uav_status_data_t* uav_status_data)
{
    _ttalink_uav_status_data_send_struct(dst_addr, src_addr, chan, uav_status_data, true);
}

#if TTALINK_MSG_ID_UAV_STATUS_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_uav_status_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, int64_t sn, float uav_home_point_distance, float uav_home_point_angle, float uav_home_land_distance, float uav_home_land_angle, float uav_flight_all_distance, float uav_flight_current_distance, float uav_flight_last_distance, float uav_flight_all_time, float uav_flight_current_time, float uav_flight_last_time, uint8_t parachute_check_state, uint8_t parachute_lock_state, uint8_t parachute_shield_able, uint8_t temp_param2, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int64_t(buf, 4, sn);
    _tta_put_float(buf, 12, uav_home_point_distance);
    _tta_put_float(buf, 16, uav_home_point_angle);
    _tta_put_float(buf, 20, uav_home_land_distance);
    _tta_put_float(buf, 24, uav_home_land_angle);
    _tta_put_float(buf, 28, uav_flight_all_distance);
    _tta_put_float(buf, 32, uav_flight_current_distance);
    _tta_put_float(buf, 36, uav_flight_last_distance);
    _tta_put_float(buf, 40, uav_flight_all_time);
    _tta_put_float(buf, 44, uav_flight_current_time);
    _tta_put_float(buf, 48, uav_flight_last_time);
    _tta_put_uint8_t(buf, 52, parachute_check_state);
    _tta_put_uint8_t(buf, 53, parachute_lock_state);
    _tta_put_uint8_t(buf, 54, parachute_shield_able);
    _tta_put_uint8_t(buf, 55, temp_param2);
    _tta_put_float(buf, 56, param2);
    _tta_put_float(buf, 60, param3);
    _tta_put_float(buf, 64, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UAV_STATUS_DATA, buf, TTALINK_MSG_ID_UAV_STATUS_DATA_MIN_LEN, TTALINK_MSG_ID_UAV_STATUS_DATA_LEN, TTALINK_MSG_ID_UAV_STATUS_DATA_CRC, nocrc);
#else
    ttalink_uav_status_data_t *packet = (ttalink_uav_status_data_t *)msgbuf;
    packet->update_time = update_time;
    packet->sn = sn;
    packet->uav_home_point_distance = uav_home_point_distance;
    packet->uav_home_point_angle = uav_home_point_angle;
    packet->uav_home_land_distance = uav_home_land_distance;
    packet->uav_home_land_angle = uav_home_land_angle;
    packet->uav_flight_all_distance = uav_flight_all_distance;
    packet->uav_flight_current_distance = uav_flight_current_distance;
    packet->uav_flight_last_distance = uav_flight_last_distance;
    packet->uav_flight_all_time = uav_flight_all_time;
    packet->uav_flight_current_time = uav_flight_current_time;
    packet->uav_flight_last_time = uav_flight_last_time;
    packet->parachute_check_state = parachute_check_state;
    packet->parachute_lock_state = parachute_lock_state;
    packet->parachute_shield_able = parachute_shield_able;
    packet->temp_param2 = temp_param2;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UAV_STATUS_DATA, (const char *)packet, TTALINK_MSG_ID_UAV_STATUS_DATA_MIN_LEN, TTALINK_MSG_ID_UAV_STATUS_DATA_LEN, TTALINK_MSG_ID_UAV_STATUS_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_uav_status_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, int64_t sn, float uav_home_point_distance, float uav_home_point_angle, float uav_home_land_distance, float uav_home_land_angle, float uav_flight_all_distance, float uav_flight_current_distance, float uav_flight_last_distance, float uav_flight_all_time, float uav_flight_current_time, float uav_flight_last_time, uint8_t parachute_check_state, uint8_t parachute_lock_state, uint8_t parachute_shield_able, uint8_t temp_param2, float param2, float param3, float param4)
{
    _ttalink_uav_status_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, sn, uav_home_point_distance, uav_home_point_angle, uav_home_land_distance, uav_home_land_angle, uav_flight_all_distance, uav_flight_current_distance, uav_flight_last_distance, uav_flight_all_time, uav_flight_current_time, uav_flight_last_time, parachute_check_state, parachute_lock_state, parachute_shield_able, temp_param2, param2, param3, param4, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_uav_status_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, int64_t sn, float uav_home_point_distance, float uav_home_point_angle, float uav_home_land_distance, float uav_home_land_angle, float uav_flight_all_distance, float uav_flight_current_distance, float uav_flight_last_distance, float uav_flight_all_time, float uav_flight_current_time, float uav_flight_last_time, uint8_t parachute_check_state, uint8_t parachute_lock_state, uint8_t parachute_shield_able, uint8_t temp_param2, float param2, float param3, float param4)
{
    _ttalink_uav_status_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, sn, uav_home_point_distance, uav_home_point_angle, uav_home_land_distance, uav_home_land_angle, uav_flight_all_distance, uav_flight_current_distance, uav_flight_last_distance, uav_flight_all_time, uav_flight_current_time, uav_flight_last_time, parachute_check_state, parachute_lock_state, parachute_shield_able, temp_param2, param2, param3, param4, true);
}
#endif

#endif

// MESSAGE UAV_STATUS_DATA UNPACKING


/**
 * @brief Get field update_time from uav_status_data message
 *
 * @return  更新时间.
 */
static inline uint32_t ttalink_uav_status_data_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field sn from uav_status_data message
 *
 * @return 设备序列号.
 */
static inline int64_t ttalink_uav_status_data_get_sn(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  4);
}

/**
 * @brief Get field uav_home_point_distance from uav_status_data message
 *
 * @return  无人机距离home点距离.
 */
static inline float ttalink_uav_status_data_get_uav_home_point_distance(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  12);
}

/**
 * @brief Get field uav_home_point_angle from uav_status_data message
 *
 * @return  无人机距离home点角度.
 */
static inline float ttalink_uav_status_data_get_uav_home_point_angle(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field uav_home_land_distance from uav_status_data message
 *
 * @return  无人机距离归航点距离.
 */
static inline float ttalink_uav_status_data_get_uav_home_land_distance(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  20);
}

/**
 * @brief Get field uav_home_land_angle from uav_status_data message
 *
 * @return  无人机距离归航点角度.
 */
static inline float ttalink_uav_status_data_get_uav_home_land_angle(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Get field uav_flight_all_distance from uav_status_data message
 *
 * @return  无人机总飞行距离.
 */
static inline float ttalink_uav_status_data_get_uav_flight_all_distance(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  28);
}

/**
 * @brief Get field uav_flight_current_distance from uav_status_data message
 *
 * @return  无人机当前飞行距离.
 */
static inline float ttalink_uav_status_data_get_uav_flight_current_distance(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Get field uav_flight_last_distance from uav_status_data message
 *
 * @return  无人机前一次飞行距离.
 */
static inline float ttalink_uav_status_data_get_uav_flight_last_distance(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  36);
}

/**
 * @brief Get field uav_flight_all_time from uav_status_data message
 *
 * @return  无人机总飞行时间.
 */
static inline float ttalink_uav_status_data_get_uav_flight_all_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  40);
}

/**
 * @brief Get field uav_flight_current_time from uav_status_data message
 *
 * @return  无人机当前飞行时间.
 */
static inline float ttalink_uav_status_data_get_uav_flight_current_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  44);
}

/**
 * @brief Get field uav_flight_last_time from uav_status_data message
 *
 * @return  无人机前一次飞行时间.
 */
static inline float ttalink_uav_status_data_get_uav_flight_last_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  48);
}

/**
 * @brief Get field parachute_check_state from uav_status_data message
 *
 * @return  降落伞 自检状态. 0=通过  0xff=自检中 >1：自检错误码（位域）
			 +------------------------------------------------------------------+
			 | 7 | 6 | 5 | 4 |    3   |       2      |       1      |     0     |
			 +------------------------------------------------------------------+
			 |   |   |   |   |  无伞  | pwm信号异常  | can通信异常  | 供电异常 |
			 +------------------------------------------------------------------+
			
 */
static inline uint8_t ttalink_uav_status_data_get_parachute_check_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  52);
}

/**
 * @brief Get field parachute_lock_state from uav_status_data message
 *
 * @return 降落伞 开锁状态. 0=自检锁定 1=解锁 2=锁定 0xff=开伞
 */
static inline uint8_t ttalink_uav_status_data_get_parachute_lock_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  53);
}

/**
 * @brief Get field parachute_shield_able from uav_status_data message
 *
 * @return  0=不屏蔽错误，1=屏蔽错误 
 */
static inline uint8_t ttalink_uav_status_data_get_parachute_shield_able(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  54);
}

/**
 * @brief Get field temp_param2 from uav_status_data message
 *
 * @return  
 */
static inline uint8_t ttalink_uav_status_data_get_temp_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  55);
}

/**
 * @brief Get field param2 from uav_status_data message
 *
 * @return  预留.
 */
static inline float ttalink_uav_status_data_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  56);
}

/**
 * @brief Get field param3 from uav_status_data message
 *
 * @return  预留.
 */
static inline float ttalink_uav_status_data_get_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  60);
}

/**
 * @brief Get field param4 from uav_status_data message
 *
 * @return  预留.
 */
static inline float ttalink_uav_status_data_get_param4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  64);
}

/**
 * @brief Decode a uav_status_data message into a struct
 *
 * @param msg The message to decode
 * @param uav_status_data C-struct to decode the message contents into
 */
static inline void ttalink_uav_status_data_decode(const ttalink_message_t* msg, ttalink_uav_status_data_t* uav_status_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    uav_status_data->update_time = ttalink_uav_status_data_get_update_time(msg);
    uav_status_data->sn = ttalink_uav_status_data_get_sn(msg);
    uav_status_data->uav_home_point_distance = ttalink_uav_status_data_get_uav_home_point_distance(msg);
    uav_status_data->uav_home_point_angle = ttalink_uav_status_data_get_uav_home_point_angle(msg);
    uav_status_data->uav_home_land_distance = ttalink_uav_status_data_get_uav_home_land_distance(msg);
    uav_status_data->uav_home_land_angle = ttalink_uav_status_data_get_uav_home_land_angle(msg);
    uav_status_data->uav_flight_all_distance = ttalink_uav_status_data_get_uav_flight_all_distance(msg);
    uav_status_data->uav_flight_current_distance = ttalink_uav_status_data_get_uav_flight_current_distance(msg);
    uav_status_data->uav_flight_last_distance = ttalink_uav_status_data_get_uav_flight_last_distance(msg);
    uav_status_data->uav_flight_all_time = ttalink_uav_status_data_get_uav_flight_all_time(msg);
    uav_status_data->uav_flight_current_time = ttalink_uav_status_data_get_uav_flight_current_time(msg);
    uav_status_data->uav_flight_last_time = ttalink_uav_status_data_get_uav_flight_last_time(msg);
    uav_status_data->parachute_check_state = ttalink_uav_status_data_get_parachute_check_state(msg);
    uav_status_data->parachute_lock_state = ttalink_uav_status_data_get_parachute_lock_state(msg);
    uav_status_data->parachute_shield_able = ttalink_uav_status_data_get_parachute_shield_able(msg);
    uav_status_data->temp_param2 = ttalink_uav_status_data_get_temp_param2(msg);
    uav_status_data->param2 = ttalink_uav_status_data_get_param2(msg);
    uav_status_data->param3 = ttalink_uav_status_data_get_param3(msg);
    uav_status_data->param4 = ttalink_uav_status_data_get_param4(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_UAV_STATUS_DATA_LEN? msg->len : TTALINK_MSG_ID_UAV_STATUS_DATA_LEN;
        memset(uav_status_data, 0, TTALINK_MSG_ID_UAV_STATUS_DATA_LEN);
    memcpy(uav_status_data, _TTA_PAYLOAD(msg), len);
#endif
}
