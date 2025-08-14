#pragma once
// MESSAGE TC_9035_PARACHUTE_STATUS PACKING

#define TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS 9035

TTAPACKED(
typedef struct __ttalink_tc_9035_parachute_status_t {
 uint32_t update_time; /*<  update_time*/
 uint8_t lock_cmd; /*<  0=NULL 1=解锁 2=锁定 0xff=开伞*/
 uint8_t lock_state; /*<  0=NULL 1=解锁 2=锁定 0xff=开伞*/
 uint8_t check_state; /*<  0=通过  0xff=自检中 >1：自检错误码（位域）
			 +------------------------------------------------------------------+
			 | 7 | 6 | 5 | 4 |    3   |       2      |       1      |     0     |
			 +------------------------------------------------------------------+
			 |   |   |   |   |  无伞  | pwm信号异常  | can通信异常  | 供电异常 |
			 +------------------------------------------------------------------+
			*/
 float vaule; /*<  数值*/
 uint8_t parachute_shield_able; /*<  0=不屏蔽错误，1=屏蔽错误 */
 uint8_t temp_param1; /*<  */
 uint8_t temp_param2; /*<  */
 uint8_t temp_param3; /*<  */
 float param2; /*<  预留参数*/
 float param3; /*<  预留参数*/
 float param4; /*<  预留参数*/
}) ttalink_tc_9035_parachute_status_t;

#define TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN 27
#define TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_MIN_LEN 27
#define TTALINK_MSG_ID_9035_LEN 27
#define TTALINK_MSG_ID_9035_MIN_LEN 27

#define TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_CRC 55
#define TTALINK_MSG_ID_9035_CRC 55



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_TC_9035_PARACHUTE_STATUS { \
    9035, \
    "TC_9035_PARACHUTE_STATUS", \
    12, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_tc_9035_parachute_status_t, update_time) }, \
         { "lock_cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_tc_9035_parachute_status_t, lock_cmd) }, \
         { "lock_state", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_tc_9035_parachute_status_t, lock_state) }, \
         { "check_state", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_tc_9035_parachute_status_t, check_state) }, \
         { "vaule", NULL, TTALINK_TYPE_FLOAT, 0, 7, offsetof(ttalink_tc_9035_parachute_status_t, vaule) }, \
         { "parachute_shield_able", NULL, TTALINK_TYPE_UINT8_T, 0, 11, offsetof(ttalink_tc_9035_parachute_status_t, parachute_shield_able) }, \
         { "temp_param1", NULL, TTALINK_TYPE_UINT8_T, 0, 12, offsetof(ttalink_tc_9035_parachute_status_t, temp_param1) }, \
         { "temp_param2", NULL, TTALINK_TYPE_UINT8_T, 0, 13, offsetof(ttalink_tc_9035_parachute_status_t, temp_param2) }, \
         { "temp_param3", NULL, TTALINK_TYPE_UINT8_T, 0, 14, offsetof(ttalink_tc_9035_parachute_status_t, temp_param3) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 15, offsetof(ttalink_tc_9035_parachute_status_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 19, offsetof(ttalink_tc_9035_parachute_status_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 23, offsetof(ttalink_tc_9035_parachute_status_t, param4) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_TC_9035_PARACHUTE_STATUS { \
    "TC_9035_PARACHUTE_STATUS", \
    12, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_tc_9035_parachute_status_t, update_time) }, \
         { "lock_cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_tc_9035_parachute_status_t, lock_cmd) }, \
         { "lock_state", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_tc_9035_parachute_status_t, lock_state) }, \
         { "check_state", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_tc_9035_parachute_status_t, check_state) }, \
         { "vaule", NULL, TTALINK_TYPE_FLOAT, 0, 7, offsetof(ttalink_tc_9035_parachute_status_t, vaule) }, \
         { "parachute_shield_able", NULL, TTALINK_TYPE_UINT8_T, 0, 11, offsetof(ttalink_tc_9035_parachute_status_t, parachute_shield_able) }, \
         { "temp_param1", NULL, TTALINK_TYPE_UINT8_T, 0, 12, offsetof(ttalink_tc_9035_parachute_status_t, temp_param1) }, \
         { "temp_param2", NULL, TTALINK_TYPE_UINT8_T, 0, 13, offsetof(ttalink_tc_9035_parachute_status_t, temp_param2) }, \
         { "temp_param3", NULL, TTALINK_TYPE_UINT8_T, 0, 14, offsetof(ttalink_tc_9035_parachute_status_t, temp_param3) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 15, offsetof(ttalink_tc_9035_parachute_status_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 19, offsetof(ttalink_tc_9035_parachute_status_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 23, offsetof(ttalink_tc_9035_parachute_status_t, param4) }, \
         } \
}
#endif


static inline uint16_t _ttalink_tc_9035_parachute_status_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t lock_cmd, uint8_t lock_state, uint8_t check_state, float vaule, uint8_t parachute_shield_able, uint8_t temp_param1, uint8_t temp_param2, uint8_t temp_param3, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, lock_cmd);
    _tta_put_uint8_t(buf, 5, lock_state);
    _tta_put_uint8_t(buf, 6, check_state);
    _tta_put_float(buf, 7, vaule);
    _tta_put_uint8_t(buf, 11, parachute_shield_able);
    _tta_put_uint8_t(buf, 12, temp_param1);
    _tta_put_uint8_t(buf, 13, temp_param2);
    _tta_put_uint8_t(buf, 14, temp_param3);
    _tta_put_float(buf, 15, param2);
    _tta_put_float(buf, 19, param3);
    _tta_put_float(buf, 23, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN);
#else
    ttalink_tc_9035_parachute_status_t packet;
    packet.update_time = update_time;
    packet.lock_cmd = lock_cmd;
    packet.lock_state = lock_state;
    packet.check_state = check_state;
    packet.vaule = vaule;
    packet.parachute_shield_able = parachute_shield_able;
    packet.temp_param1 = temp_param1;
    packet.temp_param2 = temp_param2;
    packet.temp_param3 = temp_param3;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_MIN_LEN, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_CRC, nocrc);
}

/**
 * @brief Pack a tc_9035_parachute_status message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param lock_cmd  0=NULL 1=解锁 2=锁定 0xff=开伞
 * @param lock_state  0=NULL 1=解锁 2=锁定 0xff=开伞
 * @param check_state  0=通过  0xff=自检中 >1：自检错误码（位域）
			 +------------------------------------------------------------------+
			 | 7 | 6 | 5 | 4 |    3   |       2      |       1      |     0     |
			 +------------------------------------------------------------------+
			 |   |   |   |   |  无伞  | pwm信号异常  | can通信异常  | 供电异常 |
			 +------------------------------------------------------------------+
			
 * @param vaule  数值
 * @param parachute_shield_able  0=不屏蔽错误，1=屏蔽错误 
 * @param temp_param1  
 * @param temp_param2  
 * @param temp_param3  
 * @param param2  预留参数
 * @param param3  预留参数
 * @param param4  预留参数
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9035_parachute_status_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t lock_cmd, uint8_t lock_state, uint8_t check_state, float vaule, uint8_t parachute_shield_able, uint8_t temp_param1, uint8_t temp_param2, uint8_t temp_param3, float param2, float param3, float param4)
{
    return _ttalink_tc_9035_parachute_status_pack(dst_addr, src_addr, msg,  update_time, lock_cmd, lock_state, check_state, vaule, parachute_shield_able, temp_param1, temp_param2, temp_param3, param2, param3, param4, false);
}

/**
 * @brief Pack a tc_9035_parachute_status message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param lock_cmd  0=NULL 1=解锁 2=锁定 0xff=开伞
 * @param lock_state  0=NULL 1=解锁 2=锁定 0xff=开伞
 * @param check_state  0=通过  0xff=自检中 >1：自检错误码（位域）
			 +------------------------------------------------------------------+
			 | 7 | 6 | 5 | 4 |    3   |       2      |       1      |     0     |
			 +------------------------------------------------------------------+
			 |   |   |   |   |  无伞  | pwm信号异常  | can通信异常  | 供电异常 |
			 +------------------------------------------------------------------+
			
 * @param vaule  数值
 * @param parachute_shield_able  0=不屏蔽错误，1=屏蔽错误 
 * @param temp_param1  
 * @param temp_param2  
 * @param temp_param3  
 * @param param2  预留参数
 * @param param3  预留参数
 * @param param4  预留参数
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9035_parachute_status_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t lock_cmd, uint8_t lock_state, uint8_t check_state, float vaule, uint8_t parachute_shield_able, uint8_t temp_param1, uint8_t temp_param2, uint8_t temp_param3, float param2, float param3, float param4)
{
    return _ttalink_tc_9035_parachute_status_pack(dst_addr, src_addr, msg,  update_time, lock_cmd, lock_state, check_state, vaule, parachute_shield_able, temp_param1, temp_param2, temp_param3, param2, param3, param4, true);
}


static inline uint16_t _ttalink_tc_9035_parachute_status_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t lock_cmd,uint8_t lock_state,uint8_t check_state,float vaule,uint8_t parachute_shield_able,uint8_t temp_param1,uint8_t temp_param2,uint8_t temp_param3,float param2,float param3,float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, lock_cmd);
    _tta_put_uint8_t(buf, 5, lock_state);
    _tta_put_uint8_t(buf, 6, check_state);
    _tta_put_float(buf, 7, vaule);
    _tta_put_uint8_t(buf, 11, parachute_shield_able);
    _tta_put_uint8_t(buf, 12, temp_param1);
    _tta_put_uint8_t(buf, 13, temp_param2);
    _tta_put_uint8_t(buf, 14, temp_param3);
    _tta_put_float(buf, 15, param2);
    _tta_put_float(buf, 19, param3);
    _tta_put_float(buf, 23, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN);
#else
    ttalink_tc_9035_parachute_status_t packet;
    packet.update_time = update_time;
    packet.lock_cmd = lock_cmd;
    packet.lock_state = lock_state;
    packet.check_state = check_state;
    packet.vaule = vaule;
    packet.parachute_shield_able = parachute_shield_able;
    packet.temp_param1 = temp_param1;
    packet.temp_param2 = temp_param2;
    packet.temp_param3 = temp_param3;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_MIN_LEN, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_CRC, nocrc);
}

/**
 * @brief Pack a tc_9035_parachute_status message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param lock_cmd  0=NULL 1=解锁 2=锁定 0xff=开伞
 * @param lock_state  0=NULL 1=解锁 2=锁定 0xff=开伞
 * @param check_state  0=通过  0xff=自检中 >1：自检错误码（位域）
			 +------------------------------------------------------------------+
			 | 7 | 6 | 5 | 4 |    3   |       2      |       1      |     0     |
			 +------------------------------------------------------------------+
			 |   |   |   |   |  无伞  | pwm信号异常  | can通信异常  | 供电异常 |
			 +------------------------------------------------------------------+
			
 * @param vaule  数值
 * @param parachute_shield_able  0=不屏蔽错误，1=屏蔽错误 
 * @param temp_param1  
 * @param temp_param2  
 * @param temp_param3  
 * @param param2  预留参数
 * @param param3  预留参数
 * @param param4  预留参数
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9035_parachute_status_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t lock_cmd,uint8_t lock_state,uint8_t check_state,float vaule,uint8_t parachute_shield_able,uint8_t temp_param1,uint8_t temp_param2,uint8_t temp_param3,float param2,float param3,float param4)
{
    return _ttalink_tc_9035_parachute_status_pack_chan(dst_addr, src_addr, chan, msg,  update_time, lock_cmd, lock_state, check_state, vaule, parachute_shield_able, temp_param1, temp_param2, temp_param3, param2, param3, param4, false);
}

/**
 * @brief Pack a tc_9035_parachute_status message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param lock_cmd  0=NULL 1=解锁 2=锁定 0xff=开伞
 * @param lock_state  0=NULL 1=解锁 2=锁定 0xff=开伞
 * @param check_state  0=通过  0xff=自检中 >1：自检错误码（位域）
			 +------------------------------------------------------------------+
			 | 7 | 6 | 5 | 4 |    3   |       2      |       1      |     0     |
			 +------------------------------------------------------------------+
			 |   |   |   |   |  无伞  | pwm信号异常  | can通信异常  | 供电异常 |
			 +------------------------------------------------------------------+
			
 * @param vaule  数值
 * @param parachute_shield_able  0=不屏蔽错误，1=屏蔽错误 
 * @param temp_param1  
 * @param temp_param2  
 * @param temp_param3  
 * @param param2  预留参数
 * @param param3  预留参数
 * @param param4  预留参数
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9035_parachute_status_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t lock_cmd,uint8_t lock_state,uint8_t check_state,float vaule,uint8_t parachute_shield_able,uint8_t temp_param1,uint8_t temp_param2,uint8_t temp_param3,float param2,float param3,float param4)
{
    return _ttalink_tc_9035_parachute_status_pack_chan(dst_addr, src_addr, chan, msg,  update_time, lock_cmd, lock_state, check_state, vaule, parachute_shield_able, temp_param1, temp_param2, temp_param3, param2, param3, param4, true);
}


static inline uint16_t _ttalink_tc_9035_parachute_status_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9035_parachute_status_t* tc_9035_parachute_status, bool nocrc)
{
    if(nocrc){
        return ttalink_tc_9035_parachute_status_pack_nocrc(dst_addr, src_addr, msg, tc_9035_parachute_status->update_time, tc_9035_parachute_status->lock_cmd, tc_9035_parachute_status->lock_state, tc_9035_parachute_status->check_state, tc_9035_parachute_status->vaule, tc_9035_parachute_status->parachute_shield_able, tc_9035_parachute_status->temp_param1, tc_9035_parachute_status->temp_param2, tc_9035_parachute_status->temp_param3, tc_9035_parachute_status->param2, tc_9035_parachute_status->param3, tc_9035_parachute_status->param4);
    }else{
        return ttalink_tc_9035_parachute_status_pack(dst_addr, src_addr, msg, tc_9035_parachute_status->update_time, tc_9035_parachute_status->lock_cmd, tc_9035_parachute_status->lock_state, tc_9035_parachute_status->check_state, tc_9035_parachute_status->vaule, tc_9035_parachute_status->parachute_shield_able, tc_9035_parachute_status->temp_param1, tc_9035_parachute_status->temp_param2, tc_9035_parachute_status->temp_param3, tc_9035_parachute_status->param2, tc_9035_parachute_status->param3, tc_9035_parachute_status->param4);
    }
    
}

/**
 * @brief Encode a tc_9035_parachute_status struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param tc_9035_parachute_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9035_parachute_status_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9035_parachute_status_t* tc_9035_parachute_status)
{
    return _ttalink_tc_9035_parachute_status_encode(dst_addr, src_addr, msg, tc_9035_parachute_status, false);
}

/**
 * @brief Encode a tc_9035_parachute_status struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param tc_9035_parachute_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9035_parachute_status_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9035_parachute_status_t* tc_9035_parachute_status)
{
    return _ttalink_tc_9035_parachute_status_encode(dst_addr, src_addr, msg, tc_9035_parachute_status, true);
}


static inline uint16_t _ttalink_tc_9035_parachute_status_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9035_parachute_status_t* tc_9035_parachute_status, bool nocrc)
{
    if(nocrc){
        return ttalink_tc_9035_parachute_status_pack_chan_nocrc(dst_addr, src_addr, chan, msg, tc_9035_parachute_status->update_time, tc_9035_parachute_status->lock_cmd, tc_9035_parachute_status->lock_state, tc_9035_parachute_status->check_state, tc_9035_parachute_status->vaule, tc_9035_parachute_status->parachute_shield_able, tc_9035_parachute_status->temp_param1, tc_9035_parachute_status->temp_param2, tc_9035_parachute_status->temp_param3, tc_9035_parachute_status->param2, tc_9035_parachute_status->param3, tc_9035_parachute_status->param4);
    }else{
        return ttalink_tc_9035_parachute_status_pack_chan(dst_addr, src_addr, chan, msg, tc_9035_parachute_status->update_time, tc_9035_parachute_status->lock_cmd, tc_9035_parachute_status->lock_state, tc_9035_parachute_status->check_state, tc_9035_parachute_status->vaule, tc_9035_parachute_status->parachute_shield_able, tc_9035_parachute_status->temp_param1, tc_9035_parachute_status->temp_param2, tc_9035_parachute_status->temp_param3, tc_9035_parachute_status->param2, tc_9035_parachute_status->param3, tc_9035_parachute_status->param4);
    }
}

/**
 * @brief Encode a tc_9035_parachute_status struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param tc_9035_parachute_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9035_parachute_status_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9035_parachute_status_t* tc_9035_parachute_status)
{
    return _ttalink_tc_9035_parachute_status_encode_chan(dst_addr, src_addr, chan, msg, tc_9035_parachute_status, false);
}

/**
 * @brief Encode a tc_9035_parachute_status struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param tc_9035_parachute_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9035_parachute_status_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9035_parachute_status_t* tc_9035_parachute_status)
{
    return _ttalink_tc_9035_parachute_status_encode_chan(dst_addr, src_addr, chan, msg, tc_9035_parachute_status, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_tc_9035_parachute_status_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t lock_cmd, uint8_t lock_state, uint8_t check_state, float vaule, uint8_t parachute_shield_able, uint8_t temp_param1, uint8_t temp_param2, uint8_t temp_param3, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, lock_cmd);
    _tta_put_uint8_t(buf, 5, lock_state);
    _tta_put_uint8_t(buf, 6, check_state);
    _tta_put_float(buf, 7, vaule);
    _tta_put_uint8_t(buf, 11, parachute_shield_able);
    _tta_put_uint8_t(buf, 12, temp_param1);
    _tta_put_uint8_t(buf, 13, temp_param2);
    _tta_put_uint8_t(buf, 14, temp_param3);
    _tta_put_float(buf, 15, param2);
    _tta_put_float(buf, 19, param3);
    _tta_put_float(buf, 23, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS, buf, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_MIN_LEN, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_CRC, nocrc);
#else
    ttalink_tc_9035_parachute_status_t packet;
    packet.update_time = update_time;
    packet.lock_cmd = lock_cmd;
    packet.lock_state = lock_state;
    packet.check_state = check_state;
    packet.vaule = vaule;
    packet.parachute_shield_able = parachute_shield_able;
    packet.temp_param1 = temp_param1;
    packet.temp_param2 = temp_param2;
    packet.temp_param3 = temp_param3;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS, (const char *)&packet, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_MIN_LEN, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_CRC, nocrc);
#endif
}

/**
 * @brief Send a tc_9035_parachute_status message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param lock_cmd  0=NULL 1=解锁 2=锁定 0xff=开伞
 * @param lock_state  0=NULL 1=解锁 2=锁定 0xff=开伞
 * @param check_state  0=通过  0xff=自检中 >1：自检错误码（位域）
			 +------------------------------------------------------------------+
			 | 7 | 6 | 5 | 4 |    3   |       2      |       1      |     0     |
			 +------------------------------------------------------------------+
			 |   |   |   |   |  无伞  | pwm信号异常  | can通信异常  | 供电异常 |
			 +------------------------------------------------------------------+
			
 * @param vaule  数值
 * @param parachute_shield_able  0=不屏蔽错误，1=屏蔽错误 
 * @param temp_param1  
 * @param temp_param2  
 * @param temp_param3  
 * @param param2  预留参数
 * @param param3  预留参数
 * @param param4  预留参数
 */
static inline void ttalink_tc_9035_parachute_status_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t lock_cmd, uint8_t lock_state, uint8_t check_state, float vaule, uint8_t parachute_shield_able, uint8_t temp_param1, uint8_t temp_param2, uint8_t temp_param3, float param2, float param3, float param4)
{
    _ttalink_tc_9035_parachute_status_send(dst_addr, src_addr, chan, update_time, lock_cmd, lock_state, check_state, vaule, parachute_shield_able, temp_param1, temp_param2, temp_param3, param2, param3, param4, false);
}

/**
 * @brief Send a tc_9035_parachute_status message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param lock_cmd  0=NULL 1=解锁 2=锁定 0xff=开伞
 * @param lock_state  0=NULL 1=解锁 2=锁定 0xff=开伞
 * @param check_state  0=通过  0xff=自检中 >1：自检错误码（位域）
			 +------------------------------------------------------------------+
			 | 7 | 6 | 5 | 4 |    3   |       2      |       1      |     0     |
			 +------------------------------------------------------------------+
			 |   |   |   |   |  无伞  | pwm信号异常  | can通信异常  | 供电异常 |
			 +------------------------------------------------------------------+
			
 * @param vaule  数值
 * @param parachute_shield_able  0=不屏蔽错误，1=屏蔽错误 
 * @param temp_param1  
 * @param temp_param2  
 * @param temp_param3  
 * @param param2  预留参数
 * @param param3  预留参数
 * @param param4  预留参数
 */
static inline void ttalink_tc_9035_parachute_status_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t lock_cmd, uint8_t lock_state, uint8_t check_state, float vaule, uint8_t parachute_shield_able, uint8_t temp_param1, uint8_t temp_param2, uint8_t temp_param3, float param2, float param3, float param4)
{
    _ttalink_tc_9035_parachute_status_send(dst_addr, src_addr, chan, update_time, lock_cmd, lock_state, check_state, vaule, parachute_shield_able, temp_param1, temp_param2, temp_param3, param2, param3, param4, true);
}


static inline void _ttalink_tc_9035_parachute_status_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9035_parachute_status_t* tc_9035_parachute_status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_tc_9035_parachute_status_send_nocrc(dst_addr, src_addr, chan, tc_9035_parachute_status->update_time, tc_9035_parachute_status->lock_cmd, tc_9035_parachute_status->lock_state, tc_9035_parachute_status->check_state, tc_9035_parachute_status->vaule, tc_9035_parachute_status->parachute_shield_able, tc_9035_parachute_status->temp_param1, tc_9035_parachute_status->temp_param2, tc_9035_parachute_status->temp_param3, tc_9035_parachute_status->param2, tc_9035_parachute_status->param3, tc_9035_parachute_status->param4);
    }else{
        ttalink_tc_9035_parachute_status_send(dst_addr, src_addr, chan, tc_9035_parachute_status->update_time, tc_9035_parachute_status->lock_cmd, tc_9035_parachute_status->lock_state, tc_9035_parachute_status->check_state, tc_9035_parachute_status->vaule, tc_9035_parachute_status->parachute_shield_able, tc_9035_parachute_status->temp_param1, tc_9035_parachute_status->temp_param2, tc_9035_parachute_status->temp_param3, tc_9035_parachute_status->param2, tc_9035_parachute_status->param3, tc_9035_parachute_status->param4);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS, (const char *)tc_9035_parachute_status, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_MIN_LEN, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_CRC, nocrc);
#endif
}

/**
 * @brief Send a tc_9035_parachute_status message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_tc_9035_parachute_status_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9035_parachute_status_t* tc_9035_parachute_status)
{
    _ttalink_tc_9035_parachute_status_send_struct(dst_addr, src_addr, chan, tc_9035_parachute_status, false);
}

/**
 * @brief Send a tc_9035_parachute_status message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_tc_9035_parachute_status_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9035_parachute_status_t* tc_9035_parachute_status)
{
    _ttalink_tc_9035_parachute_status_send_struct(dst_addr, src_addr, chan, tc_9035_parachute_status, true);
}

#if TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_tc_9035_parachute_status_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t lock_cmd, uint8_t lock_state, uint8_t check_state, float vaule, uint8_t parachute_shield_able, uint8_t temp_param1, uint8_t temp_param2, uint8_t temp_param3, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, lock_cmd);
    _tta_put_uint8_t(buf, 5, lock_state);
    _tta_put_uint8_t(buf, 6, check_state);
    _tta_put_float(buf, 7, vaule);
    _tta_put_uint8_t(buf, 11, parachute_shield_able);
    _tta_put_uint8_t(buf, 12, temp_param1);
    _tta_put_uint8_t(buf, 13, temp_param2);
    _tta_put_uint8_t(buf, 14, temp_param3);
    _tta_put_float(buf, 15, param2);
    _tta_put_float(buf, 19, param3);
    _tta_put_float(buf, 23, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS, buf, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_MIN_LEN, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_CRC, nocrc);
#else
    ttalink_tc_9035_parachute_status_t *packet = (ttalink_tc_9035_parachute_status_t *)msgbuf;
    packet->update_time = update_time;
    packet->lock_cmd = lock_cmd;
    packet->lock_state = lock_state;
    packet->check_state = check_state;
    packet->vaule = vaule;
    packet->parachute_shield_able = parachute_shield_able;
    packet->temp_param1 = temp_param1;
    packet->temp_param2 = temp_param2;
    packet->temp_param3 = temp_param3;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS, (const char *)packet, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_MIN_LEN, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_tc_9035_parachute_status_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t lock_cmd, uint8_t lock_state, uint8_t check_state, float vaule, uint8_t parachute_shield_able, uint8_t temp_param1, uint8_t temp_param2, uint8_t temp_param3, float param2, float param3, float param4)
{
    _ttalink_tc_9035_parachute_status_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, lock_cmd, lock_state, check_state, vaule, parachute_shield_able, temp_param1, temp_param2, temp_param3, param2, param3, param4, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_tc_9035_parachute_status_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t lock_cmd, uint8_t lock_state, uint8_t check_state, float vaule, uint8_t parachute_shield_able, uint8_t temp_param1, uint8_t temp_param2, uint8_t temp_param3, float param2, float param3, float param4)
{
    _ttalink_tc_9035_parachute_status_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, lock_cmd, lock_state, check_state, vaule, parachute_shield_able, temp_param1, temp_param2, temp_param3, param2, param3, param4, true);
}
#endif

#endif

// MESSAGE TC_9035_PARACHUTE_STATUS UNPACKING


/**
 * @brief Get field update_time from tc_9035_parachute_status message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_tc_9035_parachute_status_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field lock_cmd from tc_9035_parachute_status message
 *
 * @return  0=NULL 1=解锁 2=锁定 0xff=开伞
 */
static inline uint8_t ttalink_tc_9035_parachute_status_get_lock_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field lock_state from tc_9035_parachute_status message
 *
 * @return  0=NULL 1=解锁 2=锁定 0xff=开伞
 */
static inline uint8_t ttalink_tc_9035_parachute_status_get_lock_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field check_state from tc_9035_parachute_status message
 *
 * @return  0=通过  0xff=自检中 >1：自检错误码（位域）
			 +------------------------------------------------------------------+
			 | 7 | 6 | 5 | 4 |    3   |       2      |       1      |     0     |
			 +------------------------------------------------------------------+
			 |   |   |   |   |  无伞  | pwm信号异常  | can通信异常  | 供电异常 |
			 +------------------------------------------------------------------+
			
 */
static inline uint8_t ttalink_tc_9035_parachute_status_get_check_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field vaule from tc_9035_parachute_status message
 *
 * @return  数值
 */
static inline float ttalink_tc_9035_parachute_status_get_vaule(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  7);
}

/**
 * @brief Get field parachute_shield_able from tc_9035_parachute_status message
 *
 * @return  0=不屏蔽错误，1=屏蔽错误 
 */
static inline uint8_t ttalink_tc_9035_parachute_status_get_parachute_shield_able(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field temp_param1 from tc_9035_parachute_status message
 *
 * @return  
 */
static inline uint8_t ttalink_tc_9035_parachute_status_get_temp_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field temp_param2 from tc_9035_parachute_status message
 *
 * @return  
 */
static inline uint8_t ttalink_tc_9035_parachute_status_get_temp_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field temp_param3 from tc_9035_parachute_status message
 *
 * @return  
 */
static inline uint8_t ttalink_tc_9035_parachute_status_get_temp_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field param2 from tc_9035_parachute_status message
 *
 * @return  预留参数
 */
static inline float ttalink_tc_9035_parachute_status_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  15);
}

/**
 * @brief Get field param3 from tc_9035_parachute_status message
 *
 * @return  预留参数
 */
static inline float ttalink_tc_9035_parachute_status_get_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  19);
}

/**
 * @brief Get field param4 from tc_9035_parachute_status message
 *
 * @return  预留参数
 */
static inline float ttalink_tc_9035_parachute_status_get_param4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  23);
}

/**
 * @brief Decode a tc_9035_parachute_status message into a struct
 *
 * @param msg The message to decode
 * @param tc_9035_parachute_status C-struct to decode the message contents into
 */
static inline void ttalink_tc_9035_parachute_status_decode(const ttalink_message_t* msg, ttalink_tc_9035_parachute_status_t* tc_9035_parachute_status)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    tc_9035_parachute_status->update_time = ttalink_tc_9035_parachute_status_get_update_time(msg);
    tc_9035_parachute_status->lock_cmd = ttalink_tc_9035_parachute_status_get_lock_cmd(msg);
    tc_9035_parachute_status->lock_state = ttalink_tc_9035_parachute_status_get_lock_state(msg);
    tc_9035_parachute_status->check_state = ttalink_tc_9035_parachute_status_get_check_state(msg);
    tc_9035_parachute_status->vaule = ttalink_tc_9035_parachute_status_get_vaule(msg);
    tc_9035_parachute_status->parachute_shield_able = ttalink_tc_9035_parachute_status_get_parachute_shield_able(msg);
    tc_9035_parachute_status->temp_param1 = ttalink_tc_9035_parachute_status_get_temp_param1(msg);
    tc_9035_parachute_status->temp_param2 = ttalink_tc_9035_parachute_status_get_temp_param2(msg);
    tc_9035_parachute_status->temp_param3 = ttalink_tc_9035_parachute_status_get_temp_param3(msg);
    tc_9035_parachute_status->param2 = ttalink_tc_9035_parachute_status_get_param2(msg);
    tc_9035_parachute_status->param3 = ttalink_tc_9035_parachute_status_get_param3(msg);
    tc_9035_parachute_status->param4 = ttalink_tc_9035_parachute_status_get_param4(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN? msg->len : TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN;
        memset(tc_9035_parachute_status, 0, TTALINK_MSG_ID_TC_9035_PARACHUTE_STATUS_LEN);
    memcpy(tc_9035_parachute_status, _TTA_PAYLOAD(msg), len);
#endif
}
