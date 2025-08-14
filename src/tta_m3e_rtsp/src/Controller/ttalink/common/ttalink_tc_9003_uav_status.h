#pragma once
// MESSAGE TC_9003_UAV_STATUS PACKING

#define TTALINK_MSG_ID_TC_9003_UAV_STATUS 9003

TTAPACKED(
typedef struct __ttalink_tc_9003_uav_status_t {
 uint32_t update_time; /*<  update_time*/
 uint8_t flight_mode; /*< 飞行模式 FLIGHT_CTRL_STATUS_E*/
 uint8_t air_born; /*<  是否飞行*/
 float wp_width; /*<  航线宽度*/
 float calc_width; /*<  模拟算法计算出航线宽度*/
 float param1; /*<  预留参数*/
 float param2; /*<  预留参数*/
 float param3; /*<  预留参数*/
 float param4; /*<  预留参数*/
}) ttalink_tc_9003_uav_status_t;

#define TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN 30
#define TTALINK_MSG_ID_TC_9003_UAV_STATUS_MIN_LEN 30
#define TTALINK_MSG_ID_9003_LEN 30
#define TTALINK_MSG_ID_9003_MIN_LEN 30

#define TTALINK_MSG_ID_TC_9003_UAV_STATUS_CRC 118
#define TTALINK_MSG_ID_9003_CRC 118



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_TC_9003_UAV_STATUS { \
    9003, \
    "TC_9003_UAV_STATUS", \
    9, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_tc_9003_uav_status_t, update_time) }, \
         { "flight_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_tc_9003_uav_status_t, flight_mode) }, \
         { "air_born", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_tc_9003_uav_status_t, air_born) }, \
         { "wp_width", NULL, TTALINK_TYPE_FLOAT, 0, 6, offsetof(ttalink_tc_9003_uav_status_t, wp_width) }, \
         { "calc_width", NULL, TTALINK_TYPE_FLOAT, 0, 10, offsetof(ttalink_tc_9003_uav_status_t, calc_width) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 14, offsetof(ttalink_tc_9003_uav_status_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 18, offsetof(ttalink_tc_9003_uav_status_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 22, offsetof(ttalink_tc_9003_uav_status_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 26, offsetof(ttalink_tc_9003_uav_status_t, param4) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_TC_9003_UAV_STATUS { \
    "TC_9003_UAV_STATUS", \
    9, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_tc_9003_uav_status_t, update_time) }, \
         { "flight_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_tc_9003_uav_status_t, flight_mode) }, \
         { "air_born", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_tc_9003_uav_status_t, air_born) }, \
         { "wp_width", NULL, TTALINK_TYPE_FLOAT, 0, 6, offsetof(ttalink_tc_9003_uav_status_t, wp_width) }, \
         { "calc_width", NULL, TTALINK_TYPE_FLOAT, 0, 10, offsetof(ttalink_tc_9003_uav_status_t, calc_width) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 14, offsetof(ttalink_tc_9003_uav_status_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 18, offsetof(ttalink_tc_9003_uav_status_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 22, offsetof(ttalink_tc_9003_uav_status_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 26, offsetof(ttalink_tc_9003_uav_status_t, param4) }, \
         } \
}
#endif


static inline uint16_t _ttalink_tc_9003_uav_status_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t flight_mode, uint8_t air_born, float wp_width, float calc_width, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, flight_mode);
    _tta_put_uint8_t(buf, 5, air_born);
    _tta_put_float(buf, 6, wp_width);
    _tta_put_float(buf, 10, calc_width);
    _tta_put_float(buf, 14, param1);
    _tta_put_float(buf, 18, param2);
    _tta_put_float(buf, 22, param3);
    _tta_put_float(buf, 26, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN);
#else
    ttalink_tc_9003_uav_status_t packet;
    packet.update_time = update_time;
    packet.flight_mode = flight_mode;
    packet.air_born = air_born;
    packet.wp_width = wp_width;
    packet.calc_width = calc_width;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TC_9003_UAV_STATUS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_TC_9003_UAV_STATUS_MIN_LEN, TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN, TTALINK_MSG_ID_TC_9003_UAV_STATUS_CRC, nocrc);
}

/**
 * @brief Pack a tc_9003_uav_status message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param flight_mode 飞行模式 FLIGHT_CTRL_STATUS_E
 * @param air_born  是否飞行
 * @param wp_width  航线宽度
 * @param calc_width  模拟算法计算出航线宽度
 * @param param1  预留参数
 * @param param2  预留参数
 * @param param3  预留参数
 * @param param4  预留参数
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9003_uav_status_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t flight_mode, uint8_t air_born, float wp_width, float calc_width, float param1, float param2, float param3, float param4)
{
    return _ttalink_tc_9003_uav_status_pack(dst_addr, src_addr, msg,  update_time, flight_mode, air_born, wp_width, calc_width, param1, param2, param3, param4, false);
}

/**
 * @brief Pack a tc_9003_uav_status message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param flight_mode 飞行模式 FLIGHT_CTRL_STATUS_E
 * @param air_born  是否飞行
 * @param wp_width  航线宽度
 * @param calc_width  模拟算法计算出航线宽度
 * @param param1  预留参数
 * @param param2  预留参数
 * @param param3  预留参数
 * @param param4  预留参数
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9003_uav_status_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t flight_mode, uint8_t air_born, float wp_width, float calc_width, float param1, float param2, float param3, float param4)
{
    return _ttalink_tc_9003_uav_status_pack(dst_addr, src_addr, msg,  update_time, flight_mode, air_born, wp_width, calc_width, param1, param2, param3, param4, true);
}


static inline uint16_t _ttalink_tc_9003_uav_status_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t flight_mode,uint8_t air_born,float wp_width,float calc_width,float param1,float param2,float param3,float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, flight_mode);
    _tta_put_uint8_t(buf, 5, air_born);
    _tta_put_float(buf, 6, wp_width);
    _tta_put_float(buf, 10, calc_width);
    _tta_put_float(buf, 14, param1);
    _tta_put_float(buf, 18, param2);
    _tta_put_float(buf, 22, param3);
    _tta_put_float(buf, 26, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN);
#else
    ttalink_tc_9003_uav_status_t packet;
    packet.update_time = update_time;
    packet.flight_mode = flight_mode;
    packet.air_born = air_born;
    packet.wp_width = wp_width;
    packet.calc_width = calc_width;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TC_9003_UAV_STATUS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9003_UAV_STATUS_MIN_LEN, TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN, TTALINK_MSG_ID_TC_9003_UAV_STATUS_CRC, nocrc);
}

/**
 * @brief Pack a tc_9003_uav_status message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param flight_mode 飞行模式 FLIGHT_CTRL_STATUS_E
 * @param air_born  是否飞行
 * @param wp_width  航线宽度
 * @param calc_width  模拟算法计算出航线宽度
 * @param param1  预留参数
 * @param param2  预留参数
 * @param param3  预留参数
 * @param param4  预留参数
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9003_uav_status_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t flight_mode,uint8_t air_born,float wp_width,float calc_width,float param1,float param2,float param3,float param4)
{
    return _ttalink_tc_9003_uav_status_pack_chan(dst_addr, src_addr, chan, msg,  update_time, flight_mode, air_born, wp_width, calc_width, param1, param2, param3, param4, false);
}

/**
 * @brief Pack a tc_9003_uav_status message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param flight_mode 飞行模式 FLIGHT_CTRL_STATUS_E
 * @param air_born  是否飞行
 * @param wp_width  航线宽度
 * @param calc_width  模拟算法计算出航线宽度
 * @param param1  预留参数
 * @param param2  预留参数
 * @param param3  预留参数
 * @param param4  预留参数
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9003_uav_status_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t flight_mode,uint8_t air_born,float wp_width,float calc_width,float param1,float param2,float param3,float param4)
{
    return _ttalink_tc_9003_uav_status_pack_chan(dst_addr, src_addr, chan, msg,  update_time, flight_mode, air_born, wp_width, calc_width, param1, param2, param3, param4, true);
}


static inline uint16_t _ttalink_tc_9003_uav_status_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9003_uav_status_t* tc_9003_uav_status, bool nocrc)
{
    if(nocrc){
        return ttalink_tc_9003_uav_status_pack_nocrc(dst_addr, src_addr, msg, tc_9003_uav_status->update_time, tc_9003_uav_status->flight_mode, tc_9003_uav_status->air_born, tc_9003_uav_status->wp_width, tc_9003_uav_status->calc_width, tc_9003_uav_status->param1, tc_9003_uav_status->param2, tc_9003_uav_status->param3, tc_9003_uav_status->param4);
    }else{
        return ttalink_tc_9003_uav_status_pack(dst_addr, src_addr, msg, tc_9003_uav_status->update_time, tc_9003_uav_status->flight_mode, tc_9003_uav_status->air_born, tc_9003_uav_status->wp_width, tc_9003_uav_status->calc_width, tc_9003_uav_status->param1, tc_9003_uav_status->param2, tc_9003_uav_status->param3, tc_9003_uav_status->param4);
    }
    
}

/**
 * @brief Encode a tc_9003_uav_status struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param tc_9003_uav_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9003_uav_status_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9003_uav_status_t* tc_9003_uav_status)
{
    return _ttalink_tc_9003_uav_status_encode(dst_addr, src_addr, msg, tc_9003_uav_status, false);
}

/**
 * @brief Encode a tc_9003_uav_status struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param tc_9003_uav_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9003_uav_status_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9003_uav_status_t* tc_9003_uav_status)
{
    return _ttalink_tc_9003_uav_status_encode(dst_addr, src_addr, msg, tc_9003_uav_status, true);
}


static inline uint16_t _ttalink_tc_9003_uav_status_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9003_uav_status_t* tc_9003_uav_status, bool nocrc)
{
    if(nocrc){
        return ttalink_tc_9003_uav_status_pack_chan_nocrc(dst_addr, src_addr, chan, msg, tc_9003_uav_status->update_time, tc_9003_uav_status->flight_mode, tc_9003_uav_status->air_born, tc_9003_uav_status->wp_width, tc_9003_uav_status->calc_width, tc_9003_uav_status->param1, tc_9003_uav_status->param2, tc_9003_uav_status->param3, tc_9003_uav_status->param4);
    }else{
        return ttalink_tc_9003_uav_status_pack_chan(dst_addr, src_addr, chan, msg, tc_9003_uav_status->update_time, tc_9003_uav_status->flight_mode, tc_9003_uav_status->air_born, tc_9003_uav_status->wp_width, tc_9003_uav_status->calc_width, tc_9003_uav_status->param1, tc_9003_uav_status->param2, tc_9003_uav_status->param3, tc_9003_uav_status->param4);
    }
}

/**
 * @brief Encode a tc_9003_uav_status struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param tc_9003_uav_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9003_uav_status_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9003_uav_status_t* tc_9003_uav_status)
{
    return _ttalink_tc_9003_uav_status_encode_chan(dst_addr, src_addr, chan, msg, tc_9003_uav_status, false);
}

/**
 * @brief Encode a tc_9003_uav_status struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param tc_9003_uav_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9003_uav_status_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9003_uav_status_t* tc_9003_uav_status)
{
    return _ttalink_tc_9003_uav_status_encode_chan(dst_addr, src_addr, chan, msg, tc_9003_uav_status, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_tc_9003_uav_status_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t flight_mode, uint8_t air_born, float wp_width, float calc_width, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, flight_mode);
    _tta_put_uint8_t(buf, 5, air_born);
    _tta_put_float(buf, 6, wp_width);
    _tta_put_float(buf, 10, calc_width);
    _tta_put_float(buf, 14, param1);
    _tta_put_float(buf, 18, param2);
    _tta_put_float(buf, 22, param3);
    _tta_put_float(buf, 26, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9003_UAV_STATUS, buf, TTALINK_MSG_ID_TC_9003_UAV_STATUS_MIN_LEN, TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN, TTALINK_MSG_ID_TC_9003_UAV_STATUS_CRC, nocrc);
#else
    ttalink_tc_9003_uav_status_t packet;
    packet.update_time = update_time;
    packet.flight_mode = flight_mode;
    packet.air_born = air_born;
    packet.wp_width = wp_width;
    packet.calc_width = calc_width;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9003_UAV_STATUS, (const char *)&packet, TTALINK_MSG_ID_TC_9003_UAV_STATUS_MIN_LEN, TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN, TTALINK_MSG_ID_TC_9003_UAV_STATUS_CRC, nocrc);
#endif
}

/**
 * @brief Send a tc_9003_uav_status message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param flight_mode 飞行模式 FLIGHT_CTRL_STATUS_E
 * @param air_born  是否飞行
 * @param wp_width  航线宽度
 * @param calc_width  模拟算法计算出航线宽度
 * @param param1  预留参数
 * @param param2  预留参数
 * @param param3  预留参数
 * @param param4  预留参数
 */
static inline void ttalink_tc_9003_uav_status_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t flight_mode, uint8_t air_born, float wp_width, float calc_width, float param1, float param2, float param3, float param4)
{
    _ttalink_tc_9003_uav_status_send(dst_addr, src_addr, chan, update_time, flight_mode, air_born, wp_width, calc_width, param1, param2, param3, param4, false);
}

/**
 * @brief Send a tc_9003_uav_status message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param flight_mode 飞行模式 FLIGHT_CTRL_STATUS_E
 * @param air_born  是否飞行
 * @param wp_width  航线宽度
 * @param calc_width  模拟算法计算出航线宽度
 * @param param1  预留参数
 * @param param2  预留参数
 * @param param3  预留参数
 * @param param4  预留参数
 */
static inline void ttalink_tc_9003_uav_status_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t flight_mode, uint8_t air_born, float wp_width, float calc_width, float param1, float param2, float param3, float param4)
{
    _ttalink_tc_9003_uav_status_send(dst_addr, src_addr, chan, update_time, flight_mode, air_born, wp_width, calc_width, param1, param2, param3, param4, true);
}


static inline void _ttalink_tc_9003_uav_status_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9003_uav_status_t* tc_9003_uav_status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_tc_9003_uav_status_send_nocrc(dst_addr, src_addr, chan, tc_9003_uav_status->update_time, tc_9003_uav_status->flight_mode, tc_9003_uav_status->air_born, tc_9003_uav_status->wp_width, tc_9003_uav_status->calc_width, tc_9003_uav_status->param1, tc_9003_uav_status->param2, tc_9003_uav_status->param3, tc_9003_uav_status->param4);
    }else{
        ttalink_tc_9003_uav_status_send(dst_addr, src_addr, chan, tc_9003_uav_status->update_time, tc_9003_uav_status->flight_mode, tc_9003_uav_status->air_born, tc_9003_uav_status->wp_width, tc_9003_uav_status->calc_width, tc_9003_uav_status->param1, tc_9003_uav_status->param2, tc_9003_uav_status->param3, tc_9003_uav_status->param4);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9003_UAV_STATUS, (const char *)tc_9003_uav_status, TTALINK_MSG_ID_TC_9003_UAV_STATUS_MIN_LEN, TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN, TTALINK_MSG_ID_TC_9003_UAV_STATUS_CRC, nocrc);
#endif
}

/**
 * @brief Send a tc_9003_uav_status message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_tc_9003_uav_status_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9003_uav_status_t* tc_9003_uav_status)
{
    _ttalink_tc_9003_uav_status_send_struct(dst_addr, src_addr, chan, tc_9003_uav_status, false);
}

/**
 * @brief Send a tc_9003_uav_status message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_tc_9003_uav_status_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9003_uav_status_t* tc_9003_uav_status)
{
    _ttalink_tc_9003_uav_status_send_struct(dst_addr, src_addr, chan, tc_9003_uav_status, true);
}

#if TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_tc_9003_uav_status_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t flight_mode, uint8_t air_born, float wp_width, float calc_width, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, flight_mode);
    _tta_put_uint8_t(buf, 5, air_born);
    _tta_put_float(buf, 6, wp_width);
    _tta_put_float(buf, 10, calc_width);
    _tta_put_float(buf, 14, param1);
    _tta_put_float(buf, 18, param2);
    _tta_put_float(buf, 22, param3);
    _tta_put_float(buf, 26, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9003_UAV_STATUS, buf, TTALINK_MSG_ID_TC_9003_UAV_STATUS_MIN_LEN, TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN, TTALINK_MSG_ID_TC_9003_UAV_STATUS_CRC, nocrc);
#else
    ttalink_tc_9003_uav_status_t *packet = (ttalink_tc_9003_uav_status_t *)msgbuf;
    packet->update_time = update_time;
    packet->flight_mode = flight_mode;
    packet->air_born = air_born;
    packet->wp_width = wp_width;
    packet->calc_width = calc_width;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9003_UAV_STATUS, (const char *)packet, TTALINK_MSG_ID_TC_9003_UAV_STATUS_MIN_LEN, TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN, TTALINK_MSG_ID_TC_9003_UAV_STATUS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_tc_9003_uav_status_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t flight_mode, uint8_t air_born, float wp_width, float calc_width, float param1, float param2, float param3, float param4)
{
    _ttalink_tc_9003_uav_status_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, flight_mode, air_born, wp_width, calc_width, param1, param2, param3, param4, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_tc_9003_uav_status_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t flight_mode, uint8_t air_born, float wp_width, float calc_width, float param1, float param2, float param3, float param4)
{
    _ttalink_tc_9003_uav_status_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, flight_mode, air_born, wp_width, calc_width, param1, param2, param3, param4, true);
}
#endif

#endif

// MESSAGE TC_9003_UAV_STATUS UNPACKING


/**
 * @brief Get field update_time from tc_9003_uav_status message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_tc_9003_uav_status_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field flight_mode from tc_9003_uav_status message
 *
 * @return 飞行模式 FLIGHT_CTRL_STATUS_E
 */
static inline uint8_t ttalink_tc_9003_uav_status_get_flight_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field air_born from tc_9003_uav_status message
 *
 * @return  是否飞行
 */
static inline uint8_t ttalink_tc_9003_uav_status_get_air_born(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field wp_width from tc_9003_uav_status message
 *
 * @return  航线宽度
 */
static inline float ttalink_tc_9003_uav_status_get_wp_width(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  6);
}

/**
 * @brief Get field calc_width from tc_9003_uav_status message
 *
 * @return  模拟算法计算出航线宽度
 */
static inline float ttalink_tc_9003_uav_status_get_calc_width(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  10);
}

/**
 * @brief Get field param1 from tc_9003_uav_status message
 *
 * @return  预留参数
 */
static inline float ttalink_tc_9003_uav_status_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  14);
}

/**
 * @brief Get field param2 from tc_9003_uav_status message
 *
 * @return  预留参数
 */
static inline float ttalink_tc_9003_uav_status_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  18);
}

/**
 * @brief Get field param3 from tc_9003_uav_status message
 *
 * @return  预留参数
 */
static inline float ttalink_tc_9003_uav_status_get_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  22);
}

/**
 * @brief Get field param4 from tc_9003_uav_status message
 *
 * @return  预留参数
 */
static inline float ttalink_tc_9003_uav_status_get_param4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  26);
}

/**
 * @brief Decode a tc_9003_uav_status message into a struct
 *
 * @param msg The message to decode
 * @param tc_9003_uav_status C-struct to decode the message contents into
 */
static inline void ttalink_tc_9003_uav_status_decode(const ttalink_message_t* msg, ttalink_tc_9003_uav_status_t* tc_9003_uav_status)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    tc_9003_uav_status->update_time = ttalink_tc_9003_uav_status_get_update_time(msg);
    tc_9003_uav_status->flight_mode = ttalink_tc_9003_uav_status_get_flight_mode(msg);
    tc_9003_uav_status->air_born = ttalink_tc_9003_uav_status_get_air_born(msg);
    tc_9003_uav_status->wp_width = ttalink_tc_9003_uav_status_get_wp_width(msg);
    tc_9003_uav_status->calc_width = ttalink_tc_9003_uav_status_get_calc_width(msg);
    tc_9003_uav_status->param1 = ttalink_tc_9003_uav_status_get_param1(msg);
    tc_9003_uav_status->param2 = ttalink_tc_9003_uav_status_get_param2(msg);
    tc_9003_uav_status->param3 = ttalink_tc_9003_uav_status_get_param3(msg);
    tc_9003_uav_status->param4 = ttalink_tc_9003_uav_status_get_param4(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN? msg->len : TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN;
        memset(tc_9003_uav_status, 0, TTALINK_MSG_ID_TC_9003_UAV_STATUS_LEN);
    memcpy(tc_9003_uav_status, _TTA_PAYLOAD(msg), len);
#endif
}
