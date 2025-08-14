#pragma once
// MESSAGE DEVICE_DATA PACKING

#define TTALINK_MSG_ID_DEVICE_DATA 2009

TTAPACKED(
typedef struct __ttalink_device_data_t {
 uint32_t update_time; /*<  更新时间.*/
 float front_radar_altit; /*<  前避障雷达距离.*/
 float back_radar_altit; /*<  后避障雷达距离.*/
 float follow_radar_altit; /*<  仿地雷达高度.*/
 uint8_t front_radar_flag; /*<  前避障雷达可以标志. 0：不可用  1：可用*/
 uint8_t back_radar_flag; /*<  后避障雷达可以标志.  0：不可用  1：可用*/
 uint8_t follow_radar_flag; /*<  仿地雷达可以标志.  0：不可用  1：可用*/
 float flow_speed; /*<  流量计流速. */
 float residual_pesticide; /*<  剩余农药量（未加入）. */
 uint8_t uhc_liquid; /*<  液位计是否有药量 0：没有  1：有. */
 float cur_spray_area; /*<  当前已喷洒面积. */
 float tar_spray_area; /*<  需要喷洒的面积. */
}) ttalink_device_data_t;

#define TTALINK_MSG_ID_DEVICE_DATA_LEN 36
#define TTALINK_MSG_ID_DEVICE_DATA_MIN_LEN 36
#define TTALINK_MSG_ID_2009_LEN 36
#define TTALINK_MSG_ID_2009_MIN_LEN 36

#define TTALINK_MSG_ID_DEVICE_DATA_CRC 17
#define TTALINK_MSG_ID_2009_CRC 17



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_DEVICE_DATA { \
    2009, \
    "DEVICE_DATA", \
    12, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_device_data_t, update_time) }, \
         { "front_radar_altit", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_device_data_t, front_radar_altit) }, \
         { "back_radar_altit", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_device_data_t, back_radar_altit) }, \
         { "follow_radar_altit", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_device_data_t, follow_radar_altit) }, \
         { "front_radar_flag", NULL, TTALINK_TYPE_UINT8_T, 0, 16, offsetof(ttalink_device_data_t, front_radar_flag) }, \
         { "back_radar_flag", NULL, TTALINK_TYPE_UINT8_T, 0, 17, offsetof(ttalink_device_data_t, back_radar_flag) }, \
         { "follow_radar_flag", NULL, TTALINK_TYPE_UINT8_T, 0, 18, offsetof(ttalink_device_data_t, follow_radar_flag) }, \
         { "flow_speed", NULL, TTALINK_TYPE_FLOAT, 0, 19, offsetof(ttalink_device_data_t, flow_speed) }, \
         { "residual_pesticide", NULL, TTALINK_TYPE_FLOAT, 0, 23, offsetof(ttalink_device_data_t, residual_pesticide) }, \
         { "uhc_liquid", NULL, TTALINK_TYPE_UINT8_T, 0, 27, offsetof(ttalink_device_data_t, uhc_liquid) }, \
         { "cur_spray_area", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_device_data_t, cur_spray_area) }, \
         { "tar_spray_area", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_device_data_t, tar_spray_area) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_DEVICE_DATA { \
    "DEVICE_DATA", \
    12, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_device_data_t, update_time) }, \
         { "front_radar_altit", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_device_data_t, front_radar_altit) }, \
         { "back_radar_altit", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_device_data_t, back_radar_altit) }, \
         { "follow_radar_altit", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_device_data_t, follow_radar_altit) }, \
         { "front_radar_flag", NULL, TTALINK_TYPE_UINT8_T, 0, 16, offsetof(ttalink_device_data_t, front_radar_flag) }, \
         { "back_radar_flag", NULL, TTALINK_TYPE_UINT8_T, 0, 17, offsetof(ttalink_device_data_t, back_radar_flag) }, \
         { "follow_radar_flag", NULL, TTALINK_TYPE_UINT8_T, 0, 18, offsetof(ttalink_device_data_t, follow_radar_flag) }, \
         { "flow_speed", NULL, TTALINK_TYPE_FLOAT, 0, 19, offsetof(ttalink_device_data_t, flow_speed) }, \
         { "residual_pesticide", NULL, TTALINK_TYPE_FLOAT, 0, 23, offsetof(ttalink_device_data_t, residual_pesticide) }, \
         { "uhc_liquid", NULL, TTALINK_TYPE_UINT8_T, 0, 27, offsetof(ttalink_device_data_t, uhc_liquid) }, \
         { "cur_spray_area", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_device_data_t, cur_spray_area) }, \
         { "tar_spray_area", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_device_data_t, tar_spray_area) }, \
         } \
}
#endif


static inline uint16_t _ttalink_device_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, float front_radar_altit, float back_radar_altit, float follow_radar_altit, uint8_t front_radar_flag, uint8_t back_radar_flag, uint8_t follow_radar_flag, float flow_speed, float residual_pesticide, uint8_t uhc_liquid, float cur_spray_area, float tar_spray_area, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_DEVICE_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, front_radar_altit);
    _tta_put_float(buf, 8, back_radar_altit);
    _tta_put_float(buf, 12, follow_radar_altit);
    _tta_put_uint8_t(buf, 16, front_radar_flag);
    _tta_put_uint8_t(buf, 17, back_radar_flag);
    _tta_put_uint8_t(buf, 18, follow_radar_flag);
    _tta_put_float(buf, 19, flow_speed);
    _tta_put_float(buf, 23, residual_pesticide);
    _tta_put_uint8_t(buf, 27, uhc_liquid);
    _tta_put_float(buf, 28, cur_spray_area);
    _tta_put_float(buf, 32, tar_spray_area);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_DEVICE_DATA_LEN);
#else
    ttalink_device_data_t packet;
    packet.update_time = update_time;
    packet.front_radar_altit = front_radar_altit;
    packet.back_radar_altit = back_radar_altit;
    packet.follow_radar_altit = follow_radar_altit;
    packet.front_radar_flag = front_radar_flag;
    packet.back_radar_flag = back_radar_flag;
    packet.follow_radar_flag = follow_radar_flag;
    packet.flow_speed = flow_speed;
    packet.residual_pesticide = residual_pesticide;
    packet.uhc_liquid = uhc_liquid;
    packet.cur_spray_area = cur_spray_area;
    packet.tar_spray_area = tar_spray_area;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_DEVICE_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_DEVICE_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_DEVICE_DATA_MIN_LEN, TTALINK_MSG_ID_DEVICE_DATA_LEN, TTALINK_MSG_ID_DEVICE_DATA_CRC, nocrc);
}

/**
 * @brief Pack a device_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  更新时间.
 * @param front_radar_altit  前避障雷达距离.
 * @param back_radar_altit  后避障雷达距离.
 * @param follow_radar_altit  仿地雷达高度.
 * @param front_radar_flag  前避障雷达可以标志. 0：不可用  1：可用
 * @param back_radar_flag  后避障雷达可以标志.  0：不可用  1：可用
 * @param follow_radar_flag  仿地雷达可以标志.  0：不可用  1：可用
 * @param flow_speed  流量计流速. 
 * @param residual_pesticide  剩余农药量（未加入）. 
 * @param uhc_liquid  液位计是否有药量 0：没有  1：有. 
 * @param cur_spray_area  当前已喷洒面积. 
 * @param tar_spray_area  需要喷洒的面积. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_device_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, float front_radar_altit, float back_radar_altit, float follow_radar_altit, uint8_t front_radar_flag, uint8_t back_radar_flag, uint8_t follow_radar_flag, float flow_speed, float residual_pesticide, uint8_t uhc_liquid, float cur_spray_area, float tar_spray_area)
{
    return _ttalink_device_data_pack(dst_addr, src_addr, msg,  update_time, front_radar_altit, back_radar_altit, follow_radar_altit, front_radar_flag, back_radar_flag, follow_radar_flag, flow_speed, residual_pesticide, uhc_liquid, cur_spray_area, tar_spray_area, false);
}

/**
 * @brief Pack a device_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  更新时间.
 * @param front_radar_altit  前避障雷达距离.
 * @param back_radar_altit  后避障雷达距离.
 * @param follow_radar_altit  仿地雷达高度.
 * @param front_radar_flag  前避障雷达可以标志. 0：不可用  1：可用
 * @param back_radar_flag  后避障雷达可以标志.  0：不可用  1：可用
 * @param follow_radar_flag  仿地雷达可以标志.  0：不可用  1：可用
 * @param flow_speed  流量计流速. 
 * @param residual_pesticide  剩余农药量（未加入）. 
 * @param uhc_liquid  液位计是否有药量 0：没有  1：有. 
 * @param cur_spray_area  当前已喷洒面积. 
 * @param tar_spray_area  需要喷洒的面积. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_device_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, float front_radar_altit, float back_radar_altit, float follow_radar_altit, uint8_t front_radar_flag, uint8_t back_radar_flag, uint8_t follow_radar_flag, float flow_speed, float residual_pesticide, uint8_t uhc_liquid, float cur_spray_area, float tar_spray_area)
{
    return _ttalink_device_data_pack(dst_addr, src_addr, msg,  update_time, front_radar_altit, back_radar_altit, follow_radar_altit, front_radar_flag, back_radar_flag, follow_radar_flag, flow_speed, residual_pesticide, uhc_liquid, cur_spray_area, tar_spray_area, true);
}


static inline uint16_t _ttalink_device_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,float front_radar_altit,float back_radar_altit,float follow_radar_altit,uint8_t front_radar_flag,uint8_t back_radar_flag,uint8_t follow_radar_flag,float flow_speed,float residual_pesticide,uint8_t uhc_liquid,float cur_spray_area,float tar_spray_area, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_DEVICE_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, front_radar_altit);
    _tta_put_float(buf, 8, back_radar_altit);
    _tta_put_float(buf, 12, follow_radar_altit);
    _tta_put_uint8_t(buf, 16, front_radar_flag);
    _tta_put_uint8_t(buf, 17, back_radar_flag);
    _tta_put_uint8_t(buf, 18, follow_radar_flag);
    _tta_put_float(buf, 19, flow_speed);
    _tta_put_float(buf, 23, residual_pesticide);
    _tta_put_uint8_t(buf, 27, uhc_liquid);
    _tta_put_float(buf, 28, cur_spray_area);
    _tta_put_float(buf, 32, tar_spray_area);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_DEVICE_DATA_LEN);
#else
    ttalink_device_data_t packet;
    packet.update_time = update_time;
    packet.front_radar_altit = front_radar_altit;
    packet.back_radar_altit = back_radar_altit;
    packet.follow_radar_altit = follow_radar_altit;
    packet.front_radar_flag = front_radar_flag;
    packet.back_radar_flag = back_radar_flag;
    packet.follow_radar_flag = follow_radar_flag;
    packet.flow_speed = flow_speed;
    packet.residual_pesticide = residual_pesticide;
    packet.uhc_liquid = uhc_liquid;
    packet.cur_spray_area = cur_spray_area;
    packet.tar_spray_area = tar_spray_area;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_DEVICE_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_DEVICE_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_DEVICE_DATA_MIN_LEN, TTALINK_MSG_ID_DEVICE_DATA_LEN, TTALINK_MSG_ID_DEVICE_DATA_CRC, nocrc);
}

/**
 * @brief Pack a device_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  更新时间.
 * @param front_radar_altit  前避障雷达距离.
 * @param back_radar_altit  后避障雷达距离.
 * @param follow_radar_altit  仿地雷达高度.
 * @param front_radar_flag  前避障雷达可以标志. 0：不可用  1：可用
 * @param back_radar_flag  后避障雷达可以标志.  0：不可用  1：可用
 * @param follow_radar_flag  仿地雷达可以标志.  0：不可用  1：可用
 * @param flow_speed  流量计流速. 
 * @param residual_pesticide  剩余农药量（未加入）. 
 * @param uhc_liquid  液位计是否有药量 0：没有  1：有. 
 * @param cur_spray_area  当前已喷洒面积. 
 * @param tar_spray_area  需要喷洒的面积. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_device_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,float front_radar_altit,float back_radar_altit,float follow_radar_altit,uint8_t front_radar_flag,uint8_t back_radar_flag,uint8_t follow_radar_flag,float flow_speed,float residual_pesticide,uint8_t uhc_liquid,float cur_spray_area,float tar_spray_area)
{
    return _ttalink_device_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, front_radar_altit, back_radar_altit, follow_radar_altit, front_radar_flag, back_radar_flag, follow_radar_flag, flow_speed, residual_pesticide, uhc_liquid, cur_spray_area, tar_spray_area, false);
}

/**
 * @brief Pack a device_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  更新时间.
 * @param front_radar_altit  前避障雷达距离.
 * @param back_radar_altit  后避障雷达距离.
 * @param follow_radar_altit  仿地雷达高度.
 * @param front_radar_flag  前避障雷达可以标志. 0：不可用  1：可用
 * @param back_radar_flag  后避障雷达可以标志.  0：不可用  1：可用
 * @param follow_radar_flag  仿地雷达可以标志.  0：不可用  1：可用
 * @param flow_speed  流量计流速. 
 * @param residual_pesticide  剩余农药量（未加入）. 
 * @param uhc_liquid  液位计是否有药量 0：没有  1：有. 
 * @param cur_spray_area  当前已喷洒面积. 
 * @param tar_spray_area  需要喷洒的面积. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_device_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,float front_radar_altit,float back_radar_altit,float follow_radar_altit,uint8_t front_radar_flag,uint8_t back_radar_flag,uint8_t follow_radar_flag,float flow_speed,float residual_pesticide,uint8_t uhc_liquid,float cur_spray_area,float tar_spray_area)
{
    return _ttalink_device_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, front_radar_altit, back_radar_altit, follow_radar_altit, front_radar_flag, back_radar_flag, follow_radar_flag, flow_speed, residual_pesticide, uhc_liquid, cur_spray_area, tar_spray_area, true);
}


static inline uint16_t _ttalink_device_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_device_data_t* device_data, bool nocrc)
{
    if(nocrc){
        return ttalink_device_data_pack_nocrc(dst_addr, src_addr, msg, device_data->update_time, device_data->front_radar_altit, device_data->back_radar_altit, device_data->follow_radar_altit, device_data->front_radar_flag, device_data->back_radar_flag, device_data->follow_radar_flag, device_data->flow_speed, device_data->residual_pesticide, device_data->uhc_liquid, device_data->cur_spray_area, device_data->tar_spray_area);
    }else{
        return ttalink_device_data_pack(dst_addr, src_addr, msg, device_data->update_time, device_data->front_radar_altit, device_data->back_radar_altit, device_data->follow_radar_altit, device_data->front_radar_flag, device_data->back_radar_flag, device_data->follow_radar_flag, device_data->flow_speed, device_data->residual_pesticide, device_data->uhc_liquid, device_data->cur_spray_area, device_data->tar_spray_area);
    }
    
}

/**
 * @brief Encode a device_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param device_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_device_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_device_data_t* device_data)
{
    return _ttalink_device_data_encode(dst_addr, src_addr, msg, device_data, false);
}

/**
 * @brief Encode a device_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param device_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_device_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_device_data_t* device_data)
{
    return _ttalink_device_data_encode(dst_addr, src_addr, msg, device_data, true);
}


static inline uint16_t _ttalink_device_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_device_data_t* device_data, bool nocrc)
{
    if(nocrc){
        return ttalink_device_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, device_data->update_time, device_data->front_radar_altit, device_data->back_radar_altit, device_data->follow_radar_altit, device_data->front_radar_flag, device_data->back_radar_flag, device_data->follow_radar_flag, device_data->flow_speed, device_data->residual_pesticide, device_data->uhc_liquid, device_data->cur_spray_area, device_data->tar_spray_area);
    }else{
        return ttalink_device_data_pack_chan(dst_addr, src_addr, chan, msg, device_data->update_time, device_data->front_radar_altit, device_data->back_radar_altit, device_data->follow_radar_altit, device_data->front_radar_flag, device_data->back_radar_flag, device_data->follow_radar_flag, device_data->flow_speed, device_data->residual_pesticide, device_data->uhc_liquid, device_data->cur_spray_area, device_data->tar_spray_area);
    }
}

/**
 * @brief Encode a device_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param device_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_device_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_device_data_t* device_data)
{
    return _ttalink_device_data_encode_chan(dst_addr, src_addr, chan, msg, device_data, false);
}

/**
 * @brief Encode a device_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param device_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_device_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_device_data_t* device_data)
{
    return _ttalink_device_data_encode_chan(dst_addr, src_addr, chan, msg, device_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_device_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, float front_radar_altit, float back_radar_altit, float follow_radar_altit, uint8_t front_radar_flag, uint8_t back_radar_flag, uint8_t follow_radar_flag, float flow_speed, float residual_pesticide, uint8_t uhc_liquid, float cur_spray_area, float tar_spray_area, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_DEVICE_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, front_radar_altit);
    _tta_put_float(buf, 8, back_radar_altit);
    _tta_put_float(buf, 12, follow_radar_altit);
    _tta_put_uint8_t(buf, 16, front_radar_flag);
    _tta_put_uint8_t(buf, 17, back_radar_flag);
    _tta_put_uint8_t(buf, 18, follow_radar_flag);
    _tta_put_float(buf, 19, flow_speed);
    _tta_put_float(buf, 23, residual_pesticide);
    _tta_put_uint8_t(buf, 27, uhc_liquid);
    _tta_put_float(buf, 28, cur_spray_area);
    _tta_put_float(buf, 32, tar_spray_area);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_DEVICE_DATA, buf, TTALINK_MSG_ID_DEVICE_DATA_MIN_LEN, TTALINK_MSG_ID_DEVICE_DATA_LEN, TTALINK_MSG_ID_DEVICE_DATA_CRC, nocrc);
#else
    ttalink_device_data_t packet;
    packet.update_time = update_time;
    packet.front_radar_altit = front_radar_altit;
    packet.back_radar_altit = back_radar_altit;
    packet.follow_radar_altit = follow_radar_altit;
    packet.front_radar_flag = front_radar_flag;
    packet.back_radar_flag = back_radar_flag;
    packet.follow_radar_flag = follow_radar_flag;
    packet.flow_speed = flow_speed;
    packet.residual_pesticide = residual_pesticide;
    packet.uhc_liquid = uhc_liquid;
    packet.cur_spray_area = cur_spray_area;
    packet.tar_spray_area = tar_spray_area;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_DEVICE_DATA, (const char *)&packet, TTALINK_MSG_ID_DEVICE_DATA_MIN_LEN, TTALINK_MSG_ID_DEVICE_DATA_LEN, TTALINK_MSG_ID_DEVICE_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a device_data message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  更新时间.
 * @param front_radar_altit  前避障雷达距离.
 * @param back_radar_altit  后避障雷达距离.
 * @param follow_radar_altit  仿地雷达高度.
 * @param front_radar_flag  前避障雷达可以标志. 0：不可用  1：可用
 * @param back_radar_flag  后避障雷达可以标志.  0：不可用  1：可用
 * @param follow_radar_flag  仿地雷达可以标志.  0：不可用  1：可用
 * @param flow_speed  流量计流速. 
 * @param residual_pesticide  剩余农药量（未加入）. 
 * @param uhc_liquid  液位计是否有药量 0：没有  1：有. 
 * @param cur_spray_area  当前已喷洒面积. 
 * @param tar_spray_area  需要喷洒的面积. 
 */
static inline void ttalink_device_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, float front_radar_altit, float back_radar_altit, float follow_radar_altit, uint8_t front_radar_flag, uint8_t back_radar_flag, uint8_t follow_radar_flag, float flow_speed, float residual_pesticide, uint8_t uhc_liquid, float cur_spray_area, float tar_spray_area)
{
    _ttalink_device_data_send(dst_addr, src_addr, chan, update_time, front_radar_altit, back_radar_altit, follow_radar_altit, front_radar_flag, back_radar_flag, follow_radar_flag, flow_speed, residual_pesticide, uhc_liquid, cur_spray_area, tar_spray_area, false);
}

/**
 * @brief Send a device_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  更新时间.
 * @param front_radar_altit  前避障雷达距离.
 * @param back_radar_altit  后避障雷达距离.
 * @param follow_radar_altit  仿地雷达高度.
 * @param front_radar_flag  前避障雷达可以标志. 0：不可用  1：可用
 * @param back_radar_flag  后避障雷达可以标志.  0：不可用  1：可用
 * @param follow_radar_flag  仿地雷达可以标志.  0：不可用  1：可用
 * @param flow_speed  流量计流速. 
 * @param residual_pesticide  剩余农药量（未加入）. 
 * @param uhc_liquid  液位计是否有药量 0：没有  1：有. 
 * @param cur_spray_area  当前已喷洒面积. 
 * @param tar_spray_area  需要喷洒的面积. 
 */
static inline void ttalink_device_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, float front_radar_altit, float back_radar_altit, float follow_radar_altit, uint8_t front_radar_flag, uint8_t back_radar_flag, uint8_t follow_radar_flag, float flow_speed, float residual_pesticide, uint8_t uhc_liquid, float cur_spray_area, float tar_spray_area)
{
    _ttalink_device_data_send(dst_addr, src_addr, chan, update_time, front_radar_altit, back_radar_altit, follow_radar_altit, front_radar_flag, back_radar_flag, follow_radar_flag, flow_speed, residual_pesticide, uhc_liquid, cur_spray_area, tar_spray_area, true);
}


static inline void _ttalink_device_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_device_data_t* device_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_device_data_send_nocrc(dst_addr, src_addr, chan, device_data->update_time, device_data->front_radar_altit, device_data->back_radar_altit, device_data->follow_radar_altit, device_data->front_radar_flag, device_data->back_radar_flag, device_data->follow_radar_flag, device_data->flow_speed, device_data->residual_pesticide, device_data->uhc_liquid, device_data->cur_spray_area, device_data->tar_spray_area);
    }else{
        ttalink_device_data_send(dst_addr, src_addr, chan, device_data->update_time, device_data->front_radar_altit, device_data->back_radar_altit, device_data->follow_radar_altit, device_data->front_radar_flag, device_data->back_radar_flag, device_data->follow_radar_flag, device_data->flow_speed, device_data->residual_pesticide, device_data->uhc_liquid, device_data->cur_spray_area, device_data->tar_spray_area);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_DEVICE_DATA, (const char *)device_data, TTALINK_MSG_ID_DEVICE_DATA_MIN_LEN, TTALINK_MSG_ID_DEVICE_DATA_LEN, TTALINK_MSG_ID_DEVICE_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a device_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_device_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_device_data_t* device_data)
{
    _ttalink_device_data_send_struct(dst_addr, src_addr, chan, device_data, false);
}

/**
 * @brief Send a device_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_device_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_device_data_t* device_data)
{
    _ttalink_device_data_send_struct(dst_addr, src_addr, chan, device_data, true);
}

#if TTALINK_MSG_ID_DEVICE_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_device_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, float front_radar_altit, float back_radar_altit, float follow_radar_altit, uint8_t front_radar_flag, uint8_t back_radar_flag, uint8_t follow_radar_flag, float flow_speed, float residual_pesticide, uint8_t uhc_liquid, float cur_spray_area, float tar_spray_area, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, front_radar_altit);
    _tta_put_float(buf, 8, back_radar_altit);
    _tta_put_float(buf, 12, follow_radar_altit);
    _tta_put_uint8_t(buf, 16, front_radar_flag);
    _tta_put_uint8_t(buf, 17, back_radar_flag);
    _tta_put_uint8_t(buf, 18, follow_radar_flag);
    _tta_put_float(buf, 19, flow_speed);
    _tta_put_float(buf, 23, residual_pesticide);
    _tta_put_uint8_t(buf, 27, uhc_liquid);
    _tta_put_float(buf, 28, cur_spray_area);
    _tta_put_float(buf, 32, tar_spray_area);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_DEVICE_DATA, buf, TTALINK_MSG_ID_DEVICE_DATA_MIN_LEN, TTALINK_MSG_ID_DEVICE_DATA_LEN, TTALINK_MSG_ID_DEVICE_DATA_CRC, nocrc);
#else
    ttalink_device_data_t *packet = (ttalink_device_data_t *)msgbuf;
    packet->update_time = update_time;
    packet->front_radar_altit = front_radar_altit;
    packet->back_radar_altit = back_radar_altit;
    packet->follow_radar_altit = follow_radar_altit;
    packet->front_radar_flag = front_radar_flag;
    packet->back_radar_flag = back_radar_flag;
    packet->follow_radar_flag = follow_radar_flag;
    packet->flow_speed = flow_speed;
    packet->residual_pesticide = residual_pesticide;
    packet->uhc_liquid = uhc_liquid;
    packet->cur_spray_area = cur_spray_area;
    packet->tar_spray_area = tar_spray_area;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_DEVICE_DATA, (const char *)packet, TTALINK_MSG_ID_DEVICE_DATA_MIN_LEN, TTALINK_MSG_ID_DEVICE_DATA_LEN, TTALINK_MSG_ID_DEVICE_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_device_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, float front_radar_altit, float back_radar_altit, float follow_radar_altit, uint8_t front_radar_flag, uint8_t back_radar_flag, uint8_t follow_radar_flag, float flow_speed, float residual_pesticide, uint8_t uhc_liquid, float cur_spray_area, float tar_spray_area)
{
    _ttalink_device_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, front_radar_altit, back_radar_altit, follow_radar_altit, front_radar_flag, back_radar_flag, follow_radar_flag, flow_speed, residual_pesticide, uhc_liquid, cur_spray_area, tar_spray_area, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_device_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, float front_radar_altit, float back_radar_altit, float follow_radar_altit, uint8_t front_radar_flag, uint8_t back_radar_flag, uint8_t follow_radar_flag, float flow_speed, float residual_pesticide, uint8_t uhc_liquid, float cur_spray_area, float tar_spray_area)
{
    _ttalink_device_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, front_radar_altit, back_radar_altit, follow_radar_altit, front_radar_flag, back_radar_flag, follow_radar_flag, flow_speed, residual_pesticide, uhc_liquid, cur_spray_area, tar_spray_area, true);
}
#endif

#endif

// MESSAGE DEVICE_DATA UNPACKING


/**
 * @brief Get field update_time from device_data message
 *
 * @return  更新时间.
 */
static inline uint32_t ttalink_device_data_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field front_radar_altit from device_data message
 *
 * @return  前避障雷达距离.
 */
static inline float ttalink_device_data_get_front_radar_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  4);
}

/**
 * @brief Get field back_radar_altit from device_data message
 *
 * @return  后避障雷达距离.
 */
static inline float ttalink_device_data_get_back_radar_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  8);
}

/**
 * @brief Get field follow_radar_altit from device_data message
 *
 * @return  仿地雷达高度.
 */
static inline float ttalink_device_data_get_follow_radar_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  12);
}

/**
 * @brief Get field front_radar_flag from device_data message
 *
 * @return  前避障雷达可以标志. 0：不可用  1：可用
 */
static inline uint8_t ttalink_device_data_get_front_radar_flag(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field back_radar_flag from device_data message
 *
 * @return  后避障雷达可以标志.  0：不可用  1：可用
 */
static inline uint8_t ttalink_device_data_get_back_radar_flag(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field follow_radar_flag from device_data message
 *
 * @return  仿地雷达可以标志.  0：不可用  1：可用
 */
static inline uint8_t ttalink_device_data_get_follow_radar_flag(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field flow_speed from device_data message
 *
 * @return  流量计流速. 
 */
static inline float ttalink_device_data_get_flow_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  19);
}

/**
 * @brief Get field residual_pesticide from device_data message
 *
 * @return  剩余农药量（未加入）. 
 */
static inline float ttalink_device_data_get_residual_pesticide(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  23);
}

/**
 * @brief Get field uhc_liquid from device_data message
 *
 * @return  液位计是否有药量 0：没有  1：有. 
 */
static inline uint8_t ttalink_device_data_get_uhc_liquid(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field cur_spray_area from device_data message
 *
 * @return  当前已喷洒面积. 
 */
static inline float ttalink_device_data_get_cur_spray_area(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  28);
}

/**
 * @brief Get field tar_spray_area from device_data message
 *
 * @return  需要喷洒的面积. 
 */
static inline float ttalink_device_data_get_tar_spray_area(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Decode a device_data message into a struct
 *
 * @param msg The message to decode
 * @param device_data C-struct to decode the message contents into
 */
static inline void ttalink_device_data_decode(const ttalink_message_t* msg, ttalink_device_data_t* device_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    device_data->update_time = ttalink_device_data_get_update_time(msg);
    device_data->front_radar_altit = ttalink_device_data_get_front_radar_altit(msg);
    device_data->back_radar_altit = ttalink_device_data_get_back_radar_altit(msg);
    device_data->follow_radar_altit = ttalink_device_data_get_follow_radar_altit(msg);
    device_data->front_radar_flag = ttalink_device_data_get_front_radar_flag(msg);
    device_data->back_radar_flag = ttalink_device_data_get_back_radar_flag(msg);
    device_data->follow_radar_flag = ttalink_device_data_get_follow_radar_flag(msg);
    device_data->flow_speed = ttalink_device_data_get_flow_speed(msg);
    device_data->residual_pesticide = ttalink_device_data_get_residual_pesticide(msg);
    device_data->uhc_liquid = ttalink_device_data_get_uhc_liquid(msg);
    device_data->cur_spray_area = ttalink_device_data_get_cur_spray_area(msg);
    device_data->tar_spray_area = ttalink_device_data_get_tar_spray_area(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_DEVICE_DATA_LEN? msg->len : TTALINK_MSG_ID_DEVICE_DATA_LEN;
        memset(device_data, 0, TTALINK_MSG_ID_DEVICE_DATA_LEN);
    memcpy(device_data, _TTA_PAYLOAD(msg), len);
#endif
}
