#pragma once
// MESSAGE SENSOR_CALIBRATION_ACK PACKING

#define TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK 2151

TTAPACKED(
typedef struct __ttalink_sensor_calibration_ack_t {
 uint32_t simple_time; /*< .*/
 uint8_t calibration_type; /*< .*/
 uint8_t sensor_nums; /*< 校准的传感器数量*/
 uint8_t sensor_index; /*< 传感器序号*/
 uint8_t progress_percent; /*< 校准进度百分比取值范围0-100*/
 uint8_t ack; /*< 校准状态，使用各个校准类型下的状态枚举*/
}) ttalink_sensor_calibration_ack_t;

#define TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN 9
#define TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_MIN_LEN 9
#define TTALINK_MSG_ID_2151_LEN 9
#define TTALINK_MSG_ID_2151_MIN_LEN 9

#define TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_CRC 189
#define TTALINK_MSG_ID_2151_CRC 189



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SENSOR_CALIBRATION_ACK { \
    2151, \
    "SENSOR_CALIBRATION_ACK", \
    6, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sensor_calibration_ack_t, simple_time) }, \
         { "calibration_type", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_sensor_calibration_ack_t, calibration_type) }, \
         { "sensor_nums", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_sensor_calibration_ack_t, sensor_nums) }, \
         { "sensor_index", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_sensor_calibration_ack_t, sensor_index) }, \
         { "progress_percent", NULL, TTALINK_TYPE_UINT8_T, 0, 7, offsetof(ttalink_sensor_calibration_ack_t, progress_percent) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_sensor_calibration_ack_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SENSOR_CALIBRATION_ACK { \
    "SENSOR_CALIBRATION_ACK", \
    6, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sensor_calibration_ack_t, simple_time) }, \
         { "calibration_type", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_sensor_calibration_ack_t, calibration_type) }, \
         { "sensor_nums", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_sensor_calibration_ack_t, sensor_nums) }, \
         { "sensor_index", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_sensor_calibration_ack_t, sensor_index) }, \
         { "progress_percent", NULL, TTALINK_TYPE_UINT8_T, 0, 7, offsetof(ttalink_sensor_calibration_ack_t, progress_percent) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_sensor_calibration_ack_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sensor_calibration_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint8_t calibration_type, uint8_t sensor_nums, uint8_t sensor_index, uint8_t progress_percent, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, calibration_type);
    _tta_put_uint8_t(buf, 5, sensor_nums);
    _tta_put_uint8_t(buf, 6, sensor_index);
    _tta_put_uint8_t(buf, 7, progress_percent);
    _tta_put_uint8_t(buf, 8, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN);
#else
    ttalink_sensor_calibration_ack_t packet;
    packet.simple_time = simple_time;
    packet.calibration_type = calibration_type;
    packet.sensor_nums = sensor_nums;
    packet.sensor_index = sensor_index;
    packet.progress_percent = progress_percent;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_MIN_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_CRC, nocrc);
}

/**
 * @brief Pack a sensor_calibration_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time .
 * @param calibration_type .
 * @param sensor_nums 校准的传感器数量
 * @param sensor_index 传感器序号
 * @param progress_percent 校准进度百分比取值范围0-100
 * @param ack 校准状态，使用各个校准类型下的状态枚举
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_calibration_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint8_t calibration_type, uint8_t sensor_nums, uint8_t sensor_index, uint8_t progress_percent, uint8_t ack)
{
    return _ttalink_sensor_calibration_ack_pack(dst_addr, src_addr, msg,  simple_time, calibration_type, sensor_nums, sensor_index, progress_percent, ack, false);
}

/**
 * @brief Pack a sensor_calibration_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time .
 * @param calibration_type .
 * @param sensor_nums 校准的传感器数量
 * @param sensor_index 传感器序号
 * @param progress_percent 校准进度百分比取值范围0-100
 * @param ack 校准状态，使用各个校准类型下的状态枚举
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_calibration_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint8_t calibration_type, uint8_t sensor_nums, uint8_t sensor_index, uint8_t progress_percent, uint8_t ack)
{
    return _ttalink_sensor_calibration_ack_pack(dst_addr, src_addr, msg,  simple_time, calibration_type, sensor_nums, sensor_index, progress_percent, ack, true);
}


static inline uint16_t _ttalink_sensor_calibration_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint8_t calibration_type,uint8_t sensor_nums,uint8_t sensor_index,uint8_t progress_percent,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, calibration_type);
    _tta_put_uint8_t(buf, 5, sensor_nums);
    _tta_put_uint8_t(buf, 6, sensor_index);
    _tta_put_uint8_t(buf, 7, progress_percent);
    _tta_put_uint8_t(buf, 8, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN);
#else
    ttalink_sensor_calibration_ack_t packet;
    packet.simple_time = simple_time;
    packet.calibration_type = calibration_type;
    packet.sensor_nums = sensor_nums;
    packet.sensor_index = sensor_index;
    packet.progress_percent = progress_percent;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_MIN_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_CRC, nocrc);
}

/**
 * @brief Pack a sensor_calibration_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time .
 * @param calibration_type .
 * @param sensor_nums 校准的传感器数量
 * @param sensor_index 传感器序号
 * @param progress_percent 校准进度百分比取值范围0-100
 * @param ack 校准状态，使用各个校准类型下的状态枚举
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_calibration_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint8_t calibration_type,uint8_t sensor_nums,uint8_t sensor_index,uint8_t progress_percent,uint8_t ack)
{
    return _ttalink_sensor_calibration_ack_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, calibration_type, sensor_nums, sensor_index, progress_percent, ack, false);
}

/**
 * @brief Pack a sensor_calibration_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time .
 * @param calibration_type .
 * @param sensor_nums 校准的传感器数量
 * @param sensor_index 传感器序号
 * @param progress_percent 校准进度百分比取值范围0-100
 * @param ack 校准状态，使用各个校准类型下的状态枚举
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_calibration_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint8_t calibration_type,uint8_t sensor_nums,uint8_t sensor_index,uint8_t progress_percent,uint8_t ack)
{
    return _ttalink_sensor_calibration_ack_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, calibration_type, sensor_nums, sensor_index, progress_percent, ack, true);
}


static inline uint16_t _ttalink_sensor_calibration_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensor_calibration_ack_t* sensor_calibration_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_sensor_calibration_ack_pack_nocrc(dst_addr, src_addr, msg, sensor_calibration_ack->simple_time, sensor_calibration_ack->calibration_type, sensor_calibration_ack->sensor_nums, sensor_calibration_ack->sensor_index, sensor_calibration_ack->progress_percent, sensor_calibration_ack->ack);
    }else{
        return ttalink_sensor_calibration_ack_pack(dst_addr, src_addr, msg, sensor_calibration_ack->simple_time, sensor_calibration_ack->calibration_type, sensor_calibration_ack->sensor_nums, sensor_calibration_ack->sensor_index, sensor_calibration_ack->progress_percent, sensor_calibration_ack->ack);
    }
    
}

/**
 * @brief Encode a sensor_calibration_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sensor_calibration_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_calibration_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensor_calibration_ack_t* sensor_calibration_ack)
{
    return _ttalink_sensor_calibration_ack_encode(dst_addr, src_addr, msg, sensor_calibration_ack, false);
}

/**
 * @brief Encode a sensor_calibration_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sensor_calibration_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_calibration_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensor_calibration_ack_t* sensor_calibration_ack)
{
    return _ttalink_sensor_calibration_ack_encode(dst_addr, src_addr, msg, sensor_calibration_ack, true);
}


static inline uint16_t _ttalink_sensor_calibration_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensor_calibration_ack_t* sensor_calibration_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_sensor_calibration_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sensor_calibration_ack->simple_time, sensor_calibration_ack->calibration_type, sensor_calibration_ack->sensor_nums, sensor_calibration_ack->sensor_index, sensor_calibration_ack->progress_percent, sensor_calibration_ack->ack);
    }else{
        return ttalink_sensor_calibration_ack_pack_chan(dst_addr, src_addr, chan, msg, sensor_calibration_ack->simple_time, sensor_calibration_ack->calibration_type, sensor_calibration_ack->sensor_nums, sensor_calibration_ack->sensor_index, sensor_calibration_ack->progress_percent, sensor_calibration_ack->ack);
    }
}

/**
 * @brief Encode a sensor_calibration_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor_calibration_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_calibration_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensor_calibration_ack_t* sensor_calibration_ack)
{
    return _ttalink_sensor_calibration_ack_encode_chan(dst_addr, src_addr, chan, msg, sensor_calibration_ack, false);
}

/**
 * @brief Encode a sensor_calibration_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor_calibration_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_calibration_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensor_calibration_ack_t* sensor_calibration_ack)
{
    return _ttalink_sensor_calibration_ack_encode_chan(dst_addr, src_addr, chan, msg, sensor_calibration_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sensor_calibration_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint8_t calibration_type, uint8_t sensor_nums, uint8_t sensor_index, uint8_t progress_percent, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, calibration_type);
    _tta_put_uint8_t(buf, 5, sensor_nums);
    _tta_put_uint8_t(buf, 6, sensor_index);
    _tta_put_uint8_t(buf, 7, progress_percent);
    _tta_put_uint8_t(buf, 8, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK, buf, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_MIN_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_CRC, nocrc);
#else
    ttalink_sensor_calibration_ack_t packet;
    packet.simple_time = simple_time;
    packet.calibration_type = calibration_type;
    packet.sensor_nums = sensor_nums;
    packet.sensor_index = sensor_index;
    packet.progress_percent = progress_percent;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK, (const char *)&packet, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_MIN_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a sensor_calibration_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time .
 * @param calibration_type .
 * @param sensor_nums 校准的传感器数量
 * @param sensor_index 传感器序号
 * @param progress_percent 校准进度百分比取值范围0-100
 * @param ack 校准状态，使用各个校准类型下的状态枚举
 */
static inline void ttalink_sensor_calibration_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint8_t calibration_type, uint8_t sensor_nums, uint8_t sensor_index, uint8_t progress_percent, uint8_t ack)
{
    _ttalink_sensor_calibration_ack_send(dst_addr, src_addr, chan, simple_time, calibration_type, sensor_nums, sensor_index, progress_percent, ack, false);
}

/**
 * @brief Send a sensor_calibration_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time .
 * @param calibration_type .
 * @param sensor_nums 校准的传感器数量
 * @param sensor_index 传感器序号
 * @param progress_percent 校准进度百分比取值范围0-100
 * @param ack 校准状态，使用各个校准类型下的状态枚举
 */
static inline void ttalink_sensor_calibration_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint8_t calibration_type, uint8_t sensor_nums, uint8_t sensor_index, uint8_t progress_percent, uint8_t ack)
{
    _ttalink_sensor_calibration_ack_send(dst_addr, src_addr, chan, simple_time, calibration_type, sensor_nums, sensor_index, progress_percent, ack, true);
}


static inline void _ttalink_sensor_calibration_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensor_calibration_ack_t* sensor_calibration_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sensor_calibration_ack_send_nocrc(dst_addr, src_addr, chan, sensor_calibration_ack->simple_time, sensor_calibration_ack->calibration_type, sensor_calibration_ack->sensor_nums, sensor_calibration_ack->sensor_index, sensor_calibration_ack->progress_percent, sensor_calibration_ack->ack);
    }else{
        ttalink_sensor_calibration_ack_send(dst_addr, src_addr, chan, sensor_calibration_ack->simple_time, sensor_calibration_ack->calibration_type, sensor_calibration_ack->sensor_nums, sensor_calibration_ack->sensor_index, sensor_calibration_ack->progress_percent, sensor_calibration_ack->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK, (const char *)sensor_calibration_ack, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_MIN_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a sensor_calibration_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sensor_calibration_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensor_calibration_ack_t* sensor_calibration_ack)
{
    _ttalink_sensor_calibration_ack_send_struct(dst_addr, src_addr, chan, sensor_calibration_ack, false);
}

/**
 * @brief Send a sensor_calibration_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sensor_calibration_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensor_calibration_ack_t* sensor_calibration_ack)
{
    _ttalink_sensor_calibration_ack_send_struct(dst_addr, src_addr, chan, sensor_calibration_ack, true);
}

#if TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sensor_calibration_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint8_t calibration_type, uint8_t sensor_nums, uint8_t sensor_index, uint8_t progress_percent, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, calibration_type);
    _tta_put_uint8_t(buf, 5, sensor_nums);
    _tta_put_uint8_t(buf, 6, sensor_index);
    _tta_put_uint8_t(buf, 7, progress_percent);
    _tta_put_uint8_t(buf, 8, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK, buf, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_MIN_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_CRC, nocrc);
#else
    ttalink_sensor_calibration_ack_t *packet = (ttalink_sensor_calibration_ack_t *)msgbuf;
    packet->simple_time = simple_time;
    packet->calibration_type = calibration_type;
    packet->sensor_nums = sensor_nums;
    packet->sensor_index = sensor_index;
    packet->progress_percent = progress_percent;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK, (const char *)packet, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_MIN_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sensor_calibration_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint8_t calibration_type, uint8_t sensor_nums, uint8_t sensor_index, uint8_t progress_percent, uint8_t ack)
{
    _ttalink_sensor_calibration_ack_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, calibration_type, sensor_nums, sensor_index, progress_percent, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sensor_calibration_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint8_t calibration_type, uint8_t sensor_nums, uint8_t sensor_index, uint8_t progress_percent, uint8_t ack)
{
    _ttalink_sensor_calibration_ack_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, calibration_type, sensor_nums, sensor_index, progress_percent, ack, true);
}
#endif

#endif

// MESSAGE SENSOR_CALIBRATION_ACK UNPACKING


/**
 * @brief Get field simple_time from sensor_calibration_ack message
 *
 * @return .
 */
static inline uint32_t ttalink_sensor_calibration_ack_get_simple_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field calibration_type from sensor_calibration_ack message
 *
 * @return .
 */
static inline uint8_t ttalink_sensor_calibration_ack_get_calibration_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field sensor_nums from sensor_calibration_ack message
 *
 * @return 校准的传感器数量
 */
static inline uint8_t ttalink_sensor_calibration_ack_get_sensor_nums(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field sensor_index from sensor_calibration_ack message
 *
 * @return 传感器序号
 */
static inline uint8_t ttalink_sensor_calibration_ack_get_sensor_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field progress_percent from sensor_calibration_ack message
 *
 * @return 校准进度百分比取值范围0-100
 */
static inline uint8_t ttalink_sensor_calibration_ack_get_progress_percent(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field ack from sensor_calibration_ack message
 *
 * @return 校准状态，使用各个校准类型下的状态枚举
 */
static inline uint8_t ttalink_sensor_calibration_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Decode a sensor_calibration_ack message into a struct
 *
 * @param msg The message to decode
 * @param sensor_calibration_ack C-struct to decode the message contents into
 */
static inline void ttalink_sensor_calibration_ack_decode(const ttalink_message_t* msg, ttalink_sensor_calibration_ack_t* sensor_calibration_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sensor_calibration_ack->simple_time = ttalink_sensor_calibration_ack_get_simple_time(msg);
    sensor_calibration_ack->calibration_type = ttalink_sensor_calibration_ack_get_calibration_type(msg);
    sensor_calibration_ack->sensor_nums = ttalink_sensor_calibration_ack_get_sensor_nums(msg);
    sensor_calibration_ack->sensor_index = ttalink_sensor_calibration_ack_get_sensor_index(msg);
    sensor_calibration_ack->progress_percent = ttalink_sensor_calibration_ack_get_progress_percent(msg);
    sensor_calibration_ack->ack = ttalink_sensor_calibration_ack_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN? msg->len : TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN;
        memset(sensor_calibration_ack, 0, TTALINK_MSG_ID_SENSOR_CALIBRATION_ACK_LEN);
    memcpy(sensor_calibration_ack, _TTA_PAYLOAD(msg), len);
#endif
}
