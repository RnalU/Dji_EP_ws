#pragma once
// MESSAGE MAG_SENSORS_DATA PACKING

#define TTALINK_MSG_ID_MAG_SENSORS_DATA 2173

TTAPACKED(
typedef struct __ttalink_mag_sensors_data_t {
 uint8_t sensor_index; /*< .*/
 uint32_t update_time; /*<  .*/
 int16_t mag_raw[3]; /*<  用来传输SH-FC原始值/FC-ROS校准后的值.*/
}) ttalink_mag_sensors_data_t;

#define TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN 11
#define TTALINK_MSG_ID_MAG_SENSORS_DATA_MIN_LEN 11
#define TTALINK_MSG_ID_2173_LEN 11
#define TTALINK_MSG_ID_2173_MIN_LEN 11

#define TTALINK_MSG_ID_MAG_SENSORS_DATA_CRC 118
#define TTALINK_MSG_ID_2173_CRC 118

#define TTALINK_MSG_MAG_SENSORS_DATA_FIELD_MAG_RAW_LEN 3

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_MAG_SENSORS_DATA { \
    2173, \
    "MAG_SENSORS_DATA", \
    3, \
    {  { "sensor_index", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_mag_sensors_data_t, sensor_index) }, \
         { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 1, offsetof(ttalink_mag_sensors_data_t, update_time) }, \
         { "mag_raw", NULL, TTALINK_TYPE_INT16_T, 3, 5, offsetof(ttalink_mag_sensors_data_t, mag_raw) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_MAG_SENSORS_DATA { \
    "MAG_SENSORS_DATA", \
    3, \
    {  { "sensor_index", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_mag_sensors_data_t, sensor_index) }, \
         { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 1, offsetof(ttalink_mag_sensors_data_t, update_time) }, \
         { "mag_raw", NULL, TTALINK_TYPE_INT16_T, 3, 5, offsetof(ttalink_mag_sensors_data_t, mag_raw) }, \
         } \
}
#endif


static inline uint16_t _ttalink_mag_sensors_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t sensor_index, uint32_t update_time, const int16_t *mag_raw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN];
    _tta_put_uint8_t(buf, 0, sensor_index);
    _tta_put_uint32_t(buf, 1, update_time);
    _tta_put_int16_t_array(buf, 5, mag_raw, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN);
#else
    ttalink_mag_sensors_data_t packet;
    packet.sensor_index = sensor_index;
    packet.update_time = update_time;
    tta_array_memcpy(packet.mag_raw, mag_raw, sizeof(int16_t)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MAG_SENSORS_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_MAG_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN, TTALINK_MSG_ID_MAG_SENSORS_DATA_CRC, nocrc);
}

/**
 * @brief Pack a mag_sensors_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sensor_index .
 * @param update_time  .
 * @param mag_raw  用来传输SH-FC原始值/FC-ROS校准后的值.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mag_sensors_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t sensor_index, uint32_t update_time, const int16_t *mag_raw)
{
    return _ttalink_mag_sensors_data_pack(dst_addr, src_addr, msg,  sensor_index, update_time, mag_raw, false);
}

/**
 * @brief Pack a mag_sensors_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sensor_index .
 * @param update_time  .
 * @param mag_raw  用来传输SH-FC原始值/FC-ROS校准后的值.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mag_sensors_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t sensor_index, uint32_t update_time, const int16_t *mag_raw)
{
    return _ttalink_mag_sensors_data_pack(dst_addr, src_addr, msg,  sensor_index, update_time, mag_raw, true);
}


static inline uint16_t _ttalink_mag_sensors_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t sensor_index,uint32_t update_time,const int16_t *mag_raw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN];
    _tta_put_uint8_t(buf, 0, sensor_index);
    _tta_put_uint32_t(buf, 1, update_time);
    _tta_put_int16_t_array(buf, 5, mag_raw, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN);
#else
    ttalink_mag_sensors_data_t packet;
    packet.sensor_index = sensor_index;
    packet.update_time = update_time;
    tta_array_memcpy(packet.mag_raw, mag_raw, sizeof(int16_t)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MAG_SENSORS_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_MAG_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN, TTALINK_MSG_ID_MAG_SENSORS_DATA_CRC, nocrc);
}

/**
 * @brief Pack a mag_sensors_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor_index .
 * @param update_time  .
 * @param mag_raw  用来传输SH-FC原始值/FC-ROS校准后的值.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mag_sensors_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t sensor_index,uint32_t update_time,const int16_t *mag_raw)
{
    return _ttalink_mag_sensors_data_pack_chan(dst_addr, src_addr, chan, msg,  sensor_index, update_time, mag_raw, false);
}

/**
 * @brief Pack a mag_sensors_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor_index .
 * @param update_time  .
 * @param mag_raw  用来传输SH-FC原始值/FC-ROS校准后的值.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mag_sensors_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t sensor_index,uint32_t update_time,const int16_t *mag_raw)
{
    return _ttalink_mag_sensors_data_pack_chan(dst_addr, src_addr, chan, msg,  sensor_index, update_time, mag_raw, true);
}


static inline uint16_t _ttalink_mag_sensors_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mag_sensors_data_t* mag_sensors_data, bool nocrc)
{
    if(nocrc){
        return ttalink_mag_sensors_data_pack_nocrc(dst_addr, src_addr, msg, mag_sensors_data->sensor_index, mag_sensors_data->update_time, mag_sensors_data->mag_raw);
    }else{
        return ttalink_mag_sensors_data_pack(dst_addr, src_addr, msg, mag_sensors_data->sensor_index, mag_sensors_data->update_time, mag_sensors_data->mag_raw);
    }
    
}

/**
 * @brief Encode a mag_sensors_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param mag_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_mag_sensors_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mag_sensors_data_t* mag_sensors_data)
{
    return _ttalink_mag_sensors_data_encode(dst_addr, src_addr, msg, mag_sensors_data, false);
}

/**
 * @brief Encode a mag_sensors_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param mag_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_mag_sensors_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mag_sensors_data_t* mag_sensors_data)
{
    return _ttalink_mag_sensors_data_encode(dst_addr, src_addr, msg, mag_sensors_data, true);
}


static inline uint16_t _ttalink_mag_sensors_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mag_sensors_data_t* mag_sensors_data, bool nocrc)
{
    if(nocrc){
        return ttalink_mag_sensors_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, mag_sensors_data->sensor_index, mag_sensors_data->update_time, mag_sensors_data->mag_raw);
    }else{
        return ttalink_mag_sensors_data_pack_chan(dst_addr, src_addr, chan, msg, mag_sensors_data->sensor_index, mag_sensors_data->update_time, mag_sensors_data->mag_raw);
    }
}

/**
 * @brief Encode a mag_sensors_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mag_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_mag_sensors_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mag_sensors_data_t* mag_sensors_data)
{
    return _ttalink_mag_sensors_data_encode_chan(dst_addr, src_addr, chan, msg, mag_sensors_data, false);
}

/**
 * @brief Encode a mag_sensors_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mag_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_mag_sensors_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mag_sensors_data_t* mag_sensors_data)
{
    return _ttalink_mag_sensors_data_encode_chan(dst_addr, src_addr, chan, msg, mag_sensors_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_mag_sensors_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t sensor_index, uint32_t update_time, const int16_t *mag_raw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN];
    _tta_put_uint8_t(buf, 0, sensor_index);
    _tta_put_uint32_t(buf, 1, update_time);
    _tta_put_int16_t_array(buf, 5, mag_raw, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAG_SENSORS_DATA, buf, TTALINK_MSG_ID_MAG_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN, TTALINK_MSG_ID_MAG_SENSORS_DATA_CRC, nocrc);
#else
    ttalink_mag_sensors_data_t packet;
    packet.sensor_index = sensor_index;
    packet.update_time = update_time;
    tta_array_memcpy(packet.mag_raw, mag_raw, sizeof(int16_t)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAG_SENSORS_DATA, (const char *)&packet, TTALINK_MSG_ID_MAG_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN, TTALINK_MSG_ID_MAG_SENSORS_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a mag_sensors_data message
 * @param chan TTAlink channel to send the message
 *
 * @param sensor_index .
 * @param update_time  .
 * @param mag_raw  用来传输SH-FC原始值/FC-ROS校准后的值.
 */
static inline void ttalink_mag_sensors_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t sensor_index, uint32_t update_time, const int16_t *mag_raw)
{
    _ttalink_mag_sensors_data_send(dst_addr, src_addr, chan, sensor_index, update_time, mag_raw, false);
}

/**
 * @brief Send a mag_sensors_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param sensor_index .
 * @param update_time  .
 * @param mag_raw  用来传输SH-FC原始值/FC-ROS校准后的值.
 */
static inline void ttalink_mag_sensors_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t sensor_index, uint32_t update_time, const int16_t *mag_raw)
{
    _ttalink_mag_sensors_data_send(dst_addr, src_addr, chan, sensor_index, update_time, mag_raw, true);
}


static inline void _ttalink_mag_sensors_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mag_sensors_data_t* mag_sensors_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_mag_sensors_data_send_nocrc(dst_addr, src_addr, chan, mag_sensors_data->sensor_index, mag_sensors_data->update_time, mag_sensors_data->mag_raw);
    }else{
        ttalink_mag_sensors_data_send(dst_addr, src_addr, chan, mag_sensors_data->sensor_index, mag_sensors_data->update_time, mag_sensors_data->mag_raw);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAG_SENSORS_DATA, (const char *)mag_sensors_data, TTALINK_MSG_ID_MAG_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN, TTALINK_MSG_ID_MAG_SENSORS_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a mag_sensors_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_mag_sensors_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mag_sensors_data_t* mag_sensors_data)
{
    _ttalink_mag_sensors_data_send_struct(dst_addr, src_addr, chan, mag_sensors_data, false);
}

/**
 * @brief Send a mag_sensors_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_mag_sensors_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mag_sensors_data_t* mag_sensors_data)
{
    _ttalink_mag_sensors_data_send_struct(dst_addr, src_addr, chan, mag_sensors_data, true);
}

#if TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_mag_sensors_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t sensor_index, uint32_t update_time, const int16_t *mag_raw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, sensor_index);
    _tta_put_uint32_t(buf, 1, update_time);
    _tta_put_int16_t_array(buf, 5, mag_raw, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAG_SENSORS_DATA, buf, TTALINK_MSG_ID_MAG_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN, TTALINK_MSG_ID_MAG_SENSORS_DATA_CRC, nocrc);
#else
    ttalink_mag_sensors_data_t *packet = (ttalink_mag_sensors_data_t *)msgbuf;
    packet->sensor_index = sensor_index;
    packet->update_time = update_time;
    tta_array_memcpy(packet->mag_raw, mag_raw, sizeof(int16_t)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAG_SENSORS_DATA, (const char *)packet, TTALINK_MSG_ID_MAG_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN, TTALINK_MSG_ID_MAG_SENSORS_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_mag_sensors_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t sensor_index, uint32_t update_time, const int16_t *mag_raw)
{
    _ttalink_mag_sensors_data_send_buf(dst_addr, src_addr, msgbuf, chan, sensor_index, update_time, mag_raw, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_mag_sensors_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t sensor_index, uint32_t update_time, const int16_t *mag_raw)
{
    _ttalink_mag_sensors_data_send_buf(dst_addr, src_addr, msgbuf, chan, sensor_index, update_time, mag_raw, true);
}
#endif

#endif

// MESSAGE MAG_SENSORS_DATA UNPACKING


/**
 * @brief Get field sensor_index from mag_sensors_data message
 *
 * @return .
 */
static inline uint8_t ttalink_mag_sensors_data_get_sensor_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field update_time from mag_sensors_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_mag_sensors_data_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  1);
}

/**
 * @brief Get field mag_raw from mag_sensors_data message
 *
 * @return  用来传输SH-FC原始值/FC-ROS校准后的值.
 */
static inline uint16_t ttalink_mag_sensors_data_get_mag_raw(const ttalink_message_t* msg, int16_t *mag_raw)
{
    return _TTA_RETURN_int16_t_array(msg, mag_raw, 3,  5);
}

/**
 * @brief Decode a mag_sensors_data message into a struct
 *
 * @param msg The message to decode
 * @param mag_sensors_data C-struct to decode the message contents into
 */
static inline void ttalink_mag_sensors_data_decode(const ttalink_message_t* msg, ttalink_mag_sensors_data_t* mag_sensors_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    mag_sensors_data->sensor_index = ttalink_mag_sensors_data_get_sensor_index(msg);
    mag_sensors_data->update_time = ttalink_mag_sensors_data_get_update_time(msg);
    ttalink_mag_sensors_data_get_mag_raw(msg, mag_sensors_data->mag_raw);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN? msg->len : TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN;
        memset(mag_sensors_data, 0, TTALINK_MSG_ID_MAG_SENSORS_DATA_LEN);
    memcpy(mag_sensors_data, _TTA_PAYLOAD(msg), len);
#endif
}
