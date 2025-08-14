#pragma once
// MESSAGE SENSOR_CALIBRATION PACKING

#define TTALINK_MSG_ID_SENSOR_CALIBRATION 2150

TTAPACKED(
typedef struct __ttalink_sensor_calibration_t {
 uint32_t simple_time; /*< .*/
 uint8_t calibration_type; /*<  .*/
}) ttalink_sensor_calibration_t;

#define TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN 5
#define TTALINK_MSG_ID_SENSOR_CALIBRATION_MIN_LEN 5
#define TTALINK_MSG_ID_2150_LEN 5
#define TTALINK_MSG_ID_2150_MIN_LEN 5

#define TTALINK_MSG_ID_SENSOR_CALIBRATION_CRC 35
#define TTALINK_MSG_ID_2150_CRC 35



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SENSOR_CALIBRATION { \
    2150, \
    "SENSOR_CALIBRATION", \
    2, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sensor_calibration_t, simple_time) }, \
         { "calibration_type", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_sensor_calibration_t, calibration_type) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SENSOR_CALIBRATION { \
    "SENSOR_CALIBRATION", \
    2, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sensor_calibration_t, simple_time) }, \
         { "calibration_type", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_sensor_calibration_t, calibration_type) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sensor_calibration_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint8_t calibration_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, calibration_type);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN);
#else
    ttalink_sensor_calibration_t packet;
    packet.simple_time = simple_time;
    packet.calibration_type = calibration_type;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SENSOR_CALIBRATION;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SENSOR_CALIBRATION_MIN_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_CRC, nocrc);
}

/**
 * @brief Pack a sensor_calibration message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time .
 * @param calibration_type  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_calibration_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint8_t calibration_type)
{
    return _ttalink_sensor_calibration_pack(dst_addr, src_addr, msg,  simple_time, calibration_type, false);
}

/**
 * @brief Pack a sensor_calibration message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time .
 * @param calibration_type  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_calibration_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint8_t calibration_type)
{
    return _ttalink_sensor_calibration_pack(dst_addr, src_addr, msg,  simple_time, calibration_type, true);
}


static inline uint16_t _ttalink_sensor_calibration_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint8_t calibration_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, calibration_type);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN);
#else
    ttalink_sensor_calibration_t packet;
    packet.simple_time = simple_time;
    packet.calibration_type = calibration_type;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SENSOR_CALIBRATION;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_CALIBRATION_MIN_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_CRC, nocrc);
}

/**
 * @brief Pack a sensor_calibration message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time .
 * @param calibration_type  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_calibration_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint8_t calibration_type)
{
    return _ttalink_sensor_calibration_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, calibration_type, false);
}

/**
 * @brief Pack a sensor_calibration message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time .
 * @param calibration_type  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_calibration_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint8_t calibration_type)
{
    return _ttalink_sensor_calibration_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, calibration_type, true);
}


static inline uint16_t _ttalink_sensor_calibration_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensor_calibration_t* sensor_calibration, bool nocrc)
{
    if(nocrc){
        return ttalink_sensor_calibration_pack_nocrc(dst_addr, src_addr, msg, sensor_calibration->simple_time, sensor_calibration->calibration_type);
    }else{
        return ttalink_sensor_calibration_pack(dst_addr, src_addr, msg, sensor_calibration->simple_time, sensor_calibration->calibration_type);
    }
    
}

/**
 * @brief Encode a sensor_calibration struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sensor_calibration C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_calibration_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensor_calibration_t* sensor_calibration)
{
    return _ttalink_sensor_calibration_encode(dst_addr, src_addr, msg, sensor_calibration, false);
}

/**
 * @brief Encode a sensor_calibration struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sensor_calibration C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_calibration_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensor_calibration_t* sensor_calibration)
{
    return _ttalink_sensor_calibration_encode(dst_addr, src_addr, msg, sensor_calibration, true);
}


static inline uint16_t _ttalink_sensor_calibration_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensor_calibration_t* sensor_calibration, bool nocrc)
{
    if(nocrc){
        return ttalink_sensor_calibration_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sensor_calibration->simple_time, sensor_calibration->calibration_type);
    }else{
        return ttalink_sensor_calibration_pack_chan(dst_addr, src_addr, chan, msg, sensor_calibration->simple_time, sensor_calibration->calibration_type);
    }
}

/**
 * @brief Encode a sensor_calibration struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor_calibration C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_calibration_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensor_calibration_t* sensor_calibration)
{
    return _ttalink_sensor_calibration_encode_chan(dst_addr, src_addr, chan, msg, sensor_calibration, false);
}

/**
 * @brief Encode a sensor_calibration struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor_calibration C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_calibration_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensor_calibration_t* sensor_calibration)
{
    return _ttalink_sensor_calibration_encode_chan(dst_addr, src_addr, chan, msg, sensor_calibration, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sensor_calibration_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint8_t calibration_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, calibration_type);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_CALIBRATION, buf, TTALINK_MSG_ID_SENSOR_CALIBRATION_MIN_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_CRC, nocrc);
#else
    ttalink_sensor_calibration_t packet;
    packet.simple_time = simple_time;
    packet.calibration_type = calibration_type;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_CALIBRATION, (const char *)&packet, TTALINK_MSG_ID_SENSOR_CALIBRATION_MIN_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_CRC, nocrc);
#endif
}

/**
 * @brief Send a sensor_calibration message
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time .
 * @param calibration_type  .
 */
static inline void ttalink_sensor_calibration_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint8_t calibration_type)
{
    _ttalink_sensor_calibration_send(dst_addr, src_addr, chan, simple_time, calibration_type, false);
}

/**
 * @brief Send a sensor_calibration message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time .
 * @param calibration_type  .
 */
static inline void ttalink_sensor_calibration_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint8_t calibration_type)
{
    _ttalink_sensor_calibration_send(dst_addr, src_addr, chan, simple_time, calibration_type, true);
}


static inline void _ttalink_sensor_calibration_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensor_calibration_t* sensor_calibration, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sensor_calibration_send_nocrc(dst_addr, src_addr, chan, sensor_calibration->simple_time, sensor_calibration->calibration_type);
    }else{
        ttalink_sensor_calibration_send(dst_addr, src_addr, chan, sensor_calibration->simple_time, sensor_calibration->calibration_type);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_CALIBRATION, (const char *)sensor_calibration, TTALINK_MSG_ID_SENSOR_CALIBRATION_MIN_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_CRC, nocrc);
#endif
}

/**
 * @brief Send a sensor_calibration message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sensor_calibration_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensor_calibration_t* sensor_calibration)
{
    _ttalink_sensor_calibration_send_struct(dst_addr, src_addr, chan, sensor_calibration, false);
}

/**
 * @brief Send a sensor_calibration message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sensor_calibration_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensor_calibration_t* sensor_calibration)
{
    _ttalink_sensor_calibration_send_struct(dst_addr, src_addr, chan, sensor_calibration, true);
}

#if TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sensor_calibration_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint8_t calibration_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, calibration_type);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_CALIBRATION, buf, TTALINK_MSG_ID_SENSOR_CALIBRATION_MIN_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_CRC, nocrc);
#else
    ttalink_sensor_calibration_t *packet = (ttalink_sensor_calibration_t *)msgbuf;
    packet->simple_time = simple_time;
    packet->calibration_type = calibration_type;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_CALIBRATION, (const char *)packet, TTALINK_MSG_ID_SENSOR_CALIBRATION_MIN_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN, TTALINK_MSG_ID_SENSOR_CALIBRATION_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sensor_calibration_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint8_t calibration_type)
{
    _ttalink_sensor_calibration_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, calibration_type, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sensor_calibration_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint8_t calibration_type)
{
    _ttalink_sensor_calibration_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, calibration_type, true);
}
#endif

#endif

// MESSAGE SENSOR_CALIBRATION UNPACKING


/**
 * @brief Get field simple_time from sensor_calibration message
 *
 * @return .
 */
static inline uint32_t ttalink_sensor_calibration_get_simple_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field calibration_type from sensor_calibration message
 *
 * @return  .
 */
static inline uint8_t ttalink_sensor_calibration_get_calibration_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Decode a sensor_calibration message into a struct
 *
 * @param msg The message to decode
 * @param sensor_calibration C-struct to decode the message contents into
 */
static inline void ttalink_sensor_calibration_decode(const ttalink_message_t* msg, ttalink_sensor_calibration_t* sensor_calibration)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sensor_calibration->simple_time = ttalink_sensor_calibration_get_simple_time(msg);
    sensor_calibration->calibration_type = ttalink_sensor_calibration_get_calibration_type(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN? msg->len : TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN;
        memset(sensor_calibration, 0, TTALINK_MSG_ID_SENSOR_CALIBRATION_LEN);
    memcpy(sensor_calibration, _TTA_PAYLOAD(msg), len);
#endif
}
