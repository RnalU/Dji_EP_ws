#pragma once
// MESSAGE ROSUAV_BARO_RAW PACKING

#define TTALINK_MSG_ID_ROSUAV_BARO_RAW 8194

TTAPACKED(
typedef struct __ttalink_rosuav_baro_raw_t {
 float pressure[2]; /*<  .*/
 float attitude[2]; /*<  .*/
 float reserve[4]; /*<  .*/
}) ttalink_rosuav_baro_raw_t;

#define TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN 32
#define TTALINK_MSG_ID_ROSUAV_BARO_RAW_MIN_LEN 32
#define TTALINK_MSG_ID_8194_LEN 32
#define TTALINK_MSG_ID_8194_MIN_LEN 32

#define TTALINK_MSG_ID_ROSUAV_BARO_RAW_CRC 65
#define TTALINK_MSG_ID_8194_CRC 65

#define TTALINK_MSG_ROSUAV_BARO_RAW_FIELD_PRESSURE_LEN 2
#define TTALINK_MSG_ROSUAV_BARO_RAW_FIELD_ATTITUDE_LEN 2
#define TTALINK_MSG_ROSUAV_BARO_RAW_FIELD_RESERVE_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_ROSUAV_BARO_RAW { \
    8194, \
    "ROSUAV_BARO_RAW", \
    3, \
    {  { "pressure", NULL, TTALINK_TYPE_FLOAT, 2, 0, offsetof(ttalink_rosuav_baro_raw_t, pressure) }, \
         { "attitude", NULL, TTALINK_TYPE_FLOAT, 2, 8, offsetof(ttalink_rosuav_baro_raw_t, attitude) }, \
         { "reserve", NULL, TTALINK_TYPE_FLOAT, 4, 16, offsetof(ttalink_rosuav_baro_raw_t, reserve) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_ROSUAV_BARO_RAW { \
    "ROSUAV_BARO_RAW", \
    3, \
    {  { "pressure", NULL, TTALINK_TYPE_FLOAT, 2, 0, offsetof(ttalink_rosuav_baro_raw_t, pressure) }, \
         { "attitude", NULL, TTALINK_TYPE_FLOAT, 2, 8, offsetof(ttalink_rosuav_baro_raw_t, attitude) }, \
         { "reserve", NULL, TTALINK_TYPE_FLOAT, 4, 16, offsetof(ttalink_rosuav_baro_raw_t, reserve) }, \
         } \
}
#endif


static inline uint16_t _ttalink_rosuav_baro_raw_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *pressure, const float *attitude, const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN];

    _tta_put_float_array(buf, 0, pressure, 2);
    _tta_put_float_array(buf, 8, attitude, 2);
    _tta_put_float_array(buf, 16, reserve, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN);
#else
    ttalink_rosuav_baro_raw_t packet;

    tta_array_memcpy(packet.pressure, pressure, sizeof(float)*2);
    tta_array_memcpy(packet.attitude, attitude, sizeof(float)*2);
    tta_array_memcpy(packet.reserve, reserve, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ROSUAV_BARO_RAW;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_ROSUAV_BARO_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN, TTALINK_MSG_ID_ROSUAV_BARO_RAW_CRC, nocrc);
}

/**
 * @brief Pack a rosuav_baro_raw message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param pressure  .
 * @param attitude  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_baro_raw_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *pressure, const float *attitude, const float *reserve)
{
    return _ttalink_rosuav_baro_raw_pack(dst_addr, src_addr, msg,  pressure, attitude, reserve, false);
}

/**
 * @brief Pack a rosuav_baro_raw message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param pressure  .
 * @param attitude  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_baro_raw_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *pressure, const float *attitude, const float *reserve)
{
    return _ttalink_rosuav_baro_raw_pack(dst_addr, src_addr, msg,  pressure, attitude, reserve, true);
}


static inline uint16_t _ttalink_rosuav_baro_raw_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *pressure,const float *attitude,const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN];

    _tta_put_float_array(buf, 0, pressure, 2);
    _tta_put_float_array(buf, 8, attitude, 2);
    _tta_put_float_array(buf, 16, reserve, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN);
#else
    ttalink_rosuav_baro_raw_t packet;

    tta_array_memcpy(packet.pressure, pressure, sizeof(float)*2);
    tta_array_memcpy(packet.attitude, attitude, sizeof(float)*2);
    tta_array_memcpy(packet.reserve, reserve, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ROSUAV_BARO_RAW;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_BARO_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN, TTALINK_MSG_ID_ROSUAV_BARO_RAW_CRC, nocrc);
}

/**
 * @brief Pack a rosuav_baro_raw message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pressure  .
 * @param attitude  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_baro_raw_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *pressure,const float *attitude,const float *reserve)
{
    return _ttalink_rosuav_baro_raw_pack_chan(dst_addr, src_addr, chan, msg,  pressure, attitude, reserve, false);
}

/**
 * @brief Pack a rosuav_baro_raw message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pressure  .
 * @param attitude  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_baro_raw_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *pressure,const float *attitude,const float *reserve)
{
    return _ttalink_rosuav_baro_raw_pack_chan(dst_addr, src_addr, chan, msg,  pressure, attitude, reserve, true);
}


static inline uint16_t _ttalink_rosuav_baro_raw_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_baro_raw_t* rosuav_baro_raw, bool nocrc)
{
    if(nocrc){
        return ttalink_rosuav_baro_raw_pack_nocrc(dst_addr, src_addr, msg, rosuav_baro_raw->pressure, rosuav_baro_raw->attitude, rosuav_baro_raw->reserve);
    }else{
        return ttalink_rosuav_baro_raw_pack(dst_addr, src_addr, msg, rosuav_baro_raw->pressure, rosuav_baro_raw->attitude, rosuav_baro_raw->reserve);
    }
    
}

/**
 * @brief Encode a rosuav_baro_raw struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_baro_raw C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_baro_raw_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_baro_raw_t* rosuav_baro_raw)
{
    return _ttalink_rosuav_baro_raw_encode(dst_addr, src_addr, msg, rosuav_baro_raw, false);
}

/**
 * @brief Encode a rosuav_baro_raw struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_baro_raw C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_baro_raw_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_baro_raw_t* rosuav_baro_raw)
{
    return _ttalink_rosuav_baro_raw_encode(dst_addr, src_addr, msg, rosuav_baro_raw, true);
}


static inline uint16_t _ttalink_rosuav_baro_raw_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_baro_raw_t* rosuav_baro_raw, bool nocrc)
{
    if(nocrc){
        return ttalink_rosuav_baro_raw_pack_chan_nocrc(dst_addr, src_addr, chan, msg, rosuav_baro_raw->pressure, rosuav_baro_raw->attitude, rosuav_baro_raw->reserve);
    }else{
        return ttalink_rosuav_baro_raw_pack_chan(dst_addr, src_addr, chan, msg, rosuav_baro_raw->pressure, rosuav_baro_raw->attitude, rosuav_baro_raw->reserve);
    }
}

/**
 * @brief Encode a rosuav_baro_raw struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_baro_raw C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_baro_raw_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_baro_raw_t* rosuav_baro_raw)
{
    return _ttalink_rosuav_baro_raw_encode_chan(dst_addr, src_addr, chan, msg, rosuav_baro_raw, false);
}

/**
 * @brief Encode a rosuav_baro_raw struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_baro_raw C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_baro_raw_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_baro_raw_t* rosuav_baro_raw)
{
    return _ttalink_rosuav_baro_raw_encode_chan(dst_addr, src_addr, chan, msg, rosuav_baro_raw, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_rosuav_baro_raw_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *pressure, const float *attitude, const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN];

    _tta_put_float_array(buf, 0, pressure, 2);
    _tta_put_float_array(buf, 8, attitude, 2);
    _tta_put_float_array(buf, 16, reserve, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_BARO_RAW, buf, TTALINK_MSG_ID_ROSUAV_BARO_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN, TTALINK_MSG_ID_ROSUAV_BARO_RAW_CRC, nocrc);
#else
    ttalink_rosuav_baro_raw_t packet;

    tta_array_memcpy(packet.pressure, pressure, sizeof(float)*2);
    tta_array_memcpy(packet.attitude, attitude, sizeof(float)*2);
    tta_array_memcpy(packet.reserve, reserve, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_BARO_RAW, (const char *)&packet, TTALINK_MSG_ID_ROSUAV_BARO_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN, TTALINK_MSG_ID_ROSUAV_BARO_RAW_CRC, nocrc);
#endif
}

/**
 * @brief Send a rosuav_baro_raw message
 * @param chan TTAlink channel to send the message
 *
 * @param pressure  .
 * @param attitude  .
 * @param reserve  .
 */
static inline void ttalink_rosuav_baro_raw_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *pressure, const float *attitude, const float *reserve)
{
    _ttalink_rosuav_baro_raw_send(dst_addr, src_addr, chan, pressure, attitude, reserve, false);
}

/**
 * @brief Send a rosuav_baro_raw message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param pressure  .
 * @param attitude  .
 * @param reserve  .
 */
static inline void ttalink_rosuav_baro_raw_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *pressure, const float *attitude, const float *reserve)
{
    _ttalink_rosuav_baro_raw_send(dst_addr, src_addr, chan, pressure, attitude, reserve, true);
}


static inline void _ttalink_rosuav_baro_raw_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_baro_raw_t* rosuav_baro_raw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_rosuav_baro_raw_send_nocrc(dst_addr, src_addr, chan, rosuav_baro_raw->pressure, rosuav_baro_raw->attitude, rosuav_baro_raw->reserve);
    }else{
        ttalink_rosuav_baro_raw_send(dst_addr, src_addr, chan, rosuav_baro_raw->pressure, rosuav_baro_raw->attitude, rosuav_baro_raw->reserve);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_BARO_RAW, (const char *)rosuav_baro_raw, TTALINK_MSG_ID_ROSUAV_BARO_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN, TTALINK_MSG_ID_ROSUAV_BARO_RAW_CRC, nocrc);
#endif
}

/**
 * @brief Send a rosuav_baro_raw message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rosuav_baro_raw_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_baro_raw_t* rosuav_baro_raw)
{
    _ttalink_rosuav_baro_raw_send_struct(dst_addr, src_addr, chan, rosuav_baro_raw, false);
}

/**
 * @brief Send a rosuav_baro_raw message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rosuav_baro_raw_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_baro_raw_t* rosuav_baro_raw)
{
    _ttalink_rosuav_baro_raw_send_struct(dst_addr, src_addr, chan, rosuav_baro_raw, true);
}

#if TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_rosuav_baro_raw_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *pressure, const float *attitude, const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _tta_put_float_array(buf, 0, pressure, 2);
    _tta_put_float_array(buf, 8, attitude, 2);
    _tta_put_float_array(buf, 16, reserve, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_BARO_RAW, buf, TTALINK_MSG_ID_ROSUAV_BARO_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN, TTALINK_MSG_ID_ROSUAV_BARO_RAW_CRC, nocrc);
#else
    ttalink_rosuav_baro_raw_t *packet = (ttalink_rosuav_baro_raw_t *)msgbuf;

    tta_array_memcpy(packet->pressure, pressure, sizeof(float)*2);
    tta_array_memcpy(packet->attitude, attitude, sizeof(float)*2);
    tta_array_memcpy(packet->reserve, reserve, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_BARO_RAW, (const char *)packet, TTALINK_MSG_ID_ROSUAV_BARO_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN, TTALINK_MSG_ID_ROSUAV_BARO_RAW_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_rosuav_baro_raw_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *pressure, const float *attitude, const float *reserve)
{
    _ttalink_rosuav_baro_raw_send_buf(dst_addr, src_addr, msgbuf, chan, pressure, attitude, reserve, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_rosuav_baro_raw_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *pressure, const float *attitude, const float *reserve)
{
    _ttalink_rosuav_baro_raw_send_buf(dst_addr, src_addr, msgbuf, chan, pressure, attitude, reserve, true);
}
#endif

#endif

// MESSAGE ROSUAV_BARO_RAW UNPACKING


/**
 * @brief Get field pressure from rosuav_baro_raw message
 *
 * @return  .
 */
static inline uint16_t ttalink_rosuav_baro_raw_get_pressure(const ttalink_message_t* msg, float *pressure)
{
    return _TTA_RETURN_float_array(msg, pressure, 2,  0);
}

/**
 * @brief Get field attitude from rosuav_baro_raw message
 *
 * @return  .
 */
static inline uint16_t ttalink_rosuav_baro_raw_get_attitude(const ttalink_message_t* msg, float *attitude)
{
    return _TTA_RETURN_float_array(msg, attitude, 2,  8);
}

/**
 * @brief Get field reserve from rosuav_baro_raw message
 *
 * @return  .
 */
static inline uint16_t ttalink_rosuav_baro_raw_get_reserve(const ttalink_message_t* msg, float *reserve)
{
    return _TTA_RETURN_float_array(msg, reserve, 4,  16);
}

/**
 * @brief Decode a rosuav_baro_raw message into a struct
 *
 * @param msg The message to decode
 * @param rosuav_baro_raw C-struct to decode the message contents into
 */
static inline void ttalink_rosuav_baro_raw_decode(const ttalink_message_t* msg, ttalink_rosuav_baro_raw_t* rosuav_baro_raw)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_rosuav_baro_raw_get_pressure(msg, rosuav_baro_raw->pressure);
    ttalink_rosuav_baro_raw_get_attitude(msg, rosuav_baro_raw->attitude);
    ttalink_rosuav_baro_raw_get_reserve(msg, rosuav_baro_raw->reserve);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN? msg->len : TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN;
        memset(rosuav_baro_raw, 0, TTALINK_MSG_ID_ROSUAV_BARO_RAW_LEN);
    memcpy(rosuav_baro_raw, _TTA_PAYLOAD(msg), len);
#endif
}
