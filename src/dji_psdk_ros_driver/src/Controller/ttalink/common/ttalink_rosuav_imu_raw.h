#pragma once
// MESSAGE ROSUAV_IMU_RAW PACKING

#define TTALINK_MSG_ID_ROSUAV_IMU_RAW 8193

TTAPACKED(
typedef struct __ttalink_rosuav_imu_raw_t {
 float acc[3]; /*<  .*/
 float gyro[3]; /*<  .*/
 float reserve[4]; /*<  .*/
}) ttalink_rosuav_imu_raw_t;

#define TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN 40
#define TTALINK_MSG_ID_ROSUAV_IMU_RAW_MIN_LEN 40
#define TTALINK_MSG_ID_8193_LEN 40
#define TTALINK_MSG_ID_8193_MIN_LEN 40

#define TTALINK_MSG_ID_ROSUAV_IMU_RAW_CRC 182
#define TTALINK_MSG_ID_8193_CRC 182

#define TTALINK_MSG_ROSUAV_IMU_RAW_FIELD_ACC_LEN 3
#define TTALINK_MSG_ROSUAV_IMU_RAW_FIELD_GYRO_LEN 3
#define TTALINK_MSG_ROSUAV_IMU_RAW_FIELD_RESERVE_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_ROSUAV_IMU_RAW { \
    8193, \
    "ROSUAV_IMU_RAW", \
    3, \
    {  { "acc", NULL, TTALINK_TYPE_FLOAT, 3, 0, offsetof(ttalink_rosuav_imu_raw_t, acc) }, \
         { "gyro", NULL, TTALINK_TYPE_FLOAT, 3, 12, offsetof(ttalink_rosuav_imu_raw_t, gyro) }, \
         { "reserve", NULL, TTALINK_TYPE_FLOAT, 4, 24, offsetof(ttalink_rosuav_imu_raw_t, reserve) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_ROSUAV_IMU_RAW { \
    "ROSUAV_IMU_RAW", \
    3, \
    {  { "acc", NULL, TTALINK_TYPE_FLOAT, 3, 0, offsetof(ttalink_rosuav_imu_raw_t, acc) }, \
         { "gyro", NULL, TTALINK_TYPE_FLOAT, 3, 12, offsetof(ttalink_rosuav_imu_raw_t, gyro) }, \
         { "reserve", NULL, TTALINK_TYPE_FLOAT, 4, 24, offsetof(ttalink_rosuav_imu_raw_t, reserve) }, \
         } \
}
#endif


static inline uint16_t _ttalink_rosuav_imu_raw_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *acc, const float *gyro, const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN];

    _tta_put_float_array(buf, 0, acc, 3);
    _tta_put_float_array(buf, 12, gyro, 3);
    _tta_put_float_array(buf, 24, reserve, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN);
#else
    ttalink_rosuav_imu_raw_t packet;

    tta_array_memcpy(packet.acc, acc, sizeof(float)*3);
    tta_array_memcpy(packet.gyro, gyro, sizeof(float)*3);
    tta_array_memcpy(packet.reserve, reserve, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ROSUAV_IMU_RAW;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_ROSUAV_IMU_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN, TTALINK_MSG_ID_ROSUAV_IMU_RAW_CRC, nocrc);
}

/**
 * @brief Pack a rosuav_imu_raw message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param acc  .
 * @param gyro  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_imu_raw_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *acc, const float *gyro, const float *reserve)
{
    return _ttalink_rosuav_imu_raw_pack(dst_addr, src_addr, msg,  acc, gyro, reserve, false);
}

/**
 * @brief Pack a rosuav_imu_raw message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param acc  .
 * @param gyro  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_imu_raw_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *acc, const float *gyro, const float *reserve)
{
    return _ttalink_rosuav_imu_raw_pack(dst_addr, src_addr, msg,  acc, gyro, reserve, true);
}


static inline uint16_t _ttalink_rosuav_imu_raw_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *acc,const float *gyro,const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN];

    _tta_put_float_array(buf, 0, acc, 3);
    _tta_put_float_array(buf, 12, gyro, 3);
    _tta_put_float_array(buf, 24, reserve, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN);
#else
    ttalink_rosuav_imu_raw_t packet;

    tta_array_memcpy(packet.acc, acc, sizeof(float)*3);
    tta_array_memcpy(packet.gyro, gyro, sizeof(float)*3);
    tta_array_memcpy(packet.reserve, reserve, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ROSUAV_IMU_RAW;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_IMU_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN, TTALINK_MSG_ID_ROSUAV_IMU_RAW_CRC, nocrc);
}

/**
 * @brief Pack a rosuav_imu_raw message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param acc  .
 * @param gyro  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_imu_raw_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *acc,const float *gyro,const float *reserve)
{
    return _ttalink_rosuav_imu_raw_pack_chan(dst_addr, src_addr, chan, msg,  acc, gyro, reserve, false);
}

/**
 * @brief Pack a rosuav_imu_raw message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param acc  .
 * @param gyro  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_imu_raw_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *acc,const float *gyro,const float *reserve)
{
    return _ttalink_rosuav_imu_raw_pack_chan(dst_addr, src_addr, chan, msg,  acc, gyro, reserve, true);
}


static inline uint16_t _ttalink_rosuav_imu_raw_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_imu_raw_t* rosuav_imu_raw, bool nocrc)
{
    if(nocrc){
        return ttalink_rosuav_imu_raw_pack_nocrc(dst_addr, src_addr, msg, rosuav_imu_raw->acc, rosuav_imu_raw->gyro, rosuav_imu_raw->reserve);
    }else{
        return ttalink_rosuav_imu_raw_pack(dst_addr, src_addr, msg, rosuav_imu_raw->acc, rosuav_imu_raw->gyro, rosuav_imu_raw->reserve);
    }
    
}

/**
 * @brief Encode a rosuav_imu_raw struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_imu_raw C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_imu_raw_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_imu_raw_t* rosuav_imu_raw)
{
    return _ttalink_rosuav_imu_raw_encode(dst_addr, src_addr, msg, rosuav_imu_raw, false);
}

/**
 * @brief Encode a rosuav_imu_raw struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_imu_raw C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_imu_raw_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_imu_raw_t* rosuav_imu_raw)
{
    return _ttalink_rosuav_imu_raw_encode(dst_addr, src_addr, msg, rosuav_imu_raw, true);
}


static inline uint16_t _ttalink_rosuav_imu_raw_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_imu_raw_t* rosuav_imu_raw, bool nocrc)
{
    if(nocrc){
        return ttalink_rosuav_imu_raw_pack_chan_nocrc(dst_addr, src_addr, chan, msg, rosuav_imu_raw->acc, rosuav_imu_raw->gyro, rosuav_imu_raw->reserve);
    }else{
        return ttalink_rosuav_imu_raw_pack_chan(dst_addr, src_addr, chan, msg, rosuav_imu_raw->acc, rosuav_imu_raw->gyro, rosuav_imu_raw->reserve);
    }
}

/**
 * @brief Encode a rosuav_imu_raw struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_imu_raw C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_imu_raw_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_imu_raw_t* rosuav_imu_raw)
{
    return _ttalink_rosuav_imu_raw_encode_chan(dst_addr, src_addr, chan, msg, rosuav_imu_raw, false);
}

/**
 * @brief Encode a rosuav_imu_raw struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_imu_raw C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_imu_raw_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_imu_raw_t* rosuav_imu_raw)
{
    return _ttalink_rosuav_imu_raw_encode_chan(dst_addr, src_addr, chan, msg, rosuav_imu_raw, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_rosuav_imu_raw_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *acc, const float *gyro, const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN];

    _tta_put_float_array(buf, 0, acc, 3);
    _tta_put_float_array(buf, 12, gyro, 3);
    _tta_put_float_array(buf, 24, reserve, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_IMU_RAW, buf, TTALINK_MSG_ID_ROSUAV_IMU_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN, TTALINK_MSG_ID_ROSUAV_IMU_RAW_CRC, nocrc);
#else
    ttalink_rosuav_imu_raw_t packet;

    tta_array_memcpy(packet.acc, acc, sizeof(float)*3);
    tta_array_memcpy(packet.gyro, gyro, sizeof(float)*3);
    tta_array_memcpy(packet.reserve, reserve, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_IMU_RAW, (const char *)&packet, TTALINK_MSG_ID_ROSUAV_IMU_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN, TTALINK_MSG_ID_ROSUAV_IMU_RAW_CRC, nocrc);
#endif
}

/**
 * @brief Send a rosuav_imu_raw message
 * @param chan TTAlink channel to send the message
 *
 * @param acc  .
 * @param gyro  .
 * @param reserve  .
 */
static inline void ttalink_rosuav_imu_raw_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *acc, const float *gyro, const float *reserve)
{
    _ttalink_rosuav_imu_raw_send(dst_addr, src_addr, chan, acc, gyro, reserve, false);
}

/**
 * @brief Send a rosuav_imu_raw message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param acc  .
 * @param gyro  .
 * @param reserve  .
 */
static inline void ttalink_rosuav_imu_raw_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *acc, const float *gyro, const float *reserve)
{
    _ttalink_rosuav_imu_raw_send(dst_addr, src_addr, chan, acc, gyro, reserve, true);
}


static inline void _ttalink_rosuav_imu_raw_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_imu_raw_t* rosuav_imu_raw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_rosuav_imu_raw_send_nocrc(dst_addr, src_addr, chan, rosuav_imu_raw->acc, rosuav_imu_raw->gyro, rosuav_imu_raw->reserve);
    }else{
        ttalink_rosuav_imu_raw_send(dst_addr, src_addr, chan, rosuav_imu_raw->acc, rosuav_imu_raw->gyro, rosuav_imu_raw->reserve);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_IMU_RAW, (const char *)rosuav_imu_raw, TTALINK_MSG_ID_ROSUAV_IMU_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN, TTALINK_MSG_ID_ROSUAV_IMU_RAW_CRC, nocrc);
#endif
}

/**
 * @brief Send a rosuav_imu_raw message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rosuav_imu_raw_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_imu_raw_t* rosuav_imu_raw)
{
    _ttalink_rosuav_imu_raw_send_struct(dst_addr, src_addr, chan, rosuav_imu_raw, false);
}

/**
 * @brief Send a rosuav_imu_raw message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rosuav_imu_raw_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_imu_raw_t* rosuav_imu_raw)
{
    _ttalink_rosuav_imu_raw_send_struct(dst_addr, src_addr, chan, rosuav_imu_raw, true);
}

#if TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_rosuav_imu_raw_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *acc, const float *gyro, const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _tta_put_float_array(buf, 0, acc, 3);
    _tta_put_float_array(buf, 12, gyro, 3);
    _tta_put_float_array(buf, 24, reserve, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_IMU_RAW, buf, TTALINK_MSG_ID_ROSUAV_IMU_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN, TTALINK_MSG_ID_ROSUAV_IMU_RAW_CRC, nocrc);
#else
    ttalink_rosuav_imu_raw_t *packet = (ttalink_rosuav_imu_raw_t *)msgbuf;

    tta_array_memcpy(packet->acc, acc, sizeof(float)*3);
    tta_array_memcpy(packet->gyro, gyro, sizeof(float)*3);
    tta_array_memcpy(packet->reserve, reserve, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_IMU_RAW, (const char *)packet, TTALINK_MSG_ID_ROSUAV_IMU_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN, TTALINK_MSG_ID_ROSUAV_IMU_RAW_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_rosuav_imu_raw_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *acc, const float *gyro, const float *reserve)
{
    _ttalink_rosuav_imu_raw_send_buf(dst_addr, src_addr, msgbuf, chan, acc, gyro, reserve, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_rosuav_imu_raw_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *acc, const float *gyro, const float *reserve)
{
    _ttalink_rosuav_imu_raw_send_buf(dst_addr, src_addr, msgbuf, chan, acc, gyro, reserve, true);
}
#endif

#endif

// MESSAGE ROSUAV_IMU_RAW UNPACKING


/**
 * @brief Get field acc from rosuav_imu_raw message
 *
 * @return  .
 */
static inline uint16_t ttalink_rosuav_imu_raw_get_acc(const ttalink_message_t* msg, float *acc)
{
    return _TTA_RETURN_float_array(msg, acc, 3,  0);
}

/**
 * @brief Get field gyro from rosuav_imu_raw message
 *
 * @return  .
 */
static inline uint16_t ttalink_rosuav_imu_raw_get_gyro(const ttalink_message_t* msg, float *gyro)
{
    return _TTA_RETURN_float_array(msg, gyro, 3,  12);
}

/**
 * @brief Get field reserve from rosuav_imu_raw message
 *
 * @return  .
 */
static inline uint16_t ttalink_rosuav_imu_raw_get_reserve(const ttalink_message_t* msg, float *reserve)
{
    return _TTA_RETURN_float_array(msg, reserve, 4,  24);
}

/**
 * @brief Decode a rosuav_imu_raw message into a struct
 *
 * @param msg The message to decode
 * @param rosuav_imu_raw C-struct to decode the message contents into
 */
static inline void ttalink_rosuav_imu_raw_decode(const ttalink_message_t* msg, ttalink_rosuav_imu_raw_t* rosuav_imu_raw)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_rosuav_imu_raw_get_acc(msg, rosuav_imu_raw->acc);
    ttalink_rosuav_imu_raw_get_gyro(msg, rosuav_imu_raw->gyro);
    ttalink_rosuav_imu_raw_get_reserve(msg, rosuav_imu_raw->reserve);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN? msg->len : TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN;
        memset(rosuav_imu_raw, 0, TTALINK_MSG_ID_ROSUAV_IMU_RAW_LEN);
    memcpy(rosuav_imu_raw, _TTA_PAYLOAD(msg), len);
#endif
}
