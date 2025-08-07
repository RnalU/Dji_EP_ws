#pragma once
// MESSAGE ROSUAV_P20_ATTI PACKING

#define TTALINK_MSG_ID_ROSUAV_P20_ATTI 8195

TTAPACKED(
typedef struct __ttalink_rosuav_p20_atti_t {
 float acc[3]; /*<  .*/
 float gyro[3]; /*<  .*/
 float pitch; /*<  .*/
 float roll; /*<  .*/
 float yaw; /*<  .*/
 float reserve[4]; /*<  .*/
}) ttalink_rosuav_p20_atti_t;

#define TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN 52
#define TTALINK_MSG_ID_ROSUAV_P20_ATTI_MIN_LEN 52
#define TTALINK_MSG_ID_8195_LEN 52
#define TTALINK_MSG_ID_8195_MIN_LEN 52

#define TTALINK_MSG_ID_ROSUAV_P20_ATTI_CRC 181
#define TTALINK_MSG_ID_8195_CRC 181

#define TTALINK_MSG_ROSUAV_P20_ATTI_FIELD_ACC_LEN 3
#define TTALINK_MSG_ROSUAV_P20_ATTI_FIELD_GYRO_LEN 3
#define TTALINK_MSG_ROSUAV_P20_ATTI_FIELD_RESERVE_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_ROSUAV_P20_ATTI { \
    8195, \
    "ROSUAV_P20_ATTI", \
    6, \
    {  { "acc", NULL, TTALINK_TYPE_FLOAT, 3, 0, offsetof(ttalink_rosuav_p20_atti_t, acc) }, \
         { "gyro", NULL, TTALINK_TYPE_FLOAT, 3, 12, offsetof(ttalink_rosuav_p20_atti_t, gyro) }, \
         { "pitch", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_rosuav_p20_atti_t, pitch) }, \
         { "roll", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_rosuav_p20_atti_t, roll) }, \
         { "yaw", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_rosuav_p20_atti_t, yaw) }, \
         { "reserve", NULL, TTALINK_TYPE_FLOAT, 4, 36, offsetof(ttalink_rosuav_p20_atti_t, reserve) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_ROSUAV_P20_ATTI { \
    "ROSUAV_P20_ATTI", \
    6, \
    {  { "acc", NULL, TTALINK_TYPE_FLOAT, 3, 0, offsetof(ttalink_rosuav_p20_atti_t, acc) }, \
         { "gyro", NULL, TTALINK_TYPE_FLOAT, 3, 12, offsetof(ttalink_rosuav_p20_atti_t, gyro) }, \
         { "pitch", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_rosuav_p20_atti_t, pitch) }, \
         { "roll", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_rosuav_p20_atti_t, roll) }, \
         { "yaw", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_rosuav_p20_atti_t, yaw) }, \
         { "reserve", NULL, TTALINK_TYPE_FLOAT, 4, 36, offsetof(ttalink_rosuav_p20_atti_t, reserve) }, \
         } \
}
#endif


static inline uint16_t _ttalink_rosuav_p20_atti_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *acc, const float *gyro, float pitch, float roll, float yaw, const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN];
    _tta_put_float(buf, 24, pitch);
    _tta_put_float(buf, 28, roll);
    _tta_put_float(buf, 32, yaw);
    _tta_put_float_array(buf, 0, acc, 3);
    _tta_put_float_array(buf, 12, gyro, 3);
    _tta_put_float_array(buf, 36, reserve, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN);
#else
    ttalink_rosuav_p20_atti_t packet;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.yaw = yaw;
    tta_array_memcpy(packet.acc, acc, sizeof(float)*3);
    tta_array_memcpy(packet.gyro, gyro, sizeof(float)*3);
    tta_array_memcpy(packet.reserve, reserve, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ROSUAV_P20_ATTI;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_ROSUAV_P20_ATTI_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN, TTALINK_MSG_ID_ROSUAV_P20_ATTI_CRC, nocrc);
}

/**
 * @brief Pack a rosuav_p20_atti message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param acc  .
 * @param gyro  .
 * @param pitch  .
 * @param roll  .
 * @param yaw  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_p20_atti_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *acc, const float *gyro, float pitch, float roll, float yaw, const float *reserve)
{
    return _ttalink_rosuav_p20_atti_pack(dst_addr, src_addr, msg,  acc, gyro, pitch, roll, yaw, reserve, false);
}

/**
 * @brief Pack a rosuav_p20_atti message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param acc  .
 * @param gyro  .
 * @param pitch  .
 * @param roll  .
 * @param yaw  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_p20_atti_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *acc, const float *gyro, float pitch, float roll, float yaw, const float *reserve)
{
    return _ttalink_rosuav_p20_atti_pack(dst_addr, src_addr, msg,  acc, gyro, pitch, roll, yaw, reserve, true);
}


static inline uint16_t _ttalink_rosuav_p20_atti_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *acc,const float *gyro,float pitch,float roll,float yaw,const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN];
    _tta_put_float(buf, 24, pitch);
    _tta_put_float(buf, 28, roll);
    _tta_put_float(buf, 32, yaw);
    _tta_put_float_array(buf, 0, acc, 3);
    _tta_put_float_array(buf, 12, gyro, 3);
    _tta_put_float_array(buf, 36, reserve, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN);
#else
    ttalink_rosuav_p20_atti_t packet;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.yaw = yaw;
    tta_array_memcpy(packet.acc, acc, sizeof(float)*3);
    tta_array_memcpy(packet.gyro, gyro, sizeof(float)*3);
    tta_array_memcpy(packet.reserve, reserve, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ROSUAV_P20_ATTI;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_P20_ATTI_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN, TTALINK_MSG_ID_ROSUAV_P20_ATTI_CRC, nocrc);
}

/**
 * @brief Pack a rosuav_p20_atti message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param acc  .
 * @param gyro  .
 * @param pitch  .
 * @param roll  .
 * @param yaw  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_p20_atti_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *acc,const float *gyro,float pitch,float roll,float yaw,const float *reserve)
{
    return _ttalink_rosuav_p20_atti_pack_chan(dst_addr, src_addr, chan, msg,  acc, gyro, pitch, roll, yaw, reserve, false);
}

/**
 * @brief Pack a rosuav_p20_atti message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param acc  .
 * @param gyro  .
 * @param pitch  .
 * @param roll  .
 * @param yaw  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_p20_atti_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *acc,const float *gyro,float pitch,float roll,float yaw,const float *reserve)
{
    return _ttalink_rosuav_p20_atti_pack_chan(dst_addr, src_addr, chan, msg,  acc, gyro, pitch, roll, yaw, reserve, true);
}


static inline uint16_t _ttalink_rosuav_p20_atti_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_p20_atti_t* rosuav_p20_atti, bool nocrc)
{
    if(nocrc){
        return ttalink_rosuav_p20_atti_pack_nocrc(dst_addr, src_addr, msg, rosuav_p20_atti->acc, rosuav_p20_atti->gyro, rosuav_p20_atti->pitch, rosuav_p20_atti->roll, rosuav_p20_atti->yaw, rosuav_p20_atti->reserve);
    }else{
        return ttalink_rosuav_p20_atti_pack(dst_addr, src_addr, msg, rosuav_p20_atti->acc, rosuav_p20_atti->gyro, rosuav_p20_atti->pitch, rosuav_p20_atti->roll, rosuav_p20_atti->yaw, rosuav_p20_atti->reserve);
    }
    
}

/**
 * @brief Encode a rosuav_p20_atti struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_p20_atti C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_p20_atti_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_p20_atti_t* rosuav_p20_atti)
{
    return _ttalink_rosuav_p20_atti_encode(dst_addr, src_addr, msg, rosuav_p20_atti, false);
}

/**
 * @brief Encode a rosuav_p20_atti struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_p20_atti C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_p20_atti_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_p20_atti_t* rosuav_p20_atti)
{
    return _ttalink_rosuav_p20_atti_encode(dst_addr, src_addr, msg, rosuav_p20_atti, true);
}


static inline uint16_t _ttalink_rosuav_p20_atti_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_p20_atti_t* rosuav_p20_atti, bool nocrc)
{
    if(nocrc){
        return ttalink_rosuav_p20_atti_pack_chan_nocrc(dst_addr, src_addr, chan, msg, rosuav_p20_atti->acc, rosuav_p20_atti->gyro, rosuav_p20_atti->pitch, rosuav_p20_atti->roll, rosuav_p20_atti->yaw, rosuav_p20_atti->reserve);
    }else{
        return ttalink_rosuav_p20_atti_pack_chan(dst_addr, src_addr, chan, msg, rosuav_p20_atti->acc, rosuav_p20_atti->gyro, rosuav_p20_atti->pitch, rosuav_p20_atti->roll, rosuav_p20_atti->yaw, rosuav_p20_atti->reserve);
    }
}

/**
 * @brief Encode a rosuav_p20_atti struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_p20_atti C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_p20_atti_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_p20_atti_t* rosuav_p20_atti)
{
    return _ttalink_rosuav_p20_atti_encode_chan(dst_addr, src_addr, chan, msg, rosuav_p20_atti, false);
}

/**
 * @brief Encode a rosuav_p20_atti struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_p20_atti C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_p20_atti_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_p20_atti_t* rosuav_p20_atti)
{
    return _ttalink_rosuav_p20_atti_encode_chan(dst_addr, src_addr, chan, msg, rosuav_p20_atti, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_rosuav_p20_atti_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *acc, const float *gyro, float pitch, float roll, float yaw, const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN];
    _tta_put_float(buf, 24, pitch);
    _tta_put_float(buf, 28, roll);
    _tta_put_float(buf, 32, yaw);
    _tta_put_float_array(buf, 0, acc, 3);
    _tta_put_float_array(buf, 12, gyro, 3);
    _tta_put_float_array(buf, 36, reserve, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_P20_ATTI, buf, TTALINK_MSG_ID_ROSUAV_P20_ATTI_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN, TTALINK_MSG_ID_ROSUAV_P20_ATTI_CRC, nocrc);
#else
    ttalink_rosuav_p20_atti_t packet;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.yaw = yaw;
    tta_array_memcpy(packet.acc, acc, sizeof(float)*3);
    tta_array_memcpy(packet.gyro, gyro, sizeof(float)*3);
    tta_array_memcpy(packet.reserve, reserve, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_P20_ATTI, (const char *)&packet, TTALINK_MSG_ID_ROSUAV_P20_ATTI_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN, TTALINK_MSG_ID_ROSUAV_P20_ATTI_CRC, nocrc);
#endif
}

/**
 * @brief Send a rosuav_p20_atti message
 * @param chan TTAlink channel to send the message
 *
 * @param acc  .
 * @param gyro  .
 * @param pitch  .
 * @param roll  .
 * @param yaw  .
 * @param reserve  .
 */
static inline void ttalink_rosuav_p20_atti_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *acc, const float *gyro, float pitch, float roll, float yaw, const float *reserve)
{
    _ttalink_rosuav_p20_atti_send(dst_addr, src_addr, chan, acc, gyro, pitch, roll, yaw, reserve, false);
}

/**
 * @brief Send a rosuav_p20_atti message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param acc  .
 * @param gyro  .
 * @param pitch  .
 * @param roll  .
 * @param yaw  .
 * @param reserve  .
 */
static inline void ttalink_rosuav_p20_atti_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *acc, const float *gyro, float pitch, float roll, float yaw, const float *reserve)
{
    _ttalink_rosuav_p20_atti_send(dst_addr, src_addr, chan, acc, gyro, pitch, roll, yaw, reserve, true);
}


static inline void _ttalink_rosuav_p20_atti_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_p20_atti_t* rosuav_p20_atti, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_rosuav_p20_atti_send_nocrc(dst_addr, src_addr, chan, rosuav_p20_atti->acc, rosuav_p20_atti->gyro, rosuav_p20_atti->pitch, rosuav_p20_atti->roll, rosuav_p20_atti->yaw, rosuav_p20_atti->reserve);
    }else{
        ttalink_rosuav_p20_atti_send(dst_addr, src_addr, chan, rosuav_p20_atti->acc, rosuav_p20_atti->gyro, rosuav_p20_atti->pitch, rosuav_p20_atti->roll, rosuav_p20_atti->yaw, rosuav_p20_atti->reserve);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_P20_ATTI, (const char *)rosuav_p20_atti, TTALINK_MSG_ID_ROSUAV_P20_ATTI_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN, TTALINK_MSG_ID_ROSUAV_P20_ATTI_CRC, nocrc);
#endif
}

/**
 * @brief Send a rosuav_p20_atti message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rosuav_p20_atti_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_p20_atti_t* rosuav_p20_atti)
{
    _ttalink_rosuav_p20_atti_send_struct(dst_addr, src_addr, chan, rosuav_p20_atti, false);
}

/**
 * @brief Send a rosuav_p20_atti message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rosuav_p20_atti_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_p20_atti_t* rosuav_p20_atti)
{
    _ttalink_rosuav_p20_atti_send_struct(dst_addr, src_addr, chan, rosuav_p20_atti, true);
}

#if TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_rosuav_p20_atti_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *acc, const float *gyro, float pitch, float roll, float yaw, const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_float(buf, 24, pitch);
    _tta_put_float(buf, 28, roll);
    _tta_put_float(buf, 32, yaw);
    _tta_put_float_array(buf, 0, acc, 3);
    _tta_put_float_array(buf, 12, gyro, 3);
    _tta_put_float_array(buf, 36, reserve, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_P20_ATTI, buf, TTALINK_MSG_ID_ROSUAV_P20_ATTI_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN, TTALINK_MSG_ID_ROSUAV_P20_ATTI_CRC, nocrc);
#else
    ttalink_rosuav_p20_atti_t *packet = (ttalink_rosuav_p20_atti_t *)msgbuf;
    packet->pitch = pitch;
    packet->roll = roll;
    packet->yaw = yaw;
    tta_array_memcpy(packet->acc, acc, sizeof(float)*3);
    tta_array_memcpy(packet->gyro, gyro, sizeof(float)*3);
    tta_array_memcpy(packet->reserve, reserve, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_P20_ATTI, (const char *)packet, TTALINK_MSG_ID_ROSUAV_P20_ATTI_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN, TTALINK_MSG_ID_ROSUAV_P20_ATTI_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_rosuav_p20_atti_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *acc, const float *gyro, float pitch, float roll, float yaw, const float *reserve)
{
    _ttalink_rosuav_p20_atti_send_buf(dst_addr, src_addr, msgbuf, chan, acc, gyro, pitch, roll, yaw, reserve, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_rosuav_p20_atti_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *acc, const float *gyro, float pitch, float roll, float yaw, const float *reserve)
{
    _ttalink_rosuav_p20_atti_send_buf(dst_addr, src_addr, msgbuf, chan, acc, gyro, pitch, roll, yaw, reserve, true);
}
#endif

#endif

// MESSAGE ROSUAV_P20_ATTI UNPACKING


/**
 * @brief Get field acc from rosuav_p20_atti message
 *
 * @return  .
 */
static inline uint16_t ttalink_rosuav_p20_atti_get_acc(const ttalink_message_t* msg, float *acc)
{
    return _TTA_RETURN_float_array(msg, acc, 3,  0);
}

/**
 * @brief Get field gyro from rosuav_p20_atti message
 *
 * @return  .
 */
static inline uint16_t ttalink_rosuav_p20_atti_get_gyro(const ttalink_message_t* msg, float *gyro)
{
    return _TTA_RETURN_float_array(msg, gyro, 3,  12);
}

/**
 * @brief Get field pitch from rosuav_p20_atti message
 *
 * @return  .
 */
static inline float ttalink_rosuav_p20_atti_get_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Get field roll from rosuav_p20_atti message
 *
 * @return  .
 */
static inline float ttalink_rosuav_p20_atti_get_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  28);
}

/**
 * @brief Get field yaw from rosuav_p20_atti message
 *
 * @return  .
 */
static inline float ttalink_rosuav_p20_atti_get_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Get field reserve from rosuav_p20_atti message
 *
 * @return  .
 */
static inline uint16_t ttalink_rosuav_p20_atti_get_reserve(const ttalink_message_t* msg, float *reserve)
{
    return _TTA_RETURN_float_array(msg, reserve, 4,  36);
}

/**
 * @brief Decode a rosuav_p20_atti message into a struct
 *
 * @param msg The message to decode
 * @param rosuav_p20_atti C-struct to decode the message contents into
 */
static inline void ttalink_rosuav_p20_atti_decode(const ttalink_message_t* msg, ttalink_rosuav_p20_atti_t* rosuav_p20_atti)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_rosuav_p20_atti_get_acc(msg, rosuav_p20_atti->acc);
    ttalink_rosuav_p20_atti_get_gyro(msg, rosuav_p20_atti->gyro);
    rosuav_p20_atti->pitch = ttalink_rosuav_p20_atti_get_pitch(msg);
    rosuav_p20_atti->roll = ttalink_rosuav_p20_atti_get_roll(msg);
    rosuav_p20_atti->yaw = ttalink_rosuav_p20_atti_get_yaw(msg);
    ttalink_rosuav_p20_atti_get_reserve(msg, rosuav_p20_atti->reserve);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN? msg->len : TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN;
        memset(rosuav_p20_atti, 0, TTALINK_MSG_ID_ROSUAV_P20_ATTI_LEN);
    memcpy(rosuav_p20_atti, _TTA_PAYLOAD(msg), len);
#endif
}
