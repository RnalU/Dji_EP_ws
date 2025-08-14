#pragma once
// MESSAGE STREAM_MAG PACKING

#define TTALINK_MSG_ID_STREAM_MAG 2142

TTAPACKED(
typedef struct __ttalink_stream_mag_t {
 float mag1[3]; /*<  .*/
 float yaw1; /*<  .*/
 float mag_root1; /*<  .*/
 float mag2[3]; /*<  .*/
 float yaw2; /*<  .*/
 float mag_root2; /*<  .*/
}) ttalink_stream_mag_t;

#define TTALINK_MSG_ID_STREAM_MAG_LEN 40
#define TTALINK_MSG_ID_STREAM_MAG_MIN_LEN 40
#define TTALINK_MSG_ID_2142_LEN 40
#define TTALINK_MSG_ID_2142_MIN_LEN 40

#define TTALINK_MSG_ID_STREAM_MAG_CRC 39
#define TTALINK_MSG_ID_2142_CRC 39

#define TTALINK_MSG_STREAM_MAG_FIELD_MAG1_LEN 3
#define TTALINK_MSG_STREAM_MAG_FIELD_MAG2_LEN 3

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_STREAM_MAG { \
    2142, \
    "STREAM_MAG", \
    6, \
    {  { "mag1", NULL, TTALINK_TYPE_FLOAT, 3, 0, offsetof(ttalink_stream_mag_t, mag1) }, \
         { "yaw1", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_stream_mag_t, yaw1) }, \
         { "mag_root1", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_stream_mag_t, mag_root1) }, \
         { "mag2", NULL, TTALINK_TYPE_FLOAT, 3, 20, offsetof(ttalink_stream_mag_t, mag2) }, \
         { "yaw2", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_stream_mag_t, yaw2) }, \
         { "mag_root2", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_stream_mag_t, mag_root2) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_STREAM_MAG { \
    "STREAM_MAG", \
    6, \
    {  { "mag1", NULL, TTALINK_TYPE_FLOAT, 3, 0, offsetof(ttalink_stream_mag_t, mag1) }, \
         { "yaw1", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_stream_mag_t, yaw1) }, \
         { "mag_root1", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_stream_mag_t, mag_root1) }, \
         { "mag2", NULL, TTALINK_TYPE_FLOAT, 3, 20, offsetof(ttalink_stream_mag_t, mag2) }, \
         { "yaw2", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_stream_mag_t, yaw2) }, \
         { "mag_root2", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_stream_mag_t, mag_root2) }, \
         } \
}
#endif


static inline uint16_t _ttalink_stream_mag_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *mag1, float yaw1, float mag_root1, const float *mag2, float yaw2, float mag_root2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_STREAM_MAG_LEN];
    _tta_put_float(buf, 12, yaw1);
    _tta_put_float(buf, 16, mag_root1);
    _tta_put_float(buf, 32, yaw2);
    _tta_put_float(buf, 36, mag_root2);
    _tta_put_float_array(buf, 0, mag1, 3);
    _tta_put_float_array(buf, 20, mag2, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_STREAM_MAG_LEN);
#else
    ttalink_stream_mag_t packet;
    packet.yaw1 = yaw1;
    packet.mag_root1 = mag_root1;
    packet.yaw2 = yaw2;
    packet.mag_root2 = mag_root2;
    tta_array_memcpy(packet.mag1, mag1, sizeof(float)*3);
    tta_array_memcpy(packet.mag2, mag2, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_STREAM_MAG_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_STREAM_MAG;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_STREAM_MAG_MIN_LEN, TTALINK_MSG_ID_STREAM_MAG_LEN, TTALINK_MSG_ID_STREAM_MAG_CRC, nocrc);
}

/**
 * @brief Pack a stream_mag message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param mag1  .
 * @param yaw1  .
 * @param mag_root1  .
 * @param mag2  .
 * @param yaw2  .
 * @param mag_root2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_mag_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *mag1, float yaw1, float mag_root1, const float *mag2, float yaw2, float mag_root2)
{
    return _ttalink_stream_mag_pack(dst_addr, src_addr, msg,  mag1, yaw1, mag_root1, mag2, yaw2, mag_root2, false);
}

/**
 * @brief Pack a stream_mag message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param mag1  .
 * @param yaw1  .
 * @param mag_root1  .
 * @param mag2  .
 * @param yaw2  .
 * @param mag_root2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_mag_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *mag1, float yaw1, float mag_root1, const float *mag2, float yaw2, float mag_root2)
{
    return _ttalink_stream_mag_pack(dst_addr, src_addr, msg,  mag1, yaw1, mag_root1, mag2, yaw2, mag_root2, true);
}


static inline uint16_t _ttalink_stream_mag_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *mag1,float yaw1,float mag_root1,const float *mag2,float yaw2,float mag_root2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_STREAM_MAG_LEN];
    _tta_put_float(buf, 12, yaw1);
    _tta_put_float(buf, 16, mag_root1);
    _tta_put_float(buf, 32, yaw2);
    _tta_put_float(buf, 36, mag_root2);
    _tta_put_float_array(buf, 0, mag1, 3);
    _tta_put_float_array(buf, 20, mag2, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_STREAM_MAG_LEN);
#else
    ttalink_stream_mag_t packet;
    packet.yaw1 = yaw1;
    packet.mag_root1 = mag_root1;
    packet.yaw2 = yaw2;
    packet.mag_root2 = mag_root2;
    tta_array_memcpy(packet.mag1, mag1, sizeof(float)*3);
    tta_array_memcpy(packet.mag2, mag2, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_STREAM_MAG_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_STREAM_MAG;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_MAG_MIN_LEN, TTALINK_MSG_ID_STREAM_MAG_LEN, TTALINK_MSG_ID_STREAM_MAG_CRC, nocrc);
}

/**
 * @brief Pack a stream_mag message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mag1  .
 * @param yaw1  .
 * @param mag_root1  .
 * @param mag2  .
 * @param yaw2  .
 * @param mag_root2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_mag_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *mag1,float yaw1,float mag_root1,const float *mag2,float yaw2,float mag_root2)
{
    return _ttalink_stream_mag_pack_chan(dst_addr, src_addr, chan, msg,  mag1, yaw1, mag_root1, mag2, yaw2, mag_root2, false);
}

/**
 * @brief Pack a stream_mag message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mag1  .
 * @param yaw1  .
 * @param mag_root1  .
 * @param mag2  .
 * @param yaw2  .
 * @param mag_root2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_mag_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *mag1,float yaw1,float mag_root1,const float *mag2,float yaw2,float mag_root2)
{
    return _ttalink_stream_mag_pack_chan(dst_addr, src_addr, chan, msg,  mag1, yaw1, mag_root1, mag2, yaw2, mag_root2, true);
}


static inline uint16_t _ttalink_stream_mag_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_stream_mag_t* stream_mag, bool nocrc)
{
    if(nocrc){
        return ttalink_stream_mag_pack_nocrc(dst_addr, src_addr, msg, stream_mag->mag1, stream_mag->yaw1, stream_mag->mag_root1, stream_mag->mag2, stream_mag->yaw2, stream_mag->mag_root2);
    }else{
        return ttalink_stream_mag_pack(dst_addr, src_addr, msg, stream_mag->mag1, stream_mag->yaw1, stream_mag->mag_root1, stream_mag->mag2, stream_mag->yaw2, stream_mag->mag_root2);
    }
    
}

/**
 * @brief Encode a stream_mag struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param stream_mag C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_mag_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_stream_mag_t* stream_mag)
{
    return _ttalink_stream_mag_encode(dst_addr, src_addr, msg, stream_mag, false);
}

/**
 * @brief Encode a stream_mag struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param stream_mag C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_mag_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_stream_mag_t* stream_mag)
{
    return _ttalink_stream_mag_encode(dst_addr, src_addr, msg, stream_mag, true);
}


static inline uint16_t _ttalink_stream_mag_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_stream_mag_t* stream_mag, bool nocrc)
{
    if(nocrc){
        return ttalink_stream_mag_pack_chan_nocrc(dst_addr, src_addr, chan, msg, stream_mag->mag1, stream_mag->yaw1, stream_mag->mag_root1, stream_mag->mag2, stream_mag->yaw2, stream_mag->mag_root2);
    }else{
        return ttalink_stream_mag_pack_chan(dst_addr, src_addr, chan, msg, stream_mag->mag1, stream_mag->yaw1, stream_mag->mag_root1, stream_mag->mag2, stream_mag->yaw2, stream_mag->mag_root2);
    }
}

/**
 * @brief Encode a stream_mag struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param stream_mag C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_mag_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_stream_mag_t* stream_mag)
{
    return _ttalink_stream_mag_encode_chan(dst_addr, src_addr, chan, msg, stream_mag, false);
}

/**
 * @brief Encode a stream_mag struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param stream_mag C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_mag_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_stream_mag_t* stream_mag)
{
    return _ttalink_stream_mag_encode_chan(dst_addr, src_addr, chan, msg, stream_mag, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_stream_mag_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *mag1, float yaw1, float mag_root1, const float *mag2, float yaw2, float mag_root2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_STREAM_MAG_LEN];
    _tta_put_float(buf, 12, yaw1);
    _tta_put_float(buf, 16, mag_root1);
    _tta_put_float(buf, 32, yaw2);
    _tta_put_float(buf, 36, mag_root2);
    _tta_put_float_array(buf, 0, mag1, 3);
    _tta_put_float_array(buf, 20, mag2, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_MAG, buf, TTALINK_MSG_ID_STREAM_MAG_MIN_LEN, TTALINK_MSG_ID_STREAM_MAG_LEN, TTALINK_MSG_ID_STREAM_MAG_CRC, nocrc);
#else
    ttalink_stream_mag_t packet;
    packet.yaw1 = yaw1;
    packet.mag_root1 = mag_root1;
    packet.yaw2 = yaw2;
    packet.mag_root2 = mag_root2;
    tta_array_memcpy(packet.mag1, mag1, sizeof(float)*3);
    tta_array_memcpy(packet.mag2, mag2, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_MAG, (const char *)&packet, TTALINK_MSG_ID_STREAM_MAG_MIN_LEN, TTALINK_MSG_ID_STREAM_MAG_LEN, TTALINK_MSG_ID_STREAM_MAG_CRC, nocrc);
#endif
}

/**
 * @brief Send a stream_mag message
 * @param chan TTAlink channel to send the message
 *
 * @param mag1  .
 * @param yaw1  .
 * @param mag_root1  .
 * @param mag2  .
 * @param yaw2  .
 * @param mag_root2  .
 */
static inline void ttalink_stream_mag_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *mag1, float yaw1, float mag_root1, const float *mag2, float yaw2, float mag_root2)
{
    _ttalink_stream_mag_send(dst_addr, src_addr, chan, mag1, yaw1, mag_root1, mag2, yaw2, mag_root2, false);
}

/**
 * @brief Send a stream_mag message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param mag1  .
 * @param yaw1  .
 * @param mag_root1  .
 * @param mag2  .
 * @param yaw2  .
 * @param mag_root2  .
 */
static inline void ttalink_stream_mag_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *mag1, float yaw1, float mag_root1, const float *mag2, float yaw2, float mag_root2)
{
    _ttalink_stream_mag_send(dst_addr, src_addr, chan, mag1, yaw1, mag_root1, mag2, yaw2, mag_root2, true);
}


static inline void _ttalink_stream_mag_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_stream_mag_t* stream_mag, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_stream_mag_send_nocrc(dst_addr, src_addr, chan, stream_mag->mag1, stream_mag->yaw1, stream_mag->mag_root1, stream_mag->mag2, stream_mag->yaw2, stream_mag->mag_root2);
    }else{
        ttalink_stream_mag_send(dst_addr, src_addr, chan, stream_mag->mag1, stream_mag->yaw1, stream_mag->mag_root1, stream_mag->mag2, stream_mag->yaw2, stream_mag->mag_root2);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_MAG, (const char *)stream_mag, TTALINK_MSG_ID_STREAM_MAG_MIN_LEN, TTALINK_MSG_ID_STREAM_MAG_LEN, TTALINK_MSG_ID_STREAM_MAG_CRC, nocrc);
#endif
}

/**
 * @brief Send a stream_mag message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_stream_mag_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_stream_mag_t* stream_mag)
{
    _ttalink_stream_mag_send_struct(dst_addr, src_addr, chan, stream_mag, false);
}

/**
 * @brief Send a stream_mag message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_stream_mag_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_stream_mag_t* stream_mag)
{
    _ttalink_stream_mag_send_struct(dst_addr, src_addr, chan, stream_mag, true);
}

#if TTALINK_MSG_ID_STREAM_MAG_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_stream_mag_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *mag1, float yaw1, float mag_root1, const float *mag2, float yaw2, float mag_root2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_float(buf, 12, yaw1);
    _tta_put_float(buf, 16, mag_root1);
    _tta_put_float(buf, 32, yaw2);
    _tta_put_float(buf, 36, mag_root2);
    _tta_put_float_array(buf, 0, mag1, 3);
    _tta_put_float_array(buf, 20, mag2, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_MAG, buf, TTALINK_MSG_ID_STREAM_MAG_MIN_LEN, TTALINK_MSG_ID_STREAM_MAG_LEN, TTALINK_MSG_ID_STREAM_MAG_CRC, nocrc);
#else
    ttalink_stream_mag_t *packet = (ttalink_stream_mag_t *)msgbuf;
    packet->yaw1 = yaw1;
    packet->mag_root1 = mag_root1;
    packet->yaw2 = yaw2;
    packet->mag_root2 = mag_root2;
    tta_array_memcpy(packet->mag1, mag1, sizeof(float)*3);
    tta_array_memcpy(packet->mag2, mag2, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_MAG, (const char *)packet, TTALINK_MSG_ID_STREAM_MAG_MIN_LEN, TTALINK_MSG_ID_STREAM_MAG_LEN, TTALINK_MSG_ID_STREAM_MAG_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_stream_mag_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *mag1, float yaw1, float mag_root1, const float *mag2, float yaw2, float mag_root2)
{
    _ttalink_stream_mag_send_buf(dst_addr, src_addr, msgbuf, chan, mag1, yaw1, mag_root1, mag2, yaw2, mag_root2, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_stream_mag_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *mag1, float yaw1, float mag_root1, const float *mag2, float yaw2, float mag_root2)
{
    _ttalink_stream_mag_send_buf(dst_addr, src_addr, msgbuf, chan, mag1, yaw1, mag_root1, mag2, yaw2, mag_root2, true);
}
#endif

#endif

// MESSAGE STREAM_MAG UNPACKING


/**
 * @brief Get field mag1 from stream_mag message
 *
 * @return  .
 */
static inline uint16_t ttalink_stream_mag_get_mag1(const ttalink_message_t* msg, float *mag1)
{
    return _TTA_RETURN_float_array(msg, mag1, 3,  0);
}

/**
 * @brief Get field yaw1 from stream_mag message
 *
 * @return  .
 */
static inline float ttalink_stream_mag_get_yaw1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  12);
}

/**
 * @brief Get field mag_root1 from stream_mag message
 *
 * @return  .
 */
static inline float ttalink_stream_mag_get_mag_root1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field mag2 from stream_mag message
 *
 * @return  .
 */
static inline uint16_t ttalink_stream_mag_get_mag2(const ttalink_message_t* msg, float *mag2)
{
    return _TTA_RETURN_float_array(msg, mag2, 3,  20);
}

/**
 * @brief Get field yaw2 from stream_mag message
 *
 * @return  .
 */
static inline float ttalink_stream_mag_get_yaw2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Get field mag_root2 from stream_mag message
 *
 * @return  .
 */
static inline float ttalink_stream_mag_get_mag_root2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  36);
}

/**
 * @brief Decode a stream_mag message into a struct
 *
 * @param msg The message to decode
 * @param stream_mag C-struct to decode the message contents into
 */
static inline void ttalink_stream_mag_decode(const ttalink_message_t* msg, ttalink_stream_mag_t* stream_mag)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_stream_mag_get_mag1(msg, stream_mag->mag1);
    stream_mag->yaw1 = ttalink_stream_mag_get_yaw1(msg);
    stream_mag->mag_root1 = ttalink_stream_mag_get_mag_root1(msg);
    ttalink_stream_mag_get_mag2(msg, stream_mag->mag2);
    stream_mag->yaw2 = ttalink_stream_mag_get_yaw2(msg);
    stream_mag->mag_root2 = ttalink_stream_mag_get_mag_root2(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_STREAM_MAG_LEN? msg->len : TTALINK_MSG_ID_STREAM_MAG_LEN;
        memset(stream_mag, 0, TTALINK_MSG_ID_STREAM_MAG_LEN);
    memcpy(stream_mag, _TTA_PAYLOAD(msg), len);
#endif
}
