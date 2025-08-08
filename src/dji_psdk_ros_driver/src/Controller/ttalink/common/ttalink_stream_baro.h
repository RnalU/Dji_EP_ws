#pragma once
// MESSAGE STREAM_BARO PACKING

#define TTALINK_MSG_ID_STREAM_BARO 2146

TTAPACKED(
typedef struct __ttalink_stream_baro_t {
 int16_t temperature1; /*<  .*/
 int32_t pressure1; /*<  .*/
 float alt1; /*<  .*/
 int16_t temperature2; /*<  .*/
 int32_t pressure2; /*<  .*/
 float alt2; /*<  .*/
}) ttalink_stream_baro_t;

#define TTALINK_MSG_ID_STREAM_BARO_LEN 20
#define TTALINK_MSG_ID_STREAM_BARO_MIN_LEN 20
#define TTALINK_MSG_ID_2146_LEN 20
#define TTALINK_MSG_ID_2146_MIN_LEN 20

#define TTALINK_MSG_ID_STREAM_BARO_CRC 29
#define TTALINK_MSG_ID_2146_CRC 29



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_STREAM_BARO { \
    2146, \
    "STREAM_BARO", \
    6, \
    {  { "temperature1", NULL, TTALINK_TYPE_INT16_T, 0, 0, offsetof(ttalink_stream_baro_t, temperature1) }, \
         { "pressure1", NULL, TTALINK_TYPE_INT32_T, 0, 2, offsetof(ttalink_stream_baro_t, pressure1) }, \
         { "alt1", NULL, TTALINK_TYPE_FLOAT, 0, 6, offsetof(ttalink_stream_baro_t, alt1) }, \
         { "temperature2", NULL, TTALINK_TYPE_INT16_T, 0, 10, offsetof(ttalink_stream_baro_t, temperature2) }, \
         { "pressure2", NULL, TTALINK_TYPE_INT32_T, 0, 12, offsetof(ttalink_stream_baro_t, pressure2) }, \
         { "alt2", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_stream_baro_t, alt2) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_STREAM_BARO { \
    "STREAM_BARO", \
    6, \
    {  { "temperature1", NULL, TTALINK_TYPE_INT16_T, 0, 0, offsetof(ttalink_stream_baro_t, temperature1) }, \
         { "pressure1", NULL, TTALINK_TYPE_INT32_T, 0, 2, offsetof(ttalink_stream_baro_t, pressure1) }, \
         { "alt1", NULL, TTALINK_TYPE_FLOAT, 0, 6, offsetof(ttalink_stream_baro_t, alt1) }, \
         { "temperature2", NULL, TTALINK_TYPE_INT16_T, 0, 10, offsetof(ttalink_stream_baro_t, temperature2) }, \
         { "pressure2", NULL, TTALINK_TYPE_INT32_T, 0, 12, offsetof(ttalink_stream_baro_t, pressure2) }, \
         { "alt2", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_stream_baro_t, alt2) }, \
         } \
}
#endif


static inline uint16_t _ttalink_stream_baro_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int16_t temperature1, int32_t pressure1, float alt1, int16_t temperature2, int32_t pressure2, float alt2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_STREAM_BARO_LEN];
    _tta_put_int16_t(buf, 0, temperature1);
    _tta_put_int32_t(buf, 2, pressure1);
    _tta_put_float(buf, 6, alt1);
    _tta_put_int16_t(buf, 10, temperature2);
    _tta_put_int32_t(buf, 12, pressure2);
    _tta_put_float(buf, 16, alt2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_STREAM_BARO_LEN);
#else
    ttalink_stream_baro_t packet;
    packet.temperature1 = temperature1;
    packet.pressure1 = pressure1;
    packet.alt1 = alt1;
    packet.temperature2 = temperature2;
    packet.pressure2 = pressure2;
    packet.alt2 = alt2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_STREAM_BARO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_STREAM_BARO;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_STREAM_BARO_MIN_LEN, TTALINK_MSG_ID_STREAM_BARO_LEN, TTALINK_MSG_ID_STREAM_BARO_CRC, nocrc);
}

/**
 * @brief Pack a stream_baro message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param temperature1  .
 * @param pressure1  .
 * @param alt1  .
 * @param temperature2  .
 * @param pressure2  .
 * @param alt2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_baro_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int16_t temperature1, int32_t pressure1, float alt1, int16_t temperature2, int32_t pressure2, float alt2)
{
    return _ttalink_stream_baro_pack(dst_addr, src_addr, msg,  temperature1, pressure1, alt1, temperature2, pressure2, alt2, false);
}

/**
 * @brief Pack a stream_baro message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param temperature1  .
 * @param pressure1  .
 * @param alt1  .
 * @param temperature2  .
 * @param pressure2  .
 * @param alt2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_baro_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int16_t temperature1, int32_t pressure1, float alt1, int16_t temperature2, int32_t pressure2, float alt2)
{
    return _ttalink_stream_baro_pack(dst_addr, src_addr, msg,  temperature1, pressure1, alt1, temperature2, pressure2, alt2, true);
}


static inline uint16_t _ttalink_stream_baro_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int16_t temperature1,int32_t pressure1,float alt1,int16_t temperature2,int32_t pressure2,float alt2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_STREAM_BARO_LEN];
    _tta_put_int16_t(buf, 0, temperature1);
    _tta_put_int32_t(buf, 2, pressure1);
    _tta_put_float(buf, 6, alt1);
    _tta_put_int16_t(buf, 10, temperature2);
    _tta_put_int32_t(buf, 12, pressure2);
    _tta_put_float(buf, 16, alt2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_STREAM_BARO_LEN);
#else
    ttalink_stream_baro_t packet;
    packet.temperature1 = temperature1;
    packet.pressure1 = pressure1;
    packet.alt1 = alt1;
    packet.temperature2 = temperature2;
    packet.pressure2 = pressure2;
    packet.alt2 = alt2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_STREAM_BARO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_STREAM_BARO;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_BARO_MIN_LEN, TTALINK_MSG_ID_STREAM_BARO_LEN, TTALINK_MSG_ID_STREAM_BARO_CRC, nocrc);
}

/**
 * @brief Pack a stream_baro message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param temperature1  .
 * @param pressure1  .
 * @param alt1  .
 * @param temperature2  .
 * @param pressure2  .
 * @param alt2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_baro_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int16_t temperature1,int32_t pressure1,float alt1,int16_t temperature2,int32_t pressure2,float alt2)
{
    return _ttalink_stream_baro_pack_chan(dst_addr, src_addr, chan, msg,  temperature1, pressure1, alt1, temperature2, pressure2, alt2, false);
}

/**
 * @brief Pack a stream_baro message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param temperature1  .
 * @param pressure1  .
 * @param alt1  .
 * @param temperature2  .
 * @param pressure2  .
 * @param alt2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_baro_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int16_t temperature1,int32_t pressure1,float alt1,int16_t temperature2,int32_t pressure2,float alt2)
{
    return _ttalink_stream_baro_pack_chan(dst_addr, src_addr, chan, msg,  temperature1, pressure1, alt1, temperature2, pressure2, alt2, true);
}


static inline uint16_t _ttalink_stream_baro_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_stream_baro_t* stream_baro, bool nocrc)
{
    if(nocrc){
        return ttalink_stream_baro_pack_nocrc(dst_addr, src_addr, msg, stream_baro->temperature1, stream_baro->pressure1, stream_baro->alt1, stream_baro->temperature2, stream_baro->pressure2, stream_baro->alt2);
    }else{
        return ttalink_stream_baro_pack(dst_addr, src_addr, msg, stream_baro->temperature1, stream_baro->pressure1, stream_baro->alt1, stream_baro->temperature2, stream_baro->pressure2, stream_baro->alt2);
    }
    
}

/**
 * @brief Encode a stream_baro struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param stream_baro C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_baro_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_stream_baro_t* stream_baro)
{
    return _ttalink_stream_baro_encode(dst_addr, src_addr, msg, stream_baro, false);
}

/**
 * @brief Encode a stream_baro struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param stream_baro C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_baro_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_stream_baro_t* stream_baro)
{
    return _ttalink_stream_baro_encode(dst_addr, src_addr, msg, stream_baro, true);
}


static inline uint16_t _ttalink_stream_baro_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_stream_baro_t* stream_baro, bool nocrc)
{
    if(nocrc){
        return ttalink_stream_baro_pack_chan_nocrc(dst_addr, src_addr, chan, msg, stream_baro->temperature1, stream_baro->pressure1, stream_baro->alt1, stream_baro->temperature2, stream_baro->pressure2, stream_baro->alt2);
    }else{
        return ttalink_stream_baro_pack_chan(dst_addr, src_addr, chan, msg, stream_baro->temperature1, stream_baro->pressure1, stream_baro->alt1, stream_baro->temperature2, stream_baro->pressure2, stream_baro->alt2);
    }
}

/**
 * @brief Encode a stream_baro struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param stream_baro C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_baro_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_stream_baro_t* stream_baro)
{
    return _ttalink_stream_baro_encode_chan(dst_addr, src_addr, chan, msg, stream_baro, false);
}

/**
 * @brief Encode a stream_baro struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param stream_baro C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_baro_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_stream_baro_t* stream_baro)
{
    return _ttalink_stream_baro_encode_chan(dst_addr, src_addr, chan, msg, stream_baro, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_stream_baro_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int16_t temperature1, int32_t pressure1, float alt1, int16_t temperature2, int32_t pressure2, float alt2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_STREAM_BARO_LEN];
    _tta_put_int16_t(buf, 0, temperature1);
    _tta_put_int32_t(buf, 2, pressure1);
    _tta_put_float(buf, 6, alt1);
    _tta_put_int16_t(buf, 10, temperature2);
    _tta_put_int32_t(buf, 12, pressure2);
    _tta_put_float(buf, 16, alt2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_BARO, buf, TTALINK_MSG_ID_STREAM_BARO_MIN_LEN, TTALINK_MSG_ID_STREAM_BARO_LEN, TTALINK_MSG_ID_STREAM_BARO_CRC, nocrc);
#else
    ttalink_stream_baro_t packet;
    packet.temperature1 = temperature1;
    packet.pressure1 = pressure1;
    packet.alt1 = alt1;
    packet.temperature2 = temperature2;
    packet.pressure2 = pressure2;
    packet.alt2 = alt2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_BARO, (const char *)&packet, TTALINK_MSG_ID_STREAM_BARO_MIN_LEN, TTALINK_MSG_ID_STREAM_BARO_LEN, TTALINK_MSG_ID_STREAM_BARO_CRC, nocrc);
#endif
}

/**
 * @brief Send a stream_baro message
 * @param chan TTAlink channel to send the message
 *
 * @param temperature1  .
 * @param pressure1  .
 * @param alt1  .
 * @param temperature2  .
 * @param pressure2  .
 * @param alt2  .
 */
static inline void ttalink_stream_baro_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int16_t temperature1, int32_t pressure1, float alt1, int16_t temperature2, int32_t pressure2, float alt2)
{
    _ttalink_stream_baro_send(dst_addr, src_addr, chan, temperature1, pressure1, alt1, temperature2, pressure2, alt2, false);
}

/**
 * @brief Send a stream_baro message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param temperature1  .
 * @param pressure1  .
 * @param alt1  .
 * @param temperature2  .
 * @param pressure2  .
 * @param alt2  .
 */
static inline void ttalink_stream_baro_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int16_t temperature1, int32_t pressure1, float alt1, int16_t temperature2, int32_t pressure2, float alt2)
{
    _ttalink_stream_baro_send(dst_addr, src_addr, chan, temperature1, pressure1, alt1, temperature2, pressure2, alt2, true);
}


static inline void _ttalink_stream_baro_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_stream_baro_t* stream_baro, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_stream_baro_send_nocrc(dst_addr, src_addr, chan, stream_baro->temperature1, stream_baro->pressure1, stream_baro->alt1, stream_baro->temperature2, stream_baro->pressure2, stream_baro->alt2);
    }else{
        ttalink_stream_baro_send(dst_addr, src_addr, chan, stream_baro->temperature1, stream_baro->pressure1, stream_baro->alt1, stream_baro->temperature2, stream_baro->pressure2, stream_baro->alt2);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_BARO, (const char *)stream_baro, TTALINK_MSG_ID_STREAM_BARO_MIN_LEN, TTALINK_MSG_ID_STREAM_BARO_LEN, TTALINK_MSG_ID_STREAM_BARO_CRC, nocrc);
#endif
}

/**
 * @brief Send a stream_baro message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_stream_baro_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_stream_baro_t* stream_baro)
{
    _ttalink_stream_baro_send_struct(dst_addr, src_addr, chan, stream_baro, false);
}

/**
 * @brief Send a stream_baro message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_stream_baro_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_stream_baro_t* stream_baro)
{
    _ttalink_stream_baro_send_struct(dst_addr, src_addr, chan, stream_baro, true);
}

#if TTALINK_MSG_ID_STREAM_BARO_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_stream_baro_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int16_t temperature1, int32_t pressure1, float alt1, int16_t temperature2, int32_t pressure2, float alt2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_int16_t(buf, 0, temperature1);
    _tta_put_int32_t(buf, 2, pressure1);
    _tta_put_float(buf, 6, alt1);
    _tta_put_int16_t(buf, 10, temperature2);
    _tta_put_int32_t(buf, 12, pressure2);
    _tta_put_float(buf, 16, alt2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_BARO, buf, TTALINK_MSG_ID_STREAM_BARO_MIN_LEN, TTALINK_MSG_ID_STREAM_BARO_LEN, TTALINK_MSG_ID_STREAM_BARO_CRC, nocrc);
#else
    ttalink_stream_baro_t *packet = (ttalink_stream_baro_t *)msgbuf;
    packet->temperature1 = temperature1;
    packet->pressure1 = pressure1;
    packet->alt1 = alt1;
    packet->temperature2 = temperature2;
    packet->pressure2 = pressure2;
    packet->alt2 = alt2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_BARO, (const char *)packet, TTALINK_MSG_ID_STREAM_BARO_MIN_LEN, TTALINK_MSG_ID_STREAM_BARO_LEN, TTALINK_MSG_ID_STREAM_BARO_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_stream_baro_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int16_t temperature1, int32_t pressure1, float alt1, int16_t temperature2, int32_t pressure2, float alt2)
{
    _ttalink_stream_baro_send_buf(dst_addr, src_addr, msgbuf, chan, temperature1, pressure1, alt1, temperature2, pressure2, alt2, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_stream_baro_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int16_t temperature1, int32_t pressure1, float alt1, int16_t temperature2, int32_t pressure2, float alt2)
{
    _ttalink_stream_baro_send_buf(dst_addr, src_addr, msgbuf, chan, temperature1, pressure1, alt1, temperature2, pressure2, alt2, true);
}
#endif

#endif

// MESSAGE STREAM_BARO UNPACKING


/**
 * @brief Get field temperature1 from stream_baro message
 *
 * @return  .
 */
static inline int16_t ttalink_stream_baro_get_temperature1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field pressure1 from stream_baro message
 *
 * @return  .
 */
static inline int32_t ttalink_stream_baro_get_pressure1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  2);
}

/**
 * @brief Get field alt1 from stream_baro message
 *
 * @return  .
 */
static inline float ttalink_stream_baro_get_alt1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  6);
}

/**
 * @brief Get field temperature2 from stream_baro message
 *
 * @return  .
 */
static inline int16_t ttalink_stream_baro_get_temperature2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field pressure2 from stream_baro message
 *
 * @return  .
 */
static inline int32_t ttalink_stream_baro_get_pressure2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt2 from stream_baro message
 *
 * @return  .
 */
static inline float ttalink_stream_baro_get_alt2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Decode a stream_baro message into a struct
 *
 * @param msg The message to decode
 * @param stream_baro C-struct to decode the message contents into
 */
static inline void ttalink_stream_baro_decode(const ttalink_message_t* msg, ttalink_stream_baro_t* stream_baro)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    stream_baro->temperature1 = ttalink_stream_baro_get_temperature1(msg);
    stream_baro->pressure1 = ttalink_stream_baro_get_pressure1(msg);
    stream_baro->alt1 = ttalink_stream_baro_get_alt1(msg);
    stream_baro->temperature2 = ttalink_stream_baro_get_temperature2(msg);
    stream_baro->pressure2 = ttalink_stream_baro_get_pressure2(msg);
    stream_baro->alt2 = ttalink_stream_baro_get_alt2(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_STREAM_BARO_LEN? msg->len : TTALINK_MSG_ID_STREAM_BARO_LEN;
        memset(stream_baro, 0, TTALINK_MSG_ID_STREAM_BARO_LEN);
    memcpy(stream_baro, _TTA_PAYLOAD(msg), len);
#endif
}
