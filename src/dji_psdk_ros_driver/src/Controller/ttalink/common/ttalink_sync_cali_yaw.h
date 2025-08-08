#pragma once
// MESSAGE SYNC_CALI_YAW PACKING

#define TTALINK_MSG_ID_SYNC_CALI_YAW 2126

TTAPACKED(
typedef struct __ttalink_sync_cali_yaw_t {
 uint32_t simple_time; /*<  .*/
 float Yaw1Bias; /*<  .*/
 float Yaw2Bias; /*<  .*/
}) ttalink_sync_cali_yaw_t;

#define TTALINK_MSG_ID_SYNC_CALI_YAW_LEN 12
#define TTALINK_MSG_ID_SYNC_CALI_YAW_MIN_LEN 12
#define TTALINK_MSG_ID_2126_LEN 12
#define TTALINK_MSG_ID_2126_MIN_LEN 12

#define TTALINK_MSG_ID_SYNC_CALI_YAW_CRC 244
#define TTALINK_MSG_ID_2126_CRC 244



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SYNC_CALI_YAW { \
    2126, \
    "SYNC_CALI_YAW", \
    3, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_cali_yaw_t, simple_time) }, \
         { "Yaw1Bias", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_sync_cali_yaw_t, Yaw1Bias) }, \
         { "Yaw2Bias", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_sync_cali_yaw_t, Yaw2Bias) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SYNC_CALI_YAW { \
    "SYNC_CALI_YAW", \
    3, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_cali_yaw_t, simple_time) }, \
         { "Yaw1Bias", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_sync_cali_yaw_t, Yaw1Bias) }, \
         { "Yaw2Bias", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_sync_cali_yaw_t, Yaw2Bias) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sync_cali_yaw_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, float Yaw1Bias, float Yaw2Bias, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CALI_YAW_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float(buf, 4, Yaw1Bias);
    _tta_put_float(buf, 8, Yaw2Bias);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_CALI_YAW_LEN);
#else
    ttalink_sync_cali_yaw_t packet;
    packet.simple_time = simple_time;
    packet.Yaw1Bias = Yaw1Bias;
    packet.Yaw2Bias = Yaw2Bias;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_CALI_YAW_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_CALI_YAW;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SYNC_CALI_YAW_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_YAW_LEN, TTALINK_MSG_ID_SYNC_CALI_YAW_CRC, nocrc);
}

/**
 * @brief Pack a sync_cali_yaw message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  .
 * @param Yaw1Bias  .
 * @param Yaw2Bias  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_yaw_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, float Yaw1Bias, float Yaw2Bias)
{
    return _ttalink_sync_cali_yaw_pack(dst_addr, src_addr, msg,  simple_time, Yaw1Bias, Yaw2Bias, false);
}

/**
 * @brief Pack a sync_cali_yaw message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  .
 * @param Yaw1Bias  .
 * @param Yaw2Bias  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_yaw_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, float Yaw1Bias, float Yaw2Bias)
{
    return _ttalink_sync_cali_yaw_pack(dst_addr, src_addr, msg,  simple_time, Yaw1Bias, Yaw2Bias, true);
}


static inline uint16_t _ttalink_sync_cali_yaw_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,float Yaw1Bias,float Yaw2Bias, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CALI_YAW_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float(buf, 4, Yaw1Bias);
    _tta_put_float(buf, 8, Yaw2Bias);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_CALI_YAW_LEN);
#else
    ttalink_sync_cali_yaw_t packet;
    packet.simple_time = simple_time;
    packet.Yaw1Bias = Yaw1Bias;
    packet.Yaw2Bias = Yaw2Bias;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_CALI_YAW_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_CALI_YAW;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_YAW_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_YAW_LEN, TTALINK_MSG_ID_SYNC_CALI_YAW_CRC, nocrc);
}

/**
 * @brief Pack a sync_cali_yaw message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  .
 * @param Yaw1Bias  .
 * @param Yaw2Bias  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_yaw_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,float Yaw1Bias,float Yaw2Bias)
{
    return _ttalink_sync_cali_yaw_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, Yaw1Bias, Yaw2Bias, false);
}

/**
 * @brief Pack a sync_cali_yaw message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  .
 * @param Yaw1Bias  .
 * @param Yaw2Bias  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_yaw_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,float Yaw1Bias,float Yaw2Bias)
{
    return _ttalink_sync_cali_yaw_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, Yaw1Bias, Yaw2Bias, true);
}


static inline uint16_t _ttalink_sync_cali_yaw_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_cali_yaw_t* sync_cali_yaw, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_cali_yaw_pack_nocrc(dst_addr, src_addr, msg, sync_cali_yaw->simple_time, sync_cali_yaw->Yaw1Bias, sync_cali_yaw->Yaw2Bias);
    }else{
        return ttalink_sync_cali_yaw_pack(dst_addr, src_addr, msg, sync_cali_yaw->simple_time, sync_cali_yaw->Yaw1Bias, sync_cali_yaw->Yaw2Bias);
    }
    
}

/**
 * @brief Encode a sync_cali_yaw struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_yaw C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_yaw_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_cali_yaw_t* sync_cali_yaw)
{
    return _ttalink_sync_cali_yaw_encode(dst_addr, src_addr, msg, sync_cali_yaw, false);
}

/**
 * @brief Encode a sync_cali_yaw struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_yaw C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_yaw_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_cali_yaw_t* sync_cali_yaw)
{
    return _ttalink_sync_cali_yaw_encode(dst_addr, src_addr, msg, sync_cali_yaw, true);
}


static inline uint16_t _ttalink_sync_cali_yaw_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_cali_yaw_t* sync_cali_yaw, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_cali_yaw_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sync_cali_yaw->simple_time, sync_cali_yaw->Yaw1Bias, sync_cali_yaw->Yaw2Bias);
    }else{
        return ttalink_sync_cali_yaw_pack_chan(dst_addr, src_addr, chan, msg, sync_cali_yaw->simple_time, sync_cali_yaw->Yaw1Bias, sync_cali_yaw->Yaw2Bias);
    }
}

/**
 * @brief Encode a sync_cali_yaw struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_yaw C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_yaw_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_cali_yaw_t* sync_cali_yaw)
{
    return _ttalink_sync_cali_yaw_encode_chan(dst_addr, src_addr, chan, msg, sync_cali_yaw, false);
}

/**
 * @brief Encode a sync_cali_yaw struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_yaw C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_yaw_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_cali_yaw_t* sync_cali_yaw)
{
    return _ttalink_sync_cali_yaw_encode_chan(dst_addr, src_addr, chan, msg, sync_cali_yaw, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sync_cali_yaw_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, float Yaw1Bias, float Yaw2Bias, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CALI_YAW_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float(buf, 4, Yaw1Bias);
    _tta_put_float(buf, 8, Yaw2Bias);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_YAW, buf, TTALINK_MSG_ID_SYNC_CALI_YAW_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_YAW_LEN, TTALINK_MSG_ID_SYNC_CALI_YAW_CRC, nocrc);
#else
    ttalink_sync_cali_yaw_t packet;
    packet.simple_time = simple_time;
    packet.Yaw1Bias = Yaw1Bias;
    packet.Yaw2Bias = Yaw2Bias;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_YAW, (const char *)&packet, TTALINK_MSG_ID_SYNC_CALI_YAW_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_YAW_LEN, TTALINK_MSG_ID_SYNC_CALI_YAW_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_cali_yaw message
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  .
 * @param Yaw1Bias  .
 * @param Yaw2Bias  .
 */
static inline void ttalink_sync_cali_yaw_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, float Yaw1Bias, float Yaw2Bias)
{
    _ttalink_sync_cali_yaw_send(dst_addr, src_addr, chan, simple_time, Yaw1Bias, Yaw2Bias, false);
}

/**
 * @brief Send a sync_cali_yaw message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  .
 * @param Yaw1Bias  .
 * @param Yaw2Bias  .
 */
static inline void ttalink_sync_cali_yaw_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, float Yaw1Bias, float Yaw2Bias)
{
    _ttalink_sync_cali_yaw_send(dst_addr, src_addr, chan, simple_time, Yaw1Bias, Yaw2Bias, true);
}


static inline void _ttalink_sync_cali_yaw_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_cali_yaw_t* sync_cali_yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sync_cali_yaw_send_nocrc(dst_addr, src_addr, chan, sync_cali_yaw->simple_time, sync_cali_yaw->Yaw1Bias, sync_cali_yaw->Yaw2Bias);
    }else{
        ttalink_sync_cali_yaw_send(dst_addr, src_addr, chan, sync_cali_yaw->simple_time, sync_cali_yaw->Yaw1Bias, sync_cali_yaw->Yaw2Bias);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_YAW, (const char *)sync_cali_yaw, TTALINK_MSG_ID_SYNC_CALI_YAW_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_YAW_LEN, TTALINK_MSG_ID_SYNC_CALI_YAW_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_cali_yaw message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_cali_yaw_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_cali_yaw_t* sync_cali_yaw)
{
    _ttalink_sync_cali_yaw_send_struct(dst_addr, src_addr, chan, sync_cali_yaw, false);
}

/**
 * @brief Send a sync_cali_yaw message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_cali_yaw_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_cali_yaw_t* sync_cali_yaw)
{
    _ttalink_sync_cali_yaw_send_struct(dst_addr, src_addr, chan, sync_cali_yaw, true);
}

#if TTALINK_MSG_ID_SYNC_CALI_YAW_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sync_cali_yaw_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, float Yaw1Bias, float Yaw2Bias, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float(buf, 4, Yaw1Bias);
    _tta_put_float(buf, 8, Yaw2Bias);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_YAW, buf, TTALINK_MSG_ID_SYNC_CALI_YAW_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_YAW_LEN, TTALINK_MSG_ID_SYNC_CALI_YAW_CRC, nocrc);
#else
    ttalink_sync_cali_yaw_t *packet = (ttalink_sync_cali_yaw_t *)msgbuf;
    packet->simple_time = simple_time;
    packet->Yaw1Bias = Yaw1Bias;
    packet->Yaw2Bias = Yaw2Bias;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_YAW, (const char *)packet, TTALINK_MSG_ID_SYNC_CALI_YAW_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_YAW_LEN, TTALINK_MSG_ID_SYNC_CALI_YAW_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sync_cali_yaw_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, float Yaw1Bias, float Yaw2Bias)
{
    _ttalink_sync_cali_yaw_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, Yaw1Bias, Yaw2Bias, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sync_cali_yaw_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, float Yaw1Bias, float Yaw2Bias)
{
    _ttalink_sync_cali_yaw_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, Yaw1Bias, Yaw2Bias, true);
}
#endif

#endif

// MESSAGE SYNC_CALI_YAW UNPACKING


/**
 * @brief Get field simple_time from sync_cali_yaw message
 *
 * @return  .
 */
static inline uint32_t ttalink_sync_cali_yaw_get_simple_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field Yaw1Bias from sync_cali_yaw message
 *
 * @return  .
 */
static inline float ttalink_sync_cali_yaw_get_Yaw1Bias(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  4);
}

/**
 * @brief Get field Yaw2Bias from sync_cali_yaw message
 *
 * @return  .
 */
static inline float ttalink_sync_cali_yaw_get_Yaw2Bias(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  8);
}

/**
 * @brief Decode a sync_cali_yaw message into a struct
 *
 * @param msg The message to decode
 * @param sync_cali_yaw C-struct to decode the message contents into
 */
static inline void ttalink_sync_cali_yaw_decode(const ttalink_message_t* msg, ttalink_sync_cali_yaw_t* sync_cali_yaw)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sync_cali_yaw->simple_time = ttalink_sync_cali_yaw_get_simple_time(msg);
    sync_cali_yaw->Yaw1Bias = ttalink_sync_cali_yaw_get_Yaw1Bias(msg);
    sync_cali_yaw->Yaw2Bias = ttalink_sync_cali_yaw_get_Yaw2Bias(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SYNC_CALI_YAW_LEN? msg->len : TTALINK_MSG_ID_SYNC_CALI_YAW_LEN;
        memset(sync_cali_yaw, 0, TTALINK_MSG_ID_SYNC_CALI_YAW_LEN);
    memcpy(sync_cali_yaw, _TTA_PAYLOAD(msg), len);
#endif
}
