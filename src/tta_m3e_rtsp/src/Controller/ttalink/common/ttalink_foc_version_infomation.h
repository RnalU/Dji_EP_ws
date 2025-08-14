#pragma once
// MESSAGE FOC_VERSION_INFOMATION PACKING

#define TTALINK_MSG_ID_FOC_VERSION_INFOMATION 2018

TTAPACKED(
typedef struct __ttalink_foc_version_infomation_t {
 uint32_t update_time; /*< system time*/
}) ttalink_foc_version_infomation_t;

#define TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN 4
#define TTALINK_MSG_ID_FOC_VERSION_INFOMATION_MIN_LEN 4
#define TTALINK_MSG_ID_2018_LEN 4
#define TTALINK_MSG_ID_2018_MIN_LEN 4

#define TTALINK_MSG_ID_FOC_VERSION_INFOMATION_CRC 114
#define TTALINK_MSG_ID_2018_CRC 114



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_FOC_VERSION_INFOMATION { \
    2018, \
    "FOC_VERSION_INFOMATION", \
    1, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_foc_version_infomation_t, update_time) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_FOC_VERSION_INFOMATION { \
    "FOC_VERSION_INFOMATION", \
    1, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_foc_version_infomation_t, update_time) }, \
         } \
}
#endif


static inline uint16_t _ttalink_foc_version_infomation_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN];
    _tta_put_uint32_t(buf, 0, update_time);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN);
#else
    ttalink_foc_version_infomation_t packet;
    packet.update_time = update_time;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FOC_VERSION_INFOMATION;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_MIN_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_CRC, nocrc);
}

/**
 * @brief Pack a foc_version_infomation message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time system time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_version_infomation_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time)
{
    return _ttalink_foc_version_infomation_pack(dst_addr, src_addr, msg,  update_time, false);
}

/**
 * @brief Pack a foc_version_infomation message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time system time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_version_infomation_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time)
{
    return _ttalink_foc_version_infomation_pack(dst_addr, src_addr, msg,  update_time, true);
}


static inline uint16_t _ttalink_foc_version_infomation_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN];
    _tta_put_uint32_t(buf, 0, update_time);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN);
#else
    ttalink_foc_version_infomation_t packet;
    packet.update_time = update_time;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FOC_VERSION_INFOMATION;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_MIN_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_CRC, nocrc);
}

/**
 * @brief Pack a foc_version_infomation message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time system time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_version_infomation_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time)
{
    return _ttalink_foc_version_infomation_pack_chan(dst_addr, src_addr, chan, msg,  update_time, false);
}

/**
 * @brief Pack a foc_version_infomation message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time system time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_version_infomation_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time)
{
    return _ttalink_foc_version_infomation_pack_chan(dst_addr, src_addr, chan, msg,  update_time, true);
}


static inline uint16_t _ttalink_foc_version_infomation_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_version_infomation_t* foc_version_infomation, bool nocrc)
{
    if(nocrc){
        return ttalink_foc_version_infomation_pack_nocrc(dst_addr, src_addr, msg, foc_version_infomation->update_time);
    }else{
        return ttalink_foc_version_infomation_pack(dst_addr, src_addr, msg, foc_version_infomation->update_time);
    }
    
}

/**
 * @brief Encode a foc_version_infomation struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param foc_version_infomation C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_version_infomation_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_version_infomation_t* foc_version_infomation)
{
    return _ttalink_foc_version_infomation_encode(dst_addr, src_addr, msg, foc_version_infomation, false);
}

/**
 * @brief Encode a foc_version_infomation struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param foc_version_infomation C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_version_infomation_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_version_infomation_t* foc_version_infomation)
{
    return _ttalink_foc_version_infomation_encode(dst_addr, src_addr, msg, foc_version_infomation, true);
}


static inline uint16_t _ttalink_foc_version_infomation_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_version_infomation_t* foc_version_infomation, bool nocrc)
{
    if(nocrc){
        return ttalink_foc_version_infomation_pack_chan_nocrc(dst_addr, src_addr, chan, msg, foc_version_infomation->update_time);
    }else{
        return ttalink_foc_version_infomation_pack_chan(dst_addr, src_addr, chan, msg, foc_version_infomation->update_time);
    }
}

/**
 * @brief Encode a foc_version_infomation struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param foc_version_infomation C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_version_infomation_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_version_infomation_t* foc_version_infomation)
{
    return _ttalink_foc_version_infomation_encode_chan(dst_addr, src_addr, chan, msg, foc_version_infomation, false);
}

/**
 * @brief Encode a foc_version_infomation struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param foc_version_infomation C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_version_infomation_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_version_infomation_t* foc_version_infomation)
{
    return _ttalink_foc_version_infomation_encode_chan(dst_addr, src_addr, chan, msg, foc_version_infomation, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_foc_version_infomation_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN];
    _tta_put_uint32_t(buf, 0, update_time);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_VERSION_INFOMATION, buf, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_MIN_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_CRC, nocrc);
#else
    ttalink_foc_version_infomation_t packet;
    packet.update_time = update_time;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_VERSION_INFOMATION, (const char *)&packet, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_MIN_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_CRC, nocrc);
#endif
}

/**
 * @brief Send a foc_version_infomation message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time system time
 */
static inline void ttalink_foc_version_infomation_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time)
{
    _ttalink_foc_version_infomation_send(dst_addr, src_addr, chan, update_time, false);
}

/**
 * @brief Send a foc_version_infomation message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time system time
 */
static inline void ttalink_foc_version_infomation_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time)
{
    _ttalink_foc_version_infomation_send(dst_addr, src_addr, chan, update_time, true);
}


static inline void _ttalink_foc_version_infomation_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_version_infomation_t* foc_version_infomation, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_foc_version_infomation_send_nocrc(dst_addr, src_addr, chan, foc_version_infomation->update_time);
    }else{
        ttalink_foc_version_infomation_send(dst_addr, src_addr, chan, foc_version_infomation->update_time);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_VERSION_INFOMATION, (const char *)foc_version_infomation, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_MIN_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_CRC, nocrc);
#endif
}

/**
 * @brief Send a foc_version_infomation message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_foc_version_infomation_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_version_infomation_t* foc_version_infomation)
{
    _ttalink_foc_version_infomation_send_struct(dst_addr, src_addr, chan, foc_version_infomation, false);
}

/**
 * @brief Send a foc_version_infomation message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_foc_version_infomation_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_version_infomation_t* foc_version_infomation)
{
    _ttalink_foc_version_infomation_send_struct(dst_addr, src_addr, chan, foc_version_infomation, true);
}

#if TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_foc_version_infomation_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_VERSION_INFOMATION, buf, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_MIN_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_CRC, nocrc);
#else
    ttalink_foc_version_infomation_t *packet = (ttalink_foc_version_infomation_t *)msgbuf;
    packet->update_time = update_time;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_VERSION_INFOMATION, (const char *)packet, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_MIN_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_foc_version_infomation_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time)
{
    _ttalink_foc_version_infomation_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_foc_version_infomation_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time)
{
    _ttalink_foc_version_infomation_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, true);
}
#endif

#endif

// MESSAGE FOC_VERSION_INFOMATION UNPACKING


/**
 * @brief Get field update_time from foc_version_infomation message
 *
 * @return system time
 */
static inline uint32_t ttalink_foc_version_infomation_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a foc_version_infomation message into a struct
 *
 * @param msg The message to decode
 * @param foc_version_infomation C-struct to decode the message contents into
 */
static inline void ttalink_foc_version_infomation_decode(const ttalink_message_t* msg, ttalink_foc_version_infomation_t* foc_version_infomation)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    foc_version_infomation->update_time = ttalink_foc_version_infomation_get_update_time(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN? msg->len : TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN;
        memset(foc_version_infomation, 0, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_LEN);
    memcpy(foc_version_infomation, _TTA_PAYLOAD(msg), len);
#endif
}
