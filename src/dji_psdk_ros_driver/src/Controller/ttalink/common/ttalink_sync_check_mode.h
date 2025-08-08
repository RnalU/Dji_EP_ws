#pragma once
// MESSAGE SYNC_CHECK_MODE PACKING

#define TTALINK_MSG_ID_SYNC_CHECK_MODE 2114

TTAPACKED(
typedef struct __ttalink_sync_check_mode_t {
 uint32_t simple_time; /*<  .*/
 uint8_t ctrl_type; /*<  .*/
}) ttalink_sync_check_mode_t;

#define TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN 5
#define TTALINK_MSG_ID_SYNC_CHECK_MODE_MIN_LEN 5
#define TTALINK_MSG_ID_2114_LEN 5
#define TTALINK_MSG_ID_2114_MIN_LEN 5

#define TTALINK_MSG_ID_SYNC_CHECK_MODE_CRC 161
#define TTALINK_MSG_ID_2114_CRC 161



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SYNC_CHECK_MODE { \
    2114, \
    "SYNC_CHECK_MODE", \
    2, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_check_mode_t, simple_time) }, \
         { "ctrl_type", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_sync_check_mode_t, ctrl_type) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SYNC_CHECK_MODE { \
    "SYNC_CHECK_MODE", \
    2, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_check_mode_t, simple_time) }, \
         { "ctrl_type", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_sync_check_mode_t, ctrl_type) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sync_check_mode_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint8_t ctrl_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, ctrl_type);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN);
#else
    ttalink_sync_check_mode_t packet;
    packet.simple_time = simple_time;
    packet.ctrl_type = ctrl_type;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_CHECK_MODE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SYNC_CHECK_MODE_MIN_LEN, TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN, TTALINK_MSG_ID_SYNC_CHECK_MODE_CRC, nocrc);
}

/**
 * @brief Pack a sync_check_mode message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  .
 * @param ctrl_type  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_check_mode_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint8_t ctrl_type)
{
    return _ttalink_sync_check_mode_pack(dst_addr, src_addr, msg,  simple_time, ctrl_type, false);
}

/**
 * @brief Pack a sync_check_mode message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  .
 * @param ctrl_type  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_check_mode_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint8_t ctrl_type)
{
    return _ttalink_sync_check_mode_pack(dst_addr, src_addr, msg,  simple_time, ctrl_type, true);
}


static inline uint16_t _ttalink_sync_check_mode_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint8_t ctrl_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, ctrl_type);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN);
#else
    ttalink_sync_check_mode_t packet;
    packet.simple_time = simple_time;
    packet.ctrl_type = ctrl_type;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_CHECK_MODE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CHECK_MODE_MIN_LEN, TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN, TTALINK_MSG_ID_SYNC_CHECK_MODE_CRC, nocrc);
}

/**
 * @brief Pack a sync_check_mode message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  .
 * @param ctrl_type  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_check_mode_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint8_t ctrl_type)
{
    return _ttalink_sync_check_mode_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, ctrl_type, false);
}

/**
 * @brief Pack a sync_check_mode message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  .
 * @param ctrl_type  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_check_mode_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint8_t ctrl_type)
{
    return _ttalink_sync_check_mode_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, ctrl_type, true);
}


static inline uint16_t _ttalink_sync_check_mode_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_check_mode_t* sync_check_mode, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_check_mode_pack_nocrc(dst_addr, src_addr, msg, sync_check_mode->simple_time, sync_check_mode->ctrl_type);
    }else{
        return ttalink_sync_check_mode_pack(dst_addr, src_addr, msg, sync_check_mode->simple_time, sync_check_mode->ctrl_type);
    }
    
}

/**
 * @brief Encode a sync_check_mode struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_check_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_check_mode_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_check_mode_t* sync_check_mode)
{
    return _ttalink_sync_check_mode_encode(dst_addr, src_addr, msg, sync_check_mode, false);
}

/**
 * @brief Encode a sync_check_mode struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_check_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_check_mode_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_check_mode_t* sync_check_mode)
{
    return _ttalink_sync_check_mode_encode(dst_addr, src_addr, msg, sync_check_mode, true);
}


static inline uint16_t _ttalink_sync_check_mode_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_check_mode_t* sync_check_mode, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_check_mode_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sync_check_mode->simple_time, sync_check_mode->ctrl_type);
    }else{
        return ttalink_sync_check_mode_pack_chan(dst_addr, src_addr, chan, msg, sync_check_mode->simple_time, sync_check_mode->ctrl_type);
    }
}

/**
 * @brief Encode a sync_check_mode struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_check_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_check_mode_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_check_mode_t* sync_check_mode)
{
    return _ttalink_sync_check_mode_encode_chan(dst_addr, src_addr, chan, msg, sync_check_mode, false);
}

/**
 * @brief Encode a sync_check_mode struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_check_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_check_mode_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_check_mode_t* sync_check_mode)
{
    return _ttalink_sync_check_mode_encode_chan(dst_addr, src_addr, chan, msg, sync_check_mode, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sync_check_mode_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint8_t ctrl_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, ctrl_type);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CHECK_MODE, buf, TTALINK_MSG_ID_SYNC_CHECK_MODE_MIN_LEN, TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN, TTALINK_MSG_ID_SYNC_CHECK_MODE_CRC, nocrc);
#else
    ttalink_sync_check_mode_t packet;
    packet.simple_time = simple_time;
    packet.ctrl_type = ctrl_type;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CHECK_MODE, (const char *)&packet, TTALINK_MSG_ID_SYNC_CHECK_MODE_MIN_LEN, TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN, TTALINK_MSG_ID_SYNC_CHECK_MODE_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_check_mode message
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  .
 * @param ctrl_type  .
 */
static inline void ttalink_sync_check_mode_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint8_t ctrl_type)
{
    _ttalink_sync_check_mode_send(dst_addr, src_addr, chan, simple_time, ctrl_type, false);
}

/**
 * @brief Send a sync_check_mode message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  .
 * @param ctrl_type  .
 */
static inline void ttalink_sync_check_mode_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint8_t ctrl_type)
{
    _ttalink_sync_check_mode_send(dst_addr, src_addr, chan, simple_time, ctrl_type, true);
}


static inline void _ttalink_sync_check_mode_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_check_mode_t* sync_check_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sync_check_mode_send_nocrc(dst_addr, src_addr, chan, sync_check_mode->simple_time, sync_check_mode->ctrl_type);
    }else{
        ttalink_sync_check_mode_send(dst_addr, src_addr, chan, sync_check_mode->simple_time, sync_check_mode->ctrl_type);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CHECK_MODE, (const char *)sync_check_mode, TTALINK_MSG_ID_SYNC_CHECK_MODE_MIN_LEN, TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN, TTALINK_MSG_ID_SYNC_CHECK_MODE_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_check_mode message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_check_mode_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_check_mode_t* sync_check_mode)
{
    _ttalink_sync_check_mode_send_struct(dst_addr, src_addr, chan, sync_check_mode, false);
}

/**
 * @brief Send a sync_check_mode message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_check_mode_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_check_mode_t* sync_check_mode)
{
    _ttalink_sync_check_mode_send_struct(dst_addr, src_addr, chan, sync_check_mode, true);
}

#if TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sync_check_mode_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint8_t ctrl_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, ctrl_type);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CHECK_MODE, buf, TTALINK_MSG_ID_SYNC_CHECK_MODE_MIN_LEN, TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN, TTALINK_MSG_ID_SYNC_CHECK_MODE_CRC, nocrc);
#else
    ttalink_sync_check_mode_t *packet = (ttalink_sync_check_mode_t *)msgbuf;
    packet->simple_time = simple_time;
    packet->ctrl_type = ctrl_type;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CHECK_MODE, (const char *)packet, TTALINK_MSG_ID_SYNC_CHECK_MODE_MIN_LEN, TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN, TTALINK_MSG_ID_SYNC_CHECK_MODE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sync_check_mode_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint8_t ctrl_type)
{
    _ttalink_sync_check_mode_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, ctrl_type, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sync_check_mode_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint8_t ctrl_type)
{
    _ttalink_sync_check_mode_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, ctrl_type, true);
}
#endif

#endif

// MESSAGE SYNC_CHECK_MODE UNPACKING


/**
 * @brief Get field simple_time from sync_check_mode message
 *
 * @return  .
 */
static inline uint32_t ttalink_sync_check_mode_get_simple_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field ctrl_type from sync_check_mode message
 *
 * @return  .
 */
static inline uint8_t ttalink_sync_check_mode_get_ctrl_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Decode a sync_check_mode message into a struct
 *
 * @param msg The message to decode
 * @param sync_check_mode C-struct to decode the message contents into
 */
static inline void ttalink_sync_check_mode_decode(const ttalink_message_t* msg, ttalink_sync_check_mode_t* sync_check_mode)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sync_check_mode->simple_time = ttalink_sync_check_mode_get_simple_time(msg);
    sync_check_mode->ctrl_type = ttalink_sync_check_mode_get_ctrl_type(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN? msg->len : TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN;
        memset(sync_check_mode, 0, TTALINK_MSG_ID_SYNC_CHECK_MODE_LEN);
    memcpy(sync_check_mode, _TTA_PAYLOAD(msg), len);
#endif
}
