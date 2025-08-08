#pragma once
// MESSAGE SYNC_TIME PACKING

#define TTALINK_MSG_ID_SYNC_TIME 2004

TTAPACKED(
typedef struct __ttalink_sync_time_t {
 uint32_t simple_time; /*<  simple_time*/
 uint64_t sync_time; /*<  sync_time*/
}) ttalink_sync_time_t;

#define TTALINK_MSG_ID_SYNC_TIME_LEN 12
#define TTALINK_MSG_ID_SYNC_TIME_MIN_LEN 12
#define TTALINK_MSG_ID_2004_LEN 12
#define TTALINK_MSG_ID_2004_MIN_LEN 12

#define TTALINK_MSG_ID_SYNC_TIME_CRC 148
#define TTALINK_MSG_ID_2004_CRC 148



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SYNC_TIME { \
    2004, \
    "SYNC_TIME", \
    2, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_time_t, simple_time) }, \
         { "sync_time", NULL, TTALINK_TYPE_UINT64_T, 0, 4, offsetof(ttalink_sync_time_t, sync_time) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SYNC_TIME { \
    "SYNC_TIME", \
    2, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_time_t, simple_time) }, \
         { "sync_time", NULL, TTALINK_TYPE_UINT64_T, 0, 4, offsetof(ttalink_sync_time_t, sync_time) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sync_time_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint64_t sync_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_TIME_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint64_t(buf, 4, sync_time);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_TIME_LEN);
#else
    ttalink_sync_time_t packet;
    packet.simple_time = simple_time;
    packet.sync_time = sync_time;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_TIME_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_TIME;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SYNC_TIME_MIN_LEN, TTALINK_MSG_ID_SYNC_TIME_LEN, TTALINK_MSG_ID_SYNC_TIME_CRC, nocrc);
}

/**
 * @brief Pack a sync_time message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  simple_time
 * @param sync_time  sync_time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_time_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint64_t sync_time)
{
    return _ttalink_sync_time_pack(dst_addr, src_addr, msg,  simple_time, sync_time, false);
}

/**
 * @brief Pack a sync_time message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  simple_time
 * @param sync_time  sync_time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_time_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint64_t sync_time)
{
    return _ttalink_sync_time_pack(dst_addr, src_addr, msg,  simple_time, sync_time, true);
}


static inline uint16_t _ttalink_sync_time_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint64_t sync_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_TIME_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint64_t(buf, 4, sync_time);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_TIME_LEN);
#else
    ttalink_sync_time_t packet;
    packet.simple_time = simple_time;
    packet.sync_time = sync_time;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_TIME_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_TIME;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_TIME_MIN_LEN, TTALINK_MSG_ID_SYNC_TIME_LEN, TTALINK_MSG_ID_SYNC_TIME_CRC, nocrc);
}

/**
 * @brief Pack a sync_time message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  simple_time
 * @param sync_time  sync_time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_time_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint64_t sync_time)
{
    return _ttalink_sync_time_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, sync_time, false);
}

/**
 * @brief Pack a sync_time message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  simple_time
 * @param sync_time  sync_time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_time_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint64_t sync_time)
{
    return _ttalink_sync_time_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, sync_time, true);
}


static inline uint16_t _ttalink_sync_time_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_time_t* sync_time, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_time_pack_nocrc(dst_addr, src_addr, msg, sync_time->simple_time, sync_time->sync_time);
    }else{
        return ttalink_sync_time_pack(dst_addr, src_addr, msg, sync_time->simple_time, sync_time->sync_time);
    }
    
}

/**
 * @brief Encode a sync_time struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_time C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_time_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_time_t* sync_time)
{
    return _ttalink_sync_time_encode(dst_addr, src_addr, msg, sync_time, false);
}

/**
 * @brief Encode a sync_time struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_time C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_time_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_time_t* sync_time)
{
    return _ttalink_sync_time_encode(dst_addr, src_addr, msg, sync_time, true);
}


static inline uint16_t _ttalink_sync_time_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_time_t* sync_time, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_time_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sync_time->simple_time, sync_time->sync_time);
    }else{
        return ttalink_sync_time_pack_chan(dst_addr, src_addr, chan, msg, sync_time->simple_time, sync_time->sync_time);
    }
}

/**
 * @brief Encode a sync_time struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_time C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_time_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_time_t* sync_time)
{
    return _ttalink_sync_time_encode_chan(dst_addr, src_addr, chan, msg, sync_time, false);
}

/**
 * @brief Encode a sync_time struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_time C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_time_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_time_t* sync_time)
{
    return _ttalink_sync_time_encode_chan(dst_addr, src_addr, chan, msg, sync_time, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sync_time_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint64_t sync_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_TIME_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint64_t(buf, 4, sync_time);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_TIME, buf, TTALINK_MSG_ID_SYNC_TIME_MIN_LEN, TTALINK_MSG_ID_SYNC_TIME_LEN, TTALINK_MSG_ID_SYNC_TIME_CRC, nocrc);
#else
    ttalink_sync_time_t packet;
    packet.simple_time = simple_time;
    packet.sync_time = sync_time;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_TIME, (const char *)&packet, TTALINK_MSG_ID_SYNC_TIME_MIN_LEN, TTALINK_MSG_ID_SYNC_TIME_LEN, TTALINK_MSG_ID_SYNC_TIME_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_time message
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  simple_time
 * @param sync_time  sync_time
 */
static inline void ttalink_sync_time_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint64_t sync_time)
{
    _ttalink_sync_time_send(dst_addr, src_addr, chan, simple_time, sync_time, false);
}

/**
 * @brief Send a sync_time message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  simple_time
 * @param sync_time  sync_time
 */
static inline void ttalink_sync_time_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint64_t sync_time)
{
    _ttalink_sync_time_send(dst_addr, src_addr, chan, simple_time, sync_time, true);
}


static inline void _ttalink_sync_time_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_time_t* sync_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sync_time_send_nocrc(dst_addr, src_addr, chan, sync_time->simple_time, sync_time->sync_time);
    }else{
        ttalink_sync_time_send(dst_addr, src_addr, chan, sync_time->simple_time, sync_time->sync_time);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_TIME, (const char *)sync_time, TTALINK_MSG_ID_SYNC_TIME_MIN_LEN, TTALINK_MSG_ID_SYNC_TIME_LEN, TTALINK_MSG_ID_SYNC_TIME_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_time message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_time_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_time_t* sync_time)
{
    _ttalink_sync_time_send_struct(dst_addr, src_addr, chan, sync_time, false);
}

/**
 * @brief Send a sync_time message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_time_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_time_t* sync_time)
{
    _ttalink_sync_time_send_struct(dst_addr, src_addr, chan, sync_time, true);
}

#if TTALINK_MSG_ID_SYNC_TIME_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sync_time_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint64_t sync_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint64_t(buf, 4, sync_time);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_TIME, buf, TTALINK_MSG_ID_SYNC_TIME_MIN_LEN, TTALINK_MSG_ID_SYNC_TIME_LEN, TTALINK_MSG_ID_SYNC_TIME_CRC, nocrc);
#else
    ttalink_sync_time_t *packet = (ttalink_sync_time_t *)msgbuf;
    packet->simple_time = simple_time;
    packet->sync_time = sync_time;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_TIME, (const char *)packet, TTALINK_MSG_ID_SYNC_TIME_MIN_LEN, TTALINK_MSG_ID_SYNC_TIME_LEN, TTALINK_MSG_ID_SYNC_TIME_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sync_time_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint64_t sync_time)
{
    _ttalink_sync_time_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, sync_time, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sync_time_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint64_t sync_time)
{
    _ttalink_sync_time_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, sync_time, true);
}
#endif

#endif

// MESSAGE SYNC_TIME UNPACKING


/**
 * @brief Get field simple_time from sync_time message
 *
 * @return  simple_time
 */
static inline uint32_t ttalink_sync_time_get_simple_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field sync_time from sync_time message
 *
 * @return  sync_time
 */
static inline uint64_t ttalink_sync_time_get_sync_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint64_t(msg,  4);
}

/**
 * @brief Decode a sync_time message into a struct
 *
 * @param msg The message to decode
 * @param sync_time C-struct to decode the message contents into
 */
static inline void ttalink_sync_time_decode(const ttalink_message_t* msg, ttalink_sync_time_t* sync_time)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sync_time->simple_time = ttalink_sync_time_get_simple_time(msg);
    sync_time->sync_time = ttalink_sync_time_get_sync_time(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SYNC_TIME_LEN? msg->len : TTALINK_MSG_ID_SYNC_TIME_LEN;
        memset(sync_time, 0, TTALINK_MSG_ID_SYNC_TIME_LEN);
    memcpy(sync_time, _TTA_PAYLOAD(msg), len);
#endif
}
