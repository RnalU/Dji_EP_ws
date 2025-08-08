#pragma once
// MESSAGE SYNC_FAULT_INFO PACKING

#define TTALINK_MSG_ID_SYNC_FAULT_INFO 2103

TTAPACKED(
typedef struct __ttalink_sync_fault_info_t {
 uint8_t last_crash_flag; /*<  . .*/
}) ttalink_sync_fault_info_t;

#define TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN 1
#define TTALINK_MSG_ID_SYNC_FAULT_INFO_MIN_LEN 1
#define TTALINK_MSG_ID_2103_LEN 1
#define TTALINK_MSG_ID_2103_MIN_LEN 1

#define TTALINK_MSG_ID_SYNC_FAULT_INFO_CRC 83
#define TTALINK_MSG_ID_2103_CRC 83



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SYNC_FAULT_INFO { \
    2103, \
    "SYNC_FAULT_INFO", \
    1, \
    {  { "last_crash_flag", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_sync_fault_info_t, last_crash_flag) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SYNC_FAULT_INFO { \
    "SYNC_FAULT_INFO", \
    1, \
    {  { "last_crash_flag", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_sync_fault_info_t, last_crash_flag) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sync_fault_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t last_crash_flag, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN];
    _tta_put_uint8_t(buf, 0, last_crash_flag);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN);
#else
    ttalink_sync_fault_info_t packet;
    packet.last_crash_flag = last_crash_flag;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_FAULT_INFO;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SYNC_FAULT_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN, TTALINK_MSG_ID_SYNC_FAULT_INFO_CRC, nocrc);
}

/**
 * @brief Pack a sync_fault_info message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param last_crash_flag  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_fault_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t last_crash_flag)
{
    return _ttalink_sync_fault_info_pack(dst_addr, src_addr, msg,  last_crash_flag, false);
}

/**
 * @brief Pack a sync_fault_info message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param last_crash_flag  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_fault_info_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t last_crash_flag)
{
    return _ttalink_sync_fault_info_pack(dst_addr, src_addr, msg,  last_crash_flag, true);
}


static inline uint16_t _ttalink_sync_fault_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t last_crash_flag, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN];
    _tta_put_uint8_t(buf, 0, last_crash_flag);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN);
#else
    ttalink_sync_fault_info_t packet;
    packet.last_crash_flag = last_crash_flag;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_FAULT_INFO;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_FAULT_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN, TTALINK_MSG_ID_SYNC_FAULT_INFO_CRC, nocrc);
}

/**
 * @brief Pack a sync_fault_info message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param last_crash_flag  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_fault_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t last_crash_flag)
{
    return _ttalink_sync_fault_info_pack_chan(dst_addr, src_addr, chan, msg,  last_crash_flag, false);
}

/**
 * @brief Pack a sync_fault_info message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param last_crash_flag  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_fault_info_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t last_crash_flag)
{
    return _ttalink_sync_fault_info_pack_chan(dst_addr, src_addr, chan, msg,  last_crash_flag, true);
}


static inline uint16_t _ttalink_sync_fault_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_fault_info_t* sync_fault_info, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_fault_info_pack_nocrc(dst_addr, src_addr, msg, sync_fault_info->last_crash_flag);
    }else{
        return ttalink_sync_fault_info_pack(dst_addr, src_addr, msg, sync_fault_info->last_crash_flag);
    }
    
}

/**
 * @brief Encode a sync_fault_info struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_fault_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_fault_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_fault_info_t* sync_fault_info)
{
    return _ttalink_sync_fault_info_encode(dst_addr, src_addr, msg, sync_fault_info, false);
}

/**
 * @brief Encode a sync_fault_info struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_fault_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_fault_info_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_fault_info_t* sync_fault_info)
{
    return _ttalink_sync_fault_info_encode(dst_addr, src_addr, msg, sync_fault_info, true);
}


static inline uint16_t _ttalink_sync_fault_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_fault_info_t* sync_fault_info, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_fault_info_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sync_fault_info->last_crash_flag);
    }else{
        return ttalink_sync_fault_info_pack_chan(dst_addr, src_addr, chan, msg, sync_fault_info->last_crash_flag);
    }
}

/**
 * @brief Encode a sync_fault_info struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_fault_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_fault_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_fault_info_t* sync_fault_info)
{
    return _ttalink_sync_fault_info_encode_chan(dst_addr, src_addr, chan, msg, sync_fault_info, false);
}

/**
 * @brief Encode a sync_fault_info struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_fault_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_fault_info_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_fault_info_t* sync_fault_info)
{
    return _ttalink_sync_fault_info_encode_chan(dst_addr, src_addr, chan, msg, sync_fault_info, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sync_fault_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t last_crash_flag, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN];
    _tta_put_uint8_t(buf, 0, last_crash_flag);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_FAULT_INFO, buf, TTALINK_MSG_ID_SYNC_FAULT_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN, TTALINK_MSG_ID_SYNC_FAULT_INFO_CRC, nocrc);
#else
    ttalink_sync_fault_info_t packet;
    packet.last_crash_flag = last_crash_flag;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_FAULT_INFO, (const char *)&packet, TTALINK_MSG_ID_SYNC_FAULT_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN, TTALINK_MSG_ID_SYNC_FAULT_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_fault_info message
 * @param chan TTAlink channel to send the message
 *
 * @param last_crash_flag  . .
 */
static inline void ttalink_sync_fault_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t last_crash_flag)
{
    _ttalink_sync_fault_info_send(dst_addr, src_addr, chan, last_crash_flag, false);
}

/**
 * @brief Send a sync_fault_info message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param last_crash_flag  . .
 */
static inline void ttalink_sync_fault_info_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t last_crash_flag)
{
    _ttalink_sync_fault_info_send(dst_addr, src_addr, chan, last_crash_flag, true);
}


static inline void _ttalink_sync_fault_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_fault_info_t* sync_fault_info, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sync_fault_info_send_nocrc(dst_addr, src_addr, chan, sync_fault_info->last_crash_flag);
    }else{
        ttalink_sync_fault_info_send(dst_addr, src_addr, chan, sync_fault_info->last_crash_flag);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_FAULT_INFO, (const char *)sync_fault_info, TTALINK_MSG_ID_SYNC_FAULT_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN, TTALINK_MSG_ID_SYNC_FAULT_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_fault_info message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_fault_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_fault_info_t* sync_fault_info)
{
    _ttalink_sync_fault_info_send_struct(dst_addr, src_addr, chan, sync_fault_info, false);
}

/**
 * @brief Send a sync_fault_info message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_fault_info_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_fault_info_t* sync_fault_info)
{
    _ttalink_sync_fault_info_send_struct(dst_addr, src_addr, chan, sync_fault_info, true);
}

#if TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sync_fault_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t last_crash_flag, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, last_crash_flag);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_FAULT_INFO, buf, TTALINK_MSG_ID_SYNC_FAULT_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN, TTALINK_MSG_ID_SYNC_FAULT_INFO_CRC, nocrc);
#else
    ttalink_sync_fault_info_t *packet = (ttalink_sync_fault_info_t *)msgbuf;
    packet->last_crash_flag = last_crash_flag;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_FAULT_INFO, (const char *)packet, TTALINK_MSG_ID_SYNC_FAULT_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN, TTALINK_MSG_ID_SYNC_FAULT_INFO_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sync_fault_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t last_crash_flag)
{
    _ttalink_sync_fault_info_send_buf(dst_addr, src_addr, msgbuf, chan, last_crash_flag, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sync_fault_info_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t last_crash_flag)
{
    _ttalink_sync_fault_info_send_buf(dst_addr, src_addr, msgbuf, chan, last_crash_flag, true);
}
#endif

#endif

// MESSAGE SYNC_FAULT_INFO UNPACKING


/**
 * @brief Get field last_crash_flag from sync_fault_info message
 *
 * @return  . .
 */
static inline uint8_t ttalink_sync_fault_info_get_last_crash_flag(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a sync_fault_info message into a struct
 *
 * @param msg The message to decode
 * @param sync_fault_info C-struct to decode the message contents into
 */
static inline void ttalink_sync_fault_info_decode(const ttalink_message_t* msg, ttalink_sync_fault_info_t* sync_fault_info)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sync_fault_info->last_crash_flag = ttalink_sync_fault_info_get_last_crash_flag(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN? msg->len : TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN;
        memset(sync_fault_info, 0, TTALINK_MSG_ID_SYNC_FAULT_INFO_LEN);
    memcpy(sync_fault_info, _TTA_PAYLOAD(msg), len);
#endif
}
