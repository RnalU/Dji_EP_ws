#pragma once
// MESSAGE SYNC_CTRL_STATUS_INFO PACKING

#define TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO 2105

TTAPACKED(
typedef struct __ttalink_sync_ctrl_status_info_t {
 uint32_t simple_time; /*<  . .*/
 uint8_t motor_status; /*<  . .*/
 uint8_t flight_status; /*<  . .*/
 uint8_t flight_mode; /*<  . */
}) ttalink_sync_ctrl_status_info_t;

#define TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN 7
#define TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_MIN_LEN 7
#define TTALINK_MSG_ID_2105_LEN 7
#define TTALINK_MSG_ID_2105_MIN_LEN 7

#define TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_CRC 190
#define TTALINK_MSG_ID_2105_CRC 190



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SYNC_CTRL_STATUS_INFO { \
    2105, \
    "SYNC_CTRL_STATUS_INFO", \
    4, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_ctrl_status_info_t, simple_time) }, \
         { "motor_status", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_sync_ctrl_status_info_t, motor_status) }, \
         { "flight_status", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_sync_ctrl_status_info_t, flight_status) }, \
         { "flight_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_sync_ctrl_status_info_t, flight_mode) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SYNC_CTRL_STATUS_INFO { \
    "SYNC_CTRL_STATUS_INFO", \
    4, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_ctrl_status_info_t, simple_time) }, \
         { "motor_status", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_sync_ctrl_status_info_t, motor_status) }, \
         { "flight_status", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_sync_ctrl_status_info_t, flight_status) }, \
         { "flight_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_sync_ctrl_status_info_t, flight_mode) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sync_ctrl_status_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint8_t motor_status, uint8_t flight_status, uint8_t flight_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, motor_status);
    _tta_put_uint8_t(buf, 5, flight_status);
    _tta_put_uint8_t(buf, 6, flight_mode);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN);
#else
    ttalink_sync_ctrl_status_info_t packet;
    packet.simple_time = simple_time;
    packet.motor_status = motor_status;
    packet.flight_status = flight_status;
    packet.flight_mode = flight_mode;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_CRC, nocrc);
}

/**
 * @brief Pack a sync_ctrl_status_info message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  . .
 * @param motor_status  . .
 * @param flight_status  . .
 * @param flight_mode  . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_ctrl_status_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint8_t motor_status, uint8_t flight_status, uint8_t flight_mode)
{
    return _ttalink_sync_ctrl_status_info_pack(dst_addr, src_addr, msg,  simple_time, motor_status, flight_status, flight_mode, false);
}

/**
 * @brief Pack a sync_ctrl_status_info message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  . .
 * @param motor_status  . .
 * @param flight_status  . .
 * @param flight_mode  . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_ctrl_status_info_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint8_t motor_status, uint8_t flight_status, uint8_t flight_mode)
{
    return _ttalink_sync_ctrl_status_info_pack(dst_addr, src_addr, msg,  simple_time, motor_status, flight_status, flight_mode, true);
}


static inline uint16_t _ttalink_sync_ctrl_status_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint8_t motor_status,uint8_t flight_status,uint8_t flight_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, motor_status);
    _tta_put_uint8_t(buf, 5, flight_status);
    _tta_put_uint8_t(buf, 6, flight_mode);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN);
#else
    ttalink_sync_ctrl_status_info_t packet;
    packet.simple_time = simple_time;
    packet.motor_status = motor_status;
    packet.flight_status = flight_status;
    packet.flight_mode = flight_mode;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_CRC, nocrc);
}

/**
 * @brief Pack a sync_ctrl_status_info message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  . .
 * @param motor_status  . .
 * @param flight_status  . .
 * @param flight_mode  . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_ctrl_status_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint8_t motor_status,uint8_t flight_status,uint8_t flight_mode)
{
    return _ttalink_sync_ctrl_status_info_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, motor_status, flight_status, flight_mode, false);
}

/**
 * @brief Pack a sync_ctrl_status_info message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  . .
 * @param motor_status  . .
 * @param flight_status  . .
 * @param flight_mode  . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_ctrl_status_info_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint8_t motor_status,uint8_t flight_status,uint8_t flight_mode)
{
    return _ttalink_sync_ctrl_status_info_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, motor_status, flight_status, flight_mode, true);
}


static inline uint16_t _ttalink_sync_ctrl_status_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_ctrl_status_info_t* sync_ctrl_status_info, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_ctrl_status_info_pack_nocrc(dst_addr, src_addr, msg, sync_ctrl_status_info->simple_time, sync_ctrl_status_info->motor_status, sync_ctrl_status_info->flight_status, sync_ctrl_status_info->flight_mode);
    }else{
        return ttalink_sync_ctrl_status_info_pack(dst_addr, src_addr, msg, sync_ctrl_status_info->simple_time, sync_ctrl_status_info->motor_status, sync_ctrl_status_info->flight_status, sync_ctrl_status_info->flight_mode);
    }
    
}

/**
 * @brief Encode a sync_ctrl_status_info struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_ctrl_status_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_ctrl_status_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_ctrl_status_info_t* sync_ctrl_status_info)
{
    return _ttalink_sync_ctrl_status_info_encode(dst_addr, src_addr, msg, sync_ctrl_status_info, false);
}

/**
 * @brief Encode a sync_ctrl_status_info struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_ctrl_status_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_ctrl_status_info_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_ctrl_status_info_t* sync_ctrl_status_info)
{
    return _ttalink_sync_ctrl_status_info_encode(dst_addr, src_addr, msg, sync_ctrl_status_info, true);
}


static inline uint16_t _ttalink_sync_ctrl_status_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_ctrl_status_info_t* sync_ctrl_status_info, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_ctrl_status_info_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sync_ctrl_status_info->simple_time, sync_ctrl_status_info->motor_status, sync_ctrl_status_info->flight_status, sync_ctrl_status_info->flight_mode);
    }else{
        return ttalink_sync_ctrl_status_info_pack_chan(dst_addr, src_addr, chan, msg, sync_ctrl_status_info->simple_time, sync_ctrl_status_info->motor_status, sync_ctrl_status_info->flight_status, sync_ctrl_status_info->flight_mode);
    }
}

/**
 * @brief Encode a sync_ctrl_status_info struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_ctrl_status_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_ctrl_status_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_ctrl_status_info_t* sync_ctrl_status_info)
{
    return _ttalink_sync_ctrl_status_info_encode_chan(dst_addr, src_addr, chan, msg, sync_ctrl_status_info, false);
}

/**
 * @brief Encode a sync_ctrl_status_info struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_ctrl_status_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_ctrl_status_info_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_ctrl_status_info_t* sync_ctrl_status_info)
{
    return _ttalink_sync_ctrl_status_info_encode_chan(dst_addr, src_addr, chan, msg, sync_ctrl_status_info, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sync_ctrl_status_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint8_t motor_status, uint8_t flight_status, uint8_t flight_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, motor_status);
    _tta_put_uint8_t(buf, 5, flight_status);
    _tta_put_uint8_t(buf, 6, flight_mode);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO, buf, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_CRC, nocrc);
#else
    ttalink_sync_ctrl_status_info_t packet;
    packet.simple_time = simple_time;
    packet.motor_status = motor_status;
    packet.flight_status = flight_status;
    packet.flight_mode = flight_mode;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO, (const char *)&packet, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_ctrl_status_info message
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  . .
 * @param motor_status  . .
 * @param flight_status  . .
 * @param flight_mode  . 
 */
static inline void ttalink_sync_ctrl_status_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint8_t motor_status, uint8_t flight_status, uint8_t flight_mode)
{
    _ttalink_sync_ctrl_status_info_send(dst_addr, src_addr, chan, simple_time, motor_status, flight_status, flight_mode, false);
}

/**
 * @brief Send a sync_ctrl_status_info message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  . .
 * @param motor_status  . .
 * @param flight_status  . .
 * @param flight_mode  . 
 */
static inline void ttalink_sync_ctrl_status_info_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint8_t motor_status, uint8_t flight_status, uint8_t flight_mode)
{
    _ttalink_sync_ctrl_status_info_send(dst_addr, src_addr, chan, simple_time, motor_status, flight_status, flight_mode, true);
}


static inline void _ttalink_sync_ctrl_status_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_ctrl_status_info_t* sync_ctrl_status_info, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sync_ctrl_status_info_send_nocrc(dst_addr, src_addr, chan, sync_ctrl_status_info->simple_time, sync_ctrl_status_info->motor_status, sync_ctrl_status_info->flight_status, sync_ctrl_status_info->flight_mode);
    }else{
        ttalink_sync_ctrl_status_info_send(dst_addr, src_addr, chan, sync_ctrl_status_info->simple_time, sync_ctrl_status_info->motor_status, sync_ctrl_status_info->flight_status, sync_ctrl_status_info->flight_mode);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO, (const char *)sync_ctrl_status_info, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_ctrl_status_info message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_ctrl_status_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_ctrl_status_info_t* sync_ctrl_status_info)
{
    _ttalink_sync_ctrl_status_info_send_struct(dst_addr, src_addr, chan, sync_ctrl_status_info, false);
}

/**
 * @brief Send a sync_ctrl_status_info message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_ctrl_status_info_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_ctrl_status_info_t* sync_ctrl_status_info)
{
    _ttalink_sync_ctrl_status_info_send_struct(dst_addr, src_addr, chan, sync_ctrl_status_info, true);
}

#if TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sync_ctrl_status_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint8_t motor_status, uint8_t flight_status, uint8_t flight_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, motor_status);
    _tta_put_uint8_t(buf, 5, flight_status);
    _tta_put_uint8_t(buf, 6, flight_mode);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO, buf, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_CRC, nocrc);
#else
    ttalink_sync_ctrl_status_info_t *packet = (ttalink_sync_ctrl_status_info_t *)msgbuf;
    packet->simple_time = simple_time;
    packet->motor_status = motor_status;
    packet->flight_status = flight_status;
    packet->flight_mode = flight_mode;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO, (const char *)packet, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sync_ctrl_status_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint8_t motor_status, uint8_t flight_status, uint8_t flight_mode)
{
    _ttalink_sync_ctrl_status_info_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, motor_status, flight_status, flight_mode, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sync_ctrl_status_info_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint8_t motor_status, uint8_t flight_status, uint8_t flight_mode)
{
    _ttalink_sync_ctrl_status_info_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, motor_status, flight_status, flight_mode, true);
}
#endif

#endif

// MESSAGE SYNC_CTRL_STATUS_INFO UNPACKING


/**
 * @brief Get field simple_time from sync_ctrl_status_info message
 *
 * @return  . .
 */
static inline uint32_t ttalink_sync_ctrl_status_info_get_simple_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field motor_status from sync_ctrl_status_info message
 *
 * @return  . .
 */
static inline uint8_t ttalink_sync_ctrl_status_info_get_motor_status(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field flight_status from sync_ctrl_status_info message
 *
 * @return  . .
 */
static inline uint8_t ttalink_sync_ctrl_status_info_get_flight_status(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field flight_mode from sync_ctrl_status_info message
 *
 * @return  . 
 */
static inline uint8_t ttalink_sync_ctrl_status_info_get_flight_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Decode a sync_ctrl_status_info message into a struct
 *
 * @param msg The message to decode
 * @param sync_ctrl_status_info C-struct to decode the message contents into
 */
static inline void ttalink_sync_ctrl_status_info_decode(const ttalink_message_t* msg, ttalink_sync_ctrl_status_info_t* sync_ctrl_status_info)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sync_ctrl_status_info->simple_time = ttalink_sync_ctrl_status_info_get_simple_time(msg);
    sync_ctrl_status_info->motor_status = ttalink_sync_ctrl_status_info_get_motor_status(msg);
    sync_ctrl_status_info->flight_status = ttalink_sync_ctrl_status_info_get_flight_status(msg);
    sync_ctrl_status_info->flight_mode = ttalink_sync_ctrl_status_info_get_flight_mode(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN? msg->len : TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN;
        memset(sync_ctrl_status_info, 0, TTALINK_MSG_ID_SYNC_CTRL_STATUS_INFO_LEN);
    memcpy(sync_ctrl_status_info, _TTA_PAYLOAD(msg), len);
#endif
}
