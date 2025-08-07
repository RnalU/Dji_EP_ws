#pragma once
// MESSAGE BMS_REMAIN_TIME PACKING

#define TTALINK_MSG_ID_BMS_REMAIN_TIME 2060

TTAPACKED(
typedef struct __ttalink_bms_remain_time_t {
 uint32_t update_time; /*<  . .*/
 uint32_t remain_time; /*< BMS可用剩余时间*/
 uint32_t param1; /*< 预留*/
 uint32_t param2; /*< 预留*/
}) ttalink_bms_remain_time_t;

#define TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN 16
#define TTALINK_MSG_ID_BMS_REMAIN_TIME_MIN_LEN 16
#define TTALINK_MSG_ID_2060_LEN 16
#define TTALINK_MSG_ID_2060_MIN_LEN 16

#define TTALINK_MSG_ID_BMS_REMAIN_TIME_CRC 130
#define TTALINK_MSG_ID_2060_CRC 130



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_BMS_REMAIN_TIME { \
    2060, \
    "BMS_REMAIN_TIME", \
    4, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_bms_remain_time_t, update_time) }, \
         { "remain_time", NULL, TTALINK_TYPE_UINT32_T, 0, 4, offsetof(ttalink_bms_remain_time_t, remain_time) }, \
         { "param1", NULL, TTALINK_TYPE_UINT32_T, 0, 8, offsetof(ttalink_bms_remain_time_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_UINT32_T, 0, 12, offsetof(ttalink_bms_remain_time_t, param2) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_BMS_REMAIN_TIME { \
    "BMS_REMAIN_TIME", \
    4, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_bms_remain_time_t, update_time) }, \
         { "remain_time", NULL, TTALINK_TYPE_UINT32_T, 0, 4, offsetof(ttalink_bms_remain_time_t, remain_time) }, \
         { "param1", NULL, TTALINK_TYPE_UINT32_T, 0, 8, offsetof(ttalink_bms_remain_time_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_UINT32_T, 0, 12, offsetof(ttalink_bms_remain_time_t, param2) }, \
         } \
}
#endif


static inline uint16_t _ttalink_bms_remain_time_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t remain_time, uint32_t param1, uint32_t param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, remain_time);
    _tta_put_uint32_t(buf, 8, param1);
    _tta_put_uint32_t(buf, 12, param2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN);
#else
    ttalink_bms_remain_time_t packet;
    packet.update_time = update_time;
    packet.remain_time = remain_time;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_BMS_REMAIN_TIME;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_BMS_REMAIN_TIME_MIN_LEN, TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN, TTALINK_MSG_ID_BMS_REMAIN_TIME_CRC, nocrc);
}

/**
 * @brief Pack a bms_remain_time message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  . .
 * @param remain_time BMS可用剩余时间
 * @param param1 预留
 * @param param2 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_remain_time_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t remain_time, uint32_t param1, uint32_t param2)
{
    return _ttalink_bms_remain_time_pack(dst_addr, src_addr, msg,  update_time, remain_time, param1, param2, false);
}

/**
 * @brief Pack a bms_remain_time message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  . .
 * @param remain_time BMS可用剩余时间
 * @param param1 预留
 * @param param2 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_remain_time_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t remain_time, uint32_t param1, uint32_t param2)
{
    return _ttalink_bms_remain_time_pack(dst_addr, src_addr, msg,  update_time, remain_time, param1, param2, true);
}


static inline uint16_t _ttalink_bms_remain_time_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t remain_time,uint32_t param1,uint32_t param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, remain_time);
    _tta_put_uint32_t(buf, 8, param1);
    _tta_put_uint32_t(buf, 12, param2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN);
#else
    ttalink_bms_remain_time_t packet;
    packet.update_time = update_time;
    packet.remain_time = remain_time;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_BMS_REMAIN_TIME;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_REMAIN_TIME_MIN_LEN, TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN, TTALINK_MSG_ID_BMS_REMAIN_TIME_CRC, nocrc);
}

/**
 * @brief Pack a bms_remain_time message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  . .
 * @param remain_time BMS可用剩余时间
 * @param param1 预留
 * @param param2 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_remain_time_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t remain_time,uint32_t param1,uint32_t param2)
{
    return _ttalink_bms_remain_time_pack_chan(dst_addr, src_addr, chan, msg,  update_time, remain_time, param1, param2, false);
}

/**
 * @brief Pack a bms_remain_time message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  . .
 * @param remain_time BMS可用剩余时间
 * @param param1 预留
 * @param param2 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_remain_time_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t remain_time,uint32_t param1,uint32_t param2)
{
    return _ttalink_bms_remain_time_pack_chan(dst_addr, src_addr, chan, msg,  update_time, remain_time, param1, param2, true);
}


static inline uint16_t _ttalink_bms_remain_time_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_bms_remain_time_t* bms_remain_time, bool nocrc)
{
    if(nocrc){
        return ttalink_bms_remain_time_pack_nocrc(dst_addr, src_addr, msg, bms_remain_time->update_time, bms_remain_time->remain_time, bms_remain_time->param1, bms_remain_time->param2);
    }else{
        return ttalink_bms_remain_time_pack(dst_addr, src_addr, msg, bms_remain_time->update_time, bms_remain_time->remain_time, bms_remain_time->param1, bms_remain_time->param2);
    }
    
}

/**
 * @brief Encode a bms_remain_time struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param bms_remain_time C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_remain_time_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_bms_remain_time_t* bms_remain_time)
{
    return _ttalink_bms_remain_time_encode(dst_addr, src_addr, msg, bms_remain_time, false);
}

/**
 * @brief Encode a bms_remain_time struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param bms_remain_time C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_remain_time_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_bms_remain_time_t* bms_remain_time)
{
    return _ttalink_bms_remain_time_encode(dst_addr, src_addr, msg, bms_remain_time, true);
}


static inline uint16_t _ttalink_bms_remain_time_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_bms_remain_time_t* bms_remain_time, bool nocrc)
{
    if(nocrc){
        return ttalink_bms_remain_time_pack_chan_nocrc(dst_addr, src_addr, chan, msg, bms_remain_time->update_time, bms_remain_time->remain_time, bms_remain_time->param1, bms_remain_time->param2);
    }else{
        return ttalink_bms_remain_time_pack_chan(dst_addr, src_addr, chan, msg, bms_remain_time->update_time, bms_remain_time->remain_time, bms_remain_time->param1, bms_remain_time->param2);
    }
}

/**
 * @brief Encode a bms_remain_time struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param bms_remain_time C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_remain_time_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_bms_remain_time_t* bms_remain_time)
{
    return _ttalink_bms_remain_time_encode_chan(dst_addr, src_addr, chan, msg, bms_remain_time, false);
}

/**
 * @brief Encode a bms_remain_time struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param bms_remain_time C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_remain_time_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_bms_remain_time_t* bms_remain_time)
{
    return _ttalink_bms_remain_time_encode_chan(dst_addr, src_addr, chan, msg, bms_remain_time, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_bms_remain_time_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t remain_time, uint32_t param1, uint32_t param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, remain_time);
    _tta_put_uint32_t(buf, 8, param1);
    _tta_put_uint32_t(buf, 12, param2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_REMAIN_TIME, buf, TTALINK_MSG_ID_BMS_REMAIN_TIME_MIN_LEN, TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN, TTALINK_MSG_ID_BMS_REMAIN_TIME_CRC, nocrc);
#else
    ttalink_bms_remain_time_t packet;
    packet.update_time = update_time;
    packet.remain_time = remain_time;
    packet.param1 = param1;
    packet.param2 = param2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_REMAIN_TIME, (const char *)&packet, TTALINK_MSG_ID_BMS_REMAIN_TIME_MIN_LEN, TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN, TTALINK_MSG_ID_BMS_REMAIN_TIME_CRC, nocrc);
#endif
}

/**
 * @brief Send a bms_remain_time message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  . .
 * @param remain_time BMS可用剩余时间
 * @param param1 预留
 * @param param2 预留
 */
static inline void ttalink_bms_remain_time_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t remain_time, uint32_t param1, uint32_t param2)
{
    _ttalink_bms_remain_time_send(dst_addr, src_addr, chan, update_time, remain_time, param1, param2, false);
}

/**
 * @brief Send a bms_remain_time message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  . .
 * @param remain_time BMS可用剩余时间
 * @param param1 预留
 * @param param2 预留
 */
static inline void ttalink_bms_remain_time_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t remain_time, uint32_t param1, uint32_t param2)
{
    _ttalink_bms_remain_time_send(dst_addr, src_addr, chan, update_time, remain_time, param1, param2, true);
}


static inline void _ttalink_bms_remain_time_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_bms_remain_time_t* bms_remain_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_bms_remain_time_send_nocrc(dst_addr, src_addr, chan, bms_remain_time->update_time, bms_remain_time->remain_time, bms_remain_time->param1, bms_remain_time->param2);
    }else{
        ttalink_bms_remain_time_send(dst_addr, src_addr, chan, bms_remain_time->update_time, bms_remain_time->remain_time, bms_remain_time->param1, bms_remain_time->param2);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_REMAIN_TIME, (const char *)bms_remain_time, TTALINK_MSG_ID_BMS_REMAIN_TIME_MIN_LEN, TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN, TTALINK_MSG_ID_BMS_REMAIN_TIME_CRC, nocrc);
#endif
}

/**
 * @brief Send a bms_remain_time message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_bms_remain_time_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_bms_remain_time_t* bms_remain_time)
{
    _ttalink_bms_remain_time_send_struct(dst_addr, src_addr, chan, bms_remain_time, false);
}

/**
 * @brief Send a bms_remain_time message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_bms_remain_time_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_bms_remain_time_t* bms_remain_time)
{
    _ttalink_bms_remain_time_send_struct(dst_addr, src_addr, chan, bms_remain_time, true);
}

#if TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_bms_remain_time_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t remain_time, uint32_t param1, uint32_t param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, remain_time);
    _tta_put_uint32_t(buf, 8, param1);
    _tta_put_uint32_t(buf, 12, param2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_REMAIN_TIME, buf, TTALINK_MSG_ID_BMS_REMAIN_TIME_MIN_LEN, TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN, TTALINK_MSG_ID_BMS_REMAIN_TIME_CRC, nocrc);
#else
    ttalink_bms_remain_time_t *packet = (ttalink_bms_remain_time_t *)msgbuf;
    packet->update_time = update_time;
    packet->remain_time = remain_time;
    packet->param1 = param1;
    packet->param2 = param2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_REMAIN_TIME, (const char *)packet, TTALINK_MSG_ID_BMS_REMAIN_TIME_MIN_LEN, TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN, TTALINK_MSG_ID_BMS_REMAIN_TIME_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_bms_remain_time_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t remain_time, uint32_t param1, uint32_t param2)
{
    _ttalink_bms_remain_time_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, remain_time, param1, param2, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_bms_remain_time_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t remain_time, uint32_t param1, uint32_t param2)
{
    _ttalink_bms_remain_time_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, remain_time, param1, param2, true);
}
#endif

#endif

// MESSAGE BMS_REMAIN_TIME UNPACKING


/**
 * @brief Get field update_time from bms_remain_time message
 *
 * @return  . .
 */
static inline uint32_t ttalink_bms_remain_time_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field remain_time from bms_remain_time message
 *
 * @return BMS可用剩余时间
 */
static inline uint32_t ttalink_bms_remain_time_get_remain_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field param1 from bms_remain_time message
 *
 * @return 预留
 */
static inline uint32_t ttalink_bms_remain_time_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field param2 from bms_remain_time message
 *
 * @return 预留
 */
static inline uint32_t ttalink_bms_remain_time_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Decode a bms_remain_time message into a struct
 *
 * @param msg The message to decode
 * @param bms_remain_time C-struct to decode the message contents into
 */
static inline void ttalink_bms_remain_time_decode(const ttalink_message_t* msg, ttalink_bms_remain_time_t* bms_remain_time)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    bms_remain_time->update_time = ttalink_bms_remain_time_get_update_time(msg);
    bms_remain_time->remain_time = ttalink_bms_remain_time_get_remain_time(msg);
    bms_remain_time->param1 = ttalink_bms_remain_time_get_param1(msg);
    bms_remain_time->param2 = ttalink_bms_remain_time_get_param2(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN? msg->len : TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN;
        memset(bms_remain_time, 0, TTALINK_MSG_ID_BMS_REMAIN_TIME_LEN);
    memcpy(bms_remain_time, _TTA_PAYLOAD(msg), len);
#endif
}
