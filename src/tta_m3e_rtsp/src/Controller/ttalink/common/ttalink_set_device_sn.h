#pragma once
// MESSAGE SET_DEVICE_SN PACKING

#define TTALINK_MSG_ID_SET_DEVICE_SN 2006

TTAPACKED(
typedef struct __ttalink_set_device_sn_t {
 uint32_t update_time; /*<  更新时间*/
 uint8_t cmd_rw; /*<  读取标志  0：read  1:write*/
 int64_t sn; /*<  长度14位十进制表示*/
 uint32_t reserve; /*< 预留.*/
}) ttalink_set_device_sn_t;

#define TTALINK_MSG_ID_SET_DEVICE_SN_LEN 17
#define TTALINK_MSG_ID_SET_DEVICE_SN_MIN_LEN 17
#define TTALINK_MSG_ID_2006_LEN 17
#define TTALINK_MSG_ID_2006_MIN_LEN 17

#define TTALINK_MSG_ID_SET_DEVICE_SN_CRC 161
#define TTALINK_MSG_ID_2006_CRC 161



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SET_DEVICE_SN { \
    2006, \
    "SET_DEVICE_SN", \
    4, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_set_device_sn_t, update_time) }, \
         { "cmd_rw", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_set_device_sn_t, cmd_rw) }, \
         { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 5, offsetof(ttalink_set_device_sn_t, sn) }, \
         { "reserve", NULL, TTALINK_TYPE_UINT32_T, 0, 13, offsetof(ttalink_set_device_sn_t, reserve) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SET_DEVICE_SN { \
    "SET_DEVICE_SN", \
    4, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_set_device_sn_t, update_time) }, \
         { "cmd_rw", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_set_device_sn_t, cmd_rw) }, \
         { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 5, offsetof(ttalink_set_device_sn_t, sn) }, \
         { "reserve", NULL, TTALINK_TYPE_UINT32_T, 0, 13, offsetof(ttalink_set_device_sn_t, reserve) }, \
         } \
}
#endif


static inline uint16_t _ttalink_set_device_sn_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd_rw, int64_t sn, uint32_t reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_DEVICE_SN_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd_rw);
    _tta_put_int64_t(buf, 5, sn);
    _tta_put_uint32_t(buf, 13, reserve);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SET_DEVICE_SN_LEN);
#else
    ttalink_set_device_sn_t packet;
    packet.update_time = update_time;
    packet.cmd_rw = cmd_rw;
    packet.sn = sn;
    packet.reserve = reserve;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SET_DEVICE_SN_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SET_DEVICE_SN;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SET_DEVICE_SN_MIN_LEN, TTALINK_MSG_ID_SET_DEVICE_SN_LEN, TTALINK_MSG_ID_SET_DEVICE_SN_CRC, nocrc);
}

/**
 * @brief Pack a set_device_sn message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  更新时间
 * @param cmd_rw  读取标志  0：read  1:write
 * @param sn  长度14位十进制表示
 * @param reserve 预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_device_sn_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd_rw, int64_t sn, uint32_t reserve)
{
    return _ttalink_set_device_sn_pack(dst_addr, src_addr, msg,  update_time, cmd_rw, sn, reserve, false);
}

/**
 * @brief Pack a set_device_sn message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  更新时间
 * @param cmd_rw  读取标志  0：read  1:write
 * @param sn  长度14位十进制表示
 * @param reserve 预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_device_sn_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd_rw, int64_t sn, uint32_t reserve)
{
    return _ttalink_set_device_sn_pack(dst_addr, src_addr, msg,  update_time, cmd_rw, sn, reserve, true);
}


static inline uint16_t _ttalink_set_device_sn_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd_rw,int64_t sn,uint32_t reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_DEVICE_SN_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd_rw);
    _tta_put_int64_t(buf, 5, sn);
    _tta_put_uint32_t(buf, 13, reserve);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SET_DEVICE_SN_LEN);
#else
    ttalink_set_device_sn_t packet;
    packet.update_time = update_time;
    packet.cmd_rw = cmd_rw;
    packet.sn = sn;
    packet.reserve = reserve;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SET_DEVICE_SN_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SET_DEVICE_SN;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_DEVICE_SN_MIN_LEN, TTALINK_MSG_ID_SET_DEVICE_SN_LEN, TTALINK_MSG_ID_SET_DEVICE_SN_CRC, nocrc);
}

/**
 * @brief Pack a set_device_sn message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  更新时间
 * @param cmd_rw  读取标志  0：read  1:write
 * @param sn  长度14位十进制表示
 * @param reserve 预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_device_sn_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd_rw,int64_t sn,uint32_t reserve)
{
    return _ttalink_set_device_sn_pack_chan(dst_addr, src_addr, chan, msg,  update_time, cmd_rw, sn, reserve, false);
}

/**
 * @brief Pack a set_device_sn message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  更新时间
 * @param cmd_rw  读取标志  0：read  1:write
 * @param sn  长度14位十进制表示
 * @param reserve 预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_device_sn_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd_rw,int64_t sn,uint32_t reserve)
{
    return _ttalink_set_device_sn_pack_chan(dst_addr, src_addr, chan, msg,  update_time, cmd_rw, sn, reserve, true);
}


static inline uint16_t _ttalink_set_device_sn_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_device_sn_t* set_device_sn, bool nocrc)
{
    if(nocrc){
        return ttalink_set_device_sn_pack_nocrc(dst_addr, src_addr, msg, set_device_sn->update_time, set_device_sn->cmd_rw, set_device_sn->sn, set_device_sn->reserve);
    }else{
        return ttalink_set_device_sn_pack(dst_addr, src_addr, msg, set_device_sn->update_time, set_device_sn->cmd_rw, set_device_sn->sn, set_device_sn->reserve);
    }
    
}

/**
 * @brief Encode a set_device_sn struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param set_device_sn C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_device_sn_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_device_sn_t* set_device_sn)
{
    return _ttalink_set_device_sn_encode(dst_addr, src_addr, msg, set_device_sn, false);
}

/**
 * @brief Encode a set_device_sn struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param set_device_sn C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_device_sn_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_device_sn_t* set_device_sn)
{
    return _ttalink_set_device_sn_encode(dst_addr, src_addr, msg, set_device_sn, true);
}


static inline uint16_t _ttalink_set_device_sn_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_device_sn_t* set_device_sn, bool nocrc)
{
    if(nocrc){
        return ttalink_set_device_sn_pack_chan_nocrc(dst_addr, src_addr, chan, msg, set_device_sn->update_time, set_device_sn->cmd_rw, set_device_sn->sn, set_device_sn->reserve);
    }else{
        return ttalink_set_device_sn_pack_chan(dst_addr, src_addr, chan, msg, set_device_sn->update_time, set_device_sn->cmd_rw, set_device_sn->sn, set_device_sn->reserve);
    }
}

/**
 * @brief Encode a set_device_sn struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param set_device_sn C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_device_sn_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_device_sn_t* set_device_sn)
{
    return _ttalink_set_device_sn_encode_chan(dst_addr, src_addr, chan, msg, set_device_sn, false);
}

/**
 * @brief Encode a set_device_sn struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param set_device_sn C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_device_sn_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_device_sn_t* set_device_sn)
{
    return _ttalink_set_device_sn_encode_chan(dst_addr, src_addr, chan, msg, set_device_sn, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_set_device_sn_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd_rw, int64_t sn, uint32_t reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_DEVICE_SN_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd_rw);
    _tta_put_int64_t(buf, 5, sn);
    _tta_put_uint32_t(buf, 13, reserve);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_DEVICE_SN, buf, TTALINK_MSG_ID_SET_DEVICE_SN_MIN_LEN, TTALINK_MSG_ID_SET_DEVICE_SN_LEN, TTALINK_MSG_ID_SET_DEVICE_SN_CRC, nocrc);
#else
    ttalink_set_device_sn_t packet;
    packet.update_time = update_time;
    packet.cmd_rw = cmd_rw;
    packet.sn = sn;
    packet.reserve = reserve;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_DEVICE_SN, (const char *)&packet, TTALINK_MSG_ID_SET_DEVICE_SN_MIN_LEN, TTALINK_MSG_ID_SET_DEVICE_SN_LEN, TTALINK_MSG_ID_SET_DEVICE_SN_CRC, nocrc);
#endif
}

/**
 * @brief Send a set_device_sn message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  更新时间
 * @param cmd_rw  读取标志  0：read  1:write
 * @param sn  长度14位十进制表示
 * @param reserve 预留.
 */
static inline void ttalink_set_device_sn_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd_rw, int64_t sn, uint32_t reserve)
{
    _ttalink_set_device_sn_send(dst_addr, src_addr, chan, update_time, cmd_rw, sn, reserve, false);
}

/**
 * @brief Send a set_device_sn message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  更新时间
 * @param cmd_rw  读取标志  0：read  1:write
 * @param sn  长度14位十进制表示
 * @param reserve 预留.
 */
static inline void ttalink_set_device_sn_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd_rw, int64_t sn, uint32_t reserve)
{
    _ttalink_set_device_sn_send(dst_addr, src_addr, chan, update_time, cmd_rw, sn, reserve, true);
}


static inline void _ttalink_set_device_sn_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_device_sn_t* set_device_sn, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_set_device_sn_send_nocrc(dst_addr, src_addr, chan, set_device_sn->update_time, set_device_sn->cmd_rw, set_device_sn->sn, set_device_sn->reserve);
    }else{
        ttalink_set_device_sn_send(dst_addr, src_addr, chan, set_device_sn->update_time, set_device_sn->cmd_rw, set_device_sn->sn, set_device_sn->reserve);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_DEVICE_SN, (const char *)set_device_sn, TTALINK_MSG_ID_SET_DEVICE_SN_MIN_LEN, TTALINK_MSG_ID_SET_DEVICE_SN_LEN, TTALINK_MSG_ID_SET_DEVICE_SN_CRC, nocrc);
#endif
}

/**
 * @brief Send a set_device_sn message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_set_device_sn_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_device_sn_t* set_device_sn)
{
    _ttalink_set_device_sn_send_struct(dst_addr, src_addr, chan, set_device_sn, false);
}

/**
 * @brief Send a set_device_sn message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_set_device_sn_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_device_sn_t* set_device_sn)
{
    _ttalink_set_device_sn_send_struct(dst_addr, src_addr, chan, set_device_sn, true);
}

#if TTALINK_MSG_ID_SET_DEVICE_SN_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_set_device_sn_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd_rw, int64_t sn, uint32_t reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd_rw);
    _tta_put_int64_t(buf, 5, sn);
    _tta_put_uint32_t(buf, 13, reserve);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_DEVICE_SN, buf, TTALINK_MSG_ID_SET_DEVICE_SN_MIN_LEN, TTALINK_MSG_ID_SET_DEVICE_SN_LEN, TTALINK_MSG_ID_SET_DEVICE_SN_CRC, nocrc);
#else
    ttalink_set_device_sn_t *packet = (ttalink_set_device_sn_t *)msgbuf;
    packet->update_time = update_time;
    packet->cmd_rw = cmd_rw;
    packet->sn = sn;
    packet->reserve = reserve;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_DEVICE_SN, (const char *)packet, TTALINK_MSG_ID_SET_DEVICE_SN_MIN_LEN, TTALINK_MSG_ID_SET_DEVICE_SN_LEN, TTALINK_MSG_ID_SET_DEVICE_SN_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_set_device_sn_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd_rw, int64_t sn, uint32_t reserve)
{
    _ttalink_set_device_sn_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, cmd_rw, sn, reserve, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_set_device_sn_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd_rw, int64_t sn, uint32_t reserve)
{
    _ttalink_set_device_sn_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, cmd_rw, sn, reserve, true);
}
#endif

#endif

// MESSAGE SET_DEVICE_SN UNPACKING


/**
 * @brief Get field update_time from set_device_sn message
 *
 * @return  更新时间
 */
static inline uint32_t ttalink_set_device_sn_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field cmd_rw from set_device_sn message
 *
 * @return  读取标志  0：read  1:write
 */
static inline uint8_t ttalink_set_device_sn_get_cmd_rw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field sn from set_device_sn message
 *
 * @return  长度14位十进制表示
 */
static inline int64_t ttalink_set_device_sn_get_sn(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  5);
}

/**
 * @brief Get field reserve from set_device_sn message
 *
 * @return 预留.
 */
static inline uint32_t ttalink_set_device_sn_get_reserve(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  13);
}

/**
 * @brief Decode a set_device_sn message into a struct
 *
 * @param msg The message to decode
 * @param set_device_sn C-struct to decode the message contents into
 */
static inline void ttalink_set_device_sn_decode(const ttalink_message_t* msg, ttalink_set_device_sn_t* set_device_sn)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    set_device_sn->update_time = ttalink_set_device_sn_get_update_time(msg);
    set_device_sn->cmd_rw = ttalink_set_device_sn_get_cmd_rw(msg);
    set_device_sn->sn = ttalink_set_device_sn_get_sn(msg);
    set_device_sn->reserve = ttalink_set_device_sn_get_reserve(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SET_DEVICE_SN_LEN? msg->len : TTALINK_MSG_ID_SET_DEVICE_SN_LEN;
        memset(set_device_sn, 0, TTALINK_MSG_ID_SET_DEVICE_SN_LEN);
    memcpy(set_device_sn, _TTA_PAYLOAD(msg), len);
#endif
}
