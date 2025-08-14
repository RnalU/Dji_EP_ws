#pragma once
// MESSAGE MULTI_THROW_HOOK_ACK PACKING

#define TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK 2271

TTAPACKED(
typedef struct __ttalink_multi_throw_hook_ack_t {
 uint32_t update_time; /*<  update_time*/
 uint8_t ack; /*< ACK*/
 float param[5]; /*< 预留*/
}) ttalink_multi_throw_hook_ack_t;

#define TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN 25
#define TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_MIN_LEN 25
#define TTALINK_MSG_ID_2271_LEN 25
#define TTALINK_MSG_ID_2271_MIN_LEN 25

#define TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_CRC 110
#define TTALINK_MSG_ID_2271_CRC 110

#define TTALINK_MSG_MULTI_THROW_HOOK_ACK_FIELD_PARAM_LEN 5

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_MULTI_THROW_HOOK_ACK { \
    2271, \
    "MULTI_THROW_HOOK_ACK", \
    3, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_multi_throw_hook_ack_t, update_time) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_multi_throw_hook_ack_t, ack) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 5, 5, offsetof(ttalink_multi_throw_hook_ack_t, param) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_MULTI_THROW_HOOK_ACK { \
    "MULTI_THROW_HOOK_ACK", \
    3, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_multi_throw_hook_ack_t, update_time) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_multi_throw_hook_ack_t, ack) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 5, 5, offsetof(ttalink_multi_throw_hook_ack_t, param) }, \
         } \
}
#endif


static inline uint16_t _ttalink_multi_throw_hook_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t ack, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, ack);
    _tta_put_float_array(buf, 5, param, 5);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN);
#else
    ttalink_multi_throw_hook_ack_t packet;
    packet.update_time = update_time;
    packet.ack = ack;
    tta_array_memcpy(packet.param, param, sizeof(float)*5);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_CRC, nocrc);
}

/**
 * @brief Pack a multi_throw_hook_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param ack ACK
 * @param param 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_multi_throw_hook_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t ack, const float *param)
{
    return _ttalink_multi_throw_hook_ack_pack(dst_addr, src_addr, msg,  update_time, ack, param, false);
}

/**
 * @brief Pack a multi_throw_hook_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param ack ACK
 * @param param 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_multi_throw_hook_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t ack, const float *param)
{
    return _ttalink_multi_throw_hook_ack_pack(dst_addr, src_addr, msg,  update_time, ack, param, true);
}


static inline uint16_t _ttalink_multi_throw_hook_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t ack,const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, ack);
    _tta_put_float_array(buf, 5, param, 5);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN);
#else
    ttalink_multi_throw_hook_ack_t packet;
    packet.update_time = update_time;
    packet.ack = ack;
    tta_array_memcpy(packet.param, param, sizeof(float)*5);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_CRC, nocrc);
}

/**
 * @brief Pack a multi_throw_hook_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param ack ACK
 * @param param 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_multi_throw_hook_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t ack,const float *param)
{
    return _ttalink_multi_throw_hook_ack_pack_chan(dst_addr, src_addr, chan, msg,  update_time, ack, param, false);
}

/**
 * @brief Pack a multi_throw_hook_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param ack ACK
 * @param param 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_multi_throw_hook_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t ack,const float *param)
{
    return _ttalink_multi_throw_hook_ack_pack_chan(dst_addr, src_addr, chan, msg,  update_time, ack, param, true);
}


static inline uint16_t _ttalink_multi_throw_hook_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_multi_throw_hook_ack_t* multi_throw_hook_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_multi_throw_hook_ack_pack_nocrc(dst_addr, src_addr, msg, multi_throw_hook_ack->update_time, multi_throw_hook_ack->ack, multi_throw_hook_ack->param);
    }else{
        return ttalink_multi_throw_hook_ack_pack(dst_addr, src_addr, msg, multi_throw_hook_ack->update_time, multi_throw_hook_ack->ack, multi_throw_hook_ack->param);
    }
    
}

/**
 * @brief Encode a multi_throw_hook_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param multi_throw_hook_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_multi_throw_hook_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_multi_throw_hook_ack_t* multi_throw_hook_ack)
{
    return _ttalink_multi_throw_hook_ack_encode(dst_addr, src_addr, msg, multi_throw_hook_ack, false);
}

/**
 * @brief Encode a multi_throw_hook_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param multi_throw_hook_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_multi_throw_hook_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_multi_throw_hook_ack_t* multi_throw_hook_ack)
{
    return _ttalink_multi_throw_hook_ack_encode(dst_addr, src_addr, msg, multi_throw_hook_ack, true);
}


static inline uint16_t _ttalink_multi_throw_hook_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_multi_throw_hook_ack_t* multi_throw_hook_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_multi_throw_hook_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, multi_throw_hook_ack->update_time, multi_throw_hook_ack->ack, multi_throw_hook_ack->param);
    }else{
        return ttalink_multi_throw_hook_ack_pack_chan(dst_addr, src_addr, chan, msg, multi_throw_hook_ack->update_time, multi_throw_hook_ack->ack, multi_throw_hook_ack->param);
    }
}

/**
 * @brief Encode a multi_throw_hook_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param multi_throw_hook_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_multi_throw_hook_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_multi_throw_hook_ack_t* multi_throw_hook_ack)
{
    return _ttalink_multi_throw_hook_ack_encode_chan(dst_addr, src_addr, chan, msg, multi_throw_hook_ack, false);
}

/**
 * @brief Encode a multi_throw_hook_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param multi_throw_hook_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_multi_throw_hook_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_multi_throw_hook_ack_t* multi_throw_hook_ack)
{
    return _ttalink_multi_throw_hook_ack_encode_chan(dst_addr, src_addr, chan, msg, multi_throw_hook_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_multi_throw_hook_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t ack, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, ack);
    _tta_put_float_array(buf, 5, param, 5);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK, buf, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_CRC, nocrc);
#else
    ttalink_multi_throw_hook_ack_t packet;
    packet.update_time = update_time;
    packet.ack = ack;
    tta_array_memcpy(packet.param, param, sizeof(float)*5);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK, (const char *)&packet, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a multi_throw_hook_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param ack ACK
 * @param param 预留
 */
static inline void ttalink_multi_throw_hook_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t ack, const float *param)
{
    _ttalink_multi_throw_hook_ack_send(dst_addr, src_addr, chan, update_time, ack, param, false);
}

/**
 * @brief Send a multi_throw_hook_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param ack ACK
 * @param param 预留
 */
static inline void ttalink_multi_throw_hook_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t ack, const float *param)
{
    _ttalink_multi_throw_hook_ack_send(dst_addr, src_addr, chan, update_time, ack, param, true);
}


static inline void _ttalink_multi_throw_hook_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_multi_throw_hook_ack_t* multi_throw_hook_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_multi_throw_hook_ack_send_nocrc(dst_addr, src_addr, chan, multi_throw_hook_ack->update_time, multi_throw_hook_ack->ack, multi_throw_hook_ack->param);
    }else{
        ttalink_multi_throw_hook_ack_send(dst_addr, src_addr, chan, multi_throw_hook_ack->update_time, multi_throw_hook_ack->ack, multi_throw_hook_ack->param);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK, (const char *)multi_throw_hook_ack, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a multi_throw_hook_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_multi_throw_hook_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_multi_throw_hook_ack_t* multi_throw_hook_ack)
{
    _ttalink_multi_throw_hook_ack_send_struct(dst_addr, src_addr, chan, multi_throw_hook_ack, false);
}

/**
 * @brief Send a multi_throw_hook_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_multi_throw_hook_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_multi_throw_hook_ack_t* multi_throw_hook_ack)
{
    _ttalink_multi_throw_hook_ack_send_struct(dst_addr, src_addr, chan, multi_throw_hook_ack, true);
}

#if TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_multi_throw_hook_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t ack, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, ack);
    _tta_put_float_array(buf, 5, param, 5);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK, buf, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_CRC, nocrc);
#else
    ttalink_multi_throw_hook_ack_t *packet = (ttalink_multi_throw_hook_ack_t *)msgbuf;
    packet->update_time = update_time;
    packet->ack = ack;
    tta_array_memcpy(packet->param, param, sizeof(float)*5);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK, (const char *)packet, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_multi_throw_hook_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t ack, const float *param)
{
    _ttalink_multi_throw_hook_ack_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, ack, param, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_multi_throw_hook_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t ack, const float *param)
{
    _ttalink_multi_throw_hook_ack_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, ack, param, true);
}
#endif

#endif

// MESSAGE MULTI_THROW_HOOK_ACK UNPACKING


/**
 * @brief Get field update_time from multi_throw_hook_ack message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_multi_throw_hook_ack_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field ack from multi_throw_hook_ack message
 *
 * @return ACK
 */
static inline uint8_t ttalink_multi_throw_hook_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field param from multi_throw_hook_ack message
 *
 * @return 预留
 */
static inline uint16_t ttalink_multi_throw_hook_ack_get_param(const ttalink_message_t* msg, float *param)
{
    return _TTA_RETURN_float_array(msg, param, 5,  5);
}

/**
 * @brief Decode a multi_throw_hook_ack message into a struct
 *
 * @param msg The message to decode
 * @param multi_throw_hook_ack C-struct to decode the message contents into
 */
static inline void ttalink_multi_throw_hook_ack_decode(const ttalink_message_t* msg, ttalink_multi_throw_hook_ack_t* multi_throw_hook_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    multi_throw_hook_ack->update_time = ttalink_multi_throw_hook_ack_get_update_time(msg);
    multi_throw_hook_ack->ack = ttalink_multi_throw_hook_ack_get_ack(msg);
    ttalink_multi_throw_hook_ack_get_param(msg, multi_throw_hook_ack->param);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN? msg->len : TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN;
        memset(multi_throw_hook_ack, 0, TTALINK_MSG_ID_MULTI_THROW_HOOK_ACK_LEN);
    memcpy(multi_throw_hook_ack, _TTA_PAYLOAD(msg), len);
#endif
}
