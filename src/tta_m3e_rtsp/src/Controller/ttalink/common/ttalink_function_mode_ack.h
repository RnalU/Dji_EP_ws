#pragma once
// MESSAGE FUNCTION_MODE_ACK PACKING

#define TTALINK_MSG_ID_FUNCTION_MODE_ACK 2205

TTAPACKED(
typedef struct __ttalink_function_mode_ack_t {
 uint32_t function_cmd; /*<  .*/
 uint8_t ack; /*<  .*/
}) ttalink_function_mode_ack_t;

#define TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN 5
#define TTALINK_MSG_ID_FUNCTION_MODE_ACK_MIN_LEN 5
#define TTALINK_MSG_ID_2205_LEN 5
#define TTALINK_MSG_ID_2205_MIN_LEN 5

#define TTALINK_MSG_ID_FUNCTION_MODE_ACK_CRC 95
#define TTALINK_MSG_ID_2205_CRC 95



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_FUNCTION_MODE_ACK { \
    2205, \
    "FUNCTION_MODE_ACK", \
    2, \
    {  { "function_cmd", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_function_mode_ack_t, function_cmd) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_function_mode_ack_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_FUNCTION_MODE_ACK { \
    "FUNCTION_MODE_ACK", \
    2, \
    {  { "function_cmd", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_function_mode_ack_t, function_cmd) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_function_mode_ack_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_function_mode_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t function_cmd, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN];
    _tta_put_uint32_t(buf, 0, function_cmd);
    _tta_put_uint8_t(buf, 4, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN);
#else
    ttalink_function_mode_ack_t packet;
    packet.function_cmd = function_cmd;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FUNCTION_MODE_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_FUNCTION_MODE_ACK_MIN_LEN, TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN, TTALINK_MSG_ID_FUNCTION_MODE_ACK_CRC, nocrc);
}

/**
 * @brief Pack a function_mode_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param function_cmd  .
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_function_mode_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t function_cmd, uint8_t ack)
{
    return _ttalink_function_mode_ack_pack(dst_addr, src_addr, msg,  function_cmd, ack, false);
}

/**
 * @brief Pack a function_mode_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param function_cmd  .
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_function_mode_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t function_cmd, uint8_t ack)
{
    return _ttalink_function_mode_ack_pack(dst_addr, src_addr, msg,  function_cmd, ack, true);
}


static inline uint16_t _ttalink_function_mode_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t function_cmd,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN];
    _tta_put_uint32_t(buf, 0, function_cmd);
    _tta_put_uint8_t(buf, 4, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN);
#else
    ttalink_function_mode_ack_t packet;
    packet.function_cmd = function_cmd;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FUNCTION_MODE_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_FUNCTION_MODE_ACK_MIN_LEN, TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN, TTALINK_MSG_ID_FUNCTION_MODE_ACK_CRC, nocrc);
}

/**
 * @brief Pack a function_mode_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param function_cmd  .
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_function_mode_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t function_cmd,uint8_t ack)
{
    return _ttalink_function_mode_ack_pack_chan(dst_addr, src_addr, chan, msg,  function_cmd, ack, false);
}

/**
 * @brief Pack a function_mode_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param function_cmd  .
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_function_mode_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t function_cmd,uint8_t ack)
{
    return _ttalink_function_mode_ack_pack_chan(dst_addr, src_addr, chan, msg,  function_cmd, ack, true);
}


static inline uint16_t _ttalink_function_mode_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_function_mode_ack_t* function_mode_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_function_mode_ack_pack_nocrc(dst_addr, src_addr, msg, function_mode_ack->function_cmd, function_mode_ack->ack);
    }else{
        return ttalink_function_mode_ack_pack(dst_addr, src_addr, msg, function_mode_ack->function_cmd, function_mode_ack->ack);
    }
    
}

/**
 * @brief Encode a function_mode_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param function_mode_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_function_mode_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_function_mode_ack_t* function_mode_ack)
{
    return _ttalink_function_mode_ack_encode(dst_addr, src_addr, msg, function_mode_ack, false);
}

/**
 * @brief Encode a function_mode_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param function_mode_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_function_mode_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_function_mode_ack_t* function_mode_ack)
{
    return _ttalink_function_mode_ack_encode(dst_addr, src_addr, msg, function_mode_ack, true);
}


static inline uint16_t _ttalink_function_mode_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_function_mode_ack_t* function_mode_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_function_mode_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, function_mode_ack->function_cmd, function_mode_ack->ack);
    }else{
        return ttalink_function_mode_ack_pack_chan(dst_addr, src_addr, chan, msg, function_mode_ack->function_cmd, function_mode_ack->ack);
    }
}

/**
 * @brief Encode a function_mode_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param function_mode_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_function_mode_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_function_mode_ack_t* function_mode_ack)
{
    return _ttalink_function_mode_ack_encode_chan(dst_addr, src_addr, chan, msg, function_mode_ack, false);
}

/**
 * @brief Encode a function_mode_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param function_mode_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_function_mode_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_function_mode_ack_t* function_mode_ack)
{
    return _ttalink_function_mode_ack_encode_chan(dst_addr, src_addr, chan, msg, function_mode_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_function_mode_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t function_cmd, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN];
    _tta_put_uint32_t(buf, 0, function_cmd);
    _tta_put_uint8_t(buf, 4, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FUNCTION_MODE_ACK, buf, TTALINK_MSG_ID_FUNCTION_MODE_ACK_MIN_LEN, TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN, TTALINK_MSG_ID_FUNCTION_MODE_ACK_CRC, nocrc);
#else
    ttalink_function_mode_ack_t packet;
    packet.function_cmd = function_cmd;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FUNCTION_MODE_ACK, (const char *)&packet, TTALINK_MSG_ID_FUNCTION_MODE_ACK_MIN_LEN, TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN, TTALINK_MSG_ID_FUNCTION_MODE_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a function_mode_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param function_cmd  .
 * @param ack  .
 */
static inline void ttalink_function_mode_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t function_cmd, uint8_t ack)
{
    _ttalink_function_mode_ack_send(dst_addr, src_addr, chan, function_cmd, ack, false);
}

/**
 * @brief Send a function_mode_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param function_cmd  .
 * @param ack  .
 */
static inline void ttalink_function_mode_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t function_cmd, uint8_t ack)
{
    _ttalink_function_mode_ack_send(dst_addr, src_addr, chan, function_cmd, ack, true);
}


static inline void _ttalink_function_mode_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_function_mode_ack_t* function_mode_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_function_mode_ack_send_nocrc(dst_addr, src_addr, chan, function_mode_ack->function_cmd, function_mode_ack->ack);
    }else{
        ttalink_function_mode_ack_send(dst_addr, src_addr, chan, function_mode_ack->function_cmd, function_mode_ack->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FUNCTION_MODE_ACK, (const char *)function_mode_ack, TTALINK_MSG_ID_FUNCTION_MODE_ACK_MIN_LEN, TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN, TTALINK_MSG_ID_FUNCTION_MODE_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a function_mode_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_function_mode_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_function_mode_ack_t* function_mode_ack)
{
    _ttalink_function_mode_ack_send_struct(dst_addr, src_addr, chan, function_mode_ack, false);
}

/**
 * @brief Send a function_mode_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_function_mode_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_function_mode_ack_t* function_mode_ack)
{
    _ttalink_function_mode_ack_send_struct(dst_addr, src_addr, chan, function_mode_ack, true);
}

#if TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_function_mode_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t function_cmd, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, function_cmd);
    _tta_put_uint8_t(buf, 4, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FUNCTION_MODE_ACK, buf, TTALINK_MSG_ID_FUNCTION_MODE_ACK_MIN_LEN, TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN, TTALINK_MSG_ID_FUNCTION_MODE_ACK_CRC, nocrc);
#else
    ttalink_function_mode_ack_t *packet = (ttalink_function_mode_ack_t *)msgbuf;
    packet->function_cmd = function_cmd;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FUNCTION_MODE_ACK, (const char *)packet, TTALINK_MSG_ID_FUNCTION_MODE_ACK_MIN_LEN, TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN, TTALINK_MSG_ID_FUNCTION_MODE_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_function_mode_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t function_cmd, uint8_t ack)
{
    _ttalink_function_mode_ack_send_buf(dst_addr, src_addr, msgbuf, chan, function_cmd, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_function_mode_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t function_cmd, uint8_t ack)
{
    _ttalink_function_mode_ack_send_buf(dst_addr, src_addr, msgbuf, chan, function_cmd, ack, true);
}
#endif

#endif

// MESSAGE FUNCTION_MODE_ACK UNPACKING


/**
 * @brief Get field function_cmd from function_mode_ack message
 *
 * @return  .
 */
static inline uint32_t ttalink_function_mode_ack_get_function_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field ack from function_mode_ack message
 *
 * @return  .
 */
static inline uint8_t ttalink_function_mode_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Decode a function_mode_ack message into a struct
 *
 * @param msg The message to decode
 * @param function_mode_ack C-struct to decode the message contents into
 */
static inline void ttalink_function_mode_ack_decode(const ttalink_message_t* msg, ttalink_function_mode_ack_t* function_mode_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    function_mode_ack->function_cmd = ttalink_function_mode_ack_get_function_cmd(msg);
    function_mode_ack->ack = ttalink_function_mode_ack_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN? msg->len : TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN;
        memset(function_mode_ack, 0, TTALINK_MSG_ID_FUNCTION_MODE_ACK_LEN);
    memcpy(function_mode_ack, _TTA_PAYLOAD(msg), len);
#endif
}
