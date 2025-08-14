#pragma once
// MESSAGE FUNCTION_MODE PACKING

#define TTALINK_MSG_ID_FUNCTION_MODE 2204

TTAPACKED(
typedef struct __ttalink_function_mode_t {
 uint32_t function_cmd; /*< .*/
 float param; /*< 
			TTALINK_MISSION_START： 0=不执行断点  1=执行断点.
			TTALINK_MISSION_AGAIN：0:目标点 1:断点 2:垂足点
			TTALINK_MISSION_SET_VEL: value=想要变速的值
			TTALINK_MISSION_SET_ALT: value=想要变高的值*/
}) ttalink_function_mode_t;

#define TTALINK_MSG_ID_FUNCTION_MODE_LEN 8
#define TTALINK_MSG_ID_FUNCTION_MODE_MIN_LEN 8
#define TTALINK_MSG_ID_2204_LEN 8
#define TTALINK_MSG_ID_2204_MIN_LEN 8

#define TTALINK_MSG_ID_FUNCTION_MODE_CRC 128
#define TTALINK_MSG_ID_2204_CRC 128



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_FUNCTION_MODE { \
    2204, \
    "FUNCTION_MODE", \
    2, \
    {  { "function_cmd", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_function_mode_t, function_cmd) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_function_mode_t, param) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_FUNCTION_MODE { \
    "FUNCTION_MODE", \
    2, \
    {  { "function_cmd", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_function_mode_t, function_cmd) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_function_mode_t, param) }, \
         } \
}
#endif


static inline uint16_t _ttalink_function_mode_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t function_cmd, float param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FUNCTION_MODE_LEN];
    _tta_put_uint32_t(buf, 0, function_cmd);
    _tta_put_float(buf, 4, param);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FUNCTION_MODE_LEN);
#else
    ttalink_function_mode_t packet;
    packet.function_cmd = function_cmd;
    packet.param = param;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FUNCTION_MODE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FUNCTION_MODE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_FUNCTION_MODE_MIN_LEN, TTALINK_MSG_ID_FUNCTION_MODE_LEN, TTALINK_MSG_ID_FUNCTION_MODE_CRC, nocrc);
}

/**
 * @brief Pack a function_mode message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param function_cmd .
 * @param param 
			TTALINK_MISSION_START： 0=不执行断点  1=执行断点.
			TTALINK_MISSION_AGAIN：0:目标点 1:断点 2:垂足点
			TTALINK_MISSION_SET_VEL: value=想要变速的值
			TTALINK_MISSION_SET_ALT: value=想要变高的值
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_function_mode_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t function_cmd, float param)
{
    return _ttalink_function_mode_pack(dst_addr, src_addr, msg,  function_cmd, param, false);
}

/**
 * @brief Pack a function_mode message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param function_cmd .
 * @param param 
			TTALINK_MISSION_START： 0=不执行断点  1=执行断点.
			TTALINK_MISSION_AGAIN：0:目标点 1:断点 2:垂足点
			TTALINK_MISSION_SET_VEL: value=想要变速的值
			TTALINK_MISSION_SET_ALT: value=想要变高的值
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_function_mode_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t function_cmd, float param)
{
    return _ttalink_function_mode_pack(dst_addr, src_addr, msg,  function_cmd, param, true);
}


static inline uint16_t _ttalink_function_mode_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t function_cmd,float param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FUNCTION_MODE_LEN];
    _tta_put_uint32_t(buf, 0, function_cmd);
    _tta_put_float(buf, 4, param);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FUNCTION_MODE_LEN);
#else
    ttalink_function_mode_t packet;
    packet.function_cmd = function_cmd;
    packet.param = param;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FUNCTION_MODE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FUNCTION_MODE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_FUNCTION_MODE_MIN_LEN, TTALINK_MSG_ID_FUNCTION_MODE_LEN, TTALINK_MSG_ID_FUNCTION_MODE_CRC, nocrc);
}

/**
 * @brief Pack a function_mode message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param function_cmd .
 * @param param 
			TTALINK_MISSION_START： 0=不执行断点  1=执行断点.
			TTALINK_MISSION_AGAIN：0:目标点 1:断点 2:垂足点
			TTALINK_MISSION_SET_VEL: value=想要变速的值
			TTALINK_MISSION_SET_ALT: value=想要变高的值
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_function_mode_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t function_cmd,float param)
{
    return _ttalink_function_mode_pack_chan(dst_addr, src_addr, chan, msg,  function_cmd, param, false);
}

/**
 * @brief Pack a function_mode message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param function_cmd .
 * @param param 
			TTALINK_MISSION_START： 0=不执行断点  1=执行断点.
			TTALINK_MISSION_AGAIN：0:目标点 1:断点 2:垂足点
			TTALINK_MISSION_SET_VEL: value=想要变速的值
			TTALINK_MISSION_SET_ALT: value=想要变高的值
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_function_mode_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t function_cmd,float param)
{
    return _ttalink_function_mode_pack_chan(dst_addr, src_addr, chan, msg,  function_cmd, param, true);
}


static inline uint16_t _ttalink_function_mode_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_function_mode_t* function_mode, bool nocrc)
{
    if(nocrc){
        return ttalink_function_mode_pack_nocrc(dst_addr, src_addr, msg, function_mode->function_cmd, function_mode->param);
    }else{
        return ttalink_function_mode_pack(dst_addr, src_addr, msg, function_mode->function_cmd, function_mode->param);
    }
    
}

/**
 * @brief Encode a function_mode struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param function_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_function_mode_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_function_mode_t* function_mode)
{
    return _ttalink_function_mode_encode(dst_addr, src_addr, msg, function_mode, false);
}

/**
 * @brief Encode a function_mode struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param function_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_function_mode_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_function_mode_t* function_mode)
{
    return _ttalink_function_mode_encode(dst_addr, src_addr, msg, function_mode, true);
}


static inline uint16_t _ttalink_function_mode_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_function_mode_t* function_mode, bool nocrc)
{
    if(nocrc){
        return ttalink_function_mode_pack_chan_nocrc(dst_addr, src_addr, chan, msg, function_mode->function_cmd, function_mode->param);
    }else{
        return ttalink_function_mode_pack_chan(dst_addr, src_addr, chan, msg, function_mode->function_cmd, function_mode->param);
    }
}

/**
 * @brief Encode a function_mode struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param function_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_function_mode_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_function_mode_t* function_mode)
{
    return _ttalink_function_mode_encode_chan(dst_addr, src_addr, chan, msg, function_mode, false);
}

/**
 * @brief Encode a function_mode struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param function_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_function_mode_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_function_mode_t* function_mode)
{
    return _ttalink_function_mode_encode_chan(dst_addr, src_addr, chan, msg, function_mode, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_function_mode_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t function_cmd, float param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FUNCTION_MODE_LEN];
    _tta_put_uint32_t(buf, 0, function_cmd);
    _tta_put_float(buf, 4, param);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FUNCTION_MODE, buf, TTALINK_MSG_ID_FUNCTION_MODE_MIN_LEN, TTALINK_MSG_ID_FUNCTION_MODE_LEN, TTALINK_MSG_ID_FUNCTION_MODE_CRC, nocrc);
#else
    ttalink_function_mode_t packet;
    packet.function_cmd = function_cmd;
    packet.param = param;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FUNCTION_MODE, (const char *)&packet, TTALINK_MSG_ID_FUNCTION_MODE_MIN_LEN, TTALINK_MSG_ID_FUNCTION_MODE_LEN, TTALINK_MSG_ID_FUNCTION_MODE_CRC, nocrc);
#endif
}

/**
 * @brief Send a function_mode message
 * @param chan TTAlink channel to send the message
 *
 * @param function_cmd .
 * @param param 
			TTALINK_MISSION_START： 0=不执行断点  1=执行断点.
			TTALINK_MISSION_AGAIN：0:目标点 1:断点 2:垂足点
			TTALINK_MISSION_SET_VEL: value=想要变速的值
			TTALINK_MISSION_SET_ALT: value=想要变高的值
 */
static inline void ttalink_function_mode_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t function_cmd, float param)
{
    _ttalink_function_mode_send(dst_addr, src_addr, chan, function_cmd, param, false);
}

/**
 * @brief Send a function_mode message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param function_cmd .
 * @param param 
			TTALINK_MISSION_START： 0=不执行断点  1=执行断点.
			TTALINK_MISSION_AGAIN：0:目标点 1:断点 2:垂足点
			TTALINK_MISSION_SET_VEL: value=想要变速的值
			TTALINK_MISSION_SET_ALT: value=想要变高的值
 */
static inline void ttalink_function_mode_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t function_cmd, float param)
{
    _ttalink_function_mode_send(dst_addr, src_addr, chan, function_cmd, param, true);
}


static inline void _ttalink_function_mode_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_function_mode_t* function_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_function_mode_send_nocrc(dst_addr, src_addr, chan, function_mode->function_cmd, function_mode->param);
    }else{
        ttalink_function_mode_send(dst_addr, src_addr, chan, function_mode->function_cmd, function_mode->param);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FUNCTION_MODE, (const char *)function_mode, TTALINK_MSG_ID_FUNCTION_MODE_MIN_LEN, TTALINK_MSG_ID_FUNCTION_MODE_LEN, TTALINK_MSG_ID_FUNCTION_MODE_CRC, nocrc);
#endif
}

/**
 * @brief Send a function_mode message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_function_mode_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_function_mode_t* function_mode)
{
    _ttalink_function_mode_send_struct(dst_addr, src_addr, chan, function_mode, false);
}

/**
 * @brief Send a function_mode message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_function_mode_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_function_mode_t* function_mode)
{
    _ttalink_function_mode_send_struct(dst_addr, src_addr, chan, function_mode, true);
}

#if TTALINK_MSG_ID_FUNCTION_MODE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_function_mode_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t function_cmd, float param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, function_cmd);
    _tta_put_float(buf, 4, param);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FUNCTION_MODE, buf, TTALINK_MSG_ID_FUNCTION_MODE_MIN_LEN, TTALINK_MSG_ID_FUNCTION_MODE_LEN, TTALINK_MSG_ID_FUNCTION_MODE_CRC, nocrc);
#else
    ttalink_function_mode_t *packet = (ttalink_function_mode_t *)msgbuf;
    packet->function_cmd = function_cmd;
    packet->param = param;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FUNCTION_MODE, (const char *)packet, TTALINK_MSG_ID_FUNCTION_MODE_MIN_LEN, TTALINK_MSG_ID_FUNCTION_MODE_LEN, TTALINK_MSG_ID_FUNCTION_MODE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_function_mode_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t function_cmd, float param)
{
    _ttalink_function_mode_send_buf(dst_addr, src_addr, msgbuf, chan, function_cmd, param, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_function_mode_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t function_cmd, float param)
{
    _ttalink_function_mode_send_buf(dst_addr, src_addr, msgbuf, chan, function_cmd, param, true);
}
#endif

#endif

// MESSAGE FUNCTION_MODE UNPACKING


/**
 * @brief Get field function_cmd from function_mode message
 *
 * @return .
 */
static inline uint32_t ttalink_function_mode_get_function_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field param from function_mode message
 *
 * @return 
			TTALINK_MISSION_START： 0=不执行断点  1=执行断点.
			TTALINK_MISSION_AGAIN：0:目标点 1:断点 2:垂足点
			TTALINK_MISSION_SET_VEL: value=想要变速的值
			TTALINK_MISSION_SET_ALT: value=想要变高的值
 */
static inline float ttalink_function_mode_get_param(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  4);
}

/**
 * @brief Decode a function_mode message into a struct
 *
 * @param msg The message to decode
 * @param function_mode C-struct to decode the message contents into
 */
static inline void ttalink_function_mode_decode(const ttalink_message_t* msg, ttalink_function_mode_t* function_mode)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    function_mode->function_cmd = ttalink_function_mode_get_function_cmd(msg);
    function_mode->param = ttalink_function_mode_get_param(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_FUNCTION_MODE_LEN? msg->len : TTALINK_MSG_ID_FUNCTION_MODE_LEN;
        memset(function_mode, 0, TTALINK_MSG_ID_FUNCTION_MODE_LEN);
    memcpy(function_mode, _TTA_PAYLOAD(msg), len);
#endif
}
