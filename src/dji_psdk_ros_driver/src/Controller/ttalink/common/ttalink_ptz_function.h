#pragma once
// MESSAGE PTZ_FUNCTION PACKING

#define TTALINK_MSG_ID_PTZ_FUNCTION 2305

TTAPACKED(
typedef struct __ttalink_ptz_function_t {
 uint32_t update_time; /*<  update_time*/
 uint8_t cmd; /*<  .*/
 float param; /*<  .*/
}) ttalink_ptz_function_t;

#define TTALINK_MSG_ID_PTZ_FUNCTION_LEN 9
#define TTALINK_MSG_ID_PTZ_FUNCTION_MIN_LEN 9
#define TTALINK_MSG_ID_2305_LEN 9
#define TTALINK_MSG_ID_2305_MIN_LEN 9

#define TTALINK_MSG_ID_PTZ_FUNCTION_CRC 104
#define TTALINK_MSG_ID_2305_CRC 104



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_PTZ_FUNCTION { \
    2305, \
    "PTZ_FUNCTION", \
    3, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_ptz_function_t, update_time) }, \
         { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_ptz_function_t, cmd) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 0, 5, offsetof(ttalink_ptz_function_t, param) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_PTZ_FUNCTION { \
    "PTZ_FUNCTION", \
    3, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_ptz_function_t, update_time) }, \
         { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_ptz_function_t, cmd) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 0, 5, offsetof(ttalink_ptz_function_t, param) }, \
         } \
}
#endif


static inline uint16_t _ttalink_ptz_function_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd, float param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PTZ_FUNCTION_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);
    _tta_put_float(buf, 5, param);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PTZ_FUNCTION_LEN);
#else
    ttalink_ptz_function_t packet;
    packet.update_time = update_time;
    packet.cmd = cmd;
    packet.param = param;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PTZ_FUNCTION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PTZ_FUNCTION;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_PTZ_FUNCTION_MIN_LEN, TTALINK_MSG_ID_PTZ_FUNCTION_LEN, TTALINK_MSG_ID_PTZ_FUNCTION_CRC, nocrc);
}

/**
 * @brief Pack a ptz_function message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param cmd  .
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_function_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd, float param)
{
    return _ttalink_ptz_function_pack(dst_addr, src_addr, msg,  update_time, cmd, param, false);
}

/**
 * @brief Pack a ptz_function message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param cmd  .
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_function_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd, float param)
{
    return _ttalink_ptz_function_pack(dst_addr, src_addr, msg,  update_time, cmd, param, true);
}


static inline uint16_t _ttalink_ptz_function_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd,float param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PTZ_FUNCTION_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);
    _tta_put_float(buf, 5, param);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PTZ_FUNCTION_LEN);
#else
    ttalink_ptz_function_t packet;
    packet.update_time = update_time;
    packet.cmd = cmd;
    packet.param = param;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PTZ_FUNCTION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PTZ_FUNCTION;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_FUNCTION_MIN_LEN, TTALINK_MSG_ID_PTZ_FUNCTION_LEN, TTALINK_MSG_ID_PTZ_FUNCTION_CRC, nocrc);
}

/**
 * @brief Pack a ptz_function message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param cmd  .
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_function_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd,float param)
{
    return _ttalink_ptz_function_pack_chan(dst_addr, src_addr, chan, msg,  update_time, cmd, param, false);
}

/**
 * @brief Pack a ptz_function message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param cmd  .
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_function_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd,float param)
{
    return _ttalink_ptz_function_pack_chan(dst_addr, src_addr, chan, msg,  update_time, cmd, param, true);
}


static inline uint16_t _ttalink_ptz_function_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_ptz_function_t* ptz_function, bool nocrc)
{
    if(nocrc){
        return ttalink_ptz_function_pack_nocrc(dst_addr, src_addr, msg, ptz_function->update_time, ptz_function->cmd, ptz_function->param);
    }else{
        return ttalink_ptz_function_pack(dst_addr, src_addr, msg, ptz_function->update_time, ptz_function->cmd, ptz_function->param);
    }
    
}

/**
 * @brief Encode a ptz_function struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param ptz_function C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_function_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_ptz_function_t* ptz_function)
{
    return _ttalink_ptz_function_encode(dst_addr, src_addr, msg, ptz_function, false);
}

/**
 * @brief Encode a ptz_function struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param ptz_function C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_function_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_ptz_function_t* ptz_function)
{
    return _ttalink_ptz_function_encode(dst_addr, src_addr, msg, ptz_function, true);
}


static inline uint16_t _ttalink_ptz_function_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_ptz_function_t* ptz_function, bool nocrc)
{
    if(nocrc){
        return ttalink_ptz_function_pack_chan_nocrc(dst_addr, src_addr, chan, msg, ptz_function->update_time, ptz_function->cmd, ptz_function->param);
    }else{
        return ttalink_ptz_function_pack_chan(dst_addr, src_addr, chan, msg, ptz_function->update_time, ptz_function->cmd, ptz_function->param);
    }
}

/**
 * @brief Encode a ptz_function struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ptz_function C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_function_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_ptz_function_t* ptz_function)
{
    return _ttalink_ptz_function_encode_chan(dst_addr, src_addr, chan, msg, ptz_function, false);
}

/**
 * @brief Encode a ptz_function struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ptz_function C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_function_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_ptz_function_t* ptz_function)
{
    return _ttalink_ptz_function_encode_chan(dst_addr, src_addr, chan, msg, ptz_function, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_ptz_function_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd, float param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PTZ_FUNCTION_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);
    _tta_put_float(buf, 5, param);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_FUNCTION, buf, TTALINK_MSG_ID_PTZ_FUNCTION_MIN_LEN, TTALINK_MSG_ID_PTZ_FUNCTION_LEN, TTALINK_MSG_ID_PTZ_FUNCTION_CRC, nocrc);
#else
    ttalink_ptz_function_t packet;
    packet.update_time = update_time;
    packet.cmd = cmd;
    packet.param = param;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_FUNCTION, (const char *)&packet, TTALINK_MSG_ID_PTZ_FUNCTION_MIN_LEN, TTALINK_MSG_ID_PTZ_FUNCTION_LEN, TTALINK_MSG_ID_PTZ_FUNCTION_CRC, nocrc);
#endif
}

/**
 * @brief Send a ptz_function message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param cmd  .
 * @param param  .
 */
static inline void ttalink_ptz_function_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd, float param)
{
    _ttalink_ptz_function_send(dst_addr, src_addr, chan, update_time, cmd, param, false);
}

/**
 * @brief Send a ptz_function message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param cmd  .
 * @param param  .
 */
static inline void ttalink_ptz_function_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd, float param)
{
    _ttalink_ptz_function_send(dst_addr, src_addr, chan, update_time, cmd, param, true);
}


static inline void _ttalink_ptz_function_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_ptz_function_t* ptz_function, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_ptz_function_send_nocrc(dst_addr, src_addr, chan, ptz_function->update_time, ptz_function->cmd, ptz_function->param);
    }else{
        ttalink_ptz_function_send(dst_addr, src_addr, chan, ptz_function->update_time, ptz_function->cmd, ptz_function->param);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_FUNCTION, (const char *)ptz_function, TTALINK_MSG_ID_PTZ_FUNCTION_MIN_LEN, TTALINK_MSG_ID_PTZ_FUNCTION_LEN, TTALINK_MSG_ID_PTZ_FUNCTION_CRC, nocrc);
#endif
}

/**
 * @brief Send a ptz_function message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_ptz_function_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_ptz_function_t* ptz_function)
{
    _ttalink_ptz_function_send_struct(dst_addr, src_addr, chan, ptz_function, false);
}

/**
 * @brief Send a ptz_function message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_ptz_function_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_ptz_function_t* ptz_function)
{
    _ttalink_ptz_function_send_struct(dst_addr, src_addr, chan, ptz_function, true);
}

#if TTALINK_MSG_ID_PTZ_FUNCTION_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_ptz_function_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd, float param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);
    _tta_put_float(buf, 5, param);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_FUNCTION, buf, TTALINK_MSG_ID_PTZ_FUNCTION_MIN_LEN, TTALINK_MSG_ID_PTZ_FUNCTION_LEN, TTALINK_MSG_ID_PTZ_FUNCTION_CRC, nocrc);
#else
    ttalink_ptz_function_t *packet = (ttalink_ptz_function_t *)msgbuf;
    packet->update_time = update_time;
    packet->cmd = cmd;
    packet->param = param;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_FUNCTION, (const char *)packet, TTALINK_MSG_ID_PTZ_FUNCTION_MIN_LEN, TTALINK_MSG_ID_PTZ_FUNCTION_LEN, TTALINK_MSG_ID_PTZ_FUNCTION_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_ptz_function_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd, float param)
{
    _ttalink_ptz_function_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, cmd, param, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_ptz_function_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd, float param)
{
    _ttalink_ptz_function_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, cmd, param, true);
}
#endif

#endif

// MESSAGE PTZ_FUNCTION UNPACKING


/**
 * @brief Get field update_time from ptz_function message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_ptz_function_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field cmd from ptz_function message
 *
 * @return  .
 */
static inline uint8_t ttalink_ptz_function_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field param from ptz_function message
 *
 * @return  .
 */
static inline float ttalink_ptz_function_get_param(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  5);
}

/**
 * @brief Decode a ptz_function message into a struct
 *
 * @param msg The message to decode
 * @param ptz_function C-struct to decode the message contents into
 */
static inline void ttalink_ptz_function_decode(const ttalink_message_t* msg, ttalink_ptz_function_t* ptz_function)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ptz_function->update_time = ttalink_ptz_function_get_update_time(msg);
    ptz_function->cmd = ttalink_ptz_function_get_cmd(msg);
    ptz_function->param = ttalink_ptz_function_get_param(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_PTZ_FUNCTION_LEN? msg->len : TTALINK_MSG_ID_PTZ_FUNCTION_LEN;
        memset(ptz_function, 0, TTALINK_MSG_ID_PTZ_FUNCTION_LEN);
    memcpy(ptz_function, _TTA_PAYLOAD(msg), len);
#endif
}
