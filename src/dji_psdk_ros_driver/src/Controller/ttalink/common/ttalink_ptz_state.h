#pragma once
// MESSAGE PTZ_STATE PACKING

#define TTALINK_MSG_ID_PTZ_STATE 2310

TTAPACKED(
typedef struct __ttalink_ptz_state_t {
 uint32_t update_time; /*<  update_time*/
 uint8_t cmd; /*<  .*/
 uint8_t ack; /*<  .*/
 float param; /*<  .*/
}) ttalink_ptz_state_t;

#define TTALINK_MSG_ID_PTZ_STATE_LEN 10
#define TTALINK_MSG_ID_PTZ_STATE_MIN_LEN 10
#define TTALINK_MSG_ID_2310_LEN 10
#define TTALINK_MSG_ID_2310_MIN_LEN 10

#define TTALINK_MSG_ID_PTZ_STATE_CRC 177
#define TTALINK_MSG_ID_2310_CRC 177



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_PTZ_STATE { \
    2310, \
    "PTZ_STATE", \
    4, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_ptz_state_t, update_time) }, \
         { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_ptz_state_t, cmd) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_ptz_state_t, ack) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 0, 6, offsetof(ttalink_ptz_state_t, param) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_PTZ_STATE { \
    "PTZ_STATE", \
    4, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_ptz_state_t, update_time) }, \
         { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_ptz_state_t, cmd) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_ptz_state_t, ack) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 0, 6, offsetof(ttalink_ptz_state_t, param) }, \
         } \
}
#endif


static inline uint16_t _ttalink_ptz_state_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd, uint8_t ack, float param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PTZ_STATE_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);
    _tta_put_uint8_t(buf, 5, ack);
    _tta_put_float(buf, 6, param);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PTZ_STATE_LEN);
#else
    ttalink_ptz_state_t packet;
    packet.update_time = update_time;
    packet.cmd = cmd;
    packet.ack = ack;
    packet.param = param;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PTZ_STATE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PTZ_STATE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_PTZ_STATE_MIN_LEN, TTALINK_MSG_ID_PTZ_STATE_LEN, TTALINK_MSG_ID_PTZ_STATE_CRC, nocrc);
}

/**
 * @brief Pack a ptz_state message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param cmd  .
 * @param ack  .
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_state_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd, uint8_t ack, float param)
{
    return _ttalink_ptz_state_pack(dst_addr, src_addr, msg,  update_time, cmd, ack, param, false);
}

/**
 * @brief Pack a ptz_state message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param cmd  .
 * @param ack  .
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_state_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd, uint8_t ack, float param)
{
    return _ttalink_ptz_state_pack(dst_addr, src_addr, msg,  update_time, cmd, ack, param, true);
}


static inline uint16_t _ttalink_ptz_state_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd,uint8_t ack,float param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PTZ_STATE_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);
    _tta_put_uint8_t(buf, 5, ack);
    _tta_put_float(buf, 6, param);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PTZ_STATE_LEN);
#else
    ttalink_ptz_state_t packet;
    packet.update_time = update_time;
    packet.cmd = cmd;
    packet.ack = ack;
    packet.param = param;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PTZ_STATE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PTZ_STATE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_STATE_MIN_LEN, TTALINK_MSG_ID_PTZ_STATE_LEN, TTALINK_MSG_ID_PTZ_STATE_CRC, nocrc);
}

/**
 * @brief Pack a ptz_state message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param cmd  .
 * @param ack  .
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_state_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd,uint8_t ack,float param)
{
    return _ttalink_ptz_state_pack_chan(dst_addr, src_addr, chan, msg,  update_time, cmd, ack, param, false);
}

/**
 * @brief Pack a ptz_state message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param cmd  .
 * @param ack  .
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_state_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd,uint8_t ack,float param)
{
    return _ttalink_ptz_state_pack_chan(dst_addr, src_addr, chan, msg,  update_time, cmd, ack, param, true);
}


static inline uint16_t _ttalink_ptz_state_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_ptz_state_t* ptz_state, bool nocrc)
{
    if(nocrc){
        return ttalink_ptz_state_pack_nocrc(dst_addr, src_addr, msg, ptz_state->update_time, ptz_state->cmd, ptz_state->ack, ptz_state->param);
    }else{
        return ttalink_ptz_state_pack(dst_addr, src_addr, msg, ptz_state->update_time, ptz_state->cmd, ptz_state->ack, ptz_state->param);
    }
    
}

/**
 * @brief Encode a ptz_state struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param ptz_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_state_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_ptz_state_t* ptz_state)
{
    return _ttalink_ptz_state_encode(dst_addr, src_addr, msg, ptz_state, false);
}

/**
 * @brief Encode a ptz_state struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param ptz_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_state_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_ptz_state_t* ptz_state)
{
    return _ttalink_ptz_state_encode(dst_addr, src_addr, msg, ptz_state, true);
}


static inline uint16_t _ttalink_ptz_state_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_ptz_state_t* ptz_state, bool nocrc)
{
    if(nocrc){
        return ttalink_ptz_state_pack_chan_nocrc(dst_addr, src_addr, chan, msg, ptz_state->update_time, ptz_state->cmd, ptz_state->ack, ptz_state->param);
    }else{
        return ttalink_ptz_state_pack_chan(dst_addr, src_addr, chan, msg, ptz_state->update_time, ptz_state->cmd, ptz_state->ack, ptz_state->param);
    }
}

/**
 * @brief Encode a ptz_state struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ptz_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_state_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_ptz_state_t* ptz_state)
{
    return _ttalink_ptz_state_encode_chan(dst_addr, src_addr, chan, msg, ptz_state, false);
}

/**
 * @brief Encode a ptz_state struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ptz_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_state_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_ptz_state_t* ptz_state)
{
    return _ttalink_ptz_state_encode_chan(dst_addr, src_addr, chan, msg, ptz_state, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_ptz_state_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd, uint8_t ack, float param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PTZ_STATE_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);
    _tta_put_uint8_t(buf, 5, ack);
    _tta_put_float(buf, 6, param);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_STATE, buf, TTALINK_MSG_ID_PTZ_STATE_MIN_LEN, TTALINK_MSG_ID_PTZ_STATE_LEN, TTALINK_MSG_ID_PTZ_STATE_CRC, nocrc);
#else
    ttalink_ptz_state_t packet;
    packet.update_time = update_time;
    packet.cmd = cmd;
    packet.ack = ack;
    packet.param = param;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_STATE, (const char *)&packet, TTALINK_MSG_ID_PTZ_STATE_MIN_LEN, TTALINK_MSG_ID_PTZ_STATE_LEN, TTALINK_MSG_ID_PTZ_STATE_CRC, nocrc);
#endif
}

/**
 * @brief Send a ptz_state message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param cmd  .
 * @param ack  .
 * @param param  .
 */
static inline void ttalink_ptz_state_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd, uint8_t ack, float param)
{
    _ttalink_ptz_state_send(dst_addr, src_addr, chan, update_time, cmd, ack, param, false);
}

/**
 * @brief Send a ptz_state message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param cmd  .
 * @param ack  .
 * @param param  .
 */
static inline void ttalink_ptz_state_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd, uint8_t ack, float param)
{
    _ttalink_ptz_state_send(dst_addr, src_addr, chan, update_time, cmd, ack, param, true);
}


static inline void _ttalink_ptz_state_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_ptz_state_t* ptz_state, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_ptz_state_send_nocrc(dst_addr, src_addr, chan, ptz_state->update_time, ptz_state->cmd, ptz_state->ack, ptz_state->param);
    }else{
        ttalink_ptz_state_send(dst_addr, src_addr, chan, ptz_state->update_time, ptz_state->cmd, ptz_state->ack, ptz_state->param);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_STATE, (const char *)ptz_state, TTALINK_MSG_ID_PTZ_STATE_MIN_LEN, TTALINK_MSG_ID_PTZ_STATE_LEN, TTALINK_MSG_ID_PTZ_STATE_CRC, nocrc);
#endif
}

/**
 * @brief Send a ptz_state message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_ptz_state_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_ptz_state_t* ptz_state)
{
    _ttalink_ptz_state_send_struct(dst_addr, src_addr, chan, ptz_state, false);
}

/**
 * @brief Send a ptz_state message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_ptz_state_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_ptz_state_t* ptz_state)
{
    _ttalink_ptz_state_send_struct(dst_addr, src_addr, chan, ptz_state, true);
}

#if TTALINK_MSG_ID_PTZ_STATE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_ptz_state_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd, uint8_t ack, float param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);
    _tta_put_uint8_t(buf, 5, ack);
    _tta_put_float(buf, 6, param);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_STATE, buf, TTALINK_MSG_ID_PTZ_STATE_MIN_LEN, TTALINK_MSG_ID_PTZ_STATE_LEN, TTALINK_MSG_ID_PTZ_STATE_CRC, nocrc);
#else
    ttalink_ptz_state_t *packet = (ttalink_ptz_state_t *)msgbuf;
    packet->update_time = update_time;
    packet->cmd = cmd;
    packet->ack = ack;
    packet->param = param;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_STATE, (const char *)packet, TTALINK_MSG_ID_PTZ_STATE_MIN_LEN, TTALINK_MSG_ID_PTZ_STATE_LEN, TTALINK_MSG_ID_PTZ_STATE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_ptz_state_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd, uint8_t ack, float param)
{
    _ttalink_ptz_state_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, cmd, ack, param, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_ptz_state_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd, uint8_t ack, float param)
{
    _ttalink_ptz_state_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, cmd, ack, param, true);
}
#endif

#endif

// MESSAGE PTZ_STATE UNPACKING


/**
 * @brief Get field update_time from ptz_state message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_ptz_state_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field cmd from ptz_state message
 *
 * @return  .
 */
static inline uint8_t ttalink_ptz_state_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field ack from ptz_state message
 *
 * @return  .
 */
static inline uint8_t ttalink_ptz_state_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field param from ptz_state message
 *
 * @return  .
 */
static inline float ttalink_ptz_state_get_param(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  6);
}

/**
 * @brief Decode a ptz_state message into a struct
 *
 * @param msg The message to decode
 * @param ptz_state C-struct to decode the message contents into
 */
static inline void ttalink_ptz_state_decode(const ttalink_message_t* msg, ttalink_ptz_state_t* ptz_state)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ptz_state->update_time = ttalink_ptz_state_get_update_time(msg);
    ptz_state->cmd = ttalink_ptz_state_get_cmd(msg);
    ptz_state->ack = ttalink_ptz_state_get_ack(msg);
    ptz_state->param = ttalink_ptz_state_get_param(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_PTZ_STATE_LEN? msg->len : TTALINK_MSG_ID_PTZ_STATE_LEN;
        memset(ptz_state, 0, TTALINK_MSG_ID_PTZ_STATE_LEN);
    memcpy(ptz_state, _TTA_PAYLOAD(msg), len);
#endif
}
