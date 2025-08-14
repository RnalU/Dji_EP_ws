#pragma once
// MESSAGE SWITCH_CTRL PACKING

#define TTALINK_MSG_ID_SWITCH_CTRL 2202

TTAPACKED(
typedef struct __ttalink_switch_ctrl_t {
 int8_t switch_cmd; /*<   . */
}) ttalink_switch_ctrl_t;

#define TTALINK_MSG_ID_SWITCH_CTRL_LEN 1
#define TTALINK_MSG_ID_SWITCH_CTRL_MIN_LEN 1
#define TTALINK_MSG_ID_2202_LEN 1
#define TTALINK_MSG_ID_2202_MIN_LEN 1

#define TTALINK_MSG_ID_SWITCH_CTRL_CRC 38
#define TTALINK_MSG_ID_2202_CRC 38



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SWITCH_CTRL { \
    2202, \
    "SWITCH_CTRL", \
    1, \
    {  { "switch_cmd", NULL, TTALINK_TYPE_INT8_T, 0, 0, offsetof(ttalink_switch_ctrl_t, switch_cmd) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SWITCH_CTRL { \
    "SWITCH_CTRL", \
    1, \
    {  { "switch_cmd", NULL, TTALINK_TYPE_INT8_T, 0, 0, offsetof(ttalink_switch_ctrl_t, switch_cmd) }, \
         } \
}
#endif


static inline uint16_t _ttalink_switch_ctrl_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int8_t switch_cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SWITCH_CTRL_LEN];
    _tta_put_int8_t(buf, 0, switch_cmd);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SWITCH_CTRL_LEN);
#else
    ttalink_switch_ctrl_t packet;
    packet.switch_cmd = switch_cmd;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SWITCH_CTRL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SWITCH_CTRL;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SWITCH_CTRL_MIN_LEN, TTALINK_MSG_ID_SWITCH_CTRL_LEN, TTALINK_MSG_ID_SWITCH_CTRL_CRC, nocrc);
}

/**
 * @brief Pack a switch_ctrl message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param switch_cmd   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_switch_ctrl_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int8_t switch_cmd)
{
    return _ttalink_switch_ctrl_pack(dst_addr, src_addr, msg,  switch_cmd, false);
}

/**
 * @brief Pack a switch_ctrl message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param switch_cmd   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_switch_ctrl_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int8_t switch_cmd)
{
    return _ttalink_switch_ctrl_pack(dst_addr, src_addr, msg,  switch_cmd, true);
}


static inline uint16_t _ttalink_switch_ctrl_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int8_t switch_cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SWITCH_CTRL_LEN];
    _tta_put_int8_t(buf, 0, switch_cmd);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SWITCH_CTRL_LEN);
#else
    ttalink_switch_ctrl_t packet;
    packet.switch_cmd = switch_cmd;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SWITCH_CTRL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SWITCH_CTRL;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SWITCH_CTRL_MIN_LEN, TTALINK_MSG_ID_SWITCH_CTRL_LEN, TTALINK_MSG_ID_SWITCH_CTRL_CRC, nocrc);
}

/**
 * @brief Pack a switch_ctrl message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param switch_cmd   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_switch_ctrl_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int8_t switch_cmd)
{
    return _ttalink_switch_ctrl_pack_chan(dst_addr, src_addr, chan, msg,  switch_cmd, false);
}

/**
 * @brief Pack a switch_ctrl message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param switch_cmd   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_switch_ctrl_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int8_t switch_cmd)
{
    return _ttalink_switch_ctrl_pack_chan(dst_addr, src_addr, chan, msg,  switch_cmd, true);
}


static inline uint16_t _ttalink_switch_ctrl_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_switch_ctrl_t* switch_ctrl, bool nocrc)
{
    if(nocrc){
        return ttalink_switch_ctrl_pack_nocrc(dst_addr, src_addr, msg, switch_ctrl->switch_cmd);
    }else{
        return ttalink_switch_ctrl_pack(dst_addr, src_addr, msg, switch_ctrl->switch_cmd);
    }
    
}

/**
 * @brief Encode a switch_ctrl struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param switch_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_switch_ctrl_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_switch_ctrl_t* switch_ctrl)
{
    return _ttalink_switch_ctrl_encode(dst_addr, src_addr, msg, switch_ctrl, false);
}

/**
 * @brief Encode a switch_ctrl struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param switch_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_switch_ctrl_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_switch_ctrl_t* switch_ctrl)
{
    return _ttalink_switch_ctrl_encode(dst_addr, src_addr, msg, switch_ctrl, true);
}


static inline uint16_t _ttalink_switch_ctrl_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_switch_ctrl_t* switch_ctrl, bool nocrc)
{
    if(nocrc){
        return ttalink_switch_ctrl_pack_chan_nocrc(dst_addr, src_addr, chan, msg, switch_ctrl->switch_cmd);
    }else{
        return ttalink_switch_ctrl_pack_chan(dst_addr, src_addr, chan, msg, switch_ctrl->switch_cmd);
    }
}

/**
 * @brief Encode a switch_ctrl struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param switch_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_switch_ctrl_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_switch_ctrl_t* switch_ctrl)
{
    return _ttalink_switch_ctrl_encode_chan(dst_addr, src_addr, chan, msg, switch_ctrl, false);
}

/**
 * @brief Encode a switch_ctrl struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param switch_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_switch_ctrl_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_switch_ctrl_t* switch_ctrl)
{
    return _ttalink_switch_ctrl_encode_chan(dst_addr, src_addr, chan, msg, switch_ctrl, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_switch_ctrl_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int8_t switch_cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SWITCH_CTRL_LEN];
    _tta_put_int8_t(buf, 0, switch_cmd);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SWITCH_CTRL, buf, TTALINK_MSG_ID_SWITCH_CTRL_MIN_LEN, TTALINK_MSG_ID_SWITCH_CTRL_LEN, TTALINK_MSG_ID_SWITCH_CTRL_CRC, nocrc);
#else
    ttalink_switch_ctrl_t packet;
    packet.switch_cmd = switch_cmd;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SWITCH_CTRL, (const char *)&packet, TTALINK_MSG_ID_SWITCH_CTRL_MIN_LEN, TTALINK_MSG_ID_SWITCH_CTRL_LEN, TTALINK_MSG_ID_SWITCH_CTRL_CRC, nocrc);
#endif
}

/**
 * @brief Send a switch_ctrl message
 * @param chan TTAlink channel to send the message
 *
 * @param switch_cmd   . 
 */
static inline void ttalink_switch_ctrl_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int8_t switch_cmd)
{
    _ttalink_switch_ctrl_send(dst_addr, src_addr, chan, switch_cmd, false);
}

/**
 * @brief Send a switch_ctrl message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param switch_cmd   . 
 */
static inline void ttalink_switch_ctrl_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int8_t switch_cmd)
{
    _ttalink_switch_ctrl_send(dst_addr, src_addr, chan, switch_cmd, true);
}


static inline void _ttalink_switch_ctrl_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_switch_ctrl_t* switch_ctrl, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_switch_ctrl_send_nocrc(dst_addr, src_addr, chan, switch_ctrl->switch_cmd);
    }else{
        ttalink_switch_ctrl_send(dst_addr, src_addr, chan, switch_ctrl->switch_cmd);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SWITCH_CTRL, (const char *)switch_ctrl, TTALINK_MSG_ID_SWITCH_CTRL_MIN_LEN, TTALINK_MSG_ID_SWITCH_CTRL_LEN, TTALINK_MSG_ID_SWITCH_CTRL_CRC, nocrc);
#endif
}

/**
 * @brief Send a switch_ctrl message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_switch_ctrl_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_switch_ctrl_t* switch_ctrl)
{
    _ttalink_switch_ctrl_send_struct(dst_addr, src_addr, chan, switch_ctrl, false);
}

/**
 * @brief Send a switch_ctrl message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_switch_ctrl_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_switch_ctrl_t* switch_ctrl)
{
    _ttalink_switch_ctrl_send_struct(dst_addr, src_addr, chan, switch_ctrl, true);
}

#if TTALINK_MSG_ID_SWITCH_CTRL_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_switch_ctrl_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int8_t switch_cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_int8_t(buf, 0, switch_cmd);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SWITCH_CTRL, buf, TTALINK_MSG_ID_SWITCH_CTRL_MIN_LEN, TTALINK_MSG_ID_SWITCH_CTRL_LEN, TTALINK_MSG_ID_SWITCH_CTRL_CRC, nocrc);
#else
    ttalink_switch_ctrl_t *packet = (ttalink_switch_ctrl_t *)msgbuf;
    packet->switch_cmd = switch_cmd;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SWITCH_CTRL, (const char *)packet, TTALINK_MSG_ID_SWITCH_CTRL_MIN_LEN, TTALINK_MSG_ID_SWITCH_CTRL_LEN, TTALINK_MSG_ID_SWITCH_CTRL_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_switch_ctrl_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int8_t switch_cmd)
{
    _ttalink_switch_ctrl_send_buf(dst_addr, src_addr, msgbuf, chan, switch_cmd, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_switch_ctrl_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int8_t switch_cmd)
{
    _ttalink_switch_ctrl_send_buf(dst_addr, src_addr, msgbuf, chan, switch_cmd, true);
}
#endif

#endif

// MESSAGE SWITCH_CTRL UNPACKING


/**
 * @brief Get field switch_cmd from switch_ctrl message
 *
 * @return   . 
 */
static inline int8_t ttalink_switch_ctrl_get_switch_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int8_t(msg,  0);
}

/**
 * @brief Decode a switch_ctrl message into a struct
 *
 * @param msg The message to decode
 * @param switch_ctrl C-struct to decode the message contents into
 */
static inline void ttalink_switch_ctrl_decode(const ttalink_message_t* msg, ttalink_switch_ctrl_t* switch_ctrl)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    switch_ctrl->switch_cmd = ttalink_switch_ctrl_get_switch_cmd(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SWITCH_CTRL_LEN? msg->len : TTALINK_MSG_ID_SWITCH_CTRL_LEN;
        memset(switch_ctrl, 0, TTALINK_MSG_ID_SWITCH_CTRL_LEN);
    memcpy(switch_ctrl, _TTA_PAYLOAD(msg), len);
#endif
}
