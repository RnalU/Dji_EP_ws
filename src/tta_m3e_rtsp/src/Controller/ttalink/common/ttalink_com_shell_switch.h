#pragma once
// MESSAGE COM_SHELL_SWITCH PACKING

#define TTALINK_MSG_ID_COM_SHELL_SWITCH 2208

TTAPACKED(
typedef struct __ttalink_com_shell_switch_t {
 uint8_t com_shell_mode; /*<  .*/
}) ttalink_com_shell_switch_t;

#define TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN 1
#define TTALINK_MSG_ID_COM_SHELL_SWITCH_MIN_LEN 1
#define TTALINK_MSG_ID_2208_LEN 1
#define TTALINK_MSG_ID_2208_MIN_LEN 1

#define TTALINK_MSG_ID_COM_SHELL_SWITCH_CRC 23
#define TTALINK_MSG_ID_2208_CRC 23



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_COM_SHELL_SWITCH { \
    2208, \
    "COM_SHELL_SWITCH", \
    1, \
    {  { "com_shell_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_com_shell_switch_t, com_shell_mode) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_COM_SHELL_SWITCH { \
    "COM_SHELL_SWITCH", \
    1, \
    {  { "com_shell_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_com_shell_switch_t, com_shell_mode) }, \
         } \
}
#endif


static inline uint16_t _ttalink_com_shell_switch_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t com_shell_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN];
    _tta_put_uint8_t(buf, 0, com_shell_mode);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN);
#else
    ttalink_com_shell_switch_t packet;
    packet.com_shell_mode = com_shell_mode;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_COM_SHELL_SWITCH;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_COM_SHELL_SWITCH_MIN_LEN, TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN, TTALINK_MSG_ID_COM_SHELL_SWITCH_CRC, nocrc);
}

/**
 * @brief Pack a com_shell_switch message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param com_shell_mode  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_com_shell_switch_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t com_shell_mode)
{
    return _ttalink_com_shell_switch_pack(dst_addr, src_addr, msg,  com_shell_mode, false);
}

/**
 * @brief Pack a com_shell_switch message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param com_shell_mode  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_com_shell_switch_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t com_shell_mode)
{
    return _ttalink_com_shell_switch_pack(dst_addr, src_addr, msg,  com_shell_mode, true);
}


static inline uint16_t _ttalink_com_shell_switch_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t com_shell_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN];
    _tta_put_uint8_t(buf, 0, com_shell_mode);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN);
#else
    ttalink_com_shell_switch_t packet;
    packet.com_shell_mode = com_shell_mode;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_COM_SHELL_SWITCH;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_COM_SHELL_SWITCH_MIN_LEN, TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN, TTALINK_MSG_ID_COM_SHELL_SWITCH_CRC, nocrc);
}

/**
 * @brief Pack a com_shell_switch message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param com_shell_mode  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_com_shell_switch_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t com_shell_mode)
{
    return _ttalink_com_shell_switch_pack_chan(dst_addr, src_addr, chan, msg,  com_shell_mode, false);
}

/**
 * @brief Pack a com_shell_switch message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param com_shell_mode  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_com_shell_switch_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t com_shell_mode)
{
    return _ttalink_com_shell_switch_pack_chan(dst_addr, src_addr, chan, msg,  com_shell_mode, true);
}


static inline uint16_t _ttalink_com_shell_switch_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_com_shell_switch_t* com_shell_switch, bool nocrc)
{
    if(nocrc){
        return ttalink_com_shell_switch_pack_nocrc(dst_addr, src_addr, msg, com_shell_switch->com_shell_mode);
    }else{
        return ttalink_com_shell_switch_pack(dst_addr, src_addr, msg, com_shell_switch->com_shell_mode);
    }
    
}

/**
 * @brief Encode a com_shell_switch struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param com_shell_switch C-struct to read the message contents from
 */
static inline uint16_t ttalink_com_shell_switch_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_com_shell_switch_t* com_shell_switch)
{
    return _ttalink_com_shell_switch_encode(dst_addr, src_addr, msg, com_shell_switch, false);
}

/**
 * @brief Encode a com_shell_switch struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param com_shell_switch C-struct to read the message contents from
 */
static inline uint16_t ttalink_com_shell_switch_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_com_shell_switch_t* com_shell_switch)
{
    return _ttalink_com_shell_switch_encode(dst_addr, src_addr, msg, com_shell_switch, true);
}


static inline uint16_t _ttalink_com_shell_switch_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_com_shell_switch_t* com_shell_switch, bool nocrc)
{
    if(nocrc){
        return ttalink_com_shell_switch_pack_chan_nocrc(dst_addr, src_addr, chan, msg, com_shell_switch->com_shell_mode);
    }else{
        return ttalink_com_shell_switch_pack_chan(dst_addr, src_addr, chan, msg, com_shell_switch->com_shell_mode);
    }
}

/**
 * @brief Encode a com_shell_switch struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param com_shell_switch C-struct to read the message contents from
 */
static inline uint16_t ttalink_com_shell_switch_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_com_shell_switch_t* com_shell_switch)
{
    return _ttalink_com_shell_switch_encode_chan(dst_addr, src_addr, chan, msg, com_shell_switch, false);
}

/**
 * @brief Encode a com_shell_switch struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param com_shell_switch C-struct to read the message contents from
 */
static inline uint16_t ttalink_com_shell_switch_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_com_shell_switch_t* com_shell_switch)
{
    return _ttalink_com_shell_switch_encode_chan(dst_addr, src_addr, chan, msg, com_shell_switch, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_com_shell_switch_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t com_shell_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN];
    _tta_put_uint8_t(buf, 0, com_shell_mode);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_COM_SHELL_SWITCH, buf, TTALINK_MSG_ID_COM_SHELL_SWITCH_MIN_LEN, TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN, TTALINK_MSG_ID_COM_SHELL_SWITCH_CRC, nocrc);
#else
    ttalink_com_shell_switch_t packet;
    packet.com_shell_mode = com_shell_mode;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_COM_SHELL_SWITCH, (const char *)&packet, TTALINK_MSG_ID_COM_SHELL_SWITCH_MIN_LEN, TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN, TTALINK_MSG_ID_COM_SHELL_SWITCH_CRC, nocrc);
#endif
}

/**
 * @brief Send a com_shell_switch message
 * @param chan TTAlink channel to send the message
 *
 * @param com_shell_mode  .
 */
static inline void ttalink_com_shell_switch_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t com_shell_mode)
{
    _ttalink_com_shell_switch_send(dst_addr, src_addr, chan, com_shell_mode, false);
}

/**
 * @brief Send a com_shell_switch message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param com_shell_mode  .
 */
static inline void ttalink_com_shell_switch_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t com_shell_mode)
{
    _ttalink_com_shell_switch_send(dst_addr, src_addr, chan, com_shell_mode, true);
}


static inline void _ttalink_com_shell_switch_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_com_shell_switch_t* com_shell_switch, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_com_shell_switch_send_nocrc(dst_addr, src_addr, chan, com_shell_switch->com_shell_mode);
    }else{
        ttalink_com_shell_switch_send(dst_addr, src_addr, chan, com_shell_switch->com_shell_mode);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_COM_SHELL_SWITCH, (const char *)com_shell_switch, TTALINK_MSG_ID_COM_SHELL_SWITCH_MIN_LEN, TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN, TTALINK_MSG_ID_COM_SHELL_SWITCH_CRC, nocrc);
#endif
}

/**
 * @brief Send a com_shell_switch message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_com_shell_switch_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_com_shell_switch_t* com_shell_switch)
{
    _ttalink_com_shell_switch_send_struct(dst_addr, src_addr, chan, com_shell_switch, false);
}

/**
 * @brief Send a com_shell_switch message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_com_shell_switch_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_com_shell_switch_t* com_shell_switch)
{
    _ttalink_com_shell_switch_send_struct(dst_addr, src_addr, chan, com_shell_switch, true);
}

#if TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_com_shell_switch_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t com_shell_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, com_shell_mode);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_COM_SHELL_SWITCH, buf, TTALINK_MSG_ID_COM_SHELL_SWITCH_MIN_LEN, TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN, TTALINK_MSG_ID_COM_SHELL_SWITCH_CRC, nocrc);
#else
    ttalink_com_shell_switch_t *packet = (ttalink_com_shell_switch_t *)msgbuf;
    packet->com_shell_mode = com_shell_mode;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_COM_SHELL_SWITCH, (const char *)packet, TTALINK_MSG_ID_COM_SHELL_SWITCH_MIN_LEN, TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN, TTALINK_MSG_ID_COM_SHELL_SWITCH_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_com_shell_switch_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t com_shell_mode)
{
    _ttalink_com_shell_switch_send_buf(dst_addr, src_addr, msgbuf, chan, com_shell_mode, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_com_shell_switch_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t com_shell_mode)
{
    _ttalink_com_shell_switch_send_buf(dst_addr, src_addr, msgbuf, chan, com_shell_mode, true);
}
#endif

#endif

// MESSAGE COM_SHELL_SWITCH UNPACKING


/**
 * @brief Get field com_shell_mode from com_shell_switch message
 *
 * @return  .
 */
static inline uint8_t ttalink_com_shell_switch_get_com_shell_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a com_shell_switch message into a struct
 *
 * @param msg The message to decode
 * @param com_shell_switch C-struct to decode the message contents into
 */
static inline void ttalink_com_shell_switch_decode(const ttalink_message_t* msg, ttalink_com_shell_switch_t* com_shell_switch)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    com_shell_switch->com_shell_mode = ttalink_com_shell_switch_get_com_shell_mode(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN? msg->len : TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN;
        memset(com_shell_switch, 0, TTALINK_MSG_ID_COM_SHELL_SWITCH_LEN);
    memcpy(com_shell_switch, _TTA_PAYLOAD(msg), len);
#endif
}
