#pragma once
// MESSAGE EMBE_CONFIG_ITEMS PACKING

#define TTALINK_MSG_ID_EMBE_CONFIG_ITEMS 2082

TTAPACKED(
typedef struct __ttalink_embe_config_items_t {
 uint8_t cmd; /*<  cmd.*/
}) ttalink_embe_config_items_t;

#define TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN 1
#define TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_MIN_LEN 1
#define TTALINK_MSG_ID_2082_LEN 1
#define TTALINK_MSG_ID_2082_MIN_LEN 1

#define TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_CRC 98
#define TTALINK_MSG_ID_2082_CRC 98



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_EMBE_CONFIG_ITEMS { \
    2082, \
    "EMBE_CONFIG_ITEMS", \
    1, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_embe_config_items_t, cmd) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_EMBE_CONFIG_ITEMS { \
    "EMBE_CONFIG_ITEMS", \
    1, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_embe_config_items_t, cmd) }, \
         } \
}
#endif


static inline uint16_t _ttalink_embe_config_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, cmd);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN);
#else
    ttalink_embe_config_items_t packet;
    packet.cmd = cmd;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_EMBE_CONFIG_ITEMS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a embe_config_items message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  cmd.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_embe_config_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd)
{
    return _ttalink_embe_config_items_pack(dst_addr, src_addr, msg,  cmd, false);
}

/**
 * @brief Pack a embe_config_items message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  cmd.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_embe_config_items_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd)
{
    return _ttalink_embe_config_items_pack(dst_addr, src_addr, msg,  cmd, true);
}


static inline uint16_t _ttalink_embe_config_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, cmd);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN);
#else
    ttalink_embe_config_items_t packet;
    packet.cmd = cmd;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_EMBE_CONFIG_ITEMS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a embe_config_items message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  cmd.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_embe_config_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd)
{
    return _ttalink_embe_config_items_pack_chan(dst_addr, src_addr, chan, msg,  cmd, false);
}

/**
 * @brief Pack a embe_config_items message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  cmd.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_embe_config_items_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd)
{
    return _ttalink_embe_config_items_pack_chan(dst_addr, src_addr, chan, msg,  cmd, true);
}


static inline uint16_t _ttalink_embe_config_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_embe_config_items_t* embe_config_items, bool nocrc)
{
    if(nocrc){
        return ttalink_embe_config_items_pack_nocrc(dst_addr, src_addr, msg, embe_config_items->cmd);
    }else{
        return ttalink_embe_config_items_pack(dst_addr, src_addr, msg, embe_config_items->cmd);
    }
    
}

/**
 * @brief Encode a embe_config_items struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param embe_config_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_embe_config_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_embe_config_items_t* embe_config_items)
{
    return _ttalink_embe_config_items_encode(dst_addr, src_addr, msg, embe_config_items, false);
}

/**
 * @brief Encode a embe_config_items struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param embe_config_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_embe_config_items_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_embe_config_items_t* embe_config_items)
{
    return _ttalink_embe_config_items_encode(dst_addr, src_addr, msg, embe_config_items, true);
}


static inline uint16_t _ttalink_embe_config_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_embe_config_items_t* embe_config_items, bool nocrc)
{
    if(nocrc){
        return ttalink_embe_config_items_pack_chan_nocrc(dst_addr, src_addr, chan, msg, embe_config_items->cmd);
    }else{
        return ttalink_embe_config_items_pack_chan(dst_addr, src_addr, chan, msg, embe_config_items->cmd);
    }
}

/**
 * @brief Encode a embe_config_items struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param embe_config_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_embe_config_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_embe_config_items_t* embe_config_items)
{
    return _ttalink_embe_config_items_encode_chan(dst_addr, src_addr, chan, msg, embe_config_items, false);
}

/**
 * @brief Encode a embe_config_items struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param embe_config_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_embe_config_items_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_embe_config_items_t* embe_config_items)
{
    return _ttalink_embe_config_items_encode_chan(dst_addr, src_addr, chan, msg, embe_config_items, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_embe_config_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, cmd);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS, buf, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_CRC, nocrc);
#else
    ttalink_embe_config_items_t packet;
    packet.cmd = cmd;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS, (const char *)&packet, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a embe_config_items message
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  cmd.
 */
static inline void ttalink_embe_config_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd)
{
    _ttalink_embe_config_items_send(dst_addr, src_addr, chan, cmd, false);
}

/**
 * @brief Send a embe_config_items message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  cmd.
 */
static inline void ttalink_embe_config_items_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd)
{
    _ttalink_embe_config_items_send(dst_addr, src_addr, chan, cmd, true);
}


static inline void _ttalink_embe_config_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_embe_config_items_t* embe_config_items, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_embe_config_items_send_nocrc(dst_addr, src_addr, chan, embe_config_items->cmd);
    }else{
        ttalink_embe_config_items_send(dst_addr, src_addr, chan, embe_config_items->cmd);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS, (const char *)embe_config_items, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a embe_config_items message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_embe_config_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_embe_config_items_t* embe_config_items)
{
    _ttalink_embe_config_items_send_struct(dst_addr, src_addr, chan, embe_config_items, false);
}

/**
 * @brief Send a embe_config_items message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_embe_config_items_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_embe_config_items_t* embe_config_items)
{
    _ttalink_embe_config_items_send_struct(dst_addr, src_addr, chan, embe_config_items, true);
}

#if TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_embe_config_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, cmd);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS, buf, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_CRC, nocrc);
#else
    ttalink_embe_config_items_t *packet = (ttalink_embe_config_items_t *)msgbuf;
    packet->cmd = cmd;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS, (const char *)packet, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_embe_config_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd)
{
    _ttalink_embe_config_items_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_embe_config_items_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd)
{
    _ttalink_embe_config_items_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, true);
}
#endif

#endif

// MESSAGE EMBE_CONFIG_ITEMS UNPACKING


/**
 * @brief Get field cmd from embe_config_items message
 *
 * @return  cmd.
 */
static inline uint8_t ttalink_embe_config_items_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a embe_config_items message into a struct
 *
 * @param msg The message to decode
 * @param embe_config_items C-struct to decode the message contents into
 */
static inline void ttalink_embe_config_items_decode(const ttalink_message_t* msg, ttalink_embe_config_items_t* embe_config_items)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    embe_config_items->cmd = ttalink_embe_config_items_get_cmd(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN? msg->len : TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN;
        memset(embe_config_items, 0, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_LEN);
    memcpy(embe_config_items, _TTA_PAYLOAD(msg), len);
#endif
}
