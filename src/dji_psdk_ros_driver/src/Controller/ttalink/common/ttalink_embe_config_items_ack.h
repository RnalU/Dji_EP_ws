#pragma once
// MESSAGE EMBE_CONFIG_ITEMS_ACK PACKING

#define TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK 2083

TTAPACKED(
typedef struct __ttalink_embe_config_items_ack_t {
 uint8_t ack; /*<  .*/
}) ttalink_embe_config_items_ack_t;

#define TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN 1
#define TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_MIN_LEN 1
#define TTALINK_MSG_ID_2083_LEN 1
#define TTALINK_MSG_ID_2083_MIN_LEN 1

#define TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_CRC 0
#define TTALINK_MSG_ID_2083_CRC 0



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_EMBE_CONFIG_ITEMS_ACK { \
    2083, \
    "EMBE_CONFIG_ITEMS_ACK", \
    1, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_embe_config_items_ack_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_EMBE_CONFIG_ITEMS_ACK { \
    "EMBE_CONFIG_ITEMS_ACK", \
    1, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_embe_config_items_ack_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_embe_config_items_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN];
    _tta_put_uint8_t(buf, 0, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN);
#else
    ttalink_embe_config_items_ack_t packet;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_MIN_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_CRC, nocrc);
}

/**
 * @brief Pack a embe_config_items_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_embe_config_items_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack)
{
    return _ttalink_embe_config_items_ack_pack(dst_addr, src_addr, msg,  ack, false);
}

/**
 * @brief Pack a embe_config_items_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_embe_config_items_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack)
{
    return _ttalink_embe_config_items_ack_pack(dst_addr, src_addr, msg,  ack, true);
}


static inline uint16_t _ttalink_embe_config_items_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN];
    _tta_put_uint8_t(buf, 0, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN);
#else
    ttalink_embe_config_items_ack_t packet;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_MIN_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_CRC, nocrc);
}

/**
 * @brief Pack a embe_config_items_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_embe_config_items_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack)
{
    return _ttalink_embe_config_items_ack_pack_chan(dst_addr, src_addr, chan, msg,  ack, false);
}

/**
 * @brief Pack a embe_config_items_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_embe_config_items_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack)
{
    return _ttalink_embe_config_items_ack_pack_chan(dst_addr, src_addr, chan, msg,  ack, true);
}


static inline uint16_t _ttalink_embe_config_items_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_embe_config_items_ack_t* embe_config_items_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_embe_config_items_ack_pack_nocrc(dst_addr, src_addr, msg, embe_config_items_ack->ack);
    }else{
        return ttalink_embe_config_items_ack_pack(dst_addr, src_addr, msg, embe_config_items_ack->ack);
    }
    
}

/**
 * @brief Encode a embe_config_items_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param embe_config_items_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_embe_config_items_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_embe_config_items_ack_t* embe_config_items_ack)
{
    return _ttalink_embe_config_items_ack_encode(dst_addr, src_addr, msg, embe_config_items_ack, false);
}

/**
 * @brief Encode a embe_config_items_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param embe_config_items_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_embe_config_items_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_embe_config_items_ack_t* embe_config_items_ack)
{
    return _ttalink_embe_config_items_ack_encode(dst_addr, src_addr, msg, embe_config_items_ack, true);
}


static inline uint16_t _ttalink_embe_config_items_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_embe_config_items_ack_t* embe_config_items_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_embe_config_items_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, embe_config_items_ack->ack);
    }else{
        return ttalink_embe_config_items_ack_pack_chan(dst_addr, src_addr, chan, msg, embe_config_items_ack->ack);
    }
}

/**
 * @brief Encode a embe_config_items_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param embe_config_items_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_embe_config_items_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_embe_config_items_ack_t* embe_config_items_ack)
{
    return _ttalink_embe_config_items_ack_encode_chan(dst_addr, src_addr, chan, msg, embe_config_items_ack, false);
}

/**
 * @brief Encode a embe_config_items_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param embe_config_items_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_embe_config_items_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_embe_config_items_ack_t* embe_config_items_ack)
{
    return _ttalink_embe_config_items_ack_encode_chan(dst_addr, src_addr, chan, msg, embe_config_items_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_embe_config_items_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN];
    _tta_put_uint8_t(buf, 0, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK, buf, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_MIN_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_CRC, nocrc);
#else
    ttalink_embe_config_items_ack_t packet;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK, (const char *)&packet, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_MIN_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a embe_config_items_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param ack  .
 */
static inline void ttalink_embe_config_items_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack)
{
    _ttalink_embe_config_items_ack_send(dst_addr, src_addr, chan, ack, false);
}

/**
 * @brief Send a embe_config_items_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack  .
 */
static inline void ttalink_embe_config_items_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack)
{
    _ttalink_embe_config_items_ack_send(dst_addr, src_addr, chan, ack, true);
}


static inline void _ttalink_embe_config_items_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_embe_config_items_ack_t* embe_config_items_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_embe_config_items_ack_send_nocrc(dst_addr, src_addr, chan, embe_config_items_ack->ack);
    }else{
        ttalink_embe_config_items_ack_send(dst_addr, src_addr, chan, embe_config_items_ack->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK, (const char *)embe_config_items_ack, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_MIN_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a embe_config_items_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_embe_config_items_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_embe_config_items_ack_t* embe_config_items_ack)
{
    _ttalink_embe_config_items_ack_send_struct(dst_addr, src_addr, chan, embe_config_items_ack, false);
}

/**
 * @brief Send a embe_config_items_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_embe_config_items_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_embe_config_items_ack_t* embe_config_items_ack)
{
    _ttalink_embe_config_items_ack_send_struct(dst_addr, src_addr, chan, embe_config_items_ack, true);
}

#if TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_embe_config_items_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK, buf, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_MIN_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_CRC, nocrc);
#else
    ttalink_embe_config_items_ack_t *packet = (ttalink_embe_config_items_ack_t *)msgbuf;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK, (const char *)packet, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_MIN_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_embe_config_items_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack)
{
    _ttalink_embe_config_items_ack_send_buf(dst_addr, src_addr, msgbuf, chan, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_embe_config_items_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack)
{
    _ttalink_embe_config_items_ack_send_buf(dst_addr, src_addr, msgbuf, chan, ack, true);
}
#endif

#endif

// MESSAGE EMBE_CONFIG_ITEMS_ACK UNPACKING


/**
 * @brief Get field ack from embe_config_items_ack message
 *
 * @return  .
 */
static inline uint8_t ttalink_embe_config_items_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a embe_config_items_ack message into a struct
 *
 * @param msg The message to decode
 * @param embe_config_items_ack C-struct to decode the message contents into
 */
static inline void ttalink_embe_config_items_ack_decode(const ttalink_message_t* msg, ttalink_embe_config_items_ack_t* embe_config_items_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    embe_config_items_ack->ack = ttalink_embe_config_items_ack_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN? msg->len : TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN;
        memset(embe_config_items_ack, 0, TTALINK_MSG_ID_EMBE_CONFIG_ITEMS_ACK_LEN);
    memcpy(embe_config_items_ack, _TTA_PAYLOAD(msg), len);
#endif
}
