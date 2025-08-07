#pragma once
// MESSAGE COM_ACK PACKING

#define TTALINK_MSG_ID_COM_ACK 2252

TTAPACKED(
typedef struct __ttalink_com_ack_t {
 uint16_t com_msgid; /*<  .*/
 uint32_t com; /*<  .*/
 uint16_t ack; /*<  .*/
}) ttalink_com_ack_t;

#define TTALINK_MSG_ID_COM_ACK_LEN 8
#define TTALINK_MSG_ID_COM_ACK_MIN_LEN 8
#define TTALINK_MSG_ID_2252_LEN 8
#define TTALINK_MSG_ID_2252_MIN_LEN 8

#define TTALINK_MSG_ID_COM_ACK_CRC 182
#define TTALINK_MSG_ID_2252_CRC 182



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_COM_ACK { \
    2252, \
    "COM_ACK", \
    3, \
    {  { "com_msgid", NULL, TTALINK_TYPE_UINT16_T, 0, 0, offsetof(ttalink_com_ack_t, com_msgid) }, \
         { "com", NULL, TTALINK_TYPE_UINT32_T, 0, 2, offsetof(ttalink_com_ack_t, com) }, \
         { "ack", NULL, TTALINK_TYPE_UINT16_T, 0, 6, offsetof(ttalink_com_ack_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_COM_ACK { \
    "COM_ACK", \
    3, \
    {  { "com_msgid", NULL, TTALINK_TYPE_UINT16_T, 0, 0, offsetof(ttalink_com_ack_t, com_msgid) }, \
         { "com", NULL, TTALINK_TYPE_UINT32_T, 0, 2, offsetof(ttalink_com_ack_t, com) }, \
         { "ack", NULL, TTALINK_TYPE_UINT16_T, 0, 6, offsetof(ttalink_com_ack_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_com_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t com_msgid, uint32_t com, uint16_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_COM_ACK_LEN];
    _tta_put_uint16_t(buf, 0, com_msgid);
    _tta_put_uint32_t(buf, 2, com);
    _tta_put_uint16_t(buf, 6, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_COM_ACK_LEN);
#else
    ttalink_com_ack_t packet;
    packet.com_msgid = com_msgid;
    packet.com = com;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_COM_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_COM_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_COM_ACK_MIN_LEN, TTALINK_MSG_ID_COM_ACK_LEN, TTALINK_MSG_ID_COM_ACK_CRC, nocrc);
}

/**
 * @brief Pack a com_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param com_msgid  .
 * @param com  .
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_com_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t com_msgid, uint32_t com, uint16_t ack)
{
    return _ttalink_com_ack_pack(dst_addr, src_addr, msg,  com_msgid, com, ack, false);
}

/**
 * @brief Pack a com_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param com_msgid  .
 * @param com  .
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_com_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t com_msgid, uint32_t com, uint16_t ack)
{
    return _ttalink_com_ack_pack(dst_addr, src_addr, msg,  com_msgid, com, ack, true);
}


static inline uint16_t _ttalink_com_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t com_msgid,uint32_t com,uint16_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_COM_ACK_LEN];
    _tta_put_uint16_t(buf, 0, com_msgid);
    _tta_put_uint32_t(buf, 2, com);
    _tta_put_uint16_t(buf, 6, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_COM_ACK_LEN);
#else
    ttalink_com_ack_t packet;
    packet.com_msgid = com_msgid;
    packet.com = com;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_COM_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_COM_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_COM_ACK_MIN_LEN, TTALINK_MSG_ID_COM_ACK_LEN, TTALINK_MSG_ID_COM_ACK_CRC, nocrc);
}

/**
 * @brief Pack a com_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param com_msgid  .
 * @param com  .
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_com_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t com_msgid,uint32_t com,uint16_t ack)
{
    return _ttalink_com_ack_pack_chan(dst_addr, src_addr, chan, msg,  com_msgid, com, ack, false);
}

/**
 * @brief Pack a com_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param com_msgid  .
 * @param com  .
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_com_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t com_msgid,uint32_t com,uint16_t ack)
{
    return _ttalink_com_ack_pack_chan(dst_addr, src_addr, chan, msg,  com_msgid, com, ack, true);
}


static inline uint16_t _ttalink_com_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_com_ack_t* com_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_com_ack_pack_nocrc(dst_addr, src_addr, msg, com_ack->com_msgid, com_ack->com, com_ack->ack);
    }else{
        return ttalink_com_ack_pack(dst_addr, src_addr, msg, com_ack->com_msgid, com_ack->com, com_ack->ack);
    }
    
}

/**
 * @brief Encode a com_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param com_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_com_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_com_ack_t* com_ack)
{
    return _ttalink_com_ack_encode(dst_addr, src_addr, msg, com_ack, false);
}

/**
 * @brief Encode a com_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param com_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_com_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_com_ack_t* com_ack)
{
    return _ttalink_com_ack_encode(dst_addr, src_addr, msg, com_ack, true);
}


static inline uint16_t _ttalink_com_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_com_ack_t* com_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_com_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, com_ack->com_msgid, com_ack->com, com_ack->ack);
    }else{
        return ttalink_com_ack_pack_chan(dst_addr, src_addr, chan, msg, com_ack->com_msgid, com_ack->com, com_ack->ack);
    }
}

/**
 * @brief Encode a com_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param com_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_com_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_com_ack_t* com_ack)
{
    return _ttalink_com_ack_encode_chan(dst_addr, src_addr, chan, msg, com_ack, false);
}

/**
 * @brief Encode a com_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param com_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_com_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_com_ack_t* com_ack)
{
    return _ttalink_com_ack_encode_chan(dst_addr, src_addr, chan, msg, com_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_com_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t com_msgid, uint32_t com, uint16_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_COM_ACK_LEN];
    _tta_put_uint16_t(buf, 0, com_msgid);
    _tta_put_uint32_t(buf, 2, com);
    _tta_put_uint16_t(buf, 6, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_COM_ACK, buf, TTALINK_MSG_ID_COM_ACK_MIN_LEN, TTALINK_MSG_ID_COM_ACK_LEN, TTALINK_MSG_ID_COM_ACK_CRC, nocrc);
#else
    ttalink_com_ack_t packet;
    packet.com_msgid = com_msgid;
    packet.com = com;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_COM_ACK, (const char *)&packet, TTALINK_MSG_ID_COM_ACK_MIN_LEN, TTALINK_MSG_ID_COM_ACK_LEN, TTALINK_MSG_ID_COM_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a com_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param com_msgid  .
 * @param com  .
 * @param ack  .
 */
static inline void ttalink_com_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t com_msgid, uint32_t com, uint16_t ack)
{
    _ttalink_com_ack_send(dst_addr, src_addr, chan, com_msgid, com, ack, false);
}

/**
 * @brief Send a com_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param com_msgid  .
 * @param com  .
 * @param ack  .
 */
static inline void ttalink_com_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t com_msgid, uint32_t com, uint16_t ack)
{
    _ttalink_com_ack_send(dst_addr, src_addr, chan, com_msgid, com, ack, true);
}


static inline void _ttalink_com_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_com_ack_t* com_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_com_ack_send_nocrc(dst_addr, src_addr, chan, com_ack->com_msgid, com_ack->com, com_ack->ack);
    }else{
        ttalink_com_ack_send(dst_addr, src_addr, chan, com_ack->com_msgid, com_ack->com, com_ack->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_COM_ACK, (const char *)com_ack, TTALINK_MSG_ID_COM_ACK_MIN_LEN, TTALINK_MSG_ID_COM_ACK_LEN, TTALINK_MSG_ID_COM_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a com_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_com_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_com_ack_t* com_ack)
{
    _ttalink_com_ack_send_struct(dst_addr, src_addr, chan, com_ack, false);
}

/**
 * @brief Send a com_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_com_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_com_ack_t* com_ack)
{
    _ttalink_com_ack_send_struct(dst_addr, src_addr, chan, com_ack, true);
}

#if TTALINK_MSG_ID_COM_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_com_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t com_msgid, uint32_t com, uint16_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint16_t(buf, 0, com_msgid);
    _tta_put_uint32_t(buf, 2, com);
    _tta_put_uint16_t(buf, 6, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_COM_ACK, buf, TTALINK_MSG_ID_COM_ACK_MIN_LEN, TTALINK_MSG_ID_COM_ACK_LEN, TTALINK_MSG_ID_COM_ACK_CRC, nocrc);
#else
    ttalink_com_ack_t *packet = (ttalink_com_ack_t *)msgbuf;
    packet->com_msgid = com_msgid;
    packet->com = com;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_COM_ACK, (const char *)packet, TTALINK_MSG_ID_COM_ACK_MIN_LEN, TTALINK_MSG_ID_COM_ACK_LEN, TTALINK_MSG_ID_COM_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_com_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t com_msgid, uint32_t com, uint16_t ack)
{
    _ttalink_com_ack_send_buf(dst_addr, src_addr, msgbuf, chan, com_msgid, com, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_com_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t com_msgid, uint32_t com, uint16_t ack)
{
    _ttalink_com_ack_send_buf(dst_addr, src_addr, msgbuf, chan, com_msgid, com, ack, true);
}
#endif

#endif

// MESSAGE COM_ACK UNPACKING


/**
 * @brief Get field com_msgid from com_ack message
 *
 * @return  .
 */
static inline uint16_t ttalink_com_ack_get_com_msgid(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field com from com_ack message
 *
 * @return  .
 */
static inline uint32_t ttalink_com_ack_get_com(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  2);
}

/**
 * @brief Get field ack from com_ack message
 *
 * @return  .
 */
static inline uint16_t ttalink_com_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Decode a com_ack message into a struct
 *
 * @param msg The message to decode
 * @param com_ack C-struct to decode the message contents into
 */
static inline void ttalink_com_ack_decode(const ttalink_message_t* msg, ttalink_com_ack_t* com_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    com_ack->com_msgid = ttalink_com_ack_get_com_msgid(msg);
    com_ack->com = ttalink_com_ack_get_com(msg);
    com_ack->ack = ttalink_com_ack_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_COM_ACK_LEN? msg->len : TTALINK_MSG_ID_COM_ACK_LEN;
        memset(com_ack, 0, TTALINK_MSG_ID_COM_ACK_LEN);
    memcpy(com_ack, _TTA_PAYLOAD(msg), len);
#endif
}
