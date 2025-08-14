#pragma once
// MESSAGE POD_262_GET_PRODUCT_ALIAS PACKING

#define TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS 10262

TTAPACKED(
typedef struct __ttalink_pod_262_get_product_alias_t {
 uint8_t cmd; /*<  .*/
 uint8_t ack; /*<  .*/
 uint8_t ack_alias[32]; /*<  Product alias, with "\ 0" as the end character, the maximum length of product alias is 32 bytes.*/
}) ttalink_pod_262_get_product_alias_t;

#define TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN 34
#define TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_MIN_LEN 34
#define TTALINK_MSG_ID_10262_LEN 34
#define TTALINK_MSG_ID_10262_MIN_LEN 34

#define TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_CRC 32
#define TTALINK_MSG_ID_10262_CRC 32

#define TTALINK_MSG_POD_262_GET_PRODUCT_ALIAS_FIELD_ACK_ALIAS_LEN 32

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_262_GET_PRODUCT_ALIAS { \
    10262, \
    "POD_262_GET_PRODUCT_ALIAS", \
    3, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_262_get_product_alias_t, cmd) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_262_get_product_alias_t, ack) }, \
         { "ack_alias", NULL, TTALINK_TYPE_UINT8_T, 32, 2, offsetof(ttalink_pod_262_get_product_alias_t, ack_alias) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_262_GET_PRODUCT_ALIAS { \
    "POD_262_GET_PRODUCT_ALIAS", \
    3, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_262_get_product_alias_t, cmd) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_262_get_product_alias_t, ack) }, \
         { "ack_alias", NULL, TTALINK_TYPE_UINT8_T, 32, 2, offsetof(ttalink_pod_262_get_product_alias_t, ack_alias) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_262_get_product_alias_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t ack, const uint8_t *ack_alias, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, ack);
    _tta_put_uint8_t_array(buf, 2, ack_alias, 32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN);
#else
    ttalink_pod_262_get_product_alias_t packet;
    packet.cmd = cmd;
    packet.ack = ack;
    tta_array_memcpy(packet.ack_alias, ack_alias, sizeof(uint8_t)*32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_MIN_LEN, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_CRC, nocrc);
}

/**
 * @brief Pack a pod_262_get_product_alias message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  .
 * @param ack  .
 * @param ack_alias  Product alias, with "\ 0" as the end character, the maximum length of product alias is 32 bytes.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_262_get_product_alias_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t ack, const uint8_t *ack_alias)
{
    return _ttalink_pod_262_get_product_alias_pack(dst_addr, src_addr, msg,  cmd, ack, ack_alias, false);
}

/**
 * @brief Pack a pod_262_get_product_alias message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  .
 * @param ack  .
 * @param ack_alias  Product alias, with "\ 0" as the end character, the maximum length of product alias is 32 bytes.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_262_get_product_alias_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t ack, const uint8_t *ack_alias)
{
    return _ttalink_pod_262_get_product_alias_pack(dst_addr, src_addr, msg,  cmd, ack, ack_alias, true);
}


static inline uint16_t _ttalink_pod_262_get_product_alias_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t ack,const uint8_t *ack_alias, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, ack);
    _tta_put_uint8_t_array(buf, 2, ack_alias, 32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN);
#else
    ttalink_pod_262_get_product_alias_t packet;
    packet.cmd = cmd;
    packet.ack = ack;
    tta_array_memcpy(packet.ack_alias, ack_alias, sizeof(uint8_t)*32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_MIN_LEN, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_CRC, nocrc);
}

/**
 * @brief Pack a pod_262_get_product_alias message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  .
 * @param ack  .
 * @param ack_alias  Product alias, with "\ 0" as the end character, the maximum length of product alias is 32 bytes.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_262_get_product_alias_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t ack,const uint8_t *ack_alias)
{
    return _ttalink_pod_262_get_product_alias_pack_chan(dst_addr, src_addr, chan, msg,  cmd, ack, ack_alias, false);
}

/**
 * @brief Pack a pod_262_get_product_alias message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  .
 * @param ack  .
 * @param ack_alias  Product alias, with "\ 0" as the end character, the maximum length of product alias is 32 bytes.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_262_get_product_alias_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t ack,const uint8_t *ack_alias)
{
    return _ttalink_pod_262_get_product_alias_pack_chan(dst_addr, src_addr, chan, msg,  cmd, ack, ack_alias, true);
}


static inline uint16_t _ttalink_pod_262_get_product_alias_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_262_get_product_alias_t* pod_262_get_product_alias, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_262_get_product_alias_pack_nocrc(dst_addr, src_addr, msg, pod_262_get_product_alias->cmd, pod_262_get_product_alias->ack, pod_262_get_product_alias->ack_alias);
    }else{
        return ttalink_pod_262_get_product_alias_pack(dst_addr, src_addr, msg, pod_262_get_product_alias->cmd, pod_262_get_product_alias->ack, pod_262_get_product_alias->ack_alias);
    }
    
}

/**
 * @brief Encode a pod_262_get_product_alias struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_262_get_product_alias C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_262_get_product_alias_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_262_get_product_alias_t* pod_262_get_product_alias)
{
    return _ttalink_pod_262_get_product_alias_encode(dst_addr, src_addr, msg, pod_262_get_product_alias, false);
}

/**
 * @brief Encode a pod_262_get_product_alias struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_262_get_product_alias C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_262_get_product_alias_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_262_get_product_alias_t* pod_262_get_product_alias)
{
    return _ttalink_pod_262_get_product_alias_encode(dst_addr, src_addr, msg, pod_262_get_product_alias, true);
}


static inline uint16_t _ttalink_pod_262_get_product_alias_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_262_get_product_alias_t* pod_262_get_product_alias, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_262_get_product_alias_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_262_get_product_alias->cmd, pod_262_get_product_alias->ack, pod_262_get_product_alias->ack_alias);
    }else{
        return ttalink_pod_262_get_product_alias_pack_chan(dst_addr, src_addr, chan, msg, pod_262_get_product_alias->cmd, pod_262_get_product_alias->ack, pod_262_get_product_alias->ack_alias);
    }
}

/**
 * @brief Encode a pod_262_get_product_alias struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_262_get_product_alias C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_262_get_product_alias_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_262_get_product_alias_t* pod_262_get_product_alias)
{
    return _ttalink_pod_262_get_product_alias_encode_chan(dst_addr, src_addr, chan, msg, pod_262_get_product_alias, false);
}

/**
 * @brief Encode a pod_262_get_product_alias struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_262_get_product_alias C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_262_get_product_alias_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_262_get_product_alias_t* pod_262_get_product_alias)
{
    return _ttalink_pod_262_get_product_alias_encode_chan(dst_addr, src_addr, chan, msg, pod_262_get_product_alias, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_262_get_product_alias_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t ack, const uint8_t *ack_alias, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, ack);
    _tta_put_uint8_t_array(buf, 2, ack_alias, 32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS, buf, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_MIN_LEN, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_CRC, nocrc);
#else
    ttalink_pod_262_get_product_alias_t packet;
    packet.cmd = cmd;
    packet.ack = ack;
    tta_array_memcpy(packet.ack_alias, ack_alias, sizeof(uint8_t)*32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS, (const char *)&packet, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_MIN_LEN, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_262_get_product_alias message
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  .
 * @param ack  .
 * @param ack_alias  Product alias, with "\ 0" as the end character, the maximum length of product alias is 32 bytes.
 */
static inline void ttalink_pod_262_get_product_alias_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t ack, const uint8_t *ack_alias)
{
    _ttalink_pod_262_get_product_alias_send(dst_addr, src_addr, chan, cmd, ack, ack_alias, false);
}

/**
 * @brief Send a pod_262_get_product_alias message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  .
 * @param ack  .
 * @param ack_alias  Product alias, with "\ 0" as the end character, the maximum length of product alias is 32 bytes.
 */
static inline void ttalink_pod_262_get_product_alias_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t ack, const uint8_t *ack_alias)
{
    _ttalink_pod_262_get_product_alias_send(dst_addr, src_addr, chan, cmd, ack, ack_alias, true);
}


static inline void _ttalink_pod_262_get_product_alias_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_262_get_product_alias_t* pod_262_get_product_alias, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_262_get_product_alias_send_nocrc(dst_addr, src_addr, chan, pod_262_get_product_alias->cmd, pod_262_get_product_alias->ack, pod_262_get_product_alias->ack_alias);
    }else{
        ttalink_pod_262_get_product_alias_send(dst_addr, src_addr, chan, pod_262_get_product_alias->cmd, pod_262_get_product_alias->ack, pod_262_get_product_alias->ack_alias);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS, (const char *)pod_262_get_product_alias, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_MIN_LEN, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_262_get_product_alias message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_262_get_product_alias_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_262_get_product_alias_t* pod_262_get_product_alias)
{
    _ttalink_pod_262_get_product_alias_send_struct(dst_addr, src_addr, chan, pod_262_get_product_alias, false);
}

/**
 * @brief Send a pod_262_get_product_alias message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_262_get_product_alias_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_262_get_product_alias_t* pod_262_get_product_alias)
{
    _ttalink_pod_262_get_product_alias_send_struct(dst_addr, src_addr, chan, pod_262_get_product_alias, true);
}

#if TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_262_get_product_alias_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t ack, const uint8_t *ack_alias, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, ack);
    _tta_put_uint8_t_array(buf, 2, ack_alias, 32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS, buf, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_MIN_LEN, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_CRC, nocrc);
#else
    ttalink_pod_262_get_product_alias_t *packet = (ttalink_pod_262_get_product_alias_t *)msgbuf;
    packet->cmd = cmd;
    packet->ack = ack;
    tta_array_memcpy(packet->ack_alias, ack_alias, sizeof(uint8_t)*32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS, (const char *)packet, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_MIN_LEN, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_262_get_product_alias_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t ack, const uint8_t *ack_alias)
{
    _ttalink_pod_262_get_product_alias_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, ack, ack_alias, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_262_get_product_alias_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t ack, const uint8_t *ack_alias)
{
    _ttalink_pod_262_get_product_alias_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, ack, ack_alias, true);
}
#endif

#endif

// MESSAGE POD_262_GET_PRODUCT_ALIAS UNPACKING


/**
 * @brief Get field cmd from pod_262_get_product_alias message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_262_get_product_alias_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack from pod_262_get_product_alias message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_262_get_product_alias_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field ack_alias from pod_262_get_product_alias message
 *
 * @return  Product alias, with "\ 0" as the end character, the maximum length of product alias is 32 bytes.
 */
static inline uint16_t ttalink_pod_262_get_product_alias_get_ack_alias(const ttalink_message_t* msg, uint8_t *ack_alias)
{
    return _TTA_RETURN_uint8_t_array(msg, ack_alias, 32,  2);
}

/**
 * @brief Decode a pod_262_get_product_alias message into a struct
 *
 * @param msg The message to decode
 * @param pod_262_get_product_alias C-struct to decode the message contents into
 */
static inline void ttalink_pod_262_get_product_alias_decode(const ttalink_message_t* msg, ttalink_pod_262_get_product_alias_t* pod_262_get_product_alias)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_262_get_product_alias->cmd = ttalink_pod_262_get_product_alias_get_cmd(msg);
    pod_262_get_product_alias->ack = ttalink_pod_262_get_product_alias_get_ack(msg);
    ttalink_pod_262_get_product_alias_get_ack_alias(msg, pod_262_get_product_alias->ack_alias);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN? msg->len : TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN;
        memset(pod_262_get_product_alias, 0, TTALINK_MSG_ID_POD_262_GET_PRODUCT_ALIAS_LEN);
    memcpy(pod_262_get_product_alias, _TTA_PAYLOAD(msg), len);
#endif
}
