#pragma once
// MESSAGE POD_258_GET_VERSION PACKING

#define TTALINK_MSG_ID_POD_258_GET_VERSION 10258

TTAPACKED(
typedef struct __ttalink_pod_258_get_version_t {
 uint8_t cmd; /*<  .*/
 uint8_t ack; /*<  .*/
 uint32_t ack_version; /*<  Software version,Note :  0xaabbccdd represents version aa.bb.cc.dd.*/
}) ttalink_pod_258_get_version_t;

#define TTALINK_MSG_ID_POD_258_GET_VERSION_LEN 6
#define TTALINK_MSG_ID_POD_258_GET_VERSION_MIN_LEN 6
#define TTALINK_MSG_ID_10258_LEN 6
#define TTALINK_MSG_ID_10258_MIN_LEN 6

#define TTALINK_MSG_ID_POD_258_GET_VERSION_CRC 225
#define TTALINK_MSG_ID_10258_CRC 225



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_258_GET_VERSION { \
    10258, \
    "POD_258_GET_VERSION", \
    3, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_258_get_version_t, cmd) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_258_get_version_t, ack) }, \
         { "ack_version", NULL, TTALINK_TYPE_UINT32_T, 0, 2, offsetof(ttalink_pod_258_get_version_t, ack_version) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_258_GET_VERSION { \
    "POD_258_GET_VERSION", \
    3, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_258_get_version_t, cmd) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_258_get_version_t, ack) }, \
         { "ack_version", NULL, TTALINK_TYPE_UINT32_T, 0, 2, offsetof(ttalink_pod_258_get_version_t, ack_version) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_258_get_version_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t ack, uint32_t ack_version, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_258_GET_VERSION_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, ack);
    _tta_put_uint32_t(buf, 2, ack_version);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_258_GET_VERSION_LEN);
#else
    ttalink_pod_258_get_version_t packet;
    packet.cmd = cmd;
    packet.ack = ack;
    packet.ack_version = ack_version;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_258_GET_VERSION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_258_GET_VERSION;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_258_GET_VERSION_MIN_LEN, TTALINK_MSG_ID_POD_258_GET_VERSION_LEN, TTALINK_MSG_ID_POD_258_GET_VERSION_CRC, nocrc);
}

/**
 * @brief Pack a pod_258_get_version message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  .
 * @param ack  .
 * @param ack_version  Software version,Note :  0xaabbccdd represents version aa.bb.cc.dd.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_258_get_version_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t ack, uint32_t ack_version)
{
    return _ttalink_pod_258_get_version_pack(dst_addr, src_addr, msg,  cmd, ack, ack_version, false);
}

/**
 * @brief Pack a pod_258_get_version message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  .
 * @param ack  .
 * @param ack_version  Software version,Note :  0xaabbccdd represents version aa.bb.cc.dd.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_258_get_version_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t ack, uint32_t ack_version)
{
    return _ttalink_pod_258_get_version_pack(dst_addr, src_addr, msg,  cmd, ack, ack_version, true);
}


static inline uint16_t _ttalink_pod_258_get_version_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t ack,uint32_t ack_version, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_258_GET_VERSION_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, ack);
    _tta_put_uint32_t(buf, 2, ack_version);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_258_GET_VERSION_LEN);
#else
    ttalink_pod_258_get_version_t packet;
    packet.cmd = cmd;
    packet.ack = ack;
    packet.ack_version = ack_version;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_258_GET_VERSION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_258_GET_VERSION;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_258_GET_VERSION_MIN_LEN, TTALINK_MSG_ID_POD_258_GET_VERSION_LEN, TTALINK_MSG_ID_POD_258_GET_VERSION_CRC, nocrc);
}

/**
 * @brief Pack a pod_258_get_version message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  .
 * @param ack  .
 * @param ack_version  Software version,Note :  0xaabbccdd represents version aa.bb.cc.dd.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_258_get_version_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t ack,uint32_t ack_version)
{
    return _ttalink_pod_258_get_version_pack_chan(dst_addr, src_addr, chan, msg,  cmd, ack, ack_version, false);
}

/**
 * @brief Pack a pod_258_get_version message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  .
 * @param ack  .
 * @param ack_version  Software version,Note :  0xaabbccdd represents version aa.bb.cc.dd.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_258_get_version_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t ack,uint32_t ack_version)
{
    return _ttalink_pod_258_get_version_pack_chan(dst_addr, src_addr, chan, msg,  cmd, ack, ack_version, true);
}


static inline uint16_t _ttalink_pod_258_get_version_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_258_get_version_t* pod_258_get_version, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_258_get_version_pack_nocrc(dst_addr, src_addr, msg, pod_258_get_version->cmd, pod_258_get_version->ack, pod_258_get_version->ack_version);
    }else{
        return ttalink_pod_258_get_version_pack(dst_addr, src_addr, msg, pod_258_get_version->cmd, pod_258_get_version->ack, pod_258_get_version->ack_version);
    }
    
}

/**
 * @brief Encode a pod_258_get_version struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_258_get_version C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_258_get_version_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_258_get_version_t* pod_258_get_version)
{
    return _ttalink_pod_258_get_version_encode(dst_addr, src_addr, msg, pod_258_get_version, false);
}

/**
 * @brief Encode a pod_258_get_version struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_258_get_version C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_258_get_version_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_258_get_version_t* pod_258_get_version)
{
    return _ttalink_pod_258_get_version_encode(dst_addr, src_addr, msg, pod_258_get_version, true);
}


static inline uint16_t _ttalink_pod_258_get_version_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_258_get_version_t* pod_258_get_version, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_258_get_version_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_258_get_version->cmd, pod_258_get_version->ack, pod_258_get_version->ack_version);
    }else{
        return ttalink_pod_258_get_version_pack_chan(dst_addr, src_addr, chan, msg, pod_258_get_version->cmd, pod_258_get_version->ack, pod_258_get_version->ack_version);
    }
}

/**
 * @brief Encode a pod_258_get_version struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_258_get_version C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_258_get_version_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_258_get_version_t* pod_258_get_version)
{
    return _ttalink_pod_258_get_version_encode_chan(dst_addr, src_addr, chan, msg, pod_258_get_version, false);
}

/**
 * @brief Encode a pod_258_get_version struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_258_get_version C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_258_get_version_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_258_get_version_t* pod_258_get_version)
{
    return _ttalink_pod_258_get_version_encode_chan(dst_addr, src_addr, chan, msg, pod_258_get_version, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_258_get_version_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t ack, uint32_t ack_version, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_258_GET_VERSION_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, ack);
    _tta_put_uint32_t(buf, 2, ack_version);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_258_GET_VERSION, buf, TTALINK_MSG_ID_POD_258_GET_VERSION_MIN_LEN, TTALINK_MSG_ID_POD_258_GET_VERSION_LEN, TTALINK_MSG_ID_POD_258_GET_VERSION_CRC, nocrc);
#else
    ttalink_pod_258_get_version_t packet;
    packet.cmd = cmd;
    packet.ack = ack;
    packet.ack_version = ack_version;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_258_GET_VERSION, (const char *)&packet, TTALINK_MSG_ID_POD_258_GET_VERSION_MIN_LEN, TTALINK_MSG_ID_POD_258_GET_VERSION_LEN, TTALINK_MSG_ID_POD_258_GET_VERSION_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_258_get_version message
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  .
 * @param ack  .
 * @param ack_version  Software version,Note :  0xaabbccdd represents version aa.bb.cc.dd.
 */
static inline void ttalink_pod_258_get_version_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t ack, uint32_t ack_version)
{
    _ttalink_pod_258_get_version_send(dst_addr, src_addr, chan, cmd, ack, ack_version, false);
}

/**
 * @brief Send a pod_258_get_version message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  .
 * @param ack  .
 * @param ack_version  Software version,Note :  0xaabbccdd represents version aa.bb.cc.dd.
 */
static inline void ttalink_pod_258_get_version_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t ack, uint32_t ack_version)
{
    _ttalink_pod_258_get_version_send(dst_addr, src_addr, chan, cmd, ack, ack_version, true);
}


static inline void _ttalink_pod_258_get_version_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_258_get_version_t* pod_258_get_version, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_258_get_version_send_nocrc(dst_addr, src_addr, chan, pod_258_get_version->cmd, pod_258_get_version->ack, pod_258_get_version->ack_version);
    }else{
        ttalink_pod_258_get_version_send(dst_addr, src_addr, chan, pod_258_get_version->cmd, pod_258_get_version->ack, pod_258_get_version->ack_version);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_258_GET_VERSION, (const char *)pod_258_get_version, TTALINK_MSG_ID_POD_258_GET_VERSION_MIN_LEN, TTALINK_MSG_ID_POD_258_GET_VERSION_LEN, TTALINK_MSG_ID_POD_258_GET_VERSION_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_258_get_version message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_258_get_version_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_258_get_version_t* pod_258_get_version)
{
    _ttalink_pod_258_get_version_send_struct(dst_addr, src_addr, chan, pod_258_get_version, false);
}

/**
 * @brief Send a pod_258_get_version message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_258_get_version_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_258_get_version_t* pod_258_get_version)
{
    _ttalink_pod_258_get_version_send_struct(dst_addr, src_addr, chan, pod_258_get_version, true);
}

#if TTALINK_MSG_ID_POD_258_GET_VERSION_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_258_get_version_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t ack, uint32_t ack_version, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, ack);
    _tta_put_uint32_t(buf, 2, ack_version);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_258_GET_VERSION, buf, TTALINK_MSG_ID_POD_258_GET_VERSION_MIN_LEN, TTALINK_MSG_ID_POD_258_GET_VERSION_LEN, TTALINK_MSG_ID_POD_258_GET_VERSION_CRC, nocrc);
#else
    ttalink_pod_258_get_version_t *packet = (ttalink_pod_258_get_version_t *)msgbuf;
    packet->cmd = cmd;
    packet->ack = ack;
    packet->ack_version = ack_version;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_258_GET_VERSION, (const char *)packet, TTALINK_MSG_ID_POD_258_GET_VERSION_MIN_LEN, TTALINK_MSG_ID_POD_258_GET_VERSION_LEN, TTALINK_MSG_ID_POD_258_GET_VERSION_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_258_get_version_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t ack, uint32_t ack_version)
{
    _ttalink_pod_258_get_version_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, ack, ack_version, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_258_get_version_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t ack, uint32_t ack_version)
{
    _ttalink_pod_258_get_version_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, ack, ack_version, true);
}
#endif

#endif

// MESSAGE POD_258_GET_VERSION UNPACKING


/**
 * @brief Get field cmd from pod_258_get_version message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_258_get_version_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack from pod_258_get_version message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_258_get_version_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field ack_version from pod_258_get_version message
 *
 * @return  Software version,Note :  0xaabbccdd represents version aa.bb.cc.dd.
 */
static inline uint32_t ttalink_pod_258_get_version_get_ack_version(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  2);
}

/**
 * @brief Decode a pod_258_get_version message into a struct
 *
 * @param msg The message to decode
 * @param pod_258_get_version C-struct to decode the message contents into
 */
static inline void ttalink_pod_258_get_version_decode(const ttalink_message_t* msg, ttalink_pod_258_get_version_t* pod_258_get_version)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_258_get_version->cmd = ttalink_pod_258_get_version_get_cmd(msg);
    pod_258_get_version->ack = ttalink_pod_258_get_version_get_ack(msg);
    pod_258_get_version->ack_version = ttalink_pod_258_get_version_get_ack_version(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_258_GET_VERSION_LEN? msg->len : TTALINK_MSG_ID_POD_258_GET_VERSION_LEN;
        memset(pod_258_get_version, 0, TTALINK_MSG_ID_POD_258_GET_VERSION_LEN);
    memcpy(pod_258_get_version, _TTA_PAYLOAD(msg), len);
#endif
}
