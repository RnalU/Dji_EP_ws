#pragma once
// MESSAGE GENERAL_REQUEST_VERSION_ACK PACKING

#define TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK 4105

TTAPACKED(
typedef struct __ttalink_general_request_version_ack_t {
 int64_t sn; /*< 设备序列号.*/
 uint8_t device_type; /*< 设备类型.*/
 int8_t version[64]; /*<  .*/
 uint8_t ack; /*< This is ack.*/
}) ttalink_general_request_version_ack_t;

#define TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN 74
#define TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_MIN_LEN 74
#define TTALINK_MSG_ID_4105_LEN 74
#define TTALINK_MSG_ID_4105_MIN_LEN 74

#define TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_CRC 234
#define TTALINK_MSG_ID_4105_CRC 234

#define TTALINK_MSG_GENERAL_REQUEST_VERSION_ACK_FIELD_VERSION_LEN 64

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_GENERAL_REQUEST_VERSION_ACK { \
    4105, \
    "GENERAL_REQUEST_VERSION_ACK", \
    4, \
    {  { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 0, offsetof(ttalink_general_request_version_ack_t, sn) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_general_request_version_ack_t, device_type) }, \
         { "version", NULL, TTALINK_TYPE_INT8_T, 64, 9, offsetof(ttalink_general_request_version_ack_t, version) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 73, offsetof(ttalink_general_request_version_ack_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_GENERAL_REQUEST_VERSION_ACK { \
    "GENERAL_REQUEST_VERSION_ACK", \
    4, \
    {  { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 0, offsetof(ttalink_general_request_version_ack_t, sn) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_general_request_version_ack_t, device_type) }, \
         { "version", NULL, TTALINK_TYPE_INT8_T, 64, 9, offsetof(ttalink_general_request_version_ack_t, version) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 73, offsetof(ttalink_general_request_version_ack_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_general_request_version_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t sn, uint8_t device_type, const int8_t *version, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN];
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_uint8_t(buf, 73, ack);
    _tta_put_int8_t_array(buf, 9, version, 64);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN);
#else
    ttalink_general_request_version_ack_t packet;
    packet.sn = sn;
    packet.device_type = device_type;
    packet.ack = ack;
    tta_array_memcpy(packet.version, version, sizeof(int8_t)*64);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_MIN_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_CRC, nocrc);
}

/**
 * @brief Pack a general_request_version_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param version  .
 * @param ack This is ack.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_request_version_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t sn, uint8_t device_type, const int8_t *version, uint8_t ack)
{
    return _ttalink_general_request_version_ack_pack(dst_addr, src_addr, msg,  sn, device_type, version, ack, false);
}

/**
 * @brief Pack a general_request_version_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param version  .
 * @param ack This is ack.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_request_version_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t sn, uint8_t device_type, const int8_t *version, uint8_t ack)
{
    return _ttalink_general_request_version_ack_pack(dst_addr, src_addr, msg,  sn, device_type, version, ack, true);
}


static inline uint16_t _ttalink_general_request_version_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t sn,uint8_t device_type,const int8_t *version,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN];
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_uint8_t(buf, 73, ack);
    _tta_put_int8_t_array(buf, 9, version, 64);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN);
#else
    ttalink_general_request_version_ack_t packet;
    packet.sn = sn;
    packet.device_type = device_type;
    packet.ack = ack;
    tta_array_memcpy(packet.version, version, sizeof(int8_t)*64);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_MIN_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_CRC, nocrc);
}

/**
 * @brief Pack a general_request_version_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param version  .
 * @param ack This is ack.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_request_version_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t sn,uint8_t device_type,const int8_t *version,uint8_t ack)
{
    return _ttalink_general_request_version_ack_pack_chan(dst_addr, src_addr, chan, msg,  sn, device_type, version, ack, false);
}

/**
 * @brief Pack a general_request_version_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param version  .
 * @param ack This is ack.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_request_version_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t sn,uint8_t device_type,const int8_t *version,uint8_t ack)
{
    return _ttalink_general_request_version_ack_pack_chan(dst_addr, src_addr, chan, msg,  sn, device_type, version, ack, true);
}


static inline uint16_t _ttalink_general_request_version_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_request_version_ack_t* general_request_version_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_general_request_version_ack_pack_nocrc(dst_addr, src_addr, msg, general_request_version_ack->sn, general_request_version_ack->device_type, general_request_version_ack->version, general_request_version_ack->ack);
    }else{
        return ttalink_general_request_version_ack_pack(dst_addr, src_addr, msg, general_request_version_ack->sn, general_request_version_ack->device_type, general_request_version_ack->version, general_request_version_ack->ack);
    }
    
}

/**
 * @brief Encode a general_request_version_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_request_version_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_request_version_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_request_version_ack_t* general_request_version_ack)
{
    return _ttalink_general_request_version_ack_encode(dst_addr, src_addr, msg, general_request_version_ack, false);
}

/**
 * @brief Encode a general_request_version_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_request_version_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_request_version_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_request_version_ack_t* general_request_version_ack)
{
    return _ttalink_general_request_version_ack_encode(dst_addr, src_addr, msg, general_request_version_ack, true);
}


static inline uint16_t _ttalink_general_request_version_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_request_version_ack_t* general_request_version_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_general_request_version_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, general_request_version_ack->sn, general_request_version_ack->device_type, general_request_version_ack->version, general_request_version_ack->ack);
    }else{
        return ttalink_general_request_version_ack_pack_chan(dst_addr, src_addr, chan, msg, general_request_version_ack->sn, general_request_version_ack->device_type, general_request_version_ack->version, general_request_version_ack->ack);
    }
}

/**
 * @brief Encode a general_request_version_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_request_version_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_request_version_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_request_version_ack_t* general_request_version_ack)
{
    return _ttalink_general_request_version_ack_encode_chan(dst_addr, src_addr, chan, msg, general_request_version_ack, false);
}

/**
 * @brief Encode a general_request_version_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_request_version_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_request_version_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_request_version_ack_t* general_request_version_ack)
{
    return _ttalink_general_request_version_ack_encode_chan(dst_addr, src_addr, chan, msg, general_request_version_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_general_request_version_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t sn, uint8_t device_type, const int8_t *version, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN];
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_uint8_t(buf, 73, ack);
    _tta_put_int8_t_array(buf, 9, version, 64);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK, buf, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_MIN_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_CRC, nocrc);
#else
    ttalink_general_request_version_ack_t packet;
    packet.sn = sn;
    packet.device_type = device_type;
    packet.ack = ack;
    tta_array_memcpy(packet.version, version, sizeof(int8_t)*64);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK, (const char *)&packet, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_MIN_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_request_version_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param version  .
 * @param ack This is ack.
 */
static inline void ttalink_general_request_version_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t sn, uint8_t device_type, const int8_t *version, uint8_t ack)
{
    _ttalink_general_request_version_ack_send(dst_addr, src_addr, chan, sn, device_type, version, ack, false);
}

/**
 * @brief Send a general_request_version_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param version  .
 * @param ack This is ack.
 */
static inline void ttalink_general_request_version_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t sn, uint8_t device_type, const int8_t *version, uint8_t ack)
{
    _ttalink_general_request_version_ack_send(dst_addr, src_addr, chan, sn, device_type, version, ack, true);
}


static inline void _ttalink_general_request_version_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_request_version_ack_t* general_request_version_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_general_request_version_ack_send_nocrc(dst_addr, src_addr, chan, general_request_version_ack->sn, general_request_version_ack->device_type, general_request_version_ack->version, general_request_version_ack->ack);
    }else{
        ttalink_general_request_version_ack_send(dst_addr, src_addr, chan, general_request_version_ack->sn, general_request_version_ack->device_type, general_request_version_ack->version, general_request_version_ack->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK, (const char *)general_request_version_ack, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_MIN_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_request_version_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_request_version_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_request_version_ack_t* general_request_version_ack)
{
    _ttalink_general_request_version_ack_send_struct(dst_addr, src_addr, chan, general_request_version_ack, false);
}

/**
 * @brief Send a general_request_version_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_request_version_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_request_version_ack_t* general_request_version_ack)
{
    _ttalink_general_request_version_ack_send_struct(dst_addr, src_addr, chan, general_request_version_ack, true);
}

#if TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_general_request_version_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t sn, uint8_t device_type, const int8_t *version, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_uint8_t(buf, 73, ack);
    _tta_put_int8_t_array(buf, 9, version, 64);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK, buf, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_MIN_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_CRC, nocrc);
#else
    ttalink_general_request_version_ack_t *packet = (ttalink_general_request_version_ack_t *)msgbuf;
    packet->sn = sn;
    packet->device_type = device_type;
    packet->ack = ack;
    tta_array_memcpy(packet->version, version, sizeof(int8_t)*64);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK, (const char *)packet, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_MIN_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_general_request_version_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t sn, uint8_t device_type, const int8_t *version, uint8_t ack)
{
    _ttalink_general_request_version_ack_send_buf(dst_addr, src_addr, msgbuf, chan, sn, device_type, version, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_general_request_version_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t sn, uint8_t device_type, const int8_t *version, uint8_t ack)
{
    _ttalink_general_request_version_ack_send_buf(dst_addr, src_addr, msgbuf, chan, sn, device_type, version, ack, true);
}
#endif

#endif

// MESSAGE GENERAL_REQUEST_VERSION_ACK UNPACKING


/**
 * @brief Get field sn from general_request_version_ack message
 *
 * @return 设备序列号.
 */
static inline int64_t ttalink_general_request_version_ack_get_sn(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  0);
}

/**
 * @brief Get field device_type from general_request_version_ack message
 *
 * @return 设备类型.
 */
static inline uint8_t ttalink_general_request_version_ack_get_device_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field version from general_request_version_ack message
 *
 * @return  .
 */
static inline uint16_t ttalink_general_request_version_ack_get_version(const ttalink_message_t* msg, int8_t *version)
{
    return _TTA_RETURN_int8_t_array(msg, version, 64,  9);
}

/**
 * @brief Get field ack from general_request_version_ack message
 *
 * @return This is ack.
 */
static inline uint8_t ttalink_general_request_version_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  73);
}

/**
 * @brief Decode a general_request_version_ack message into a struct
 *
 * @param msg The message to decode
 * @param general_request_version_ack C-struct to decode the message contents into
 */
static inline void ttalink_general_request_version_ack_decode(const ttalink_message_t* msg, ttalink_general_request_version_ack_t* general_request_version_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    general_request_version_ack->sn = ttalink_general_request_version_ack_get_sn(msg);
    general_request_version_ack->device_type = ttalink_general_request_version_ack_get_device_type(msg);
    ttalink_general_request_version_ack_get_version(msg, general_request_version_ack->version);
    general_request_version_ack->ack = ttalink_general_request_version_ack_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN? msg->len : TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN;
        memset(general_request_version_ack, 0, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_ACK_LEN);
    memcpy(general_request_version_ack, _TTA_PAYLOAD(msg), len);
#endif
}
