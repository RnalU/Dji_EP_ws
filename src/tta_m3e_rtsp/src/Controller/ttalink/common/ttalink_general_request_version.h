#pragma once
// MESSAGE GENERAL_REQUEST_VERSION PACKING

#define TTALINK_MSG_ID_GENERAL_REQUEST_VERSION 4104

TTAPACKED(
typedef struct __ttalink_general_request_version_t {
 int64_t sn; /*< 设备序列号.*/
 uint8_t device_type; /*< 设备类型.*/
}) ttalink_general_request_version_t;

#define TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN 9
#define TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_MIN_LEN 9
#define TTALINK_MSG_ID_4104_LEN 9
#define TTALINK_MSG_ID_4104_MIN_LEN 9

#define TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_CRC 25
#define TTALINK_MSG_ID_4104_CRC 25



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_GENERAL_REQUEST_VERSION { \
    4104, \
    "GENERAL_REQUEST_VERSION", \
    2, \
    {  { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 0, offsetof(ttalink_general_request_version_t, sn) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_general_request_version_t, device_type) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_GENERAL_REQUEST_VERSION { \
    "GENERAL_REQUEST_VERSION", \
    2, \
    {  { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 0, offsetof(ttalink_general_request_version_t, sn) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_general_request_version_t, device_type) }, \
         } \
}
#endif


static inline uint16_t _ttalink_general_request_version_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t sn, uint8_t device_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN];
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN);
#else
    ttalink_general_request_version_t packet;
    packet.sn = sn;
    packet.device_type = device_type;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_REQUEST_VERSION;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_MIN_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_CRC, nocrc);
}

/**
 * @brief Pack a general_request_version message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_request_version_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t sn, uint8_t device_type)
{
    return _ttalink_general_request_version_pack(dst_addr, src_addr, msg,  sn, device_type, false);
}

/**
 * @brief Pack a general_request_version message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_request_version_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t sn, uint8_t device_type)
{
    return _ttalink_general_request_version_pack(dst_addr, src_addr, msg,  sn, device_type, true);
}


static inline uint16_t _ttalink_general_request_version_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t sn,uint8_t device_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN];
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN);
#else
    ttalink_general_request_version_t packet;
    packet.sn = sn;
    packet.device_type = device_type;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_REQUEST_VERSION;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_MIN_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_CRC, nocrc);
}

/**
 * @brief Pack a general_request_version message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_request_version_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t sn,uint8_t device_type)
{
    return _ttalink_general_request_version_pack_chan(dst_addr, src_addr, chan, msg,  sn, device_type, false);
}

/**
 * @brief Pack a general_request_version message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_request_version_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t sn,uint8_t device_type)
{
    return _ttalink_general_request_version_pack_chan(dst_addr, src_addr, chan, msg,  sn, device_type, true);
}


static inline uint16_t _ttalink_general_request_version_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_request_version_t* general_request_version, bool nocrc)
{
    if(nocrc){
        return ttalink_general_request_version_pack_nocrc(dst_addr, src_addr, msg, general_request_version->sn, general_request_version->device_type);
    }else{
        return ttalink_general_request_version_pack(dst_addr, src_addr, msg, general_request_version->sn, general_request_version->device_type);
    }
    
}

/**
 * @brief Encode a general_request_version struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_request_version C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_request_version_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_request_version_t* general_request_version)
{
    return _ttalink_general_request_version_encode(dst_addr, src_addr, msg, general_request_version, false);
}

/**
 * @brief Encode a general_request_version struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_request_version C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_request_version_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_request_version_t* general_request_version)
{
    return _ttalink_general_request_version_encode(dst_addr, src_addr, msg, general_request_version, true);
}


static inline uint16_t _ttalink_general_request_version_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_request_version_t* general_request_version, bool nocrc)
{
    if(nocrc){
        return ttalink_general_request_version_pack_chan_nocrc(dst_addr, src_addr, chan, msg, general_request_version->sn, general_request_version->device_type);
    }else{
        return ttalink_general_request_version_pack_chan(dst_addr, src_addr, chan, msg, general_request_version->sn, general_request_version->device_type);
    }
}

/**
 * @brief Encode a general_request_version struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_request_version C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_request_version_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_request_version_t* general_request_version)
{
    return _ttalink_general_request_version_encode_chan(dst_addr, src_addr, chan, msg, general_request_version, false);
}

/**
 * @brief Encode a general_request_version struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_request_version C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_request_version_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_request_version_t* general_request_version)
{
    return _ttalink_general_request_version_encode_chan(dst_addr, src_addr, chan, msg, general_request_version, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_general_request_version_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t sn, uint8_t device_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN];
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION, buf, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_MIN_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_CRC, nocrc);
#else
    ttalink_general_request_version_t packet;
    packet.sn = sn;
    packet.device_type = device_type;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION, (const char *)&packet, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_MIN_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_request_version message
 * @param chan TTAlink channel to send the message
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 */
static inline void ttalink_general_request_version_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t sn, uint8_t device_type)
{
    _ttalink_general_request_version_send(dst_addr, src_addr, chan, sn, device_type, false);
}

/**
 * @brief Send a general_request_version message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 */
static inline void ttalink_general_request_version_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t sn, uint8_t device_type)
{
    _ttalink_general_request_version_send(dst_addr, src_addr, chan, sn, device_type, true);
}


static inline void _ttalink_general_request_version_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_request_version_t* general_request_version, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_general_request_version_send_nocrc(dst_addr, src_addr, chan, general_request_version->sn, general_request_version->device_type);
    }else{
        ttalink_general_request_version_send(dst_addr, src_addr, chan, general_request_version->sn, general_request_version->device_type);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION, (const char *)general_request_version, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_MIN_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_request_version message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_request_version_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_request_version_t* general_request_version)
{
    _ttalink_general_request_version_send_struct(dst_addr, src_addr, chan, general_request_version, false);
}

/**
 * @brief Send a general_request_version message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_request_version_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_request_version_t* general_request_version)
{
    _ttalink_general_request_version_send_struct(dst_addr, src_addr, chan, general_request_version, true);
}

#if TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_general_request_version_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t sn, uint8_t device_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION, buf, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_MIN_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_CRC, nocrc);
#else
    ttalink_general_request_version_t *packet = (ttalink_general_request_version_t *)msgbuf;
    packet->sn = sn;
    packet->device_type = device_type;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION, (const char *)packet, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_MIN_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_general_request_version_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t sn, uint8_t device_type)
{
    _ttalink_general_request_version_send_buf(dst_addr, src_addr, msgbuf, chan, sn, device_type, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_general_request_version_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t sn, uint8_t device_type)
{
    _ttalink_general_request_version_send_buf(dst_addr, src_addr, msgbuf, chan, sn, device_type, true);
}
#endif

#endif

// MESSAGE GENERAL_REQUEST_VERSION UNPACKING


/**
 * @brief Get field sn from general_request_version message
 *
 * @return 设备序列号.
 */
static inline int64_t ttalink_general_request_version_get_sn(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  0);
}

/**
 * @brief Get field device_type from general_request_version message
 *
 * @return 设备类型.
 */
static inline uint8_t ttalink_general_request_version_get_device_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Decode a general_request_version message into a struct
 *
 * @param msg The message to decode
 * @param general_request_version C-struct to decode the message contents into
 */
static inline void ttalink_general_request_version_decode(const ttalink_message_t* msg, ttalink_general_request_version_t* general_request_version)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    general_request_version->sn = ttalink_general_request_version_get_sn(msg);
    general_request_version->device_type = ttalink_general_request_version_get_device_type(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN? msg->len : TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN;
        memset(general_request_version, 0, TTALINK_MSG_ID_GENERAL_REQUEST_VERSION_LEN);
    memcpy(general_request_version, _TTA_PAYLOAD(msg), len);
#endif
}
