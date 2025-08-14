#pragma once
// MESSAGE REQUEST_MODULE_VERSION_ACK PACKING

#define TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK 2221

TTAPACKED(
typedef struct __ttalink_request_module_version_ack_t {
 uint8_t module_type; /*<  .*/
 int8_t version[64]; /*<  .*/
 uint8_t ack; /*< This is ack.*/
}) ttalink_request_module_version_ack_t;

#define TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN 66
#define TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_MIN_LEN 66
#define TTALINK_MSG_ID_2221_LEN 66
#define TTALINK_MSG_ID_2221_MIN_LEN 66

#define TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_CRC 196
#define TTALINK_MSG_ID_2221_CRC 196

#define TTALINK_MSG_REQUEST_MODULE_VERSION_ACK_FIELD_VERSION_LEN 64

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_REQUEST_MODULE_VERSION_ACK { \
    2221, \
    "REQUEST_MODULE_VERSION_ACK", \
    3, \
    {  { "module_type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_request_module_version_ack_t, module_type) }, \
         { "version", NULL, TTALINK_TYPE_INT8_T, 64, 1, offsetof(ttalink_request_module_version_ack_t, version) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 65, offsetof(ttalink_request_module_version_ack_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_REQUEST_MODULE_VERSION_ACK { \
    "REQUEST_MODULE_VERSION_ACK", \
    3, \
    {  { "module_type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_request_module_version_ack_t, module_type) }, \
         { "version", NULL, TTALINK_TYPE_INT8_T, 64, 1, offsetof(ttalink_request_module_version_ack_t, version) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 65, offsetof(ttalink_request_module_version_ack_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_request_module_version_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t module_type, const int8_t *version, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN];
    _tta_put_uint8_t(buf, 0, module_type);
    _tta_put_uint8_t(buf, 65, ack);
    _tta_put_int8_t_array(buf, 1, version, 64);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN);
#else
    ttalink_request_module_version_ack_t packet;
    packet.module_type = module_type;
    packet.ack = ack;
    tta_array_memcpy(packet.version, version, sizeof(int8_t)*64);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_CRC, nocrc);
}

/**
 * @brief Pack a request_module_version_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param module_type  .
 * @param version  .
 * @param ack This is ack.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_module_version_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t module_type, const int8_t *version, uint8_t ack)
{
    return _ttalink_request_module_version_ack_pack(dst_addr, src_addr, msg,  module_type, version, ack, false);
}

/**
 * @brief Pack a request_module_version_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param module_type  .
 * @param version  .
 * @param ack This is ack.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_module_version_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t module_type, const int8_t *version, uint8_t ack)
{
    return _ttalink_request_module_version_ack_pack(dst_addr, src_addr, msg,  module_type, version, ack, true);
}


static inline uint16_t _ttalink_request_module_version_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t module_type,const int8_t *version,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN];
    _tta_put_uint8_t(buf, 0, module_type);
    _tta_put_uint8_t(buf, 65, ack);
    _tta_put_int8_t_array(buf, 1, version, 64);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN);
#else
    ttalink_request_module_version_ack_t packet;
    packet.module_type = module_type;
    packet.ack = ack;
    tta_array_memcpy(packet.version, version, sizeof(int8_t)*64);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_CRC, nocrc);
}

/**
 * @brief Pack a request_module_version_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param module_type  .
 * @param version  .
 * @param ack This is ack.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_module_version_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t module_type,const int8_t *version,uint8_t ack)
{
    return _ttalink_request_module_version_ack_pack_chan(dst_addr, src_addr, chan, msg,  module_type, version, ack, false);
}

/**
 * @brief Pack a request_module_version_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param module_type  .
 * @param version  .
 * @param ack This is ack.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_module_version_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t module_type,const int8_t *version,uint8_t ack)
{
    return _ttalink_request_module_version_ack_pack_chan(dst_addr, src_addr, chan, msg,  module_type, version, ack, true);
}


static inline uint16_t _ttalink_request_module_version_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_module_version_ack_t* request_module_version_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_request_module_version_ack_pack_nocrc(dst_addr, src_addr, msg, request_module_version_ack->module_type, request_module_version_ack->version, request_module_version_ack->ack);
    }else{
        return ttalink_request_module_version_ack_pack(dst_addr, src_addr, msg, request_module_version_ack->module_type, request_module_version_ack->version, request_module_version_ack->ack);
    }
    
}

/**
 * @brief Encode a request_module_version_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_module_version_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_module_version_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_module_version_ack_t* request_module_version_ack)
{
    return _ttalink_request_module_version_ack_encode(dst_addr, src_addr, msg, request_module_version_ack, false);
}

/**
 * @brief Encode a request_module_version_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_module_version_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_module_version_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_module_version_ack_t* request_module_version_ack)
{
    return _ttalink_request_module_version_ack_encode(dst_addr, src_addr, msg, request_module_version_ack, true);
}


static inline uint16_t _ttalink_request_module_version_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_module_version_ack_t* request_module_version_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_request_module_version_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, request_module_version_ack->module_type, request_module_version_ack->version, request_module_version_ack->ack);
    }else{
        return ttalink_request_module_version_ack_pack_chan(dst_addr, src_addr, chan, msg, request_module_version_ack->module_type, request_module_version_ack->version, request_module_version_ack->ack);
    }
}

/**
 * @brief Encode a request_module_version_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_module_version_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_module_version_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_module_version_ack_t* request_module_version_ack)
{
    return _ttalink_request_module_version_ack_encode_chan(dst_addr, src_addr, chan, msg, request_module_version_ack, false);
}

/**
 * @brief Encode a request_module_version_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_module_version_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_module_version_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_module_version_ack_t* request_module_version_ack)
{
    return _ttalink_request_module_version_ack_encode_chan(dst_addr, src_addr, chan, msg, request_module_version_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_request_module_version_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t module_type, const int8_t *version, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN];
    _tta_put_uint8_t(buf, 0, module_type);
    _tta_put_uint8_t(buf, 65, ack);
    _tta_put_int8_t_array(buf, 1, version, 64);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK, buf, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_CRC, nocrc);
#else
    ttalink_request_module_version_ack_t packet;
    packet.module_type = module_type;
    packet.ack = ack;
    tta_array_memcpy(packet.version, version, sizeof(int8_t)*64);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK, (const char *)&packet, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_module_version_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param module_type  .
 * @param version  .
 * @param ack This is ack.
 */
static inline void ttalink_request_module_version_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t module_type, const int8_t *version, uint8_t ack)
{
    _ttalink_request_module_version_ack_send(dst_addr, src_addr, chan, module_type, version, ack, false);
}

/**
 * @brief Send a request_module_version_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param module_type  .
 * @param version  .
 * @param ack This is ack.
 */
static inline void ttalink_request_module_version_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t module_type, const int8_t *version, uint8_t ack)
{
    _ttalink_request_module_version_ack_send(dst_addr, src_addr, chan, module_type, version, ack, true);
}


static inline void _ttalink_request_module_version_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_module_version_ack_t* request_module_version_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_request_module_version_ack_send_nocrc(dst_addr, src_addr, chan, request_module_version_ack->module_type, request_module_version_ack->version, request_module_version_ack->ack);
    }else{
        ttalink_request_module_version_ack_send(dst_addr, src_addr, chan, request_module_version_ack->module_type, request_module_version_ack->version, request_module_version_ack->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK, (const char *)request_module_version_ack, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_module_version_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_module_version_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_module_version_ack_t* request_module_version_ack)
{
    _ttalink_request_module_version_ack_send_struct(dst_addr, src_addr, chan, request_module_version_ack, false);
}

/**
 * @brief Send a request_module_version_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_module_version_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_module_version_ack_t* request_module_version_ack)
{
    _ttalink_request_module_version_ack_send_struct(dst_addr, src_addr, chan, request_module_version_ack, true);
}

#if TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_request_module_version_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t module_type, const int8_t *version, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, module_type);
    _tta_put_uint8_t(buf, 65, ack);
    _tta_put_int8_t_array(buf, 1, version, 64);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK, buf, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_CRC, nocrc);
#else
    ttalink_request_module_version_ack_t *packet = (ttalink_request_module_version_ack_t *)msgbuf;
    packet->module_type = module_type;
    packet->ack = ack;
    tta_array_memcpy(packet->version, version, sizeof(int8_t)*64);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK, (const char *)packet, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_request_module_version_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t module_type, const int8_t *version, uint8_t ack)
{
    _ttalink_request_module_version_ack_send_buf(dst_addr, src_addr, msgbuf, chan, module_type, version, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_request_module_version_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t module_type, const int8_t *version, uint8_t ack)
{
    _ttalink_request_module_version_ack_send_buf(dst_addr, src_addr, msgbuf, chan, module_type, version, ack, true);
}
#endif

#endif

// MESSAGE REQUEST_MODULE_VERSION_ACK UNPACKING


/**
 * @brief Get field module_type from request_module_version_ack message
 *
 * @return  .
 */
static inline uint8_t ttalink_request_module_version_ack_get_module_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field version from request_module_version_ack message
 *
 * @return  .
 */
static inline uint16_t ttalink_request_module_version_ack_get_version(const ttalink_message_t* msg, int8_t *version)
{
    return _TTA_RETURN_int8_t_array(msg, version, 64,  1);
}

/**
 * @brief Get field ack from request_module_version_ack message
 *
 * @return This is ack.
 */
static inline uint8_t ttalink_request_module_version_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  65);
}

/**
 * @brief Decode a request_module_version_ack message into a struct
 *
 * @param msg The message to decode
 * @param request_module_version_ack C-struct to decode the message contents into
 */
static inline void ttalink_request_module_version_ack_decode(const ttalink_message_t* msg, ttalink_request_module_version_ack_t* request_module_version_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    request_module_version_ack->module_type = ttalink_request_module_version_ack_get_module_type(msg);
    ttalink_request_module_version_ack_get_version(msg, request_module_version_ack->version);
    request_module_version_ack->ack = ttalink_request_module_version_ack_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN? msg->len : TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN;
        memset(request_module_version_ack, 0, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_ACK_LEN);
    memcpy(request_module_version_ack, _TTA_PAYLOAD(msg), len);
#endif
}
