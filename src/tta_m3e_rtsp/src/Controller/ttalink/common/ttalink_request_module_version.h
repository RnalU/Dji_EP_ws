#pragma once
// MESSAGE REQUEST_MODULE_VERSION PACKING

#define TTALINK_MSG_ID_REQUEST_MODULE_VERSION 2220

TTAPACKED(
typedef struct __ttalink_request_module_version_t {
 uint8_t module_type; /*<  .*/
}) ttalink_request_module_version_t;

#define TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN 1
#define TTALINK_MSG_ID_REQUEST_MODULE_VERSION_MIN_LEN 1
#define TTALINK_MSG_ID_2220_LEN 1
#define TTALINK_MSG_ID_2220_MIN_LEN 1

#define TTALINK_MSG_ID_REQUEST_MODULE_VERSION_CRC 229
#define TTALINK_MSG_ID_2220_CRC 229



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_REQUEST_MODULE_VERSION { \
    2220, \
    "REQUEST_MODULE_VERSION", \
    1, \
    {  { "module_type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_request_module_version_t, module_type) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_REQUEST_MODULE_VERSION { \
    "REQUEST_MODULE_VERSION", \
    1, \
    {  { "module_type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_request_module_version_t, module_type) }, \
         } \
}
#endif


static inline uint16_t _ttalink_request_module_version_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t module_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN];
    _tta_put_uint8_t(buf, 0, module_type);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN);
#else
    ttalink_request_module_version_t packet;
    packet.module_type = module_type;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_MODULE_VERSION;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_MIN_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_CRC, nocrc);
}

/**
 * @brief Pack a request_module_version message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param module_type  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_module_version_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t module_type)
{
    return _ttalink_request_module_version_pack(dst_addr, src_addr, msg,  module_type, false);
}

/**
 * @brief Pack a request_module_version message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param module_type  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_module_version_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t module_type)
{
    return _ttalink_request_module_version_pack(dst_addr, src_addr, msg,  module_type, true);
}


static inline uint16_t _ttalink_request_module_version_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t module_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN];
    _tta_put_uint8_t(buf, 0, module_type);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN);
#else
    ttalink_request_module_version_t packet;
    packet.module_type = module_type;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_MODULE_VERSION;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_MIN_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_CRC, nocrc);
}

/**
 * @brief Pack a request_module_version message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param module_type  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_module_version_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t module_type)
{
    return _ttalink_request_module_version_pack_chan(dst_addr, src_addr, chan, msg,  module_type, false);
}

/**
 * @brief Pack a request_module_version message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param module_type  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_module_version_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t module_type)
{
    return _ttalink_request_module_version_pack_chan(dst_addr, src_addr, chan, msg,  module_type, true);
}


static inline uint16_t _ttalink_request_module_version_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_module_version_t* request_module_version, bool nocrc)
{
    if(nocrc){
        return ttalink_request_module_version_pack_nocrc(dst_addr, src_addr, msg, request_module_version->module_type);
    }else{
        return ttalink_request_module_version_pack(dst_addr, src_addr, msg, request_module_version->module_type);
    }
    
}

/**
 * @brief Encode a request_module_version struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_module_version C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_module_version_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_module_version_t* request_module_version)
{
    return _ttalink_request_module_version_encode(dst_addr, src_addr, msg, request_module_version, false);
}

/**
 * @brief Encode a request_module_version struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_module_version C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_module_version_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_module_version_t* request_module_version)
{
    return _ttalink_request_module_version_encode(dst_addr, src_addr, msg, request_module_version, true);
}


static inline uint16_t _ttalink_request_module_version_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_module_version_t* request_module_version, bool nocrc)
{
    if(nocrc){
        return ttalink_request_module_version_pack_chan_nocrc(dst_addr, src_addr, chan, msg, request_module_version->module_type);
    }else{
        return ttalink_request_module_version_pack_chan(dst_addr, src_addr, chan, msg, request_module_version->module_type);
    }
}

/**
 * @brief Encode a request_module_version struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_module_version C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_module_version_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_module_version_t* request_module_version)
{
    return _ttalink_request_module_version_encode_chan(dst_addr, src_addr, chan, msg, request_module_version, false);
}

/**
 * @brief Encode a request_module_version struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_module_version C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_module_version_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_module_version_t* request_module_version)
{
    return _ttalink_request_module_version_encode_chan(dst_addr, src_addr, chan, msg, request_module_version, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_request_module_version_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t module_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN];
    _tta_put_uint8_t(buf, 0, module_type);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_MODULE_VERSION, buf, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_MIN_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_CRC, nocrc);
#else
    ttalink_request_module_version_t packet;
    packet.module_type = module_type;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_MODULE_VERSION, (const char *)&packet, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_MIN_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_module_version message
 * @param chan TTAlink channel to send the message
 *
 * @param module_type  .
 */
static inline void ttalink_request_module_version_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t module_type)
{
    _ttalink_request_module_version_send(dst_addr, src_addr, chan, module_type, false);
}

/**
 * @brief Send a request_module_version message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param module_type  .
 */
static inline void ttalink_request_module_version_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t module_type)
{
    _ttalink_request_module_version_send(dst_addr, src_addr, chan, module_type, true);
}


static inline void _ttalink_request_module_version_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_module_version_t* request_module_version, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_request_module_version_send_nocrc(dst_addr, src_addr, chan, request_module_version->module_type);
    }else{
        ttalink_request_module_version_send(dst_addr, src_addr, chan, request_module_version->module_type);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_MODULE_VERSION, (const char *)request_module_version, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_MIN_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_module_version message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_module_version_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_module_version_t* request_module_version)
{
    _ttalink_request_module_version_send_struct(dst_addr, src_addr, chan, request_module_version, false);
}

/**
 * @brief Send a request_module_version message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_module_version_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_module_version_t* request_module_version)
{
    _ttalink_request_module_version_send_struct(dst_addr, src_addr, chan, request_module_version, true);
}

#if TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_request_module_version_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t module_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, module_type);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_MODULE_VERSION, buf, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_MIN_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_CRC, nocrc);
#else
    ttalink_request_module_version_t *packet = (ttalink_request_module_version_t *)msgbuf;
    packet->module_type = module_type;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_MODULE_VERSION, (const char *)packet, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_MIN_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_request_module_version_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t module_type)
{
    _ttalink_request_module_version_send_buf(dst_addr, src_addr, msgbuf, chan, module_type, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_request_module_version_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t module_type)
{
    _ttalink_request_module_version_send_buf(dst_addr, src_addr, msgbuf, chan, module_type, true);
}
#endif

#endif

// MESSAGE REQUEST_MODULE_VERSION UNPACKING


/**
 * @brief Get field module_type from request_module_version message
 *
 * @return  .
 */
static inline uint8_t ttalink_request_module_version_get_module_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a request_module_version message into a struct
 *
 * @param msg The message to decode
 * @param request_module_version C-struct to decode the message contents into
 */
static inline void ttalink_request_module_version_decode(const ttalink_message_t* msg, ttalink_request_module_version_t* request_module_version)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    request_module_version->module_type = ttalink_request_module_version_get_module_type(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN? msg->len : TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN;
        memset(request_module_version, 0, TTALINK_MSG_ID_REQUEST_MODULE_VERSION_LEN);
    memcpy(request_module_version, _TTA_PAYLOAD(msg), len);
#endif
}
