#pragma once
// MESSAGE REQUEST_UPLOAD_IMG PACKING

#define TTALINK_MSG_ID_REQUEST_UPLOAD_IMG 2222

TTAPACKED(
typedef struct __ttalink_request_upload_img_t {
 uint8_t type; /*<  .*/
 uint32_t count; /*<  .*/
 uint16_t block; /*<  .*/
}) ttalink_request_upload_img_t;

#define TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN 7
#define TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_MIN_LEN 7
#define TTALINK_MSG_ID_2222_LEN 7
#define TTALINK_MSG_ID_2222_MIN_LEN 7

#define TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_CRC 163
#define TTALINK_MSG_ID_2222_CRC 163



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_REQUEST_UPLOAD_IMG { \
    2222, \
    "REQUEST_UPLOAD_IMG", \
    3, \
    {  { "type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_request_upload_img_t, type) }, \
         { "count", NULL, TTALINK_TYPE_UINT32_T, 0, 1, offsetof(ttalink_request_upload_img_t, count) }, \
         { "block", NULL, TTALINK_TYPE_UINT16_T, 0, 5, offsetof(ttalink_request_upload_img_t, block) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_REQUEST_UPLOAD_IMG { \
    "REQUEST_UPLOAD_IMG", \
    3, \
    {  { "type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_request_upload_img_t, type) }, \
         { "count", NULL, TTALINK_TYPE_UINT32_T, 0, 1, offsetof(ttalink_request_upload_img_t, count) }, \
         { "block", NULL, TTALINK_TYPE_UINT16_T, 0, 5, offsetof(ttalink_request_upload_img_t, block) }, \
         } \
}
#endif


static inline uint16_t _ttalink_request_upload_img_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t type, uint32_t count, uint16_t block, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN];
    _tta_put_uint8_t(buf, 0, type);
    _tta_put_uint32_t(buf, 1, count);
    _tta_put_uint16_t(buf, 5, block);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN);
#else
    ttalink_request_upload_img_t packet;
    packet.type = type;
    packet.count = count;
    packet.block = block;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_UPLOAD_IMG;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_MIN_LEN, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_CRC, nocrc);
}

/**
 * @brief Pack a request_upload_img message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param type  .
 * @param count  .
 * @param block  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_upload_img_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t type, uint32_t count, uint16_t block)
{
    return _ttalink_request_upload_img_pack(dst_addr, src_addr, msg,  type, count, block, false);
}

/**
 * @brief Pack a request_upload_img message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param type  .
 * @param count  .
 * @param block  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_upload_img_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t type, uint32_t count, uint16_t block)
{
    return _ttalink_request_upload_img_pack(dst_addr, src_addr, msg,  type, count, block, true);
}


static inline uint16_t _ttalink_request_upload_img_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t type,uint32_t count,uint16_t block, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN];
    _tta_put_uint8_t(buf, 0, type);
    _tta_put_uint32_t(buf, 1, count);
    _tta_put_uint16_t(buf, 5, block);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN);
#else
    ttalink_request_upload_img_t packet;
    packet.type = type;
    packet.count = count;
    packet.block = block;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_UPLOAD_IMG;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_MIN_LEN, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_CRC, nocrc);
}

/**
 * @brief Pack a request_upload_img message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param type  .
 * @param count  .
 * @param block  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_upload_img_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t type,uint32_t count,uint16_t block)
{
    return _ttalink_request_upload_img_pack_chan(dst_addr, src_addr, chan, msg,  type, count, block, false);
}

/**
 * @brief Pack a request_upload_img message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param type  .
 * @param count  .
 * @param block  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_upload_img_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t type,uint32_t count,uint16_t block)
{
    return _ttalink_request_upload_img_pack_chan(dst_addr, src_addr, chan, msg,  type, count, block, true);
}


static inline uint16_t _ttalink_request_upload_img_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_upload_img_t* request_upload_img, bool nocrc)
{
    if(nocrc){
        return ttalink_request_upload_img_pack_nocrc(dst_addr, src_addr, msg, request_upload_img->type, request_upload_img->count, request_upload_img->block);
    }else{
        return ttalink_request_upload_img_pack(dst_addr, src_addr, msg, request_upload_img->type, request_upload_img->count, request_upload_img->block);
    }
    
}

/**
 * @brief Encode a request_upload_img struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_upload_img C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_upload_img_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_upload_img_t* request_upload_img)
{
    return _ttalink_request_upload_img_encode(dst_addr, src_addr, msg, request_upload_img, false);
}

/**
 * @brief Encode a request_upload_img struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_upload_img C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_upload_img_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_upload_img_t* request_upload_img)
{
    return _ttalink_request_upload_img_encode(dst_addr, src_addr, msg, request_upload_img, true);
}


static inline uint16_t _ttalink_request_upload_img_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_upload_img_t* request_upload_img, bool nocrc)
{
    if(nocrc){
        return ttalink_request_upload_img_pack_chan_nocrc(dst_addr, src_addr, chan, msg, request_upload_img->type, request_upload_img->count, request_upload_img->block);
    }else{
        return ttalink_request_upload_img_pack_chan(dst_addr, src_addr, chan, msg, request_upload_img->type, request_upload_img->count, request_upload_img->block);
    }
}

/**
 * @brief Encode a request_upload_img struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_upload_img C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_upload_img_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_upload_img_t* request_upload_img)
{
    return _ttalink_request_upload_img_encode_chan(dst_addr, src_addr, chan, msg, request_upload_img, false);
}

/**
 * @brief Encode a request_upload_img struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_upload_img C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_upload_img_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_upload_img_t* request_upload_img)
{
    return _ttalink_request_upload_img_encode_chan(dst_addr, src_addr, chan, msg, request_upload_img, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_request_upload_img_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t type, uint32_t count, uint16_t block, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN];
    _tta_put_uint8_t(buf, 0, type);
    _tta_put_uint32_t(buf, 1, count);
    _tta_put_uint16_t(buf, 5, block);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG, buf, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_MIN_LEN, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_CRC, nocrc);
#else
    ttalink_request_upload_img_t packet;
    packet.type = type;
    packet.count = count;
    packet.block = block;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG, (const char *)&packet, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_MIN_LEN, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_upload_img message
 * @param chan TTAlink channel to send the message
 *
 * @param type  .
 * @param count  .
 * @param block  .
 */
static inline void ttalink_request_upload_img_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t type, uint32_t count, uint16_t block)
{
    _ttalink_request_upload_img_send(dst_addr, src_addr, chan, type, count, block, false);
}

/**
 * @brief Send a request_upload_img message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param type  .
 * @param count  .
 * @param block  .
 */
static inline void ttalink_request_upload_img_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t type, uint32_t count, uint16_t block)
{
    _ttalink_request_upload_img_send(dst_addr, src_addr, chan, type, count, block, true);
}


static inline void _ttalink_request_upload_img_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_upload_img_t* request_upload_img, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_request_upload_img_send_nocrc(dst_addr, src_addr, chan, request_upload_img->type, request_upload_img->count, request_upload_img->block);
    }else{
        ttalink_request_upload_img_send(dst_addr, src_addr, chan, request_upload_img->type, request_upload_img->count, request_upload_img->block);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG, (const char *)request_upload_img, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_MIN_LEN, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_upload_img message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_upload_img_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_upload_img_t* request_upload_img)
{
    _ttalink_request_upload_img_send_struct(dst_addr, src_addr, chan, request_upload_img, false);
}

/**
 * @brief Send a request_upload_img message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_upload_img_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_upload_img_t* request_upload_img)
{
    _ttalink_request_upload_img_send_struct(dst_addr, src_addr, chan, request_upload_img, true);
}

#if TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_request_upload_img_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t type, uint32_t count, uint16_t block, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, type);
    _tta_put_uint32_t(buf, 1, count);
    _tta_put_uint16_t(buf, 5, block);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG, buf, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_MIN_LEN, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_CRC, nocrc);
#else
    ttalink_request_upload_img_t *packet = (ttalink_request_upload_img_t *)msgbuf;
    packet->type = type;
    packet->count = count;
    packet->block = block;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG, (const char *)packet, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_MIN_LEN, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_request_upload_img_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t type, uint32_t count, uint16_t block)
{
    _ttalink_request_upload_img_send_buf(dst_addr, src_addr, msgbuf, chan, type, count, block, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_request_upload_img_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t type, uint32_t count, uint16_t block)
{
    _ttalink_request_upload_img_send_buf(dst_addr, src_addr, msgbuf, chan, type, count, block, true);
}
#endif

#endif

// MESSAGE REQUEST_UPLOAD_IMG UNPACKING


/**
 * @brief Get field type from request_upload_img message
 *
 * @return  .
 */
static inline uint8_t ttalink_request_upload_img_get_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field count from request_upload_img message
 *
 * @return  .
 */
static inline uint32_t ttalink_request_upload_img_get_count(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  1);
}

/**
 * @brief Get field block from request_upload_img message
 *
 * @return  .
 */
static inline uint16_t ttalink_request_upload_img_get_block(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  5);
}

/**
 * @brief Decode a request_upload_img message into a struct
 *
 * @param msg The message to decode
 * @param request_upload_img C-struct to decode the message contents into
 */
static inline void ttalink_request_upload_img_decode(const ttalink_message_t* msg, ttalink_request_upload_img_t* request_upload_img)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    request_upload_img->type = ttalink_request_upload_img_get_type(msg);
    request_upload_img->count = ttalink_request_upload_img_get_count(msg);
    request_upload_img->block = ttalink_request_upload_img_get_block(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN? msg->len : TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN;
        memset(request_upload_img, 0, TTALINK_MSG_ID_REQUEST_UPLOAD_IMG_LEN);
    memcpy(request_upload_img, _TTA_PAYLOAD(msg), len);
#endif
}
