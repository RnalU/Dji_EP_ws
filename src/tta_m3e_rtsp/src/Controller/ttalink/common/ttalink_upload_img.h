#pragma once
// MESSAGE UPLOAD_IMG PACKING

#define TTALINK_MSG_ID_UPLOAD_IMG 2224

TTAPACKED(
typedef struct __ttalink_upload_img_t {
 uint32_t count; /*<  .*/
 uint16_t size; /*<  .*/
 uint8_t data[512]; /*<  .*/
}) ttalink_upload_img_t;

#define TTALINK_MSG_ID_UPLOAD_IMG_LEN 518
#define TTALINK_MSG_ID_UPLOAD_IMG_MIN_LEN 518
#define TTALINK_MSG_ID_2224_LEN 518
#define TTALINK_MSG_ID_2224_MIN_LEN 518

#define TTALINK_MSG_ID_UPLOAD_IMG_CRC 201
#define TTALINK_MSG_ID_2224_CRC 201

#define TTALINK_MSG_UPLOAD_IMG_FIELD_DATA_LEN 512

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_UPLOAD_IMG { \
    2224, \
    "UPLOAD_IMG", \
    3, \
    {  { "count", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_upload_img_t, count) }, \
         { "size", NULL, TTALINK_TYPE_UINT16_T, 0, 4, offsetof(ttalink_upload_img_t, size) }, \
         { "data", NULL, TTALINK_TYPE_UINT8_T, 512, 6, offsetof(ttalink_upload_img_t, data) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_UPLOAD_IMG { \
    "UPLOAD_IMG", \
    3, \
    {  { "count", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_upload_img_t, count) }, \
         { "size", NULL, TTALINK_TYPE_UINT16_T, 0, 4, offsetof(ttalink_upload_img_t, size) }, \
         { "data", NULL, TTALINK_TYPE_UINT8_T, 512, 6, offsetof(ttalink_upload_img_t, data) }, \
         } \
}
#endif


static inline uint16_t _ttalink_upload_img_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t count, uint16_t size, const uint8_t *data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_UPLOAD_IMG_LEN];
    _tta_put_uint32_t(buf, 0, count);
    _tta_put_uint16_t(buf, 4, size);
    _tta_put_uint8_t_array(buf, 6, data, 512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_UPLOAD_IMG_LEN);
#else
    ttalink_upload_img_t packet;
    packet.count = count;
    packet.size = size;
    tta_array_memcpy(packet.data, data, sizeof(uint8_t)*512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_UPLOAD_IMG_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_UPLOAD_IMG;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_UPLOAD_IMG_MIN_LEN, TTALINK_MSG_ID_UPLOAD_IMG_LEN, TTALINK_MSG_ID_UPLOAD_IMG_CRC, nocrc);
}

/**
 * @brief Pack a upload_img message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param count  .
 * @param size  .
 * @param data  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_upload_img_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t count, uint16_t size, const uint8_t *data)
{
    return _ttalink_upload_img_pack(dst_addr, src_addr, msg,  count, size, data, false);
}

/**
 * @brief Pack a upload_img message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param count  .
 * @param size  .
 * @param data  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_upload_img_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t count, uint16_t size, const uint8_t *data)
{
    return _ttalink_upload_img_pack(dst_addr, src_addr, msg,  count, size, data, true);
}


static inline uint16_t _ttalink_upload_img_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t count,uint16_t size,const uint8_t *data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_UPLOAD_IMG_LEN];
    _tta_put_uint32_t(buf, 0, count);
    _tta_put_uint16_t(buf, 4, size);
    _tta_put_uint8_t_array(buf, 6, data, 512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_UPLOAD_IMG_LEN);
#else
    ttalink_upload_img_t packet;
    packet.count = count;
    packet.size = size;
    tta_array_memcpy(packet.data, data, sizeof(uint8_t)*512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_UPLOAD_IMG_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_UPLOAD_IMG;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_UPLOAD_IMG_MIN_LEN, TTALINK_MSG_ID_UPLOAD_IMG_LEN, TTALINK_MSG_ID_UPLOAD_IMG_CRC, nocrc);
}

/**
 * @brief Pack a upload_img message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param count  .
 * @param size  .
 * @param data  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_upload_img_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t count,uint16_t size,const uint8_t *data)
{
    return _ttalink_upload_img_pack_chan(dst_addr, src_addr, chan, msg,  count, size, data, false);
}

/**
 * @brief Pack a upload_img message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param count  .
 * @param size  .
 * @param data  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_upload_img_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t count,uint16_t size,const uint8_t *data)
{
    return _ttalink_upload_img_pack_chan(dst_addr, src_addr, chan, msg,  count, size, data, true);
}


static inline uint16_t _ttalink_upload_img_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_upload_img_t* upload_img, bool nocrc)
{
    if(nocrc){
        return ttalink_upload_img_pack_nocrc(dst_addr, src_addr, msg, upload_img->count, upload_img->size, upload_img->data);
    }else{
        return ttalink_upload_img_pack(dst_addr, src_addr, msg, upload_img->count, upload_img->size, upload_img->data);
    }
    
}

/**
 * @brief Encode a upload_img struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param upload_img C-struct to read the message contents from
 */
static inline uint16_t ttalink_upload_img_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_upload_img_t* upload_img)
{
    return _ttalink_upload_img_encode(dst_addr, src_addr, msg, upload_img, false);
}

/**
 * @brief Encode a upload_img struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param upload_img C-struct to read the message contents from
 */
static inline uint16_t ttalink_upload_img_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_upload_img_t* upload_img)
{
    return _ttalink_upload_img_encode(dst_addr, src_addr, msg, upload_img, true);
}


static inline uint16_t _ttalink_upload_img_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_upload_img_t* upload_img, bool nocrc)
{
    if(nocrc){
        return ttalink_upload_img_pack_chan_nocrc(dst_addr, src_addr, chan, msg, upload_img->count, upload_img->size, upload_img->data);
    }else{
        return ttalink_upload_img_pack_chan(dst_addr, src_addr, chan, msg, upload_img->count, upload_img->size, upload_img->data);
    }
}

/**
 * @brief Encode a upload_img struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param upload_img C-struct to read the message contents from
 */
static inline uint16_t ttalink_upload_img_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_upload_img_t* upload_img)
{
    return _ttalink_upload_img_encode_chan(dst_addr, src_addr, chan, msg, upload_img, false);
}

/**
 * @brief Encode a upload_img struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param upload_img C-struct to read the message contents from
 */
static inline uint16_t ttalink_upload_img_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_upload_img_t* upload_img)
{
    return _ttalink_upload_img_encode_chan(dst_addr, src_addr, chan, msg, upload_img, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_upload_img_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t count, uint16_t size, const uint8_t *data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_UPLOAD_IMG_LEN];
    _tta_put_uint32_t(buf, 0, count);
    _tta_put_uint16_t(buf, 4, size);
    _tta_put_uint8_t_array(buf, 6, data, 512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPLOAD_IMG, buf, TTALINK_MSG_ID_UPLOAD_IMG_MIN_LEN, TTALINK_MSG_ID_UPLOAD_IMG_LEN, TTALINK_MSG_ID_UPLOAD_IMG_CRC, nocrc);
#else
    ttalink_upload_img_t packet;
    packet.count = count;
    packet.size = size;
    tta_array_memcpy(packet.data, data, sizeof(uint8_t)*512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPLOAD_IMG, (const char *)&packet, TTALINK_MSG_ID_UPLOAD_IMG_MIN_LEN, TTALINK_MSG_ID_UPLOAD_IMG_LEN, TTALINK_MSG_ID_UPLOAD_IMG_CRC, nocrc);
#endif
}

/**
 * @brief Send a upload_img message
 * @param chan TTAlink channel to send the message
 *
 * @param count  .
 * @param size  .
 * @param data  .
 */
static inline void ttalink_upload_img_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t count, uint16_t size, const uint8_t *data)
{
    _ttalink_upload_img_send(dst_addr, src_addr, chan, count, size, data, false);
}

/**
 * @brief Send a upload_img message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param count  .
 * @param size  .
 * @param data  .
 */
static inline void ttalink_upload_img_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t count, uint16_t size, const uint8_t *data)
{
    _ttalink_upload_img_send(dst_addr, src_addr, chan, count, size, data, true);
}


static inline void _ttalink_upload_img_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_upload_img_t* upload_img, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_upload_img_send_nocrc(dst_addr, src_addr, chan, upload_img->count, upload_img->size, upload_img->data);
    }else{
        ttalink_upload_img_send(dst_addr, src_addr, chan, upload_img->count, upload_img->size, upload_img->data);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPLOAD_IMG, (const char *)upload_img, TTALINK_MSG_ID_UPLOAD_IMG_MIN_LEN, TTALINK_MSG_ID_UPLOAD_IMG_LEN, TTALINK_MSG_ID_UPLOAD_IMG_CRC, nocrc);
#endif
}

/**
 * @brief Send a upload_img message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_upload_img_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_upload_img_t* upload_img)
{
    _ttalink_upload_img_send_struct(dst_addr, src_addr, chan, upload_img, false);
}

/**
 * @brief Send a upload_img message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_upload_img_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_upload_img_t* upload_img)
{
    _ttalink_upload_img_send_struct(dst_addr, src_addr, chan, upload_img, true);
}

#if TTALINK_MSG_ID_UPLOAD_IMG_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_upload_img_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t count, uint16_t size, const uint8_t *data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, count);
    _tta_put_uint16_t(buf, 4, size);
    _tta_put_uint8_t_array(buf, 6, data, 512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPLOAD_IMG, buf, TTALINK_MSG_ID_UPLOAD_IMG_MIN_LEN, TTALINK_MSG_ID_UPLOAD_IMG_LEN, TTALINK_MSG_ID_UPLOAD_IMG_CRC, nocrc);
#else
    ttalink_upload_img_t *packet = (ttalink_upload_img_t *)msgbuf;
    packet->count = count;
    packet->size = size;
    tta_array_memcpy(packet->data, data, sizeof(uint8_t)*512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPLOAD_IMG, (const char *)packet, TTALINK_MSG_ID_UPLOAD_IMG_MIN_LEN, TTALINK_MSG_ID_UPLOAD_IMG_LEN, TTALINK_MSG_ID_UPLOAD_IMG_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_upload_img_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t count, uint16_t size, const uint8_t *data)
{
    _ttalink_upload_img_send_buf(dst_addr, src_addr, msgbuf, chan, count, size, data, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_upload_img_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t count, uint16_t size, const uint8_t *data)
{
    _ttalink_upload_img_send_buf(dst_addr, src_addr, msgbuf, chan, count, size, data, true);
}
#endif

#endif

// MESSAGE UPLOAD_IMG UNPACKING


/**
 * @brief Get field count from upload_img message
 *
 * @return  .
 */
static inline uint32_t ttalink_upload_img_get_count(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field size from upload_img message
 *
 * @return  .
 */
static inline uint16_t ttalink_upload_img_get_size(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field data from upload_img message
 *
 * @return  .
 */
static inline uint16_t ttalink_upload_img_get_data(const ttalink_message_t* msg, uint8_t *data)
{
    return _TTA_RETURN_uint8_t_array(msg, data, 512,  6);
}

/**
 * @brief Decode a upload_img message into a struct
 *
 * @param msg The message to decode
 * @param upload_img C-struct to decode the message contents into
 */
static inline void ttalink_upload_img_decode(const ttalink_message_t* msg, ttalink_upload_img_t* upload_img)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    upload_img->count = ttalink_upload_img_get_count(msg);
    upload_img->size = ttalink_upload_img_get_size(msg);
    ttalink_upload_img_get_data(msg, upload_img->data);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_UPLOAD_IMG_LEN? msg->len : TTALINK_MSG_ID_UPLOAD_IMG_LEN;
        memset(upload_img, 0, TTALINK_MSG_ID_UPLOAD_IMG_LEN);
    memcpy(upload_img, _TTA_PAYLOAD(msg), len);
#endif
}
