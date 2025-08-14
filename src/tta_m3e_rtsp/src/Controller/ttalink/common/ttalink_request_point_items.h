#pragma once
// MESSAGE REQUEST_POINT_ITEMS PACKING

#define TTALINK_MSG_ID_REQUEST_POINT_ITEMS 2186

TTAPACKED(
typedef struct __ttalink_request_point_items_t {
 uint8_t mode; /*<  0=home点 1=归航点 2=指点飞行点 3=环绕点 4=兴趣点 5=航线断点 6=AB航线点断点. */
 float param1; /*<   . .*/
}) ttalink_request_point_items_t;

#define TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN 5
#define TTALINK_MSG_ID_REQUEST_POINT_ITEMS_MIN_LEN 5
#define TTALINK_MSG_ID_2186_LEN 5
#define TTALINK_MSG_ID_2186_MIN_LEN 5

#define TTALINK_MSG_ID_REQUEST_POINT_ITEMS_CRC 13
#define TTALINK_MSG_ID_2186_CRC 13



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_REQUEST_POINT_ITEMS { \
    2186, \
    "REQUEST_POINT_ITEMS", \
    2, \
    {  { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_request_point_items_t, mode) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 1, offsetof(ttalink_request_point_items_t, param1) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_REQUEST_POINT_ITEMS { \
    "REQUEST_POINT_ITEMS", \
    2, \
    {  { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_request_point_items_t, mode) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 1, offsetof(ttalink_request_point_items_t, param1) }, \
         } \
}
#endif


static inline uint16_t _ttalink_request_point_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t mode, float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_float(buf, 1, param1);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN);
#else
    ttalink_request_point_items_t packet;
    packet.mode = mode;
    packet.param1 = param1;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_POINT_ITEMS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a request_point_items message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param mode  0=home点 1=归航点 2=指点飞行点 3=环绕点 4=兴趣点 5=航线断点 6=AB航线点断点. 
 * @param param1   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_point_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t mode, float param1)
{
    return _ttalink_request_point_items_pack(dst_addr, src_addr, msg,  mode, param1, false);
}

/**
 * @brief Pack a request_point_items message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param mode  0=home点 1=归航点 2=指点飞行点 3=环绕点 4=兴趣点 5=航线断点 6=AB航线点断点. 
 * @param param1   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_point_items_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t mode, float param1)
{
    return _ttalink_request_point_items_pack(dst_addr, src_addr, msg,  mode, param1, true);
}


static inline uint16_t _ttalink_request_point_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t mode,float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_float(buf, 1, param1);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN);
#else
    ttalink_request_point_items_t packet;
    packet.mode = mode;
    packet.param1 = param1;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_POINT_ITEMS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a request_point_items message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mode  0=home点 1=归航点 2=指点飞行点 3=环绕点 4=兴趣点 5=航线断点 6=AB航线点断点. 
 * @param param1   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_point_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t mode,float param1)
{
    return _ttalink_request_point_items_pack_chan(dst_addr, src_addr, chan, msg,  mode, param1, false);
}

/**
 * @brief Pack a request_point_items message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mode  0=home点 1=归航点 2=指点飞行点 3=环绕点 4=兴趣点 5=航线断点 6=AB航线点断点. 
 * @param param1   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_point_items_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t mode,float param1)
{
    return _ttalink_request_point_items_pack_chan(dst_addr, src_addr, chan, msg,  mode, param1, true);
}


static inline uint16_t _ttalink_request_point_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_point_items_t* request_point_items, bool nocrc)
{
    if(nocrc){
        return ttalink_request_point_items_pack_nocrc(dst_addr, src_addr, msg, request_point_items->mode, request_point_items->param1);
    }else{
        return ttalink_request_point_items_pack(dst_addr, src_addr, msg, request_point_items->mode, request_point_items->param1);
    }
    
}

/**
 * @brief Encode a request_point_items struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_point_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_point_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_point_items_t* request_point_items)
{
    return _ttalink_request_point_items_encode(dst_addr, src_addr, msg, request_point_items, false);
}

/**
 * @brief Encode a request_point_items struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_point_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_point_items_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_point_items_t* request_point_items)
{
    return _ttalink_request_point_items_encode(dst_addr, src_addr, msg, request_point_items, true);
}


static inline uint16_t _ttalink_request_point_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_point_items_t* request_point_items, bool nocrc)
{
    if(nocrc){
        return ttalink_request_point_items_pack_chan_nocrc(dst_addr, src_addr, chan, msg, request_point_items->mode, request_point_items->param1);
    }else{
        return ttalink_request_point_items_pack_chan(dst_addr, src_addr, chan, msg, request_point_items->mode, request_point_items->param1);
    }
}

/**
 * @brief Encode a request_point_items struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_point_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_point_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_point_items_t* request_point_items)
{
    return _ttalink_request_point_items_encode_chan(dst_addr, src_addr, chan, msg, request_point_items, false);
}

/**
 * @brief Encode a request_point_items struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_point_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_point_items_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_point_items_t* request_point_items)
{
    return _ttalink_request_point_items_encode_chan(dst_addr, src_addr, chan, msg, request_point_items, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_request_point_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t mode, float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_float(buf, 1, param1);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_POINT_ITEMS, buf, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_CRC, nocrc);
#else
    ttalink_request_point_items_t packet;
    packet.mode = mode;
    packet.param1 = param1;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_POINT_ITEMS, (const char *)&packet, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_point_items message
 * @param chan TTAlink channel to send the message
 *
 * @param mode  0=home点 1=归航点 2=指点飞行点 3=环绕点 4=兴趣点 5=航线断点 6=AB航线点断点. 
 * @param param1   . .
 */
static inline void ttalink_request_point_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t mode, float param1)
{
    _ttalink_request_point_items_send(dst_addr, src_addr, chan, mode, param1, false);
}

/**
 * @brief Send a request_point_items message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param mode  0=home点 1=归航点 2=指点飞行点 3=环绕点 4=兴趣点 5=航线断点 6=AB航线点断点. 
 * @param param1   . .
 */
static inline void ttalink_request_point_items_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t mode, float param1)
{
    _ttalink_request_point_items_send(dst_addr, src_addr, chan, mode, param1, true);
}


static inline void _ttalink_request_point_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_point_items_t* request_point_items, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_request_point_items_send_nocrc(dst_addr, src_addr, chan, request_point_items->mode, request_point_items->param1);
    }else{
        ttalink_request_point_items_send(dst_addr, src_addr, chan, request_point_items->mode, request_point_items->param1);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_POINT_ITEMS, (const char *)request_point_items, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_point_items message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_point_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_point_items_t* request_point_items)
{
    _ttalink_request_point_items_send_struct(dst_addr, src_addr, chan, request_point_items, false);
}

/**
 * @brief Send a request_point_items message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_point_items_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_point_items_t* request_point_items)
{
    _ttalink_request_point_items_send_struct(dst_addr, src_addr, chan, request_point_items, true);
}

#if TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_request_point_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t mode, float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_float(buf, 1, param1);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_POINT_ITEMS, buf, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_CRC, nocrc);
#else
    ttalink_request_point_items_t *packet = (ttalink_request_point_items_t *)msgbuf;
    packet->mode = mode;
    packet->param1 = param1;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_POINT_ITEMS, (const char *)packet, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_request_point_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t mode, float param1)
{
    _ttalink_request_point_items_send_buf(dst_addr, src_addr, msgbuf, chan, mode, param1, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_request_point_items_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t mode, float param1)
{
    _ttalink_request_point_items_send_buf(dst_addr, src_addr, msgbuf, chan, mode, param1, true);
}
#endif

#endif

// MESSAGE REQUEST_POINT_ITEMS UNPACKING


/**
 * @brief Get field mode from request_point_items message
 *
 * @return  0=home点 1=归航点 2=指点飞行点 3=环绕点 4=兴趣点 5=航线断点 6=AB航线点断点. 
 */
static inline uint8_t ttalink_request_point_items_get_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field param1 from request_point_items message
 *
 * @return   . .
 */
static inline float ttalink_request_point_items_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  1);
}

/**
 * @brief Decode a request_point_items message into a struct
 *
 * @param msg The message to decode
 * @param request_point_items C-struct to decode the message contents into
 */
static inline void ttalink_request_point_items_decode(const ttalink_message_t* msg, ttalink_request_point_items_t* request_point_items)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    request_point_items->mode = ttalink_request_point_items_get_mode(msg);
    request_point_items->param1 = ttalink_request_point_items_get_param1(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN? msg->len : TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN;
        memset(request_point_items, 0, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_LEN);
    memcpy(request_point_items, _TTA_PAYLOAD(msg), len);
#endif
}
