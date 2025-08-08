#pragma once
// MESSAGE REQUEST_POINT_ITEMS_ACK PACKING

#define TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK 2187

TTAPACKED(
typedef struct __ttalink_request_point_items_ack_t {
 uint8_t ack; /*<  .*/
 uint8_t mode; /*<  0=home点 1=归航点 2=指点飞行点 3=环绕点 4=兴趣点 5=航线断点 6=AB航线点断点.. */
 int32_t longi; /*<  . */
 int32_t latit; /*<  . */
 float altit; /*<   . .*/
 float param1; /*<   . .*/
}) ttalink_request_point_items_ack_t;

#define TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN 18
#define TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_MIN_LEN 18
#define TTALINK_MSG_ID_2187_LEN 18
#define TTALINK_MSG_ID_2187_MIN_LEN 18

#define TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_CRC 201
#define TTALINK_MSG_ID_2187_CRC 201



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_REQUEST_POINT_ITEMS_ACK { \
    2187, \
    "REQUEST_POINT_ITEMS_ACK", \
    6, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_request_point_items_ack_t, ack) }, \
         { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_request_point_items_ack_t, mode) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 2, offsetof(ttalink_request_point_items_ack_t, longi) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 6, offsetof(ttalink_request_point_items_ack_t, latit) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 10, offsetof(ttalink_request_point_items_ack_t, altit) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 14, offsetof(ttalink_request_point_items_ack_t, param1) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_REQUEST_POINT_ITEMS_ACK { \
    "REQUEST_POINT_ITEMS_ACK", \
    6, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_request_point_items_ack_t, ack) }, \
         { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_request_point_items_ack_t, mode) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 2, offsetof(ttalink_request_point_items_ack_t, longi) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 6, offsetof(ttalink_request_point_items_ack_t, latit) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 10, offsetof(ttalink_request_point_items_ack_t, altit) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 14, offsetof(ttalink_request_point_items_ack_t, param1) }, \
         } \
}
#endif


static inline uint16_t _ttalink_request_point_items_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t mode, int32_t longi, int32_t latit, float altit, float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, mode);
    _tta_put_int32_t(buf, 2, longi);
    _tta_put_int32_t(buf, 6, latit);
    _tta_put_float(buf, 10, altit);
    _tta_put_float(buf, 14, param1);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN);
#else
    ttalink_request_point_items_ack_t packet;
    packet.ack = ack;
    packet.mode = mode;
    packet.longi = longi;
    packet.latit = latit;
    packet.altit = altit;
    packet.param1 = param1;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_CRC, nocrc);
}

/**
 * @brief Pack a request_point_items_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack  .
 * @param mode  0=home点 1=归航点 2=指点飞行点 3=环绕点 4=兴趣点 5=航线断点 6=AB航线点断点.. 
 * @param longi  . 
 * @param latit  . 
 * @param altit   . .
 * @param param1   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_point_items_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t mode, int32_t longi, int32_t latit, float altit, float param1)
{
    return _ttalink_request_point_items_ack_pack(dst_addr, src_addr, msg,  ack, mode, longi, latit, altit, param1, false);
}

/**
 * @brief Pack a request_point_items_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack  .
 * @param mode  0=home点 1=归航点 2=指点飞行点 3=环绕点 4=兴趣点 5=航线断点 6=AB航线点断点.. 
 * @param longi  . 
 * @param latit  . 
 * @param altit   . .
 * @param param1   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_point_items_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t mode, int32_t longi, int32_t latit, float altit, float param1)
{
    return _ttalink_request_point_items_ack_pack(dst_addr, src_addr, msg,  ack, mode, longi, latit, altit, param1, true);
}


static inline uint16_t _ttalink_request_point_items_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t mode,int32_t longi,int32_t latit,float altit,float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, mode);
    _tta_put_int32_t(buf, 2, longi);
    _tta_put_int32_t(buf, 6, latit);
    _tta_put_float(buf, 10, altit);
    _tta_put_float(buf, 14, param1);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN);
#else
    ttalink_request_point_items_ack_t packet;
    packet.ack = ack;
    packet.mode = mode;
    packet.longi = longi;
    packet.latit = latit;
    packet.altit = altit;
    packet.param1 = param1;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_CRC, nocrc);
}

/**
 * @brief Pack a request_point_items_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack  .
 * @param mode  0=home点 1=归航点 2=指点飞行点 3=环绕点 4=兴趣点 5=航线断点 6=AB航线点断点.. 
 * @param longi  . 
 * @param latit  . 
 * @param altit   . .
 * @param param1   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_point_items_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t mode,int32_t longi,int32_t latit,float altit,float param1)
{
    return _ttalink_request_point_items_ack_pack_chan(dst_addr, src_addr, chan, msg,  ack, mode, longi, latit, altit, param1, false);
}

/**
 * @brief Pack a request_point_items_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack  .
 * @param mode  0=home点 1=归航点 2=指点飞行点 3=环绕点 4=兴趣点 5=航线断点 6=AB航线点断点.. 
 * @param longi  . 
 * @param latit  . 
 * @param altit   . .
 * @param param1   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_point_items_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t mode,int32_t longi,int32_t latit,float altit,float param1)
{
    return _ttalink_request_point_items_ack_pack_chan(dst_addr, src_addr, chan, msg,  ack, mode, longi, latit, altit, param1, true);
}


static inline uint16_t _ttalink_request_point_items_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_point_items_ack_t* request_point_items_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_request_point_items_ack_pack_nocrc(dst_addr, src_addr, msg, request_point_items_ack->ack, request_point_items_ack->mode, request_point_items_ack->longi, request_point_items_ack->latit, request_point_items_ack->altit, request_point_items_ack->param1);
    }else{
        return ttalink_request_point_items_ack_pack(dst_addr, src_addr, msg, request_point_items_ack->ack, request_point_items_ack->mode, request_point_items_ack->longi, request_point_items_ack->latit, request_point_items_ack->altit, request_point_items_ack->param1);
    }
    
}

/**
 * @brief Encode a request_point_items_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_point_items_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_point_items_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_point_items_ack_t* request_point_items_ack)
{
    return _ttalink_request_point_items_ack_encode(dst_addr, src_addr, msg, request_point_items_ack, false);
}

/**
 * @brief Encode a request_point_items_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_point_items_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_point_items_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_point_items_ack_t* request_point_items_ack)
{
    return _ttalink_request_point_items_ack_encode(dst_addr, src_addr, msg, request_point_items_ack, true);
}


static inline uint16_t _ttalink_request_point_items_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_point_items_ack_t* request_point_items_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_request_point_items_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, request_point_items_ack->ack, request_point_items_ack->mode, request_point_items_ack->longi, request_point_items_ack->latit, request_point_items_ack->altit, request_point_items_ack->param1);
    }else{
        return ttalink_request_point_items_ack_pack_chan(dst_addr, src_addr, chan, msg, request_point_items_ack->ack, request_point_items_ack->mode, request_point_items_ack->longi, request_point_items_ack->latit, request_point_items_ack->altit, request_point_items_ack->param1);
    }
}

/**
 * @brief Encode a request_point_items_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_point_items_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_point_items_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_point_items_ack_t* request_point_items_ack)
{
    return _ttalink_request_point_items_ack_encode_chan(dst_addr, src_addr, chan, msg, request_point_items_ack, false);
}

/**
 * @brief Encode a request_point_items_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_point_items_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_point_items_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_point_items_ack_t* request_point_items_ack)
{
    return _ttalink_request_point_items_ack_encode_chan(dst_addr, src_addr, chan, msg, request_point_items_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_request_point_items_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t mode, int32_t longi, int32_t latit, float altit, float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, mode);
    _tta_put_int32_t(buf, 2, longi);
    _tta_put_int32_t(buf, 6, latit);
    _tta_put_float(buf, 10, altit);
    _tta_put_float(buf, 14, param1);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK, buf, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_CRC, nocrc);
#else
    ttalink_request_point_items_ack_t packet;
    packet.ack = ack;
    packet.mode = mode;
    packet.longi = longi;
    packet.latit = latit;
    packet.altit = altit;
    packet.param1 = param1;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK, (const char *)&packet, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_point_items_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param ack  .
 * @param mode  0=home点 1=归航点 2=指点飞行点 3=环绕点 4=兴趣点 5=航线断点 6=AB航线点断点.. 
 * @param longi  . 
 * @param latit  . 
 * @param altit   . .
 * @param param1   . .
 */
static inline void ttalink_request_point_items_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t mode, int32_t longi, int32_t latit, float altit, float param1)
{
    _ttalink_request_point_items_ack_send(dst_addr, src_addr, chan, ack, mode, longi, latit, altit, param1, false);
}

/**
 * @brief Send a request_point_items_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack  .
 * @param mode  0=home点 1=归航点 2=指点飞行点 3=环绕点 4=兴趣点 5=航线断点 6=AB航线点断点.. 
 * @param longi  . 
 * @param latit  . 
 * @param altit   . .
 * @param param1   . .
 */
static inline void ttalink_request_point_items_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t mode, int32_t longi, int32_t latit, float altit, float param1)
{
    _ttalink_request_point_items_ack_send(dst_addr, src_addr, chan, ack, mode, longi, latit, altit, param1, true);
}


static inline void _ttalink_request_point_items_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_point_items_ack_t* request_point_items_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_request_point_items_ack_send_nocrc(dst_addr, src_addr, chan, request_point_items_ack->ack, request_point_items_ack->mode, request_point_items_ack->longi, request_point_items_ack->latit, request_point_items_ack->altit, request_point_items_ack->param1);
    }else{
        ttalink_request_point_items_ack_send(dst_addr, src_addr, chan, request_point_items_ack->ack, request_point_items_ack->mode, request_point_items_ack->longi, request_point_items_ack->latit, request_point_items_ack->altit, request_point_items_ack->param1);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK, (const char *)request_point_items_ack, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_point_items_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_point_items_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_point_items_ack_t* request_point_items_ack)
{
    _ttalink_request_point_items_ack_send_struct(dst_addr, src_addr, chan, request_point_items_ack, false);
}

/**
 * @brief Send a request_point_items_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_point_items_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_point_items_ack_t* request_point_items_ack)
{
    _ttalink_request_point_items_ack_send_struct(dst_addr, src_addr, chan, request_point_items_ack, true);
}

#if TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_request_point_items_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t mode, int32_t longi, int32_t latit, float altit, float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, mode);
    _tta_put_int32_t(buf, 2, longi);
    _tta_put_int32_t(buf, 6, latit);
    _tta_put_float(buf, 10, altit);
    _tta_put_float(buf, 14, param1);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK, buf, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_CRC, nocrc);
#else
    ttalink_request_point_items_ack_t *packet = (ttalink_request_point_items_ack_t *)msgbuf;
    packet->ack = ack;
    packet->mode = mode;
    packet->longi = longi;
    packet->latit = latit;
    packet->altit = altit;
    packet->param1 = param1;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK, (const char *)packet, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_request_point_items_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t mode, int32_t longi, int32_t latit, float altit, float param1)
{
    _ttalink_request_point_items_ack_send_buf(dst_addr, src_addr, msgbuf, chan, ack, mode, longi, latit, altit, param1, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_request_point_items_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t mode, int32_t longi, int32_t latit, float altit, float param1)
{
    _ttalink_request_point_items_ack_send_buf(dst_addr, src_addr, msgbuf, chan, ack, mode, longi, latit, altit, param1, true);
}
#endif

#endif

// MESSAGE REQUEST_POINT_ITEMS_ACK UNPACKING


/**
 * @brief Get field ack from request_point_items_ack message
 *
 * @return  .
 */
static inline uint8_t ttalink_request_point_items_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field mode from request_point_items_ack message
 *
 * @return  0=home点 1=归航点 2=指点飞行点 3=环绕点 4=兴趣点 5=航线断点 6=AB航线点断点.. 
 */
static inline uint8_t ttalink_request_point_items_ack_get_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field longi from request_point_items_ack message
 *
 * @return  . 
 */
static inline int32_t ttalink_request_point_items_ack_get_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  2);
}

/**
 * @brief Get field latit from request_point_items_ack message
 *
 * @return  . 
 */
static inline int32_t ttalink_request_point_items_ack_get_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  6);
}

/**
 * @brief Get field altit from request_point_items_ack message
 *
 * @return   . .
 */
static inline float ttalink_request_point_items_ack_get_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  10);
}

/**
 * @brief Get field param1 from request_point_items_ack message
 *
 * @return   . .
 */
static inline float ttalink_request_point_items_ack_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  14);
}

/**
 * @brief Decode a request_point_items_ack message into a struct
 *
 * @param msg The message to decode
 * @param request_point_items_ack C-struct to decode the message contents into
 */
static inline void ttalink_request_point_items_ack_decode(const ttalink_message_t* msg, ttalink_request_point_items_ack_t* request_point_items_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    request_point_items_ack->ack = ttalink_request_point_items_ack_get_ack(msg);
    request_point_items_ack->mode = ttalink_request_point_items_ack_get_mode(msg);
    request_point_items_ack->longi = ttalink_request_point_items_ack_get_longi(msg);
    request_point_items_ack->latit = ttalink_request_point_items_ack_get_latit(msg);
    request_point_items_ack->altit = ttalink_request_point_items_ack_get_altit(msg);
    request_point_items_ack->param1 = ttalink_request_point_items_ack_get_param1(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN? msg->len : TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN;
        memset(request_point_items_ack, 0, TTALINK_MSG_ID_REQUEST_POINT_ITEMS_ACK_LEN);
    memcpy(request_point_items_ack, _TTA_PAYLOAD(msg), len);
#endif
}
