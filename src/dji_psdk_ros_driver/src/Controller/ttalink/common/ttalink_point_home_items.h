#pragma once
// MESSAGE POINT_HOME_ITEMS PACKING

#define TTALINK_MSG_ID_POINT_HOME_ITEMS 2194

TTAPACKED(
typedef struct __ttalink_point_home_items_t {
 uint8_t state; /*<  .*/
 int32_t latit; /*<  .*/
 int32_t longi; /*<  .*/
 float altit; /*<  .*/
}) ttalink_point_home_items_t;

#define TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN 13
#define TTALINK_MSG_ID_POINT_HOME_ITEMS_MIN_LEN 13
#define TTALINK_MSG_ID_2194_LEN 13
#define TTALINK_MSG_ID_2194_MIN_LEN 13

#define TTALINK_MSG_ID_POINT_HOME_ITEMS_CRC 107
#define TTALINK_MSG_ID_2194_CRC 107



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POINT_HOME_ITEMS { \
    2194, \
    "POINT_HOME_ITEMS", \
    4, \
    {  { "state", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_point_home_items_t, state) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 1, offsetof(ttalink_point_home_items_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 5, offsetof(ttalink_point_home_items_t, longi) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 9, offsetof(ttalink_point_home_items_t, altit) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POINT_HOME_ITEMS { \
    "POINT_HOME_ITEMS", \
    4, \
    {  { "state", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_point_home_items_t, state) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 1, offsetof(ttalink_point_home_items_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 5, offsetof(ttalink_point_home_items_t, longi) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 9, offsetof(ttalink_point_home_items_t, altit) }, \
         } \
}
#endif


static inline uint16_t _ttalink_point_home_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t state, int32_t latit, int32_t longi, float altit, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, state);
    _tta_put_int32_t(buf, 1, latit);
    _tta_put_int32_t(buf, 5, longi);
    _tta_put_float(buf, 9, altit);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN);
#else
    ttalink_point_home_items_t packet;
    packet.state = state;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POINT_HOME_ITEMS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POINT_HOME_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN, TTALINK_MSG_ID_POINT_HOME_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a point_home_items message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param state  .
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_point_home_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t state, int32_t latit, int32_t longi, float altit)
{
    return _ttalink_point_home_items_pack(dst_addr, src_addr, msg,  state, latit, longi, altit, false);
}

/**
 * @brief Pack a point_home_items message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param state  .
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_point_home_items_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t state, int32_t latit, int32_t longi, float altit)
{
    return _ttalink_point_home_items_pack(dst_addr, src_addr, msg,  state, latit, longi, altit, true);
}


static inline uint16_t _ttalink_point_home_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t state,int32_t latit,int32_t longi,float altit, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, state);
    _tta_put_int32_t(buf, 1, latit);
    _tta_put_int32_t(buf, 5, longi);
    _tta_put_float(buf, 9, altit);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN);
#else
    ttalink_point_home_items_t packet;
    packet.state = state;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POINT_HOME_ITEMS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POINT_HOME_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN, TTALINK_MSG_ID_POINT_HOME_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a point_home_items message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param state  .
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_point_home_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t state,int32_t latit,int32_t longi,float altit)
{
    return _ttalink_point_home_items_pack_chan(dst_addr, src_addr, chan, msg,  state, latit, longi, altit, false);
}

/**
 * @brief Pack a point_home_items message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param state  .
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_point_home_items_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t state,int32_t latit,int32_t longi,float altit)
{
    return _ttalink_point_home_items_pack_chan(dst_addr, src_addr, chan, msg,  state, latit, longi, altit, true);
}


static inline uint16_t _ttalink_point_home_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_point_home_items_t* point_home_items, bool nocrc)
{
    if(nocrc){
        return ttalink_point_home_items_pack_nocrc(dst_addr, src_addr, msg, point_home_items->state, point_home_items->latit, point_home_items->longi, point_home_items->altit);
    }else{
        return ttalink_point_home_items_pack(dst_addr, src_addr, msg, point_home_items->state, point_home_items->latit, point_home_items->longi, point_home_items->altit);
    }
    
}

/**
 * @brief Encode a point_home_items struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param point_home_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_point_home_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_point_home_items_t* point_home_items)
{
    return _ttalink_point_home_items_encode(dst_addr, src_addr, msg, point_home_items, false);
}

/**
 * @brief Encode a point_home_items struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param point_home_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_point_home_items_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_point_home_items_t* point_home_items)
{
    return _ttalink_point_home_items_encode(dst_addr, src_addr, msg, point_home_items, true);
}


static inline uint16_t _ttalink_point_home_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_point_home_items_t* point_home_items, bool nocrc)
{
    if(nocrc){
        return ttalink_point_home_items_pack_chan_nocrc(dst_addr, src_addr, chan, msg, point_home_items->state, point_home_items->latit, point_home_items->longi, point_home_items->altit);
    }else{
        return ttalink_point_home_items_pack_chan(dst_addr, src_addr, chan, msg, point_home_items->state, point_home_items->latit, point_home_items->longi, point_home_items->altit);
    }
}

/**
 * @brief Encode a point_home_items struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param point_home_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_point_home_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_point_home_items_t* point_home_items)
{
    return _ttalink_point_home_items_encode_chan(dst_addr, src_addr, chan, msg, point_home_items, false);
}

/**
 * @brief Encode a point_home_items struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param point_home_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_point_home_items_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_point_home_items_t* point_home_items)
{
    return _ttalink_point_home_items_encode_chan(dst_addr, src_addr, chan, msg, point_home_items, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_point_home_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t state, int32_t latit, int32_t longi, float altit, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, state);
    _tta_put_int32_t(buf, 1, latit);
    _tta_put_int32_t(buf, 5, longi);
    _tta_put_float(buf, 9, altit);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POINT_HOME_ITEMS, buf, TTALINK_MSG_ID_POINT_HOME_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN, TTALINK_MSG_ID_POINT_HOME_ITEMS_CRC, nocrc);
#else
    ttalink_point_home_items_t packet;
    packet.state = state;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POINT_HOME_ITEMS, (const char *)&packet, TTALINK_MSG_ID_POINT_HOME_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN, TTALINK_MSG_ID_POINT_HOME_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a point_home_items message
 * @param chan TTAlink channel to send the message
 *
 * @param state  .
 * @param latit  .
 * @param longi  .
 * @param altit  .
 */
static inline void ttalink_point_home_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t state, int32_t latit, int32_t longi, float altit)
{
    _ttalink_point_home_items_send(dst_addr, src_addr, chan, state, latit, longi, altit, false);
}

/**
 * @brief Send a point_home_items message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param state  .
 * @param latit  .
 * @param longi  .
 * @param altit  .
 */
static inline void ttalink_point_home_items_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t state, int32_t latit, int32_t longi, float altit)
{
    _ttalink_point_home_items_send(dst_addr, src_addr, chan, state, latit, longi, altit, true);
}


static inline void _ttalink_point_home_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_point_home_items_t* point_home_items, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_point_home_items_send_nocrc(dst_addr, src_addr, chan, point_home_items->state, point_home_items->latit, point_home_items->longi, point_home_items->altit);
    }else{
        ttalink_point_home_items_send(dst_addr, src_addr, chan, point_home_items->state, point_home_items->latit, point_home_items->longi, point_home_items->altit);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POINT_HOME_ITEMS, (const char *)point_home_items, TTALINK_MSG_ID_POINT_HOME_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN, TTALINK_MSG_ID_POINT_HOME_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a point_home_items message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_point_home_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_point_home_items_t* point_home_items)
{
    _ttalink_point_home_items_send_struct(dst_addr, src_addr, chan, point_home_items, false);
}

/**
 * @brief Send a point_home_items message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_point_home_items_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_point_home_items_t* point_home_items)
{
    _ttalink_point_home_items_send_struct(dst_addr, src_addr, chan, point_home_items, true);
}

#if TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_point_home_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t state, int32_t latit, int32_t longi, float altit, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, state);
    _tta_put_int32_t(buf, 1, latit);
    _tta_put_int32_t(buf, 5, longi);
    _tta_put_float(buf, 9, altit);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POINT_HOME_ITEMS, buf, TTALINK_MSG_ID_POINT_HOME_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN, TTALINK_MSG_ID_POINT_HOME_ITEMS_CRC, nocrc);
#else
    ttalink_point_home_items_t *packet = (ttalink_point_home_items_t *)msgbuf;
    packet->state = state;
    packet->latit = latit;
    packet->longi = longi;
    packet->altit = altit;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POINT_HOME_ITEMS, (const char *)packet, TTALINK_MSG_ID_POINT_HOME_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN, TTALINK_MSG_ID_POINT_HOME_ITEMS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_point_home_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t state, int32_t latit, int32_t longi, float altit)
{
    _ttalink_point_home_items_send_buf(dst_addr, src_addr, msgbuf, chan, state, latit, longi, altit, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_point_home_items_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t state, int32_t latit, int32_t longi, float altit)
{
    _ttalink_point_home_items_send_buf(dst_addr, src_addr, msgbuf, chan, state, latit, longi, altit, true);
}
#endif

#endif

// MESSAGE POINT_HOME_ITEMS UNPACKING


/**
 * @brief Get field state from point_home_items message
 *
 * @return  .
 */
static inline uint8_t ttalink_point_home_items_get_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field latit from point_home_items message
 *
 * @return  .
 */
static inline int32_t ttalink_point_home_items_get_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  1);
}

/**
 * @brief Get field longi from point_home_items message
 *
 * @return  .
 */
static inline int32_t ttalink_point_home_items_get_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  5);
}

/**
 * @brief Get field altit from point_home_items message
 *
 * @return  .
 */
static inline float ttalink_point_home_items_get_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  9);
}

/**
 * @brief Decode a point_home_items message into a struct
 *
 * @param msg The message to decode
 * @param point_home_items C-struct to decode the message contents into
 */
static inline void ttalink_point_home_items_decode(const ttalink_message_t* msg, ttalink_point_home_items_t* point_home_items)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    point_home_items->state = ttalink_point_home_items_get_state(msg);
    point_home_items->latit = ttalink_point_home_items_get_latit(msg);
    point_home_items->longi = ttalink_point_home_items_get_longi(msg);
    point_home_items->altit = ttalink_point_home_items_get_altit(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN? msg->len : TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN;
        memset(point_home_items, 0, TTALINK_MSG_ID_POINT_HOME_ITEMS_LEN);
    memcpy(point_home_items, _TTA_PAYLOAD(msg), len);
#endif
}
