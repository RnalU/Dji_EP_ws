#pragma once
// MESSAGE POD_1074_SET_FOCUS_AREA PACKING

#define TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA 11074

TTAPACKED(
typedef struct __ttalink_pod_1074_set_focus_area_t {
 float horizontal; /*<  Horizontal area coordinates, ranging from 0.0 to 1.0. The point [0.0, 0.0] indicates the upper left corner of the screen.*/
 float vertical; /*<  Vertical area coordinates from 0.0 to 1.0..*/
 uint8_t ack; /*< .*/
}) ttalink_pod_1074_set_focus_area_t;

#define TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN 9
#define TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_MIN_LEN 9
#define TTALINK_MSG_ID_11074_LEN 9
#define TTALINK_MSG_ID_11074_MIN_LEN 9

#define TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_CRC 143
#define TTALINK_MSG_ID_11074_CRC 143



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1074_SET_FOCUS_AREA { \
    11074, \
    "POD_1074_SET_FOCUS_AREA", \
    3, \
    {  { "horizontal", NULL, TTALINK_TYPE_FLOAT, 0, 0, offsetof(ttalink_pod_1074_set_focus_area_t, horizontal) }, \
         { "vertical", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_pod_1074_set_focus_area_t, vertical) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_pod_1074_set_focus_area_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1074_SET_FOCUS_AREA { \
    "POD_1074_SET_FOCUS_AREA", \
    3, \
    {  { "horizontal", NULL, TTALINK_TYPE_FLOAT, 0, 0, offsetof(ttalink_pod_1074_set_focus_area_t, horizontal) }, \
         { "vertical", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_pod_1074_set_focus_area_t, vertical) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_pod_1074_set_focus_area_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1074_set_focus_area_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float horizontal, float vertical, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN];
    _tta_put_float(buf, 0, horizontal);
    _tta_put_float(buf, 4, vertical);
    _tta_put_uint8_t(buf, 8, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN);
#else
    ttalink_pod_1074_set_focus_area_t packet;
    packet.horizontal = horizontal;
    packet.vertical = vertical;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_MIN_LEN, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_CRC, nocrc);
}

/**
 * @brief Pack a pod_1074_set_focus_area message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param horizontal  Horizontal area coordinates, ranging from 0.0 to 1.0. The point [0.0, 0.0] indicates the upper left corner of the screen.
 * @param vertical  Vertical area coordinates from 0.0 to 1.0..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1074_set_focus_area_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float horizontal, float vertical, uint8_t ack)
{
    return _ttalink_pod_1074_set_focus_area_pack(dst_addr, src_addr, msg,  horizontal, vertical, ack, false);
}

/**
 * @brief Pack a pod_1074_set_focus_area message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param horizontal  Horizontal area coordinates, ranging from 0.0 to 1.0. The point [0.0, 0.0] indicates the upper left corner of the screen.
 * @param vertical  Vertical area coordinates from 0.0 to 1.0..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1074_set_focus_area_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float horizontal, float vertical, uint8_t ack)
{
    return _ttalink_pod_1074_set_focus_area_pack(dst_addr, src_addr, msg,  horizontal, vertical, ack, true);
}


static inline uint16_t _ttalink_pod_1074_set_focus_area_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float horizontal,float vertical,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN];
    _tta_put_float(buf, 0, horizontal);
    _tta_put_float(buf, 4, vertical);
    _tta_put_uint8_t(buf, 8, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN);
#else
    ttalink_pod_1074_set_focus_area_t packet;
    packet.horizontal = horizontal;
    packet.vertical = vertical;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_MIN_LEN, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_CRC, nocrc);
}

/**
 * @brief Pack a pod_1074_set_focus_area message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param horizontal  Horizontal area coordinates, ranging from 0.0 to 1.0. The point [0.0, 0.0] indicates the upper left corner of the screen.
 * @param vertical  Vertical area coordinates from 0.0 to 1.0..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1074_set_focus_area_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float horizontal,float vertical,uint8_t ack)
{
    return _ttalink_pod_1074_set_focus_area_pack_chan(dst_addr, src_addr, chan, msg,  horizontal, vertical, ack, false);
}

/**
 * @brief Pack a pod_1074_set_focus_area message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param horizontal  Horizontal area coordinates, ranging from 0.0 to 1.0. The point [0.0, 0.0] indicates the upper left corner of the screen.
 * @param vertical  Vertical area coordinates from 0.0 to 1.0..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1074_set_focus_area_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float horizontal,float vertical,uint8_t ack)
{
    return _ttalink_pod_1074_set_focus_area_pack_chan(dst_addr, src_addr, chan, msg,  horizontal, vertical, ack, true);
}


static inline uint16_t _ttalink_pod_1074_set_focus_area_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1074_set_focus_area_t* pod_1074_set_focus_area, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1074_set_focus_area_pack_nocrc(dst_addr, src_addr, msg, pod_1074_set_focus_area->horizontal, pod_1074_set_focus_area->vertical, pod_1074_set_focus_area->ack);
    }else{
        return ttalink_pod_1074_set_focus_area_pack(dst_addr, src_addr, msg, pod_1074_set_focus_area->horizontal, pod_1074_set_focus_area->vertical, pod_1074_set_focus_area->ack);
    }
    
}

/**
 * @brief Encode a pod_1074_set_focus_area struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1074_set_focus_area C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1074_set_focus_area_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1074_set_focus_area_t* pod_1074_set_focus_area)
{
    return _ttalink_pod_1074_set_focus_area_encode(dst_addr, src_addr, msg, pod_1074_set_focus_area, false);
}

/**
 * @brief Encode a pod_1074_set_focus_area struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1074_set_focus_area C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1074_set_focus_area_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1074_set_focus_area_t* pod_1074_set_focus_area)
{
    return _ttalink_pod_1074_set_focus_area_encode(dst_addr, src_addr, msg, pod_1074_set_focus_area, true);
}


static inline uint16_t _ttalink_pod_1074_set_focus_area_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1074_set_focus_area_t* pod_1074_set_focus_area, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1074_set_focus_area_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1074_set_focus_area->horizontal, pod_1074_set_focus_area->vertical, pod_1074_set_focus_area->ack);
    }else{
        return ttalink_pod_1074_set_focus_area_pack_chan(dst_addr, src_addr, chan, msg, pod_1074_set_focus_area->horizontal, pod_1074_set_focus_area->vertical, pod_1074_set_focus_area->ack);
    }
}

/**
 * @brief Encode a pod_1074_set_focus_area struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1074_set_focus_area C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1074_set_focus_area_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1074_set_focus_area_t* pod_1074_set_focus_area)
{
    return _ttalink_pod_1074_set_focus_area_encode_chan(dst_addr, src_addr, chan, msg, pod_1074_set_focus_area, false);
}

/**
 * @brief Encode a pod_1074_set_focus_area struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1074_set_focus_area C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1074_set_focus_area_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1074_set_focus_area_t* pod_1074_set_focus_area)
{
    return _ttalink_pod_1074_set_focus_area_encode_chan(dst_addr, src_addr, chan, msg, pod_1074_set_focus_area, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1074_set_focus_area_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float horizontal, float vertical, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN];
    _tta_put_float(buf, 0, horizontal);
    _tta_put_float(buf, 4, vertical);
    _tta_put_uint8_t(buf, 8, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA, buf, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_MIN_LEN, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_CRC, nocrc);
#else
    ttalink_pod_1074_set_focus_area_t packet;
    packet.horizontal = horizontal;
    packet.vertical = vertical;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA, (const char *)&packet, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_MIN_LEN, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1074_set_focus_area message
 * @param chan TTAlink channel to send the message
 *
 * @param horizontal  Horizontal area coordinates, ranging from 0.0 to 1.0. The point [0.0, 0.0] indicates the upper left corner of the screen.
 * @param vertical  Vertical area coordinates from 0.0 to 1.0..
 * @param ack .
 */
static inline void ttalink_pod_1074_set_focus_area_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float horizontal, float vertical, uint8_t ack)
{
    _ttalink_pod_1074_set_focus_area_send(dst_addr, src_addr, chan, horizontal, vertical, ack, false);
}

/**
 * @brief Send a pod_1074_set_focus_area message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param horizontal  Horizontal area coordinates, ranging from 0.0 to 1.0. The point [0.0, 0.0] indicates the upper left corner of the screen.
 * @param vertical  Vertical area coordinates from 0.0 to 1.0..
 * @param ack .
 */
static inline void ttalink_pod_1074_set_focus_area_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float horizontal, float vertical, uint8_t ack)
{
    _ttalink_pod_1074_set_focus_area_send(dst_addr, src_addr, chan, horizontal, vertical, ack, true);
}


static inline void _ttalink_pod_1074_set_focus_area_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1074_set_focus_area_t* pod_1074_set_focus_area, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1074_set_focus_area_send_nocrc(dst_addr, src_addr, chan, pod_1074_set_focus_area->horizontal, pod_1074_set_focus_area->vertical, pod_1074_set_focus_area->ack);
    }else{
        ttalink_pod_1074_set_focus_area_send(dst_addr, src_addr, chan, pod_1074_set_focus_area->horizontal, pod_1074_set_focus_area->vertical, pod_1074_set_focus_area->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA, (const char *)pod_1074_set_focus_area, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_MIN_LEN, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1074_set_focus_area message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1074_set_focus_area_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1074_set_focus_area_t* pod_1074_set_focus_area)
{
    _ttalink_pod_1074_set_focus_area_send_struct(dst_addr, src_addr, chan, pod_1074_set_focus_area, false);
}

/**
 * @brief Send a pod_1074_set_focus_area message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1074_set_focus_area_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1074_set_focus_area_t* pod_1074_set_focus_area)
{
    _ttalink_pod_1074_set_focus_area_send_struct(dst_addr, src_addr, chan, pod_1074_set_focus_area, true);
}

#if TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1074_set_focus_area_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float horizontal, float vertical, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_float(buf, 0, horizontal);
    _tta_put_float(buf, 4, vertical);
    _tta_put_uint8_t(buf, 8, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA, buf, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_MIN_LEN, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_CRC, nocrc);
#else
    ttalink_pod_1074_set_focus_area_t *packet = (ttalink_pod_1074_set_focus_area_t *)msgbuf;
    packet->horizontal = horizontal;
    packet->vertical = vertical;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA, (const char *)packet, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_MIN_LEN, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1074_set_focus_area_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float horizontal, float vertical, uint8_t ack)
{
    _ttalink_pod_1074_set_focus_area_send_buf(dst_addr, src_addr, msgbuf, chan, horizontal, vertical, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1074_set_focus_area_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float horizontal, float vertical, uint8_t ack)
{
    _ttalink_pod_1074_set_focus_area_send_buf(dst_addr, src_addr, msgbuf, chan, horizontal, vertical, ack, true);
}
#endif

#endif

// MESSAGE POD_1074_SET_FOCUS_AREA UNPACKING


/**
 * @brief Get field horizontal from pod_1074_set_focus_area message
 *
 * @return  Horizontal area coordinates, ranging from 0.0 to 1.0. The point [0.0, 0.0] indicates the upper left corner of the screen.
 */
static inline float ttalink_pod_1074_set_focus_area_get_horizontal(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  0);
}

/**
 * @brief Get field vertical from pod_1074_set_focus_area message
 *
 * @return  Vertical area coordinates from 0.0 to 1.0..
 */
static inline float ttalink_pod_1074_set_focus_area_get_vertical(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  4);
}

/**
 * @brief Get field ack from pod_1074_set_focus_area message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1074_set_focus_area_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Decode a pod_1074_set_focus_area message into a struct
 *
 * @param msg The message to decode
 * @param pod_1074_set_focus_area C-struct to decode the message contents into
 */
static inline void ttalink_pod_1074_set_focus_area_decode(const ttalink_message_t* msg, ttalink_pod_1074_set_focus_area_t* pod_1074_set_focus_area)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1074_set_focus_area->horizontal = ttalink_pod_1074_set_focus_area_get_horizontal(msg);
    pod_1074_set_focus_area->vertical = ttalink_pod_1074_set_focus_area_get_vertical(msg);
    pod_1074_set_focus_area->ack = ttalink_pod_1074_set_focus_area_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN? msg->len : TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN;
        memset(pod_1074_set_focus_area, 0, TTALINK_MSG_ID_POD_1074_SET_FOCUS_AREA_LEN);
    memcpy(pod_1074_set_focus_area, _TTA_PAYLOAD(msg), len);
#endif
}
