#pragma once
// MESSAGE POD_1088_START_OPTICAL_ZOOM PACKING

#define TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM 11088

TTAPACKED(
typedef struct __ttalink_pod_1088_start_optical_zoom_t {
 uint8_t zoom_speed; /*< Zoom speed.
												72 :  is the slowest. The lens zooms in at the slowest speed.
												73 :  slow. The lens zooms in at a slow speed.
												74 :  a little slower. The magnification speed of the lens is slightly slower than the normal speed..*/
 uint8_t zoom_direction; /*< zoom direction.
													1 :  Zoom in. The lens will zoom in. The focal length increases, the field of view narrows and the magnification increases.
													0 :  Zoom out. The lens zooms. The focal length decreases, the field of view widens and the magnification decreases..*/
 uint8_t ack; /*< .*/
}) ttalink_pod_1088_start_optical_zoom_t;

#define TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN 3
#define TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_MIN_LEN 3
#define TTALINK_MSG_ID_11088_LEN 3
#define TTALINK_MSG_ID_11088_MIN_LEN 3

#define TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_CRC 220
#define TTALINK_MSG_ID_11088_CRC 220



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1088_START_OPTICAL_ZOOM { \
    11088, \
    "POD_1088_START_OPTICAL_ZOOM", \
    3, \
    {  { "zoom_speed", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1088_start_optical_zoom_t, zoom_speed) }, \
         { "zoom_direction", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1088_start_optical_zoom_t, zoom_direction) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_pod_1088_start_optical_zoom_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1088_START_OPTICAL_ZOOM { \
    "POD_1088_START_OPTICAL_ZOOM", \
    3, \
    {  { "zoom_speed", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1088_start_optical_zoom_t, zoom_speed) }, \
         { "zoom_direction", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1088_start_optical_zoom_t, zoom_direction) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_pod_1088_start_optical_zoom_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1088_start_optical_zoom_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t zoom_speed, uint8_t zoom_direction, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN];
    _tta_put_uint8_t(buf, 0, zoom_speed);
    _tta_put_uint8_t(buf, 1, zoom_direction);
    _tta_put_uint8_t(buf, 2, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN);
#else
    ttalink_pod_1088_start_optical_zoom_t packet;
    packet.zoom_speed = zoom_speed;
    packet.zoom_direction = zoom_direction;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_MIN_LEN, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_CRC, nocrc);
}

/**
 * @brief Pack a pod_1088_start_optical_zoom message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param zoom_speed Zoom speed.
												72 :  is the slowest. The lens zooms in at the slowest speed.
												73 :  slow. The lens zooms in at a slow speed.
												74 :  a little slower. The magnification speed of the lens is slightly slower than the normal speed..
 * @param zoom_direction zoom direction.
													1 :  Zoom in. The lens will zoom in. The focal length increases, the field of view narrows and the magnification increases.
													0 :  Zoom out. The lens zooms. The focal length decreases, the field of view widens and the magnification decreases..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1088_start_optical_zoom_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t zoom_speed, uint8_t zoom_direction, uint8_t ack)
{
    return _ttalink_pod_1088_start_optical_zoom_pack(dst_addr, src_addr, msg,  zoom_speed, zoom_direction, ack, false);
}

/**
 * @brief Pack a pod_1088_start_optical_zoom message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param zoom_speed Zoom speed.
												72 :  is the slowest. The lens zooms in at the slowest speed.
												73 :  slow. The lens zooms in at a slow speed.
												74 :  a little slower. The magnification speed of the lens is slightly slower than the normal speed..
 * @param zoom_direction zoom direction.
													1 :  Zoom in. The lens will zoom in. The focal length increases, the field of view narrows and the magnification increases.
													0 :  Zoom out. The lens zooms. The focal length decreases, the field of view widens and the magnification decreases..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1088_start_optical_zoom_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t zoom_speed, uint8_t zoom_direction, uint8_t ack)
{
    return _ttalink_pod_1088_start_optical_zoom_pack(dst_addr, src_addr, msg,  zoom_speed, zoom_direction, ack, true);
}


static inline uint16_t _ttalink_pod_1088_start_optical_zoom_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t zoom_speed,uint8_t zoom_direction,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN];
    _tta_put_uint8_t(buf, 0, zoom_speed);
    _tta_put_uint8_t(buf, 1, zoom_direction);
    _tta_put_uint8_t(buf, 2, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN);
#else
    ttalink_pod_1088_start_optical_zoom_t packet;
    packet.zoom_speed = zoom_speed;
    packet.zoom_direction = zoom_direction;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_MIN_LEN, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_CRC, nocrc);
}

/**
 * @brief Pack a pod_1088_start_optical_zoom message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param zoom_speed Zoom speed.
												72 :  is the slowest. The lens zooms in at the slowest speed.
												73 :  slow. The lens zooms in at a slow speed.
												74 :  a little slower. The magnification speed of the lens is slightly slower than the normal speed..
 * @param zoom_direction zoom direction.
													1 :  Zoom in. The lens will zoom in. The focal length increases, the field of view narrows and the magnification increases.
													0 :  Zoom out. The lens zooms. The focal length decreases, the field of view widens and the magnification decreases..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1088_start_optical_zoom_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t zoom_speed,uint8_t zoom_direction,uint8_t ack)
{
    return _ttalink_pod_1088_start_optical_zoom_pack_chan(dst_addr, src_addr, chan, msg,  zoom_speed, zoom_direction, ack, false);
}

/**
 * @brief Pack a pod_1088_start_optical_zoom message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param zoom_speed Zoom speed.
												72 :  is the slowest. The lens zooms in at the slowest speed.
												73 :  slow. The lens zooms in at a slow speed.
												74 :  a little slower. The magnification speed of the lens is slightly slower than the normal speed..
 * @param zoom_direction zoom direction.
													1 :  Zoom in. The lens will zoom in. The focal length increases, the field of view narrows and the magnification increases.
													0 :  Zoom out. The lens zooms. The focal length decreases, the field of view widens and the magnification decreases..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1088_start_optical_zoom_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t zoom_speed,uint8_t zoom_direction,uint8_t ack)
{
    return _ttalink_pod_1088_start_optical_zoom_pack_chan(dst_addr, src_addr, chan, msg,  zoom_speed, zoom_direction, ack, true);
}


static inline uint16_t _ttalink_pod_1088_start_optical_zoom_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1088_start_optical_zoom_t* pod_1088_start_optical_zoom, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1088_start_optical_zoom_pack_nocrc(dst_addr, src_addr, msg, pod_1088_start_optical_zoom->zoom_speed, pod_1088_start_optical_zoom->zoom_direction, pod_1088_start_optical_zoom->ack);
    }else{
        return ttalink_pod_1088_start_optical_zoom_pack(dst_addr, src_addr, msg, pod_1088_start_optical_zoom->zoom_speed, pod_1088_start_optical_zoom->zoom_direction, pod_1088_start_optical_zoom->ack);
    }
    
}

/**
 * @brief Encode a pod_1088_start_optical_zoom struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1088_start_optical_zoom C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1088_start_optical_zoom_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1088_start_optical_zoom_t* pod_1088_start_optical_zoom)
{
    return _ttalink_pod_1088_start_optical_zoom_encode(dst_addr, src_addr, msg, pod_1088_start_optical_zoom, false);
}

/**
 * @brief Encode a pod_1088_start_optical_zoom struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1088_start_optical_zoom C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1088_start_optical_zoom_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1088_start_optical_zoom_t* pod_1088_start_optical_zoom)
{
    return _ttalink_pod_1088_start_optical_zoom_encode(dst_addr, src_addr, msg, pod_1088_start_optical_zoom, true);
}


static inline uint16_t _ttalink_pod_1088_start_optical_zoom_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1088_start_optical_zoom_t* pod_1088_start_optical_zoom, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1088_start_optical_zoom_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1088_start_optical_zoom->zoom_speed, pod_1088_start_optical_zoom->zoom_direction, pod_1088_start_optical_zoom->ack);
    }else{
        return ttalink_pod_1088_start_optical_zoom_pack_chan(dst_addr, src_addr, chan, msg, pod_1088_start_optical_zoom->zoom_speed, pod_1088_start_optical_zoom->zoom_direction, pod_1088_start_optical_zoom->ack);
    }
}

/**
 * @brief Encode a pod_1088_start_optical_zoom struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1088_start_optical_zoom C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1088_start_optical_zoom_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1088_start_optical_zoom_t* pod_1088_start_optical_zoom)
{
    return _ttalink_pod_1088_start_optical_zoom_encode_chan(dst_addr, src_addr, chan, msg, pod_1088_start_optical_zoom, false);
}

/**
 * @brief Encode a pod_1088_start_optical_zoom struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1088_start_optical_zoom C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1088_start_optical_zoom_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1088_start_optical_zoom_t* pod_1088_start_optical_zoom)
{
    return _ttalink_pod_1088_start_optical_zoom_encode_chan(dst_addr, src_addr, chan, msg, pod_1088_start_optical_zoom, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1088_start_optical_zoom_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t zoom_speed, uint8_t zoom_direction, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN];
    _tta_put_uint8_t(buf, 0, zoom_speed);
    _tta_put_uint8_t(buf, 1, zoom_direction);
    _tta_put_uint8_t(buf, 2, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM, buf, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_MIN_LEN, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_CRC, nocrc);
#else
    ttalink_pod_1088_start_optical_zoom_t packet;
    packet.zoom_speed = zoom_speed;
    packet.zoom_direction = zoom_direction;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM, (const char *)&packet, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_MIN_LEN, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1088_start_optical_zoom message
 * @param chan TTAlink channel to send the message
 *
 * @param zoom_speed Zoom speed.
												72 :  is the slowest. The lens zooms in at the slowest speed.
												73 :  slow. The lens zooms in at a slow speed.
												74 :  a little slower. The magnification speed of the lens is slightly slower than the normal speed..
 * @param zoom_direction zoom direction.
													1 :  Zoom in. The lens will zoom in. The focal length increases, the field of view narrows and the magnification increases.
													0 :  Zoom out. The lens zooms. The focal length decreases, the field of view widens and the magnification decreases..
 * @param ack .
 */
static inline void ttalink_pod_1088_start_optical_zoom_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t zoom_speed, uint8_t zoom_direction, uint8_t ack)
{
    _ttalink_pod_1088_start_optical_zoom_send(dst_addr, src_addr, chan, zoom_speed, zoom_direction, ack, false);
}

/**
 * @brief Send a pod_1088_start_optical_zoom message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param zoom_speed Zoom speed.
												72 :  is the slowest. The lens zooms in at the slowest speed.
												73 :  slow. The lens zooms in at a slow speed.
												74 :  a little slower. The magnification speed of the lens is slightly slower than the normal speed..
 * @param zoom_direction zoom direction.
													1 :  Zoom in. The lens will zoom in. The focal length increases, the field of view narrows and the magnification increases.
													0 :  Zoom out. The lens zooms. The focal length decreases, the field of view widens and the magnification decreases..
 * @param ack .
 */
static inline void ttalink_pod_1088_start_optical_zoom_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t zoom_speed, uint8_t zoom_direction, uint8_t ack)
{
    _ttalink_pod_1088_start_optical_zoom_send(dst_addr, src_addr, chan, zoom_speed, zoom_direction, ack, true);
}


static inline void _ttalink_pod_1088_start_optical_zoom_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1088_start_optical_zoom_t* pod_1088_start_optical_zoom, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1088_start_optical_zoom_send_nocrc(dst_addr, src_addr, chan, pod_1088_start_optical_zoom->zoom_speed, pod_1088_start_optical_zoom->zoom_direction, pod_1088_start_optical_zoom->ack);
    }else{
        ttalink_pod_1088_start_optical_zoom_send(dst_addr, src_addr, chan, pod_1088_start_optical_zoom->zoom_speed, pod_1088_start_optical_zoom->zoom_direction, pod_1088_start_optical_zoom->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM, (const char *)pod_1088_start_optical_zoom, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_MIN_LEN, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1088_start_optical_zoom message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1088_start_optical_zoom_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1088_start_optical_zoom_t* pod_1088_start_optical_zoom)
{
    _ttalink_pod_1088_start_optical_zoom_send_struct(dst_addr, src_addr, chan, pod_1088_start_optical_zoom, false);
}

/**
 * @brief Send a pod_1088_start_optical_zoom message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1088_start_optical_zoom_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1088_start_optical_zoom_t* pod_1088_start_optical_zoom)
{
    _ttalink_pod_1088_start_optical_zoom_send_struct(dst_addr, src_addr, chan, pod_1088_start_optical_zoom, true);
}

#if TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1088_start_optical_zoom_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t zoom_speed, uint8_t zoom_direction, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, zoom_speed);
    _tta_put_uint8_t(buf, 1, zoom_direction);
    _tta_put_uint8_t(buf, 2, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM, buf, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_MIN_LEN, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_CRC, nocrc);
#else
    ttalink_pod_1088_start_optical_zoom_t *packet = (ttalink_pod_1088_start_optical_zoom_t *)msgbuf;
    packet->zoom_speed = zoom_speed;
    packet->zoom_direction = zoom_direction;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM, (const char *)packet, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_MIN_LEN, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1088_start_optical_zoom_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t zoom_speed, uint8_t zoom_direction, uint8_t ack)
{
    _ttalink_pod_1088_start_optical_zoom_send_buf(dst_addr, src_addr, msgbuf, chan, zoom_speed, zoom_direction, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1088_start_optical_zoom_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t zoom_speed, uint8_t zoom_direction, uint8_t ack)
{
    _ttalink_pod_1088_start_optical_zoom_send_buf(dst_addr, src_addr, msgbuf, chan, zoom_speed, zoom_direction, ack, true);
}
#endif

#endif

// MESSAGE POD_1088_START_OPTICAL_ZOOM UNPACKING


/**
 * @brief Get field zoom_speed from pod_1088_start_optical_zoom message
 *
 * @return Zoom speed.
												72 :  is the slowest. The lens zooms in at the slowest speed.
												73 :  slow. The lens zooms in at a slow speed.
												74 :  a little slower. The magnification speed of the lens is slightly slower than the normal speed..
 */
static inline uint8_t ttalink_pod_1088_start_optical_zoom_get_zoom_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field zoom_direction from pod_1088_start_optical_zoom message
 *
 * @return zoom direction.
													1 :  Zoom in. The lens will zoom in. The focal length increases, the field of view narrows and the magnification increases.
													0 :  Zoom out. The lens zooms. The focal length decreases, the field of view widens and the magnification decreases..
 */
static inline uint8_t ttalink_pod_1088_start_optical_zoom_get_zoom_direction(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field ack from pod_1088_start_optical_zoom message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1088_start_optical_zoom_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a pod_1088_start_optical_zoom message into a struct
 *
 * @param msg The message to decode
 * @param pod_1088_start_optical_zoom C-struct to decode the message contents into
 */
static inline void ttalink_pod_1088_start_optical_zoom_decode(const ttalink_message_t* msg, ttalink_pod_1088_start_optical_zoom_t* pod_1088_start_optical_zoom)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1088_start_optical_zoom->zoom_speed = ttalink_pod_1088_start_optical_zoom_get_zoom_speed(msg);
    pod_1088_start_optical_zoom->zoom_direction = ttalink_pod_1088_start_optical_zoom_get_zoom_direction(msg);
    pod_1088_start_optical_zoom->ack = ttalink_pod_1088_start_optical_zoom_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN? msg->len : TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN;
        memset(pod_1088_start_optical_zoom, 0, TTALINK_MSG_ID_POD_1088_START_OPTICAL_ZOOM_LEN);
    memcpy(pod_1088_start_optical_zoom, _TTA_PAYLOAD(msg), len);
#endif
}
