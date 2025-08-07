#pragma once
// MESSAGE POD_1651_SEARCHLIGHT_MODE_CTRL PACKING

#define TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL 11651

TTAPACKED(
typedef struct __ttalink_pod_1651_searchlight_mode_ctrl_t {
 uint8_t switch_mode; /*< 0=Light 1=twinkle 2=breath*/
 uint16_t start_time; /*< 0~65535 ms.*/
 uint16_t end_time; /*< 0~65535 ms.*/
 uint8_t ack; /*< .*/
}) ttalink_pod_1651_searchlight_mode_ctrl_t;

#define TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN 6
#define TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_MIN_LEN 6
#define TTALINK_MSG_ID_11651_LEN 6
#define TTALINK_MSG_ID_11651_MIN_LEN 6

#define TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_CRC 36
#define TTALINK_MSG_ID_11651_CRC 36



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1651_SEARCHLIGHT_MODE_CTRL { \
    11651, \
    "POD_1651_SEARCHLIGHT_MODE_CTRL", \
    4, \
    {  { "switch_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1651_searchlight_mode_ctrl_t, switch_mode) }, \
         { "start_time", NULL, TTALINK_TYPE_UINT16_T, 0, 1, offsetof(ttalink_pod_1651_searchlight_mode_ctrl_t, start_time) }, \
         { "end_time", NULL, TTALINK_TYPE_UINT16_T, 0, 3, offsetof(ttalink_pod_1651_searchlight_mode_ctrl_t, end_time) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_pod_1651_searchlight_mode_ctrl_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1651_SEARCHLIGHT_MODE_CTRL { \
    "POD_1651_SEARCHLIGHT_MODE_CTRL", \
    4, \
    {  { "switch_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1651_searchlight_mode_ctrl_t, switch_mode) }, \
         { "start_time", NULL, TTALINK_TYPE_UINT16_T, 0, 1, offsetof(ttalink_pod_1651_searchlight_mode_ctrl_t, start_time) }, \
         { "end_time", NULL, TTALINK_TYPE_UINT16_T, 0, 3, offsetof(ttalink_pod_1651_searchlight_mode_ctrl_t, end_time) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_pod_1651_searchlight_mode_ctrl_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1651_searchlight_mode_ctrl_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t switch_mode, uint16_t start_time, uint16_t end_time, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN];
    _tta_put_uint8_t(buf, 0, switch_mode);
    _tta_put_uint16_t(buf, 1, start_time);
    _tta_put_uint16_t(buf, 3, end_time);
    _tta_put_uint8_t(buf, 5, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN);
#else
    ttalink_pod_1651_searchlight_mode_ctrl_t packet;
    packet.switch_mode = switch_mode;
    packet.start_time = start_time;
    packet.end_time = end_time;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_CRC, nocrc);
}

/**
 * @brief Pack a pod_1651_searchlight_mode_ctrl message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param switch_mode 0=Light 1=twinkle 2=breath
 * @param start_time 0~65535 ms.
 * @param end_time 0~65535 ms.
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1651_searchlight_mode_ctrl_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t switch_mode, uint16_t start_time, uint16_t end_time, uint8_t ack)
{
    return _ttalink_pod_1651_searchlight_mode_ctrl_pack(dst_addr, src_addr, msg,  switch_mode, start_time, end_time, ack, false);
}

/**
 * @brief Pack a pod_1651_searchlight_mode_ctrl message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param switch_mode 0=Light 1=twinkle 2=breath
 * @param start_time 0~65535 ms.
 * @param end_time 0~65535 ms.
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1651_searchlight_mode_ctrl_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t switch_mode, uint16_t start_time, uint16_t end_time, uint8_t ack)
{
    return _ttalink_pod_1651_searchlight_mode_ctrl_pack(dst_addr, src_addr, msg,  switch_mode, start_time, end_time, ack, true);
}


static inline uint16_t _ttalink_pod_1651_searchlight_mode_ctrl_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t switch_mode,uint16_t start_time,uint16_t end_time,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN];
    _tta_put_uint8_t(buf, 0, switch_mode);
    _tta_put_uint16_t(buf, 1, start_time);
    _tta_put_uint16_t(buf, 3, end_time);
    _tta_put_uint8_t(buf, 5, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN);
#else
    ttalink_pod_1651_searchlight_mode_ctrl_t packet;
    packet.switch_mode = switch_mode;
    packet.start_time = start_time;
    packet.end_time = end_time;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_CRC, nocrc);
}

/**
 * @brief Pack a pod_1651_searchlight_mode_ctrl message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param switch_mode 0=Light 1=twinkle 2=breath
 * @param start_time 0~65535 ms.
 * @param end_time 0~65535 ms.
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1651_searchlight_mode_ctrl_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t switch_mode,uint16_t start_time,uint16_t end_time,uint8_t ack)
{
    return _ttalink_pod_1651_searchlight_mode_ctrl_pack_chan(dst_addr, src_addr, chan, msg,  switch_mode, start_time, end_time, ack, false);
}

/**
 * @brief Pack a pod_1651_searchlight_mode_ctrl message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param switch_mode 0=Light 1=twinkle 2=breath
 * @param start_time 0~65535 ms.
 * @param end_time 0~65535 ms.
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1651_searchlight_mode_ctrl_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t switch_mode,uint16_t start_time,uint16_t end_time,uint8_t ack)
{
    return _ttalink_pod_1651_searchlight_mode_ctrl_pack_chan(dst_addr, src_addr, chan, msg,  switch_mode, start_time, end_time, ack, true);
}


static inline uint16_t _ttalink_pod_1651_searchlight_mode_ctrl_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1651_searchlight_mode_ctrl_t* pod_1651_searchlight_mode_ctrl, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1651_searchlight_mode_ctrl_pack_nocrc(dst_addr, src_addr, msg, pod_1651_searchlight_mode_ctrl->switch_mode, pod_1651_searchlight_mode_ctrl->start_time, pod_1651_searchlight_mode_ctrl->end_time, pod_1651_searchlight_mode_ctrl->ack);
    }else{
        return ttalink_pod_1651_searchlight_mode_ctrl_pack(dst_addr, src_addr, msg, pod_1651_searchlight_mode_ctrl->switch_mode, pod_1651_searchlight_mode_ctrl->start_time, pod_1651_searchlight_mode_ctrl->end_time, pod_1651_searchlight_mode_ctrl->ack);
    }
    
}

/**
 * @brief Encode a pod_1651_searchlight_mode_ctrl struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1651_searchlight_mode_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1651_searchlight_mode_ctrl_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1651_searchlight_mode_ctrl_t* pod_1651_searchlight_mode_ctrl)
{
    return _ttalink_pod_1651_searchlight_mode_ctrl_encode(dst_addr, src_addr, msg, pod_1651_searchlight_mode_ctrl, false);
}

/**
 * @brief Encode a pod_1651_searchlight_mode_ctrl struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1651_searchlight_mode_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1651_searchlight_mode_ctrl_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1651_searchlight_mode_ctrl_t* pod_1651_searchlight_mode_ctrl)
{
    return _ttalink_pod_1651_searchlight_mode_ctrl_encode(dst_addr, src_addr, msg, pod_1651_searchlight_mode_ctrl, true);
}


static inline uint16_t _ttalink_pod_1651_searchlight_mode_ctrl_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1651_searchlight_mode_ctrl_t* pod_1651_searchlight_mode_ctrl, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1651_searchlight_mode_ctrl_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1651_searchlight_mode_ctrl->switch_mode, pod_1651_searchlight_mode_ctrl->start_time, pod_1651_searchlight_mode_ctrl->end_time, pod_1651_searchlight_mode_ctrl->ack);
    }else{
        return ttalink_pod_1651_searchlight_mode_ctrl_pack_chan(dst_addr, src_addr, chan, msg, pod_1651_searchlight_mode_ctrl->switch_mode, pod_1651_searchlight_mode_ctrl->start_time, pod_1651_searchlight_mode_ctrl->end_time, pod_1651_searchlight_mode_ctrl->ack);
    }
}

/**
 * @brief Encode a pod_1651_searchlight_mode_ctrl struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1651_searchlight_mode_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1651_searchlight_mode_ctrl_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1651_searchlight_mode_ctrl_t* pod_1651_searchlight_mode_ctrl)
{
    return _ttalink_pod_1651_searchlight_mode_ctrl_encode_chan(dst_addr, src_addr, chan, msg, pod_1651_searchlight_mode_ctrl, false);
}

/**
 * @brief Encode a pod_1651_searchlight_mode_ctrl struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1651_searchlight_mode_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1651_searchlight_mode_ctrl_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1651_searchlight_mode_ctrl_t* pod_1651_searchlight_mode_ctrl)
{
    return _ttalink_pod_1651_searchlight_mode_ctrl_encode_chan(dst_addr, src_addr, chan, msg, pod_1651_searchlight_mode_ctrl, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1651_searchlight_mode_ctrl_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t switch_mode, uint16_t start_time, uint16_t end_time, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN];
    _tta_put_uint8_t(buf, 0, switch_mode);
    _tta_put_uint16_t(buf, 1, start_time);
    _tta_put_uint16_t(buf, 3, end_time);
    _tta_put_uint8_t(buf, 5, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL, buf, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_CRC, nocrc);
#else
    ttalink_pod_1651_searchlight_mode_ctrl_t packet;
    packet.switch_mode = switch_mode;
    packet.start_time = start_time;
    packet.end_time = end_time;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL, (const char *)&packet, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1651_searchlight_mode_ctrl message
 * @param chan TTAlink channel to send the message
 *
 * @param switch_mode 0=Light 1=twinkle 2=breath
 * @param start_time 0~65535 ms.
 * @param end_time 0~65535 ms.
 * @param ack .
 */
static inline void ttalink_pod_1651_searchlight_mode_ctrl_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t switch_mode, uint16_t start_time, uint16_t end_time, uint8_t ack)
{
    _ttalink_pod_1651_searchlight_mode_ctrl_send(dst_addr, src_addr, chan, switch_mode, start_time, end_time, ack, false);
}

/**
 * @brief Send a pod_1651_searchlight_mode_ctrl message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param switch_mode 0=Light 1=twinkle 2=breath
 * @param start_time 0~65535 ms.
 * @param end_time 0~65535 ms.
 * @param ack .
 */
static inline void ttalink_pod_1651_searchlight_mode_ctrl_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t switch_mode, uint16_t start_time, uint16_t end_time, uint8_t ack)
{
    _ttalink_pod_1651_searchlight_mode_ctrl_send(dst_addr, src_addr, chan, switch_mode, start_time, end_time, ack, true);
}


static inline void _ttalink_pod_1651_searchlight_mode_ctrl_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1651_searchlight_mode_ctrl_t* pod_1651_searchlight_mode_ctrl, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1651_searchlight_mode_ctrl_send_nocrc(dst_addr, src_addr, chan, pod_1651_searchlight_mode_ctrl->switch_mode, pod_1651_searchlight_mode_ctrl->start_time, pod_1651_searchlight_mode_ctrl->end_time, pod_1651_searchlight_mode_ctrl->ack);
    }else{
        ttalink_pod_1651_searchlight_mode_ctrl_send(dst_addr, src_addr, chan, pod_1651_searchlight_mode_ctrl->switch_mode, pod_1651_searchlight_mode_ctrl->start_time, pod_1651_searchlight_mode_ctrl->end_time, pod_1651_searchlight_mode_ctrl->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL, (const char *)pod_1651_searchlight_mode_ctrl, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1651_searchlight_mode_ctrl message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1651_searchlight_mode_ctrl_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1651_searchlight_mode_ctrl_t* pod_1651_searchlight_mode_ctrl)
{
    _ttalink_pod_1651_searchlight_mode_ctrl_send_struct(dst_addr, src_addr, chan, pod_1651_searchlight_mode_ctrl, false);
}

/**
 * @brief Send a pod_1651_searchlight_mode_ctrl message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1651_searchlight_mode_ctrl_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1651_searchlight_mode_ctrl_t* pod_1651_searchlight_mode_ctrl)
{
    _ttalink_pod_1651_searchlight_mode_ctrl_send_struct(dst_addr, src_addr, chan, pod_1651_searchlight_mode_ctrl, true);
}

#if TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1651_searchlight_mode_ctrl_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t switch_mode, uint16_t start_time, uint16_t end_time, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, switch_mode);
    _tta_put_uint16_t(buf, 1, start_time);
    _tta_put_uint16_t(buf, 3, end_time);
    _tta_put_uint8_t(buf, 5, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL, buf, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_CRC, nocrc);
#else
    ttalink_pod_1651_searchlight_mode_ctrl_t *packet = (ttalink_pod_1651_searchlight_mode_ctrl_t *)msgbuf;
    packet->switch_mode = switch_mode;
    packet->start_time = start_time;
    packet->end_time = end_time;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL, (const char *)packet, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1651_searchlight_mode_ctrl_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t switch_mode, uint16_t start_time, uint16_t end_time, uint8_t ack)
{
    _ttalink_pod_1651_searchlight_mode_ctrl_send_buf(dst_addr, src_addr, msgbuf, chan, switch_mode, start_time, end_time, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1651_searchlight_mode_ctrl_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t switch_mode, uint16_t start_time, uint16_t end_time, uint8_t ack)
{
    _ttalink_pod_1651_searchlight_mode_ctrl_send_buf(dst_addr, src_addr, msgbuf, chan, switch_mode, start_time, end_time, ack, true);
}
#endif

#endif

// MESSAGE POD_1651_SEARCHLIGHT_MODE_CTRL UNPACKING


/**
 * @brief Get field switch_mode from pod_1651_searchlight_mode_ctrl message
 *
 * @return 0=Light 1=twinkle 2=breath
 */
static inline uint8_t ttalink_pod_1651_searchlight_mode_ctrl_get_switch_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field start_time from pod_1651_searchlight_mode_ctrl message
 *
 * @return 0~65535 ms.
 */
static inline uint16_t ttalink_pod_1651_searchlight_mode_ctrl_get_start_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  1);
}

/**
 * @brief Get field end_time from pod_1651_searchlight_mode_ctrl message
 *
 * @return 0~65535 ms.
 */
static inline uint16_t ttalink_pod_1651_searchlight_mode_ctrl_get_end_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  3);
}

/**
 * @brief Get field ack from pod_1651_searchlight_mode_ctrl message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1651_searchlight_mode_ctrl_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Decode a pod_1651_searchlight_mode_ctrl message into a struct
 *
 * @param msg The message to decode
 * @param pod_1651_searchlight_mode_ctrl C-struct to decode the message contents into
 */
static inline void ttalink_pod_1651_searchlight_mode_ctrl_decode(const ttalink_message_t* msg, ttalink_pod_1651_searchlight_mode_ctrl_t* pod_1651_searchlight_mode_ctrl)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1651_searchlight_mode_ctrl->switch_mode = ttalink_pod_1651_searchlight_mode_ctrl_get_switch_mode(msg);
    pod_1651_searchlight_mode_ctrl->start_time = ttalink_pod_1651_searchlight_mode_ctrl_get_start_time(msg);
    pod_1651_searchlight_mode_ctrl->end_time = ttalink_pod_1651_searchlight_mode_ctrl_get_end_time(msg);
    pod_1651_searchlight_mode_ctrl->ack = ttalink_pod_1651_searchlight_mode_ctrl_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN? msg->len : TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN;
        memset(pod_1651_searchlight_mode_ctrl, 0, TTALINK_MSG_ID_POD_1651_SEARCHLIGHT_MODE_CTRL_LEN);
    memcpy(pod_1651_searchlight_mode_ctrl, _TTA_PAYLOAD(msg), len);
#endif
}
