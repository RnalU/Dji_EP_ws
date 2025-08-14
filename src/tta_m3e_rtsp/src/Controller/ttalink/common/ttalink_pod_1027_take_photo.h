#pragma once
// MESSAGE POD_1027_TAKE_PHOTO PACKING

#define TTALINK_MSG_ID_POD_1027_TAKE_PHOTO 11027

TTAPACKED(
typedef struct __ttalink_pod_1027_take_photo_t {
 uint8_t state; /*<  0 :  Stop photographing; 1 :  Start taking photos;*/
 uint8_t ack; /*<  .*/
}) ttalink_pod_1027_take_photo_t;

#define TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN 2
#define TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_MIN_LEN 2
#define TTALINK_MSG_ID_11027_LEN 2
#define TTALINK_MSG_ID_11027_MIN_LEN 2

#define TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_CRC 123
#define TTALINK_MSG_ID_11027_CRC 123



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1027_TAKE_PHOTO { \
    11027, \
    "POD_1027_TAKE_PHOTO", \
    2, \
    {  { "state", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1027_take_photo_t, state) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1027_take_photo_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1027_TAKE_PHOTO { \
    "POD_1027_TAKE_PHOTO", \
    2, \
    {  { "state", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1027_take_photo_t, state) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1027_take_photo_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1027_take_photo_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t state, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN];
    _tta_put_uint8_t(buf, 0, state);
    _tta_put_uint8_t(buf, 1, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN);
#else
    ttalink_pod_1027_take_photo_t packet;
    packet.state = state;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1027_TAKE_PHOTO;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_MIN_LEN, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_CRC, nocrc);
}

/**
 * @brief Pack a pod_1027_take_photo message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param state  0 :  Stop photographing; 1 :  Start taking photos;
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1027_take_photo_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t state, uint8_t ack)
{
    return _ttalink_pod_1027_take_photo_pack(dst_addr, src_addr, msg,  state, ack, false);
}

/**
 * @brief Pack a pod_1027_take_photo message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param state  0 :  Stop photographing; 1 :  Start taking photos;
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1027_take_photo_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t state, uint8_t ack)
{
    return _ttalink_pod_1027_take_photo_pack(dst_addr, src_addr, msg,  state, ack, true);
}


static inline uint16_t _ttalink_pod_1027_take_photo_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t state,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN];
    _tta_put_uint8_t(buf, 0, state);
    _tta_put_uint8_t(buf, 1, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN);
#else
    ttalink_pod_1027_take_photo_t packet;
    packet.state = state;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1027_TAKE_PHOTO;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_MIN_LEN, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_CRC, nocrc);
}

/**
 * @brief Pack a pod_1027_take_photo message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param state  0 :  Stop photographing; 1 :  Start taking photos;
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1027_take_photo_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t state,uint8_t ack)
{
    return _ttalink_pod_1027_take_photo_pack_chan(dst_addr, src_addr, chan, msg,  state, ack, false);
}

/**
 * @brief Pack a pod_1027_take_photo message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param state  0 :  Stop photographing; 1 :  Start taking photos;
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1027_take_photo_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t state,uint8_t ack)
{
    return _ttalink_pod_1027_take_photo_pack_chan(dst_addr, src_addr, chan, msg,  state, ack, true);
}


static inline uint16_t _ttalink_pod_1027_take_photo_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1027_take_photo_t* pod_1027_take_photo, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1027_take_photo_pack_nocrc(dst_addr, src_addr, msg, pod_1027_take_photo->state, pod_1027_take_photo->ack);
    }else{
        return ttalink_pod_1027_take_photo_pack(dst_addr, src_addr, msg, pod_1027_take_photo->state, pod_1027_take_photo->ack);
    }
    
}

/**
 * @brief Encode a pod_1027_take_photo struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1027_take_photo C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1027_take_photo_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1027_take_photo_t* pod_1027_take_photo)
{
    return _ttalink_pod_1027_take_photo_encode(dst_addr, src_addr, msg, pod_1027_take_photo, false);
}

/**
 * @brief Encode a pod_1027_take_photo struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1027_take_photo C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1027_take_photo_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1027_take_photo_t* pod_1027_take_photo)
{
    return _ttalink_pod_1027_take_photo_encode(dst_addr, src_addr, msg, pod_1027_take_photo, true);
}


static inline uint16_t _ttalink_pod_1027_take_photo_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1027_take_photo_t* pod_1027_take_photo, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1027_take_photo_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1027_take_photo->state, pod_1027_take_photo->ack);
    }else{
        return ttalink_pod_1027_take_photo_pack_chan(dst_addr, src_addr, chan, msg, pod_1027_take_photo->state, pod_1027_take_photo->ack);
    }
}

/**
 * @brief Encode a pod_1027_take_photo struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1027_take_photo C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1027_take_photo_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1027_take_photo_t* pod_1027_take_photo)
{
    return _ttalink_pod_1027_take_photo_encode_chan(dst_addr, src_addr, chan, msg, pod_1027_take_photo, false);
}

/**
 * @brief Encode a pod_1027_take_photo struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1027_take_photo C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1027_take_photo_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1027_take_photo_t* pod_1027_take_photo)
{
    return _ttalink_pod_1027_take_photo_encode_chan(dst_addr, src_addr, chan, msg, pod_1027_take_photo, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1027_take_photo_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t state, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN];
    _tta_put_uint8_t(buf, 0, state);
    _tta_put_uint8_t(buf, 1, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO, buf, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_MIN_LEN, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_CRC, nocrc);
#else
    ttalink_pod_1027_take_photo_t packet;
    packet.state = state;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO, (const char *)&packet, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_MIN_LEN, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1027_take_photo message
 * @param chan TTAlink channel to send the message
 *
 * @param state  0 :  Stop photographing; 1 :  Start taking photos;
 * @param ack  .
 */
static inline void ttalink_pod_1027_take_photo_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t state, uint8_t ack)
{
    _ttalink_pod_1027_take_photo_send(dst_addr, src_addr, chan, state, ack, false);
}

/**
 * @brief Send a pod_1027_take_photo message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param state  0 :  Stop photographing; 1 :  Start taking photos;
 * @param ack  .
 */
static inline void ttalink_pod_1027_take_photo_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t state, uint8_t ack)
{
    _ttalink_pod_1027_take_photo_send(dst_addr, src_addr, chan, state, ack, true);
}


static inline void _ttalink_pod_1027_take_photo_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1027_take_photo_t* pod_1027_take_photo, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1027_take_photo_send_nocrc(dst_addr, src_addr, chan, pod_1027_take_photo->state, pod_1027_take_photo->ack);
    }else{
        ttalink_pod_1027_take_photo_send(dst_addr, src_addr, chan, pod_1027_take_photo->state, pod_1027_take_photo->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO, (const char *)pod_1027_take_photo, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_MIN_LEN, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1027_take_photo message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1027_take_photo_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1027_take_photo_t* pod_1027_take_photo)
{
    _ttalink_pod_1027_take_photo_send_struct(dst_addr, src_addr, chan, pod_1027_take_photo, false);
}

/**
 * @brief Send a pod_1027_take_photo message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1027_take_photo_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1027_take_photo_t* pod_1027_take_photo)
{
    _ttalink_pod_1027_take_photo_send_struct(dst_addr, src_addr, chan, pod_1027_take_photo, true);
}

#if TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1027_take_photo_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t state, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, state);
    _tta_put_uint8_t(buf, 1, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO, buf, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_MIN_LEN, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_CRC, nocrc);
#else
    ttalink_pod_1027_take_photo_t *packet = (ttalink_pod_1027_take_photo_t *)msgbuf;
    packet->state = state;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO, (const char *)packet, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_MIN_LEN, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1027_take_photo_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t state, uint8_t ack)
{
    _ttalink_pod_1027_take_photo_send_buf(dst_addr, src_addr, msgbuf, chan, state, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1027_take_photo_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t state, uint8_t ack)
{
    _ttalink_pod_1027_take_photo_send_buf(dst_addr, src_addr, msgbuf, chan, state, ack, true);
}
#endif

#endif

// MESSAGE POD_1027_TAKE_PHOTO UNPACKING


/**
 * @brief Get field state from pod_1027_take_photo message
 *
 * @return  0 :  Stop photographing; 1 :  Start taking photos;
 */
static inline uint8_t ttalink_pod_1027_take_photo_get_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack from pod_1027_take_photo message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_1027_take_photo_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a pod_1027_take_photo message into a struct
 *
 * @param msg The message to decode
 * @param pod_1027_take_photo C-struct to decode the message contents into
 */
static inline void ttalink_pod_1027_take_photo_decode(const ttalink_message_t* msg, ttalink_pod_1027_take_photo_t* pod_1027_take_photo)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1027_take_photo->state = ttalink_pod_1027_take_photo_get_state(msg);
    pod_1027_take_photo->ack = ttalink_pod_1027_take_photo_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN? msg->len : TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN;
        memset(pod_1027_take_photo, 0, TTALINK_MSG_ID_POD_1027_TAKE_PHOTO_LEN);
    memcpy(pod_1027_take_photo, _TTA_PAYLOAD(msg), len);
#endif
}
