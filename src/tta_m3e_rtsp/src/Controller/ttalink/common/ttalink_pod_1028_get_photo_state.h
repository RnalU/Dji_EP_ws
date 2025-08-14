#pragma once
// MESSAGE POD_1028_GET_PHOTO_STATE PACKING

#define TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE 11028

TTAPACKED(
typedef struct __ttalink_pod_1028_get_photo_state_t {
 uint8_t ack; /*<  */
 uint8_t ack_state; /*<  Bit [0] :  taking a single picture;
													Bit [1] :  is the interval photo taken;
													Bit [2] :  taking a burst picture;
													Bit [3] :  store photos;
													Bit [4-7] :  reserved.*/
 uint16_t ack_delay; /*<  Interval Photography countdown in seconds.*/
}) ttalink_pod_1028_get_photo_state_t;

#define TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN 4
#define TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_MIN_LEN 4
#define TTALINK_MSG_ID_11028_LEN 4
#define TTALINK_MSG_ID_11028_MIN_LEN 4

#define TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_CRC 49
#define TTALINK_MSG_ID_11028_CRC 49



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1028_GET_PHOTO_STATE { \
    11028, \
    "POD_1028_GET_PHOTO_STATE", \
    3, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1028_get_photo_state_t, ack) }, \
         { "ack_state", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1028_get_photo_state_t, ack_state) }, \
         { "ack_delay", NULL, TTALINK_TYPE_UINT16_T, 0, 2, offsetof(ttalink_pod_1028_get_photo_state_t, ack_delay) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1028_GET_PHOTO_STATE { \
    "POD_1028_GET_PHOTO_STATE", \
    3, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1028_get_photo_state_t, ack) }, \
         { "ack_state", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1028_get_photo_state_t, ack_state) }, \
         { "ack_delay", NULL, TTALINK_TYPE_UINT16_T, 0, 2, offsetof(ttalink_pod_1028_get_photo_state_t, ack_delay) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1028_get_photo_state_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_state, uint16_t ack_delay, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_state);
    _tta_put_uint16_t(buf, 2, ack_delay);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN);
#else
    ttalink_pod_1028_get_photo_state_t packet;
    packet.ack = ack;
    packet.ack_state = ack_state;
    packet.ack_delay = ack_delay;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_MIN_LEN, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_CRC, nocrc);
}

/**
 * @brief Pack a pod_1028_get_photo_state message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack  
 * @param ack_state  Bit [0] :  taking a single picture;
													Bit [1] :  is the interval photo taken;
													Bit [2] :  taking a burst picture;
													Bit [3] :  store photos;
													Bit [4-7] :  reserved.
 * @param ack_delay  Interval Photography countdown in seconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1028_get_photo_state_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_state, uint16_t ack_delay)
{
    return _ttalink_pod_1028_get_photo_state_pack(dst_addr, src_addr, msg,  ack, ack_state, ack_delay, false);
}

/**
 * @brief Pack a pod_1028_get_photo_state message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack  
 * @param ack_state  Bit [0] :  taking a single picture;
													Bit [1] :  is the interval photo taken;
													Bit [2] :  taking a burst picture;
													Bit [3] :  store photos;
													Bit [4-7] :  reserved.
 * @param ack_delay  Interval Photography countdown in seconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1028_get_photo_state_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_state, uint16_t ack_delay)
{
    return _ttalink_pod_1028_get_photo_state_pack(dst_addr, src_addr, msg,  ack, ack_state, ack_delay, true);
}


static inline uint16_t _ttalink_pod_1028_get_photo_state_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_state,uint16_t ack_delay, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_state);
    _tta_put_uint16_t(buf, 2, ack_delay);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN);
#else
    ttalink_pod_1028_get_photo_state_t packet;
    packet.ack = ack;
    packet.ack_state = ack_state;
    packet.ack_delay = ack_delay;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_MIN_LEN, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_CRC, nocrc);
}

/**
 * @brief Pack a pod_1028_get_photo_state message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack  
 * @param ack_state  Bit [0] :  taking a single picture;
													Bit [1] :  is the interval photo taken;
													Bit [2] :  taking a burst picture;
													Bit [3] :  store photos;
													Bit [4-7] :  reserved.
 * @param ack_delay  Interval Photography countdown in seconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1028_get_photo_state_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_state,uint16_t ack_delay)
{
    return _ttalink_pod_1028_get_photo_state_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_state, ack_delay, false);
}

/**
 * @brief Pack a pod_1028_get_photo_state message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack  
 * @param ack_state  Bit [0] :  taking a single picture;
													Bit [1] :  is the interval photo taken;
													Bit [2] :  taking a burst picture;
													Bit [3] :  store photos;
													Bit [4-7] :  reserved.
 * @param ack_delay  Interval Photography countdown in seconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1028_get_photo_state_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_state,uint16_t ack_delay)
{
    return _ttalink_pod_1028_get_photo_state_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_state, ack_delay, true);
}


static inline uint16_t _ttalink_pod_1028_get_photo_state_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1028_get_photo_state_t* pod_1028_get_photo_state, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1028_get_photo_state_pack_nocrc(dst_addr, src_addr, msg, pod_1028_get_photo_state->ack, pod_1028_get_photo_state->ack_state, pod_1028_get_photo_state->ack_delay);
    }else{
        return ttalink_pod_1028_get_photo_state_pack(dst_addr, src_addr, msg, pod_1028_get_photo_state->ack, pod_1028_get_photo_state->ack_state, pod_1028_get_photo_state->ack_delay);
    }
    
}

/**
 * @brief Encode a pod_1028_get_photo_state struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1028_get_photo_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1028_get_photo_state_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1028_get_photo_state_t* pod_1028_get_photo_state)
{
    return _ttalink_pod_1028_get_photo_state_encode(dst_addr, src_addr, msg, pod_1028_get_photo_state, false);
}

/**
 * @brief Encode a pod_1028_get_photo_state struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1028_get_photo_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1028_get_photo_state_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1028_get_photo_state_t* pod_1028_get_photo_state)
{
    return _ttalink_pod_1028_get_photo_state_encode(dst_addr, src_addr, msg, pod_1028_get_photo_state, true);
}


static inline uint16_t _ttalink_pod_1028_get_photo_state_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1028_get_photo_state_t* pod_1028_get_photo_state, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1028_get_photo_state_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1028_get_photo_state->ack, pod_1028_get_photo_state->ack_state, pod_1028_get_photo_state->ack_delay);
    }else{
        return ttalink_pod_1028_get_photo_state_pack_chan(dst_addr, src_addr, chan, msg, pod_1028_get_photo_state->ack, pod_1028_get_photo_state->ack_state, pod_1028_get_photo_state->ack_delay);
    }
}

/**
 * @brief Encode a pod_1028_get_photo_state struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1028_get_photo_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1028_get_photo_state_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1028_get_photo_state_t* pod_1028_get_photo_state)
{
    return _ttalink_pod_1028_get_photo_state_encode_chan(dst_addr, src_addr, chan, msg, pod_1028_get_photo_state, false);
}

/**
 * @brief Encode a pod_1028_get_photo_state struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1028_get_photo_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1028_get_photo_state_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1028_get_photo_state_t* pod_1028_get_photo_state)
{
    return _ttalink_pod_1028_get_photo_state_encode_chan(dst_addr, src_addr, chan, msg, pod_1028_get_photo_state, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1028_get_photo_state_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_state, uint16_t ack_delay, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_state);
    _tta_put_uint16_t(buf, 2, ack_delay);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE, buf, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_MIN_LEN, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_CRC, nocrc);
#else
    ttalink_pod_1028_get_photo_state_t packet;
    packet.ack = ack;
    packet.ack_state = ack_state;
    packet.ack_delay = ack_delay;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE, (const char *)&packet, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_MIN_LEN, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1028_get_photo_state message
 * @param chan TTAlink channel to send the message
 *
 * @param ack  
 * @param ack_state  Bit [0] :  taking a single picture;
													Bit [1] :  is the interval photo taken;
													Bit [2] :  taking a burst picture;
													Bit [3] :  store photos;
													Bit [4-7] :  reserved.
 * @param ack_delay  Interval Photography countdown in seconds.
 */
static inline void ttalink_pod_1028_get_photo_state_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_state, uint16_t ack_delay)
{
    _ttalink_pod_1028_get_photo_state_send(dst_addr, src_addr, chan, ack, ack_state, ack_delay, false);
}

/**
 * @brief Send a pod_1028_get_photo_state message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack  
 * @param ack_state  Bit [0] :  taking a single picture;
													Bit [1] :  is the interval photo taken;
													Bit [2] :  taking a burst picture;
													Bit [3] :  store photos;
													Bit [4-7] :  reserved.
 * @param ack_delay  Interval Photography countdown in seconds.
 */
static inline void ttalink_pod_1028_get_photo_state_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_state, uint16_t ack_delay)
{
    _ttalink_pod_1028_get_photo_state_send(dst_addr, src_addr, chan, ack, ack_state, ack_delay, true);
}


static inline void _ttalink_pod_1028_get_photo_state_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1028_get_photo_state_t* pod_1028_get_photo_state, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1028_get_photo_state_send_nocrc(dst_addr, src_addr, chan, pod_1028_get_photo_state->ack, pod_1028_get_photo_state->ack_state, pod_1028_get_photo_state->ack_delay);
    }else{
        ttalink_pod_1028_get_photo_state_send(dst_addr, src_addr, chan, pod_1028_get_photo_state->ack, pod_1028_get_photo_state->ack_state, pod_1028_get_photo_state->ack_delay);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE, (const char *)pod_1028_get_photo_state, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_MIN_LEN, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1028_get_photo_state message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1028_get_photo_state_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1028_get_photo_state_t* pod_1028_get_photo_state)
{
    _ttalink_pod_1028_get_photo_state_send_struct(dst_addr, src_addr, chan, pod_1028_get_photo_state, false);
}

/**
 * @brief Send a pod_1028_get_photo_state message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1028_get_photo_state_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1028_get_photo_state_t* pod_1028_get_photo_state)
{
    _ttalink_pod_1028_get_photo_state_send_struct(dst_addr, src_addr, chan, pod_1028_get_photo_state, true);
}

#if TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1028_get_photo_state_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_state, uint16_t ack_delay, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_state);
    _tta_put_uint16_t(buf, 2, ack_delay);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE, buf, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_MIN_LEN, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_CRC, nocrc);
#else
    ttalink_pod_1028_get_photo_state_t *packet = (ttalink_pod_1028_get_photo_state_t *)msgbuf;
    packet->ack = ack;
    packet->ack_state = ack_state;
    packet->ack_delay = ack_delay;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE, (const char *)packet, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_MIN_LEN, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1028_get_photo_state_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_state, uint16_t ack_delay)
{
    _ttalink_pod_1028_get_photo_state_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_state, ack_delay, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1028_get_photo_state_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_state, uint16_t ack_delay)
{
    _ttalink_pod_1028_get_photo_state_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_state, ack_delay, true);
}
#endif

#endif

// MESSAGE POD_1028_GET_PHOTO_STATE UNPACKING


/**
 * @brief Get field ack from pod_1028_get_photo_state message
 *
 * @return  
 */
static inline uint8_t ttalink_pod_1028_get_photo_state_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack_state from pod_1028_get_photo_state message
 *
 * @return  Bit [0] :  taking a single picture;
													Bit [1] :  is the interval photo taken;
													Bit [2] :  taking a burst picture;
													Bit [3] :  store photos;
													Bit [4-7] :  reserved.
 */
static inline uint8_t ttalink_pod_1028_get_photo_state_get_ack_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field ack_delay from pod_1028_get_photo_state message
 *
 * @return  Interval Photography countdown in seconds.
 */
static inline uint16_t ttalink_pod_1028_get_photo_state_get_ack_delay(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a pod_1028_get_photo_state message into a struct
 *
 * @param msg The message to decode
 * @param pod_1028_get_photo_state C-struct to decode the message contents into
 */
static inline void ttalink_pod_1028_get_photo_state_decode(const ttalink_message_t* msg, ttalink_pod_1028_get_photo_state_t* pod_1028_get_photo_state)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1028_get_photo_state->ack = ttalink_pod_1028_get_photo_state_get_ack(msg);
    pod_1028_get_photo_state->ack_state = ttalink_pod_1028_get_photo_state_get_ack_state(msg);
    pod_1028_get_photo_state->ack_delay = ttalink_pod_1028_get_photo_state_get_ack_delay(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN? msg->len : TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN;
        memset(pod_1028_get_photo_state, 0, TTALINK_MSG_ID_POD_1028_GET_PHOTO_STATE_LEN);
    memcpy(pod_1028_get_photo_state, _TTA_PAYLOAD(msg), len);
#endif
}
