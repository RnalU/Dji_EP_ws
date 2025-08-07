#pragma once
// MESSAGE TC_9001_LED_STATE PACKING

#define TTALINK_MSG_ID_TC_9001_LED_STATE 9001

TTAPACKED(
typedef struct __ttalink_tc_9001_led_state_t {
 uint32_t update_time; /*<  update_time*/
 uint8_t mode; /*<  */
}) ttalink_tc_9001_led_state_t;

#define TTALINK_MSG_ID_TC_9001_LED_STATE_LEN 5
#define TTALINK_MSG_ID_TC_9001_LED_STATE_MIN_LEN 5
#define TTALINK_MSG_ID_9001_LEN 5
#define TTALINK_MSG_ID_9001_MIN_LEN 5

#define TTALINK_MSG_ID_TC_9001_LED_STATE_CRC 246
#define TTALINK_MSG_ID_9001_CRC 246



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_TC_9001_LED_STATE { \
    9001, \
    "TC_9001_LED_STATE", \
    2, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_tc_9001_led_state_t, update_time) }, \
         { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_tc_9001_led_state_t, mode) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_TC_9001_LED_STATE { \
    "TC_9001_LED_STATE", \
    2, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_tc_9001_led_state_t, update_time) }, \
         { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_tc_9001_led_state_t, mode) }, \
         } \
}
#endif


static inline uint16_t _ttalink_tc_9001_led_state_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9001_LED_STATE_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, mode);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TC_9001_LED_STATE_LEN);
#else
    ttalink_tc_9001_led_state_t packet;
    packet.update_time = update_time;
    packet.mode = mode;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TC_9001_LED_STATE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TC_9001_LED_STATE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_TC_9001_LED_STATE_MIN_LEN, TTALINK_MSG_ID_TC_9001_LED_STATE_LEN, TTALINK_MSG_ID_TC_9001_LED_STATE_CRC, nocrc);
}

/**
 * @brief Pack a tc_9001_led_state message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param mode  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9001_led_state_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t mode)
{
    return _ttalink_tc_9001_led_state_pack(dst_addr, src_addr, msg,  update_time, mode, false);
}

/**
 * @brief Pack a tc_9001_led_state message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param mode  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9001_led_state_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t mode)
{
    return _ttalink_tc_9001_led_state_pack(dst_addr, src_addr, msg,  update_time, mode, true);
}


static inline uint16_t _ttalink_tc_9001_led_state_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9001_LED_STATE_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, mode);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TC_9001_LED_STATE_LEN);
#else
    ttalink_tc_9001_led_state_t packet;
    packet.update_time = update_time;
    packet.mode = mode;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TC_9001_LED_STATE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TC_9001_LED_STATE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9001_LED_STATE_MIN_LEN, TTALINK_MSG_ID_TC_9001_LED_STATE_LEN, TTALINK_MSG_ID_TC_9001_LED_STATE_CRC, nocrc);
}

/**
 * @brief Pack a tc_9001_led_state message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param mode  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9001_led_state_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t mode)
{
    return _ttalink_tc_9001_led_state_pack_chan(dst_addr, src_addr, chan, msg,  update_time, mode, false);
}

/**
 * @brief Pack a tc_9001_led_state message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param mode  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9001_led_state_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t mode)
{
    return _ttalink_tc_9001_led_state_pack_chan(dst_addr, src_addr, chan, msg,  update_time, mode, true);
}


static inline uint16_t _ttalink_tc_9001_led_state_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9001_led_state_t* tc_9001_led_state, bool nocrc)
{
    if(nocrc){
        return ttalink_tc_9001_led_state_pack_nocrc(dst_addr, src_addr, msg, tc_9001_led_state->update_time, tc_9001_led_state->mode);
    }else{
        return ttalink_tc_9001_led_state_pack(dst_addr, src_addr, msg, tc_9001_led_state->update_time, tc_9001_led_state->mode);
    }
    
}

/**
 * @brief Encode a tc_9001_led_state struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param tc_9001_led_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9001_led_state_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9001_led_state_t* tc_9001_led_state)
{
    return _ttalink_tc_9001_led_state_encode(dst_addr, src_addr, msg, tc_9001_led_state, false);
}

/**
 * @brief Encode a tc_9001_led_state struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param tc_9001_led_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9001_led_state_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9001_led_state_t* tc_9001_led_state)
{
    return _ttalink_tc_9001_led_state_encode(dst_addr, src_addr, msg, tc_9001_led_state, true);
}


static inline uint16_t _ttalink_tc_9001_led_state_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9001_led_state_t* tc_9001_led_state, bool nocrc)
{
    if(nocrc){
        return ttalink_tc_9001_led_state_pack_chan_nocrc(dst_addr, src_addr, chan, msg, tc_9001_led_state->update_time, tc_9001_led_state->mode);
    }else{
        return ttalink_tc_9001_led_state_pack_chan(dst_addr, src_addr, chan, msg, tc_9001_led_state->update_time, tc_9001_led_state->mode);
    }
}

/**
 * @brief Encode a tc_9001_led_state struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param tc_9001_led_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9001_led_state_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9001_led_state_t* tc_9001_led_state)
{
    return _ttalink_tc_9001_led_state_encode_chan(dst_addr, src_addr, chan, msg, tc_9001_led_state, false);
}

/**
 * @brief Encode a tc_9001_led_state struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param tc_9001_led_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9001_led_state_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9001_led_state_t* tc_9001_led_state)
{
    return _ttalink_tc_9001_led_state_encode_chan(dst_addr, src_addr, chan, msg, tc_9001_led_state, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_tc_9001_led_state_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9001_LED_STATE_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, mode);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9001_LED_STATE, buf, TTALINK_MSG_ID_TC_9001_LED_STATE_MIN_LEN, TTALINK_MSG_ID_TC_9001_LED_STATE_LEN, TTALINK_MSG_ID_TC_9001_LED_STATE_CRC, nocrc);
#else
    ttalink_tc_9001_led_state_t packet;
    packet.update_time = update_time;
    packet.mode = mode;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9001_LED_STATE, (const char *)&packet, TTALINK_MSG_ID_TC_9001_LED_STATE_MIN_LEN, TTALINK_MSG_ID_TC_9001_LED_STATE_LEN, TTALINK_MSG_ID_TC_9001_LED_STATE_CRC, nocrc);
#endif
}

/**
 * @brief Send a tc_9001_led_state message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param mode  
 */
static inline void ttalink_tc_9001_led_state_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t mode)
{
    _ttalink_tc_9001_led_state_send(dst_addr, src_addr, chan, update_time, mode, false);
}

/**
 * @brief Send a tc_9001_led_state message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param mode  
 */
static inline void ttalink_tc_9001_led_state_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t mode)
{
    _ttalink_tc_9001_led_state_send(dst_addr, src_addr, chan, update_time, mode, true);
}


static inline void _ttalink_tc_9001_led_state_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9001_led_state_t* tc_9001_led_state, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_tc_9001_led_state_send_nocrc(dst_addr, src_addr, chan, tc_9001_led_state->update_time, tc_9001_led_state->mode);
    }else{
        ttalink_tc_9001_led_state_send(dst_addr, src_addr, chan, tc_9001_led_state->update_time, tc_9001_led_state->mode);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9001_LED_STATE, (const char *)tc_9001_led_state, TTALINK_MSG_ID_TC_9001_LED_STATE_MIN_LEN, TTALINK_MSG_ID_TC_9001_LED_STATE_LEN, TTALINK_MSG_ID_TC_9001_LED_STATE_CRC, nocrc);
#endif
}

/**
 * @brief Send a tc_9001_led_state message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_tc_9001_led_state_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9001_led_state_t* tc_9001_led_state)
{
    _ttalink_tc_9001_led_state_send_struct(dst_addr, src_addr, chan, tc_9001_led_state, false);
}

/**
 * @brief Send a tc_9001_led_state message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_tc_9001_led_state_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9001_led_state_t* tc_9001_led_state)
{
    _ttalink_tc_9001_led_state_send_struct(dst_addr, src_addr, chan, tc_9001_led_state, true);
}

#if TTALINK_MSG_ID_TC_9001_LED_STATE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_tc_9001_led_state_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, mode);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9001_LED_STATE, buf, TTALINK_MSG_ID_TC_9001_LED_STATE_MIN_LEN, TTALINK_MSG_ID_TC_9001_LED_STATE_LEN, TTALINK_MSG_ID_TC_9001_LED_STATE_CRC, nocrc);
#else
    ttalink_tc_9001_led_state_t *packet = (ttalink_tc_9001_led_state_t *)msgbuf;
    packet->update_time = update_time;
    packet->mode = mode;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9001_LED_STATE, (const char *)packet, TTALINK_MSG_ID_TC_9001_LED_STATE_MIN_LEN, TTALINK_MSG_ID_TC_9001_LED_STATE_LEN, TTALINK_MSG_ID_TC_9001_LED_STATE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_tc_9001_led_state_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t mode)
{
    _ttalink_tc_9001_led_state_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, mode, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_tc_9001_led_state_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t mode)
{
    _ttalink_tc_9001_led_state_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, mode, true);
}
#endif

#endif

// MESSAGE TC_9001_LED_STATE UNPACKING


/**
 * @brief Get field update_time from tc_9001_led_state message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_tc_9001_led_state_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field mode from tc_9001_led_state message
 *
 * @return  
 */
static inline uint8_t ttalink_tc_9001_led_state_get_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Decode a tc_9001_led_state message into a struct
 *
 * @param msg The message to decode
 * @param tc_9001_led_state C-struct to decode the message contents into
 */
static inline void ttalink_tc_9001_led_state_decode(const ttalink_message_t* msg, ttalink_tc_9001_led_state_t* tc_9001_led_state)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    tc_9001_led_state->update_time = ttalink_tc_9001_led_state_get_update_time(msg);
    tc_9001_led_state->mode = ttalink_tc_9001_led_state_get_mode(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_TC_9001_LED_STATE_LEN? msg->len : TTALINK_MSG_ID_TC_9001_LED_STATE_LEN;
        memset(tc_9001_led_state, 0, TTALINK_MSG_ID_TC_9001_LED_STATE_LEN);
    memcpy(tc_9001_led_state, _TTA_PAYLOAD(msg), len);
#endif
}
