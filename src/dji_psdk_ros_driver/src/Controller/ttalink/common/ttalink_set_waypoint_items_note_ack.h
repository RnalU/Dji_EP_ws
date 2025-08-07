#pragma once
// MESSAGE SET_WAYPOINT_ITEMS_NOTE_ACK PACKING

#define TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK 2207

TTAPACKED(
typedef struct __ttalink_set_waypoint_items_note_ack_t {
 uint8_t ack; /*<  (非0为错误). */
 float param1; /*<  .*/
 float param2; /*<  .*/
}) ttalink_set_waypoint_items_note_ack_t;

#define TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN 9
#define TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_MIN_LEN 9
#define TTALINK_MSG_ID_2207_LEN 9
#define TTALINK_MSG_ID_2207_MIN_LEN 9

#define TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_CRC 104
#define TTALINK_MSG_ID_2207_CRC 104



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SET_WAYPOINT_ITEMS_NOTE_ACK { \
    2207, \
    "SET_WAYPOINT_ITEMS_NOTE_ACK", \
    3, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_set_waypoint_items_note_ack_t, ack) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 1, offsetof(ttalink_set_waypoint_items_note_ack_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 5, offsetof(ttalink_set_waypoint_items_note_ack_t, param2) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SET_WAYPOINT_ITEMS_NOTE_ACK { \
    "SET_WAYPOINT_ITEMS_NOTE_ACK", \
    3, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_set_waypoint_items_note_ack_t, ack) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 1, offsetof(ttalink_set_waypoint_items_note_ack_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 5, offsetof(ttalink_set_waypoint_items_note_ack_t, param2) }, \
         } \
}
#endif


static inline uint16_t _ttalink_set_waypoint_items_note_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_float(buf, 1, param1);
    _tta_put_float(buf, 5, param2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN);
#else
    ttalink_set_waypoint_items_note_ack_t packet;
    packet.ack = ack;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_CRC, nocrc);
}

/**
 * @brief Pack a set_waypoint_items_note_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack  (非0为错误). 
 * @param param1  .
 * @param param2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_items_note_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, float param1, float param2)
{
    return _ttalink_set_waypoint_items_note_ack_pack(dst_addr, src_addr, msg,  ack, param1, param2, false);
}

/**
 * @brief Pack a set_waypoint_items_note_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack  (非0为错误). 
 * @param param1  .
 * @param param2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_items_note_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, float param1, float param2)
{
    return _ttalink_set_waypoint_items_note_ack_pack(dst_addr, src_addr, msg,  ack, param1, param2, true);
}


static inline uint16_t _ttalink_set_waypoint_items_note_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,float param1,float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_float(buf, 1, param1);
    _tta_put_float(buf, 5, param2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN);
#else
    ttalink_set_waypoint_items_note_ack_t packet;
    packet.ack = ack;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_CRC, nocrc);
}

/**
 * @brief Pack a set_waypoint_items_note_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack  (非0为错误). 
 * @param param1  .
 * @param param2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_items_note_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,float param1,float param2)
{
    return _ttalink_set_waypoint_items_note_ack_pack_chan(dst_addr, src_addr, chan, msg,  ack, param1, param2, false);
}

/**
 * @brief Pack a set_waypoint_items_note_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack  (非0为错误). 
 * @param param1  .
 * @param param2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_items_note_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,float param1,float param2)
{
    return _ttalink_set_waypoint_items_note_ack_pack_chan(dst_addr, src_addr, chan, msg,  ack, param1, param2, true);
}


static inline uint16_t _ttalink_set_waypoint_items_note_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_waypoint_items_note_ack_t* set_waypoint_items_note_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_set_waypoint_items_note_ack_pack_nocrc(dst_addr, src_addr, msg, set_waypoint_items_note_ack->ack, set_waypoint_items_note_ack->param1, set_waypoint_items_note_ack->param2);
    }else{
        return ttalink_set_waypoint_items_note_ack_pack(dst_addr, src_addr, msg, set_waypoint_items_note_ack->ack, set_waypoint_items_note_ack->param1, set_waypoint_items_note_ack->param2);
    }
    
}

/**
 * @brief Encode a set_waypoint_items_note_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_items_note_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_items_note_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_waypoint_items_note_ack_t* set_waypoint_items_note_ack)
{
    return _ttalink_set_waypoint_items_note_ack_encode(dst_addr, src_addr, msg, set_waypoint_items_note_ack, false);
}

/**
 * @brief Encode a set_waypoint_items_note_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_items_note_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_items_note_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_waypoint_items_note_ack_t* set_waypoint_items_note_ack)
{
    return _ttalink_set_waypoint_items_note_ack_encode(dst_addr, src_addr, msg, set_waypoint_items_note_ack, true);
}


static inline uint16_t _ttalink_set_waypoint_items_note_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_waypoint_items_note_ack_t* set_waypoint_items_note_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_set_waypoint_items_note_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, set_waypoint_items_note_ack->ack, set_waypoint_items_note_ack->param1, set_waypoint_items_note_ack->param2);
    }else{
        return ttalink_set_waypoint_items_note_ack_pack_chan(dst_addr, src_addr, chan, msg, set_waypoint_items_note_ack->ack, set_waypoint_items_note_ack->param1, set_waypoint_items_note_ack->param2);
    }
}

/**
 * @brief Encode a set_waypoint_items_note_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_items_note_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_items_note_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_waypoint_items_note_ack_t* set_waypoint_items_note_ack)
{
    return _ttalink_set_waypoint_items_note_ack_encode_chan(dst_addr, src_addr, chan, msg, set_waypoint_items_note_ack, false);
}

/**
 * @brief Encode a set_waypoint_items_note_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_items_note_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_items_note_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_waypoint_items_note_ack_t* set_waypoint_items_note_ack)
{
    return _ttalink_set_waypoint_items_note_ack_encode_chan(dst_addr, src_addr, chan, msg, set_waypoint_items_note_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_set_waypoint_items_note_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_float(buf, 1, param1);
    _tta_put_float(buf, 5, param2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK, buf, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_CRC, nocrc);
#else
    ttalink_set_waypoint_items_note_ack_t packet;
    packet.ack = ack;
    packet.param1 = param1;
    packet.param2 = param2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK, (const char *)&packet, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a set_waypoint_items_note_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param ack  (非0为错误). 
 * @param param1  .
 * @param param2  .
 */
static inline void ttalink_set_waypoint_items_note_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, float param1, float param2)
{
    _ttalink_set_waypoint_items_note_ack_send(dst_addr, src_addr, chan, ack, param1, param2, false);
}

/**
 * @brief Send a set_waypoint_items_note_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack  (非0为错误). 
 * @param param1  .
 * @param param2  .
 */
static inline void ttalink_set_waypoint_items_note_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, float param1, float param2)
{
    _ttalink_set_waypoint_items_note_ack_send(dst_addr, src_addr, chan, ack, param1, param2, true);
}


static inline void _ttalink_set_waypoint_items_note_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_waypoint_items_note_ack_t* set_waypoint_items_note_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_set_waypoint_items_note_ack_send_nocrc(dst_addr, src_addr, chan, set_waypoint_items_note_ack->ack, set_waypoint_items_note_ack->param1, set_waypoint_items_note_ack->param2);
    }else{
        ttalink_set_waypoint_items_note_ack_send(dst_addr, src_addr, chan, set_waypoint_items_note_ack->ack, set_waypoint_items_note_ack->param1, set_waypoint_items_note_ack->param2);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK, (const char *)set_waypoint_items_note_ack, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a set_waypoint_items_note_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_set_waypoint_items_note_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_waypoint_items_note_ack_t* set_waypoint_items_note_ack)
{
    _ttalink_set_waypoint_items_note_ack_send_struct(dst_addr, src_addr, chan, set_waypoint_items_note_ack, false);
}

/**
 * @brief Send a set_waypoint_items_note_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_set_waypoint_items_note_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_waypoint_items_note_ack_t* set_waypoint_items_note_ack)
{
    _ttalink_set_waypoint_items_note_ack_send_struct(dst_addr, src_addr, chan, set_waypoint_items_note_ack, true);
}

#if TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_set_waypoint_items_note_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_float(buf, 1, param1);
    _tta_put_float(buf, 5, param2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK, buf, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_CRC, nocrc);
#else
    ttalink_set_waypoint_items_note_ack_t *packet = (ttalink_set_waypoint_items_note_ack_t *)msgbuf;
    packet->ack = ack;
    packet->param1 = param1;
    packet->param2 = param2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK, (const char *)packet, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_set_waypoint_items_note_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, float param1, float param2)
{
    _ttalink_set_waypoint_items_note_ack_send_buf(dst_addr, src_addr, msgbuf, chan, ack, param1, param2, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_set_waypoint_items_note_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, float param1, float param2)
{
    _ttalink_set_waypoint_items_note_ack_send_buf(dst_addr, src_addr, msgbuf, chan, ack, param1, param2, true);
}
#endif

#endif

// MESSAGE SET_WAYPOINT_ITEMS_NOTE_ACK UNPACKING


/**
 * @brief Get field ack from set_waypoint_items_note_ack message
 *
 * @return  (非0为错误). 
 */
static inline uint8_t ttalink_set_waypoint_items_note_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field param1 from set_waypoint_items_note_ack message
 *
 * @return  .
 */
static inline float ttalink_set_waypoint_items_note_ack_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  1);
}

/**
 * @brief Get field param2 from set_waypoint_items_note_ack message
 *
 * @return  .
 */
static inline float ttalink_set_waypoint_items_note_ack_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  5);
}

/**
 * @brief Decode a set_waypoint_items_note_ack message into a struct
 *
 * @param msg The message to decode
 * @param set_waypoint_items_note_ack C-struct to decode the message contents into
 */
static inline void ttalink_set_waypoint_items_note_ack_decode(const ttalink_message_t* msg, ttalink_set_waypoint_items_note_ack_t* set_waypoint_items_note_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    set_waypoint_items_note_ack->ack = ttalink_set_waypoint_items_note_ack_get_ack(msg);
    set_waypoint_items_note_ack->param1 = ttalink_set_waypoint_items_note_ack_get_param1(msg);
    set_waypoint_items_note_ack->param2 = ttalink_set_waypoint_items_note_ack_get_param2(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN? msg->len : TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN;
        memset(set_waypoint_items_note_ack, 0, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_ACK_LEN);
    memcpy(set_waypoint_items_note_ack, _TTA_PAYLOAD(msg), len);
#endif
}
