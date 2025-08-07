#pragma once
// MESSAGE SET_WAYPOINT_MISSION_ACK PACKING

#define TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK 2197

TTAPACKED(
typedef struct __ttalink_set_waypoint_mission_ack_t {
 int32_t index; /*<  .*/
 uint8_t item_cmd; /*<  .*/
 uint8_t ack; /*<  .*/
}) ttalink_set_waypoint_mission_ack_t;

#define TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN 6
#define TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_MIN_LEN 6
#define TTALINK_MSG_ID_2197_LEN 6
#define TTALINK_MSG_ID_2197_MIN_LEN 6

#define TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_CRC 113
#define TTALINK_MSG_ID_2197_CRC 113



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SET_WAYPOINT_MISSION_ACK { \
    2197, \
    "SET_WAYPOINT_MISSION_ACK", \
    3, \
    {  { "index", NULL, TTALINK_TYPE_INT32_T, 0, 0, offsetof(ttalink_set_waypoint_mission_ack_t, index) }, \
         { "item_cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_set_waypoint_mission_ack_t, item_cmd) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_set_waypoint_mission_ack_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SET_WAYPOINT_MISSION_ACK { \
    "SET_WAYPOINT_MISSION_ACK", \
    3, \
    {  { "index", NULL, TTALINK_TYPE_INT32_T, 0, 0, offsetof(ttalink_set_waypoint_mission_ack_t, index) }, \
         { "item_cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_set_waypoint_mission_ack_t, item_cmd) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_set_waypoint_mission_ack_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_set_waypoint_mission_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int32_t index, uint8_t item_cmd, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN];
    _tta_put_int32_t(buf, 0, index);
    _tta_put_uint8_t(buf, 4, item_cmd);
    _tta_put_uint8_t(buf, 5, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN);
#else
    ttalink_set_waypoint_mission_ack_t packet;
    packet.index = index;
    packet.item_cmd = item_cmd;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_CRC, nocrc);
}

/**
 * @brief Pack a set_waypoint_mission_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param index  .
 * @param item_cmd  .
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_mission_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int32_t index, uint8_t item_cmd, uint8_t ack)
{
    return _ttalink_set_waypoint_mission_ack_pack(dst_addr, src_addr, msg,  index, item_cmd, ack, false);
}

/**
 * @brief Pack a set_waypoint_mission_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param index  .
 * @param item_cmd  .
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_mission_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int32_t index, uint8_t item_cmd, uint8_t ack)
{
    return _ttalink_set_waypoint_mission_ack_pack(dst_addr, src_addr, msg,  index, item_cmd, ack, true);
}


static inline uint16_t _ttalink_set_waypoint_mission_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int32_t index,uint8_t item_cmd,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN];
    _tta_put_int32_t(buf, 0, index);
    _tta_put_uint8_t(buf, 4, item_cmd);
    _tta_put_uint8_t(buf, 5, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN);
#else
    ttalink_set_waypoint_mission_ack_t packet;
    packet.index = index;
    packet.item_cmd = item_cmd;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_CRC, nocrc);
}

/**
 * @brief Pack a set_waypoint_mission_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param index  .
 * @param item_cmd  .
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_mission_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int32_t index,uint8_t item_cmd,uint8_t ack)
{
    return _ttalink_set_waypoint_mission_ack_pack_chan(dst_addr, src_addr, chan, msg,  index, item_cmd, ack, false);
}

/**
 * @brief Pack a set_waypoint_mission_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param index  .
 * @param item_cmd  .
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_mission_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int32_t index,uint8_t item_cmd,uint8_t ack)
{
    return _ttalink_set_waypoint_mission_ack_pack_chan(dst_addr, src_addr, chan, msg,  index, item_cmd, ack, true);
}


static inline uint16_t _ttalink_set_waypoint_mission_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_waypoint_mission_ack_t* set_waypoint_mission_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_set_waypoint_mission_ack_pack_nocrc(dst_addr, src_addr, msg, set_waypoint_mission_ack->index, set_waypoint_mission_ack->item_cmd, set_waypoint_mission_ack->ack);
    }else{
        return ttalink_set_waypoint_mission_ack_pack(dst_addr, src_addr, msg, set_waypoint_mission_ack->index, set_waypoint_mission_ack->item_cmd, set_waypoint_mission_ack->ack);
    }
    
}

/**
 * @brief Encode a set_waypoint_mission_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_mission_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_mission_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_waypoint_mission_ack_t* set_waypoint_mission_ack)
{
    return _ttalink_set_waypoint_mission_ack_encode(dst_addr, src_addr, msg, set_waypoint_mission_ack, false);
}

/**
 * @brief Encode a set_waypoint_mission_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_mission_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_mission_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_waypoint_mission_ack_t* set_waypoint_mission_ack)
{
    return _ttalink_set_waypoint_mission_ack_encode(dst_addr, src_addr, msg, set_waypoint_mission_ack, true);
}


static inline uint16_t _ttalink_set_waypoint_mission_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_waypoint_mission_ack_t* set_waypoint_mission_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_set_waypoint_mission_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, set_waypoint_mission_ack->index, set_waypoint_mission_ack->item_cmd, set_waypoint_mission_ack->ack);
    }else{
        return ttalink_set_waypoint_mission_ack_pack_chan(dst_addr, src_addr, chan, msg, set_waypoint_mission_ack->index, set_waypoint_mission_ack->item_cmd, set_waypoint_mission_ack->ack);
    }
}

/**
 * @brief Encode a set_waypoint_mission_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_mission_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_mission_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_waypoint_mission_ack_t* set_waypoint_mission_ack)
{
    return _ttalink_set_waypoint_mission_ack_encode_chan(dst_addr, src_addr, chan, msg, set_waypoint_mission_ack, false);
}

/**
 * @brief Encode a set_waypoint_mission_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_mission_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_mission_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_waypoint_mission_ack_t* set_waypoint_mission_ack)
{
    return _ttalink_set_waypoint_mission_ack_encode_chan(dst_addr, src_addr, chan, msg, set_waypoint_mission_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_set_waypoint_mission_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int32_t index, uint8_t item_cmd, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN];
    _tta_put_int32_t(buf, 0, index);
    _tta_put_uint8_t(buf, 4, item_cmd);
    _tta_put_uint8_t(buf, 5, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK, buf, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_CRC, nocrc);
#else
    ttalink_set_waypoint_mission_ack_t packet;
    packet.index = index;
    packet.item_cmd = item_cmd;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK, (const char *)&packet, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a set_waypoint_mission_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param index  .
 * @param item_cmd  .
 * @param ack  .
 */
static inline void ttalink_set_waypoint_mission_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int32_t index, uint8_t item_cmd, uint8_t ack)
{
    _ttalink_set_waypoint_mission_ack_send(dst_addr, src_addr, chan, index, item_cmd, ack, false);
}

/**
 * @brief Send a set_waypoint_mission_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param index  .
 * @param item_cmd  .
 * @param ack  .
 */
static inline void ttalink_set_waypoint_mission_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int32_t index, uint8_t item_cmd, uint8_t ack)
{
    _ttalink_set_waypoint_mission_ack_send(dst_addr, src_addr, chan, index, item_cmd, ack, true);
}


static inline void _ttalink_set_waypoint_mission_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_waypoint_mission_ack_t* set_waypoint_mission_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_set_waypoint_mission_ack_send_nocrc(dst_addr, src_addr, chan, set_waypoint_mission_ack->index, set_waypoint_mission_ack->item_cmd, set_waypoint_mission_ack->ack);
    }else{
        ttalink_set_waypoint_mission_ack_send(dst_addr, src_addr, chan, set_waypoint_mission_ack->index, set_waypoint_mission_ack->item_cmd, set_waypoint_mission_ack->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK, (const char *)set_waypoint_mission_ack, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a set_waypoint_mission_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_set_waypoint_mission_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_waypoint_mission_ack_t* set_waypoint_mission_ack)
{
    _ttalink_set_waypoint_mission_ack_send_struct(dst_addr, src_addr, chan, set_waypoint_mission_ack, false);
}

/**
 * @brief Send a set_waypoint_mission_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_set_waypoint_mission_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_waypoint_mission_ack_t* set_waypoint_mission_ack)
{
    _ttalink_set_waypoint_mission_ack_send_struct(dst_addr, src_addr, chan, set_waypoint_mission_ack, true);
}

#if TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_set_waypoint_mission_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int32_t index, uint8_t item_cmd, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_int32_t(buf, 0, index);
    _tta_put_uint8_t(buf, 4, item_cmd);
    _tta_put_uint8_t(buf, 5, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK, buf, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_CRC, nocrc);
#else
    ttalink_set_waypoint_mission_ack_t *packet = (ttalink_set_waypoint_mission_ack_t *)msgbuf;
    packet->index = index;
    packet->item_cmd = item_cmd;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK, (const char *)packet, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_set_waypoint_mission_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int32_t index, uint8_t item_cmd, uint8_t ack)
{
    _ttalink_set_waypoint_mission_ack_send_buf(dst_addr, src_addr, msgbuf, chan, index, item_cmd, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_set_waypoint_mission_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int32_t index, uint8_t item_cmd, uint8_t ack)
{
    _ttalink_set_waypoint_mission_ack_send_buf(dst_addr, src_addr, msgbuf, chan, index, item_cmd, ack, true);
}
#endif

#endif

// MESSAGE SET_WAYPOINT_MISSION_ACK UNPACKING


/**
 * @brief Get field index from set_waypoint_mission_ack message
 *
 * @return  .
 */
static inline int32_t ttalink_set_waypoint_mission_ack_get_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field item_cmd from set_waypoint_mission_ack message
 *
 * @return  .
 */
static inline uint8_t ttalink_set_waypoint_mission_ack_get_item_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field ack from set_waypoint_mission_ack message
 *
 * @return  .
 */
static inline uint8_t ttalink_set_waypoint_mission_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Decode a set_waypoint_mission_ack message into a struct
 *
 * @param msg The message to decode
 * @param set_waypoint_mission_ack C-struct to decode the message contents into
 */
static inline void ttalink_set_waypoint_mission_ack_decode(const ttalink_message_t* msg, ttalink_set_waypoint_mission_ack_t* set_waypoint_mission_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    set_waypoint_mission_ack->index = ttalink_set_waypoint_mission_ack_get_index(msg);
    set_waypoint_mission_ack->item_cmd = ttalink_set_waypoint_mission_ack_get_item_cmd(msg);
    set_waypoint_mission_ack->ack = ttalink_set_waypoint_mission_ack_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN? msg->len : TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN;
        memset(set_waypoint_mission_ack, 0, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_ACK_LEN);
    memcpy(set_waypoint_mission_ack, _TTA_PAYLOAD(msg), len);
#endif
}
