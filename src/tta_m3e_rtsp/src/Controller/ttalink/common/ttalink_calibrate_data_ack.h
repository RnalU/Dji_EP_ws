#pragma once
// MESSAGE CALIBRATE_DATA_ACK PACKING

#define TTALINK_MSG_ID_CALIBRATE_DATA_ACK 2128

TTAPACKED(
typedef struct __ttalink_calibrate_data_ack_t {
 uint8_t index; /*< This is send index.*/
 uint8_t ack; /*< This is ack.*/
}) ttalink_calibrate_data_ack_t;

#define TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN 2
#define TTALINK_MSG_ID_CALIBRATE_DATA_ACK_MIN_LEN 2
#define TTALINK_MSG_ID_2128_LEN 2
#define TTALINK_MSG_ID_2128_MIN_LEN 2

#define TTALINK_MSG_ID_CALIBRATE_DATA_ACK_CRC 117
#define TTALINK_MSG_ID_2128_CRC 117



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_CALIBRATE_DATA_ACK { \
    2128, \
    "CALIBRATE_DATA_ACK", \
    2, \
    {  { "index", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_calibrate_data_ack_t, index) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_calibrate_data_ack_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_CALIBRATE_DATA_ACK { \
    "CALIBRATE_DATA_ACK", \
    2, \
    {  { "index", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_calibrate_data_ack_t, index) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_calibrate_data_ack_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_calibrate_data_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t index, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN];
    _tta_put_uint8_t(buf, 0, index);
    _tta_put_uint8_t(buf, 1, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN);
#else
    ttalink_calibrate_data_ack_t packet;
    packet.index = index;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_CALIBRATE_DATA_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_MIN_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_CRC, nocrc);
}

/**
 * @brief Pack a calibrate_data_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param index This is send index.
 * @param ack This is ack.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_calibrate_data_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t index, uint8_t ack)
{
    return _ttalink_calibrate_data_ack_pack(dst_addr, src_addr, msg,  index, ack, false);
}

/**
 * @brief Pack a calibrate_data_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param index This is send index.
 * @param ack This is ack.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_calibrate_data_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t index, uint8_t ack)
{
    return _ttalink_calibrate_data_ack_pack(dst_addr, src_addr, msg,  index, ack, true);
}


static inline uint16_t _ttalink_calibrate_data_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t index,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN];
    _tta_put_uint8_t(buf, 0, index);
    _tta_put_uint8_t(buf, 1, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN);
#else
    ttalink_calibrate_data_ack_t packet;
    packet.index = index;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_CALIBRATE_DATA_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_MIN_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_CRC, nocrc);
}

/**
 * @brief Pack a calibrate_data_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param index This is send index.
 * @param ack This is ack.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_calibrate_data_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t index,uint8_t ack)
{
    return _ttalink_calibrate_data_ack_pack_chan(dst_addr, src_addr, chan, msg,  index, ack, false);
}

/**
 * @brief Pack a calibrate_data_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param index This is send index.
 * @param ack This is ack.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_calibrate_data_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t index,uint8_t ack)
{
    return _ttalink_calibrate_data_ack_pack_chan(dst_addr, src_addr, chan, msg,  index, ack, true);
}


static inline uint16_t _ttalink_calibrate_data_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_calibrate_data_ack_t* calibrate_data_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_calibrate_data_ack_pack_nocrc(dst_addr, src_addr, msg, calibrate_data_ack->index, calibrate_data_ack->ack);
    }else{
        return ttalink_calibrate_data_ack_pack(dst_addr, src_addr, msg, calibrate_data_ack->index, calibrate_data_ack->ack);
    }
    
}

/**
 * @brief Encode a calibrate_data_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param calibrate_data_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_calibrate_data_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_calibrate_data_ack_t* calibrate_data_ack)
{
    return _ttalink_calibrate_data_ack_encode(dst_addr, src_addr, msg, calibrate_data_ack, false);
}

/**
 * @brief Encode a calibrate_data_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param calibrate_data_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_calibrate_data_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_calibrate_data_ack_t* calibrate_data_ack)
{
    return _ttalink_calibrate_data_ack_encode(dst_addr, src_addr, msg, calibrate_data_ack, true);
}


static inline uint16_t _ttalink_calibrate_data_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_calibrate_data_ack_t* calibrate_data_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_calibrate_data_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, calibrate_data_ack->index, calibrate_data_ack->ack);
    }else{
        return ttalink_calibrate_data_ack_pack_chan(dst_addr, src_addr, chan, msg, calibrate_data_ack->index, calibrate_data_ack->ack);
    }
}

/**
 * @brief Encode a calibrate_data_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param calibrate_data_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_calibrate_data_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_calibrate_data_ack_t* calibrate_data_ack)
{
    return _ttalink_calibrate_data_ack_encode_chan(dst_addr, src_addr, chan, msg, calibrate_data_ack, false);
}

/**
 * @brief Encode a calibrate_data_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param calibrate_data_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_calibrate_data_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_calibrate_data_ack_t* calibrate_data_ack)
{
    return _ttalink_calibrate_data_ack_encode_chan(dst_addr, src_addr, chan, msg, calibrate_data_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_calibrate_data_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t index, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN];
    _tta_put_uint8_t(buf, 0, index);
    _tta_put_uint8_t(buf, 1, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CALIBRATE_DATA_ACK, buf, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_MIN_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_CRC, nocrc);
#else
    ttalink_calibrate_data_ack_t packet;
    packet.index = index;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CALIBRATE_DATA_ACK, (const char *)&packet, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_MIN_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a calibrate_data_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param index This is send index.
 * @param ack This is ack.
 */
static inline void ttalink_calibrate_data_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t index, uint8_t ack)
{
    _ttalink_calibrate_data_ack_send(dst_addr, src_addr, chan, index, ack, false);
}

/**
 * @brief Send a calibrate_data_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param index This is send index.
 * @param ack This is ack.
 */
static inline void ttalink_calibrate_data_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t index, uint8_t ack)
{
    _ttalink_calibrate_data_ack_send(dst_addr, src_addr, chan, index, ack, true);
}


static inline void _ttalink_calibrate_data_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_calibrate_data_ack_t* calibrate_data_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_calibrate_data_ack_send_nocrc(dst_addr, src_addr, chan, calibrate_data_ack->index, calibrate_data_ack->ack);
    }else{
        ttalink_calibrate_data_ack_send(dst_addr, src_addr, chan, calibrate_data_ack->index, calibrate_data_ack->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CALIBRATE_DATA_ACK, (const char *)calibrate_data_ack, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_MIN_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a calibrate_data_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_calibrate_data_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_calibrate_data_ack_t* calibrate_data_ack)
{
    _ttalink_calibrate_data_ack_send_struct(dst_addr, src_addr, chan, calibrate_data_ack, false);
}

/**
 * @brief Send a calibrate_data_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_calibrate_data_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_calibrate_data_ack_t* calibrate_data_ack)
{
    _ttalink_calibrate_data_ack_send_struct(dst_addr, src_addr, chan, calibrate_data_ack, true);
}

#if TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_calibrate_data_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t index, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, index);
    _tta_put_uint8_t(buf, 1, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CALIBRATE_DATA_ACK, buf, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_MIN_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_CRC, nocrc);
#else
    ttalink_calibrate_data_ack_t *packet = (ttalink_calibrate_data_ack_t *)msgbuf;
    packet->index = index;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CALIBRATE_DATA_ACK, (const char *)packet, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_MIN_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_calibrate_data_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t index, uint8_t ack)
{
    _ttalink_calibrate_data_ack_send_buf(dst_addr, src_addr, msgbuf, chan, index, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_calibrate_data_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t index, uint8_t ack)
{
    _ttalink_calibrate_data_ack_send_buf(dst_addr, src_addr, msgbuf, chan, index, ack, true);
}
#endif

#endif

// MESSAGE CALIBRATE_DATA_ACK UNPACKING


/**
 * @brief Get field index from calibrate_data_ack message
 *
 * @return This is send index.
 */
static inline uint8_t ttalink_calibrate_data_ack_get_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack from calibrate_data_ack message
 *
 * @return This is ack.
 */
static inline uint8_t ttalink_calibrate_data_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a calibrate_data_ack message into a struct
 *
 * @param msg The message to decode
 * @param calibrate_data_ack C-struct to decode the message contents into
 */
static inline void ttalink_calibrate_data_ack_decode(const ttalink_message_t* msg, ttalink_calibrate_data_ack_t* calibrate_data_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    calibrate_data_ack->index = ttalink_calibrate_data_ack_get_index(msg);
    calibrate_data_ack->ack = ttalink_calibrate_data_ack_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN? msg->len : TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN;
        memset(calibrate_data_ack, 0, TTALINK_MSG_ID_CALIBRATE_DATA_ACK_LEN);
    memcpy(calibrate_data_ack, _TTA_PAYLOAD(msg), len);
#endif
}
