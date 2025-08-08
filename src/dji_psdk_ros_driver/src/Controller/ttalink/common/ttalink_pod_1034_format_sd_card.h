#pragma once
// MESSAGE POD_1034_FORMAT_SD_CARD PACKING

#define TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD 11034

TTAPACKED(
typedef struct __ttalink_pod_1034_format_sd_card_t {
 uint8_t ack; /*<  .*/
 uint8_t ack_reserve; /*< Reserve.*/
}) ttalink_pod_1034_format_sd_card_t;

#define TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN 2
#define TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_MIN_LEN 2
#define TTALINK_MSG_ID_11034_LEN 2
#define TTALINK_MSG_ID_11034_MIN_LEN 2

#define TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_CRC 202
#define TTALINK_MSG_ID_11034_CRC 202



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1034_FORMAT_SD_CARD { \
    11034, \
    "POD_1034_FORMAT_SD_CARD", \
    2, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1034_format_sd_card_t, ack) }, \
         { "ack_reserve", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1034_format_sd_card_t, ack_reserve) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1034_FORMAT_SD_CARD { \
    "POD_1034_FORMAT_SD_CARD", \
    2, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1034_format_sd_card_t, ack) }, \
         { "ack_reserve", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1034_format_sd_card_t, ack_reserve) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1034_format_sd_card_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_reserve);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN);
#else
    ttalink_pod_1034_format_sd_card_t packet;
    packet.ack = ack;
    packet.ack_reserve = ack_reserve;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_MIN_LEN, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_CRC, nocrc);
}

/**
 * @brief Pack a pod_1034_format_sd_card message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack  .
 * @param ack_reserve Reserve.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1034_format_sd_card_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_reserve)
{
    return _ttalink_pod_1034_format_sd_card_pack(dst_addr, src_addr, msg,  ack, ack_reserve, false);
}

/**
 * @brief Pack a pod_1034_format_sd_card message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack  .
 * @param ack_reserve Reserve.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1034_format_sd_card_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_reserve)
{
    return _ttalink_pod_1034_format_sd_card_pack(dst_addr, src_addr, msg,  ack, ack_reserve, true);
}


static inline uint16_t _ttalink_pod_1034_format_sd_card_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_reserve);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN);
#else
    ttalink_pod_1034_format_sd_card_t packet;
    packet.ack = ack;
    packet.ack_reserve = ack_reserve;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_MIN_LEN, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_CRC, nocrc);
}

/**
 * @brief Pack a pod_1034_format_sd_card message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack  .
 * @param ack_reserve Reserve.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1034_format_sd_card_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_reserve)
{
    return _ttalink_pod_1034_format_sd_card_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_reserve, false);
}

/**
 * @brief Pack a pod_1034_format_sd_card message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack  .
 * @param ack_reserve Reserve.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1034_format_sd_card_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_reserve)
{
    return _ttalink_pod_1034_format_sd_card_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_reserve, true);
}


static inline uint16_t _ttalink_pod_1034_format_sd_card_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1034_format_sd_card_t* pod_1034_format_sd_card, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1034_format_sd_card_pack_nocrc(dst_addr, src_addr, msg, pod_1034_format_sd_card->ack, pod_1034_format_sd_card->ack_reserve);
    }else{
        return ttalink_pod_1034_format_sd_card_pack(dst_addr, src_addr, msg, pod_1034_format_sd_card->ack, pod_1034_format_sd_card->ack_reserve);
    }
    
}

/**
 * @brief Encode a pod_1034_format_sd_card struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1034_format_sd_card C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1034_format_sd_card_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1034_format_sd_card_t* pod_1034_format_sd_card)
{
    return _ttalink_pod_1034_format_sd_card_encode(dst_addr, src_addr, msg, pod_1034_format_sd_card, false);
}

/**
 * @brief Encode a pod_1034_format_sd_card struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1034_format_sd_card C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1034_format_sd_card_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1034_format_sd_card_t* pod_1034_format_sd_card)
{
    return _ttalink_pod_1034_format_sd_card_encode(dst_addr, src_addr, msg, pod_1034_format_sd_card, true);
}


static inline uint16_t _ttalink_pod_1034_format_sd_card_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1034_format_sd_card_t* pod_1034_format_sd_card, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1034_format_sd_card_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1034_format_sd_card->ack, pod_1034_format_sd_card->ack_reserve);
    }else{
        return ttalink_pod_1034_format_sd_card_pack_chan(dst_addr, src_addr, chan, msg, pod_1034_format_sd_card->ack, pod_1034_format_sd_card->ack_reserve);
    }
}

/**
 * @brief Encode a pod_1034_format_sd_card struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1034_format_sd_card C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1034_format_sd_card_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1034_format_sd_card_t* pod_1034_format_sd_card)
{
    return _ttalink_pod_1034_format_sd_card_encode_chan(dst_addr, src_addr, chan, msg, pod_1034_format_sd_card, false);
}

/**
 * @brief Encode a pod_1034_format_sd_card struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1034_format_sd_card C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1034_format_sd_card_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1034_format_sd_card_t* pod_1034_format_sd_card)
{
    return _ttalink_pod_1034_format_sd_card_encode_chan(dst_addr, src_addr, chan, msg, pod_1034_format_sd_card, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1034_format_sd_card_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_reserve);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD, buf, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_MIN_LEN, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_CRC, nocrc);
#else
    ttalink_pod_1034_format_sd_card_t packet;
    packet.ack = ack;
    packet.ack_reserve = ack_reserve;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD, (const char *)&packet, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_MIN_LEN, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1034_format_sd_card message
 * @param chan TTAlink channel to send the message
 *
 * @param ack  .
 * @param ack_reserve Reserve.
 */
static inline void ttalink_pod_1034_format_sd_card_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_reserve)
{
    _ttalink_pod_1034_format_sd_card_send(dst_addr, src_addr, chan, ack, ack_reserve, false);
}

/**
 * @brief Send a pod_1034_format_sd_card message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack  .
 * @param ack_reserve Reserve.
 */
static inline void ttalink_pod_1034_format_sd_card_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_reserve)
{
    _ttalink_pod_1034_format_sd_card_send(dst_addr, src_addr, chan, ack, ack_reserve, true);
}


static inline void _ttalink_pod_1034_format_sd_card_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1034_format_sd_card_t* pod_1034_format_sd_card, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1034_format_sd_card_send_nocrc(dst_addr, src_addr, chan, pod_1034_format_sd_card->ack, pod_1034_format_sd_card->ack_reserve);
    }else{
        ttalink_pod_1034_format_sd_card_send(dst_addr, src_addr, chan, pod_1034_format_sd_card->ack, pod_1034_format_sd_card->ack_reserve);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD, (const char *)pod_1034_format_sd_card, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_MIN_LEN, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1034_format_sd_card message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1034_format_sd_card_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1034_format_sd_card_t* pod_1034_format_sd_card)
{
    _ttalink_pod_1034_format_sd_card_send_struct(dst_addr, src_addr, chan, pod_1034_format_sd_card, false);
}

/**
 * @brief Send a pod_1034_format_sd_card message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1034_format_sd_card_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1034_format_sd_card_t* pod_1034_format_sd_card)
{
    _ttalink_pod_1034_format_sd_card_send_struct(dst_addr, src_addr, chan, pod_1034_format_sd_card, true);
}

#if TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1034_format_sd_card_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_reserve);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD, buf, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_MIN_LEN, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_CRC, nocrc);
#else
    ttalink_pod_1034_format_sd_card_t *packet = (ttalink_pod_1034_format_sd_card_t *)msgbuf;
    packet->ack = ack;
    packet->ack_reserve = ack_reserve;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD, (const char *)packet, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_MIN_LEN, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1034_format_sd_card_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_reserve)
{
    _ttalink_pod_1034_format_sd_card_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_reserve, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1034_format_sd_card_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_reserve)
{
    _ttalink_pod_1034_format_sd_card_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_reserve, true);
}
#endif

#endif

// MESSAGE POD_1034_FORMAT_SD_CARD UNPACKING


/**
 * @brief Get field ack from pod_1034_format_sd_card message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_1034_format_sd_card_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack_reserve from pod_1034_format_sd_card message
 *
 * @return Reserve.
 */
static inline uint8_t ttalink_pod_1034_format_sd_card_get_ack_reserve(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a pod_1034_format_sd_card message into a struct
 *
 * @param msg The message to decode
 * @param pod_1034_format_sd_card C-struct to decode the message contents into
 */
static inline void ttalink_pod_1034_format_sd_card_decode(const ttalink_message_t* msg, ttalink_pod_1034_format_sd_card_t* pod_1034_format_sd_card)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1034_format_sd_card->ack = ttalink_pod_1034_format_sd_card_get_ack(msg);
    pod_1034_format_sd_card->ack_reserve = ttalink_pod_1034_format_sd_card_get_ack_reserve(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN? msg->len : TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN;
        memset(pod_1034_format_sd_card, 0, TTALINK_MSG_ID_POD_1034_FORMAT_SD_CARD_LEN);
    memcpy(pod_1034_format_sd_card, _TTA_PAYLOAD(msg), len);
#endif
}
