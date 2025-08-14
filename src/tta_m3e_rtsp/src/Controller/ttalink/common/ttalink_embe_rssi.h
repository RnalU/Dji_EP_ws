#pragma once
// MESSAGE EMBE_RSSI PACKING

#define TTALINK_MSG_ID_EMBE_RSSI 2089

TTAPACKED(
typedef struct __ttalink_embe_rssi_t {
 uint8_t rssi; /*<  0-5.*/
}) ttalink_embe_rssi_t;

#define TTALINK_MSG_ID_EMBE_RSSI_LEN 1
#define TTALINK_MSG_ID_EMBE_RSSI_MIN_LEN 1
#define TTALINK_MSG_ID_2089_LEN 1
#define TTALINK_MSG_ID_2089_MIN_LEN 1

#define TTALINK_MSG_ID_EMBE_RSSI_CRC 10
#define TTALINK_MSG_ID_2089_CRC 10



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_EMBE_RSSI { \
    2089, \
    "EMBE_RSSI", \
    1, \
    {  { "rssi", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_embe_rssi_t, rssi) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_EMBE_RSSI { \
    "EMBE_RSSI", \
    1, \
    {  { "rssi", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_embe_rssi_t, rssi) }, \
         } \
}
#endif


static inline uint16_t _ttalink_embe_rssi_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t rssi, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_EMBE_RSSI_LEN];
    _tta_put_uint8_t(buf, 0, rssi);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_EMBE_RSSI_LEN);
#else
    ttalink_embe_rssi_t packet;
    packet.rssi = rssi;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_EMBE_RSSI_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_EMBE_RSSI;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_EMBE_RSSI_MIN_LEN, TTALINK_MSG_ID_EMBE_RSSI_LEN, TTALINK_MSG_ID_EMBE_RSSI_CRC, nocrc);
}

/**
 * @brief Pack a embe_rssi message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param rssi  0-5.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_embe_rssi_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t rssi)
{
    return _ttalink_embe_rssi_pack(dst_addr, src_addr, msg,  rssi, false);
}

/**
 * @brief Pack a embe_rssi message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param rssi  0-5.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_embe_rssi_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t rssi)
{
    return _ttalink_embe_rssi_pack(dst_addr, src_addr, msg,  rssi, true);
}


static inline uint16_t _ttalink_embe_rssi_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t rssi, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_EMBE_RSSI_LEN];
    _tta_put_uint8_t(buf, 0, rssi);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_EMBE_RSSI_LEN);
#else
    ttalink_embe_rssi_t packet;
    packet.rssi = rssi;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_EMBE_RSSI_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_EMBE_RSSI;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_EMBE_RSSI_MIN_LEN, TTALINK_MSG_ID_EMBE_RSSI_LEN, TTALINK_MSG_ID_EMBE_RSSI_CRC, nocrc);
}

/**
 * @brief Pack a embe_rssi message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rssi  0-5.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_embe_rssi_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t rssi)
{
    return _ttalink_embe_rssi_pack_chan(dst_addr, src_addr, chan, msg,  rssi, false);
}

/**
 * @brief Pack a embe_rssi message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rssi  0-5.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_embe_rssi_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t rssi)
{
    return _ttalink_embe_rssi_pack_chan(dst_addr, src_addr, chan, msg,  rssi, true);
}


static inline uint16_t _ttalink_embe_rssi_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_embe_rssi_t* embe_rssi, bool nocrc)
{
    if(nocrc){
        return ttalink_embe_rssi_pack_nocrc(dst_addr, src_addr, msg, embe_rssi->rssi);
    }else{
        return ttalink_embe_rssi_pack(dst_addr, src_addr, msg, embe_rssi->rssi);
    }
    
}

/**
 * @brief Encode a embe_rssi struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param embe_rssi C-struct to read the message contents from
 */
static inline uint16_t ttalink_embe_rssi_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_embe_rssi_t* embe_rssi)
{
    return _ttalink_embe_rssi_encode(dst_addr, src_addr, msg, embe_rssi, false);
}

/**
 * @brief Encode a embe_rssi struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param embe_rssi C-struct to read the message contents from
 */
static inline uint16_t ttalink_embe_rssi_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_embe_rssi_t* embe_rssi)
{
    return _ttalink_embe_rssi_encode(dst_addr, src_addr, msg, embe_rssi, true);
}


static inline uint16_t _ttalink_embe_rssi_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_embe_rssi_t* embe_rssi, bool nocrc)
{
    if(nocrc){
        return ttalink_embe_rssi_pack_chan_nocrc(dst_addr, src_addr, chan, msg, embe_rssi->rssi);
    }else{
        return ttalink_embe_rssi_pack_chan(dst_addr, src_addr, chan, msg, embe_rssi->rssi);
    }
}

/**
 * @brief Encode a embe_rssi struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param embe_rssi C-struct to read the message contents from
 */
static inline uint16_t ttalink_embe_rssi_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_embe_rssi_t* embe_rssi)
{
    return _ttalink_embe_rssi_encode_chan(dst_addr, src_addr, chan, msg, embe_rssi, false);
}

/**
 * @brief Encode a embe_rssi struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param embe_rssi C-struct to read the message contents from
 */
static inline uint16_t ttalink_embe_rssi_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_embe_rssi_t* embe_rssi)
{
    return _ttalink_embe_rssi_encode_chan(dst_addr, src_addr, chan, msg, embe_rssi, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_embe_rssi_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t rssi, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_EMBE_RSSI_LEN];
    _tta_put_uint8_t(buf, 0, rssi);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EMBE_RSSI, buf, TTALINK_MSG_ID_EMBE_RSSI_MIN_LEN, TTALINK_MSG_ID_EMBE_RSSI_LEN, TTALINK_MSG_ID_EMBE_RSSI_CRC, nocrc);
#else
    ttalink_embe_rssi_t packet;
    packet.rssi = rssi;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EMBE_RSSI, (const char *)&packet, TTALINK_MSG_ID_EMBE_RSSI_MIN_LEN, TTALINK_MSG_ID_EMBE_RSSI_LEN, TTALINK_MSG_ID_EMBE_RSSI_CRC, nocrc);
#endif
}

/**
 * @brief Send a embe_rssi message
 * @param chan TTAlink channel to send the message
 *
 * @param rssi  0-5.
 */
static inline void ttalink_embe_rssi_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t rssi)
{
    _ttalink_embe_rssi_send(dst_addr, src_addr, chan, rssi, false);
}

/**
 * @brief Send a embe_rssi message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param rssi  0-5.
 */
static inline void ttalink_embe_rssi_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t rssi)
{
    _ttalink_embe_rssi_send(dst_addr, src_addr, chan, rssi, true);
}


static inline void _ttalink_embe_rssi_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_embe_rssi_t* embe_rssi, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_embe_rssi_send_nocrc(dst_addr, src_addr, chan, embe_rssi->rssi);
    }else{
        ttalink_embe_rssi_send(dst_addr, src_addr, chan, embe_rssi->rssi);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EMBE_RSSI, (const char *)embe_rssi, TTALINK_MSG_ID_EMBE_RSSI_MIN_LEN, TTALINK_MSG_ID_EMBE_RSSI_LEN, TTALINK_MSG_ID_EMBE_RSSI_CRC, nocrc);
#endif
}

/**
 * @brief Send a embe_rssi message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_embe_rssi_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_embe_rssi_t* embe_rssi)
{
    _ttalink_embe_rssi_send_struct(dst_addr, src_addr, chan, embe_rssi, false);
}

/**
 * @brief Send a embe_rssi message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_embe_rssi_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_embe_rssi_t* embe_rssi)
{
    _ttalink_embe_rssi_send_struct(dst_addr, src_addr, chan, embe_rssi, true);
}

#if TTALINK_MSG_ID_EMBE_RSSI_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_embe_rssi_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t rssi, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, rssi);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EMBE_RSSI, buf, TTALINK_MSG_ID_EMBE_RSSI_MIN_LEN, TTALINK_MSG_ID_EMBE_RSSI_LEN, TTALINK_MSG_ID_EMBE_RSSI_CRC, nocrc);
#else
    ttalink_embe_rssi_t *packet = (ttalink_embe_rssi_t *)msgbuf;
    packet->rssi = rssi;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EMBE_RSSI, (const char *)packet, TTALINK_MSG_ID_EMBE_RSSI_MIN_LEN, TTALINK_MSG_ID_EMBE_RSSI_LEN, TTALINK_MSG_ID_EMBE_RSSI_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_embe_rssi_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t rssi)
{
    _ttalink_embe_rssi_send_buf(dst_addr, src_addr, msgbuf, chan, rssi, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_embe_rssi_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t rssi)
{
    _ttalink_embe_rssi_send_buf(dst_addr, src_addr, msgbuf, chan, rssi, true);
}
#endif

#endif

// MESSAGE EMBE_RSSI UNPACKING


/**
 * @brief Get field rssi from embe_rssi message
 *
 * @return  0-5.
 */
static inline uint8_t ttalink_embe_rssi_get_rssi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a embe_rssi message into a struct
 *
 * @param msg The message to decode
 * @param embe_rssi C-struct to decode the message contents into
 */
static inline void ttalink_embe_rssi_decode(const ttalink_message_t* msg, ttalink_embe_rssi_t* embe_rssi)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    embe_rssi->rssi = ttalink_embe_rssi_get_rssi(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_EMBE_RSSI_LEN? msg->len : TTALINK_MSG_ID_EMBE_RSSI_LEN;
        memset(embe_rssi, 0, TTALINK_MSG_ID_EMBE_RSSI_LEN);
    memcpy(embe_rssi, _TTA_PAYLOAD(msg), len);
#endif
}
