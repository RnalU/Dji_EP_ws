#pragma once
// MESSAGE FILES_OPS PACKING

#define TTALINK_MSG_ID_FILES_OPS 2251

TTAPACKED(
typedef struct __ttalink_files_ops_t {
 int8_t payload[512]; /*< This is file payload.*/
}) ttalink_files_ops_t;

#define TTALINK_MSG_ID_FILES_OPS_LEN 512
#define TTALINK_MSG_ID_FILES_OPS_MIN_LEN 512
#define TTALINK_MSG_ID_2251_LEN 512
#define TTALINK_MSG_ID_2251_MIN_LEN 512

#define TTALINK_MSG_ID_FILES_OPS_CRC 88
#define TTALINK_MSG_ID_2251_CRC 88

#define TTALINK_MSG_FILES_OPS_FIELD_PAYLOAD_LEN 512

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_FILES_OPS { \
    2251, \
    "FILES_OPS", \
    1, \
    {  { "payload", NULL, TTALINK_TYPE_INT8_T, 512, 0, offsetof(ttalink_files_ops_t, payload) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_FILES_OPS { \
    "FILES_OPS", \
    1, \
    {  { "payload", NULL, TTALINK_TYPE_INT8_T, 512, 0, offsetof(ttalink_files_ops_t, payload) }, \
         } \
}
#endif


static inline uint16_t _ttalink_files_ops_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FILES_OPS_LEN];

    _tta_put_int8_t_array(buf, 0, payload, 512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FILES_OPS_LEN);
#else
    ttalink_files_ops_t packet;

    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FILES_OPS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FILES_OPS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_FILES_OPS_MIN_LEN, TTALINK_MSG_ID_FILES_OPS_LEN, TTALINK_MSG_ID_FILES_OPS_CRC, nocrc);
}

/**
 * @brief Pack a files_ops message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param payload This is file payload.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_files_ops_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const int8_t *payload)
{
    return _ttalink_files_ops_pack(dst_addr, src_addr, msg,  payload, false);
}

/**
 * @brief Pack a files_ops message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param payload This is file payload.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_files_ops_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const int8_t *payload)
{
    return _ttalink_files_ops_pack(dst_addr, src_addr, msg,  payload, true);
}


static inline uint16_t _ttalink_files_ops_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FILES_OPS_LEN];

    _tta_put_int8_t_array(buf, 0, payload, 512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FILES_OPS_LEN);
#else
    ttalink_files_ops_t packet;

    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FILES_OPS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FILES_OPS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_FILES_OPS_MIN_LEN, TTALINK_MSG_ID_FILES_OPS_LEN, TTALINK_MSG_ID_FILES_OPS_CRC, nocrc);
}

/**
 * @brief Pack a files_ops message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param payload This is file payload.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_files_ops_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const int8_t *payload)
{
    return _ttalink_files_ops_pack_chan(dst_addr, src_addr, chan, msg,  payload, false);
}

/**
 * @brief Pack a files_ops message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param payload This is file payload.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_files_ops_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const int8_t *payload)
{
    return _ttalink_files_ops_pack_chan(dst_addr, src_addr, chan, msg,  payload, true);
}


static inline uint16_t _ttalink_files_ops_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_files_ops_t* files_ops, bool nocrc)
{
    if(nocrc){
        return ttalink_files_ops_pack_nocrc(dst_addr, src_addr, msg, files_ops->payload);
    }else{
        return ttalink_files_ops_pack(dst_addr, src_addr, msg, files_ops->payload);
    }
    
}

/**
 * @brief Encode a files_ops struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param files_ops C-struct to read the message contents from
 */
static inline uint16_t ttalink_files_ops_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_files_ops_t* files_ops)
{
    return _ttalink_files_ops_encode(dst_addr, src_addr, msg, files_ops, false);
}

/**
 * @brief Encode a files_ops struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param files_ops C-struct to read the message contents from
 */
static inline uint16_t ttalink_files_ops_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_files_ops_t* files_ops)
{
    return _ttalink_files_ops_encode(dst_addr, src_addr, msg, files_ops, true);
}


static inline uint16_t _ttalink_files_ops_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_files_ops_t* files_ops, bool nocrc)
{
    if(nocrc){
        return ttalink_files_ops_pack_chan_nocrc(dst_addr, src_addr, chan, msg, files_ops->payload);
    }else{
        return ttalink_files_ops_pack_chan(dst_addr, src_addr, chan, msg, files_ops->payload);
    }
}

/**
 * @brief Encode a files_ops struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param files_ops C-struct to read the message contents from
 */
static inline uint16_t ttalink_files_ops_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_files_ops_t* files_ops)
{
    return _ttalink_files_ops_encode_chan(dst_addr, src_addr, chan, msg, files_ops, false);
}

/**
 * @brief Encode a files_ops struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param files_ops C-struct to read the message contents from
 */
static inline uint16_t ttalink_files_ops_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_files_ops_t* files_ops)
{
    return _ttalink_files_ops_encode_chan(dst_addr, src_addr, chan, msg, files_ops, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_files_ops_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FILES_OPS_LEN];

    _tta_put_int8_t_array(buf, 0, payload, 512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FILES_OPS, buf, TTALINK_MSG_ID_FILES_OPS_MIN_LEN, TTALINK_MSG_ID_FILES_OPS_LEN, TTALINK_MSG_ID_FILES_OPS_CRC, nocrc);
#else
    ttalink_files_ops_t packet;

    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FILES_OPS, (const char *)&packet, TTALINK_MSG_ID_FILES_OPS_MIN_LEN, TTALINK_MSG_ID_FILES_OPS_LEN, TTALINK_MSG_ID_FILES_OPS_CRC, nocrc);
#endif
}

/**
 * @brief Send a files_ops message
 * @param chan TTAlink channel to send the message
 *
 * @param payload This is file payload.
 */
static inline void ttalink_files_ops_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const int8_t *payload)
{
    _ttalink_files_ops_send(dst_addr, src_addr, chan, payload, false);
}

/**
 * @brief Send a files_ops message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param payload This is file payload.
 */
static inline void ttalink_files_ops_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const int8_t *payload)
{
    _ttalink_files_ops_send(dst_addr, src_addr, chan, payload, true);
}


static inline void _ttalink_files_ops_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_files_ops_t* files_ops, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_files_ops_send_nocrc(dst_addr, src_addr, chan, files_ops->payload);
    }else{
        ttalink_files_ops_send(dst_addr, src_addr, chan, files_ops->payload);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FILES_OPS, (const char *)files_ops, TTALINK_MSG_ID_FILES_OPS_MIN_LEN, TTALINK_MSG_ID_FILES_OPS_LEN, TTALINK_MSG_ID_FILES_OPS_CRC, nocrc);
#endif
}

/**
 * @brief Send a files_ops message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_files_ops_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_files_ops_t* files_ops)
{
    _ttalink_files_ops_send_struct(dst_addr, src_addr, chan, files_ops, false);
}

/**
 * @brief Send a files_ops message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_files_ops_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_files_ops_t* files_ops)
{
    _ttalink_files_ops_send_struct(dst_addr, src_addr, chan, files_ops, true);
}

#if TTALINK_MSG_ID_FILES_OPS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_files_ops_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _tta_put_int8_t_array(buf, 0, payload, 512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FILES_OPS, buf, TTALINK_MSG_ID_FILES_OPS_MIN_LEN, TTALINK_MSG_ID_FILES_OPS_LEN, TTALINK_MSG_ID_FILES_OPS_CRC, nocrc);
#else
    ttalink_files_ops_t *packet = (ttalink_files_ops_t *)msgbuf;

    tta_array_memcpy(packet->payload, payload, sizeof(int8_t)*512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FILES_OPS, (const char *)packet, TTALINK_MSG_ID_FILES_OPS_MIN_LEN, TTALINK_MSG_ID_FILES_OPS_LEN, TTALINK_MSG_ID_FILES_OPS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_files_ops_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const int8_t *payload)
{
    _ttalink_files_ops_send_buf(dst_addr, src_addr, msgbuf, chan, payload, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_files_ops_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const int8_t *payload)
{
    _ttalink_files_ops_send_buf(dst_addr, src_addr, msgbuf, chan, payload, true);
}
#endif

#endif

// MESSAGE FILES_OPS UNPACKING


/**
 * @brief Get field payload from files_ops message
 *
 * @return This is file payload.
 */
static inline uint16_t ttalink_files_ops_get_payload(const ttalink_message_t* msg, int8_t *payload)
{
    return _TTA_RETURN_int8_t_array(msg, payload, 512,  0);
}

/**
 * @brief Decode a files_ops message into a struct
 *
 * @param msg The message to decode
 * @param files_ops C-struct to decode the message contents into
 */
static inline void ttalink_files_ops_decode(const ttalink_message_t* msg, ttalink_files_ops_t* files_ops)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_files_ops_get_payload(msg, files_ops->payload);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_FILES_OPS_LEN? msg->len : TTALINK_MSG_ID_FILES_OPS_LEN;
        memset(files_ops, 0, TTALINK_MSG_ID_FILES_OPS_LEN);
    memcpy(files_ops, _TTA_PAYLOAD(msg), len);
#endif
}
