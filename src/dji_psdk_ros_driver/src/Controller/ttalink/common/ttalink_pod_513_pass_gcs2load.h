#pragma once
// MESSAGE POD_513_PASS_GCS2LOAD PACKING

#define TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD 10513

TTAPACKED(
typedef struct __ttalink_pod_513_pass_gcs2load_t {
 uint8_t cmd[32]; /*<  Transmit data.*/
}) ttalink_pod_513_pass_gcs2load_t;

#define TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN 32
#define TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_MIN_LEN 32
#define TTALINK_MSG_ID_10513_LEN 32
#define TTALINK_MSG_ID_10513_MIN_LEN 32

#define TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_CRC 131
#define TTALINK_MSG_ID_10513_CRC 131

#define TTALINK_MSG_POD_513_PASS_GCS2LOAD_FIELD_CMD_LEN 32

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_513_PASS_GCS2LOAD { \
    10513, \
    "POD_513_PASS_GCS2LOAD", \
    1, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 32, 0, offsetof(ttalink_pod_513_pass_gcs2load_t, cmd) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_513_PASS_GCS2LOAD { \
    "POD_513_PASS_GCS2LOAD", \
    1, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 32, 0, offsetof(ttalink_pod_513_pass_gcs2load_t, cmd) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_513_pass_gcs2load_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint8_t *cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN];

    _tta_put_uint8_t_array(buf, 0, cmd, 32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN);
#else
    ttalink_pod_513_pass_gcs2load_t packet;

    tta_array_memcpy(packet.cmd, cmd, sizeof(uint8_t)*32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_MIN_LEN, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_CRC, nocrc);
}

/**
 * @brief Pack a pod_513_pass_gcs2load message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  Transmit data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_513_pass_gcs2load_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint8_t *cmd)
{
    return _ttalink_pod_513_pass_gcs2load_pack(dst_addr, src_addr, msg,  cmd, false);
}

/**
 * @brief Pack a pod_513_pass_gcs2load message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  Transmit data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_513_pass_gcs2load_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint8_t *cmd)
{
    return _ttalink_pod_513_pass_gcs2load_pack(dst_addr, src_addr, msg,  cmd, true);
}


static inline uint16_t _ttalink_pod_513_pass_gcs2load_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint8_t *cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN];

    _tta_put_uint8_t_array(buf, 0, cmd, 32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN);
#else
    ttalink_pod_513_pass_gcs2load_t packet;

    tta_array_memcpy(packet.cmd, cmd, sizeof(uint8_t)*32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_MIN_LEN, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_CRC, nocrc);
}

/**
 * @brief Pack a pod_513_pass_gcs2load message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  Transmit data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_513_pass_gcs2load_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint8_t *cmd)
{
    return _ttalink_pod_513_pass_gcs2load_pack_chan(dst_addr, src_addr, chan, msg,  cmd, false);
}

/**
 * @brief Pack a pod_513_pass_gcs2load message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  Transmit data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_513_pass_gcs2load_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint8_t *cmd)
{
    return _ttalink_pod_513_pass_gcs2load_pack_chan(dst_addr, src_addr, chan, msg,  cmd, true);
}


static inline uint16_t _ttalink_pod_513_pass_gcs2load_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_513_pass_gcs2load_t* pod_513_pass_gcs2load, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_513_pass_gcs2load_pack_nocrc(dst_addr, src_addr, msg, pod_513_pass_gcs2load->cmd);
    }else{
        return ttalink_pod_513_pass_gcs2load_pack(dst_addr, src_addr, msg, pod_513_pass_gcs2load->cmd);
    }
    
}

/**
 * @brief Encode a pod_513_pass_gcs2load struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_513_pass_gcs2load C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_513_pass_gcs2load_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_513_pass_gcs2load_t* pod_513_pass_gcs2load)
{
    return _ttalink_pod_513_pass_gcs2load_encode(dst_addr, src_addr, msg, pod_513_pass_gcs2load, false);
}

/**
 * @brief Encode a pod_513_pass_gcs2load struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_513_pass_gcs2load C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_513_pass_gcs2load_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_513_pass_gcs2load_t* pod_513_pass_gcs2load)
{
    return _ttalink_pod_513_pass_gcs2load_encode(dst_addr, src_addr, msg, pod_513_pass_gcs2load, true);
}


static inline uint16_t _ttalink_pod_513_pass_gcs2load_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_513_pass_gcs2load_t* pod_513_pass_gcs2load, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_513_pass_gcs2load_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_513_pass_gcs2load->cmd);
    }else{
        return ttalink_pod_513_pass_gcs2load_pack_chan(dst_addr, src_addr, chan, msg, pod_513_pass_gcs2load->cmd);
    }
}

/**
 * @brief Encode a pod_513_pass_gcs2load struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_513_pass_gcs2load C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_513_pass_gcs2load_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_513_pass_gcs2load_t* pod_513_pass_gcs2load)
{
    return _ttalink_pod_513_pass_gcs2load_encode_chan(dst_addr, src_addr, chan, msg, pod_513_pass_gcs2load, false);
}

/**
 * @brief Encode a pod_513_pass_gcs2load struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_513_pass_gcs2load C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_513_pass_gcs2load_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_513_pass_gcs2load_t* pod_513_pass_gcs2load)
{
    return _ttalink_pod_513_pass_gcs2load_encode_chan(dst_addr, src_addr, chan, msg, pod_513_pass_gcs2load, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_513_pass_gcs2load_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint8_t *cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN];

    _tta_put_uint8_t_array(buf, 0, cmd, 32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD, buf, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_MIN_LEN, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_CRC, nocrc);
#else
    ttalink_pod_513_pass_gcs2load_t packet;

    tta_array_memcpy(packet.cmd, cmd, sizeof(uint8_t)*32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD, (const char *)&packet, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_MIN_LEN, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_513_pass_gcs2load message
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  Transmit data.
 */
static inline void ttalink_pod_513_pass_gcs2load_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint8_t *cmd)
{
    _ttalink_pod_513_pass_gcs2load_send(dst_addr, src_addr, chan, cmd, false);
}

/**
 * @brief Send a pod_513_pass_gcs2load message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  Transmit data.
 */
static inline void ttalink_pod_513_pass_gcs2load_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint8_t *cmd)
{
    _ttalink_pod_513_pass_gcs2load_send(dst_addr, src_addr, chan, cmd, true);
}


static inline void _ttalink_pod_513_pass_gcs2load_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_513_pass_gcs2load_t* pod_513_pass_gcs2load, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_513_pass_gcs2load_send_nocrc(dst_addr, src_addr, chan, pod_513_pass_gcs2load->cmd);
    }else{
        ttalink_pod_513_pass_gcs2load_send(dst_addr, src_addr, chan, pod_513_pass_gcs2load->cmd);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD, (const char *)pod_513_pass_gcs2load, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_MIN_LEN, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_513_pass_gcs2load message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_513_pass_gcs2load_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_513_pass_gcs2load_t* pod_513_pass_gcs2load)
{
    _ttalink_pod_513_pass_gcs2load_send_struct(dst_addr, src_addr, chan, pod_513_pass_gcs2load, false);
}

/**
 * @brief Send a pod_513_pass_gcs2load message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_513_pass_gcs2load_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_513_pass_gcs2load_t* pod_513_pass_gcs2load)
{
    _ttalink_pod_513_pass_gcs2load_send_struct(dst_addr, src_addr, chan, pod_513_pass_gcs2load, true);
}

#if TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_513_pass_gcs2load_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint8_t *cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _tta_put_uint8_t_array(buf, 0, cmd, 32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD, buf, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_MIN_LEN, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_CRC, nocrc);
#else
    ttalink_pod_513_pass_gcs2load_t *packet = (ttalink_pod_513_pass_gcs2load_t *)msgbuf;

    tta_array_memcpy(packet->cmd, cmd, sizeof(uint8_t)*32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD, (const char *)packet, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_MIN_LEN, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_513_pass_gcs2load_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint8_t *cmd)
{
    _ttalink_pod_513_pass_gcs2load_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_513_pass_gcs2load_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint8_t *cmd)
{
    _ttalink_pod_513_pass_gcs2load_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, true);
}
#endif

#endif

// MESSAGE POD_513_PASS_GCS2LOAD UNPACKING


/**
 * @brief Get field cmd from pod_513_pass_gcs2load message
 *
 * @return  Transmit data.
 */
static inline uint16_t ttalink_pod_513_pass_gcs2load_get_cmd(const ttalink_message_t* msg, uint8_t *cmd)
{
    return _TTA_RETURN_uint8_t_array(msg, cmd, 32,  0);
}

/**
 * @brief Decode a pod_513_pass_gcs2load message into a struct
 *
 * @param msg The message to decode
 * @param pod_513_pass_gcs2load C-struct to decode the message contents into
 */
static inline void ttalink_pod_513_pass_gcs2load_decode(const ttalink_message_t* msg, ttalink_pod_513_pass_gcs2load_t* pod_513_pass_gcs2load)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_pod_513_pass_gcs2load_get_cmd(msg, pod_513_pass_gcs2load->cmd);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN? msg->len : TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN;
        memset(pod_513_pass_gcs2load, 0, TTALINK_MSG_ID_POD_513_PASS_GCS2LOAD_LEN);
    memcpy(pod_513_pass_gcs2load, _TTA_PAYLOAD(msg), len);
#endif
}
