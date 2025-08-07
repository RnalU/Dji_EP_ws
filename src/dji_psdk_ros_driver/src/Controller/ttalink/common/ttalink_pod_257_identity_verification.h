#pragma once
// MESSAGE POD_257_IDENTITY_VERIFICATION PACKING

#define TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION 10257

TTAPACKED(
typedef struct __ttalink_pod_257_identity_verification_t {
 uint8_t cmd[16]; /*<  Random number string.*/
 uint8_t ack; /*<  .*/
 uint8_t ack_md5[16]; /*<  MD5.*/
}) ttalink_pod_257_identity_verification_t;

#define TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN 33
#define TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_MIN_LEN 33
#define TTALINK_MSG_ID_10257_LEN 33
#define TTALINK_MSG_ID_10257_MIN_LEN 33

#define TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_CRC 81
#define TTALINK_MSG_ID_10257_CRC 81

#define TTALINK_MSG_POD_257_IDENTITY_VERIFICATION_FIELD_CMD_LEN 16
#define TTALINK_MSG_POD_257_IDENTITY_VERIFICATION_FIELD_ACK_MD5_LEN 16

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_257_IDENTITY_VERIFICATION { \
    10257, \
    "POD_257_IDENTITY_VERIFICATION", \
    3, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 16, 0, offsetof(ttalink_pod_257_identity_verification_t, cmd) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 16, offsetof(ttalink_pod_257_identity_verification_t, ack) }, \
         { "ack_md5", NULL, TTALINK_TYPE_UINT8_T, 16, 17, offsetof(ttalink_pod_257_identity_verification_t, ack_md5) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_257_IDENTITY_VERIFICATION { \
    "POD_257_IDENTITY_VERIFICATION", \
    3, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 16, 0, offsetof(ttalink_pod_257_identity_verification_t, cmd) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 16, offsetof(ttalink_pod_257_identity_verification_t, ack) }, \
         { "ack_md5", NULL, TTALINK_TYPE_UINT8_T, 16, 17, offsetof(ttalink_pod_257_identity_verification_t, ack_md5) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_257_identity_verification_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint8_t *cmd, uint8_t ack, const uint8_t *ack_md5, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN];
    _tta_put_uint8_t(buf, 16, ack);
    _tta_put_uint8_t_array(buf, 0, cmd, 16);
    _tta_put_uint8_t_array(buf, 17, ack_md5, 16);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN);
#else
    ttalink_pod_257_identity_verification_t packet;
    packet.ack = ack;
    tta_array_memcpy(packet.cmd, cmd, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.ack_md5, ack_md5, sizeof(uint8_t)*16);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_MIN_LEN, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_CRC, nocrc);
}

/**
 * @brief Pack a pod_257_identity_verification message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  Random number string.
 * @param ack  .
 * @param ack_md5  MD5.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_257_identity_verification_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint8_t *cmd, uint8_t ack, const uint8_t *ack_md5)
{
    return _ttalink_pod_257_identity_verification_pack(dst_addr, src_addr, msg,  cmd, ack, ack_md5, false);
}

/**
 * @brief Pack a pod_257_identity_verification message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  Random number string.
 * @param ack  .
 * @param ack_md5  MD5.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_257_identity_verification_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint8_t *cmd, uint8_t ack, const uint8_t *ack_md5)
{
    return _ttalink_pod_257_identity_verification_pack(dst_addr, src_addr, msg,  cmd, ack, ack_md5, true);
}


static inline uint16_t _ttalink_pod_257_identity_verification_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint8_t *cmd,uint8_t ack,const uint8_t *ack_md5, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN];
    _tta_put_uint8_t(buf, 16, ack);
    _tta_put_uint8_t_array(buf, 0, cmd, 16);
    _tta_put_uint8_t_array(buf, 17, ack_md5, 16);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN);
#else
    ttalink_pod_257_identity_verification_t packet;
    packet.ack = ack;
    tta_array_memcpy(packet.cmd, cmd, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.ack_md5, ack_md5, sizeof(uint8_t)*16);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_MIN_LEN, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_CRC, nocrc);
}

/**
 * @brief Pack a pod_257_identity_verification message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  Random number string.
 * @param ack  .
 * @param ack_md5  MD5.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_257_identity_verification_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint8_t *cmd,uint8_t ack,const uint8_t *ack_md5)
{
    return _ttalink_pod_257_identity_verification_pack_chan(dst_addr, src_addr, chan, msg,  cmd, ack, ack_md5, false);
}

/**
 * @brief Pack a pod_257_identity_verification message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  Random number string.
 * @param ack  .
 * @param ack_md5  MD5.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_257_identity_verification_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint8_t *cmd,uint8_t ack,const uint8_t *ack_md5)
{
    return _ttalink_pod_257_identity_verification_pack_chan(dst_addr, src_addr, chan, msg,  cmd, ack, ack_md5, true);
}


static inline uint16_t _ttalink_pod_257_identity_verification_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_257_identity_verification_t* pod_257_identity_verification, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_257_identity_verification_pack_nocrc(dst_addr, src_addr, msg, pod_257_identity_verification->cmd, pod_257_identity_verification->ack, pod_257_identity_verification->ack_md5);
    }else{
        return ttalink_pod_257_identity_verification_pack(dst_addr, src_addr, msg, pod_257_identity_verification->cmd, pod_257_identity_verification->ack, pod_257_identity_verification->ack_md5);
    }
    
}

/**
 * @brief Encode a pod_257_identity_verification struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_257_identity_verification C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_257_identity_verification_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_257_identity_verification_t* pod_257_identity_verification)
{
    return _ttalink_pod_257_identity_verification_encode(dst_addr, src_addr, msg, pod_257_identity_verification, false);
}

/**
 * @brief Encode a pod_257_identity_verification struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_257_identity_verification C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_257_identity_verification_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_257_identity_verification_t* pod_257_identity_verification)
{
    return _ttalink_pod_257_identity_verification_encode(dst_addr, src_addr, msg, pod_257_identity_verification, true);
}


static inline uint16_t _ttalink_pod_257_identity_verification_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_257_identity_verification_t* pod_257_identity_verification, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_257_identity_verification_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_257_identity_verification->cmd, pod_257_identity_verification->ack, pod_257_identity_verification->ack_md5);
    }else{
        return ttalink_pod_257_identity_verification_pack_chan(dst_addr, src_addr, chan, msg, pod_257_identity_verification->cmd, pod_257_identity_verification->ack, pod_257_identity_verification->ack_md5);
    }
}

/**
 * @brief Encode a pod_257_identity_verification struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_257_identity_verification C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_257_identity_verification_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_257_identity_verification_t* pod_257_identity_verification)
{
    return _ttalink_pod_257_identity_verification_encode_chan(dst_addr, src_addr, chan, msg, pod_257_identity_verification, false);
}

/**
 * @brief Encode a pod_257_identity_verification struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_257_identity_verification C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_257_identity_verification_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_257_identity_verification_t* pod_257_identity_verification)
{
    return _ttalink_pod_257_identity_verification_encode_chan(dst_addr, src_addr, chan, msg, pod_257_identity_verification, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_257_identity_verification_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint8_t *cmd, uint8_t ack, const uint8_t *ack_md5, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN];
    _tta_put_uint8_t(buf, 16, ack);
    _tta_put_uint8_t_array(buf, 0, cmd, 16);
    _tta_put_uint8_t_array(buf, 17, ack_md5, 16);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION, buf, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_MIN_LEN, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_CRC, nocrc);
#else
    ttalink_pod_257_identity_verification_t packet;
    packet.ack = ack;
    tta_array_memcpy(packet.cmd, cmd, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.ack_md5, ack_md5, sizeof(uint8_t)*16);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION, (const char *)&packet, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_MIN_LEN, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_257_identity_verification message
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  Random number string.
 * @param ack  .
 * @param ack_md5  MD5.
 */
static inline void ttalink_pod_257_identity_verification_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint8_t *cmd, uint8_t ack, const uint8_t *ack_md5)
{
    _ttalink_pod_257_identity_verification_send(dst_addr, src_addr, chan, cmd, ack, ack_md5, false);
}

/**
 * @brief Send a pod_257_identity_verification message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  Random number string.
 * @param ack  .
 * @param ack_md5  MD5.
 */
static inline void ttalink_pod_257_identity_verification_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint8_t *cmd, uint8_t ack, const uint8_t *ack_md5)
{
    _ttalink_pod_257_identity_verification_send(dst_addr, src_addr, chan, cmd, ack, ack_md5, true);
}


static inline void _ttalink_pod_257_identity_verification_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_257_identity_verification_t* pod_257_identity_verification, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_257_identity_verification_send_nocrc(dst_addr, src_addr, chan, pod_257_identity_verification->cmd, pod_257_identity_verification->ack, pod_257_identity_verification->ack_md5);
    }else{
        ttalink_pod_257_identity_verification_send(dst_addr, src_addr, chan, pod_257_identity_verification->cmd, pod_257_identity_verification->ack, pod_257_identity_verification->ack_md5);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION, (const char *)pod_257_identity_verification, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_MIN_LEN, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_257_identity_verification message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_257_identity_verification_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_257_identity_verification_t* pod_257_identity_verification)
{
    _ttalink_pod_257_identity_verification_send_struct(dst_addr, src_addr, chan, pod_257_identity_verification, false);
}

/**
 * @brief Send a pod_257_identity_verification message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_257_identity_verification_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_257_identity_verification_t* pod_257_identity_verification)
{
    _ttalink_pod_257_identity_verification_send_struct(dst_addr, src_addr, chan, pod_257_identity_verification, true);
}

#if TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_257_identity_verification_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint8_t *cmd, uint8_t ack, const uint8_t *ack_md5, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 16, ack);
    _tta_put_uint8_t_array(buf, 0, cmd, 16);
    _tta_put_uint8_t_array(buf, 17, ack_md5, 16);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION, buf, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_MIN_LEN, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_CRC, nocrc);
#else
    ttalink_pod_257_identity_verification_t *packet = (ttalink_pod_257_identity_verification_t *)msgbuf;
    packet->ack = ack;
    tta_array_memcpy(packet->cmd, cmd, sizeof(uint8_t)*16);
    tta_array_memcpy(packet->ack_md5, ack_md5, sizeof(uint8_t)*16);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION, (const char *)packet, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_MIN_LEN, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_257_identity_verification_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint8_t *cmd, uint8_t ack, const uint8_t *ack_md5)
{
    _ttalink_pod_257_identity_verification_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, ack, ack_md5, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_257_identity_verification_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint8_t *cmd, uint8_t ack, const uint8_t *ack_md5)
{
    _ttalink_pod_257_identity_verification_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, ack, ack_md5, true);
}
#endif

#endif

// MESSAGE POD_257_IDENTITY_VERIFICATION UNPACKING


/**
 * @brief Get field cmd from pod_257_identity_verification message
 *
 * @return  Random number string.
 */
static inline uint16_t ttalink_pod_257_identity_verification_get_cmd(const ttalink_message_t* msg, uint8_t *cmd)
{
    return _TTA_RETURN_uint8_t_array(msg, cmd, 16,  0);
}

/**
 * @brief Get field ack from pod_257_identity_verification message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_257_identity_verification_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field ack_md5 from pod_257_identity_verification message
 *
 * @return  MD5.
 */
static inline uint16_t ttalink_pod_257_identity_verification_get_ack_md5(const ttalink_message_t* msg, uint8_t *ack_md5)
{
    return _TTA_RETURN_uint8_t_array(msg, ack_md5, 16,  17);
}

/**
 * @brief Decode a pod_257_identity_verification message into a struct
 *
 * @param msg The message to decode
 * @param pod_257_identity_verification C-struct to decode the message contents into
 */
static inline void ttalink_pod_257_identity_verification_decode(const ttalink_message_t* msg, ttalink_pod_257_identity_verification_t* pod_257_identity_verification)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_pod_257_identity_verification_get_cmd(msg, pod_257_identity_verification->cmd);
    pod_257_identity_verification->ack = ttalink_pod_257_identity_verification_get_ack(msg);
    ttalink_pod_257_identity_verification_get_ack_md5(msg, pod_257_identity_verification->ack_md5);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN? msg->len : TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN;
        memset(pod_257_identity_verification, 0, TTALINK_MSG_ID_POD_257_IDENTITY_VERIFICATION_LEN);
    memcpy(pod_257_identity_verification, _TTA_PAYLOAD(msg), len);
#endif
}
