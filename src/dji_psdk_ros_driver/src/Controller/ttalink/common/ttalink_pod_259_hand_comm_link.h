#pragma once
// MESSAGE POD_259_HAND_COMM_LINK PACKING

#define TTALINK_MSG_ID_POD_259_HAND_COMM_LINK 10259

TTAPACKED(
typedef struct __ttalink_pod_259_hand_comm_link_t {
 uint8_t cmd[32]; /*<  .*/
 uint8_t ack; /*<  .*/
 uint8_t ack_data[32]; /*<  .*/
}) ttalink_pod_259_hand_comm_link_t;

#define TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN 65
#define TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_MIN_LEN 65
#define TTALINK_MSG_ID_10259_LEN 65
#define TTALINK_MSG_ID_10259_MIN_LEN 65

#define TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_CRC 127
#define TTALINK_MSG_ID_10259_CRC 127

#define TTALINK_MSG_POD_259_HAND_COMM_LINK_FIELD_CMD_LEN 32
#define TTALINK_MSG_POD_259_HAND_COMM_LINK_FIELD_ACK_DATA_LEN 32

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_259_HAND_COMM_LINK { \
    10259, \
    "POD_259_HAND_COMM_LINK", \
    3, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 32, 0, offsetof(ttalink_pod_259_hand_comm_link_t, cmd) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 32, offsetof(ttalink_pod_259_hand_comm_link_t, ack) }, \
         { "ack_data", NULL, TTALINK_TYPE_UINT8_T, 32, 33, offsetof(ttalink_pod_259_hand_comm_link_t, ack_data) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_259_HAND_COMM_LINK { \
    "POD_259_HAND_COMM_LINK", \
    3, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 32, 0, offsetof(ttalink_pod_259_hand_comm_link_t, cmd) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 32, offsetof(ttalink_pod_259_hand_comm_link_t, ack) }, \
         { "ack_data", NULL, TTALINK_TYPE_UINT8_T, 32, 33, offsetof(ttalink_pod_259_hand_comm_link_t, ack_data) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_259_hand_comm_link_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint8_t *cmd, uint8_t ack, const uint8_t *ack_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN];
    _tta_put_uint8_t(buf, 32, ack);
    _tta_put_uint8_t_array(buf, 0, cmd, 32);
    _tta_put_uint8_t_array(buf, 33, ack_data, 32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN);
#else
    ttalink_pod_259_hand_comm_link_t packet;
    packet.ack = ack;
    tta_array_memcpy(packet.cmd, cmd, sizeof(uint8_t)*32);
    tta_array_memcpy(packet.ack_data, ack_data, sizeof(uint8_t)*32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_259_HAND_COMM_LINK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_MIN_LEN, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_CRC, nocrc);
}

/**
 * @brief Pack a pod_259_hand_comm_link message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  .
 * @param ack  .
 * @param ack_data  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_259_hand_comm_link_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint8_t *cmd, uint8_t ack, const uint8_t *ack_data)
{
    return _ttalink_pod_259_hand_comm_link_pack(dst_addr, src_addr, msg,  cmd, ack, ack_data, false);
}

/**
 * @brief Pack a pod_259_hand_comm_link message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  .
 * @param ack  .
 * @param ack_data  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_259_hand_comm_link_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint8_t *cmd, uint8_t ack, const uint8_t *ack_data)
{
    return _ttalink_pod_259_hand_comm_link_pack(dst_addr, src_addr, msg,  cmd, ack, ack_data, true);
}


static inline uint16_t _ttalink_pod_259_hand_comm_link_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint8_t *cmd,uint8_t ack,const uint8_t *ack_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN];
    _tta_put_uint8_t(buf, 32, ack);
    _tta_put_uint8_t_array(buf, 0, cmd, 32);
    _tta_put_uint8_t_array(buf, 33, ack_data, 32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN);
#else
    ttalink_pod_259_hand_comm_link_t packet;
    packet.ack = ack;
    tta_array_memcpy(packet.cmd, cmd, sizeof(uint8_t)*32);
    tta_array_memcpy(packet.ack_data, ack_data, sizeof(uint8_t)*32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_259_HAND_COMM_LINK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_MIN_LEN, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_CRC, nocrc);
}

/**
 * @brief Pack a pod_259_hand_comm_link message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  .
 * @param ack  .
 * @param ack_data  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_259_hand_comm_link_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint8_t *cmd,uint8_t ack,const uint8_t *ack_data)
{
    return _ttalink_pod_259_hand_comm_link_pack_chan(dst_addr, src_addr, chan, msg,  cmd, ack, ack_data, false);
}

/**
 * @brief Pack a pod_259_hand_comm_link message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  .
 * @param ack  .
 * @param ack_data  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_259_hand_comm_link_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint8_t *cmd,uint8_t ack,const uint8_t *ack_data)
{
    return _ttalink_pod_259_hand_comm_link_pack_chan(dst_addr, src_addr, chan, msg,  cmd, ack, ack_data, true);
}


static inline uint16_t _ttalink_pod_259_hand_comm_link_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_259_hand_comm_link_t* pod_259_hand_comm_link, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_259_hand_comm_link_pack_nocrc(dst_addr, src_addr, msg, pod_259_hand_comm_link->cmd, pod_259_hand_comm_link->ack, pod_259_hand_comm_link->ack_data);
    }else{
        return ttalink_pod_259_hand_comm_link_pack(dst_addr, src_addr, msg, pod_259_hand_comm_link->cmd, pod_259_hand_comm_link->ack, pod_259_hand_comm_link->ack_data);
    }
    
}

/**
 * @brief Encode a pod_259_hand_comm_link struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_259_hand_comm_link C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_259_hand_comm_link_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_259_hand_comm_link_t* pod_259_hand_comm_link)
{
    return _ttalink_pod_259_hand_comm_link_encode(dst_addr, src_addr, msg, pod_259_hand_comm_link, false);
}

/**
 * @brief Encode a pod_259_hand_comm_link struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_259_hand_comm_link C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_259_hand_comm_link_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_259_hand_comm_link_t* pod_259_hand_comm_link)
{
    return _ttalink_pod_259_hand_comm_link_encode(dst_addr, src_addr, msg, pod_259_hand_comm_link, true);
}


static inline uint16_t _ttalink_pod_259_hand_comm_link_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_259_hand_comm_link_t* pod_259_hand_comm_link, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_259_hand_comm_link_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_259_hand_comm_link->cmd, pod_259_hand_comm_link->ack, pod_259_hand_comm_link->ack_data);
    }else{
        return ttalink_pod_259_hand_comm_link_pack_chan(dst_addr, src_addr, chan, msg, pod_259_hand_comm_link->cmd, pod_259_hand_comm_link->ack, pod_259_hand_comm_link->ack_data);
    }
}

/**
 * @brief Encode a pod_259_hand_comm_link struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_259_hand_comm_link C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_259_hand_comm_link_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_259_hand_comm_link_t* pod_259_hand_comm_link)
{
    return _ttalink_pod_259_hand_comm_link_encode_chan(dst_addr, src_addr, chan, msg, pod_259_hand_comm_link, false);
}

/**
 * @brief Encode a pod_259_hand_comm_link struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_259_hand_comm_link C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_259_hand_comm_link_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_259_hand_comm_link_t* pod_259_hand_comm_link)
{
    return _ttalink_pod_259_hand_comm_link_encode_chan(dst_addr, src_addr, chan, msg, pod_259_hand_comm_link, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_259_hand_comm_link_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint8_t *cmd, uint8_t ack, const uint8_t *ack_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN];
    _tta_put_uint8_t(buf, 32, ack);
    _tta_put_uint8_t_array(buf, 0, cmd, 32);
    _tta_put_uint8_t_array(buf, 33, ack_data, 32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK, buf, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_MIN_LEN, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_CRC, nocrc);
#else
    ttalink_pod_259_hand_comm_link_t packet;
    packet.ack = ack;
    tta_array_memcpy(packet.cmd, cmd, sizeof(uint8_t)*32);
    tta_array_memcpy(packet.ack_data, ack_data, sizeof(uint8_t)*32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK, (const char *)&packet, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_MIN_LEN, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_259_hand_comm_link message
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  .
 * @param ack  .
 * @param ack_data  .
 */
static inline void ttalink_pod_259_hand_comm_link_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint8_t *cmd, uint8_t ack, const uint8_t *ack_data)
{
    _ttalink_pod_259_hand_comm_link_send(dst_addr, src_addr, chan, cmd, ack, ack_data, false);
}

/**
 * @brief Send a pod_259_hand_comm_link message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  .
 * @param ack  .
 * @param ack_data  .
 */
static inline void ttalink_pod_259_hand_comm_link_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint8_t *cmd, uint8_t ack, const uint8_t *ack_data)
{
    _ttalink_pod_259_hand_comm_link_send(dst_addr, src_addr, chan, cmd, ack, ack_data, true);
}


static inline void _ttalink_pod_259_hand_comm_link_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_259_hand_comm_link_t* pod_259_hand_comm_link, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_259_hand_comm_link_send_nocrc(dst_addr, src_addr, chan, pod_259_hand_comm_link->cmd, pod_259_hand_comm_link->ack, pod_259_hand_comm_link->ack_data);
    }else{
        ttalink_pod_259_hand_comm_link_send(dst_addr, src_addr, chan, pod_259_hand_comm_link->cmd, pod_259_hand_comm_link->ack, pod_259_hand_comm_link->ack_data);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK, (const char *)pod_259_hand_comm_link, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_MIN_LEN, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_259_hand_comm_link message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_259_hand_comm_link_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_259_hand_comm_link_t* pod_259_hand_comm_link)
{
    _ttalink_pod_259_hand_comm_link_send_struct(dst_addr, src_addr, chan, pod_259_hand_comm_link, false);
}

/**
 * @brief Send a pod_259_hand_comm_link message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_259_hand_comm_link_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_259_hand_comm_link_t* pod_259_hand_comm_link)
{
    _ttalink_pod_259_hand_comm_link_send_struct(dst_addr, src_addr, chan, pod_259_hand_comm_link, true);
}

#if TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_259_hand_comm_link_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint8_t *cmd, uint8_t ack, const uint8_t *ack_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 32, ack);
    _tta_put_uint8_t_array(buf, 0, cmd, 32);
    _tta_put_uint8_t_array(buf, 33, ack_data, 32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK, buf, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_MIN_LEN, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_CRC, nocrc);
#else
    ttalink_pod_259_hand_comm_link_t *packet = (ttalink_pod_259_hand_comm_link_t *)msgbuf;
    packet->ack = ack;
    tta_array_memcpy(packet->cmd, cmd, sizeof(uint8_t)*32);
    tta_array_memcpy(packet->ack_data, ack_data, sizeof(uint8_t)*32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK, (const char *)packet, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_MIN_LEN, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_259_hand_comm_link_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint8_t *cmd, uint8_t ack, const uint8_t *ack_data)
{
    _ttalink_pod_259_hand_comm_link_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, ack, ack_data, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_259_hand_comm_link_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint8_t *cmd, uint8_t ack, const uint8_t *ack_data)
{
    _ttalink_pod_259_hand_comm_link_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, ack, ack_data, true);
}
#endif

#endif

// MESSAGE POD_259_HAND_COMM_LINK UNPACKING


/**
 * @brief Get field cmd from pod_259_hand_comm_link message
 *
 * @return  .
 */
static inline uint16_t ttalink_pod_259_hand_comm_link_get_cmd(const ttalink_message_t* msg, uint8_t *cmd)
{
    return _TTA_RETURN_uint8_t_array(msg, cmd, 32,  0);
}

/**
 * @brief Get field ack from pod_259_hand_comm_link message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_259_hand_comm_link_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field ack_data from pod_259_hand_comm_link message
 *
 * @return  .
 */
static inline uint16_t ttalink_pod_259_hand_comm_link_get_ack_data(const ttalink_message_t* msg, uint8_t *ack_data)
{
    return _TTA_RETURN_uint8_t_array(msg, ack_data, 32,  33);
}

/**
 * @brief Decode a pod_259_hand_comm_link message into a struct
 *
 * @param msg The message to decode
 * @param pod_259_hand_comm_link C-struct to decode the message contents into
 */
static inline void ttalink_pod_259_hand_comm_link_decode(const ttalink_message_t* msg, ttalink_pod_259_hand_comm_link_t* pod_259_hand_comm_link)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_pod_259_hand_comm_link_get_cmd(msg, pod_259_hand_comm_link->cmd);
    pod_259_hand_comm_link->ack = ttalink_pod_259_hand_comm_link_get_ack(msg);
    ttalink_pod_259_hand_comm_link_get_ack_data(msg, pod_259_hand_comm_link->ack_data);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN? msg->len : TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN;
        memset(pod_259_hand_comm_link, 0, TTALINK_MSG_ID_POD_259_HAND_COMM_LINK_LEN);
    memcpy(pod_259_hand_comm_link, _TTA_PAYLOAD(msg), len);
#endif
}
