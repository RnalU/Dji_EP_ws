#pragma once
// MESSAGE POD_1652_SEARCHLIGHT_MORSE_CTRL PACKING

#define TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL 11652

TTAPACKED(
typedef struct __ttalink_pod_1652_searchlight_morse_ctrl_t {
 uint8_t cmd; /*<  0 : close 1 : open */
 uint8_t effect; /*<  0 : no 1 : yes */
 uint8_t data[32]; /*< */
 uint8_t ack; /*< .*/
}) ttalink_pod_1652_searchlight_morse_ctrl_t;

#define TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN 35
#define TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_MIN_LEN 35
#define TTALINK_MSG_ID_11652_LEN 35
#define TTALINK_MSG_ID_11652_MIN_LEN 35

#define TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_CRC 155
#define TTALINK_MSG_ID_11652_CRC 155

#define TTALINK_MSG_POD_1652_SEARCHLIGHT_MORSE_CTRL_FIELD_DATA_LEN 32

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1652_SEARCHLIGHT_MORSE_CTRL { \
    11652, \
    "POD_1652_SEARCHLIGHT_MORSE_CTRL", \
    4, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1652_searchlight_morse_ctrl_t, cmd) }, \
         { "effect", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1652_searchlight_morse_ctrl_t, effect) }, \
         { "data", NULL, TTALINK_TYPE_UINT8_T, 32, 2, offsetof(ttalink_pod_1652_searchlight_morse_ctrl_t, data) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 34, offsetof(ttalink_pod_1652_searchlight_morse_ctrl_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1652_SEARCHLIGHT_MORSE_CTRL { \
    "POD_1652_SEARCHLIGHT_MORSE_CTRL", \
    4, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1652_searchlight_morse_ctrl_t, cmd) }, \
         { "effect", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1652_searchlight_morse_ctrl_t, effect) }, \
         { "data", NULL, TTALINK_TYPE_UINT8_T, 32, 2, offsetof(ttalink_pod_1652_searchlight_morse_ctrl_t, data) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 34, offsetof(ttalink_pod_1652_searchlight_morse_ctrl_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1652_searchlight_morse_ctrl_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t effect, const uint8_t *data, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, effect);
    _tta_put_uint8_t(buf, 34, ack);
    _tta_put_uint8_t_array(buf, 2, data, 32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN);
#else
    ttalink_pod_1652_searchlight_morse_ctrl_t packet;
    packet.cmd = cmd;
    packet.effect = effect;
    packet.ack = ack;
    tta_array_memcpy(packet.data, data, sizeof(uint8_t)*32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_CRC, nocrc);
}

/**
 * @brief Pack a pod_1652_searchlight_morse_ctrl message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  0 : close 1 : open 
 * @param effect  0 : no 1 : yes 
 * @param data 
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1652_searchlight_morse_ctrl_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t effect, const uint8_t *data, uint8_t ack)
{
    return _ttalink_pod_1652_searchlight_morse_ctrl_pack(dst_addr, src_addr, msg,  cmd, effect, data, ack, false);
}

/**
 * @brief Pack a pod_1652_searchlight_morse_ctrl message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  0 : close 1 : open 
 * @param effect  0 : no 1 : yes 
 * @param data 
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1652_searchlight_morse_ctrl_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t effect, const uint8_t *data, uint8_t ack)
{
    return _ttalink_pod_1652_searchlight_morse_ctrl_pack(dst_addr, src_addr, msg,  cmd, effect, data, ack, true);
}


static inline uint16_t _ttalink_pod_1652_searchlight_morse_ctrl_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t effect,const uint8_t *data,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, effect);
    _tta_put_uint8_t(buf, 34, ack);
    _tta_put_uint8_t_array(buf, 2, data, 32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN);
#else
    ttalink_pod_1652_searchlight_morse_ctrl_t packet;
    packet.cmd = cmd;
    packet.effect = effect;
    packet.ack = ack;
    tta_array_memcpy(packet.data, data, sizeof(uint8_t)*32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_CRC, nocrc);
}

/**
 * @brief Pack a pod_1652_searchlight_morse_ctrl message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  0 : close 1 : open 
 * @param effect  0 : no 1 : yes 
 * @param data 
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1652_searchlight_morse_ctrl_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t effect,const uint8_t *data,uint8_t ack)
{
    return _ttalink_pod_1652_searchlight_morse_ctrl_pack_chan(dst_addr, src_addr, chan, msg,  cmd, effect, data, ack, false);
}

/**
 * @brief Pack a pod_1652_searchlight_morse_ctrl message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  0 : close 1 : open 
 * @param effect  0 : no 1 : yes 
 * @param data 
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1652_searchlight_morse_ctrl_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t effect,const uint8_t *data,uint8_t ack)
{
    return _ttalink_pod_1652_searchlight_morse_ctrl_pack_chan(dst_addr, src_addr, chan, msg,  cmd, effect, data, ack, true);
}


static inline uint16_t _ttalink_pod_1652_searchlight_morse_ctrl_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1652_searchlight_morse_ctrl_t* pod_1652_searchlight_morse_ctrl, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1652_searchlight_morse_ctrl_pack_nocrc(dst_addr, src_addr, msg, pod_1652_searchlight_morse_ctrl->cmd, pod_1652_searchlight_morse_ctrl->effect, pod_1652_searchlight_morse_ctrl->data, pod_1652_searchlight_morse_ctrl->ack);
    }else{
        return ttalink_pod_1652_searchlight_morse_ctrl_pack(dst_addr, src_addr, msg, pod_1652_searchlight_morse_ctrl->cmd, pod_1652_searchlight_morse_ctrl->effect, pod_1652_searchlight_morse_ctrl->data, pod_1652_searchlight_morse_ctrl->ack);
    }
    
}

/**
 * @brief Encode a pod_1652_searchlight_morse_ctrl struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1652_searchlight_morse_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1652_searchlight_morse_ctrl_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1652_searchlight_morse_ctrl_t* pod_1652_searchlight_morse_ctrl)
{
    return _ttalink_pod_1652_searchlight_morse_ctrl_encode(dst_addr, src_addr, msg, pod_1652_searchlight_morse_ctrl, false);
}

/**
 * @brief Encode a pod_1652_searchlight_morse_ctrl struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1652_searchlight_morse_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1652_searchlight_morse_ctrl_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1652_searchlight_morse_ctrl_t* pod_1652_searchlight_morse_ctrl)
{
    return _ttalink_pod_1652_searchlight_morse_ctrl_encode(dst_addr, src_addr, msg, pod_1652_searchlight_morse_ctrl, true);
}


static inline uint16_t _ttalink_pod_1652_searchlight_morse_ctrl_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1652_searchlight_morse_ctrl_t* pod_1652_searchlight_morse_ctrl, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1652_searchlight_morse_ctrl_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1652_searchlight_morse_ctrl->cmd, pod_1652_searchlight_morse_ctrl->effect, pod_1652_searchlight_morse_ctrl->data, pod_1652_searchlight_morse_ctrl->ack);
    }else{
        return ttalink_pod_1652_searchlight_morse_ctrl_pack_chan(dst_addr, src_addr, chan, msg, pod_1652_searchlight_morse_ctrl->cmd, pod_1652_searchlight_morse_ctrl->effect, pod_1652_searchlight_morse_ctrl->data, pod_1652_searchlight_morse_ctrl->ack);
    }
}

/**
 * @brief Encode a pod_1652_searchlight_morse_ctrl struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1652_searchlight_morse_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1652_searchlight_morse_ctrl_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1652_searchlight_morse_ctrl_t* pod_1652_searchlight_morse_ctrl)
{
    return _ttalink_pod_1652_searchlight_morse_ctrl_encode_chan(dst_addr, src_addr, chan, msg, pod_1652_searchlight_morse_ctrl, false);
}

/**
 * @brief Encode a pod_1652_searchlight_morse_ctrl struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1652_searchlight_morse_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1652_searchlight_morse_ctrl_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1652_searchlight_morse_ctrl_t* pod_1652_searchlight_morse_ctrl)
{
    return _ttalink_pod_1652_searchlight_morse_ctrl_encode_chan(dst_addr, src_addr, chan, msg, pod_1652_searchlight_morse_ctrl, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1652_searchlight_morse_ctrl_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t effect, const uint8_t *data, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, effect);
    _tta_put_uint8_t(buf, 34, ack);
    _tta_put_uint8_t_array(buf, 2, data, 32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL, buf, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_CRC, nocrc);
#else
    ttalink_pod_1652_searchlight_morse_ctrl_t packet;
    packet.cmd = cmd;
    packet.effect = effect;
    packet.ack = ack;
    tta_array_memcpy(packet.data, data, sizeof(uint8_t)*32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL, (const char *)&packet, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1652_searchlight_morse_ctrl message
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  0 : close 1 : open 
 * @param effect  0 : no 1 : yes 
 * @param data 
 * @param ack .
 */
static inline void ttalink_pod_1652_searchlight_morse_ctrl_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t effect, const uint8_t *data, uint8_t ack)
{
    _ttalink_pod_1652_searchlight_morse_ctrl_send(dst_addr, src_addr, chan, cmd, effect, data, ack, false);
}

/**
 * @brief Send a pod_1652_searchlight_morse_ctrl message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  0 : close 1 : open 
 * @param effect  0 : no 1 : yes 
 * @param data 
 * @param ack .
 */
static inline void ttalink_pod_1652_searchlight_morse_ctrl_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t effect, const uint8_t *data, uint8_t ack)
{
    _ttalink_pod_1652_searchlight_morse_ctrl_send(dst_addr, src_addr, chan, cmd, effect, data, ack, true);
}


static inline void _ttalink_pod_1652_searchlight_morse_ctrl_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1652_searchlight_morse_ctrl_t* pod_1652_searchlight_morse_ctrl, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1652_searchlight_morse_ctrl_send_nocrc(dst_addr, src_addr, chan, pod_1652_searchlight_morse_ctrl->cmd, pod_1652_searchlight_morse_ctrl->effect, pod_1652_searchlight_morse_ctrl->data, pod_1652_searchlight_morse_ctrl->ack);
    }else{
        ttalink_pod_1652_searchlight_morse_ctrl_send(dst_addr, src_addr, chan, pod_1652_searchlight_morse_ctrl->cmd, pod_1652_searchlight_morse_ctrl->effect, pod_1652_searchlight_morse_ctrl->data, pod_1652_searchlight_morse_ctrl->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL, (const char *)pod_1652_searchlight_morse_ctrl, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1652_searchlight_morse_ctrl message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1652_searchlight_morse_ctrl_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1652_searchlight_morse_ctrl_t* pod_1652_searchlight_morse_ctrl)
{
    _ttalink_pod_1652_searchlight_morse_ctrl_send_struct(dst_addr, src_addr, chan, pod_1652_searchlight_morse_ctrl, false);
}

/**
 * @brief Send a pod_1652_searchlight_morse_ctrl message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1652_searchlight_morse_ctrl_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1652_searchlight_morse_ctrl_t* pod_1652_searchlight_morse_ctrl)
{
    _ttalink_pod_1652_searchlight_morse_ctrl_send_struct(dst_addr, src_addr, chan, pod_1652_searchlight_morse_ctrl, true);
}

#if TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1652_searchlight_morse_ctrl_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t effect, const uint8_t *data, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, effect);
    _tta_put_uint8_t(buf, 34, ack);
    _tta_put_uint8_t_array(buf, 2, data, 32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL, buf, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_CRC, nocrc);
#else
    ttalink_pod_1652_searchlight_morse_ctrl_t *packet = (ttalink_pod_1652_searchlight_morse_ctrl_t *)msgbuf;
    packet->cmd = cmd;
    packet->effect = effect;
    packet->ack = ack;
    tta_array_memcpy(packet->data, data, sizeof(uint8_t)*32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL, (const char *)packet, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1652_searchlight_morse_ctrl_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t effect, const uint8_t *data, uint8_t ack)
{
    _ttalink_pod_1652_searchlight_morse_ctrl_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, effect, data, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1652_searchlight_morse_ctrl_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t effect, const uint8_t *data, uint8_t ack)
{
    _ttalink_pod_1652_searchlight_morse_ctrl_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, effect, data, ack, true);
}
#endif

#endif

// MESSAGE POD_1652_SEARCHLIGHT_MORSE_CTRL UNPACKING


/**
 * @brief Get field cmd from pod_1652_searchlight_morse_ctrl message
 *
 * @return  0 : close 1 : open 
 */
static inline uint8_t ttalink_pod_1652_searchlight_morse_ctrl_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field effect from pod_1652_searchlight_morse_ctrl message
 *
 * @return  0 : no 1 : yes 
 */
static inline uint8_t ttalink_pod_1652_searchlight_morse_ctrl_get_effect(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field data from pod_1652_searchlight_morse_ctrl message
 *
 * @return 
 */
static inline uint16_t ttalink_pod_1652_searchlight_morse_ctrl_get_data(const ttalink_message_t* msg, uint8_t *data)
{
    return _TTA_RETURN_uint8_t_array(msg, data, 32,  2);
}

/**
 * @brief Get field ack from pod_1652_searchlight_morse_ctrl message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1652_searchlight_morse_ctrl_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Decode a pod_1652_searchlight_morse_ctrl message into a struct
 *
 * @param msg The message to decode
 * @param pod_1652_searchlight_morse_ctrl C-struct to decode the message contents into
 */
static inline void ttalink_pod_1652_searchlight_morse_ctrl_decode(const ttalink_message_t* msg, ttalink_pod_1652_searchlight_morse_ctrl_t* pod_1652_searchlight_morse_ctrl)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1652_searchlight_morse_ctrl->cmd = ttalink_pod_1652_searchlight_morse_ctrl_get_cmd(msg);
    pod_1652_searchlight_morse_ctrl->effect = ttalink_pod_1652_searchlight_morse_ctrl_get_effect(msg);
    ttalink_pod_1652_searchlight_morse_ctrl_get_data(msg, pod_1652_searchlight_morse_ctrl->data);
    pod_1652_searchlight_morse_ctrl->ack = ttalink_pod_1652_searchlight_morse_ctrl_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN? msg->len : TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN;
        memset(pod_1652_searchlight_morse_ctrl, 0, TTALINK_MSG_ID_POD_1652_SEARCHLIGHT_MORSE_CTRL_LEN);
    memcpy(pod_1652_searchlight_morse_ctrl, _TTA_PAYLOAD(msg), len);
#endif
}
