#pragma once
// MESSAGE WP_EVENT_CTRL_ACK PACKING

#define TTALINK_MSG_ID_WP_EVENT_CTRL_ACK 2159

TTAPACKED(
typedef struct __ttalink_wp_event_ctrl_ack_t {
 uint32_t update_time; /*< .*/
 uint32_t ctrl_cmd; /*<    */
 uint8_t ctrl_action; /*<   0=NULL  */
 uint32_t ctrl_time; /*<  预留控制时间*/
 uint8_t ack; /*<   0=正常 1=错误. */
}) ttalink_wp_event_ctrl_ack_t;

#define TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN 14
#define TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_MIN_LEN 14
#define TTALINK_MSG_ID_2159_LEN 14
#define TTALINK_MSG_ID_2159_MIN_LEN 14

#define TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_CRC 179
#define TTALINK_MSG_ID_2159_CRC 179



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_WP_EVENT_CTRL_ACK { \
    2159, \
    "WP_EVENT_CTRL_ACK", \
    5, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_wp_event_ctrl_ack_t, update_time) }, \
         { "ctrl_cmd", NULL, TTALINK_TYPE_UINT32_T, 0, 4, offsetof(ttalink_wp_event_ctrl_ack_t, ctrl_cmd) }, \
         { "ctrl_action", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_wp_event_ctrl_ack_t, ctrl_action) }, \
         { "ctrl_time", NULL, TTALINK_TYPE_UINT32_T, 0, 9, offsetof(ttalink_wp_event_ctrl_ack_t, ctrl_time) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 13, offsetof(ttalink_wp_event_ctrl_ack_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_WP_EVENT_CTRL_ACK { \
    "WP_EVENT_CTRL_ACK", \
    5, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_wp_event_ctrl_ack_t, update_time) }, \
         { "ctrl_cmd", NULL, TTALINK_TYPE_UINT32_T, 0, 4, offsetof(ttalink_wp_event_ctrl_ack_t, ctrl_cmd) }, \
         { "ctrl_action", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_wp_event_ctrl_ack_t, ctrl_action) }, \
         { "ctrl_time", NULL, TTALINK_TYPE_UINT32_T, 0, 9, offsetof(ttalink_wp_event_ctrl_ack_t, ctrl_time) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 13, offsetof(ttalink_wp_event_ctrl_ack_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_wp_event_ctrl_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t ctrl_cmd, uint8_t ctrl_action, uint32_t ctrl_time, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, ctrl_cmd);
    _tta_put_uint8_t(buf, 8, ctrl_action);
    _tta_put_uint32_t(buf, 9, ctrl_time);
    _tta_put_uint8_t(buf, 13, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN);
#else
    ttalink_wp_event_ctrl_ack_t packet;
    packet.update_time = update_time;
    packet.ctrl_cmd = ctrl_cmd;
    packet.ctrl_action = ctrl_action;
    packet.ctrl_time = ctrl_time;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_WP_EVENT_CTRL_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_MIN_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_CRC, nocrc);
}

/**
 * @brief Pack a wp_event_ctrl_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time .
 * @param ctrl_cmd    
 * @param ctrl_action   0=NULL  
 * @param ctrl_time  预留控制时间
 * @param ack   0=正常 1=错误. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_wp_event_ctrl_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t ctrl_cmd, uint8_t ctrl_action, uint32_t ctrl_time, uint8_t ack)
{
    return _ttalink_wp_event_ctrl_ack_pack(dst_addr, src_addr, msg,  update_time, ctrl_cmd, ctrl_action, ctrl_time, ack, false);
}

/**
 * @brief Pack a wp_event_ctrl_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time .
 * @param ctrl_cmd    
 * @param ctrl_action   0=NULL  
 * @param ctrl_time  预留控制时间
 * @param ack   0=正常 1=错误. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_wp_event_ctrl_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t ctrl_cmd, uint8_t ctrl_action, uint32_t ctrl_time, uint8_t ack)
{
    return _ttalink_wp_event_ctrl_ack_pack(dst_addr, src_addr, msg,  update_time, ctrl_cmd, ctrl_action, ctrl_time, ack, true);
}


static inline uint16_t _ttalink_wp_event_ctrl_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t ctrl_cmd,uint8_t ctrl_action,uint32_t ctrl_time,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, ctrl_cmd);
    _tta_put_uint8_t(buf, 8, ctrl_action);
    _tta_put_uint32_t(buf, 9, ctrl_time);
    _tta_put_uint8_t(buf, 13, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN);
#else
    ttalink_wp_event_ctrl_ack_t packet;
    packet.update_time = update_time;
    packet.ctrl_cmd = ctrl_cmd;
    packet.ctrl_action = ctrl_action;
    packet.ctrl_time = ctrl_time;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_WP_EVENT_CTRL_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_MIN_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_CRC, nocrc);
}

/**
 * @brief Pack a wp_event_ctrl_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time .
 * @param ctrl_cmd    
 * @param ctrl_action   0=NULL  
 * @param ctrl_time  预留控制时间
 * @param ack   0=正常 1=错误. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_wp_event_ctrl_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t ctrl_cmd,uint8_t ctrl_action,uint32_t ctrl_time,uint8_t ack)
{
    return _ttalink_wp_event_ctrl_ack_pack_chan(dst_addr, src_addr, chan, msg,  update_time, ctrl_cmd, ctrl_action, ctrl_time, ack, false);
}

/**
 * @brief Pack a wp_event_ctrl_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time .
 * @param ctrl_cmd    
 * @param ctrl_action   0=NULL  
 * @param ctrl_time  预留控制时间
 * @param ack   0=正常 1=错误. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_wp_event_ctrl_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t ctrl_cmd,uint8_t ctrl_action,uint32_t ctrl_time,uint8_t ack)
{
    return _ttalink_wp_event_ctrl_ack_pack_chan(dst_addr, src_addr, chan, msg,  update_time, ctrl_cmd, ctrl_action, ctrl_time, ack, true);
}


static inline uint16_t _ttalink_wp_event_ctrl_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_wp_event_ctrl_ack_t* wp_event_ctrl_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_wp_event_ctrl_ack_pack_nocrc(dst_addr, src_addr, msg, wp_event_ctrl_ack->update_time, wp_event_ctrl_ack->ctrl_cmd, wp_event_ctrl_ack->ctrl_action, wp_event_ctrl_ack->ctrl_time, wp_event_ctrl_ack->ack);
    }else{
        return ttalink_wp_event_ctrl_ack_pack(dst_addr, src_addr, msg, wp_event_ctrl_ack->update_time, wp_event_ctrl_ack->ctrl_cmd, wp_event_ctrl_ack->ctrl_action, wp_event_ctrl_ack->ctrl_time, wp_event_ctrl_ack->ack);
    }
    
}

/**
 * @brief Encode a wp_event_ctrl_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param wp_event_ctrl_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_wp_event_ctrl_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_wp_event_ctrl_ack_t* wp_event_ctrl_ack)
{
    return _ttalink_wp_event_ctrl_ack_encode(dst_addr, src_addr, msg, wp_event_ctrl_ack, false);
}

/**
 * @brief Encode a wp_event_ctrl_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param wp_event_ctrl_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_wp_event_ctrl_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_wp_event_ctrl_ack_t* wp_event_ctrl_ack)
{
    return _ttalink_wp_event_ctrl_ack_encode(dst_addr, src_addr, msg, wp_event_ctrl_ack, true);
}


static inline uint16_t _ttalink_wp_event_ctrl_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_wp_event_ctrl_ack_t* wp_event_ctrl_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_wp_event_ctrl_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, wp_event_ctrl_ack->update_time, wp_event_ctrl_ack->ctrl_cmd, wp_event_ctrl_ack->ctrl_action, wp_event_ctrl_ack->ctrl_time, wp_event_ctrl_ack->ack);
    }else{
        return ttalink_wp_event_ctrl_ack_pack_chan(dst_addr, src_addr, chan, msg, wp_event_ctrl_ack->update_time, wp_event_ctrl_ack->ctrl_cmd, wp_event_ctrl_ack->ctrl_action, wp_event_ctrl_ack->ctrl_time, wp_event_ctrl_ack->ack);
    }
}

/**
 * @brief Encode a wp_event_ctrl_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param wp_event_ctrl_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_wp_event_ctrl_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_wp_event_ctrl_ack_t* wp_event_ctrl_ack)
{
    return _ttalink_wp_event_ctrl_ack_encode_chan(dst_addr, src_addr, chan, msg, wp_event_ctrl_ack, false);
}

/**
 * @brief Encode a wp_event_ctrl_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param wp_event_ctrl_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_wp_event_ctrl_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_wp_event_ctrl_ack_t* wp_event_ctrl_ack)
{
    return _ttalink_wp_event_ctrl_ack_encode_chan(dst_addr, src_addr, chan, msg, wp_event_ctrl_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_wp_event_ctrl_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t ctrl_cmd, uint8_t ctrl_action, uint32_t ctrl_time, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, ctrl_cmd);
    _tta_put_uint8_t(buf, 8, ctrl_action);
    _tta_put_uint32_t(buf, 9, ctrl_time);
    _tta_put_uint8_t(buf, 13, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK, buf, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_MIN_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_CRC, nocrc);
#else
    ttalink_wp_event_ctrl_ack_t packet;
    packet.update_time = update_time;
    packet.ctrl_cmd = ctrl_cmd;
    packet.ctrl_action = ctrl_action;
    packet.ctrl_time = ctrl_time;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK, (const char *)&packet, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_MIN_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a wp_event_ctrl_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time .
 * @param ctrl_cmd    
 * @param ctrl_action   0=NULL  
 * @param ctrl_time  预留控制时间
 * @param ack   0=正常 1=错误. 
 */
static inline void ttalink_wp_event_ctrl_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t ctrl_cmd, uint8_t ctrl_action, uint32_t ctrl_time, uint8_t ack)
{
    _ttalink_wp_event_ctrl_ack_send(dst_addr, src_addr, chan, update_time, ctrl_cmd, ctrl_action, ctrl_time, ack, false);
}

/**
 * @brief Send a wp_event_ctrl_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time .
 * @param ctrl_cmd    
 * @param ctrl_action   0=NULL  
 * @param ctrl_time  预留控制时间
 * @param ack   0=正常 1=错误. 
 */
static inline void ttalink_wp_event_ctrl_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t ctrl_cmd, uint8_t ctrl_action, uint32_t ctrl_time, uint8_t ack)
{
    _ttalink_wp_event_ctrl_ack_send(dst_addr, src_addr, chan, update_time, ctrl_cmd, ctrl_action, ctrl_time, ack, true);
}


static inline void _ttalink_wp_event_ctrl_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_wp_event_ctrl_ack_t* wp_event_ctrl_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_wp_event_ctrl_ack_send_nocrc(dst_addr, src_addr, chan, wp_event_ctrl_ack->update_time, wp_event_ctrl_ack->ctrl_cmd, wp_event_ctrl_ack->ctrl_action, wp_event_ctrl_ack->ctrl_time, wp_event_ctrl_ack->ack);
    }else{
        ttalink_wp_event_ctrl_ack_send(dst_addr, src_addr, chan, wp_event_ctrl_ack->update_time, wp_event_ctrl_ack->ctrl_cmd, wp_event_ctrl_ack->ctrl_action, wp_event_ctrl_ack->ctrl_time, wp_event_ctrl_ack->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK, (const char *)wp_event_ctrl_ack, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_MIN_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a wp_event_ctrl_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_wp_event_ctrl_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_wp_event_ctrl_ack_t* wp_event_ctrl_ack)
{
    _ttalink_wp_event_ctrl_ack_send_struct(dst_addr, src_addr, chan, wp_event_ctrl_ack, false);
}

/**
 * @brief Send a wp_event_ctrl_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_wp_event_ctrl_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_wp_event_ctrl_ack_t* wp_event_ctrl_ack)
{
    _ttalink_wp_event_ctrl_ack_send_struct(dst_addr, src_addr, chan, wp_event_ctrl_ack, true);
}

#if TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_wp_event_ctrl_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t ctrl_cmd, uint8_t ctrl_action, uint32_t ctrl_time, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, ctrl_cmd);
    _tta_put_uint8_t(buf, 8, ctrl_action);
    _tta_put_uint32_t(buf, 9, ctrl_time);
    _tta_put_uint8_t(buf, 13, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK, buf, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_MIN_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_CRC, nocrc);
#else
    ttalink_wp_event_ctrl_ack_t *packet = (ttalink_wp_event_ctrl_ack_t *)msgbuf;
    packet->update_time = update_time;
    packet->ctrl_cmd = ctrl_cmd;
    packet->ctrl_action = ctrl_action;
    packet->ctrl_time = ctrl_time;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK, (const char *)packet, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_MIN_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_wp_event_ctrl_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t ctrl_cmd, uint8_t ctrl_action, uint32_t ctrl_time, uint8_t ack)
{
    _ttalink_wp_event_ctrl_ack_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, ctrl_cmd, ctrl_action, ctrl_time, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_wp_event_ctrl_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t ctrl_cmd, uint8_t ctrl_action, uint32_t ctrl_time, uint8_t ack)
{
    _ttalink_wp_event_ctrl_ack_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, ctrl_cmd, ctrl_action, ctrl_time, ack, true);
}
#endif

#endif

// MESSAGE WP_EVENT_CTRL_ACK UNPACKING


/**
 * @brief Get field update_time from wp_event_ctrl_ack message
 *
 * @return .
 */
static inline uint32_t ttalink_wp_event_ctrl_ack_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field ctrl_cmd from wp_event_ctrl_ack message
 *
 * @return    
 */
static inline uint32_t ttalink_wp_event_ctrl_ack_get_ctrl_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field ctrl_action from wp_event_ctrl_ack message
 *
 * @return   0=NULL  
 */
static inline uint8_t ttalink_wp_event_ctrl_ack_get_ctrl_action(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field ctrl_time from wp_event_ctrl_ack message
 *
 * @return  预留控制时间
 */
static inline uint32_t ttalink_wp_event_ctrl_ack_get_ctrl_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  9);
}

/**
 * @brief Get field ack from wp_event_ctrl_ack message
 *
 * @return   0=正常 1=错误. 
 */
static inline uint8_t ttalink_wp_event_ctrl_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Decode a wp_event_ctrl_ack message into a struct
 *
 * @param msg The message to decode
 * @param wp_event_ctrl_ack C-struct to decode the message contents into
 */
static inline void ttalink_wp_event_ctrl_ack_decode(const ttalink_message_t* msg, ttalink_wp_event_ctrl_ack_t* wp_event_ctrl_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    wp_event_ctrl_ack->update_time = ttalink_wp_event_ctrl_ack_get_update_time(msg);
    wp_event_ctrl_ack->ctrl_cmd = ttalink_wp_event_ctrl_ack_get_ctrl_cmd(msg);
    wp_event_ctrl_ack->ctrl_action = ttalink_wp_event_ctrl_ack_get_ctrl_action(msg);
    wp_event_ctrl_ack->ctrl_time = ttalink_wp_event_ctrl_ack_get_ctrl_time(msg);
    wp_event_ctrl_ack->ack = ttalink_wp_event_ctrl_ack_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN? msg->len : TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN;
        memset(wp_event_ctrl_ack, 0, TTALINK_MSG_ID_WP_EVENT_CTRL_ACK_LEN);
    memcpy(wp_event_ctrl_ack, _TTA_PAYLOAD(msg), len);
#endif
}
