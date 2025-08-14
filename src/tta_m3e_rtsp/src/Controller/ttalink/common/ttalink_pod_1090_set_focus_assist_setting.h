#pragma once
// MESSAGE POD_1090_SET_FOCUS_ASSIST_SETTING PACKING

#define TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING 11090

TTAPACKED(
typedef struct __ttalink_pod_1090_set_focus_assist_setting_t {
 uint8_t assist_setting; /*< Focus assistant settings.
													Bit [0] - the lens focus assistant enables autofocus.
													Bit [1] - lens focus assistant can be used for manual focus..*/
 uint8_t ack; /*< .*/
}) ttalink_pod_1090_set_focus_assist_setting_t;

#define TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN 2
#define TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_MIN_LEN 2
#define TTALINK_MSG_ID_11090_LEN 2
#define TTALINK_MSG_ID_11090_MIN_LEN 2

#define TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_CRC 61
#define TTALINK_MSG_ID_11090_CRC 61



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1090_SET_FOCUS_ASSIST_SETTING { \
    11090, \
    "POD_1090_SET_FOCUS_ASSIST_SETTING", \
    2, \
    {  { "assist_setting", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1090_set_focus_assist_setting_t, assist_setting) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1090_set_focus_assist_setting_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1090_SET_FOCUS_ASSIST_SETTING { \
    "POD_1090_SET_FOCUS_ASSIST_SETTING", \
    2, \
    {  { "assist_setting", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1090_set_focus_assist_setting_t, assist_setting) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1090_set_focus_assist_setting_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1090_set_focus_assist_setting_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t assist_setting, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN];
    _tta_put_uint8_t(buf, 0, assist_setting);
    _tta_put_uint8_t(buf, 1, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN);
#else
    ttalink_pod_1090_set_focus_assist_setting_t packet;
    packet.assist_setting = assist_setting;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_MIN_LEN, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_CRC, nocrc);
}

/**
 * @brief Pack a pod_1090_set_focus_assist_setting message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param assist_setting Focus assistant settings.
													Bit [0] - the lens focus assistant enables autofocus.
													Bit [1] - lens focus assistant can be used for manual focus..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1090_set_focus_assist_setting_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t assist_setting, uint8_t ack)
{
    return _ttalink_pod_1090_set_focus_assist_setting_pack(dst_addr, src_addr, msg,  assist_setting, ack, false);
}

/**
 * @brief Pack a pod_1090_set_focus_assist_setting message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param assist_setting Focus assistant settings.
													Bit [0] - the lens focus assistant enables autofocus.
													Bit [1] - lens focus assistant can be used for manual focus..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1090_set_focus_assist_setting_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t assist_setting, uint8_t ack)
{
    return _ttalink_pod_1090_set_focus_assist_setting_pack(dst_addr, src_addr, msg,  assist_setting, ack, true);
}


static inline uint16_t _ttalink_pod_1090_set_focus_assist_setting_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t assist_setting,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN];
    _tta_put_uint8_t(buf, 0, assist_setting);
    _tta_put_uint8_t(buf, 1, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN);
#else
    ttalink_pod_1090_set_focus_assist_setting_t packet;
    packet.assist_setting = assist_setting;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_MIN_LEN, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_CRC, nocrc);
}

/**
 * @brief Pack a pod_1090_set_focus_assist_setting message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param assist_setting Focus assistant settings.
													Bit [0] - the lens focus assistant enables autofocus.
													Bit [1] - lens focus assistant can be used for manual focus..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1090_set_focus_assist_setting_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t assist_setting,uint8_t ack)
{
    return _ttalink_pod_1090_set_focus_assist_setting_pack_chan(dst_addr, src_addr, chan, msg,  assist_setting, ack, false);
}

/**
 * @brief Pack a pod_1090_set_focus_assist_setting message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param assist_setting Focus assistant settings.
													Bit [0] - the lens focus assistant enables autofocus.
													Bit [1] - lens focus assistant can be used for manual focus..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1090_set_focus_assist_setting_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t assist_setting,uint8_t ack)
{
    return _ttalink_pod_1090_set_focus_assist_setting_pack_chan(dst_addr, src_addr, chan, msg,  assist_setting, ack, true);
}


static inline uint16_t _ttalink_pod_1090_set_focus_assist_setting_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1090_set_focus_assist_setting_t* pod_1090_set_focus_assist_setting, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1090_set_focus_assist_setting_pack_nocrc(dst_addr, src_addr, msg, pod_1090_set_focus_assist_setting->assist_setting, pod_1090_set_focus_assist_setting->ack);
    }else{
        return ttalink_pod_1090_set_focus_assist_setting_pack(dst_addr, src_addr, msg, pod_1090_set_focus_assist_setting->assist_setting, pod_1090_set_focus_assist_setting->ack);
    }
    
}

/**
 * @brief Encode a pod_1090_set_focus_assist_setting struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1090_set_focus_assist_setting C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1090_set_focus_assist_setting_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1090_set_focus_assist_setting_t* pod_1090_set_focus_assist_setting)
{
    return _ttalink_pod_1090_set_focus_assist_setting_encode(dst_addr, src_addr, msg, pod_1090_set_focus_assist_setting, false);
}

/**
 * @brief Encode a pod_1090_set_focus_assist_setting struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1090_set_focus_assist_setting C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1090_set_focus_assist_setting_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1090_set_focus_assist_setting_t* pod_1090_set_focus_assist_setting)
{
    return _ttalink_pod_1090_set_focus_assist_setting_encode(dst_addr, src_addr, msg, pod_1090_set_focus_assist_setting, true);
}


static inline uint16_t _ttalink_pod_1090_set_focus_assist_setting_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1090_set_focus_assist_setting_t* pod_1090_set_focus_assist_setting, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1090_set_focus_assist_setting_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1090_set_focus_assist_setting->assist_setting, pod_1090_set_focus_assist_setting->ack);
    }else{
        return ttalink_pod_1090_set_focus_assist_setting_pack_chan(dst_addr, src_addr, chan, msg, pod_1090_set_focus_assist_setting->assist_setting, pod_1090_set_focus_assist_setting->ack);
    }
}

/**
 * @brief Encode a pod_1090_set_focus_assist_setting struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1090_set_focus_assist_setting C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1090_set_focus_assist_setting_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1090_set_focus_assist_setting_t* pod_1090_set_focus_assist_setting)
{
    return _ttalink_pod_1090_set_focus_assist_setting_encode_chan(dst_addr, src_addr, chan, msg, pod_1090_set_focus_assist_setting, false);
}

/**
 * @brief Encode a pod_1090_set_focus_assist_setting struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1090_set_focus_assist_setting C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1090_set_focus_assist_setting_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1090_set_focus_assist_setting_t* pod_1090_set_focus_assist_setting)
{
    return _ttalink_pod_1090_set_focus_assist_setting_encode_chan(dst_addr, src_addr, chan, msg, pod_1090_set_focus_assist_setting, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1090_set_focus_assist_setting_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t assist_setting, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN];
    _tta_put_uint8_t(buf, 0, assist_setting);
    _tta_put_uint8_t(buf, 1, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING, buf, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_MIN_LEN, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_CRC, nocrc);
#else
    ttalink_pod_1090_set_focus_assist_setting_t packet;
    packet.assist_setting = assist_setting;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING, (const char *)&packet, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_MIN_LEN, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1090_set_focus_assist_setting message
 * @param chan TTAlink channel to send the message
 *
 * @param assist_setting Focus assistant settings.
													Bit [0] - the lens focus assistant enables autofocus.
													Bit [1] - lens focus assistant can be used for manual focus..
 * @param ack .
 */
static inline void ttalink_pod_1090_set_focus_assist_setting_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t assist_setting, uint8_t ack)
{
    _ttalink_pod_1090_set_focus_assist_setting_send(dst_addr, src_addr, chan, assist_setting, ack, false);
}

/**
 * @brief Send a pod_1090_set_focus_assist_setting message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param assist_setting Focus assistant settings.
													Bit [0] - the lens focus assistant enables autofocus.
													Bit [1] - lens focus assistant can be used for manual focus..
 * @param ack .
 */
static inline void ttalink_pod_1090_set_focus_assist_setting_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t assist_setting, uint8_t ack)
{
    _ttalink_pod_1090_set_focus_assist_setting_send(dst_addr, src_addr, chan, assist_setting, ack, true);
}


static inline void _ttalink_pod_1090_set_focus_assist_setting_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1090_set_focus_assist_setting_t* pod_1090_set_focus_assist_setting, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1090_set_focus_assist_setting_send_nocrc(dst_addr, src_addr, chan, pod_1090_set_focus_assist_setting->assist_setting, pod_1090_set_focus_assist_setting->ack);
    }else{
        ttalink_pod_1090_set_focus_assist_setting_send(dst_addr, src_addr, chan, pod_1090_set_focus_assist_setting->assist_setting, pod_1090_set_focus_assist_setting->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING, (const char *)pod_1090_set_focus_assist_setting, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_MIN_LEN, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1090_set_focus_assist_setting message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1090_set_focus_assist_setting_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1090_set_focus_assist_setting_t* pod_1090_set_focus_assist_setting)
{
    _ttalink_pod_1090_set_focus_assist_setting_send_struct(dst_addr, src_addr, chan, pod_1090_set_focus_assist_setting, false);
}

/**
 * @brief Send a pod_1090_set_focus_assist_setting message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1090_set_focus_assist_setting_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1090_set_focus_assist_setting_t* pod_1090_set_focus_assist_setting)
{
    _ttalink_pod_1090_set_focus_assist_setting_send_struct(dst_addr, src_addr, chan, pod_1090_set_focus_assist_setting, true);
}

#if TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1090_set_focus_assist_setting_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t assist_setting, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, assist_setting);
    _tta_put_uint8_t(buf, 1, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING, buf, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_MIN_LEN, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_CRC, nocrc);
#else
    ttalink_pod_1090_set_focus_assist_setting_t *packet = (ttalink_pod_1090_set_focus_assist_setting_t *)msgbuf;
    packet->assist_setting = assist_setting;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING, (const char *)packet, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_MIN_LEN, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1090_set_focus_assist_setting_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t assist_setting, uint8_t ack)
{
    _ttalink_pod_1090_set_focus_assist_setting_send_buf(dst_addr, src_addr, msgbuf, chan, assist_setting, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1090_set_focus_assist_setting_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t assist_setting, uint8_t ack)
{
    _ttalink_pod_1090_set_focus_assist_setting_send_buf(dst_addr, src_addr, msgbuf, chan, assist_setting, ack, true);
}
#endif

#endif

// MESSAGE POD_1090_SET_FOCUS_ASSIST_SETTING UNPACKING


/**
 * @brief Get field assist_setting from pod_1090_set_focus_assist_setting message
 *
 * @return Focus assistant settings.
													Bit [0] - the lens focus assistant enables autofocus.
													Bit [1] - lens focus assistant can be used for manual focus..
 */
static inline uint8_t ttalink_pod_1090_set_focus_assist_setting_get_assist_setting(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack from pod_1090_set_focus_assist_setting message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1090_set_focus_assist_setting_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a pod_1090_set_focus_assist_setting message into a struct
 *
 * @param msg The message to decode
 * @param pod_1090_set_focus_assist_setting C-struct to decode the message contents into
 */
static inline void ttalink_pod_1090_set_focus_assist_setting_decode(const ttalink_message_t* msg, ttalink_pod_1090_set_focus_assist_setting_t* pod_1090_set_focus_assist_setting)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1090_set_focus_assist_setting->assist_setting = ttalink_pod_1090_set_focus_assist_setting_get_assist_setting(msg);
    pod_1090_set_focus_assist_setting->ack = ttalink_pod_1090_set_focus_assist_setting_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN? msg->len : TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN;
        memset(pod_1090_set_focus_assist_setting, 0, TTALINK_MSG_ID_POD_1090_SET_FOCUS_ASSIST_SETTING_LEN);
    memcpy(pod_1090_set_focus_assist_setting, _TTA_PAYLOAD(msg), len);
#endif
}
