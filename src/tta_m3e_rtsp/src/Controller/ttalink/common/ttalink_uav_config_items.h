#pragma once
// MESSAGE UAV_CONFIG_ITEMS PACKING

#define TTALINK_MSG_ID_UAV_CONFIG_ITEMS 2084

TTAPACKED(
typedef struct __ttalink_uav_config_items_t {
 uint8_t frame_type; /*<  .*/
 uint8_t voltage_protection; /*<  .*/
 uint8_t wp_num; /*<  .*/
 uint32_t wp_crc; /*<  .*/
}) ttalink_uav_config_items_t;

#define TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN 7
#define TTALINK_MSG_ID_UAV_CONFIG_ITEMS_MIN_LEN 7
#define TTALINK_MSG_ID_2084_LEN 7
#define TTALINK_MSG_ID_2084_MIN_LEN 7

#define TTALINK_MSG_ID_UAV_CONFIG_ITEMS_CRC 169
#define TTALINK_MSG_ID_2084_CRC 169



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_UAV_CONFIG_ITEMS { \
    2084, \
    "UAV_CONFIG_ITEMS", \
    4, \
    {  { "frame_type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_uav_config_items_t, frame_type) }, \
         { "voltage_protection", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_uav_config_items_t, voltage_protection) }, \
         { "wp_num", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_uav_config_items_t, wp_num) }, \
         { "wp_crc", NULL, TTALINK_TYPE_UINT32_T, 0, 3, offsetof(ttalink_uav_config_items_t, wp_crc) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_UAV_CONFIG_ITEMS { \
    "UAV_CONFIG_ITEMS", \
    4, \
    {  { "frame_type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_uav_config_items_t, frame_type) }, \
         { "voltage_protection", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_uav_config_items_t, voltage_protection) }, \
         { "wp_num", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_uav_config_items_t, wp_num) }, \
         { "wp_crc", NULL, TTALINK_TYPE_UINT32_T, 0, 3, offsetof(ttalink_uav_config_items_t, wp_crc) }, \
         } \
}
#endif


static inline uint16_t _ttalink_uav_config_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t frame_type, uint8_t voltage_protection, uint8_t wp_num, uint32_t wp_crc, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, frame_type);
    _tta_put_uint8_t(buf, 1, voltage_protection);
    _tta_put_uint8_t(buf, 2, wp_num);
    _tta_put_uint32_t(buf, 3, wp_crc);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN);
#else
    ttalink_uav_config_items_t packet;
    packet.frame_type = frame_type;
    packet.voltage_protection = voltage_protection;
    packet.wp_num = wp_num;
    packet.wp_crc = wp_crc;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_UAV_CONFIG_ITEMS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a uav_config_items message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param frame_type  .
 * @param voltage_protection  .
 * @param wp_num  .
 * @param wp_crc  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_uav_config_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t frame_type, uint8_t voltage_protection, uint8_t wp_num, uint32_t wp_crc)
{
    return _ttalink_uav_config_items_pack(dst_addr, src_addr, msg,  frame_type, voltage_protection, wp_num, wp_crc, false);
}

/**
 * @brief Pack a uav_config_items message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param frame_type  .
 * @param voltage_protection  .
 * @param wp_num  .
 * @param wp_crc  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_uav_config_items_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t frame_type, uint8_t voltage_protection, uint8_t wp_num, uint32_t wp_crc)
{
    return _ttalink_uav_config_items_pack(dst_addr, src_addr, msg,  frame_type, voltage_protection, wp_num, wp_crc, true);
}


static inline uint16_t _ttalink_uav_config_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t frame_type,uint8_t voltage_protection,uint8_t wp_num,uint32_t wp_crc, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, frame_type);
    _tta_put_uint8_t(buf, 1, voltage_protection);
    _tta_put_uint8_t(buf, 2, wp_num);
    _tta_put_uint32_t(buf, 3, wp_crc);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN);
#else
    ttalink_uav_config_items_t packet;
    packet.frame_type = frame_type;
    packet.voltage_protection = voltage_protection;
    packet.wp_num = wp_num;
    packet.wp_crc = wp_crc;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_UAV_CONFIG_ITEMS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a uav_config_items message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param frame_type  .
 * @param voltage_protection  .
 * @param wp_num  .
 * @param wp_crc  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_uav_config_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t frame_type,uint8_t voltage_protection,uint8_t wp_num,uint32_t wp_crc)
{
    return _ttalink_uav_config_items_pack_chan(dst_addr, src_addr, chan, msg,  frame_type, voltage_protection, wp_num, wp_crc, false);
}

/**
 * @brief Pack a uav_config_items message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param frame_type  .
 * @param voltage_protection  .
 * @param wp_num  .
 * @param wp_crc  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_uav_config_items_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t frame_type,uint8_t voltage_protection,uint8_t wp_num,uint32_t wp_crc)
{
    return _ttalink_uav_config_items_pack_chan(dst_addr, src_addr, chan, msg,  frame_type, voltage_protection, wp_num, wp_crc, true);
}


static inline uint16_t _ttalink_uav_config_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_uav_config_items_t* uav_config_items, bool nocrc)
{
    if(nocrc){
        return ttalink_uav_config_items_pack_nocrc(dst_addr, src_addr, msg, uav_config_items->frame_type, uav_config_items->voltage_protection, uav_config_items->wp_num, uav_config_items->wp_crc);
    }else{
        return ttalink_uav_config_items_pack(dst_addr, src_addr, msg, uav_config_items->frame_type, uav_config_items->voltage_protection, uav_config_items->wp_num, uav_config_items->wp_crc);
    }
    
}

/**
 * @brief Encode a uav_config_items struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param uav_config_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_uav_config_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_uav_config_items_t* uav_config_items)
{
    return _ttalink_uav_config_items_encode(dst_addr, src_addr, msg, uav_config_items, false);
}

/**
 * @brief Encode a uav_config_items struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param uav_config_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_uav_config_items_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_uav_config_items_t* uav_config_items)
{
    return _ttalink_uav_config_items_encode(dst_addr, src_addr, msg, uav_config_items, true);
}


static inline uint16_t _ttalink_uav_config_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_uav_config_items_t* uav_config_items, bool nocrc)
{
    if(nocrc){
        return ttalink_uav_config_items_pack_chan_nocrc(dst_addr, src_addr, chan, msg, uav_config_items->frame_type, uav_config_items->voltage_protection, uav_config_items->wp_num, uav_config_items->wp_crc);
    }else{
        return ttalink_uav_config_items_pack_chan(dst_addr, src_addr, chan, msg, uav_config_items->frame_type, uav_config_items->voltage_protection, uav_config_items->wp_num, uav_config_items->wp_crc);
    }
}

/**
 * @brief Encode a uav_config_items struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param uav_config_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_uav_config_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_uav_config_items_t* uav_config_items)
{
    return _ttalink_uav_config_items_encode_chan(dst_addr, src_addr, chan, msg, uav_config_items, false);
}

/**
 * @brief Encode a uav_config_items struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param uav_config_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_uav_config_items_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_uav_config_items_t* uav_config_items)
{
    return _ttalink_uav_config_items_encode_chan(dst_addr, src_addr, chan, msg, uav_config_items, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_uav_config_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t frame_type, uint8_t voltage_protection, uint8_t wp_num, uint32_t wp_crc, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, frame_type);
    _tta_put_uint8_t(buf, 1, voltage_protection);
    _tta_put_uint8_t(buf, 2, wp_num);
    _tta_put_uint32_t(buf, 3, wp_crc);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UAV_CONFIG_ITEMS, buf, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_CRC, nocrc);
#else
    ttalink_uav_config_items_t packet;
    packet.frame_type = frame_type;
    packet.voltage_protection = voltage_protection;
    packet.wp_num = wp_num;
    packet.wp_crc = wp_crc;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UAV_CONFIG_ITEMS, (const char *)&packet, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a uav_config_items message
 * @param chan TTAlink channel to send the message
 *
 * @param frame_type  .
 * @param voltage_protection  .
 * @param wp_num  .
 * @param wp_crc  .
 */
static inline void ttalink_uav_config_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t frame_type, uint8_t voltage_protection, uint8_t wp_num, uint32_t wp_crc)
{
    _ttalink_uav_config_items_send(dst_addr, src_addr, chan, frame_type, voltage_protection, wp_num, wp_crc, false);
}

/**
 * @brief Send a uav_config_items message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param frame_type  .
 * @param voltage_protection  .
 * @param wp_num  .
 * @param wp_crc  .
 */
static inline void ttalink_uav_config_items_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t frame_type, uint8_t voltage_protection, uint8_t wp_num, uint32_t wp_crc)
{
    _ttalink_uav_config_items_send(dst_addr, src_addr, chan, frame_type, voltage_protection, wp_num, wp_crc, true);
}


static inline void _ttalink_uav_config_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_uav_config_items_t* uav_config_items, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_uav_config_items_send_nocrc(dst_addr, src_addr, chan, uav_config_items->frame_type, uav_config_items->voltage_protection, uav_config_items->wp_num, uav_config_items->wp_crc);
    }else{
        ttalink_uav_config_items_send(dst_addr, src_addr, chan, uav_config_items->frame_type, uav_config_items->voltage_protection, uav_config_items->wp_num, uav_config_items->wp_crc);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UAV_CONFIG_ITEMS, (const char *)uav_config_items, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a uav_config_items message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_uav_config_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_uav_config_items_t* uav_config_items)
{
    _ttalink_uav_config_items_send_struct(dst_addr, src_addr, chan, uav_config_items, false);
}

/**
 * @brief Send a uav_config_items message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_uav_config_items_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_uav_config_items_t* uav_config_items)
{
    _ttalink_uav_config_items_send_struct(dst_addr, src_addr, chan, uav_config_items, true);
}

#if TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_uav_config_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t frame_type, uint8_t voltage_protection, uint8_t wp_num, uint32_t wp_crc, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, frame_type);
    _tta_put_uint8_t(buf, 1, voltage_protection);
    _tta_put_uint8_t(buf, 2, wp_num);
    _tta_put_uint32_t(buf, 3, wp_crc);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UAV_CONFIG_ITEMS, buf, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_CRC, nocrc);
#else
    ttalink_uav_config_items_t *packet = (ttalink_uav_config_items_t *)msgbuf;
    packet->frame_type = frame_type;
    packet->voltage_protection = voltage_protection;
    packet->wp_num = wp_num;
    packet->wp_crc = wp_crc;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UAV_CONFIG_ITEMS, (const char *)packet, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_uav_config_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t frame_type, uint8_t voltage_protection, uint8_t wp_num, uint32_t wp_crc)
{
    _ttalink_uav_config_items_send_buf(dst_addr, src_addr, msgbuf, chan, frame_type, voltage_protection, wp_num, wp_crc, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_uav_config_items_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t frame_type, uint8_t voltage_protection, uint8_t wp_num, uint32_t wp_crc)
{
    _ttalink_uav_config_items_send_buf(dst_addr, src_addr, msgbuf, chan, frame_type, voltage_protection, wp_num, wp_crc, true);
}
#endif

#endif

// MESSAGE UAV_CONFIG_ITEMS UNPACKING


/**
 * @brief Get field frame_type from uav_config_items message
 *
 * @return  .
 */
static inline uint8_t ttalink_uav_config_items_get_frame_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field voltage_protection from uav_config_items message
 *
 * @return  .
 */
static inline uint8_t ttalink_uav_config_items_get_voltage_protection(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field wp_num from uav_config_items message
 *
 * @return  .
 */
static inline uint8_t ttalink_uav_config_items_get_wp_num(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field wp_crc from uav_config_items message
 *
 * @return  .
 */
static inline uint32_t ttalink_uav_config_items_get_wp_crc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  3);
}

/**
 * @brief Decode a uav_config_items message into a struct
 *
 * @param msg The message to decode
 * @param uav_config_items C-struct to decode the message contents into
 */
static inline void ttalink_uav_config_items_decode(const ttalink_message_t* msg, ttalink_uav_config_items_t* uav_config_items)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    uav_config_items->frame_type = ttalink_uav_config_items_get_frame_type(msg);
    uav_config_items->voltage_protection = ttalink_uav_config_items_get_voltage_protection(msg);
    uav_config_items->wp_num = ttalink_uav_config_items_get_wp_num(msg);
    uav_config_items->wp_crc = ttalink_uav_config_items_get_wp_crc(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN? msg->len : TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN;
        memset(uav_config_items, 0, TTALINK_MSG_ID_UAV_CONFIG_ITEMS_LEN);
    memcpy(uav_config_items, _TTA_PAYLOAD(msg), len);
#endif
}
