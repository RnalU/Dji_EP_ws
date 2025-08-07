#pragma once
// MESSAGE GCS_CONFIG_ITEMS PACKING

#define TTALINK_MSG_ID_GCS_CONFIG_ITEMS 2080

TTAPACKED(
typedef struct __ttalink_gcs_config_items_t {
 uint8_t altit_switch; /*<  .*/
}) ttalink_gcs_config_items_t;

#define TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN 1
#define TTALINK_MSG_ID_GCS_CONFIG_ITEMS_MIN_LEN 1
#define TTALINK_MSG_ID_2080_LEN 1
#define TTALINK_MSG_ID_2080_MIN_LEN 1

#define TTALINK_MSG_ID_GCS_CONFIG_ITEMS_CRC 110
#define TTALINK_MSG_ID_2080_CRC 110



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_GCS_CONFIG_ITEMS { \
    2080, \
    "GCS_CONFIG_ITEMS", \
    1, \
    {  { "altit_switch", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_gcs_config_items_t, altit_switch) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_GCS_CONFIG_ITEMS { \
    "GCS_CONFIG_ITEMS", \
    1, \
    {  { "altit_switch", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_gcs_config_items_t, altit_switch) }, \
         } \
}
#endif


static inline uint16_t _ttalink_gcs_config_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t altit_switch, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, altit_switch);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN);
#else
    ttalink_gcs_config_items_t packet;
    packet.altit_switch = altit_switch;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GCS_CONFIG_ITEMS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a gcs_config_items message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param altit_switch  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_gcs_config_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t altit_switch)
{
    return _ttalink_gcs_config_items_pack(dst_addr, src_addr, msg,  altit_switch, false);
}

/**
 * @brief Pack a gcs_config_items message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param altit_switch  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_gcs_config_items_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t altit_switch)
{
    return _ttalink_gcs_config_items_pack(dst_addr, src_addr, msg,  altit_switch, true);
}


static inline uint16_t _ttalink_gcs_config_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t altit_switch, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, altit_switch);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN);
#else
    ttalink_gcs_config_items_t packet;
    packet.altit_switch = altit_switch;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GCS_CONFIG_ITEMS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a gcs_config_items message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param altit_switch  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_gcs_config_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t altit_switch)
{
    return _ttalink_gcs_config_items_pack_chan(dst_addr, src_addr, chan, msg,  altit_switch, false);
}

/**
 * @brief Pack a gcs_config_items message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param altit_switch  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_gcs_config_items_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t altit_switch)
{
    return _ttalink_gcs_config_items_pack_chan(dst_addr, src_addr, chan, msg,  altit_switch, true);
}


static inline uint16_t _ttalink_gcs_config_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_gcs_config_items_t* gcs_config_items, bool nocrc)
{
    if(nocrc){
        return ttalink_gcs_config_items_pack_nocrc(dst_addr, src_addr, msg, gcs_config_items->altit_switch);
    }else{
        return ttalink_gcs_config_items_pack(dst_addr, src_addr, msg, gcs_config_items->altit_switch);
    }
    
}

/**
 * @brief Encode a gcs_config_items struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param gcs_config_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_gcs_config_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_gcs_config_items_t* gcs_config_items)
{
    return _ttalink_gcs_config_items_encode(dst_addr, src_addr, msg, gcs_config_items, false);
}

/**
 * @brief Encode a gcs_config_items struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param gcs_config_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_gcs_config_items_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_gcs_config_items_t* gcs_config_items)
{
    return _ttalink_gcs_config_items_encode(dst_addr, src_addr, msg, gcs_config_items, true);
}


static inline uint16_t _ttalink_gcs_config_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_gcs_config_items_t* gcs_config_items, bool nocrc)
{
    if(nocrc){
        return ttalink_gcs_config_items_pack_chan_nocrc(dst_addr, src_addr, chan, msg, gcs_config_items->altit_switch);
    }else{
        return ttalink_gcs_config_items_pack_chan(dst_addr, src_addr, chan, msg, gcs_config_items->altit_switch);
    }
}

/**
 * @brief Encode a gcs_config_items struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param gcs_config_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_gcs_config_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_gcs_config_items_t* gcs_config_items)
{
    return _ttalink_gcs_config_items_encode_chan(dst_addr, src_addr, chan, msg, gcs_config_items, false);
}

/**
 * @brief Encode a gcs_config_items struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param gcs_config_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_gcs_config_items_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_gcs_config_items_t* gcs_config_items)
{
    return _ttalink_gcs_config_items_encode_chan(dst_addr, src_addr, chan, msg, gcs_config_items, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_gcs_config_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t altit_switch, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, altit_switch);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GCS_CONFIG_ITEMS, buf, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_CRC, nocrc);
#else
    ttalink_gcs_config_items_t packet;
    packet.altit_switch = altit_switch;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GCS_CONFIG_ITEMS, (const char *)&packet, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a gcs_config_items message
 * @param chan TTAlink channel to send the message
 *
 * @param altit_switch  .
 */
static inline void ttalink_gcs_config_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t altit_switch)
{
    _ttalink_gcs_config_items_send(dst_addr, src_addr, chan, altit_switch, false);
}

/**
 * @brief Send a gcs_config_items message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param altit_switch  .
 */
static inline void ttalink_gcs_config_items_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t altit_switch)
{
    _ttalink_gcs_config_items_send(dst_addr, src_addr, chan, altit_switch, true);
}


static inline void _ttalink_gcs_config_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_gcs_config_items_t* gcs_config_items, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_gcs_config_items_send_nocrc(dst_addr, src_addr, chan, gcs_config_items->altit_switch);
    }else{
        ttalink_gcs_config_items_send(dst_addr, src_addr, chan, gcs_config_items->altit_switch);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GCS_CONFIG_ITEMS, (const char *)gcs_config_items, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a gcs_config_items message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_gcs_config_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_gcs_config_items_t* gcs_config_items)
{
    _ttalink_gcs_config_items_send_struct(dst_addr, src_addr, chan, gcs_config_items, false);
}

/**
 * @brief Send a gcs_config_items message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_gcs_config_items_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_gcs_config_items_t* gcs_config_items)
{
    _ttalink_gcs_config_items_send_struct(dst_addr, src_addr, chan, gcs_config_items, true);
}

#if TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_gcs_config_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t altit_switch, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, altit_switch);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GCS_CONFIG_ITEMS, buf, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_CRC, nocrc);
#else
    ttalink_gcs_config_items_t *packet = (ttalink_gcs_config_items_t *)msgbuf;
    packet->altit_switch = altit_switch;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GCS_CONFIG_ITEMS, (const char *)packet, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_gcs_config_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t altit_switch)
{
    _ttalink_gcs_config_items_send_buf(dst_addr, src_addr, msgbuf, chan, altit_switch, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_gcs_config_items_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t altit_switch)
{
    _ttalink_gcs_config_items_send_buf(dst_addr, src_addr, msgbuf, chan, altit_switch, true);
}
#endif

#endif

// MESSAGE GCS_CONFIG_ITEMS UNPACKING


/**
 * @brief Get field altit_switch from gcs_config_items message
 *
 * @return  .
 */
static inline uint8_t ttalink_gcs_config_items_get_altit_switch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a gcs_config_items message into a struct
 *
 * @param msg The message to decode
 * @param gcs_config_items C-struct to decode the message contents into
 */
static inline void ttalink_gcs_config_items_decode(const ttalink_message_t* msg, ttalink_gcs_config_items_t* gcs_config_items)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    gcs_config_items->altit_switch = ttalink_gcs_config_items_get_altit_switch(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN? msg->len : TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN;
        memset(gcs_config_items, 0, TTALINK_MSG_ID_GCS_CONFIG_ITEMS_LEN);
    memcpy(gcs_config_items, _TTA_PAYLOAD(msg), len);
#endif
}
