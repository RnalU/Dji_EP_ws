#pragma once
// MESSAGE FOC_SET_PRIVATE_ADDR PACKING

#define TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR 2012

TTAPACKED(
typedef struct __ttalink_foc_set_private_addr_t {
 uint32_t update_time; /*< system time*/
 uint32_t addr; /*< new address*/
}) ttalink_foc_set_private_addr_t;

#define TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN 8
#define TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_MIN_LEN 8
#define TTALINK_MSG_ID_2012_LEN 8
#define TTALINK_MSG_ID_2012_MIN_LEN 8

#define TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_CRC 233
#define TTALINK_MSG_ID_2012_CRC 233



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_FOC_SET_PRIVATE_ADDR { \
    2012, \
    "FOC_SET_PRIVATE_ADDR", \
    2, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_foc_set_private_addr_t, update_time) }, \
         { "addr", NULL, TTALINK_TYPE_UINT32_T, 0, 4, offsetof(ttalink_foc_set_private_addr_t, addr) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_FOC_SET_PRIVATE_ADDR { \
    "FOC_SET_PRIVATE_ADDR", \
    2, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_foc_set_private_addr_t, update_time) }, \
         { "addr", NULL, TTALINK_TYPE_UINT32_T, 0, 4, offsetof(ttalink_foc_set_private_addr_t, addr) }, \
         } \
}
#endif


static inline uint16_t _ttalink_foc_set_private_addr_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t addr, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, addr);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN);
#else
    ttalink_foc_set_private_addr_t packet;
    packet.update_time = update_time;
    packet.addr = addr;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_MIN_LEN, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_CRC, nocrc);
}

/**
 * @brief Pack a foc_set_private_addr message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time system time
 * @param addr new address
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_set_private_addr_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t addr)
{
    return _ttalink_foc_set_private_addr_pack(dst_addr, src_addr, msg,  update_time, addr, false);
}

/**
 * @brief Pack a foc_set_private_addr message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time system time
 * @param addr new address
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_set_private_addr_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t addr)
{
    return _ttalink_foc_set_private_addr_pack(dst_addr, src_addr, msg,  update_time, addr, true);
}


static inline uint16_t _ttalink_foc_set_private_addr_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t addr, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, addr);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN);
#else
    ttalink_foc_set_private_addr_t packet;
    packet.update_time = update_time;
    packet.addr = addr;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_MIN_LEN, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_CRC, nocrc);
}

/**
 * @brief Pack a foc_set_private_addr message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time system time
 * @param addr new address
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_set_private_addr_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t addr)
{
    return _ttalink_foc_set_private_addr_pack_chan(dst_addr, src_addr, chan, msg,  update_time, addr, false);
}

/**
 * @brief Pack a foc_set_private_addr message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time system time
 * @param addr new address
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_set_private_addr_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t addr)
{
    return _ttalink_foc_set_private_addr_pack_chan(dst_addr, src_addr, chan, msg,  update_time, addr, true);
}


static inline uint16_t _ttalink_foc_set_private_addr_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_set_private_addr_t* foc_set_private_addr, bool nocrc)
{
    if(nocrc){
        return ttalink_foc_set_private_addr_pack_nocrc(dst_addr, src_addr, msg, foc_set_private_addr->update_time, foc_set_private_addr->addr);
    }else{
        return ttalink_foc_set_private_addr_pack(dst_addr, src_addr, msg, foc_set_private_addr->update_time, foc_set_private_addr->addr);
    }
    
}

/**
 * @brief Encode a foc_set_private_addr struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param foc_set_private_addr C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_set_private_addr_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_set_private_addr_t* foc_set_private_addr)
{
    return _ttalink_foc_set_private_addr_encode(dst_addr, src_addr, msg, foc_set_private_addr, false);
}

/**
 * @brief Encode a foc_set_private_addr struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param foc_set_private_addr C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_set_private_addr_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_set_private_addr_t* foc_set_private_addr)
{
    return _ttalink_foc_set_private_addr_encode(dst_addr, src_addr, msg, foc_set_private_addr, true);
}


static inline uint16_t _ttalink_foc_set_private_addr_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_set_private_addr_t* foc_set_private_addr, bool nocrc)
{
    if(nocrc){
        return ttalink_foc_set_private_addr_pack_chan_nocrc(dst_addr, src_addr, chan, msg, foc_set_private_addr->update_time, foc_set_private_addr->addr);
    }else{
        return ttalink_foc_set_private_addr_pack_chan(dst_addr, src_addr, chan, msg, foc_set_private_addr->update_time, foc_set_private_addr->addr);
    }
}

/**
 * @brief Encode a foc_set_private_addr struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param foc_set_private_addr C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_set_private_addr_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_set_private_addr_t* foc_set_private_addr)
{
    return _ttalink_foc_set_private_addr_encode_chan(dst_addr, src_addr, chan, msg, foc_set_private_addr, false);
}

/**
 * @brief Encode a foc_set_private_addr struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param foc_set_private_addr C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_set_private_addr_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_set_private_addr_t* foc_set_private_addr)
{
    return _ttalink_foc_set_private_addr_encode_chan(dst_addr, src_addr, chan, msg, foc_set_private_addr, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_foc_set_private_addr_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t addr, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, addr);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR, buf, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_MIN_LEN, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_CRC, nocrc);
#else
    ttalink_foc_set_private_addr_t packet;
    packet.update_time = update_time;
    packet.addr = addr;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR, (const char *)&packet, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_MIN_LEN, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_CRC, nocrc);
#endif
}

/**
 * @brief Send a foc_set_private_addr message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time system time
 * @param addr new address
 */
static inline void ttalink_foc_set_private_addr_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t addr)
{
    _ttalink_foc_set_private_addr_send(dst_addr, src_addr, chan, update_time, addr, false);
}

/**
 * @brief Send a foc_set_private_addr message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time system time
 * @param addr new address
 */
static inline void ttalink_foc_set_private_addr_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t addr)
{
    _ttalink_foc_set_private_addr_send(dst_addr, src_addr, chan, update_time, addr, true);
}


static inline void _ttalink_foc_set_private_addr_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_set_private_addr_t* foc_set_private_addr, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_foc_set_private_addr_send_nocrc(dst_addr, src_addr, chan, foc_set_private_addr->update_time, foc_set_private_addr->addr);
    }else{
        ttalink_foc_set_private_addr_send(dst_addr, src_addr, chan, foc_set_private_addr->update_time, foc_set_private_addr->addr);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR, (const char *)foc_set_private_addr, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_MIN_LEN, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_CRC, nocrc);
#endif
}

/**
 * @brief Send a foc_set_private_addr message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_foc_set_private_addr_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_set_private_addr_t* foc_set_private_addr)
{
    _ttalink_foc_set_private_addr_send_struct(dst_addr, src_addr, chan, foc_set_private_addr, false);
}

/**
 * @brief Send a foc_set_private_addr message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_foc_set_private_addr_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_set_private_addr_t* foc_set_private_addr)
{
    _ttalink_foc_set_private_addr_send_struct(dst_addr, src_addr, chan, foc_set_private_addr, true);
}

#if TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_foc_set_private_addr_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t addr, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, addr);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR, buf, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_MIN_LEN, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_CRC, nocrc);
#else
    ttalink_foc_set_private_addr_t *packet = (ttalink_foc_set_private_addr_t *)msgbuf;
    packet->update_time = update_time;
    packet->addr = addr;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR, (const char *)packet, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_MIN_LEN, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_foc_set_private_addr_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t addr)
{
    _ttalink_foc_set_private_addr_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, addr, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_foc_set_private_addr_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t addr)
{
    _ttalink_foc_set_private_addr_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, addr, true);
}
#endif

#endif

// MESSAGE FOC_SET_PRIVATE_ADDR UNPACKING


/**
 * @brief Get field update_time from foc_set_private_addr message
 *
 * @return system time
 */
static inline uint32_t ttalink_foc_set_private_addr_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field addr from foc_set_private_addr message
 *
 * @return new address
 */
static inline uint32_t ttalink_foc_set_private_addr_get_addr(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Decode a foc_set_private_addr message into a struct
 *
 * @param msg The message to decode
 * @param foc_set_private_addr C-struct to decode the message contents into
 */
static inline void ttalink_foc_set_private_addr_decode(const ttalink_message_t* msg, ttalink_foc_set_private_addr_t* foc_set_private_addr)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    foc_set_private_addr->update_time = ttalink_foc_set_private_addr_get_update_time(msg);
    foc_set_private_addr->addr = ttalink_foc_set_private_addr_get_addr(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN? msg->len : TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN;
        memset(foc_set_private_addr, 0, TTALINK_MSG_ID_FOC_SET_PRIVATE_ADDR_LEN);
    memcpy(foc_set_private_addr, _TTA_PAYLOAD(msg), len);
#endif
}
