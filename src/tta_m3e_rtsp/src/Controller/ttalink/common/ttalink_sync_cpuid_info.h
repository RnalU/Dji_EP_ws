#pragma once
// MESSAGE SYNC_CPUID_INFO PACKING

#define TTALINK_MSG_ID_SYNC_CPUID_INFO 2100

TTAPACKED(
typedef struct __ttalink_sync_cpuid_info_t {
 uint32_t UID[3]; /*< unique identification.*/
 uint32_t id; /*< thumbnail code.*/
}) ttalink_sync_cpuid_info_t;

#define TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN 16
#define TTALINK_MSG_ID_SYNC_CPUID_INFO_MIN_LEN 16
#define TTALINK_MSG_ID_2100_LEN 16
#define TTALINK_MSG_ID_2100_MIN_LEN 16

#define TTALINK_MSG_ID_SYNC_CPUID_INFO_CRC 81
#define TTALINK_MSG_ID_2100_CRC 81

#define TTALINK_MSG_SYNC_CPUID_INFO_FIELD_UID_LEN 3

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SYNC_CPUID_INFO { \
    2100, \
    "SYNC_CPUID_INFO", \
    2, \
    {  { "UID", NULL, TTALINK_TYPE_UINT32_T, 3, 0, offsetof(ttalink_sync_cpuid_info_t, UID) }, \
         { "id", NULL, TTALINK_TYPE_UINT32_T, 0, 12, offsetof(ttalink_sync_cpuid_info_t, id) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SYNC_CPUID_INFO { \
    "SYNC_CPUID_INFO", \
    2, \
    {  { "UID", NULL, TTALINK_TYPE_UINT32_T, 3, 0, offsetof(ttalink_sync_cpuid_info_t, UID) }, \
         { "id", NULL, TTALINK_TYPE_UINT32_T, 0, 12, offsetof(ttalink_sync_cpuid_info_t, id) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sync_cpuid_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint32_t *UID, uint32_t id, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN];
    _tta_put_uint32_t(buf, 12, id);
    _tta_put_uint32_t_array(buf, 0, UID, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN);
#else
    ttalink_sync_cpuid_info_t packet;
    packet.id = id;
    tta_array_memcpy(packet.UID, UID, sizeof(uint32_t)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_CPUID_INFO;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SYNC_CPUID_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN, TTALINK_MSG_ID_SYNC_CPUID_INFO_CRC, nocrc);
}

/**
 * @brief Pack a sync_cpuid_info message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param UID unique identification.
 * @param id thumbnail code.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cpuid_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint32_t *UID, uint32_t id)
{
    return _ttalink_sync_cpuid_info_pack(dst_addr, src_addr, msg,  UID, id, false);
}

/**
 * @brief Pack a sync_cpuid_info message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param UID unique identification.
 * @param id thumbnail code.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cpuid_info_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint32_t *UID, uint32_t id)
{
    return _ttalink_sync_cpuid_info_pack(dst_addr, src_addr, msg,  UID, id, true);
}


static inline uint16_t _ttalink_sync_cpuid_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint32_t *UID,uint32_t id, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN];
    _tta_put_uint32_t(buf, 12, id);
    _tta_put_uint32_t_array(buf, 0, UID, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN);
#else
    ttalink_sync_cpuid_info_t packet;
    packet.id = id;
    tta_array_memcpy(packet.UID, UID, sizeof(uint32_t)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_CPUID_INFO;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CPUID_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN, TTALINK_MSG_ID_SYNC_CPUID_INFO_CRC, nocrc);
}

/**
 * @brief Pack a sync_cpuid_info message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param UID unique identification.
 * @param id thumbnail code.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cpuid_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint32_t *UID,uint32_t id)
{
    return _ttalink_sync_cpuid_info_pack_chan(dst_addr, src_addr, chan, msg,  UID, id, false);
}

/**
 * @brief Pack a sync_cpuid_info message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param UID unique identification.
 * @param id thumbnail code.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cpuid_info_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint32_t *UID,uint32_t id)
{
    return _ttalink_sync_cpuid_info_pack_chan(dst_addr, src_addr, chan, msg,  UID, id, true);
}


static inline uint16_t _ttalink_sync_cpuid_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_cpuid_info_t* sync_cpuid_info, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_cpuid_info_pack_nocrc(dst_addr, src_addr, msg, sync_cpuid_info->UID, sync_cpuid_info->id);
    }else{
        return ttalink_sync_cpuid_info_pack(dst_addr, src_addr, msg, sync_cpuid_info->UID, sync_cpuid_info->id);
    }
    
}

/**
 * @brief Encode a sync_cpuid_info struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_cpuid_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cpuid_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_cpuid_info_t* sync_cpuid_info)
{
    return _ttalink_sync_cpuid_info_encode(dst_addr, src_addr, msg, sync_cpuid_info, false);
}

/**
 * @brief Encode a sync_cpuid_info struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_cpuid_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cpuid_info_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_cpuid_info_t* sync_cpuid_info)
{
    return _ttalink_sync_cpuid_info_encode(dst_addr, src_addr, msg, sync_cpuid_info, true);
}


static inline uint16_t _ttalink_sync_cpuid_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_cpuid_info_t* sync_cpuid_info, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_cpuid_info_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sync_cpuid_info->UID, sync_cpuid_info->id);
    }else{
        return ttalink_sync_cpuid_info_pack_chan(dst_addr, src_addr, chan, msg, sync_cpuid_info->UID, sync_cpuid_info->id);
    }
}

/**
 * @brief Encode a sync_cpuid_info struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_cpuid_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cpuid_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_cpuid_info_t* sync_cpuid_info)
{
    return _ttalink_sync_cpuid_info_encode_chan(dst_addr, src_addr, chan, msg, sync_cpuid_info, false);
}

/**
 * @brief Encode a sync_cpuid_info struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_cpuid_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cpuid_info_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_cpuid_info_t* sync_cpuid_info)
{
    return _ttalink_sync_cpuid_info_encode_chan(dst_addr, src_addr, chan, msg, sync_cpuid_info, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sync_cpuid_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint32_t *UID, uint32_t id, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN];
    _tta_put_uint32_t(buf, 12, id);
    _tta_put_uint32_t_array(buf, 0, UID, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CPUID_INFO, buf, TTALINK_MSG_ID_SYNC_CPUID_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN, TTALINK_MSG_ID_SYNC_CPUID_INFO_CRC, nocrc);
#else
    ttalink_sync_cpuid_info_t packet;
    packet.id = id;
    tta_array_memcpy(packet.UID, UID, sizeof(uint32_t)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CPUID_INFO, (const char *)&packet, TTALINK_MSG_ID_SYNC_CPUID_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN, TTALINK_MSG_ID_SYNC_CPUID_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_cpuid_info message
 * @param chan TTAlink channel to send the message
 *
 * @param UID unique identification.
 * @param id thumbnail code.
 */
static inline void ttalink_sync_cpuid_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint32_t *UID, uint32_t id)
{
    _ttalink_sync_cpuid_info_send(dst_addr, src_addr, chan, UID, id, false);
}

/**
 * @brief Send a sync_cpuid_info message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param UID unique identification.
 * @param id thumbnail code.
 */
static inline void ttalink_sync_cpuid_info_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint32_t *UID, uint32_t id)
{
    _ttalink_sync_cpuid_info_send(dst_addr, src_addr, chan, UID, id, true);
}


static inline void _ttalink_sync_cpuid_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_cpuid_info_t* sync_cpuid_info, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sync_cpuid_info_send_nocrc(dst_addr, src_addr, chan, sync_cpuid_info->UID, sync_cpuid_info->id);
    }else{
        ttalink_sync_cpuid_info_send(dst_addr, src_addr, chan, sync_cpuid_info->UID, sync_cpuid_info->id);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CPUID_INFO, (const char *)sync_cpuid_info, TTALINK_MSG_ID_SYNC_CPUID_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN, TTALINK_MSG_ID_SYNC_CPUID_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_cpuid_info message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_cpuid_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_cpuid_info_t* sync_cpuid_info)
{
    _ttalink_sync_cpuid_info_send_struct(dst_addr, src_addr, chan, sync_cpuid_info, false);
}

/**
 * @brief Send a sync_cpuid_info message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_cpuid_info_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_cpuid_info_t* sync_cpuid_info)
{
    _ttalink_sync_cpuid_info_send_struct(dst_addr, src_addr, chan, sync_cpuid_info, true);
}

#if TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sync_cpuid_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint32_t *UID, uint32_t id, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 12, id);
    _tta_put_uint32_t_array(buf, 0, UID, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CPUID_INFO, buf, TTALINK_MSG_ID_SYNC_CPUID_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN, TTALINK_MSG_ID_SYNC_CPUID_INFO_CRC, nocrc);
#else
    ttalink_sync_cpuid_info_t *packet = (ttalink_sync_cpuid_info_t *)msgbuf;
    packet->id = id;
    tta_array_memcpy(packet->UID, UID, sizeof(uint32_t)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CPUID_INFO, (const char *)packet, TTALINK_MSG_ID_SYNC_CPUID_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN, TTALINK_MSG_ID_SYNC_CPUID_INFO_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sync_cpuid_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint32_t *UID, uint32_t id)
{
    _ttalink_sync_cpuid_info_send_buf(dst_addr, src_addr, msgbuf, chan, UID, id, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sync_cpuid_info_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint32_t *UID, uint32_t id)
{
    _ttalink_sync_cpuid_info_send_buf(dst_addr, src_addr, msgbuf, chan, UID, id, true);
}
#endif

#endif

// MESSAGE SYNC_CPUID_INFO UNPACKING


/**
 * @brief Get field UID from sync_cpuid_info message
 *
 * @return unique identification.
 */
static inline uint16_t ttalink_sync_cpuid_info_get_UID(const ttalink_message_t* msg, uint32_t *UID)
{
    return _TTA_RETURN_uint32_t_array(msg, UID, 3,  0);
}

/**
 * @brief Get field id from sync_cpuid_info message
 *
 * @return thumbnail code.
 */
static inline uint32_t ttalink_sync_cpuid_info_get_id(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Decode a sync_cpuid_info message into a struct
 *
 * @param msg The message to decode
 * @param sync_cpuid_info C-struct to decode the message contents into
 */
static inline void ttalink_sync_cpuid_info_decode(const ttalink_message_t* msg, ttalink_sync_cpuid_info_t* sync_cpuid_info)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_sync_cpuid_info_get_UID(msg, sync_cpuid_info->UID);
    sync_cpuid_info->id = ttalink_sync_cpuid_info_get_id(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN? msg->len : TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN;
        memset(sync_cpuid_info, 0, TTALINK_MSG_ID_SYNC_CPUID_INFO_LEN);
    memcpy(sync_cpuid_info, _TTA_PAYLOAD(msg), len);
#endif
}
