#pragma once
// MESSAGE FOC_VERSION_INFOMATION_ACK PACKING

#define TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK 2019

TTAPACKED(
typedef struct __ttalink_foc_version_infomation_ack_t {
 uint32_t update_time; /*< system time*/
 uint8_t hardware_infomation[10]; /*< hardware version*/
 uint8_t software_infomation[10]; /*< software version*/
}) ttalink_foc_version_infomation_ack_t;

#define TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN 24
#define TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_MIN_LEN 24
#define TTALINK_MSG_ID_2019_LEN 24
#define TTALINK_MSG_ID_2019_MIN_LEN 24

#define TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_CRC 109
#define TTALINK_MSG_ID_2019_CRC 109

#define TTALINK_MSG_FOC_VERSION_INFOMATION_ACK_FIELD_HARDWARE_INFOMATION_LEN 10
#define TTALINK_MSG_FOC_VERSION_INFOMATION_ACK_FIELD_SOFTWARE_INFOMATION_LEN 10

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_FOC_VERSION_INFOMATION_ACK { \
    2019, \
    "FOC_VERSION_INFOMATION_ACK", \
    3, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_foc_version_infomation_ack_t, update_time) }, \
         { "hardware_infomation", NULL, TTALINK_TYPE_UINT8_T, 10, 4, offsetof(ttalink_foc_version_infomation_ack_t, hardware_infomation) }, \
         { "software_infomation", NULL, TTALINK_TYPE_UINT8_T, 10, 14, offsetof(ttalink_foc_version_infomation_ack_t, software_infomation) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_FOC_VERSION_INFOMATION_ACK { \
    "FOC_VERSION_INFOMATION_ACK", \
    3, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_foc_version_infomation_ack_t, update_time) }, \
         { "hardware_infomation", NULL, TTALINK_TYPE_UINT8_T, 10, 4, offsetof(ttalink_foc_version_infomation_ack_t, hardware_infomation) }, \
         { "software_infomation", NULL, TTALINK_TYPE_UINT8_T, 10, 14, offsetof(ttalink_foc_version_infomation_ack_t, software_infomation) }, \
         } \
}
#endif


static inline uint16_t _ttalink_foc_version_infomation_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, const uint8_t *hardware_infomation, const uint8_t *software_infomation, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t_array(buf, 4, hardware_infomation, 10);
    _tta_put_uint8_t_array(buf, 14, software_infomation, 10);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN);
#else
    ttalink_foc_version_infomation_ack_t packet;
    packet.update_time = update_time;
    tta_array_memcpy(packet.hardware_infomation, hardware_infomation, sizeof(uint8_t)*10);
    tta_array_memcpy(packet.software_infomation, software_infomation, sizeof(uint8_t)*10);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_CRC, nocrc);
}

/**
 * @brief Pack a foc_version_infomation_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time system time
 * @param hardware_infomation hardware version
 * @param software_infomation software version
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_version_infomation_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, const uint8_t *hardware_infomation, const uint8_t *software_infomation)
{
    return _ttalink_foc_version_infomation_ack_pack(dst_addr, src_addr, msg,  update_time, hardware_infomation, software_infomation, false);
}

/**
 * @brief Pack a foc_version_infomation_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time system time
 * @param hardware_infomation hardware version
 * @param software_infomation software version
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_version_infomation_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, const uint8_t *hardware_infomation, const uint8_t *software_infomation)
{
    return _ttalink_foc_version_infomation_ack_pack(dst_addr, src_addr, msg,  update_time, hardware_infomation, software_infomation, true);
}


static inline uint16_t _ttalink_foc_version_infomation_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,const uint8_t *hardware_infomation,const uint8_t *software_infomation, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t_array(buf, 4, hardware_infomation, 10);
    _tta_put_uint8_t_array(buf, 14, software_infomation, 10);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN);
#else
    ttalink_foc_version_infomation_ack_t packet;
    packet.update_time = update_time;
    tta_array_memcpy(packet.hardware_infomation, hardware_infomation, sizeof(uint8_t)*10);
    tta_array_memcpy(packet.software_infomation, software_infomation, sizeof(uint8_t)*10);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_CRC, nocrc);
}

/**
 * @brief Pack a foc_version_infomation_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time system time
 * @param hardware_infomation hardware version
 * @param software_infomation software version
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_version_infomation_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,const uint8_t *hardware_infomation,const uint8_t *software_infomation)
{
    return _ttalink_foc_version_infomation_ack_pack_chan(dst_addr, src_addr, chan, msg,  update_time, hardware_infomation, software_infomation, false);
}

/**
 * @brief Pack a foc_version_infomation_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time system time
 * @param hardware_infomation hardware version
 * @param software_infomation software version
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_version_infomation_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,const uint8_t *hardware_infomation,const uint8_t *software_infomation)
{
    return _ttalink_foc_version_infomation_ack_pack_chan(dst_addr, src_addr, chan, msg,  update_time, hardware_infomation, software_infomation, true);
}


static inline uint16_t _ttalink_foc_version_infomation_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_version_infomation_ack_t* foc_version_infomation_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_foc_version_infomation_ack_pack_nocrc(dst_addr, src_addr, msg, foc_version_infomation_ack->update_time, foc_version_infomation_ack->hardware_infomation, foc_version_infomation_ack->software_infomation);
    }else{
        return ttalink_foc_version_infomation_ack_pack(dst_addr, src_addr, msg, foc_version_infomation_ack->update_time, foc_version_infomation_ack->hardware_infomation, foc_version_infomation_ack->software_infomation);
    }
    
}

/**
 * @brief Encode a foc_version_infomation_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param foc_version_infomation_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_version_infomation_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_version_infomation_ack_t* foc_version_infomation_ack)
{
    return _ttalink_foc_version_infomation_ack_encode(dst_addr, src_addr, msg, foc_version_infomation_ack, false);
}

/**
 * @brief Encode a foc_version_infomation_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param foc_version_infomation_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_version_infomation_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_version_infomation_ack_t* foc_version_infomation_ack)
{
    return _ttalink_foc_version_infomation_ack_encode(dst_addr, src_addr, msg, foc_version_infomation_ack, true);
}


static inline uint16_t _ttalink_foc_version_infomation_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_version_infomation_ack_t* foc_version_infomation_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_foc_version_infomation_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, foc_version_infomation_ack->update_time, foc_version_infomation_ack->hardware_infomation, foc_version_infomation_ack->software_infomation);
    }else{
        return ttalink_foc_version_infomation_ack_pack_chan(dst_addr, src_addr, chan, msg, foc_version_infomation_ack->update_time, foc_version_infomation_ack->hardware_infomation, foc_version_infomation_ack->software_infomation);
    }
}

/**
 * @brief Encode a foc_version_infomation_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param foc_version_infomation_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_version_infomation_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_version_infomation_ack_t* foc_version_infomation_ack)
{
    return _ttalink_foc_version_infomation_ack_encode_chan(dst_addr, src_addr, chan, msg, foc_version_infomation_ack, false);
}

/**
 * @brief Encode a foc_version_infomation_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param foc_version_infomation_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_version_infomation_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_version_infomation_ack_t* foc_version_infomation_ack)
{
    return _ttalink_foc_version_infomation_ack_encode_chan(dst_addr, src_addr, chan, msg, foc_version_infomation_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_foc_version_infomation_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, const uint8_t *hardware_infomation, const uint8_t *software_infomation, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t_array(buf, 4, hardware_infomation, 10);
    _tta_put_uint8_t_array(buf, 14, software_infomation, 10);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK, buf, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_CRC, nocrc);
#else
    ttalink_foc_version_infomation_ack_t packet;
    packet.update_time = update_time;
    tta_array_memcpy(packet.hardware_infomation, hardware_infomation, sizeof(uint8_t)*10);
    tta_array_memcpy(packet.software_infomation, software_infomation, sizeof(uint8_t)*10);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK, (const char *)&packet, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a foc_version_infomation_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time system time
 * @param hardware_infomation hardware version
 * @param software_infomation software version
 */
static inline void ttalink_foc_version_infomation_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, const uint8_t *hardware_infomation, const uint8_t *software_infomation)
{
    _ttalink_foc_version_infomation_ack_send(dst_addr, src_addr, chan, update_time, hardware_infomation, software_infomation, false);
}

/**
 * @brief Send a foc_version_infomation_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time system time
 * @param hardware_infomation hardware version
 * @param software_infomation software version
 */
static inline void ttalink_foc_version_infomation_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, const uint8_t *hardware_infomation, const uint8_t *software_infomation)
{
    _ttalink_foc_version_infomation_ack_send(dst_addr, src_addr, chan, update_time, hardware_infomation, software_infomation, true);
}


static inline void _ttalink_foc_version_infomation_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_version_infomation_ack_t* foc_version_infomation_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_foc_version_infomation_ack_send_nocrc(dst_addr, src_addr, chan, foc_version_infomation_ack->update_time, foc_version_infomation_ack->hardware_infomation, foc_version_infomation_ack->software_infomation);
    }else{
        ttalink_foc_version_infomation_ack_send(dst_addr, src_addr, chan, foc_version_infomation_ack->update_time, foc_version_infomation_ack->hardware_infomation, foc_version_infomation_ack->software_infomation);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK, (const char *)foc_version_infomation_ack, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a foc_version_infomation_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_foc_version_infomation_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_version_infomation_ack_t* foc_version_infomation_ack)
{
    _ttalink_foc_version_infomation_ack_send_struct(dst_addr, src_addr, chan, foc_version_infomation_ack, false);
}

/**
 * @brief Send a foc_version_infomation_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_foc_version_infomation_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_version_infomation_ack_t* foc_version_infomation_ack)
{
    _ttalink_foc_version_infomation_ack_send_struct(dst_addr, src_addr, chan, foc_version_infomation_ack, true);
}

#if TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_foc_version_infomation_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, const uint8_t *hardware_infomation, const uint8_t *software_infomation, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t_array(buf, 4, hardware_infomation, 10);
    _tta_put_uint8_t_array(buf, 14, software_infomation, 10);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK, buf, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_CRC, nocrc);
#else
    ttalink_foc_version_infomation_ack_t *packet = (ttalink_foc_version_infomation_ack_t *)msgbuf;
    packet->update_time = update_time;
    tta_array_memcpy(packet->hardware_infomation, hardware_infomation, sizeof(uint8_t)*10);
    tta_array_memcpy(packet->software_infomation, software_infomation, sizeof(uint8_t)*10);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK, (const char *)packet, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_foc_version_infomation_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, const uint8_t *hardware_infomation, const uint8_t *software_infomation)
{
    _ttalink_foc_version_infomation_ack_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, hardware_infomation, software_infomation, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_foc_version_infomation_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, const uint8_t *hardware_infomation, const uint8_t *software_infomation)
{
    _ttalink_foc_version_infomation_ack_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, hardware_infomation, software_infomation, true);
}
#endif

#endif

// MESSAGE FOC_VERSION_INFOMATION_ACK UNPACKING


/**
 * @brief Get field update_time from foc_version_infomation_ack message
 *
 * @return system time
 */
static inline uint32_t ttalink_foc_version_infomation_ack_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field hardware_infomation from foc_version_infomation_ack message
 *
 * @return hardware version
 */
static inline uint16_t ttalink_foc_version_infomation_ack_get_hardware_infomation(const ttalink_message_t* msg, uint8_t *hardware_infomation)
{
    return _TTA_RETURN_uint8_t_array(msg, hardware_infomation, 10,  4);
}

/**
 * @brief Get field software_infomation from foc_version_infomation_ack message
 *
 * @return software version
 */
static inline uint16_t ttalink_foc_version_infomation_ack_get_software_infomation(const ttalink_message_t* msg, uint8_t *software_infomation)
{
    return _TTA_RETURN_uint8_t_array(msg, software_infomation, 10,  14);
}

/**
 * @brief Decode a foc_version_infomation_ack message into a struct
 *
 * @param msg The message to decode
 * @param foc_version_infomation_ack C-struct to decode the message contents into
 */
static inline void ttalink_foc_version_infomation_ack_decode(const ttalink_message_t* msg, ttalink_foc_version_infomation_ack_t* foc_version_infomation_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    foc_version_infomation_ack->update_time = ttalink_foc_version_infomation_ack_get_update_time(msg);
    ttalink_foc_version_infomation_ack_get_hardware_infomation(msg, foc_version_infomation_ack->hardware_infomation);
    ttalink_foc_version_infomation_ack_get_software_infomation(msg, foc_version_infomation_ack->software_infomation);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN? msg->len : TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN;
        memset(foc_version_infomation_ack, 0, TTALINK_MSG_ID_FOC_VERSION_INFOMATION_ACK_LEN);
    memcpy(foc_version_infomation_ack, _TTA_PAYLOAD(msg), len);
#endif
}
