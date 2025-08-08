#pragma once
// MESSAGE POD_1534_UPDATE_PTZ_ERROR PACKING

#define TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR 11534

TTAPACKED(
typedef struct __ttalink_pod_1534_update_ptz_error_t {
 uint8_t ack; /*< .*/
 uint8_t ack_state1; /*< */
 uint8_t ack_state2; /*< */
 uint8_t ack_state3; /*< */
 uint8_t ack_state4; /*< */
}) ttalink_pod_1534_update_ptz_error_t;

#define TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN 5
#define TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_MIN_LEN 5
#define TTALINK_MSG_ID_11534_LEN 5
#define TTALINK_MSG_ID_11534_MIN_LEN 5

#define TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_CRC 153
#define TTALINK_MSG_ID_11534_CRC 153



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1534_UPDATE_PTZ_ERROR { \
    11534, \
    "POD_1534_UPDATE_PTZ_ERROR", \
    5, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1534_update_ptz_error_t, ack) }, \
         { "ack_state1", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1534_update_ptz_error_t, ack_state1) }, \
         { "ack_state2", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_pod_1534_update_ptz_error_t, ack_state2) }, \
         { "ack_state3", NULL, TTALINK_TYPE_UINT8_T, 0, 3, offsetof(ttalink_pod_1534_update_ptz_error_t, ack_state3) }, \
         { "ack_state4", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_pod_1534_update_ptz_error_t, ack_state4) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1534_UPDATE_PTZ_ERROR { \
    "POD_1534_UPDATE_PTZ_ERROR", \
    5, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1534_update_ptz_error_t, ack) }, \
         { "ack_state1", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1534_update_ptz_error_t, ack_state1) }, \
         { "ack_state2", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_pod_1534_update_ptz_error_t, ack_state2) }, \
         { "ack_state3", NULL, TTALINK_TYPE_UINT8_T, 0, 3, offsetof(ttalink_pod_1534_update_ptz_error_t, ack_state3) }, \
         { "ack_state4", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_pod_1534_update_ptz_error_t, ack_state4) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1534_update_ptz_error_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_state1, uint8_t ack_state2, uint8_t ack_state3, uint8_t ack_state4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_state1);
    _tta_put_uint8_t(buf, 2, ack_state2);
    _tta_put_uint8_t(buf, 3, ack_state3);
    _tta_put_uint8_t(buf, 4, ack_state4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN);
#else
    ttalink_pod_1534_update_ptz_error_t packet;
    packet.ack = ack;
    packet.ack_state1 = ack_state1;
    packet.ack_state2 = ack_state2;
    packet.ack_state3 = ack_state3;
    packet.ack_state4 = ack_state4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_MIN_LEN, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_CRC, nocrc);
}

/**
 * @brief Pack a pod_1534_update_ptz_error message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param ack_state1 
 * @param ack_state2 
 * @param ack_state3 
 * @param ack_state4 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1534_update_ptz_error_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_state1, uint8_t ack_state2, uint8_t ack_state3, uint8_t ack_state4)
{
    return _ttalink_pod_1534_update_ptz_error_pack(dst_addr, src_addr, msg,  ack, ack_state1, ack_state2, ack_state3, ack_state4, false);
}

/**
 * @brief Pack a pod_1534_update_ptz_error message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param ack_state1 
 * @param ack_state2 
 * @param ack_state3 
 * @param ack_state4 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1534_update_ptz_error_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_state1, uint8_t ack_state2, uint8_t ack_state3, uint8_t ack_state4)
{
    return _ttalink_pod_1534_update_ptz_error_pack(dst_addr, src_addr, msg,  ack, ack_state1, ack_state2, ack_state3, ack_state4, true);
}


static inline uint16_t _ttalink_pod_1534_update_ptz_error_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_state1,uint8_t ack_state2,uint8_t ack_state3,uint8_t ack_state4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_state1);
    _tta_put_uint8_t(buf, 2, ack_state2);
    _tta_put_uint8_t(buf, 3, ack_state3);
    _tta_put_uint8_t(buf, 4, ack_state4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN);
#else
    ttalink_pod_1534_update_ptz_error_t packet;
    packet.ack = ack;
    packet.ack_state1 = ack_state1;
    packet.ack_state2 = ack_state2;
    packet.ack_state3 = ack_state3;
    packet.ack_state4 = ack_state4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_MIN_LEN, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_CRC, nocrc);
}

/**
 * @brief Pack a pod_1534_update_ptz_error message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param ack_state1 
 * @param ack_state2 
 * @param ack_state3 
 * @param ack_state4 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1534_update_ptz_error_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_state1,uint8_t ack_state2,uint8_t ack_state3,uint8_t ack_state4)
{
    return _ttalink_pod_1534_update_ptz_error_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_state1, ack_state2, ack_state3, ack_state4, false);
}

/**
 * @brief Pack a pod_1534_update_ptz_error message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param ack_state1 
 * @param ack_state2 
 * @param ack_state3 
 * @param ack_state4 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1534_update_ptz_error_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_state1,uint8_t ack_state2,uint8_t ack_state3,uint8_t ack_state4)
{
    return _ttalink_pod_1534_update_ptz_error_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_state1, ack_state2, ack_state3, ack_state4, true);
}


static inline uint16_t _ttalink_pod_1534_update_ptz_error_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1534_update_ptz_error_t* pod_1534_update_ptz_error, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1534_update_ptz_error_pack_nocrc(dst_addr, src_addr, msg, pod_1534_update_ptz_error->ack, pod_1534_update_ptz_error->ack_state1, pod_1534_update_ptz_error->ack_state2, pod_1534_update_ptz_error->ack_state3, pod_1534_update_ptz_error->ack_state4);
    }else{
        return ttalink_pod_1534_update_ptz_error_pack(dst_addr, src_addr, msg, pod_1534_update_ptz_error->ack, pod_1534_update_ptz_error->ack_state1, pod_1534_update_ptz_error->ack_state2, pod_1534_update_ptz_error->ack_state3, pod_1534_update_ptz_error->ack_state4);
    }
    
}

/**
 * @brief Encode a pod_1534_update_ptz_error struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1534_update_ptz_error C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1534_update_ptz_error_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1534_update_ptz_error_t* pod_1534_update_ptz_error)
{
    return _ttalink_pod_1534_update_ptz_error_encode(dst_addr, src_addr, msg, pod_1534_update_ptz_error, false);
}

/**
 * @brief Encode a pod_1534_update_ptz_error struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1534_update_ptz_error C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1534_update_ptz_error_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1534_update_ptz_error_t* pod_1534_update_ptz_error)
{
    return _ttalink_pod_1534_update_ptz_error_encode(dst_addr, src_addr, msg, pod_1534_update_ptz_error, true);
}


static inline uint16_t _ttalink_pod_1534_update_ptz_error_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1534_update_ptz_error_t* pod_1534_update_ptz_error, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1534_update_ptz_error_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1534_update_ptz_error->ack, pod_1534_update_ptz_error->ack_state1, pod_1534_update_ptz_error->ack_state2, pod_1534_update_ptz_error->ack_state3, pod_1534_update_ptz_error->ack_state4);
    }else{
        return ttalink_pod_1534_update_ptz_error_pack_chan(dst_addr, src_addr, chan, msg, pod_1534_update_ptz_error->ack, pod_1534_update_ptz_error->ack_state1, pod_1534_update_ptz_error->ack_state2, pod_1534_update_ptz_error->ack_state3, pod_1534_update_ptz_error->ack_state4);
    }
}

/**
 * @brief Encode a pod_1534_update_ptz_error struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1534_update_ptz_error C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1534_update_ptz_error_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1534_update_ptz_error_t* pod_1534_update_ptz_error)
{
    return _ttalink_pod_1534_update_ptz_error_encode_chan(dst_addr, src_addr, chan, msg, pod_1534_update_ptz_error, false);
}

/**
 * @brief Encode a pod_1534_update_ptz_error struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1534_update_ptz_error C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1534_update_ptz_error_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1534_update_ptz_error_t* pod_1534_update_ptz_error)
{
    return _ttalink_pod_1534_update_ptz_error_encode_chan(dst_addr, src_addr, chan, msg, pod_1534_update_ptz_error, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1534_update_ptz_error_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_state1, uint8_t ack_state2, uint8_t ack_state3, uint8_t ack_state4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_state1);
    _tta_put_uint8_t(buf, 2, ack_state2);
    _tta_put_uint8_t(buf, 3, ack_state3);
    _tta_put_uint8_t(buf, 4, ack_state4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR, buf, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_MIN_LEN, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_CRC, nocrc);
#else
    ttalink_pod_1534_update_ptz_error_t packet;
    packet.ack = ack;
    packet.ack_state1 = ack_state1;
    packet.ack_state2 = ack_state2;
    packet.ack_state3 = ack_state3;
    packet.ack_state4 = ack_state4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR, (const char *)&packet, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_MIN_LEN, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1534_update_ptz_error message
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param ack_state1 
 * @param ack_state2 
 * @param ack_state3 
 * @param ack_state4 
 */
static inline void ttalink_pod_1534_update_ptz_error_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_state1, uint8_t ack_state2, uint8_t ack_state3, uint8_t ack_state4)
{
    _ttalink_pod_1534_update_ptz_error_send(dst_addr, src_addr, chan, ack, ack_state1, ack_state2, ack_state3, ack_state4, false);
}

/**
 * @brief Send a pod_1534_update_ptz_error message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param ack_state1 
 * @param ack_state2 
 * @param ack_state3 
 * @param ack_state4 
 */
static inline void ttalink_pod_1534_update_ptz_error_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_state1, uint8_t ack_state2, uint8_t ack_state3, uint8_t ack_state4)
{
    _ttalink_pod_1534_update_ptz_error_send(dst_addr, src_addr, chan, ack, ack_state1, ack_state2, ack_state3, ack_state4, true);
}


static inline void _ttalink_pod_1534_update_ptz_error_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1534_update_ptz_error_t* pod_1534_update_ptz_error, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1534_update_ptz_error_send_nocrc(dst_addr, src_addr, chan, pod_1534_update_ptz_error->ack, pod_1534_update_ptz_error->ack_state1, pod_1534_update_ptz_error->ack_state2, pod_1534_update_ptz_error->ack_state3, pod_1534_update_ptz_error->ack_state4);
    }else{
        ttalink_pod_1534_update_ptz_error_send(dst_addr, src_addr, chan, pod_1534_update_ptz_error->ack, pod_1534_update_ptz_error->ack_state1, pod_1534_update_ptz_error->ack_state2, pod_1534_update_ptz_error->ack_state3, pod_1534_update_ptz_error->ack_state4);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR, (const char *)pod_1534_update_ptz_error, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_MIN_LEN, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1534_update_ptz_error message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1534_update_ptz_error_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1534_update_ptz_error_t* pod_1534_update_ptz_error)
{
    _ttalink_pod_1534_update_ptz_error_send_struct(dst_addr, src_addr, chan, pod_1534_update_ptz_error, false);
}

/**
 * @brief Send a pod_1534_update_ptz_error message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1534_update_ptz_error_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1534_update_ptz_error_t* pod_1534_update_ptz_error)
{
    _ttalink_pod_1534_update_ptz_error_send_struct(dst_addr, src_addr, chan, pod_1534_update_ptz_error, true);
}

#if TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1534_update_ptz_error_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_state1, uint8_t ack_state2, uint8_t ack_state3, uint8_t ack_state4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_state1);
    _tta_put_uint8_t(buf, 2, ack_state2);
    _tta_put_uint8_t(buf, 3, ack_state3);
    _tta_put_uint8_t(buf, 4, ack_state4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR, buf, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_MIN_LEN, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_CRC, nocrc);
#else
    ttalink_pod_1534_update_ptz_error_t *packet = (ttalink_pod_1534_update_ptz_error_t *)msgbuf;
    packet->ack = ack;
    packet->ack_state1 = ack_state1;
    packet->ack_state2 = ack_state2;
    packet->ack_state3 = ack_state3;
    packet->ack_state4 = ack_state4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR, (const char *)packet, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_MIN_LEN, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1534_update_ptz_error_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_state1, uint8_t ack_state2, uint8_t ack_state3, uint8_t ack_state4)
{
    _ttalink_pod_1534_update_ptz_error_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_state1, ack_state2, ack_state3, ack_state4, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1534_update_ptz_error_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_state1, uint8_t ack_state2, uint8_t ack_state3, uint8_t ack_state4)
{
    _ttalink_pod_1534_update_ptz_error_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_state1, ack_state2, ack_state3, ack_state4, true);
}
#endif

#endif

// MESSAGE POD_1534_UPDATE_PTZ_ERROR UNPACKING


/**
 * @brief Get field ack from pod_1534_update_ptz_error message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1534_update_ptz_error_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack_state1 from pod_1534_update_ptz_error message
 *
 * @return 
 */
static inline uint8_t ttalink_pod_1534_update_ptz_error_get_ack_state1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field ack_state2 from pod_1534_update_ptz_error message
 *
 * @return 
 */
static inline uint8_t ttalink_pod_1534_update_ptz_error_get_ack_state2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field ack_state3 from pod_1534_update_ptz_error message
 *
 * @return 
 */
static inline uint8_t ttalink_pod_1534_update_ptz_error_get_ack_state3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field ack_state4 from pod_1534_update_ptz_error message
 *
 * @return 
 */
static inline uint8_t ttalink_pod_1534_update_ptz_error_get_ack_state4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Decode a pod_1534_update_ptz_error message into a struct
 *
 * @param msg The message to decode
 * @param pod_1534_update_ptz_error C-struct to decode the message contents into
 */
static inline void ttalink_pod_1534_update_ptz_error_decode(const ttalink_message_t* msg, ttalink_pod_1534_update_ptz_error_t* pod_1534_update_ptz_error)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1534_update_ptz_error->ack = ttalink_pod_1534_update_ptz_error_get_ack(msg);
    pod_1534_update_ptz_error->ack_state1 = ttalink_pod_1534_update_ptz_error_get_ack_state1(msg);
    pod_1534_update_ptz_error->ack_state2 = ttalink_pod_1534_update_ptz_error_get_ack_state2(msg);
    pod_1534_update_ptz_error->ack_state3 = ttalink_pod_1534_update_ptz_error_get_ack_state3(msg);
    pod_1534_update_ptz_error->ack_state4 = ttalink_pod_1534_update_ptz_error_get_ack_state4(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN? msg->len : TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN;
        memset(pod_1534_update_ptz_error, 0, TTALINK_MSG_ID_POD_1534_UPDATE_PTZ_ERROR_LEN);
    memcpy(pod_1534_update_ptz_error, _TTA_PAYLOAD(msg), len);
#endif
}
