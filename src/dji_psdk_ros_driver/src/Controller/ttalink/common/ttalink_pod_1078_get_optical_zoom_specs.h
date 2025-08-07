#pragma once
// MESSAGE POD_1078_GET_OPTICAL_ZOOM_SPECS PACKING

#define TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS 11078

TTAPACKED(
typedef struct __ttalink_pod_1078_get_optical_zoom_specs_t {
 uint8_t ack; /*< .*/
 uint16_t ack_zoom_max; /*<  The maximum focal length of the lens is 0.1mm..*/
 uint16_t ack_zoom_min; /*<  The minimum focal length of the lens is 0.1mm..*/
 uint16_t ack_zoom_interval; /*<  Minimum interval of focal length change..*/
}) ttalink_pod_1078_get_optical_zoom_specs_t;

#define TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN 7
#define TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_MIN_LEN 7
#define TTALINK_MSG_ID_11078_LEN 7
#define TTALINK_MSG_ID_11078_MIN_LEN 7

#define TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_CRC 171
#define TTALINK_MSG_ID_11078_CRC 171



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1078_GET_OPTICAL_ZOOM_SPECS { \
    11078, \
    "POD_1078_GET_OPTICAL_ZOOM_SPECS", \
    4, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1078_get_optical_zoom_specs_t, ack) }, \
         { "ack_zoom_max", NULL, TTALINK_TYPE_UINT16_T, 0, 1, offsetof(ttalink_pod_1078_get_optical_zoom_specs_t, ack_zoom_max) }, \
         { "ack_zoom_min", NULL, TTALINK_TYPE_UINT16_T, 0, 3, offsetof(ttalink_pod_1078_get_optical_zoom_specs_t, ack_zoom_min) }, \
         { "ack_zoom_interval", NULL, TTALINK_TYPE_UINT16_T, 0, 5, offsetof(ttalink_pod_1078_get_optical_zoom_specs_t, ack_zoom_interval) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1078_GET_OPTICAL_ZOOM_SPECS { \
    "POD_1078_GET_OPTICAL_ZOOM_SPECS", \
    4, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1078_get_optical_zoom_specs_t, ack) }, \
         { "ack_zoom_max", NULL, TTALINK_TYPE_UINT16_T, 0, 1, offsetof(ttalink_pod_1078_get_optical_zoom_specs_t, ack_zoom_max) }, \
         { "ack_zoom_min", NULL, TTALINK_TYPE_UINT16_T, 0, 3, offsetof(ttalink_pod_1078_get_optical_zoom_specs_t, ack_zoom_min) }, \
         { "ack_zoom_interval", NULL, TTALINK_TYPE_UINT16_T, 0, 5, offsetof(ttalink_pod_1078_get_optical_zoom_specs_t, ack_zoom_interval) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1078_get_optical_zoom_specs_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint16_t ack_zoom_max, uint16_t ack_zoom_min, uint16_t ack_zoom_interval, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint16_t(buf, 1, ack_zoom_max);
    _tta_put_uint16_t(buf, 3, ack_zoom_min);
    _tta_put_uint16_t(buf, 5, ack_zoom_interval);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN);
#else
    ttalink_pod_1078_get_optical_zoom_specs_t packet;
    packet.ack = ack;
    packet.ack_zoom_max = ack_zoom_max;
    packet.ack_zoom_min = ack_zoom_min;
    packet.ack_zoom_interval = ack_zoom_interval;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_MIN_LEN, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_CRC, nocrc);
}

/**
 * @brief Pack a pod_1078_get_optical_zoom_specs message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param ack_zoom_max  The maximum focal length of the lens is 0.1mm..
 * @param ack_zoom_min  The minimum focal length of the lens is 0.1mm..
 * @param ack_zoom_interval  Minimum interval of focal length change..
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1078_get_optical_zoom_specs_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint16_t ack_zoom_max, uint16_t ack_zoom_min, uint16_t ack_zoom_interval)
{
    return _ttalink_pod_1078_get_optical_zoom_specs_pack(dst_addr, src_addr, msg,  ack, ack_zoom_max, ack_zoom_min, ack_zoom_interval, false);
}

/**
 * @brief Pack a pod_1078_get_optical_zoom_specs message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param ack_zoom_max  The maximum focal length of the lens is 0.1mm..
 * @param ack_zoom_min  The minimum focal length of the lens is 0.1mm..
 * @param ack_zoom_interval  Minimum interval of focal length change..
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1078_get_optical_zoom_specs_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint16_t ack_zoom_max, uint16_t ack_zoom_min, uint16_t ack_zoom_interval)
{
    return _ttalink_pod_1078_get_optical_zoom_specs_pack(dst_addr, src_addr, msg,  ack, ack_zoom_max, ack_zoom_min, ack_zoom_interval, true);
}


static inline uint16_t _ttalink_pod_1078_get_optical_zoom_specs_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint16_t ack_zoom_max,uint16_t ack_zoom_min,uint16_t ack_zoom_interval, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint16_t(buf, 1, ack_zoom_max);
    _tta_put_uint16_t(buf, 3, ack_zoom_min);
    _tta_put_uint16_t(buf, 5, ack_zoom_interval);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN);
#else
    ttalink_pod_1078_get_optical_zoom_specs_t packet;
    packet.ack = ack;
    packet.ack_zoom_max = ack_zoom_max;
    packet.ack_zoom_min = ack_zoom_min;
    packet.ack_zoom_interval = ack_zoom_interval;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_MIN_LEN, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_CRC, nocrc);
}

/**
 * @brief Pack a pod_1078_get_optical_zoom_specs message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param ack_zoom_max  The maximum focal length of the lens is 0.1mm..
 * @param ack_zoom_min  The minimum focal length of the lens is 0.1mm..
 * @param ack_zoom_interval  Minimum interval of focal length change..
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1078_get_optical_zoom_specs_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint16_t ack_zoom_max,uint16_t ack_zoom_min,uint16_t ack_zoom_interval)
{
    return _ttalink_pod_1078_get_optical_zoom_specs_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_zoom_max, ack_zoom_min, ack_zoom_interval, false);
}

/**
 * @brief Pack a pod_1078_get_optical_zoom_specs message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param ack_zoom_max  The maximum focal length of the lens is 0.1mm..
 * @param ack_zoom_min  The minimum focal length of the lens is 0.1mm..
 * @param ack_zoom_interval  Minimum interval of focal length change..
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1078_get_optical_zoom_specs_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint16_t ack_zoom_max,uint16_t ack_zoom_min,uint16_t ack_zoom_interval)
{
    return _ttalink_pod_1078_get_optical_zoom_specs_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_zoom_max, ack_zoom_min, ack_zoom_interval, true);
}


static inline uint16_t _ttalink_pod_1078_get_optical_zoom_specs_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1078_get_optical_zoom_specs_t* pod_1078_get_optical_zoom_specs, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1078_get_optical_zoom_specs_pack_nocrc(dst_addr, src_addr, msg, pod_1078_get_optical_zoom_specs->ack, pod_1078_get_optical_zoom_specs->ack_zoom_max, pod_1078_get_optical_zoom_specs->ack_zoom_min, pod_1078_get_optical_zoom_specs->ack_zoom_interval);
    }else{
        return ttalink_pod_1078_get_optical_zoom_specs_pack(dst_addr, src_addr, msg, pod_1078_get_optical_zoom_specs->ack, pod_1078_get_optical_zoom_specs->ack_zoom_max, pod_1078_get_optical_zoom_specs->ack_zoom_min, pod_1078_get_optical_zoom_specs->ack_zoom_interval);
    }
    
}

/**
 * @brief Encode a pod_1078_get_optical_zoom_specs struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1078_get_optical_zoom_specs C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1078_get_optical_zoom_specs_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1078_get_optical_zoom_specs_t* pod_1078_get_optical_zoom_specs)
{
    return _ttalink_pod_1078_get_optical_zoom_specs_encode(dst_addr, src_addr, msg, pod_1078_get_optical_zoom_specs, false);
}

/**
 * @brief Encode a pod_1078_get_optical_zoom_specs struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1078_get_optical_zoom_specs C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1078_get_optical_zoom_specs_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1078_get_optical_zoom_specs_t* pod_1078_get_optical_zoom_specs)
{
    return _ttalink_pod_1078_get_optical_zoom_specs_encode(dst_addr, src_addr, msg, pod_1078_get_optical_zoom_specs, true);
}


static inline uint16_t _ttalink_pod_1078_get_optical_zoom_specs_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1078_get_optical_zoom_specs_t* pod_1078_get_optical_zoom_specs, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1078_get_optical_zoom_specs_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1078_get_optical_zoom_specs->ack, pod_1078_get_optical_zoom_specs->ack_zoom_max, pod_1078_get_optical_zoom_specs->ack_zoom_min, pod_1078_get_optical_zoom_specs->ack_zoom_interval);
    }else{
        return ttalink_pod_1078_get_optical_zoom_specs_pack_chan(dst_addr, src_addr, chan, msg, pod_1078_get_optical_zoom_specs->ack, pod_1078_get_optical_zoom_specs->ack_zoom_max, pod_1078_get_optical_zoom_specs->ack_zoom_min, pod_1078_get_optical_zoom_specs->ack_zoom_interval);
    }
}

/**
 * @brief Encode a pod_1078_get_optical_zoom_specs struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1078_get_optical_zoom_specs C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1078_get_optical_zoom_specs_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1078_get_optical_zoom_specs_t* pod_1078_get_optical_zoom_specs)
{
    return _ttalink_pod_1078_get_optical_zoom_specs_encode_chan(dst_addr, src_addr, chan, msg, pod_1078_get_optical_zoom_specs, false);
}

/**
 * @brief Encode a pod_1078_get_optical_zoom_specs struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1078_get_optical_zoom_specs C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1078_get_optical_zoom_specs_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1078_get_optical_zoom_specs_t* pod_1078_get_optical_zoom_specs)
{
    return _ttalink_pod_1078_get_optical_zoom_specs_encode_chan(dst_addr, src_addr, chan, msg, pod_1078_get_optical_zoom_specs, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1078_get_optical_zoom_specs_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint16_t ack_zoom_max, uint16_t ack_zoom_min, uint16_t ack_zoom_interval, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint16_t(buf, 1, ack_zoom_max);
    _tta_put_uint16_t(buf, 3, ack_zoom_min);
    _tta_put_uint16_t(buf, 5, ack_zoom_interval);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS, buf, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_MIN_LEN, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_CRC, nocrc);
#else
    ttalink_pod_1078_get_optical_zoom_specs_t packet;
    packet.ack = ack;
    packet.ack_zoom_max = ack_zoom_max;
    packet.ack_zoom_min = ack_zoom_min;
    packet.ack_zoom_interval = ack_zoom_interval;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS, (const char *)&packet, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_MIN_LEN, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1078_get_optical_zoom_specs message
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param ack_zoom_max  The maximum focal length of the lens is 0.1mm..
 * @param ack_zoom_min  The minimum focal length of the lens is 0.1mm..
 * @param ack_zoom_interval  Minimum interval of focal length change..
 */
static inline void ttalink_pod_1078_get_optical_zoom_specs_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint16_t ack_zoom_max, uint16_t ack_zoom_min, uint16_t ack_zoom_interval)
{
    _ttalink_pod_1078_get_optical_zoom_specs_send(dst_addr, src_addr, chan, ack, ack_zoom_max, ack_zoom_min, ack_zoom_interval, false);
}

/**
 * @brief Send a pod_1078_get_optical_zoom_specs message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param ack_zoom_max  The maximum focal length of the lens is 0.1mm..
 * @param ack_zoom_min  The minimum focal length of the lens is 0.1mm..
 * @param ack_zoom_interval  Minimum interval of focal length change..
 */
static inline void ttalink_pod_1078_get_optical_zoom_specs_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint16_t ack_zoom_max, uint16_t ack_zoom_min, uint16_t ack_zoom_interval)
{
    _ttalink_pod_1078_get_optical_zoom_specs_send(dst_addr, src_addr, chan, ack, ack_zoom_max, ack_zoom_min, ack_zoom_interval, true);
}


static inline void _ttalink_pod_1078_get_optical_zoom_specs_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1078_get_optical_zoom_specs_t* pod_1078_get_optical_zoom_specs, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1078_get_optical_zoom_specs_send_nocrc(dst_addr, src_addr, chan, pod_1078_get_optical_zoom_specs->ack, pod_1078_get_optical_zoom_specs->ack_zoom_max, pod_1078_get_optical_zoom_specs->ack_zoom_min, pod_1078_get_optical_zoom_specs->ack_zoom_interval);
    }else{
        ttalink_pod_1078_get_optical_zoom_specs_send(dst_addr, src_addr, chan, pod_1078_get_optical_zoom_specs->ack, pod_1078_get_optical_zoom_specs->ack_zoom_max, pod_1078_get_optical_zoom_specs->ack_zoom_min, pod_1078_get_optical_zoom_specs->ack_zoom_interval);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS, (const char *)pod_1078_get_optical_zoom_specs, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_MIN_LEN, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1078_get_optical_zoom_specs message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1078_get_optical_zoom_specs_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1078_get_optical_zoom_specs_t* pod_1078_get_optical_zoom_specs)
{
    _ttalink_pod_1078_get_optical_zoom_specs_send_struct(dst_addr, src_addr, chan, pod_1078_get_optical_zoom_specs, false);
}

/**
 * @brief Send a pod_1078_get_optical_zoom_specs message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1078_get_optical_zoom_specs_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1078_get_optical_zoom_specs_t* pod_1078_get_optical_zoom_specs)
{
    _ttalink_pod_1078_get_optical_zoom_specs_send_struct(dst_addr, src_addr, chan, pod_1078_get_optical_zoom_specs, true);
}

#if TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1078_get_optical_zoom_specs_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint16_t ack_zoom_max, uint16_t ack_zoom_min, uint16_t ack_zoom_interval, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint16_t(buf, 1, ack_zoom_max);
    _tta_put_uint16_t(buf, 3, ack_zoom_min);
    _tta_put_uint16_t(buf, 5, ack_zoom_interval);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS, buf, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_MIN_LEN, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_CRC, nocrc);
#else
    ttalink_pod_1078_get_optical_zoom_specs_t *packet = (ttalink_pod_1078_get_optical_zoom_specs_t *)msgbuf;
    packet->ack = ack;
    packet->ack_zoom_max = ack_zoom_max;
    packet->ack_zoom_min = ack_zoom_min;
    packet->ack_zoom_interval = ack_zoom_interval;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS, (const char *)packet, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_MIN_LEN, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1078_get_optical_zoom_specs_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint16_t ack_zoom_max, uint16_t ack_zoom_min, uint16_t ack_zoom_interval)
{
    _ttalink_pod_1078_get_optical_zoom_specs_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_zoom_max, ack_zoom_min, ack_zoom_interval, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1078_get_optical_zoom_specs_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint16_t ack_zoom_max, uint16_t ack_zoom_min, uint16_t ack_zoom_interval)
{
    _ttalink_pod_1078_get_optical_zoom_specs_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_zoom_max, ack_zoom_min, ack_zoom_interval, true);
}
#endif

#endif

// MESSAGE POD_1078_GET_OPTICAL_ZOOM_SPECS UNPACKING


/**
 * @brief Get field ack from pod_1078_get_optical_zoom_specs message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1078_get_optical_zoom_specs_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack_zoom_max from pod_1078_get_optical_zoom_specs message
 *
 * @return  The maximum focal length of the lens is 0.1mm..
 */
static inline uint16_t ttalink_pod_1078_get_optical_zoom_specs_get_ack_zoom_max(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  1);
}

/**
 * @brief Get field ack_zoom_min from pod_1078_get_optical_zoom_specs message
 *
 * @return  The minimum focal length of the lens is 0.1mm..
 */
static inline uint16_t ttalink_pod_1078_get_optical_zoom_specs_get_ack_zoom_min(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  3);
}

/**
 * @brief Get field ack_zoom_interval from pod_1078_get_optical_zoom_specs message
 *
 * @return  Minimum interval of focal length change..
 */
static inline uint16_t ttalink_pod_1078_get_optical_zoom_specs_get_ack_zoom_interval(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  5);
}

/**
 * @brief Decode a pod_1078_get_optical_zoom_specs message into a struct
 *
 * @param msg The message to decode
 * @param pod_1078_get_optical_zoom_specs C-struct to decode the message contents into
 */
static inline void ttalink_pod_1078_get_optical_zoom_specs_decode(const ttalink_message_t* msg, ttalink_pod_1078_get_optical_zoom_specs_t* pod_1078_get_optical_zoom_specs)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1078_get_optical_zoom_specs->ack = ttalink_pod_1078_get_optical_zoom_specs_get_ack(msg);
    pod_1078_get_optical_zoom_specs->ack_zoom_max = ttalink_pod_1078_get_optical_zoom_specs_get_ack_zoom_max(msg);
    pod_1078_get_optical_zoom_specs->ack_zoom_min = ttalink_pod_1078_get_optical_zoom_specs_get_ack_zoom_min(msg);
    pod_1078_get_optical_zoom_specs->ack_zoom_interval = ttalink_pod_1078_get_optical_zoom_specs_get_ack_zoom_interval(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN? msg->len : TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN;
        memset(pod_1078_get_optical_zoom_specs, 0, TTALINK_MSG_ID_POD_1078_GET_OPTICAL_ZOOM_SPECS_LEN);
    memcpy(pod_1078_get_optical_zoom_specs, _TTA_PAYLOAD(msg), len);
#endif
}
