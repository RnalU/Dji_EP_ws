#pragma once
// MESSAGE POD_1058_SET_MEASURE_AREA PACKING

#define TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA 11058

TTAPACKED(
typedef struct __ttalink_pod_1058_set_measure_area_t {
 uint8_t area; /*<  Point measurement position coordinates
												Bit [0 : 3] :  column coordinates, 0-11
												Bit [4 : 7] :  row coordinates, 0-7*/
 uint8_t ack; /*< .*/
}) ttalink_pod_1058_set_measure_area_t;

#define TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN 2
#define TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_MIN_LEN 2
#define TTALINK_MSG_ID_11058_LEN 2
#define TTALINK_MSG_ID_11058_MIN_LEN 2

#define TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_CRC 171
#define TTALINK_MSG_ID_11058_CRC 171



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1058_SET_MEASURE_AREA { \
    11058, \
    "POD_1058_SET_MEASURE_AREA", \
    2, \
    {  { "area", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1058_set_measure_area_t, area) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1058_set_measure_area_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1058_SET_MEASURE_AREA { \
    "POD_1058_SET_MEASURE_AREA", \
    2, \
    {  { "area", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1058_set_measure_area_t, area) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1058_set_measure_area_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1058_set_measure_area_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t area, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN];
    _tta_put_uint8_t(buf, 0, area);
    _tta_put_uint8_t(buf, 1, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN);
#else
    ttalink_pod_1058_set_measure_area_t packet;
    packet.area = area;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_MIN_LEN, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_CRC, nocrc);
}

/**
 * @brief Pack a pod_1058_set_measure_area message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param area  Point measurement position coordinates
												Bit [0 : 3] :  column coordinates, 0-11
												Bit [4 : 7] :  row coordinates, 0-7
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1058_set_measure_area_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t area, uint8_t ack)
{
    return _ttalink_pod_1058_set_measure_area_pack(dst_addr, src_addr, msg,  area, ack, false);
}

/**
 * @brief Pack a pod_1058_set_measure_area message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param area  Point measurement position coordinates
												Bit [0 : 3] :  column coordinates, 0-11
												Bit [4 : 7] :  row coordinates, 0-7
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1058_set_measure_area_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t area, uint8_t ack)
{
    return _ttalink_pod_1058_set_measure_area_pack(dst_addr, src_addr, msg,  area, ack, true);
}


static inline uint16_t _ttalink_pod_1058_set_measure_area_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t area,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN];
    _tta_put_uint8_t(buf, 0, area);
    _tta_put_uint8_t(buf, 1, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN);
#else
    ttalink_pod_1058_set_measure_area_t packet;
    packet.area = area;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_MIN_LEN, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_CRC, nocrc);
}

/**
 * @brief Pack a pod_1058_set_measure_area message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param area  Point measurement position coordinates
												Bit [0 : 3] :  column coordinates, 0-11
												Bit [4 : 7] :  row coordinates, 0-7
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1058_set_measure_area_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t area,uint8_t ack)
{
    return _ttalink_pod_1058_set_measure_area_pack_chan(dst_addr, src_addr, chan, msg,  area, ack, false);
}

/**
 * @brief Pack a pod_1058_set_measure_area message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param area  Point measurement position coordinates
												Bit [0 : 3] :  column coordinates, 0-11
												Bit [4 : 7] :  row coordinates, 0-7
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1058_set_measure_area_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t area,uint8_t ack)
{
    return _ttalink_pod_1058_set_measure_area_pack_chan(dst_addr, src_addr, chan, msg,  area, ack, true);
}


static inline uint16_t _ttalink_pod_1058_set_measure_area_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1058_set_measure_area_t* pod_1058_set_measure_area, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1058_set_measure_area_pack_nocrc(dst_addr, src_addr, msg, pod_1058_set_measure_area->area, pod_1058_set_measure_area->ack);
    }else{
        return ttalink_pod_1058_set_measure_area_pack(dst_addr, src_addr, msg, pod_1058_set_measure_area->area, pod_1058_set_measure_area->ack);
    }
    
}

/**
 * @brief Encode a pod_1058_set_measure_area struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1058_set_measure_area C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1058_set_measure_area_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1058_set_measure_area_t* pod_1058_set_measure_area)
{
    return _ttalink_pod_1058_set_measure_area_encode(dst_addr, src_addr, msg, pod_1058_set_measure_area, false);
}

/**
 * @brief Encode a pod_1058_set_measure_area struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1058_set_measure_area C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1058_set_measure_area_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1058_set_measure_area_t* pod_1058_set_measure_area)
{
    return _ttalink_pod_1058_set_measure_area_encode(dst_addr, src_addr, msg, pod_1058_set_measure_area, true);
}


static inline uint16_t _ttalink_pod_1058_set_measure_area_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1058_set_measure_area_t* pod_1058_set_measure_area, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1058_set_measure_area_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1058_set_measure_area->area, pod_1058_set_measure_area->ack);
    }else{
        return ttalink_pod_1058_set_measure_area_pack_chan(dst_addr, src_addr, chan, msg, pod_1058_set_measure_area->area, pod_1058_set_measure_area->ack);
    }
}

/**
 * @brief Encode a pod_1058_set_measure_area struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1058_set_measure_area C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1058_set_measure_area_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1058_set_measure_area_t* pod_1058_set_measure_area)
{
    return _ttalink_pod_1058_set_measure_area_encode_chan(dst_addr, src_addr, chan, msg, pod_1058_set_measure_area, false);
}

/**
 * @brief Encode a pod_1058_set_measure_area struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1058_set_measure_area C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1058_set_measure_area_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1058_set_measure_area_t* pod_1058_set_measure_area)
{
    return _ttalink_pod_1058_set_measure_area_encode_chan(dst_addr, src_addr, chan, msg, pod_1058_set_measure_area, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1058_set_measure_area_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t area, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN];
    _tta_put_uint8_t(buf, 0, area);
    _tta_put_uint8_t(buf, 1, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA, buf, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_MIN_LEN, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_CRC, nocrc);
#else
    ttalink_pod_1058_set_measure_area_t packet;
    packet.area = area;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA, (const char *)&packet, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_MIN_LEN, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1058_set_measure_area message
 * @param chan TTAlink channel to send the message
 *
 * @param area  Point measurement position coordinates
												Bit [0 : 3] :  column coordinates, 0-11
												Bit [4 : 7] :  row coordinates, 0-7
 * @param ack .
 */
static inline void ttalink_pod_1058_set_measure_area_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t area, uint8_t ack)
{
    _ttalink_pod_1058_set_measure_area_send(dst_addr, src_addr, chan, area, ack, false);
}

/**
 * @brief Send a pod_1058_set_measure_area message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param area  Point measurement position coordinates
												Bit [0 : 3] :  column coordinates, 0-11
												Bit [4 : 7] :  row coordinates, 0-7
 * @param ack .
 */
static inline void ttalink_pod_1058_set_measure_area_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t area, uint8_t ack)
{
    _ttalink_pod_1058_set_measure_area_send(dst_addr, src_addr, chan, area, ack, true);
}


static inline void _ttalink_pod_1058_set_measure_area_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1058_set_measure_area_t* pod_1058_set_measure_area, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1058_set_measure_area_send_nocrc(dst_addr, src_addr, chan, pod_1058_set_measure_area->area, pod_1058_set_measure_area->ack);
    }else{
        ttalink_pod_1058_set_measure_area_send(dst_addr, src_addr, chan, pod_1058_set_measure_area->area, pod_1058_set_measure_area->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA, (const char *)pod_1058_set_measure_area, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_MIN_LEN, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1058_set_measure_area message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1058_set_measure_area_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1058_set_measure_area_t* pod_1058_set_measure_area)
{
    _ttalink_pod_1058_set_measure_area_send_struct(dst_addr, src_addr, chan, pod_1058_set_measure_area, false);
}

/**
 * @brief Send a pod_1058_set_measure_area message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1058_set_measure_area_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1058_set_measure_area_t* pod_1058_set_measure_area)
{
    _ttalink_pod_1058_set_measure_area_send_struct(dst_addr, src_addr, chan, pod_1058_set_measure_area, true);
}

#if TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1058_set_measure_area_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t area, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, area);
    _tta_put_uint8_t(buf, 1, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA, buf, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_MIN_LEN, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_CRC, nocrc);
#else
    ttalink_pod_1058_set_measure_area_t *packet = (ttalink_pod_1058_set_measure_area_t *)msgbuf;
    packet->area = area;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA, (const char *)packet, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_MIN_LEN, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1058_set_measure_area_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t area, uint8_t ack)
{
    _ttalink_pod_1058_set_measure_area_send_buf(dst_addr, src_addr, msgbuf, chan, area, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1058_set_measure_area_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t area, uint8_t ack)
{
    _ttalink_pod_1058_set_measure_area_send_buf(dst_addr, src_addr, msgbuf, chan, area, ack, true);
}
#endif

#endif

// MESSAGE POD_1058_SET_MEASURE_AREA UNPACKING


/**
 * @brief Get field area from pod_1058_set_measure_area message
 *
 * @return  Point measurement position coordinates
												Bit [0 : 3] :  column coordinates, 0-11
												Bit [4 : 7] :  row coordinates, 0-7
 */
static inline uint8_t ttalink_pod_1058_set_measure_area_get_area(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack from pod_1058_set_measure_area message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1058_set_measure_area_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a pod_1058_set_measure_area message into a struct
 *
 * @param msg The message to decode
 * @param pod_1058_set_measure_area C-struct to decode the message contents into
 */
static inline void ttalink_pod_1058_set_measure_area_decode(const ttalink_message_t* msg, ttalink_pod_1058_set_measure_area_t* pod_1058_set_measure_area)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1058_set_measure_area->area = ttalink_pod_1058_set_measure_area_get_area(msg);
    pod_1058_set_measure_area->ack = ttalink_pod_1058_set_measure_area_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN? msg->len : TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN;
        memset(pod_1058_set_measure_area, 0, TTALINK_MSG_ID_POD_1058_SET_MEASURE_AREA_LEN);
    memcpy(pod_1058_set_measure_area, _TTA_PAYLOAD(msg), len);
#endif
}
