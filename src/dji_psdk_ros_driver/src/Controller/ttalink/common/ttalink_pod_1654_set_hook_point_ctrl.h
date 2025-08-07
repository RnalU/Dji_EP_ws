#pragma once
// MESSAGE POD_1654_SET_HOOK_POINT_CTRL PACKING

#define TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL 11654

TTAPACKED(
typedef struct __ttalink_pod_1654_set_hook_point_ctrl_t {
 uint8_t cmd; /*< 0 : check_all  1~3 : point1-3*/
 uint8_t point1; /*< 0=close 1=open*/
 uint8_t point2; /*< 0=close 1=open*/
 uint8_t point3; /*< 0=close 1=open*/
 uint8_t led; /*< light 0~100*/
 uint8_t ack; /*< .*/
}) ttalink_pod_1654_set_hook_point_ctrl_t;

#define TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN 6
#define TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_MIN_LEN 6
#define TTALINK_MSG_ID_11654_LEN 6
#define TTALINK_MSG_ID_11654_MIN_LEN 6

#define TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_CRC 119
#define TTALINK_MSG_ID_11654_CRC 119



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1654_SET_HOOK_POINT_CTRL { \
    11654, \
    "POD_1654_SET_HOOK_POINT_CTRL", \
    6, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1654_set_hook_point_ctrl_t, cmd) }, \
         { "point1", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1654_set_hook_point_ctrl_t, point1) }, \
         { "point2", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_pod_1654_set_hook_point_ctrl_t, point2) }, \
         { "point3", NULL, TTALINK_TYPE_UINT8_T, 0, 3, offsetof(ttalink_pod_1654_set_hook_point_ctrl_t, point3) }, \
         { "led", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_pod_1654_set_hook_point_ctrl_t, led) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_pod_1654_set_hook_point_ctrl_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1654_SET_HOOK_POINT_CTRL { \
    "POD_1654_SET_HOOK_POINT_CTRL", \
    6, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1654_set_hook_point_ctrl_t, cmd) }, \
         { "point1", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1654_set_hook_point_ctrl_t, point1) }, \
         { "point2", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_pod_1654_set_hook_point_ctrl_t, point2) }, \
         { "point3", NULL, TTALINK_TYPE_UINT8_T, 0, 3, offsetof(ttalink_pod_1654_set_hook_point_ctrl_t, point3) }, \
         { "led", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_pod_1654_set_hook_point_ctrl_t, led) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_pod_1654_set_hook_point_ctrl_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1654_set_hook_point_ctrl_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t point1, uint8_t point2, uint8_t point3, uint8_t led, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, point1);
    _tta_put_uint8_t(buf, 2, point2);
    _tta_put_uint8_t(buf, 3, point3);
    _tta_put_uint8_t(buf, 4, led);
    _tta_put_uint8_t(buf, 5, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN);
#else
    ttalink_pod_1654_set_hook_point_ctrl_t packet;
    packet.cmd = cmd;
    packet.point1 = point1;
    packet.point2 = point2;
    packet.point3 = point3;
    packet.led = led;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_CRC, nocrc);
}

/**
 * @brief Pack a pod_1654_set_hook_point_ctrl message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd 0 : check_all  1~3 : point1-3
 * @param point1 0=close 1=open
 * @param point2 0=close 1=open
 * @param point3 0=close 1=open
 * @param led light 0~100
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1654_set_hook_point_ctrl_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t point1, uint8_t point2, uint8_t point3, uint8_t led, uint8_t ack)
{
    return _ttalink_pod_1654_set_hook_point_ctrl_pack(dst_addr, src_addr, msg,  cmd, point1, point2, point3, led, ack, false);
}

/**
 * @brief Pack a pod_1654_set_hook_point_ctrl message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd 0 : check_all  1~3 : point1-3
 * @param point1 0=close 1=open
 * @param point2 0=close 1=open
 * @param point3 0=close 1=open
 * @param led light 0~100
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1654_set_hook_point_ctrl_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t point1, uint8_t point2, uint8_t point3, uint8_t led, uint8_t ack)
{
    return _ttalink_pod_1654_set_hook_point_ctrl_pack(dst_addr, src_addr, msg,  cmd, point1, point2, point3, led, ack, true);
}


static inline uint16_t _ttalink_pod_1654_set_hook_point_ctrl_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t point1,uint8_t point2,uint8_t point3,uint8_t led,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, point1);
    _tta_put_uint8_t(buf, 2, point2);
    _tta_put_uint8_t(buf, 3, point3);
    _tta_put_uint8_t(buf, 4, led);
    _tta_put_uint8_t(buf, 5, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN);
#else
    ttalink_pod_1654_set_hook_point_ctrl_t packet;
    packet.cmd = cmd;
    packet.point1 = point1;
    packet.point2 = point2;
    packet.point3 = point3;
    packet.led = led;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_CRC, nocrc);
}

/**
 * @brief Pack a pod_1654_set_hook_point_ctrl message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd 0 : check_all  1~3 : point1-3
 * @param point1 0=close 1=open
 * @param point2 0=close 1=open
 * @param point3 0=close 1=open
 * @param led light 0~100
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1654_set_hook_point_ctrl_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t point1,uint8_t point2,uint8_t point3,uint8_t led,uint8_t ack)
{
    return _ttalink_pod_1654_set_hook_point_ctrl_pack_chan(dst_addr, src_addr, chan, msg,  cmd, point1, point2, point3, led, ack, false);
}

/**
 * @brief Pack a pod_1654_set_hook_point_ctrl message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd 0 : check_all  1~3 : point1-3
 * @param point1 0=close 1=open
 * @param point2 0=close 1=open
 * @param point3 0=close 1=open
 * @param led light 0~100
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1654_set_hook_point_ctrl_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t point1,uint8_t point2,uint8_t point3,uint8_t led,uint8_t ack)
{
    return _ttalink_pod_1654_set_hook_point_ctrl_pack_chan(dst_addr, src_addr, chan, msg,  cmd, point1, point2, point3, led, ack, true);
}


static inline uint16_t _ttalink_pod_1654_set_hook_point_ctrl_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1654_set_hook_point_ctrl_t* pod_1654_set_hook_point_ctrl, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1654_set_hook_point_ctrl_pack_nocrc(dst_addr, src_addr, msg, pod_1654_set_hook_point_ctrl->cmd, pod_1654_set_hook_point_ctrl->point1, pod_1654_set_hook_point_ctrl->point2, pod_1654_set_hook_point_ctrl->point3, pod_1654_set_hook_point_ctrl->led, pod_1654_set_hook_point_ctrl->ack);
    }else{
        return ttalink_pod_1654_set_hook_point_ctrl_pack(dst_addr, src_addr, msg, pod_1654_set_hook_point_ctrl->cmd, pod_1654_set_hook_point_ctrl->point1, pod_1654_set_hook_point_ctrl->point2, pod_1654_set_hook_point_ctrl->point3, pod_1654_set_hook_point_ctrl->led, pod_1654_set_hook_point_ctrl->ack);
    }
    
}

/**
 * @brief Encode a pod_1654_set_hook_point_ctrl struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1654_set_hook_point_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1654_set_hook_point_ctrl_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1654_set_hook_point_ctrl_t* pod_1654_set_hook_point_ctrl)
{
    return _ttalink_pod_1654_set_hook_point_ctrl_encode(dst_addr, src_addr, msg, pod_1654_set_hook_point_ctrl, false);
}

/**
 * @brief Encode a pod_1654_set_hook_point_ctrl struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1654_set_hook_point_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1654_set_hook_point_ctrl_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1654_set_hook_point_ctrl_t* pod_1654_set_hook_point_ctrl)
{
    return _ttalink_pod_1654_set_hook_point_ctrl_encode(dst_addr, src_addr, msg, pod_1654_set_hook_point_ctrl, true);
}


static inline uint16_t _ttalink_pod_1654_set_hook_point_ctrl_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1654_set_hook_point_ctrl_t* pod_1654_set_hook_point_ctrl, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1654_set_hook_point_ctrl_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1654_set_hook_point_ctrl->cmd, pod_1654_set_hook_point_ctrl->point1, pod_1654_set_hook_point_ctrl->point2, pod_1654_set_hook_point_ctrl->point3, pod_1654_set_hook_point_ctrl->led, pod_1654_set_hook_point_ctrl->ack);
    }else{
        return ttalink_pod_1654_set_hook_point_ctrl_pack_chan(dst_addr, src_addr, chan, msg, pod_1654_set_hook_point_ctrl->cmd, pod_1654_set_hook_point_ctrl->point1, pod_1654_set_hook_point_ctrl->point2, pod_1654_set_hook_point_ctrl->point3, pod_1654_set_hook_point_ctrl->led, pod_1654_set_hook_point_ctrl->ack);
    }
}

/**
 * @brief Encode a pod_1654_set_hook_point_ctrl struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1654_set_hook_point_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1654_set_hook_point_ctrl_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1654_set_hook_point_ctrl_t* pod_1654_set_hook_point_ctrl)
{
    return _ttalink_pod_1654_set_hook_point_ctrl_encode_chan(dst_addr, src_addr, chan, msg, pod_1654_set_hook_point_ctrl, false);
}

/**
 * @brief Encode a pod_1654_set_hook_point_ctrl struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1654_set_hook_point_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1654_set_hook_point_ctrl_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1654_set_hook_point_ctrl_t* pod_1654_set_hook_point_ctrl)
{
    return _ttalink_pod_1654_set_hook_point_ctrl_encode_chan(dst_addr, src_addr, chan, msg, pod_1654_set_hook_point_ctrl, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1654_set_hook_point_ctrl_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t point1, uint8_t point2, uint8_t point3, uint8_t led, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, point1);
    _tta_put_uint8_t(buf, 2, point2);
    _tta_put_uint8_t(buf, 3, point3);
    _tta_put_uint8_t(buf, 4, led);
    _tta_put_uint8_t(buf, 5, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL, buf, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_CRC, nocrc);
#else
    ttalink_pod_1654_set_hook_point_ctrl_t packet;
    packet.cmd = cmd;
    packet.point1 = point1;
    packet.point2 = point2;
    packet.point3 = point3;
    packet.led = led;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL, (const char *)&packet, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1654_set_hook_point_ctrl message
 * @param chan TTAlink channel to send the message
 *
 * @param cmd 0 : check_all  1~3 : point1-3
 * @param point1 0=close 1=open
 * @param point2 0=close 1=open
 * @param point3 0=close 1=open
 * @param led light 0~100
 * @param ack .
 */
static inline void ttalink_pod_1654_set_hook_point_ctrl_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t point1, uint8_t point2, uint8_t point3, uint8_t led, uint8_t ack)
{
    _ttalink_pod_1654_set_hook_point_ctrl_send(dst_addr, src_addr, chan, cmd, point1, point2, point3, led, ack, false);
}

/**
 * @brief Send a pod_1654_set_hook_point_ctrl message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param cmd 0 : check_all  1~3 : point1-3
 * @param point1 0=close 1=open
 * @param point2 0=close 1=open
 * @param point3 0=close 1=open
 * @param led light 0~100
 * @param ack .
 */
static inline void ttalink_pod_1654_set_hook_point_ctrl_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t point1, uint8_t point2, uint8_t point3, uint8_t led, uint8_t ack)
{
    _ttalink_pod_1654_set_hook_point_ctrl_send(dst_addr, src_addr, chan, cmd, point1, point2, point3, led, ack, true);
}


static inline void _ttalink_pod_1654_set_hook_point_ctrl_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1654_set_hook_point_ctrl_t* pod_1654_set_hook_point_ctrl, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1654_set_hook_point_ctrl_send_nocrc(dst_addr, src_addr, chan, pod_1654_set_hook_point_ctrl->cmd, pod_1654_set_hook_point_ctrl->point1, pod_1654_set_hook_point_ctrl->point2, pod_1654_set_hook_point_ctrl->point3, pod_1654_set_hook_point_ctrl->led, pod_1654_set_hook_point_ctrl->ack);
    }else{
        ttalink_pod_1654_set_hook_point_ctrl_send(dst_addr, src_addr, chan, pod_1654_set_hook_point_ctrl->cmd, pod_1654_set_hook_point_ctrl->point1, pod_1654_set_hook_point_ctrl->point2, pod_1654_set_hook_point_ctrl->point3, pod_1654_set_hook_point_ctrl->led, pod_1654_set_hook_point_ctrl->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL, (const char *)pod_1654_set_hook_point_ctrl, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1654_set_hook_point_ctrl message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1654_set_hook_point_ctrl_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1654_set_hook_point_ctrl_t* pod_1654_set_hook_point_ctrl)
{
    _ttalink_pod_1654_set_hook_point_ctrl_send_struct(dst_addr, src_addr, chan, pod_1654_set_hook_point_ctrl, false);
}

/**
 * @brief Send a pod_1654_set_hook_point_ctrl message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1654_set_hook_point_ctrl_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1654_set_hook_point_ctrl_t* pod_1654_set_hook_point_ctrl)
{
    _ttalink_pod_1654_set_hook_point_ctrl_send_struct(dst_addr, src_addr, chan, pod_1654_set_hook_point_ctrl, true);
}

#if TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1654_set_hook_point_ctrl_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t point1, uint8_t point2, uint8_t point3, uint8_t led, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, point1);
    _tta_put_uint8_t(buf, 2, point2);
    _tta_put_uint8_t(buf, 3, point3);
    _tta_put_uint8_t(buf, 4, led);
    _tta_put_uint8_t(buf, 5, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL, buf, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_CRC, nocrc);
#else
    ttalink_pod_1654_set_hook_point_ctrl_t *packet = (ttalink_pod_1654_set_hook_point_ctrl_t *)msgbuf;
    packet->cmd = cmd;
    packet->point1 = point1;
    packet->point2 = point2;
    packet->point3 = point3;
    packet->led = led;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL, (const char *)packet, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_MIN_LEN, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1654_set_hook_point_ctrl_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t point1, uint8_t point2, uint8_t point3, uint8_t led, uint8_t ack)
{
    _ttalink_pod_1654_set_hook_point_ctrl_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, point1, point2, point3, led, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1654_set_hook_point_ctrl_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t point1, uint8_t point2, uint8_t point3, uint8_t led, uint8_t ack)
{
    _ttalink_pod_1654_set_hook_point_ctrl_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, point1, point2, point3, led, ack, true);
}
#endif

#endif

// MESSAGE POD_1654_SET_HOOK_POINT_CTRL UNPACKING


/**
 * @brief Get field cmd from pod_1654_set_hook_point_ctrl message
 *
 * @return 0 : check_all  1~3 : point1-3
 */
static inline uint8_t ttalink_pod_1654_set_hook_point_ctrl_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field point1 from pod_1654_set_hook_point_ctrl message
 *
 * @return 0=close 1=open
 */
static inline uint8_t ttalink_pod_1654_set_hook_point_ctrl_get_point1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field point2 from pod_1654_set_hook_point_ctrl message
 *
 * @return 0=close 1=open
 */
static inline uint8_t ttalink_pod_1654_set_hook_point_ctrl_get_point2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field point3 from pod_1654_set_hook_point_ctrl message
 *
 * @return 0=close 1=open
 */
static inline uint8_t ttalink_pod_1654_set_hook_point_ctrl_get_point3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field led from pod_1654_set_hook_point_ctrl message
 *
 * @return light 0~100
 */
static inline uint8_t ttalink_pod_1654_set_hook_point_ctrl_get_led(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field ack from pod_1654_set_hook_point_ctrl message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1654_set_hook_point_ctrl_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Decode a pod_1654_set_hook_point_ctrl message into a struct
 *
 * @param msg The message to decode
 * @param pod_1654_set_hook_point_ctrl C-struct to decode the message contents into
 */
static inline void ttalink_pod_1654_set_hook_point_ctrl_decode(const ttalink_message_t* msg, ttalink_pod_1654_set_hook_point_ctrl_t* pod_1654_set_hook_point_ctrl)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1654_set_hook_point_ctrl->cmd = ttalink_pod_1654_set_hook_point_ctrl_get_cmd(msg);
    pod_1654_set_hook_point_ctrl->point1 = ttalink_pod_1654_set_hook_point_ctrl_get_point1(msg);
    pod_1654_set_hook_point_ctrl->point2 = ttalink_pod_1654_set_hook_point_ctrl_get_point2(msg);
    pod_1654_set_hook_point_ctrl->point3 = ttalink_pod_1654_set_hook_point_ctrl_get_point3(msg);
    pod_1654_set_hook_point_ctrl->led = ttalink_pod_1654_set_hook_point_ctrl_get_led(msg);
    pod_1654_set_hook_point_ctrl->ack = ttalink_pod_1654_set_hook_point_ctrl_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN? msg->len : TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN;
        memset(pod_1654_set_hook_point_ctrl, 0, TTALINK_MSG_ID_POD_1654_SET_HOOK_POINT_CTRL_LEN);
    memcpy(pod_1654_set_hook_point_ctrl, _TTA_PAYLOAD(msg), len);
#endif
}
