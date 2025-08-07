#pragma once
// MESSAGE POD_1281_GET_PTZ_STATUS PACKING

#define TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS 11281

TTAPACKED(
typedef struct __ttalink_pod_1281_get_ptz_status_t {
 uint8_t ack; /*< .*/
 uint8_t ack_mode; /*< PTZ mode
												0 :  free mode
												1 :  Follow mode
												2 :  FPV mode..*/
 uint8_t ack_state; /*< PTZ state
												Bit [0] :  installation direction of PTZ
												0 :  downward installation;
												1 :  Install up
												Bit [1] :  yaw limit reached
												0 :  limit not reached;
												1 :  Reach the limit
												Bit [2] :  whether the rolling reaches the limit
												0 :  limit not reached;
												1 :  Reach the limit
												Bit [3] :  whether the pitch reaches the limit
												0 :  limit not reached;
												1 :  Reach the limit.*/
 uint16_t ack_pitch; /*< Current pitch axis angle in 0.1 degrees.*/
 uint16_t ack_roll; /*< Current roll axis angle in 0.1 degrees.*/
 uint16_t ack_yaw; /*< Current yaw axis angle in 0.1 degrees.*/
}) ttalink_pod_1281_get_ptz_status_t;

#define TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN 9
#define TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_MIN_LEN 9
#define TTALINK_MSG_ID_11281_LEN 9
#define TTALINK_MSG_ID_11281_MIN_LEN 9

#define TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_CRC 2
#define TTALINK_MSG_ID_11281_CRC 2



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1281_GET_PTZ_STATUS { \
    11281, \
    "POD_1281_GET_PTZ_STATUS", \
    6, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1281_get_ptz_status_t, ack) }, \
         { "ack_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1281_get_ptz_status_t, ack_mode) }, \
         { "ack_state", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_pod_1281_get_ptz_status_t, ack_state) }, \
         { "ack_pitch", NULL, TTALINK_TYPE_UINT16_T, 0, 3, offsetof(ttalink_pod_1281_get_ptz_status_t, ack_pitch) }, \
         { "ack_roll", NULL, TTALINK_TYPE_UINT16_T, 0, 5, offsetof(ttalink_pod_1281_get_ptz_status_t, ack_roll) }, \
         { "ack_yaw", NULL, TTALINK_TYPE_UINT16_T, 0, 7, offsetof(ttalink_pod_1281_get_ptz_status_t, ack_yaw) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1281_GET_PTZ_STATUS { \
    "POD_1281_GET_PTZ_STATUS", \
    6, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1281_get_ptz_status_t, ack) }, \
         { "ack_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1281_get_ptz_status_t, ack_mode) }, \
         { "ack_state", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_pod_1281_get_ptz_status_t, ack_state) }, \
         { "ack_pitch", NULL, TTALINK_TYPE_UINT16_T, 0, 3, offsetof(ttalink_pod_1281_get_ptz_status_t, ack_pitch) }, \
         { "ack_roll", NULL, TTALINK_TYPE_UINT16_T, 0, 5, offsetof(ttalink_pod_1281_get_ptz_status_t, ack_roll) }, \
         { "ack_yaw", NULL, TTALINK_TYPE_UINT16_T, 0, 7, offsetof(ttalink_pod_1281_get_ptz_status_t, ack_yaw) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1281_get_ptz_status_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_mode, uint8_t ack_state, uint16_t ack_pitch, uint16_t ack_roll, uint16_t ack_yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_mode);
    _tta_put_uint8_t(buf, 2, ack_state);
    _tta_put_uint16_t(buf, 3, ack_pitch);
    _tta_put_uint16_t(buf, 5, ack_roll);
    _tta_put_uint16_t(buf, 7, ack_yaw);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN);
#else
    ttalink_pod_1281_get_ptz_status_t packet;
    packet.ack = ack;
    packet.ack_mode = ack_mode;
    packet.ack_state = ack_state;
    packet.ack_pitch = ack_pitch;
    packet.ack_roll = ack_roll;
    packet.ack_yaw = ack_yaw;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_MIN_LEN, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_CRC, nocrc);
}

/**
 * @brief Pack a pod_1281_get_ptz_status message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param ack_mode PTZ mode
												0 :  free mode
												1 :  Follow mode
												2 :  FPV mode..
 * @param ack_state PTZ state
												Bit [0] :  installation direction of PTZ
												0 :  downward installation;
												1 :  Install up
												Bit [1] :  yaw limit reached
												0 :  limit not reached;
												1 :  Reach the limit
												Bit [2] :  whether the rolling reaches the limit
												0 :  limit not reached;
												1 :  Reach the limit
												Bit [3] :  whether the pitch reaches the limit
												0 :  limit not reached;
												1 :  Reach the limit.
 * @param ack_pitch Current pitch axis angle in 0.1 degrees.
 * @param ack_roll Current roll axis angle in 0.1 degrees.
 * @param ack_yaw Current yaw axis angle in 0.1 degrees.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1281_get_ptz_status_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_mode, uint8_t ack_state, uint16_t ack_pitch, uint16_t ack_roll, uint16_t ack_yaw)
{
    return _ttalink_pod_1281_get_ptz_status_pack(dst_addr, src_addr, msg,  ack, ack_mode, ack_state, ack_pitch, ack_roll, ack_yaw, false);
}

/**
 * @brief Pack a pod_1281_get_ptz_status message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param ack_mode PTZ mode
												0 :  free mode
												1 :  Follow mode
												2 :  FPV mode..
 * @param ack_state PTZ state
												Bit [0] :  installation direction of PTZ
												0 :  downward installation;
												1 :  Install up
												Bit [1] :  yaw limit reached
												0 :  limit not reached;
												1 :  Reach the limit
												Bit [2] :  whether the rolling reaches the limit
												0 :  limit not reached;
												1 :  Reach the limit
												Bit [3] :  whether the pitch reaches the limit
												0 :  limit not reached;
												1 :  Reach the limit.
 * @param ack_pitch Current pitch axis angle in 0.1 degrees.
 * @param ack_roll Current roll axis angle in 0.1 degrees.
 * @param ack_yaw Current yaw axis angle in 0.1 degrees.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1281_get_ptz_status_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_mode, uint8_t ack_state, uint16_t ack_pitch, uint16_t ack_roll, uint16_t ack_yaw)
{
    return _ttalink_pod_1281_get_ptz_status_pack(dst_addr, src_addr, msg,  ack, ack_mode, ack_state, ack_pitch, ack_roll, ack_yaw, true);
}


static inline uint16_t _ttalink_pod_1281_get_ptz_status_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_mode,uint8_t ack_state,uint16_t ack_pitch,uint16_t ack_roll,uint16_t ack_yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_mode);
    _tta_put_uint8_t(buf, 2, ack_state);
    _tta_put_uint16_t(buf, 3, ack_pitch);
    _tta_put_uint16_t(buf, 5, ack_roll);
    _tta_put_uint16_t(buf, 7, ack_yaw);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN);
#else
    ttalink_pod_1281_get_ptz_status_t packet;
    packet.ack = ack;
    packet.ack_mode = ack_mode;
    packet.ack_state = ack_state;
    packet.ack_pitch = ack_pitch;
    packet.ack_roll = ack_roll;
    packet.ack_yaw = ack_yaw;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_MIN_LEN, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_CRC, nocrc);
}

/**
 * @brief Pack a pod_1281_get_ptz_status message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param ack_mode PTZ mode
												0 :  free mode
												1 :  Follow mode
												2 :  FPV mode..
 * @param ack_state PTZ state
												Bit [0] :  installation direction of PTZ
												0 :  downward installation;
												1 :  Install up
												Bit [1] :  yaw limit reached
												0 :  limit not reached;
												1 :  Reach the limit
												Bit [2] :  whether the rolling reaches the limit
												0 :  limit not reached;
												1 :  Reach the limit
												Bit [3] :  whether the pitch reaches the limit
												0 :  limit not reached;
												1 :  Reach the limit.
 * @param ack_pitch Current pitch axis angle in 0.1 degrees.
 * @param ack_roll Current roll axis angle in 0.1 degrees.
 * @param ack_yaw Current yaw axis angle in 0.1 degrees.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1281_get_ptz_status_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_mode,uint8_t ack_state,uint16_t ack_pitch,uint16_t ack_roll,uint16_t ack_yaw)
{
    return _ttalink_pod_1281_get_ptz_status_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_mode, ack_state, ack_pitch, ack_roll, ack_yaw, false);
}

/**
 * @brief Pack a pod_1281_get_ptz_status message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param ack_mode PTZ mode
												0 :  free mode
												1 :  Follow mode
												2 :  FPV mode..
 * @param ack_state PTZ state
												Bit [0] :  installation direction of PTZ
												0 :  downward installation;
												1 :  Install up
												Bit [1] :  yaw limit reached
												0 :  limit not reached;
												1 :  Reach the limit
												Bit [2] :  whether the rolling reaches the limit
												0 :  limit not reached;
												1 :  Reach the limit
												Bit [3] :  whether the pitch reaches the limit
												0 :  limit not reached;
												1 :  Reach the limit.
 * @param ack_pitch Current pitch axis angle in 0.1 degrees.
 * @param ack_roll Current roll axis angle in 0.1 degrees.
 * @param ack_yaw Current yaw axis angle in 0.1 degrees.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1281_get_ptz_status_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_mode,uint8_t ack_state,uint16_t ack_pitch,uint16_t ack_roll,uint16_t ack_yaw)
{
    return _ttalink_pod_1281_get_ptz_status_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_mode, ack_state, ack_pitch, ack_roll, ack_yaw, true);
}


static inline uint16_t _ttalink_pod_1281_get_ptz_status_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1281_get_ptz_status_t* pod_1281_get_ptz_status, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1281_get_ptz_status_pack_nocrc(dst_addr, src_addr, msg, pod_1281_get_ptz_status->ack, pod_1281_get_ptz_status->ack_mode, pod_1281_get_ptz_status->ack_state, pod_1281_get_ptz_status->ack_pitch, pod_1281_get_ptz_status->ack_roll, pod_1281_get_ptz_status->ack_yaw);
    }else{
        return ttalink_pod_1281_get_ptz_status_pack(dst_addr, src_addr, msg, pod_1281_get_ptz_status->ack, pod_1281_get_ptz_status->ack_mode, pod_1281_get_ptz_status->ack_state, pod_1281_get_ptz_status->ack_pitch, pod_1281_get_ptz_status->ack_roll, pod_1281_get_ptz_status->ack_yaw);
    }
    
}

/**
 * @brief Encode a pod_1281_get_ptz_status struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1281_get_ptz_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1281_get_ptz_status_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1281_get_ptz_status_t* pod_1281_get_ptz_status)
{
    return _ttalink_pod_1281_get_ptz_status_encode(dst_addr, src_addr, msg, pod_1281_get_ptz_status, false);
}

/**
 * @brief Encode a pod_1281_get_ptz_status struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1281_get_ptz_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1281_get_ptz_status_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1281_get_ptz_status_t* pod_1281_get_ptz_status)
{
    return _ttalink_pod_1281_get_ptz_status_encode(dst_addr, src_addr, msg, pod_1281_get_ptz_status, true);
}


static inline uint16_t _ttalink_pod_1281_get_ptz_status_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1281_get_ptz_status_t* pod_1281_get_ptz_status, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1281_get_ptz_status_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1281_get_ptz_status->ack, pod_1281_get_ptz_status->ack_mode, pod_1281_get_ptz_status->ack_state, pod_1281_get_ptz_status->ack_pitch, pod_1281_get_ptz_status->ack_roll, pod_1281_get_ptz_status->ack_yaw);
    }else{
        return ttalink_pod_1281_get_ptz_status_pack_chan(dst_addr, src_addr, chan, msg, pod_1281_get_ptz_status->ack, pod_1281_get_ptz_status->ack_mode, pod_1281_get_ptz_status->ack_state, pod_1281_get_ptz_status->ack_pitch, pod_1281_get_ptz_status->ack_roll, pod_1281_get_ptz_status->ack_yaw);
    }
}

/**
 * @brief Encode a pod_1281_get_ptz_status struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1281_get_ptz_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1281_get_ptz_status_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1281_get_ptz_status_t* pod_1281_get_ptz_status)
{
    return _ttalink_pod_1281_get_ptz_status_encode_chan(dst_addr, src_addr, chan, msg, pod_1281_get_ptz_status, false);
}

/**
 * @brief Encode a pod_1281_get_ptz_status struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1281_get_ptz_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1281_get_ptz_status_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1281_get_ptz_status_t* pod_1281_get_ptz_status)
{
    return _ttalink_pod_1281_get_ptz_status_encode_chan(dst_addr, src_addr, chan, msg, pod_1281_get_ptz_status, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1281_get_ptz_status_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_mode, uint8_t ack_state, uint16_t ack_pitch, uint16_t ack_roll, uint16_t ack_yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_mode);
    _tta_put_uint8_t(buf, 2, ack_state);
    _tta_put_uint16_t(buf, 3, ack_pitch);
    _tta_put_uint16_t(buf, 5, ack_roll);
    _tta_put_uint16_t(buf, 7, ack_yaw);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS, buf, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_MIN_LEN, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_CRC, nocrc);
#else
    ttalink_pod_1281_get_ptz_status_t packet;
    packet.ack = ack;
    packet.ack_mode = ack_mode;
    packet.ack_state = ack_state;
    packet.ack_pitch = ack_pitch;
    packet.ack_roll = ack_roll;
    packet.ack_yaw = ack_yaw;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS, (const char *)&packet, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_MIN_LEN, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1281_get_ptz_status message
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param ack_mode PTZ mode
												0 :  free mode
												1 :  Follow mode
												2 :  FPV mode..
 * @param ack_state PTZ state
												Bit [0] :  installation direction of PTZ
												0 :  downward installation;
												1 :  Install up
												Bit [1] :  yaw limit reached
												0 :  limit not reached;
												1 :  Reach the limit
												Bit [2] :  whether the rolling reaches the limit
												0 :  limit not reached;
												1 :  Reach the limit
												Bit [3] :  whether the pitch reaches the limit
												0 :  limit not reached;
												1 :  Reach the limit.
 * @param ack_pitch Current pitch axis angle in 0.1 degrees.
 * @param ack_roll Current roll axis angle in 0.1 degrees.
 * @param ack_yaw Current yaw axis angle in 0.1 degrees.
 */
static inline void ttalink_pod_1281_get_ptz_status_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_mode, uint8_t ack_state, uint16_t ack_pitch, uint16_t ack_roll, uint16_t ack_yaw)
{
    _ttalink_pod_1281_get_ptz_status_send(dst_addr, src_addr, chan, ack, ack_mode, ack_state, ack_pitch, ack_roll, ack_yaw, false);
}

/**
 * @brief Send a pod_1281_get_ptz_status message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param ack_mode PTZ mode
												0 :  free mode
												1 :  Follow mode
												2 :  FPV mode..
 * @param ack_state PTZ state
												Bit [0] :  installation direction of PTZ
												0 :  downward installation;
												1 :  Install up
												Bit [1] :  yaw limit reached
												0 :  limit not reached;
												1 :  Reach the limit
												Bit [2] :  whether the rolling reaches the limit
												0 :  limit not reached;
												1 :  Reach the limit
												Bit [3] :  whether the pitch reaches the limit
												0 :  limit not reached;
												1 :  Reach the limit.
 * @param ack_pitch Current pitch axis angle in 0.1 degrees.
 * @param ack_roll Current roll axis angle in 0.1 degrees.
 * @param ack_yaw Current yaw axis angle in 0.1 degrees.
 */
static inline void ttalink_pod_1281_get_ptz_status_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_mode, uint8_t ack_state, uint16_t ack_pitch, uint16_t ack_roll, uint16_t ack_yaw)
{
    _ttalink_pod_1281_get_ptz_status_send(dst_addr, src_addr, chan, ack, ack_mode, ack_state, ack_pitch, ack_roll, ack_yaw, true);
}


static inline void _ttalink_pod_1281_get_ptz_status_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1281_get_ptz_status_t* pod_1281_get_ptz_status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1281_get_ptz_status_send_nocrc(dst_addr, src_addr, chan, pod_1281_get_ptz_status->ack, pod_1281_get_ptz_status->ack_mode, pod_1281_get_ptz_status->ack_state, pod_1281_get_ptz_status->ack_pitch, pod_1281_get_ptz_status->ack_roll, pod_1281_get_ptz_status->ack_yaw);
    }else{
        ttalink_pod_1281_get_ptz_status_send(dst_addr, src_addr, chan, pod_1281_get_ptz_status->ack, pod_1281_get_ptz_status->ack_mode, pod_1281_get_ptz_status->ack_state, pod_1281_get_ptz_status->ack_pitch, pod_1281_get_ptz_status->ack_roll, pod_1281_get_ptz_status->ack_yaw);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS, (const char *)pod_1281_get_ptz_status, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_MIN_LEN, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1281_get_ptz_status message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1281_get_ptz_status_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1281_get_ptz_status_t* pod_1281_get_ptz_status)
{
    _ttalink_pod_1281_get_ptz_status_send_struct(dst_addr, src_addr, chan, pod_1281_get_ptz_status, false);
}

/**
 * @brief Send a pod_1281_get_ptz_status message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1281_get_ptz_status_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1281_get_ptz_status_t* pod_1281_get_ptz_status)
{
    _ttalink_pod_1281_get_ptz_status_send_struct(dst_addr, src_addr, chan, pod_1281_get_ptz_status, true);
}

#if TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1281_get_ptz_status_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_mode, uint8_t ack_state, uint16_t ack_pitch, uint16_t ack_roll, uint16_t ack_yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_mode);
    _tta_put_uint8_t(buf, 2, ack_state);
    _tta_put_uint16_t(buf, 3, ack_pitch);
    _tta_put_uint16_t(buf, 5, ack_roll);
    _tta_put_uint16_t(buf, 7, ack_yaw);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS, buf, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_MIN_LEN, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_CRC, nocrc);
#else
    ttalink_pod_1281_get_ptz_status_t *packet = (ttalink_pod_1281_get_ptz_status_t *)msgbuf;
    packet->ack = ack;
    packet->ack_mode = ack_mode;
    packet->ack_state = ack_state;
    packet->ack_pitch = ack_pitch;
    packet->ack_roll = ack_roll;
    packet->ack_yaw = ack_yaw;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS, (const char *)packet, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_MIN_LEN, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1281_get_ptz_status_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_mode, uint8_t ack_state, uint16_t ack_pitch, uint16_t ack_roll, uint16_t ack_yaw)
{
    _ttalink_pod_1281_get_ptz_status_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_mode, ack_state, ack_pitch, ack_roll, ack_yaw, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1281_get_ptz_status_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_mode, uint8_t ack_state, uint16_t ack_pitch, uint16_t ack_roll, uint16_t ack_yaw)
{
    _ttalink_pod_1281_get_ptz_status_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_mode, ack_state, ack_pitch, ack_roll, ack_yaw, true);
}
#endif

#endif

// MESSAGE POD_1281_GET_PTZ_STATUS UNPACKING


/**
 * @brief Get field ack from pod_1281_get_ptz_status message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1281_get_ptz_status_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack_mode from pod_1281_get_ptz_status message
 *
 * @return PTZ mode
												0 :  free mode
												1 :  Follow mode
												2 :  FPV mode..
 */
static inline uint8_t ttalink_pod_1281_get_ptz_status_get_ack_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field ack_state from pod_1281_get_ptz_status message
 *
 * @return PTZ state
												Bit [0] :  installation direction of PTZ
												0 :  downward installation;
												1 :  Install up
												Bit [1] :  yaw limit reached
												0 :  limit not reached;
												1 :  Reach the limit
												Bit [2] :  whether the rolling reaches the limit
												0 :  limit not reached;
												1 :  Reach the limit
												Bit [3] :  whether the pitch reaches the limit
												0 :  limit not reached;
												1 :  Reach the limit.
 */
static inline uint8_t ttalink_pod_1281_get_ptz_status_get_ack_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field ack_pitch from pod_1281_get_ptz_status message
 *
 * @return Current pitch axis angle in 0.1 degrees.
 */
static inline uint16_t ttalink_pod_1281_get_ptz_status_get_ack_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  3);
}

/**
 * @brief Get field ack_roll from pod_1281_get_ptz_status message
 *
 * @return Current roll axis angle in 0.1 degrees.
 */
static inline uint16_t ttalink_pod_1281_get_ptz_status_get_ack_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  5);
}

/**
 * @brief Get field ack_yaw from pod_1281_get_ptz_status message
 *
 * @return Current yaw axis angle in 0.1 degrees.
 */
static inline uint16_t ttalink_pod_1281_get_ptz_status_get_ack_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  7);
}

/**
 * @brief Decode a pod_1281_get_ptz_status message into a struct
 *
 * @param msg The message to decode
 * @param pod_1281_get_ptz_status C-struct to decode the message contents into
 */
static inline void ttalink_pod_1281_get_ptz_status_decode(const ttalink_message_t* msg, ttalink_pod_1281_get_ptz_status_t* pod_1281_get_ptz_status)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1281_get_ptz_status->ack = ttalink_pod_1281_get_ptz_status_get_ack(msg);
    pod_1281_get_ptz_status->ack_mode = ttalink_pod_1281_get_ptz_status_get_ack_mode(msg);
    pod_1281_get_ptz_status->ack_state = ttalink_pod_1281_get_ptz_status_get_ack_state(msg);
    pod_1281_get_ptz_status->ack_pitch = ttalink_pod_1281_get_ptz_status_get_ack_pitch(msg);
    pod_1281_get_ptz_status->ack_roll = ttalink_pod_1281_get_ptz_status_get_ack_roll(msg);
    pod_1281_get_ptz_status->ack_yaw = ttalink_pod_1281_get_ptz_status_get_ack_yaw(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN? msg->len : TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN;
        memset(pod_1281_get_ptz_status, 0, TTALINK_MSG_ID_POD_1281_GET_PTZ_STATUS_LEN);
    memcpy(pod_1281_get_ptz_status, _TTA_PAYLOAD(msg), len);
#endif
}
