#pragma once
// MESSAGE POD_1288_UPDATE_PTZ_CALI_PROGRESS PACKING

#define TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS 11288

TTAPACKED(
typedef struct __ttalink_pod_1288_update_ptz_cali_progress_t {
 uint8_t ack; /*< .*/
 uint8_t ack_progress; /*< Percentage of frame calibration progress, ranging from 0 to 100..*/
 uint8_t ack_cali_state; /*< Cloud platform calibration status
													0 :  Calibration complete;
													1 :  Calibration is in progress;
													2 :  Calibration failed..*/
}) ttalink_pod_1288_update_ptz_cali_progress_t;

#define TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN 3
#define TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_MIN_LEN 3
#define TTALINK_MSG_ID_11288_LEN 3
#define TTALINK_MSG_ID_11288_MIN_LEN 3

#define TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_CRC 184
#define TTALINK_MSG_ID_11288_CRC 184



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1288_UPDATE_PTZ_CALI_PROGRESS { \
    11288, \
    "POD_1288_UPDATE_PTZ_CALI_PROGRESS", \
    3, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1288_update_ptz_cali_progress_t, ack) }, \
         { "ack_progress", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1288_update_ptz_cali_progress_t, ack_progress) }, \
         { "ack_cali_state", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_pod_1288_update_ptz_cali_progress_t, ack_cali_state) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1288_UPDATE_PTZ_CALI_PROGRESS { \
    "POD_1288_UPDATE_PTZ_CALI_PROGRESS", \
    3, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1288_update_ptz_cali_progress_t, ack) }, \
         { "ack_progress", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1288_update_ptz_cali_progress_t, ack_progress) }, \
         { "ack_cali_state", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_pod_1288_update_ptz_cali_progress_t, ack_cali_state) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1288_update_ptz_cali_progress_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_progress, uint8_t ack_cali_state, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_progress);
    _tta_put_uint8_t(buf, 2, ack_cali_state);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN);
#else
    ttalink_pod_1288_update_ptz_cali_progress_t packet;
    packet.ack = ack;
    packet.ack_progress = ack_progress;
    packet.ack_cali_state = ack_cali_state;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_MIN_LEN, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_CRC, nocrc);
}

/**
 * @brief Pack a pod_1288_update_ptz_cali_progress message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param ack_progress Percentage of frame calibration progress, ranging from 0 to 100..
 * @param ack_cali_state Cloud platform calibration status
													0 :  Calibration complete;
													1 :  Calibration is in progress;
													2 :  Calibration failed..
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1288_update_ptz_cali_progress_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_progress, uint8_t ack_cali_state)
{
    return _ttalink_pod_1288_update_ptz_cali_progress_pack(dst_addr, src_addr, msg,  ack, ack_progress, ack_cali_state, false);
}

/**
 * @brief Pack a pod_1288_update_ptz_cali_progress message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param ack_progress Percentage of frame calibration progress, ranging from 0 to 100..
 * @param ack_cali_state Cloud platform calibration status
													0 :  Calibration complete;
													1 :  Calibration is in progress;
													2 :  Calibration failed..
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1288_update_ptz_cali_progress_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_progress, uint8_t ack_cali_state)
{
    return _ttalink_pod_1288_update_ptz_cali_progress_pack(dst_addr, src_addr, msg,  ack, ack_progress, ack_cali_state, true);
}


static inline uint16_t _ttalink_pod_1288_update_ptz_cali_progress_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_progress,uint8_t ack_cali_state, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_progress);
    _tta_put_uint8_t(buf, 2, ack_cali_state);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN);
#else
    ttalink_pod_1288_update_ptz_cali_progress_t packet;
    packet.ack = ack;
    packet.ack_progress = ack_progress;
    packet.ack_cali_state = ack_cali_state;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_MIN_LEN, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_CRC, nocrc);
}

/**
 * @brief Pack a pod_1288_update_ptz_cali_progress message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param ack_progress Percentage of frame calibration progress, ranging from 0 to 100..
 * @param ack_cali_state Cloud platform calibration status
													0 :  Calibration complete;
													1 :  Calibration is in progress;
													2 :  Calibration failed..
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1288_update_ptz_cali_progress_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_progress,uint8_t ack_cali_state)
{
    return _ttalink_pod_1288_update_ptz_cali_progress_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_progress, ack_cali_state, false);
}

/**
 * @brief Pack a pod_1288_update_ptz_cali_progress message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param ack_progress Percentage of frame calibration progress, ranging from 0 to 100..
 * @param ack_cali_state Cloud platform calibration status
													0 :  Calibration complete;
													1 :  Calibration is in progress;
													2 :  Calibration failed..
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1288_update_ptz_cali_progress_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_progress,uint8_t ack_cali_state)
{
    return _ttalink_pod_1288_update_ptz_cali_progress_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_progress, ack_cali_state, true);
}


static inline uint16_t _ttalink_pod_1288_update_ptz_cali_progress_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1288_update_ptz_cali_progress_t* pod_1288_update_ptz_cali_progress, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1288_update_ptz_cali_progress_pack_nocrc(dst_addr, src_addr, msg, pod_1288_update_ptz_cali_progress->ack, pod_1288_update_ptz_cali_progress->ack_progress, pod_1288_update_ptz_cali_progress->ack_cali_state);
    }else{
        return ttalink_pod_1288_update_ptz_cali_progress_pack(dst_addr, src_addr, msg, pod_1288_update_ptz_cali_progress->ack, pod_1288_update_ptz_cali_progress->ack_progress, pod_1288_update_ptz_cali_progress->ack_cali_state);
    }
    
}

/**
 * @brief Encode a pod_1288_update_ptz_cali_progress struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1288_update_ptz_cali_progress C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1288_update_ptz_cali_progress_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1288_update_ptz_cali_progress_t* pod_1288_update_ptz_cali_progress)
{
    return _ttalink_pod_1288_update_ptz_cali_progress_encode(dst_addr, src_addr, msg, pod_1288_update_ptz_cali_progress, false);
}

/**
 * @brief Encode a pod_1288_update_ptz_cali_progress struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1288_update_ptz_cali_progress C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1288_update_ptz_cali_progress_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1288_update_ptz_cali_progress_t* pod_1288_update_ptz_cali_progress)
{
    return _ttalink_pod_1288_update_ptz_cali_progress_encode(dst_addr, src_addr, msg, pod_1288_update_ptz_cali_progress, true);
}


static inline uint16_t _ttalink_pod_1288_update_ptz_cali_progress_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1288_update_ptz_cali_progress_t* pod_1288_update_ptz_cali_progress, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1288_update_ptz_cali_progress_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1288_update_ptz_cali_progress->ack, pod_1288_update_ptz_cali_progress->ack_progress, pod_1288_update_ptz_cali_progress->ack_cali_state);
    }else{
        return ttalink_pod_1288_update_ptz_cali_progress_pack_chan(dst_addr, src_addr, chan, msg, pod_1288_update_ptz_cali_progress->ack, pod_1288_update_ptz_cali_progress->ack_progress, pod_1288_update_ptz_cali_progress->ack_cali_state);
    }
}

/**
 * @brief Encode a pod_1288_update_ptz_cali_progress struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1288_update_ptz_cali_progress C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1288_update_ptz_cali_progress_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1288_update_ptz_cali_progress_t* pod_1288_update_ptz_cali_progress)
{
    return _ttalink_pod_1288_update_ptz_cali_progress_encode_chan(dst_addr, src_addr, chan, msg, pod_1288_update_ptz_cali_progress, false);
}

/**
 * @brief Encode a pod_1288_update_ptz_cali_progress struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1288_update_ptz_cali_progress C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1288_update_ptz_cali_progress_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1288_update_ptz_cali_progress_t* pod_1288_update_ptz_cali_progress)
{
    return _ttalink_pod_1288_update_ptz_cali_progress_encode_chan(dst_addr, src_addr, chan, msg, pod_1288_update_ptz_cali_progress, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1288_update_ptz_cali_progress_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_progress, uint8_t ack_cali_state, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_progress);
    _tta_put_uint8_t(buf, 2, ack_cali_state);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS, buf, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_MIN_LEN, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_CRC, nocrc);
#else
    ttalink_pod_1288_update_ptz_cali_progress_t packet;
    packet.ack = ack;
    packet.ack_progress = ack_progress;
    packet.ack_cali_state = ack_cali_state;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS, (const char *)&packet, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_MIN_LEN, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1288_update_ptz_cali_progress message
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param ack_progress Percentage of frame calibration progress, ranging from 0 to 100..
 * @param ack_cali_state Cloud platform calibration status
													0 :  Calibration complete;
													1 :  Calibration is in progress;
													2 :  Calibration failed..
 */
static inline void ttalink_pod_1288_update_ptz_cali_progress_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_progress, uint8_t ack_cali_state)
{
    _ttalink_pod_1288_update_ptz_cali_progress_send(dst_addr, src_addr, chan, ack, ack_progress, ack_cali_state, false);
}

/**
 * @brief Send a pod_1288_update_ptz_cali_progress message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param ack_progress Percentage of frame calibration progress, ranging from 0 to 100..
 * @param ack_cali_state Cloud platform calibration status
													0 :  Calibration complete;
													1 :  Calibration is in progress;
													2 :  Calibration failed..
 */
static inline void ttalink_pod_1288_update_ptz_cali_progress_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_progress, uint8_t ack_cali_state)
{
    _ttalink_pod_1288_update_ptz_cali_progress_send(dst_addr, src_addr, chan, ack, ack_progress, ack_cali_state, true);
}


static inline void _ttalink_pod_1288_update_ptz_cali_progress_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1288_update_ptz_cali_progress_t* pod_1288_update_ptz_cali_progress, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1288_update_ptz_cali_progress_send_nocrc(dst_addr, src_addr, chan, pod_1288_update_ptz_cali_progress->ack, pod_1288_update_ptz_cali_progress->ack_progress, pod_1288_update_ptz_cali_progress->ack_cali_state);
    }else{
        ttalink_pod_1288_update_ptz_cali_progress_send(dst_addr, src_addr, chan, pod_1288_update_ptz_cali_progress->ack, pod_1288_update_ptz_cali_progress->ack_progress, pod_1288_update_ptz_cali_progress->ack_cali_state);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS, (const char *)pod_1288_update_ptz_cali_progress, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_MIN_LEN, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1288_update_ptz_cali_progress message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1288_update_ptz_cali_progress_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1288_update_ptz_cali_progress_t* pod_1288_update_ptz_cali_progress)
{
    _ttalink_pod_1288_update_ptz_cali_progress_send_struct(dst_addr, src_addr, chan, pod_1288_update_ptz_cali_progress, false);
}

/**
 * @brief Send a pod_1288_update_ptz_cali_progress message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1288_update_ptz_cali_progress_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1288_update_ptz_cali_progress_t* pod_1288_update_ptz_cali_progress)
{
    _ttalink_pod_1288_update_ptz_cali_progress_send_struct(dst_addr, src_addr, chan, pod_1288_update_ptz_cali_progress, true);
}

#if TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1288_update_ptz_cali_progress_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_progress, uint8_t ack_cali_state, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_progress);
    _tta_put_uint8_t(buf, 2, ack_cali_state);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS, buf, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_MIN_LEN, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_CRC, nocrc);
#else
    ttalink_pod_1288_update_ptz_cali_progress_t *packet = (ttalink_pod_1288_update_ptz_cali_progress_t *)msgbuf;
    packet->ack = ack;
    packet->ack_progress = ack_progress;
    packet->ack_cali_state = ack_cali_state;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS, (const char *)packet, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_MIN_LEN, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1288_update_ptz_cali_progress_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_progress, uint8_t ack_cali_state)
{
    _ttalink_pod_1288_update_ptz_cali_progress_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_progress, ack_cali_state, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1288_update_ptz_cali_progress_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_progress, uint8_t ack_cali_state)
{
    _ttalink_pod_1288_update_ptz_cali_progress_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_progress, ack_cali_state, true);
}
#endif

#endif

// MESSAGE POD_1288_UPDATE_PTZ_CALI_PROGRESS UNPACKING


/**
 * @brief Get field ack from pod_1288_update_ptz_cali_progress message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1288_update_ptz_cali_progress_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack_progress from pod_1288_update_ptz_cali_progress message
 *
 * @return Percentage of frame calibration progress, ranging from 0 to 100..
 */
static inline uint8_t ttalink_pod_1288_update_ptz_cali_progress_get_ack_progress(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field ack_cali_state from pod_1288_update_ptz_cali_progress message
 *
 * @return Cloud platform calibration status
													0 :  Calibration complete;
													1 :  Calibration is in progress;
													2 :  Calibration failed..
 */
static inline uint8_t ttalink_pod_1288_update_ptz_cali_progress_get_ack_cali_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a pod_1288_update_ptz_cali_progress message into a struct
 *
 * @param msg The message to decode
 * @param pod_1288_update_ptz_cali_progress C-struct to decode the message contents into
 */
static inline void ttalink_pod_1288_update_ptz_cali_progress_decode(const ttalink_message_t* msg, ttalink_pod_1288_update_ptz_cali_progress_t* pod_1288_update_ptz_cali_progress)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1288_update_ptz_cali_progress->ack = ttalink_pod_1288_update_ptz_cali_progress_get_ack(msg);
    pod_1288_update_ptz_cali_progress->ack_progress = ttalink_pod_1288_update_ptz_cali_progress_get_ack_progress(msg);
    pod_1288_update_ptz_cali_progress->ack_cali_state = ttalink_pod_1288_update_ptz_cali_progress_get_ack_cali_state(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN? msg->len : TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN;
        memset(pod_1288_update_ptz_cali_progress, 0, TTALINK_MSG_ID_POD_1288_UPDATE_PTZ_CALI_PROGRESS_LEN);
    memcpy(pod_1288_update_ptz_cali_progress, _TTA_PAYLOAD(msg), len);
#endif
}
