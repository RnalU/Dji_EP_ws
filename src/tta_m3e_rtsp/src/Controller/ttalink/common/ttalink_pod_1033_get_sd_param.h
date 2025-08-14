#pragma once
// MESSAGE POD_1033_GET_SD_PARAM PACKING

#define TTALINK_MSG_ID_POD_1033_GET_SD_PARAM 11033

TTAPACKED(
typedef struct __ttalink_pod_1033_get_sd_param_t {
 uint8_t ack; /*<  .*/
 uint16_t ack_sd_state; /*<  SD card status
													Bit [0] :  inserted
													Bit [1] :  initializing
													Bit [2] :  read only
													Bit [3] :  formatted
													Bit [4] :  formatting
													Bit [5] :  full
													Bit [6] :  verified
													Bit [7] :  is not a valid format
													Bit [8] :  error
													Bit [9-15] :  reserved*/
 uint32_t ack_sd_capacity; /*< SD card capacity (in MB, LSB in front, MSB in back).*/
 uint32_t ack_sd_remaining_capacity; /*< SD card capacity (in MB, LSB in front, MSB in back).*/
 uint32_t ack_sd_remaining_shoot; /*< (SB in front, MSB in back).*/
 uint32_t ack_sd_remaining_recording_time; /*< Remaining recording time (unit :  second, LSB before, MSB after).*/
}) ttalink_pod_1033_get_sd_param_t;

#define TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN 19
#define TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_MIN_LEN 19
#define TTALINK_MSG_ID_11033_LEN 19
#define TTALINK_MSG_ID_11033_MIN_LEN 19

#define TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_CRC 247
#define TTALINK_MSG_ID_11033_CRC 247



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1033_GET_SD_PARAM { \
    11033, \
    "POD_1033_GET_SD_PARAM", \
    6, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1033_get_sd_param_t, ack) }, \
         { "ack_sd_state", NULL, TTALINK_TYPE_UINT16_T, 0, 1, offsetof(ttalink_pod_1033_get_sd_param_t, ack_sd_state) }, \
         { "ack_sd_capacity", NULL, TTALINK_TYPE_UINT32_T, 0, 3, offsetof(ttalink_pod_1033_get_sd_param_t, ack_sd_capacity) }, \
         { "ack_sd_remaining_capacity", NULL, TTALINK_TYPE_UINT32_T, 0, 7, offsetof(ttalink_pod_1033_get_sd_param_t, ack_sd_remaining_capacity) }, \
         { "ack_sd_remaining_shoot", NULL, TTALINK_TYPE_UINT32_T, 0, 11, offsetof(ttalink_pod_1033_get_sd_param_t, ack_sd_remaining_shoot) }, \
         { "ack_sd_remaining_recording_time", NULL, TTALINK_TYPE_UINT32_T, 0, 15, offsetof(ttalink_pod_1033_get_sd_param_t, ack_sd_remaining_recording_time) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1033_GET_SD_PARAM { \
    "POD_1033_GET_SD_PARAM", \
    6, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1033_get_sd_param_t, ack) }, \
         { "ack_sd_state", NULL, TTALINK_TYPE_UINT16_T, 0, 1, offsetof(ttalink_pod_1033_get_sd_param_t, ack_sd_state) }, \
         { "ack_sd_capacity", NULL, TTALINK_TYPE_UINT32_T, 0, 3, offsetof(ttalink_pod_1033_get_sd_param_t, ack_sd_capacity) }, \
         { "ack_sd_remaining_capacity", NULL, TTALINK_TYPE_UINT32_T, 0, 7, offsetof(ttalink_pod_1033_get_sd_param_t, ack_sd_remaining_capacity) }, \
         { "ack_sd_remaining_shoot", NULL, TTALINK_TYPE_UINT32_T, 0, 11, offsetof(ttalink_pod_1033_get_sd_param_t, ack_sd_remaining_shoot) }, \
         { "ack_sd_remaining_recording_time", NULL, TTALINK_TYPE_UINT32_T, 0, 15, offsetof(ttalink_pod_1033_get_sd_param_t, ack_sd_remaining_recording_time) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1033_get_sd_param_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint16_t ack_sd_state, uint32_t ack_sd_capacity, uint32_t ack_sd_remaining_capacity, uint32_t ack_sd_remaining_shoot, uint32_t ack_sd_remaining_recording_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint16_t(buf, 1, ack_sd_state);
    _tta_put_uint32_t(buf, 3, ack_sd_capacity);
    _tta_put_uint32_t(buf, 7, ack_sd_remaining_capacity);
    _tta_put_uint32_t(buf, 11, ack_sd_remaining_shoot);
    _tta_put_uint32_t(buf, 15, ack_sd_remaining_recording_time);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN);
#else
    ttalink_pod_1033_get_sd_param_t packet;
    packet.ack = ack;
    packet.ack_sd_state = ack_sd_state;
    packet.ack_sd_capacity = ack_sd_capacity;
    packet.ack_sd_remaining_capacity = ack_sd_remaining_capacity;
    packet.ack_sd_remaining_shoot = ack_sd_remaining_shoot;
    packet.ack_sd_remaining_recording_time = ack_sd_remaining_recording_time;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1033_GET_SD_PARAM;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_MIN_LEN, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_CRC, nocrc);
}

/**
 * @brief Pack a pod_1033_get_sd_param message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack  .
 * @param ack_sd_state  SD card status
													Bit [0] :  inserted
													Bit [1] :  initializing
													Bit [2] :  read only
													Bit [3] :  formatted
													Bit [4] :  formatting
													Bit [5] :  full
													Bit [6] :  verified
													Bit [7] :  is not a valid format
													Bit [8] :  error
													Bit [9-15] :  reserved
 * @param ack_sd_capacity SD card capacity (in MB, LSB in front, MSB in back).
 * @param ack_sd_remaining_capacity SD card capacity (in MB, LSB in front, MSB in back).
 * @param ack_sd_remaining_shoot (SB in front, MSB in back).
 * @param ack_sd_remaining_recording_time Remaining recording time (unit :  second, LSB before, MSB after).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1033_get_sd_param_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint16_t ack_sd_state, uint32_t ack_sd_capacity, uint32_t ack_sd_remaining_capacity, uint32_t ack_sd_remaining_shoot, uint32_t ack_sd_remaining_recording_time)
{
    return _ttalink_pod_1033_get_sd_param_pack(dst_addr, src_addr, msg,  ack, ack_sd_state, ack_sd_capacity, ack_sd_remaining_capacity, ack_sd_remaining_shoot, ack_sd_remaining_recording_time, false);
}

/**
 * @brief Pack a pod_1033_get_sd_param message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack  .
 * @param ack_sd_state  SD card status
													Bit [0] :  inserted
													Bit [1] :  initializing
													Bit [2] :  read only
													Bit [3] :  formatted
													Bit [4] :  formatting
													Bit [5] :  full
													Bit [6] :  verified
													Bit [7] :  is not a valid format
													Bit [8] :  error
													Bit [9-15] :  reserved
 * @param ack_sd_capacity SD card capacity (in MB, LSB in front, MSB in back).
 * @param ack_sd_remaining_capacity SD card capacity (in MB, LSB in front, MSB in back).
 * @param ack_sd_remaining_shoot (SB in front, MSB in back).
 * @param ack_sd_remaining_recording_time Remaining recording time (unit :  second, LSB before, MSB after).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1033_get_sd_param_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint16_t ack_sd_state, uint32_t ack_sd_capacity, uint32_t ack_sd_remaining_capacity, uint32_t ack_sd_remaining_shoot, uint32_t ack_sd_remaining_recording_time)
{
    return _ttalink_pod_1033_get_sd_param_pack(dst_addr, src_addr, msg,  ack, ack_sd_state, ack_sd_capacity, ack_sd_remaining_capacity, ack_sd_remaining_shoot, ack_sd_remaining_recording_time, true);
}


static inline uint16_t _ttalink_pod_1033_get_sd_param_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint16_t ack_sd_state,uint32_t ack_sd_capacity,uint32_t ack_sd_remaining_capacity,uint32_t ack_sd_remaining_shoot,uint32_t ack_sd_remaining_recording_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint16_t(buf, 1, ack_sd_state);
    _tta_put_uint32_t(buf, 3, ack_sd_capacity);
    _tta_put_uint32_t(buf, 7, ack_sd_remaining_capacity);
    _tta_put_uint32_t(buf, 11, ack_sd_remaining_shoot);
    _tta_put_uint32_t(buf, 15, ack_sd_remaining_recording_time);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN);
#else
    ttalink_pod_1033_get_sd_param_t packet;
    packet.ack = ack;
    packet.ack_sd_state = ack_sd_state;
    packet.ack_sd_capacity = ack_sd_capacity;
    packet.ack_sd_remaining_capacity = ack_sd_remaining_capacity;
    packet.ack_sd_remaining_shoot = ack_sd_remaining_shoot;
    packet.ack_sd_remaining_recording_time = ack_sd_remaining_recording_time;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1033_GET_SD_PARAM;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_MIN_LEN, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_CRC, nocrc);
}

/**
 * @brief Pack a pod_1033_get_sd_param message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack  .
 * @param ack_sd_state  SD card status
													Bit [0] :  inserted
													Bit [1] :  initializing
													Bit [2] :  read only
													Bit [3] :  formatted
													Bit [4] :  formatting
													Bit [5] :  full
													Bit [6] :  verified
													Bit [7] :  is not a valid format
													Bit [8] :  error
													Bit [9-15] :  reserved
 * @param ack_sd_capacity SD card capacity (in MB, LSB in front, MSB in back).
 * @param ack_sd_remaining_capacity SD card capacity (in MB, LSB in front, MSB in back).
 * @param ack_sd_remaining_shoot (SB in front, MSB in back).
 * @param ack_sd_remaining_recording_time Remaining recording time (unit :  second, LSB before, MSB after).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1033_get_sd_param_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint16_t ack_sd_state,uint32_t ack_sd_capacity,uint32_t ack_sd_remaining_capacity,uint32_t ack_sd_remaining_shoot,uint32_t ack_sd_remaining_recording_time)
{
    return _ttalink_pod_1033_get_sd_param_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_sd_state, ack_sd_capacity, ack_sd_remaining_capacity, ack_sd_remaining_shoot, ack_sd_remaining_recording_time, false);
}

/**
 * @brief Pack a pod_1033_get_sd_param message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack  .
 * @param ack_sd_state  SD card status
													Bit [0] :  inserted
													Bit [1] :  initializing
													Bit [2] :  read only
													Bit [3] :  formatted
													Bit [4] :  formatting
													Bit [5] :  full
													Bit [6] :  verified
													Bit [7] :  is not a valid format
													Bit [8] :  error
													Bit [9-15] :  reserved
 * @param ack_sd_capacity SD card capacity (in MB, LSB in front, MSB in back).
 * @param ack_sd_remaining_capacity SD card capacity (in MB, LSB in front, MSB in back).
 * @param ack_sd_remaining_shoot (SB in front, MSB in back).
 * @param ack_sd_remaining_recording_time Remaining recording time (unit :  second, LSB before, MSB after).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1033_get_sd_param_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint16_t ack_sd_state,uint32_t ack_sd_capacity,uint32_t ack_sd_remaining_capacity,uint32_t ack_sd_remaining_shoot,uint32_t ack_sd_remaining_recording_time)
{
    return _ttalink_pod_1033_get_sd_param_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_sd_state, ack_sd_capacity, ack_sd_remaining_capacity, ack_sd_remaining_shoot, ack_sd_remaining_recording_time, true);
}


static inline uint16_t _ttalink_pod_1033_get_sd_param_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1033_get_sd_param_t* pod_1033_get_sd_param, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1033_get_sd_param_pack_nocrc(dst_addr, src_addr, msg, pod_1033_get_sd_param->ack, pod_1033_get_sd_param->ack_sd_state, pod_1033_get_sd_param->ack_sd_capacity, pod_1033_get_sd_param->ack_sd_remaining_capacity, pod_1033_get_sd_param->ack_sd_remaining_shoot, pod_1033_get_sd_param->ack_sd_remaining_recording_time);
    }else{
        return ttalink_pod_1033_get_sd_param_pack(dst_addr, src_addr, msg, pod_1033_get_sd_param->ack, pod_1033_get_sd_param->ack_sd_state, pod_1033_get_sd_param->ack_sd_capacity, pod_1033_get_sd_param->ack_sd_remaining_capacity, pod_1033_get_sd_param->ack_sd_remaining_shoot, pod_1033_get_sd_param->ack_sd_remaining_recording_time);
    }
    
}

/**
 * @brief Encode a pod_1033_get_sd_param struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1033_get_sd_param C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1033_get_sd_param_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1033_get_sd_param_t* pod_1033_get_sd_param)
{
    return _ttalink_pod_1033_get_sd_param_encode(dst_addr, src_addr, msg, pod_1033_get_sd_param, false);
}

/**
 * @brief Encode a pod_1033_get_sd_param struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1033_get_sd_param C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1033_get_sd_param_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1033_get_sd_param_t* pod_1033_get_sd_param)
{
    return _ttalink_pod_1033_get_sd_param_encode(dst_addr, src_addr, msg, pod_1033_get_sd_param, true);
}


static inline uint16_t _ttalink_pod_1033_get_sd_param_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1033_get_sd_param_t* pod_1033_get_sd_param, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1033_get_sd_param_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1033_get_sd_param->ack, pod_1033_get_sd_param->ack_sd_state, pod_1033_get_sd_param->ack_sd_capacity, pod_1033_get_sd_param->ack_sd_remaining_capacity, pod_1033_get_sd_param->ack_sd_remaining_shoot, pod_1033_get_sd_param->ack_sd_remaining_recording_time);
    }else{
        return ttalink_pod_1033_get_sd_param_pack_chan(dst_addr, src_addr, chan, msg, pod_1033_get_sd_param->ack, pod_1033_get_sd_param->ack_sd_state, pod_1033_get_sd_param->ack_sd_capacity, pod_1033_get_sd_param->ack_sd_remaining_capacity, pod_1033_get_sd_param->ack_sd_remaining_shoot, pod_1033_get_sd_param->ack_sd_remaining_recording_time);
    }
}

/**
 * @brief Encode a pod_1033_get_sd_param struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1033_get_sd_param C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1033_get_sd_param_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1033_get_sd_param_t* pod_1033_get_sd_param)
{
    return _ttalink_pod_1033_get_sd_param_encode_chan(dst_addr, src_addr, chan, msg, pod_1033_get_sd_param, false);
}

/**
 * @brief Encode a pod_1033_get_sd_param struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1033_get_sd_param C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1033_get_sd_param_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1033_get_sd_param_t* pod_1033_get_sd_param)
{
    return _ttalink_pod_1033_get_sd_param_encode_chan(dst_addr, src_addr, chan, msg, pod_1033_get_sd_param, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1033_get_sd_param_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint16_t ack_sd_state, uint32_t ack_sd_capacity, uint32_t ack_sd_remaining_capacity, uint32_t ack_sd_remaining_shoot, uint32_t ack_sd_remaining_recording_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint16_t(buf, 1, ack_sd_state);
    _tta_put_uint32_t(buf, 3, ack_sd_capacity);
    _tta_put_uint32_t(buf, 7, ack_sd_remaining_capacity);
    _tta_put_uint32_t(buf, 11, ack_sd_remaining_shoot);
    _tta_put_uint32_t(buf, 15, ack_sd_remaining_recording_time);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM, buf, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_MIN_LEN, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_CRC, nocrc);
#else
    ttalink_pod_1033_get_sd_param_t packet;
    packet.ack = ack;
    packet.ack_sd_state = ack_sd_state;
    packet.ack_sd_capacity = ack_sd_capacity;
    packet.ack_sd_remaining_capacity = ack_sd_remaining_capacity;
    packet.ack_sd_remaining_shoot = ack_sd_remaining_shoot;
    packet.ack_sd_remaining_recording_time = ack_sd_remaining_recording_time;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM, (const char *)&packet, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_MIN_LEN, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1033_get_sd_param message
 * @param chan TTAlink channel to send the message
 *
 * @param ack  .
 * @param ack_sd_state  SD card status
													Bit [0] :  inserted
													Bit [1] :  initializing
													Bit [2] :  read only
													Bit [3] :  formatted
													Bit [4] :  formatting
													Bit [5] :  full
													Bit [6] :  verified
													Bit [7] :  is not a valid format
													Bit [8] :  error
													Bit [9-15] :  reserved
 * @param ack_sd_capacity SD card capacity (in MB, LSB in front, MSB in back).
 * @param ack_sd_remaining_capacity SD card capacity (in MB, LSB in front, MSB in back).
 * @param ack_sd_remaining_shoot (SB in front, MSB in back).
 * @param ack_sd_remaining_recording_time Remaining recording time (unit :  second, LSB before, MSB after).
 */
static inline void ttalink_pod_1033_get_sd_param_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint16_t ack_sd_state, uint32_t ack_sd_capacity, uint32_t ack_sd_remaining_capacity, uint32_t ack_sd_remaining_shoot, uint32_t ack_sd_remaining_recording_time)
{
    _ttalink_pod_1033_get_sd_param_send(dst_addr, src_addr, chan, ack, ack_sd_state, ack_sd_capacity, ack_sd_remaining_capacity, ack_sd_remaining_shoot, ack_sd_remaining_recording_time, false);
}

/**
 * @brief Send a pod_1033_get_sd_param message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack  .
 * @param ack_sd_state  SD card status
													Bit [0] :  inserted
													Bit [1] :  initializing
													Bit [2] :  read only
													Bit [3] :  formatted
													Bit [4] :  formatting
													Bit [5] :  full
													Bit [6] :  verified
													Bit [7] :  is not a valid format
													Bit [8] :  error
													Bit [9-15] :  reserved
 * @param ack_sd_capacity SD card capacity (in MB, LSB in front, MSB in back).
 * @param ack_sd_remaining_capacity SD card capacity (in MB, LSB in front, MSB in back).
 * @param ack_sd_remaining_shoot (SB in front, MSB in back).
 * @param ack_sd_remaining_recording_time Remaining recording time (unit :  second, LSB before, MSB after).
 */
static inline void ttalink_pod_1033_get_sd_param_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint16_t ack_sd_state, uint32_t ack_sd_capacity, uint32_t ack_sd_remaining_capacity, uint32_t ack_sd_remaining_shoot, uint32_t ack_sd_remaining_recording_time)
{
    _ttalink_pod_1033_get_sd_param_send(dst_addr, src_addr, chan, ack, ack_sd_state, ack_sd_capacity, ack_sd_remaining_capacity, ack_sd_remaining_shoot, ack_sd_remaining_recording_time, true);
}


static inline void _ttalink_pod_1033_get_sd_param_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1033_get_sd_param_t* pod_1033_get_sd_param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1033_get_sd_param_send_nocrc(dst_addr, src_addr, chan, pod_1033_get_sd_param->ack, pod_1033_get_sd_param->ack_sd_state, pod_1033_get_sd_param->ack_sd_capacity, pod_1033_get_sd_param->ack_sd_remaining_capacity, pod_1033_get_sd_param->ack_sd_remaining_shoot, pod_1033_get_sd_param->ack_sd_remaining_recording_time);
    }else{
        ttalink_pod_1033_get_sd_param_send(dst_addr, src_addr, chan, pod_1033_get_sd_param->ack, pod_1033_get_sd_param->ack_sd_state, pod_1033_get_sd_param->ack_sd_capacity, pod_1033_get_sd_param->ack_sd_remaining_capacity, pod_1033_get_sd_param->ack_sd_remaining_shoot, pod_1033_get_sd_param->ack_sd_remaining_recording_time);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM, (const char *)pod_1033_get_sd_param, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_MIN_LEN, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1033_get_sd_param message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1033_get_sd_param_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1033_get_sd_param_t* pod_1033_get_sd_param)
{
    _ttalink_pod_1033_get_sd_param_send_struct(dst_addr, src_addr, chan, pod_1033_get_sd_param, false);
}

/**
 * @brief Send a pod_1033_get_sd_param message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1033_get_sd_param_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1033_get_sd_param_t* pod_1033_get_sd_param)
{
    _ttalink_pod_1033_get_sd_param_send_struct(dst_addr, src_addr, chan, pod_1033_get_sd_param, true);
}

#if TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1033_get_sd_param_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint16_t ack_sd_state, uint32_t ack_sd_capacity, uint32_t ack_sd_remaining_capacity, uint32_t ack_sd_remaining_shoot, uint32_t ack_sd_remaining_recording_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint16_t(buf, 1, ack_sd_state);
    _tta_put_uint32_t(buf, 3, ack_sd_capacity);
    _tta_put_uint32_t(buf, 7, ack_sd_remaining_capacity);
    _tta_put_uint32_t(buf, 11, ack_sd_remaining_shoot);
    _tta_put_uint32_t(buf, 15, ack_sd_remaining_recording_time);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM, buf, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_MIN_LEN, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_CRC, nocrc);
#else
    ttalink_pod_1033_get_sd_param_t *packet = (ttalink_pod_1033_get_sd_param_t *)msgbuf;
    packet->ack = ack;
    packet->ack_sd_state = ack_sd_state;
    packet->ack_sd_capacity = ack_sd_capacity;
    packet->ack_sd_remaining_capacity = ack_sd_remaining_capacity;
    packet->ack_sd_remaining_shoot = ack_sd_remaining_shoot;
    packet->ack_sd_remaining_recording_time = ack_sd_remaining_recording_time;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM, (const char *)packet, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_MIN_LEN, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1033_get_sd_param_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint16_t ack_sd_state, uint32_t ack_sd_capacity, uint32_t ack_sd_remaining_capacity, uint32_t ack_sd_remaining_shoot, uint32_t ack_sd_remaining_recording_time)
{
    _ttalink_pod_1033_get_sd_param_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_sd_state, ack_sd_capacity, ack_sd_remaining_capacity, ack_sd_remaining_shoot, ack_sd_remaining_recording_time, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1033_get_sd_param_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint16_t ack_sd_state, uint32_t ack_sd_capacity, uint32_t ack_sd_remaining_capacity, uint32_t ack_sd_remaining_shoot, uint32_t ack_sd_remaining_recording_time)
{
    _ttalink_pod_1033_get_sd_param_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_sd_state, ack_sd_capacity, ack_sd_remaining_capacity, ack_sd_remaining_shoot, ack_sd_remaining_recording_time, true);
}
#endif

#endif

// MESSAGE POD_1033_GET_SD_PARAM UNPACKING


/**
 * @brief Get field ack from pod_1033_get_sd_param message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_1033_get_sd_param_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack_sd_state from pod_1033_get_sd_param message
 *
 * @return  SD card status
													Bit [0] :  inserted
													Bit [1] :  initializing
													Bit [2] :  read only
													Bit [3] :  formatted
													Bit [4] :  formatting
													Bit [5] :  full
													Bit [6] :  verified
													Bit [7] :  is not a valid format
													Bit [8] :  error
													Bit [9-15] :  reserved
 */
static inline uint16_t ttalink_pod_1033_get_sd_param_get_ack_sd_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  1);
}

/**
 * @brief Get field ack_sd_capacity from pod_1033_get_sd_param message
 *
 * @return SD card capacity (in MB, LSB in front, MSB in back).
 */
static inline uint32_t ttalink_pod_1033_get_sd_param_get_ack_sd_capacity(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  3);
}

/**
 * @brief Get field ack_sd_remaining_capacity from pod_1033_get_sd_param message
 *
 * @return SD card capacity (in MB, LSB in front, MSB in back).
 */
static inline uint32_t ttalink_pod_1033_get_sd_param_get_ack_sd_remaining_capacity(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  7);
}

/**
 * @brief Get field ack_sd_remaining_shoot from pod_1033_get_sd_param message
 *
 * @return (SB in front, MSB in back).
 */
static inline uint32_t ttalink_pod_1033_get_sd_param_get_ack_sd_remaining_shoot(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  11);
}

/**
 * @brief Get field ack_sd_remaining_recording_time from pod_1033_get_sd_param message
 *
 * @return Remaining recording time (unit :  second, LSB before, MSB after).
 */
static inline uint32_t ttalink_pod_1033_get_sd_param_get_ack_sd_remaining_recording_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  15);
}

/**
 * @brief Decode a pod_1033_get_sd_param message into a struct
 *
 * @param msg The message to decode
 * @param pod_1033_get_sd_param C-struct to decode the message contents into
 */
static inline void ttalink_pod_1033_get_sd_param_decode(const ttalink_message_t* msg, ttalink_pod_1033_get_sd_param_t* pod_1033_get_sd_param)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1033_get_sd_param->ack = ttalink_pod_1033_get_sd_param_get_ack(msg);
    pod_1033_get_sd_param->ack_sd_state = ttalink_pod_1033_get_sd_param_get_ack_sd_state(msg);
    pod_1033_get_sd_param->ack_sd_capacity = ttalink_pod_1033_get_sd_param_get_ack_sd_capacity(msg);
    pod_1033_get_sd_param->ack_sd_remaining_capacity = ttalink_pod_1033_get_sd_param_get_ack_sd_remaining_capacity(msg);
    pod_1033_get_sd_param->ack_sd_remaining_shoot = ttalink_pod_1033_get_sd_param_get_ack_sd_remaining_shoot(msg);
    pod_1033_get_sd_param->ack_sd_remaining_recording_time = ttalink_pod_1033_get_sd_param_get_ack_sd_remaining_recording_time(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN? msg->len : TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN;
        memset(pod_1033_get_sd_param, 0, TTALINK_MSG_ID_POD_1033_GET_SD_PARAM_LEN);
    memcpy(pod_1033_get_sd_param, _TTA_PAYLOAD(msg), len);
#endif
}
