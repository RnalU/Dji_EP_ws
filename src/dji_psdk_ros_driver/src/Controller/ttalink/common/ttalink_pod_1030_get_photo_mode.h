#pragma once
// MESSAGE POD_1030_GET_PHOTO_MODE PACKING

#define TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE 11030

TTAPACKED(
typedef struct __ttalink_pod_1030_get_photo_mode_t {
 uint8_t ack; /*<  */
 uint8_t ack_type; /*<  Uint8, photo type
													1 :  Single shot;
													4 :  Continuous shooting;
													6 :  Interval beat;.*/
 uint16_t ack_shootting_number; /*<  Number of consecutive shots (effective when the photo type is continuous)*/
 uint8_t ack_timer_camera_number; /*<  Uint8, number of photos taken regularly (effective when the type of photos taken is intermittent)
																	0 :  reservation
																	1-254 :  number of intervals;
																	255 :  keep shooting until the camera receives the stop interval shooting command;*/
 uint8_t ack_photo_interval; /*<  Time interval between shots (effective when the type of shot is interval shot)
															0 :  reservation
															1-255 :  time interval between shots (in seconds)*/
}) ttalink_pod_1030_get_photo_mode_t;

#define TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN 6
#define TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_MIN_LEN 6
#define TTALINK_MSG_ID_11030_LEN 6
#define TTALINK_MSG_ID_11030_MIN_LEN 6

#define TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_CRC 71
#define TTALINK_MSG_ID_11030_CRC 71



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1030_GET_PHOTO_MODE { \
    11030, \
    "POD_1030_GET_PHOTO_MODE", \
    5, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1030_get_photo_mode_t, ack) }, \
         { "ack_type", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1030_get_photo_mode_t, ack_type) }, \
         { "ack_shootting_number", NULL, TTALINK_TYPE_UINT16_T, 0, 2, offsetof(ttalink_pod_1030_get_photo_mode_t, ack_shootting_number) }, \
         { "ack_timer_camera_number", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_pod_1030_get_photo_mode_t, ack_timer_camera_number) }, \
         { "ack_photo_interval", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_pod_1030_get_photo_mode_t, ack_photo_interval) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1030_GET_PHOTO_MODE { \
    "POD_1030_GET_PHOTO_MODE", \
    5, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1030_get_photo_mode_t, ack) }, \
         { "ack_type", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1030_get_photo_mode_t, ack_type) }, \
         { "ack_shootting_number", NULL, TTALINK_TYPE_UINT16_T, 0, 2, offsetof(ttalink_pod_1030_get_photo_mode_t, ack_shootting_number) }, \
         { "ack_timer_camera_number", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_pod_1030_get_photo_mode_t, ack_timer_camera_number) }, \
         { "ack_photo_interval", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_pod_1030_get_photo_mode_t, ack_photo_interval) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1030_get_photo_mode_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_type, uint16_t ack_shootting_number, uint8_t ack_timer_camera_number, uint8_t ack_photo_interval, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_type);
    _tta_put_uint16_t(buf, 2, ack_shootting_number);
    _tta_put_uint8_t(buf, 4, ack_timer_camera_number);
    _tta_put_uint8_t(buf, 5, ack_photo_interval);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN);
#else
    ttalink_pod_1030_get_photo_mode_t packet;
    packet.ack = ack;
    packet.ack_type = ack_type;
    packet.ack_shootting_number = ack_shootting_number;
    packet.ack_timer_camera_number = ack_timer_camera_number;
    packet.ack_photo_interval = ack_photo_interval;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_CRC, nocrc);
}

/**
 * @brief Pack a pod_1030_get_photo_mode message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack  
 * @param ack_type  Uint8, photo type
													1 :  Single shot;
													4 :  Continuous shooting;
													6 :  Interval beat;.
 * @param ack_shootting_number  Number of consecutive shots (effective when the photo type is continuous)
 * @param ack_timer_camera_number  Uint8, number of photos taken regularly (effective when the type of photos taken is intermittent)
																	0 :  reservation
																	1-254 :  number of intervals;
																	255 :  keep shooting until the camera receives the stop interval shooting command;
 * @param ack_photo_interval  Time interval between shots (effective when the type of shot is interval shot)
															0 :  reservation
															1-255 :  time interval between shots (in seconds)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1030_get_photo_mode_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_type, uint16_t ack_shootting_number, uint8_t ack_timer_camera_number, uint8_t ack_photo_interval)
{
    return _ttalink_pod_1030_get_photo_mode_pack(dst_addr, src_addr, msg,  ack, ack_type, ack_shootting_number, ack_timer_camera_number, ack_photo_interval, false);
}

/**
 * @brief Pack a pod_1030_get_photo_mode message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack  
 * @param ack_type  Uint8, photo type
													1 :  Single shot;
													4 :  Continuous shooting;
													6 :  Interval beat;.
 * @param ack_shootting_number  Number of consecutive shots (effective when the photo type is continuous)
 * @param ack_timer_camera_number  Uint8, number of photos taken regularly (effective when the type of photos taken is intermittent)
																	0 :  reservation
																	1-254 :  number of intervals;
																	255 :  keep shooting until the camera receives the stop interval shooting command;
 * @param ack_photo_interval  Time interval between shots (effective when the type of shot is interval shot)
															0 :  reservation
															1-255 :  time interval between shots (in seconds)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1030_get_photo_mode_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_type, uint16_t ack_shootting_number, uint8_t ack_timer_camera_number, uint8_t ack_photo_interval)
{
    return _ttalink_pod_1030_get_photo_mode_pack(dst_addr, src_addr, msg,  ack, ack_type, ack_shootting_number, ack_timer_camera_number, ack_photo_interval, true);
}


static inline uint16_t _ttalink_pod_1030_get_photo_mode_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_type,uint16_t ack_shootting_number,uint8_t ack_timer_camera_number,uint8_t ack_photo_interval, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_type);
    _tta_put_uint16_t(buf, 2, ack_shootting_number);
    _tta_put_uint8_t(buf, 4, ack_timer_camera_number);
    _tta_put_uint8_t(buf, 5, ack_photo_interval);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN);
#else
    ttalink_pod_1030_get_photo_mode_t packet;
    packet.ack = ack;
    packet.ack_type = ack_type;
    packet.ack_shootting_number = ack_shootting_number;
    packet.ack_timer_camera_number = ack_timer_camera_number;
    packet.ack_photo_interval = ack_photo_interval;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_CRC, nocrc);
}

/**
 * @brief Pack a pod_1030_get_photo_mode message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack  
 * @param ack_type  Uint8, photo type
													1 :  Single shot;
													4 :  Continuous shooting;
													6 :  Interval beat;.
 * @param ack_shootting_number  Number of consecutive shots (effective when the photo type is continuous)
 * @param ack_timer_camera_number  Uint8, number of photos taken regularly (effective when the type of photos taken is intermittent)
																	0 :  reservation
																	1-254 :  number of intervals;
																	255 :  keep shooting until the camera receives the stop interval shooting command;
 * @param ack_photo_interval  Time interval between shots (effective when the type of shot is interval shot)
															0 :  reservation
															1-255 :  time interval between shots (in seconds)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1030_get_photo_mode_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_type,uint16_t ack_shootting_number,uint8_t ack_timer_camera_number,uint8_t ack_photo_interval)
{
    return _ttalink_pod_1030_get_photo_mode_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_type, ack_shootting_number, ack_timer_camera_number, ack_photo_interval, false);
}

/**
 * @brief Pack a pod_1030_get_photo_mode message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack  
 * @param ack_type  Uint8, photo type
													1 :  Single shot;
													4 :  Continuous shooting;
													6 :  Interval beat;.
 * @param ack_shootting_number  Number of consecutive shots (effective when the photo type is continuous)
 * @param ack_timer_camera_number  Uint8, number of photos taken regularly (effective when the type of photos taken is intermittent)
																	0 :  reservation
																	1-254 :  number of intervals;
																	255 :  keep shooting until the camera receives the stop interval shooting command;
 * @param ack_photo_interval  Time interval between shots (effective when the type of shot is interval shot)
															0 :  reservation
															1-255 :  time interval between shots (in seconds)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1030_get_photo_mode_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_type,uint16_t ack_shootting_number,uint8_t ack_timer_camera_number,uint8_t ack_photo_interval)
{
    return _ttalink_pod_1030_get_photo_mode_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_type, ack_shootting_number, ack_timer_camera_number, ack_photo_interval, true);
}


static inline uint16_t _ttalink_pod_1030_get_photo_mode_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1030_get_photo_mode_t* pod_1030_get_photo_mode, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1030_get_photo_mode_pack_nocrc(dst_addr, src_addr, msg, pod_1030_get_photo_mode->ack, pod_1030_get_photo_mode->ack_type, pod_1030_get_photo_mode->ack_shootting_number, pod_1030_get_photo_mode->ack_timer_camera_number, pod_1030_get_photo_mode->ack_photo_interval);
    }else{
        return ttalink_pod_1030_get_photo_mode_pack(dst_addr, src_addr, msg, pod_1030_get_photo_mode->ack, pod_1030_get_photo_mode->ack_type, pod_1030_get_photo_mode->ack_shootting_number, pod_1030_get_photo_mode->ack_timer_camera_number, pod_1030_get_photo_mode->ack_photo_interval);
    }
    
}

/**
 * @brief Encode a pod_1030_get_photo_mode struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1030_get_photo_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1030_get_photo_mode_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1030_get_photo_mode_t* pod_1030_get_photo_mode)
{
    return _ttalink_pod_1030_get_photo_mode_encode(dst_addr, src_addr, msg, pod_1030_get_photo_mode, false);
}

/**
 * @brief Encode a pod_1030_get_photo_mode struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1030_get_photo_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1030_get_photo_mode_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1030_get_photo_mode_t* pod_1030_get_photo_mode)
{
    return _ttalink_pod_1030_get_photo_mode_encode(dst_addr, src_addr, msg, pod_1030_get_photo_mode, true);
}


static inline uint16_t _ttalink_pod_1030_get_photo_mode_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1030_get_photo_mode_t* pod_1030_get_photo_mode, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1030_get_photo_mode_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1030_get_photo_mode->ack, pod_1030_get_photo_mode->ack_type, pod_1030_get_photo_mode->ack_shootting_number, pod_1030_get_photo_mode->ack_timer_camera_number, pod_1030_get_photo_mode->ack_photo_interval);
    }else{
        return ttalink_pod_1030_get_photo_mode_pack_chan(dst_addr, src_addr, chan, msg, pod_1030_get_photo_mode->ack, pod_1030_get_photo_mode->ack_type, pod_1030_get_photo_mode->ack_shootting_number, pod_1030_get_photo_mode->ack_timer_camera_number, pod_1030_get_photo_mode->ack_photo_interval);
    }
}

/**
 * @brief Encode a pod_1030_get_photo_mode struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1030_get_photo_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1030_get_photo_mode_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1030_get_photo_mode_t* pod_1030_get_photo_mode)
{
    return _ttalink_pod_1030_get_photo_mode_encode_chan(dst_addr, src_addr, chan, msg, pod_1030_get_photo_mode, false);
}

/**
 * @brief Encode a pod_1030_get_photo_mode struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1030_get_photo_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1030_get_photo_mode_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1030_get_photo_mode_t* pod_1030_get_photo_mode)
{
    return _ttalink_pod_1030_get_photo_mode_encode_chan(dst_addr, src_addr, chan, msg, pod_1030_get_photo_mode, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1030_get_photo_mode_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_type, uint16_t ack_shootting_number, uint8_t ack_timer_camera_number, uint8_t ack_photo_interval, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_type);
    _tta_put_uint16_t(buf, 2, ack_shootting_number);
    _tta_put_uint8_t(buf, 4, ack_timer_camera_number);
    _tta_put_uint8_t(buf, 5, ack_photo_interval);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE, buf, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_CRC, nocrc);
#else
    ttalink_pod_1030_get_photo_mode_t packet;
    packet.ack = ack;
    packet.ack_type = ack_type;
    packet.ack_shootting_number = ack_shootting_number;
    packet.ack_timer_camera_number = ack_timer_camera_number;
    packet.ack_photo_interval = ack_photo_interval;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE, (const char *)&packet, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1030_get_photo_mode message
 * @param chan TTAlink channel to send the message
 *
 * @param ack  
 * @param ack_type  Uint8, photo type
													1 :  Single shot;
													4 :  Continuous shooting;
													6 :  Interval beat;.
 * @param ack_shootting_number  Number of consecutive shots (effective when the photo type is continuous)
 * @param ack_timer_camera_number  Uint8, number of photos taken regularly (effective when the type of photos taken is intermittent)
																	0 :  reservation
																	1-254 :  number of intervals;
																	255 :  keep shooting until the camera receives the stop interval shooting command;
 * @param ack_photo_interval  Time interval between shots (effective when the type of shot is interval shot)
															0 :  reservation
															1-255 :  time interval between shots (in seconds)
 */
static inline void ttalink_pod_1030_get_photo_mode_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_type, uint16_t ack_shootting_number, uint8_t ack_timer_camera_number, uint8_t ack_photo_interval)
{
    _ttalink_pod_1030_get_photo_mode_send(dst_addr, src_addr, chan, ack, ack_type, ack_shootting_number, ack_timer_camera_number, ack_photo_interval, false);
}

/**
 * @brief Send a pod_1030_get_photo_mode message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack  
 * @param ack_type  Uint8, photo type
													1 :  Single shot;
													4 :  Continuous shooting;
													6 :  Interval beat;.
 * @param ack_shootting_number  Number of consecutive shots (effective when the photo type is continuous)
 * @param ack_timer_camera_number  Uint8, number of photos taken regularly (effective when the type of photos taken is intermittent)
																	0 :  reservation
																	1-254 :  number of intervals;
																	255 :  keep shooting until the camera receives the stop interval shooting command;
 * @param ack_photo_interval  Time interval between shots (effective when the type of shot is interval shot)
															0 :  reservation
															1-255 :  time interval between shots (in seconds)
 */
static inline void ttalink_pod_1030_get_photo_mode_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_type, uint16_t ack_shootting_number, uint8_t ack_timer_camera_number, uint8_t ack_photo_interval)
{
    _ttalink_pod_1030_get_photo_mode_send(dst_addr, src_addr, chan, ack, ack_type, ack_shootting_number, ack_timer_camera_number, ack_photo_interval, true);
}


static inline void _ttalink_pod_1030_get_photo_mode_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1030_get_photo_mode_t* pod_1030_get_photo_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1030_get_photo_mode_send_nocrc(dst_addr, src_addr, chan, pod_1030_get_photo_mode->ack, pod_1030_get_photo_mode->ack_type, pod_1030_get_photo_mode->ack_shootting_number, pod_1030_get_photo_mode->ack_timer_camera_number, pod_1030_get_photo_mode->ack_photo_interval);
    }else{
        ttalink_pod_1030_get_photo_mode_send(dst_addr, src_addr, chan, pod_1030_get_photo_mode->ack, pod_1030_get_photo_mode->ack_type, pod_1030_get_photo_mode->ack_shootting_number, pod_1030_get_photo_mode->ack_timer_camera_number, pod_1030_get_photo_mode->ack_photo_interval);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE, (const char *)pod_1030_get_photo_mode, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1030_get_photo_mode message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1030_get_photo_mode_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1030_get_photo_mode_t* pod_1030_get_photo_mode)
{
    _ttalink_pod_1030_get_photo_mode_send_struct(dst_addr, src_addr, chan, pod_1030_get_photo_mode, false);
}

/**
 * @brief Send a pod_1030_get_photo_mode message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1030_get_photo_mode_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1030_get_photo_mode_t* pod_1030_get_photo_mode)
{
    _ttalink_pod_1030_get_photo_mode_send_struct(dst_addr, src_addr, chan, pod_1030_get_photo_mode, true);
}

#if TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1030_get_photo_mode_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_type, uint16_t ack_shootting_number, uint8_t ack_timer_camera_number, uint8_t ack_photo_interval, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_type);
    _tta_put_uint16_t(buf, 2, ack_shootting_number);
    _tta_put_uint8_t(buf, 4, ack_timer_camera_number);
    _tta_put_uint8_t(buf, 5, ack_photo_interval);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE, buf, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_CRC, nocrc);
#else
    ttalink_pod_1030_get_photo_mode_t *packet = (ttalink_pod_1030_get_photo_mode_t *)msgbuf;
    packet->ack = ack;
    packet->ack_type = ack_type;
    packet->ack_shootting_number = ack_shootting_number;
    packet->ack_timer_camera_number = ack_timer_camera_number;
    packet->ack_photo_interval = ack_photo_interval;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE, (const char *)packet, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1030_get_photo_mode_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_type, uint16_t ack_shootting_number, uint8_t ack_timer_camera_number, uint8_t ack_photo_interval)
{
    _ttalink_pod_1030_get_photo_mode_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_type, ack_shootting_number, ack_timer_camera_number, ack_photo_interval, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1030_get_photo_mode_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_type, uint16_t ack_shootting_number, uint8_t ack_timer_camera_number, uint8_t ack_photo_interval)
{
    _ttalink_pod_1030_get_photo_mode_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_type, ack_shootting_number, ack_timer_camera_number, ack_photo_interval, true);
}
#endif

#endif

// MESSAGE POD_1030_GET_PHOTO_MODE UNPACKING


/**
 * @brief Get field ack from pod_1030_get_photo_mode message
 *
 * @return  
 */
static inline uint8_t ttalink_pod_1030_get_photo_mode_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack_type from pod_1030_get_photo_mode message
 *
 * @return  Uint8, photo type
													1 :  Single shot;
													4 :  Continuous shooting;
													6 :  Interval beat;.
 */
static inline uint8_t ttalink_pod_1030_get_photo_mode_get_ack_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field ack_shootting_number from pod_1030_get_photo_mode message
 *
 * @return  Number of consecutive shots (effective when the photo type is continuous)
 */
static inline uint16_t ttalink_pod_1030_get_photo_mode_get_ack_shootting_number(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field ack_timer_camera_number from pod_1030_get_photo_mode message
 *
 * @return  Uint8, number of photos taken regularly (effective when the type of photos taken is intermittent)
																	0 :  reservation
																	1-254 :  number of intervals;
																	255 :  keep shooting until the camera receives the stop interval shooting command;
 */
static inline uint8_t ttalink_pod_1030_get_photo_mode_get_ack_timer_camera_number(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field ack_photo_interval from pod_1030_get_photo_mode message
 *
 * @return  Time interval between shots (effective when the type of shot is interval shot)
															0 :  reservation
															1-255 :  time interval between shots (in seconds)
 */
static inline uint8_t ttalink_pod_1030_get_photo_mode_get_ack_photo_interval(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Decode a pod_1030_get_photo_mode message into a struct
 *
 * @param msg The message to decode
 * @param pod_1030_get_photo_mode C-struct to decode the message contents into
 */
static inline void ttalink_pod_1030_get_photo_mode_decode(const ttalink_message_t* msg, ttalink_pod_1030_get_photo_mode_t* pod_1030_get_photo_mode)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1030_get_photo_mode->ack = ttalink_pod_1030_get_photo_mode_get_ack(msg);
    pod_1030_get_photo_mode->ack_type = ttalink_pod_1030_get_photo_mode_get_ack_type(msg);
    pod_1030_get_photo_mode->ack_shootting_number = ttalink_pod_1030_get_photo_mode_get_ack_shootting_number(msg);
    pod_1030_get_photo_mode->ack_timer_camera_number = ttalink_pod_1030_get_photo_mode_get_ack_timer_camera_number(msg);
    pod_1030_get_photo_mode->ack_photo_interval = ttalink_pod_1030_get_photo_mode_get_ack_photo_interval(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN? msg->len : TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN;
        memset(pod_1030_get_photo_mode, 0, TTALINK_MSG_ID_POD_1030_GET_PHOTO_MODE_LEN);
    memcpy(pod_1030_get_photo_mode, _TTA_PAYLOAD(msg), len);
#endif
}
