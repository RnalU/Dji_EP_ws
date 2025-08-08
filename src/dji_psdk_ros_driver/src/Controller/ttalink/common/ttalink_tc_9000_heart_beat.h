#pragma once
// MESSAGE TC_9000_HEART_BEAT PACKING

#define TTALINK_MSG_ID_TC_9000_HEART_BEAT 9000

TTAPACKED(
typedef struct __ttalink_tc_9000_heart_beat_t {
 uint32_t update_time; /*<  update_time*/
 uint32_t cpu_id; /*<  */
 uint8_t device_type; /*< This is device_type, use TTA_DEVICE_TYPE enum.*/
 uint32_t device_pod1; /*<  */
 uint32_t device_pod2; /*<  */
 uint32_t device_pod3; /*<  */
 uint8_t param_uchar[2]; /*<  */
 float param_float[2]; /*<  */
}) ttalink_tc_9000_heart_beat_t;

#define TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN 31
#define TTALINK_MSG_ID_TC_9000_HEART_BEAT_MIN_LEN 31
#define TTALINK_MSG_ID_9000_LEN 31
#define TTALINK_MSG_ID_9000_MIN_LEN 31

#define TTALINK_MSG_ID_TC_9000_HEART_BEAT_CRC 240
#define TTALINK_MSG_ID_9000_CRC 240

#define TTALINK_MSG_TC_9000_HEART_BEAT_FIELD_PARAM_UCHAR_LEN 2
#define TTALINK_MSG_TC_9000_HEART_BEAT_FIELD_PARAM_FLOAT_LEN 2

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_TC_9000_HEART_BEAT { \
    9000, \
    "TC_9000_HEART_BEAT", \
    8, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_tc_9000_heart_beat_t, update_time) }, \
         { "cpu_id", NULL, TTALINK_TYPE_UINT32_T, 0, 4, offsetof(ttalink_tc_9000_heart_beat_t, cpu_id) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_tc_9000_heart_beat_t, device_type) }, \
         { "device_pod1", NULL, TTALINK_TYPE_UINT32_T, 0, 9, offsetof(ttalink_tc_9000_heart_beat_t, device_pod1) }, \
         { "device_pod2", NULL, TTALINK_TYPE_UINT32_T, 0, 13, offsetof(ttalink_tc_9000_heart_beat_t, device_pod2) }, \
         { "device_pod3", NULL, TTALINK_TYPE_UINT32_T, 0, 17, offsetof(ttalink_tc_9000_heart_beat_t, device_pod3) }, \
         { "param_uchar", NULL, TTALINK_TYPE_UINT8_T, 2, 21, offsetof(ttalink_tc_9000_heart_beat_t, param_uchar) }, \
         { "param_float", NULL, TTALINK_TYPE_FLOAT, 2, 23, offsetof(ttalink_tc_9000_heart_beat_t, param_float) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_TC_9000_HEART_BEAT { \
    "TC_9000_HEART_BEAT", \
    8, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_tc_9000_heart_beat_t, update_time) }, \
         { "cpu_id", NULL, TTALINK_TYPE_UINT32_T, 0, 4, offsetof(ttalink_tc_9000_heart_beat_t, cpu_id) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_tc_9000_heart_beat_t, device_type) }, \
         { "device_pod1", NULL, TTALINK_TYPE_UINT32_T, 0, 9, offsetof(ttalink_tc_9000_heart_beat_t, device_pod1) }, \
         { "device_pod2", NULL, TTALINK_TYPE_UINT32_T, 0, 13, offsetof(ttalink_tc_9000_heart_beat_t, device_pod2) }, \
         { "device_pod3", NULL, TTALINK_TYPE_UINT32_T, 0, 17, offsetof(ttalink_tc_9000_heart_beat_t, device_pod3) }, \
         { "param_uchar", NULL, TTALINK_TYPE_UINT8_T, 2, 21, offsetof(ttalink_tc_9000_heart_beat_t, param_uchar) }, \
         { "param_float", NULL, TTALINK_TYPE_FLOAT, 2, 23, offsetof(ttalink_tc_9000_heart_beat_t, param_float) }, \
         } \
}
#endif


static inline uint16_t _ttalink_tc_9000_heart_beat_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t cpu_id, uint8_t device_type, uint32_t device_pod1, uint32_t device_pod2, uint32_t device_pod3, const uint8_t *param_uchar, const float *param_float, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, cpu_id);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_uint32_t(buf, 9, device_pod1);
    _tta_put_uint32_t(buf, 13, device_pod2);
    _tta_put_uint32_t(buf, 17, device_pod3);
    _tta_put_uint8_t_array(buf, 21, param_uchar, 2);
    _tta_put_float_array(buf, 23, param_float, 2);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN);
#else
    ttalink_tc_9000_heart_beat_t packet;
    packet.update_time = update_time;
    packet.cpu_id = cpu_id;
    packet.device_type = device_type;
    packet.device_pod1 = device_pod1;
    packet.device_pod2 = device_pod2;
    packet.device_pod3 = device_pod3;
    tta_array_memcpy(packet.param_uchar, param_uchar, sizeof(uint8_t)*2);
    tta_array_memcpy(packet.param_float, param_float, sizeof(float)*2);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TC_9000_HEART_BEAT;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_TC_9000_HEART_BEAT_MIN_LEN, TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN, TTALINK_MSG_ID_TC_9000_HEART_BEAT_CRC, nocrc);
}

/**
 * @brief Pack a tc_9000_heart_beat message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param cpu_id  
 * @param device_type This is device_type, use TTA_DEVICE_TYPE enum.
 * @param device_pod1  
 * @param device_pod2  
 * @param device_pod3  
 * @param param_uchar  
 * @param param_float  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9000_heart_beat_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t cpu_id, uint8_t device_type, uint32_t device_pod1, uint32_t device_pod2, uint32_t device_pod3, const uint8_t *param_uchar, const float *param_float)
{
    return _ttalink_tc_9000_heart_beat_pack(dst_addr, src_addr, msg,  update_time, cpu_id, device_type, device_pod1, device_pod2, device_pod3, param_uchar, param_float, false);
}

/**
 * @brief Pack a tc_9000_heart_beat message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param cpu_id  
 * @param device_type This is device_type, use TTA_DEVICE_TYPE enum.
 * @param device_pod1  
 * @param device_pod2  
 * @param device_pod3  
 * @param param_uchar  
 * @param param_float  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9000_heart_beat_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t cpu_id, uint8_t device_type, uint32_t device_pod1, uint32_t device_pod2, uint32_t device_pod3, const uint8_t *param_uchar, const float *param_float)
{
    return _ttalink_tc_9000_heart_beat_pack(dst_addr, src_addr, msg,  update_time, cpu_id, device_type, device_pod1, device_pod2, device_pod3, param_uchar, param_float, true);
}


static inline uint16_t _ttalink_tc_9000_heart_beat_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t cpu_id,uint8_t device_type,uint32_t device_pod1,uint32_t device_pod2,uint32_t device_pod3,const uint8_t *param_uchar,const float *param_float, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, cpu_id);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_uint32_t(buf, 9, device_pod1);
    _tta_put_uint32_t(buf, 13, device_pod2);
    _tta_put_uint32_t(buf, 17, device_pod3);
    _tta_put_uint8_t_array(buf, 21, param_uchar, 2);
    _tta_put_float_array(buf, 23, param_float, 2);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN);
#else
    ttalink_tc_9000_heart_beat_t packet;
    packet.update_time = update_time;
    packet.cpu_id = cpu_id;
    packet.device_type = device_type;
    packet.device_pod1 = device_pod1;
    packet.device_pod2 = device_pod2;
    packet.device_pod3 = device_pod3;
    tta_array_memcpy(packet.param_uchar, param_uchar, sizeof(uint8_t)*2);
    tta_array_memcpy(packet.param_float, param_float, sizeof(float)*2);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TC_9000_HEART_BEAT;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9000_HEART_BEAT_MIN_LEN, TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN, TTALINK_MSG_ID_TC_9000_HEART_BEAT_CRC, nocrc);
}

/**
 * @brief Pack a tc_9000_heart_beat message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param cpu_id  
 * @param device_type This is device_type, use TTA_DEVICE_TYPE enum.
 * @param device_pod1  
 * @param device_pod2  
 * @param device_pod3  
 * @param param_uchar  
 * @param param_float  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9000_heart_beat_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t cpu_id,uint8_t device_type,uint32_t device_pod1,uint32_t device_pod2,uint32_t device_pod3,const uint8_t *param_uchar,const float *param_float)
{
    return _ttalink_tc_9000_heart_beat_pack_chan(dst_addr, src_addr, chan, msg,  update_time, cpu_id, device_type, device_pod1, device_pod2, device_pod3, param_uchar, param_float, false);
}

/**
 * @brief Pack a tc_9000_heart_beat message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param cpu_id  
 * @param device_type This is device_type, use TTA_DEVICE_TYPE enum.
 * @param device_pod1  
 * @param device_pod2  
 * @param device_pod3  
 * @param param_uchar  
 * @param param_float  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9000_heart_beat_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t cpu_id,uint8_t device_type,uint32_t device_pod1,uint32_t device_pod2,uint32_t device_pod3,const uint8_t *param_uchar,const float *param_float)
{
    return _ttalink_tc_9000_heart_beat_pack_chan(dst_addr, src_addr, chan, msg,  update_time, cpu_id, device_type, device_pod1, device_pod2, device_pod3, param_uchar, param_float, true);
}


static inline uint16_t _ttalink_tc_9000_heart_beat_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9000_heart_beat_t* tc_9000_heart_beat, bool nocrc)
{
    if(nocrc){
        return ttalink_tc_9000_heart_beat_pack_nocrc(dst_addr, src_addr, msg, tc_9000_heart_beat->update_time, tc_9000_heart_beat->cpu_id, tc_9000_heart_beat->device_type, tc_9000_heart_beat->device_pod1, tc_9000_heart_beat->device_pod2, tc_9000_heart_beat->device_pod3, tc_9000_heart_beat->param_uchar, tc_9000_heart_beat->param_float);
    }else{
        return ttalink_tc_9000_heart_beat_pack(dst_addr, src_addr, msg, tc_9000_heart_beat->update_time, tc_9000_heart_beat->cpu_id, tc_9000_heart_beat->device_type, tc_9000_heart_beat->device_pod1, tc_9000_heart_beat->device_pod2, tc_9000_heart_beat->device_pod3, tc_9000_heart_beat->param_uchar, tc_9000_heart_beat->param_float);
    }
    
}

/**
 * @brief Encode a tc_9000_heart_beat struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param tc_9000_heart_beat C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9000_heart_beat_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9000_heart_beat_t* tc_9000_heart_beat)
{
    return _ttalink_tc_9000_heart_beat_encode(dst_addr, src_addr, msg, tc_9000_heart_beat, false);
}

/**
 * @brief Encode a tc_9000_heart_beat struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param tc_9000_heart_beat C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9000_heart_beat_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9000_heart_beat_t* tc_9000_heart_beat)
{
    return _ttalink_tc_9000_heart_beat_encode(dst_addr, src_addr, msg, tc_9000_heart_beat, true);
}


static inline uint16_t _ttalink_tc_9000_heart_beat_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9000_heart_beat_t* tc_9000_heart_beat, bool nocrc)
{
    if(nocrc){
        return ttalink_tc_9000_heart_beat_pack_chan_nocrc(dst_addr, src_addr, chan, msg, tc_9000_heart_beat->update_time, tc_9000_heart_beat->cpu_id, tc_9000_heart_beat->device_type, tc_9000_heart_beat->device_pod1, tc_9000_heart_beat->device_pod2, tc_9000_heart_beat->device_pod3, tc_9000_heart_beat->param_uchar, tc_9000_heart_beat->param_float);
    }else{
        return ttalink_tc_9000_heart_beat_pack_chan(dst_addr, src_addr, chan, msg, tc_9000_heart_beat->update_time, tc_9000_heart_beat->cpu_id, tc_9000_heart_beat->device_type, tc_9000_heart_beat->device_pod1, tc_9000_heart_beat->device_pod2, tc_9000_heart_beat->device_pod3, tc_9000_heart_beat->param_uchar, tc_9000_heart_beat->param_float);
    }
}

/**
 * @brief Encode a tc_9000_heart_beat struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param tc_9000_heart_beat C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9000_heart_beat_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9000_heart_beat_t* tc_9000_heart_beat)
{
    return _ttalink_tc_9000_heart_beat_encode_chan(dst_addr, src_addr, chan, msg, tc_9000_heart_beat, false);
}

/**
 * @brief Encode a tc_9000_heart_beat struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param tc_9000_heart_beat C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9000_heart_beat_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9000_heart_beat_t* tc_9000_heart_beat)
{
    return _ttalink_tc_9000_heart_beat_encode_chan(dst_addr, src_addr, chan, msg, tc_9000_heart_beat, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_tc_9000_heart_beat_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t cpu_id, uint8_t device_type, uint32_t device_pod1, uint32_t device_pod2, uint32_t device_pod3, const uint8_t *param_uchar, const float *param_float, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, cpu_id);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_uint32_t(buf, 9, device_pod1);
    _tta_put_uint32_t(buf, 13, device_pod2);
    _tta_put_uint32_t(buf, 17, device_pod3);
    _tta_put_uint8_t_array(buf, 21, param_uchar, 2);
    _tta_put_float_array(buf, 23, param_float, 2);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9000_HEART_BEAT, buf, TTALINK_MSG_ID_TC_9000_HEART_BEAT_MIN_LEN, TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN, TTALINK_MSG_ID_TC_9000_HEART_BEAT_CRC, nocrc);
#else
    ttalink_tc_9000_heart_beat_t packet;
    packet.update_time = update_time;
    packet.cpu_id = cpu_id;
    packet.device_type = device_type;
    packet.device_pod1 = device_pod1;
    packet.device_pod2 = device_pod2;
    packet.device_pod3 = device_pod3;
    tta_array_memcpy(packet.param_uchar, param_uchar, sizeof(uint8_t)*2);
    tta_array_memcpy(packet.param_float, param_float, sizeof(float)*2);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9000_HEART_BEAT, (const char *)&packet, TTALINK_MSG_ID_TC_9000_HEART_BEAT_MIN_LEN, TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN, TTALINK_MSG_ID_TC_9000_HEART_BEAT_CRC, nocrc);
#endif
}

/**
 * @brief Send a tc_9000_heart_beat message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param cpu_id  
 * @param device_type This is device_type, use TTA_DEVICE_TYPE enum.
 * @param device_pod1  
 * @param device_pod2  
 * @param device_pod3  
 * @param param_uchar  
 * @param param_float  
 */
static inline void ttalink_tc_9000_heart_beat_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t cpu_id, uint8_t device_type, uint32_t device_pod1, uint32_t device_pod2, uint32_t device_pod3, const uint8_t *param_uchar, const float *param_float)
{
    _ttalink_tc_9000_heart_beat_send(dst_addr, src_addr, chan, update_time, cpu_id, device_type, device_pod1, device_pod2, device_pod3, param_uchar, param_float, false);
}

/**
 * @brief Send a tc_9000_heart_beat message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param cpu_id  
 * @param device_type This is device_type, use TTA_DEVICE_TYPE enum.
 * @param device_pod1  
 * @param device_pod2  
 * @param device_pod3  
 * @param param_uchar  
 * @param param_float  
 */
static inline void ttalink_tc_9000_heart_beat_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t cpu_id, uint8_t device_type, uint32_t device_pod1, uint32_t device_pod2, uint32_t device_pod3, const uint8_t *param_uchar, const float *param_float)
{
    _ttalink_tc_9000_heart_beat_send(dst_addr, src_addr, chan, update_time, cpu_id, device_type, device_pod1, device_pod2, device_pod3, param_uchar, param_float, true);
}


static inline void _ttalink_tc_9000_heart_beat_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9000_heart_beat_t* tc_9000_heart_beat, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_tc_9000_heart_beat_send_nocrc(dst_addr, src_addr, chan, tc_9000_heart_beat->update_time, tc_9000_heart_beat->cpu_id, tc_9000_heart_beat->device_type, tc_9000_heart_beat->device_pod1, tc_9000_heart_beat->device_pod2, tc_9000_heart_beat->device_pod3, tc_9000_heart_beat->param_uchar, tc_9000_heart_beat->param_float);
    }else{
        ttalink_tc_9000_heart_beat_send(dst_addr, src_addr, chan, tc_9000_heart_beat->update_time, tc_9000_heart_beat->cpu_id, tc_9000_heart_beat->device_type, tc_9000_heart_beat->device_pod1, tc_9000_heart_beat->device_pod2, tc_9000_heart_beat->device_pod3, tc_9000_heart_beat->param_uchar, tc_9000_heart_beat->param_float);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9000_HEART_BEAT, (const char *)tc_9000_heart_beat, TTALINK_MSG_ID_TC_9000_HEART_BEAT_MIN_LEN, TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN, TTALINK_MSG_ID_TC_9000_HEART_BEAT_CRC, nocrc);
#endif
}

/**
 * @brief Send a tc_9000_heart_beat message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_tc_9000_heart_beat_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9000_heart_beat_t* tc_9000_heart_beat)
{
    _ttalink_tc_9000_heart_beat_send_struct(dst_addr, src_addr, chan, tc_9000_heart_beat, false);
}

/**
 * @brief Send a tc_9000_heart_beat message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_tc_9000_heart_beat_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9000_heart_beat_t* tc_9000_heart_beat)
{
    _ttalink_tc_9000_heart_beat_send_struct(dst_addr, src_addr, chan, tc_9000_heart_beat, true);
}

#if TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_tc_9000_heart_beat_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t cpu_id, uint8_t device_type, uint32_t device_pod1, uint32_t device_pod2, uint32_t device_pod3, const uint8_t *param_uchar, const float *param_float, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, cpu_id);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_uint32_t(buf, 9, device_pod1);
    _tta_put_uint32_t(buf, 13, device_pod2);
    _tta_put_uint32_t(buf, 17, device_pod3);
    _tta_put_uint8_t_array(buf, 21, param_uchar, 2);
    _tta_put_float_array(buf, 23, param_float, 2);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9000_HEART_BEAT, buf, TTALINK_MSG_ID_TC_9000_HEART_BEAT_MIN_LEN, TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN, TTALINK_MSG_ID_TC_9000_HEART_BEAT_CRC, nocrc);
#else
    ttalink_tc_9000_heart_beat_t *packet = (ttalink_tc_9000_heart_beat_t *)msgbuf;
    packet->update_time = update_time;
    packet->cpu_id = cpu_id;
    packet->device_type = device_type;
    packet->device_pod1 = device_pod1;
    packet->device_pod2 = device_pod2;
    packet->device_pod3 = device_pod3;
    tta_array_memcpy(packet->param_uchar, param_uchar, sizeof(uint8_t)*2);
    tta_array_memcpy(packet->param_float, param_float, sizeof(float)*2);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9000_HEART_BEAT, (const char *)packet, TTALINK_MSG_ID_TC_9000_HEART_BEAT_MIN_LEN, TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN, TTALINK_MSG_ID_TC_9000_HEART_BEAT_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_tc_9000_heart_beat_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t cpu_id, uint8_t device_type, uint32_t device_pod1, uint32_t device_pod2, uint32_t device_pod3, const uint8_t *param_uchar, const float *param_float)
{
    _ttalink_tc_9000_heart_beat_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, cpu_id, device_type, device_pod1, device_pod2, device_pod3, param_uchar, param_float, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_tc_9000_heart_beat_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t cpu_id, uint8_t device_type, uint32_t device_pod1, uint32_t device_pod2, uint32_t device_pod3, const uint8_t *param_uchar, const float *param_float)
{
    _ttalink_tc_9000_heart_beat_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, cpu_id, device_type, device_pod1, device_pod2, device_pod3, param_uchar, param_float, true);
}
#endif

#endif

// MESSAGE TC_9000_HEART_BEAT UNPACKING


/**
 * @brief Get field update_time from tc_9000_heart_beat message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_tc_9000_heart_beat_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field cpu_id from tc_9000_heart_beat message
 *
 * @return  
 */
static inline uint32_t ttalink_tc_9000_heart_beat_get_cpu_id(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field device_type from tc_9000_heart_beat message
 *
 * @return This is device_type, use TTA_DEVICE_TYPE enum.
 */
static inline uint8_t ttalink_tc_9000_heart_beat_get_device_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field device_pod1 from tc_9000_heart_beat message
 *
 * @return  
 */
static inline uint32_t ttalink_tc_9000_heart_beat_get_device_pod1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  9);
}

/**
 * @brief Get field device_pod2 from tc_9000_heart_beat message
 *
 * @return  
 */
static inline uint32_t ttalink_tc_9000_heart_beat_get_device_pod2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  13);
}

/**
 * @brief Get field device_pod3 from tc_9000_heart_beat message
 *
 * @return  
 */
static inline uint32_t ttalink_tc_9000_heart_beat_get_device_pod3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  17);
}

/**
 * @brief Get field param_uchar from tc_9000_heart_beat message
 *
 * @return  
 */
static inline uint16_t ttalink_tc_9000_heart_beat_get_param_uchar(const ttalink_message_t* msg, uint8_t *param_uchar)
{
    return _TTA_RETURN_uint8_t_array(msg, param_uchar, 2,  21);
}

/**
 * @brief Get field param_float from tc_9000_heart_beat message
 *
 * @return  
 */
static inline uint16_t ttalink_tc_9000_heart_beat_get_param_float(const ttalink_message_t* msg, float *param_float)
{
    return _TTA_RETURN_float_array(msg, param_float, 2,  23);
}

/**
 * @brief Decode a tc_9000_heart_beat message into a struct
 *
 * @param msg The message to decode
 * @param tc_9000_heart_beat C-struct to decode the message contents into
 */
static inline void ttalink_tc_9000_heart_beat_decode(const ttalink_message_t* msg, ttalink_tc_9000_heart_beat_t* tc_9000_heart_beat)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    tc_9000_heart_beat->update_time = ttalink_tc_9000_heart_beat_get_update_time(msg);
    tc_9000_heart_beat->cpu_id = ttalink_tc_9000_heart_beat_get_cpu_id(msg);
    tc_9000_heart_beat->device_type = ttalink_tc_9000_heart_beat_get_device_type(msg);
    tc_9000_heart_beat->device_pod1 = ttalink_tc_9000_heart_beat_get_device_pod1(msg);
    tc_9000_heart_beat->device_pod2 = ttalink_tc_9000_heart_beat_get_device_pod2(msg);
    tc_9000_heart_beat->device_pod3 = ttalink_tc_9000_heart_beat_get_device_pod3(msg);
    ttalink_tc_9000_heart_beat_get_param_uchar(msg, tc_9000_heart_beat->param_uchar);
    ttalink_tc_9000_heart_beat_get_param_float(msg, tc_9000_heart_beat->param_float);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN? msg->len : TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN;
        memset(tc_9000_heart_beat, 0, TTALINK_MSG_ID_TC_9000_HEART_BEAT_LEN);
    memcpy(tc_9000_heart_beat, _TTA_PAYLOAD(msg), len);
#endif
}
