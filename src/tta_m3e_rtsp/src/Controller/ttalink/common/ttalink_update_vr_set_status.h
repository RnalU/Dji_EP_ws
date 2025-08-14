#pragma once
// MESSAGE UPDATE_VR_SET_STATUS PACKING

#define TTALINK_MSG_ID_UPDATE_VR_SET_STATUS 2226

TTAPACKED(
typedef struct __ttalink_update_vr_set_status_t {
 uint32_t update_time; /*<  */
 int8_t status_id[20]; /*<   */
 double status_value; /*< .*/
 float param1; /*<  .*/
 float param2; /*<  .*/
 float param3; /*<  .*/
 float param4; /*<  .*/
}) ttalink_update_vr_set_status_t;

#define TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN 48
#define TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_MIN_LEN 48
#define TTALINK_MSG_ID_2226_LEN 48
#define TTALINK_MSG_ID_2226_MIN_LEN 48

#define TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_CRC 245
#define TTALINK_MSG_ID_2226_CRC 245

#define TTALINK_MSG_UPDATE_VR_SET_STATUS_FIELD_STATUS_ID_LEN 20

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_UPDATE_VR_SET_STATUS { \
    2226, \
    "UPDATE_VR_SET_STATUS", \
    7, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_update_vr_set_status_t, update_time) }, \
         { "status_id", NULL, TTALINK_TYPE_INT8_T, 20, 4, offsetof(ttalink_update_vr_set_status_t, status_id) }, \
         { "status_value", NULL, TTALINK_TYPE_DOUBLE, 0, 24, offsetof(ttalink_update_vr_set_status_t, status_value) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_update_vr_set_status_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_update_vr_set_status_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_update_vr_set_status_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_update_vr_set_status_t, param4) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_UPDATE_VR_SET_STATUS { \
    "UPDATE_VR_SET_STATUS", \
    7, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_update_vr_set_status_t, update_time) }, \
         { "status_id", NULL, TTALINK_TYPE_INT8_T, 20, 4, offsetof(ttalink_update_vr_set_status_t, status_id) }, \
         { "status_value", NULL, TTALINK_TYPE_DOUBLE, 0, 24, offsetof(ttalink_update_vr_set_status_t, status_value) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_update_vr_set_status_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_update_vr_set_status_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_update_vr_set_status_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_update_vr_set_status_t, param4) }, \
         } \
}
#endif


static inline uint16_t _ttalink_update_vr_set_status_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, const int8_t *status_id, double status_value, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_double(buf, 24, status_value);
    _tta_put_float(buf, 32, param1);
    _tta_put_float(buf, 36, param2);
    _tta_put_float(buf, 40, param3);
    _tta_put_float(buf, 44, param4);
    _tta_put_int8_t_array(buf, 4, status_id, 20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN);
#else
    ttalink_update_vr_set_status_t packet;
    packet.update_time = update_time;
    packet.status_value = status_value;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    tta_array_memcpy(packet.status_id, status_id, sizeof(int8_t)*20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_UPDATE_VR_SET_STATUS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_CRC, nocrc);
}

/**
 * @brief Pack a update_vr_set_status message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  
 * @param status_id   
 * @param status_value .
 * @param param1  .
 * @param param2  .
 * @param param3  .
 * @param param4  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_update_vr_set_status_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, const int8_t *status_id, double status_value, float param1, float param2, float param3, float param4)
{
    return _ttalink_update_vr_set_status_pack(dst_addr, src_addr, msg,  update_time, status_id, status_value, param1, param2, param3, param4, false);
}

/**
 * @brief Pack a update_vr_set_status message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  
 * @param status_id   
 * @param status_value .
 * @param param1  .
 * @param param2  .
 * @param param3  .
 * @param param4  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_update_vr_set_status_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, const int8_t *status_id, double status_value, float param1, float param2, float param3, float param4)
{
    return _ttalink_update_vr_set_status_pack(dst_addr, src_addr, msg,  update_time, status_id, status_value, param1, param2, param3, param4, true);
}


static inline uint16_t _ttalink_update_vr_set_status_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,const int8_t *status_id,double status_value,float param1,float param2,float param3,float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_double(buf, 24, status_value);
    _tta_put_float(buf, 32, param1);
    _tta_put_float(buf, 36, param2);
    _tta_put_float(buf, 40, param3);
    _tta_put_float(buf, 44, param4);
    _tta_put_int8_t_array(buf, 4, status_id, 20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN);
#else
    ttalink_update_vr_set_status_t packet;
    packet.update_time = update_time;
    packet.status_value = status_value;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    tta_array_memcpy(packet.status_id, status_id, sizeof(int8_t)*20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_UPDATE_VR_SET_STATUS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_CRC, nocrc);
}

/**
 * @brief Pack a update_vr_set_status message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  
 * @param status_id   
 * @param status_value .
 * @param param1  .
 * @param param2  .
 * @param param3  .
 * @param param4  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_update_vr_set_status_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,const int8_t *status_id,double status_value,float param1,float param2,float param3,float param4)
{
    return _ttalink_update_vr_set_status_pack_chan(dst_addr, src_addr, chan, msg,  update_time, status_id, status_value, param1, param2, param3, param4, false);
}

/**
 * @brief Pack a update_vr_set_status message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  
 * @param status_id   
 * @param status_value .
 * @param param1  .
 * @param param2  .
 * @param param3  .
 * @param param4  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_update_vr_set_status_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,const int8_t *status_id,double status_value,float param1,float param2,float param3,float param4)
{
    return _ttalink_update_vr_set_status_pack_chan(dst_addr, src_addr, chan, msg,  update_time, status_id, status_value, param1, param2, param3, param4, true);
}


static inline uint16_t _ttalink_update_vr_set_status_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_update_vr_set_status_t* update_vr_set_status, bool nocrc)
{
    if(nocrc){
        return ttalink_update_vr_set_status_pack_nocrc(dst_addr, src_addr, msg, update_vr_set_status->update_time, update_vr_set_status->status_id, update_vr_set_status->status_value, update_vr_set_status->param1, update_vr_set_status->param2, update_vr_set_status->param3, update_vr_set_status->param4);
    }else{
        return ttalink_update_vr_set_status_pack(dst_addr, src_addr, msg, update_vr_set_status->update_time, update_vr_set_status->status_id, update_vr_set_status->status_value, update_vr_set_status->param1, update_vr_set_status->param2, update_vr_set_status->param3, update_vr_set_status->param4);
    }
    
}

/**
 * @brief Encode a update_vr_set_status struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param update_vr_set_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_update_vr_set_status_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_update_vr_set_status_t* update_vr_set_status)
{
    return _ttalink_update_vr_set_status_encode(dst_addr, src_addr, msg, update_vr_set_status, false);
}

/**
 * @brief Encode a update_vr_set_status struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param update_vr_set_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_update_vr_set_status_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_update_vr_set_status_t* update_vr_set_status)
{
    return _ttalink_update_vr_set_status_encode(dst_addr, src_addr, msg, update_vr_set_status, true);
}


static inline uint16_t _ttalink_update_vr_set_status_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_update_vr_set_status_t* update_vr_set_status, bool nocrc)
{
    if(nocrc){
        return ttalink_update_vr_set_status_pack_chan_nocrc(dst_addr, src_addr, chan, msg, update_vr_set_status->update_time, update_vr_set_status->status_id, update_vr_set_status->status_value, update_vr_set_status->param1, update_vr_set_status->param2, update_vr_set_status->param3, update_vr_set_status->param4);
    }else{
        return ttalink_update_vr_set_status_pack_chan(dst_addr, src_addr, chan, msg, update_vr_set_status->update_time, update_vr_set_status->status_id, update_vr_set_status->status_value, update_vr_set_status->param1, update_vr_set_status->param2, update_vr_set_status->param3, update_vr_set_status->param4);
    }
}

/**
 * @brief Encode a update_vr_set_status struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_vr_set_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_update_vr_set_status_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_update_vr_set_status_t* update_vr_set_status)
{
    return _ttalink_update_vr_set_status_encode_chan(dst_addr, src_addr, chan, msg, update_vr_set_status, false);
}

/**
 * @brief Encode a update_vr_set_status struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_vr_set_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_update_vr_set_status_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_update_vr_set_status_t* update_vr_set_status)
{
    return _ttalink_update_vr_set_status_encode_chan(dst_addr, src_addr, chan, msg, update_vr_set_status, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_update_vr_set_status_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, const int8_t *status_id, double status_value, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_double(buf, 24, status_value);
    _tta_put_float(buf, 32, param1);
    _tta_put_float(buf, 36, param2);
    _tta_put_float(buf, 40, param3);
    _tta_put_float(buf, 44, param4);
    _tta_put_int8_t_array(buf, 4, status_id, 20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS, buf, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_CRC, nocrc);
#else
    ttalink_update_vr_set_status_t packet;
    packet.update_time = update_time;
    packet.status_value = status_value;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    tta_array_memcpy(packet.status_id, status_id, sizeof(int8_t)*20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS, (const char *)&packet, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_CRC, nocrc);
#endif
}

/**
 * @brief Send a update_vr_set_status message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  
 * @param status_id   
 * @param status_value .
 * @param param1  .
 * @param param2  .
 * @param param3  .
 * @param param4  .
 */
static inline void ttalink_update_vr_set_status_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, const int8_t *status_id, double status_value, float param1, float param2, float param3, float param4)
{
    _ttalink_update_vr_set_status_send(dst_addr, src_addr, chan, update_time, status_id, status_value, param1, param2, param3, param4, false);
}

/**
 * @brief Send a update_vr_set_status message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  
 * @param status_id   
 * @param status_value .
 * @param param1  .
 * @param param2  .
 * @param param3  .
 * @param param4  .
 */
static inline void ttalink_update_vr_set_status_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, const int8_t *status_id, double status_value, float param1, float param2, float param3, float param4)
{
    _ttalink_update_vr_set_status_send(dst_addr, src_addr, chan, update_time, status_id, status_value, param1, param2, param3, param4, true);
}


static inline void _ttalink_update_vr_set_status_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_update_vr_set_status_t* update_vr_set_status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_update_vr_set_status_send_nocrc(dst_addr, src_addr, chan, update_vr_set_status->update_time, update_vr_set_status->status_id, update_vr_set_status->status_value, update_vr_set_status->param1, update_vr_set_status->param2, update_vr_set_status->param3, update_vr_set_status->param4);
    }else{
        ttalink_update_vr_set_status_send(dst_addr, src_addr, chan, update_vr_set_status->update_time, update_vr_set_status->status_id, update_vr_set_status->status_value, update_vr_set_status->param1, update_vr_set_status->param2, update_vr_set_status->param3, update_vr_set_status->param4);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS, (const char *)update_vr_set_status, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_CRC, nocrc);
#endif
}

/**
 * @brief Send a update_vr_set_status message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_update_vr_set_status_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_update_vr_set_status_t* update_vr_set_status)
{
    _ttalink_update_vr_set_status_send_struct(dst_addr, src_addr, chan, update_vr_set_status, false);
}

/**
 * @brief Send a update_vr_set_status message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_update_vr_set_status_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_update_vr_set_status_t* update_vr_set_status)
{
    _ttalink_update_vr_set_status_send_struct(dst_addr, src_addr, chan, update_vr_set_status, true);
}

#if TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_update_vr_set_status_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, const int8_t *status_id, double status_value, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_double(buf, 24, status_value);
    _tta_put_float(buf, 32, param1);
    _tta_put_float(buf, 36, param2);
    _tta_put_float(buf, 40, param3);
    _tta_put_float(buf, 44, param4);
    _tta_put_int8_t_array(buf, 4, status_id, 20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS, buf, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_CRC, nocrc);
#else
    ttalink_update_vr_set_status_t *packet = (ttalink_update_vr_set_status_t *)msgbuf;
    packet->update_time = update_time;
    packet->status_value = status_value;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;
    tta_array_memcpy(packet->status_id, status_id, sizeof(int8_t)*20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS, (const char *)packet, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_update_vr_set_status_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, const int8_t *status_id, double status_value, float param1, float param2, float param3, float param4)
{
    _ttalink_update_vr_set_status_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, status_id, status_value, param1, param2, param3, param4, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_update_vr_set_status_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, const int8_t *status_id, double status_value, float param1, float param2, float param3, float param4)
{
    _ttalink_update_vr_set_status_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, status_id, status_value, param1, param2, param3, param4, true);
}
#endif

#endif

// MESSAGE UPDATE_VR_SET_STATUS UNPACKING


/**
 * @brief Get field update_time from update_vr_set_status message
 *
 * @return  
 */
static inline uint32_t ttalink_update_vr_set_status_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field status_id from update_vr_set_status message
 *
 * @return   
 */
static inline uint16_t ttalink_update_vr_set_status_get_status_id(const ttalink_message_t* msg, int8_t *status_id)
{
    return _TTA_RETURN_int8_t_array(msg, status_id, 20,  4);
}

/**
 * @brief Get field status_value from update_vr_set_status message
 *
 * @return .
 */
static inline double ttalink_update_vr_set_status_get_status_value(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  24);
}

/**
 * @brief Get field param1 from update_vr_set_status message
 *
 * @return  .
 */
static inline float ttalink_update_vr_set_status_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Get field param2 from update_vr_set_status message
 *
 * @return  .
 */
static inline float ttalink_update_vr_set_status_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  36);
}

/**
 * @brief Get field param3 from update_vr_set_status message
 *
 * @return  .
 */
static inline float ttalink_update_vr_set_status_get_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  40);
}

/**
 * @brief Get field param4 from update_vr_set_status message
 *
 * @return  .
 */
static inline float ttalink_update_vr_set_status_get_param4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  44);
}

/**
 * @brief Decode a update_vr_set_status message into a struct
 *
 * @param msg The message to decode
 * @param update_vr_set_status C-struct to decode the message contents into
 */
static inline void ttalink_update_vr_set_status_decode(const ttalink_message_t* msg, ttalink_update_vr_set_status_t* update_vr_set_status)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    update_vr_set_status->update_time = ttalink_update_vr_set_status_get_update_time(msg);
    ttalink_update_vr_set_status_get_status_id(msg, update_vr_set_status->status_id);
    update_vr_set_status->status_value = ttalink_update_vr_set_status_get_status_value(msg);
    update_vr_set_status->param1 = ttalink_update_vr_set_status_get_param1(msg);
    update_vr_set_status->param2 = ttalink_update_vr_set_status_get_param2(msg);
    update_vr_set_status->param3 = ttalink_update_vr_set_status_get_param3(msg);
    update_vr_set_status->param4 = ttalink_update_vr_set_status_get_param4(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN? msg->len : TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN;
        memset(update_vr_set_status, 0, TTALINK_MSG_ID_UPDATE_VR_SET_STATUS_LEN);
    memcpy(update_vr_set_status, _TTA_PAYLOAD(msg), len);
#endif
}
