#pragma once
// MESSAGE SEND_POINT_ITEMS PACKING

#define TTALINK_MSG_ID_SEND_POINT_ITEMS 2218

TTAPACKED(
typedef struct __ttalink_send_point_items_t {
 uint8_t mode; /*<  默认0 */
 uint8_t num; /*<  有效点数目 默认3个. */
 int32_t longi[5]; /*<  1e-7. */
 int32_t latit[5]; /*<  1e-7. */
 float altit[5]; /*<   . .*/
 float param1; /*<   . .*/
 float param2; /*<   . .*/
 float param3; /*<   . .*/
}) ttalink_send_point_items_t;

#define TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN 74
#define TTALINK_MSG_ID_SEND_POINT_ITEMS_MIN_LEN 74
#define TTALINK_MSG_ID_2218_LEN 74
#define TTALINK_MSG_ID_2218_MIN_LEN 74

#define TTALINK_MSG_ID_SEND_POINT_ITEMS_CRC 11
#define TTALINK_MSG_ID_2218_CRC 11

#define TTALINK_MSG_SEND_POINT_ITEMS_FIELD_LONGI_LEN 5
#define TTALINK_MSG_SEND_POINT_ITEMS_FIELD_LATIT_LEN 5
#define TTALINK_MSG_SEND_POINT_ITEMS_FIELD_ALTIT_LEN 5

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SEND_POINT_ITEMS { \
    2218, \
    "SEND_POINT_ITEMS", \
    8, \
    {  { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_send_point_items_t, mode) }, \
         { "num", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_send_point_items_t, num) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 5, 2, offsetof(ttalink_send_point_items_t, longi) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 5, 22, offsetof(ttalink_send_point_items_t, latit) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 5, 42, offsetof(ttalink_send_point_items_t, altit) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 62, offsetof(ttalink_send_point_items_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 66, offsetof(ttalink_send_point_items_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 70, offsetof(ttalink_send_point_items_t, param3) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SEND_POINT_ITEMS { \
    "SEND_POINT_ITEMS", \
    8, \
    {  { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_send_point_items_t, mode) }, \
         { "num", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_send_point_items_t, num) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 5, 2, offsetof(ttalink_send_point_items_t, longi) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 5, 22, offsetof(ttalink_send_point_items_t, latit) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 5, 42, offsetof(ttalink_send_point_items_t, altit) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 62, offsetof(ttalink_send_point_items_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 66, offsetof(ttalink_send_point_items_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 70, offsetof(ttalink_send_point_items_t, param3) }, \
         } \
}
#endif


static inline uint16_t _ttalink_send_point_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t mode, uint8_t num, const int32_t *longi, const int32_t *latit, const float *altit, float param1, float param2, float param3, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, num);
    _tta_put_float(buf, 62, param1);
    _tta_put_float(buf, 66, param2);
    _tta_put_float(buf, 70, param3);
    _tta_put_int32_t_array(buf, 2, longi, 5);
    _tta_put_int32_t_array(buf, 22, latit, 5);
    _tta_put_float_array(buf, 42, altit, 5);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN);
#else
    ttalink_send_point_items_t packet;
    packet.mode = mode;
    packet.num = num;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    tta_array_memcpy(packet.longi, longi, sizeof(int32_t)*5);
    tta_array_memcpy(packet.latit, latit, sizeof(int32_t)*5);
    tta_array_memcpy(packet.altit, altit, sizeof(float)*5);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SEND_POINT_ITEMS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SEND_POINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN, TTALINK_MSG_ID_SEND_POINT_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a send_point_items message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param mode  默认0 
 * @param num  有效点数目 默认3个. 
 * @param longi  1e-7. 
 * @param latit  1e-7. 
 * @param altit   . .
 * @param param1   . .
 * @param param2   . .
 * @param param3   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_send_point_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t mode, uint8_t num, const int32_t *longi, const int32_t *latit, const float *altit, float param1, float param2, float param3)
{
    return _ttalink_send_point_items_pack(dst_addr, src_addr, msg,  mode, num, longi, latit, altit, param1, param2, param3, false);
}

/**
 * @brief Pack a send_point_items message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param mode  默认0 
 * @param num  有效点数目 默认3个. 
 * @param longi  1e-7. 
 * @param latit  1e-7. 
 * @param altit   . .
 * @param param1   . .
 * @param param2   . .
 * @param param3   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_send_point_items_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t mode, uint8_t num, const int32_t *longi, const int32_t *latit, const float *altit, float param1, float param2, float param3)
{
    return _ttalink_send_point_items_pack(dst_addr, src_addr, msg,  mode, num, longi, latit, altit, param1, param2, param3, true);
}


static inline uint16_t _ttalink_send_point_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t mode,uint8_t num,const int32_t *longi,const int32_t *latit,const float *altit,float param1,float param2,float param3, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, num);
    _tta_put_float(buf, 62, param1);
    _tta_put_float(buf, 66, param2);
    _tta_put_float(buf, 70, param3);
    _tta_put_int32_t_array(buf, 2, longi, 5);
    _tta_put_int32_t_array(buf, 22, latit, 5);
    _tta_put_float_array(buf, 42, altit, 5);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN);
#else
    ttalink_send_point_items_t packet;
    packet.mode = mode;
    packet.num = num;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    tta_array_memcpy(packet.longi, longi, sizeof(int32_t)*5);
    tta_array_memcpy(packet.latit, latit, sizeof(int32_t)*5);
    tta_array_memcpy(packet.altit, altit, sizeof(float)*5);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SEND_POINT_ITEMS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SEND_POINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN, TTALINK_MSG_ID_SEND_POINT_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a send_point_items message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mode  默认0 
 * @param num  有效点数目 默认3个. 
 * @param longi  1e-7. 
 * @param latit  1e-7. 
 * @param altit   . .
 * @param param1   . .
 * @param param2   . .
 * @param param3   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_send_point_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t mode,uint8_t num,const int32_t *longi,const int32_t *latit,const float *altit,float param1,float param2,float param3)
{
    return _ttalink_send_point_items_pack_chan(dst_addr, src_addr, chan, msg,  mode, num, longi, latit, altit, param1, param2, param3, false);
}

/**
 * @brief Pack a send_point_items message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mode  默认0 
 * @param num  有效点数目 默认3个. 
 * @param longi  1e-7. 
 * @param latit  1e-7. 
 * @param altit   . .
 * @param param1   . .
 * @param param2   . .
 * @param param3   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_send_point_items_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t mode,uint8_t num,const int32_t *longi,const int32_t *latit,const float *altit,float param1,float param2,float param3)
{
    return _ttalink_send_point_items_pack_chan(dst_addr, src_addr, chan, msg,  mode, num, longi, latit, altit, param1, param2, param3, true);
}


static inline uint16_t _ttalink_send_point_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_send_point_items_t* send_point_items, bool nocrc)
{
    if(nocrc){
        return ttalink_send_point_items_pack_nocrc(dst_addr, src_addr, msg, send_point_items->mode, send_point_items->num, send_point_items->longi, send_point_items->latit, send_point_items->altit, send_point_items->param1, send_point_items->param2, send_point_items->param3);
    }else{
        return ttalink_send_point_items_pack(dst_addr, src_addr, msg, send_point_items->mode, send_point_items->num, send_point_items->longi, send_point_items->latit, send_point_items->altit, send_point_items->param1, send_point_items->param2, send_point_items->param3);
    }
    
}

/**
 * @brief Encode a send_point_items struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param send_point_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_send_point_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_send_point_items_t* send_point_items)
{
    return _ttalink_send_point_items_encode(dst_addr, src_addr, msg, send_point_items, false);
}

/**
 * @brief Encode a send_point_items struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param send_point_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_send_point_items_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_send_point_items_t* send_point_items)
{
    return _ttalink_send_point_items_encode(dst_addr, src_addr, msg, send_point_items, true);
}


static inline uint16_t _ttalink_send_point_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_send_point_items_t* send_point_items, bool nocrc)
{
    if(nocrc){
        return ttalink_send_point_items_pack_chan_nocrc(dst_addr, src_addr, chan, msg, send_point_items->mode, send_point_items->num, send_point_items->longi, send_point_items->latit, send_point_items->altit, send_point_items->param1, send_point_items->param2, send_point_items->param3);
    }else{
        return ttalink_send_point_items_pack_chan(dst_addr, src_addr, chan, msg, send_point_items->mode, send_point_items->num, send_point_items->longi, send_point_items->latit, send_point_items->altit, send_point_items->param1, send_point_items->param2, send_point_items->param3);
    }
}

/**
 * @brief Encode a send_point_items struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param send_point_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_send_point_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_send_point_items_t* send_point_items)
{
    return _ttalink_send_point_items_encode_chan(dst_addr, src_addr, chan, msg, send_point_items, false);
}

/**
 * @brief Encode a send_point_items struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param send_point_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_send_point_items_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_send_point_items_t* send_point_items)
{
    return _ttalink_send_point_items_encode_chan(dst_addr, src_addr, chan, msg, send_point_items, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_send_point_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t mode, uint8_t num, const int32_t *longi, const int32_t *latit, const float *altit, float param1, float param2, float param3, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, num);
    _tta_put_float(buf, 62, param1);
    _tta_put_float(buf, 66, param2);
    _tta_put_float(buf, 70, param3);
    _tta_put_int32_t_array(buf, 2, longi, 5);
    _tta_put_int32_t_array(buf, 22, latit, 5);
    _tta_put_float_array(buf, 42, altit, 5);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SEND_POINT_ITEMS, buf, TTALINK_MSG_ID_SEND_POINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN, TTALINK_MSG_ID_SEND_POINT_ITEMS_CRC, nocrc);
#else
    ttalink_send_point_items_t packet;
    packet.mode = mode;
    packet.num = num;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    tta_array_memcpy(packet.longi, longi, sizeof(int32_t)*5);
    tta_array_memcpy(packet.latit, latit, sizeof(int32_t)*5);
    tta_array_memcpy(packet.altit, altit, sizeof(float)*5);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SEND_POINT_ITEMS, (const char *)&packet, TTALINK_MSG_ID_SEND_POINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN, TTALINK_MSG_ID_SEND_POINT_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a send_point_items message
 * @param chan TTAlink channel to send the message
 *
 * @param mode  默认0 
 * @param num  有效点数目 默认3个. 
 * @param longi  1e-7. 
 * @param latit  1e-7. 
 * @param altit   . .
 * @param param1   . .
 * @param param2   . .
 * @param param3   . .
 */
static inline void ttalink_send_point_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t mode, uint8_t num, const int32_t *longi, const int32_t *latit, const float *altit, float param1, float param2, float param3)
{
    _ttalink_send_point_items_send(dst_addr, src_addr, chan, mode, num, longi, latit, altit, param1, param2, param3, false);
}

/**
 * @brief Send a send_point_items message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param mode  默认0 
 * @param num  有效点数目 默认3个. 
 * @param longi  1e-7. 
 * @param latit  1e-7. 
 * @param altit   . .
 * @param param1   . .
 * @param param2   . .
 * @param param3   . .
 */
static inline void ttalink_send_point_items_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t mode, uint8_t num, const int32_t *longi, const int32_t *latit, const float *altit, float param1, float param2, float param3)
{
    _ttalink_send_point_items_send(dst_addr, src_addr, chan, mode, num, longi, latit, altit, param1, param2, param3, true);
}


static inline void _ttalink_send_point_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_send_point_items_t* send_point_items, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_send_point_items_send_nocrc(dst_addr, src_addr, chan, send_point_items->mode, send_point_items->num, send_point_items->longi, send_point_items->latit, send_point_items->altit, send_point_items->param1, send_point_items->param2, send_point_items->param3);
    }else{
        ttalink_send_point_items_send(dst_addr, src_addr, chan, send_point_items->mode, send_point_items->num, send_point_items->longi, send_point_items->latit, send_point_items->altit, send_point_items->param1, send_point_items->param2, send_point_items->param3);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SEND_POINT_ITEMS, (const char *)send_point_items, TTALINK_MSG_ID_SEND_POINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN, TTALINK_MSG_ID_SEND_POINT_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a send_point_items message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_send_point_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_send_point_items_t* send_point_items)
{
    _ttalink_send_point_items_send_struct(dst_addr, src_addr, chan, send_point_items, false);
}

/**
 * @brief Send a send_point_items message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_send_point_items_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_send_point_items_t* send_point_items)
{
    _ttalink_send_point_items_send_struct(dst_addr, src_addr, chan, send_point_items, true);
}

#if TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_send_point_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t mode, uint8_t num, const int32_t *longi, const int32_t *latit, const float *altit, float param1, float param2, float param3, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, num);
    _tta_put_float(buf, 62, param1);
    _tta_put_float(buf, 66, param2);
    _tta_put_float(buf, 70, param3);
    _tta_put_int32_t_array(buf, 2, longi, 5);
    _tta_put_int32_t_array(buf, 22, latit, 5);
    _tta_put_float_array(buf, 42, altit, 5);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SEND_POINT_ITEMS, buf, TTALINK_MSG_ID_SEND_POINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN, TTALINK_MSG_ID_SEND_POINT_ITEMS_CRC, nocrc);
#else
    ttalink_send_point_items_t *packet = (ttalink_send_point_items_t *)msgbuf;
    packet->mode = mode;
    packet->num = num;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    tta_array_memcpy(packet->longi, longi, sizeof(int32_t)*5);
    tta_array_memcpy(packet->latit, latit, sizeof(int32_t)*5);
    tta_array_memcpy(packet->altit, altit, sizeof(float)*5);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SEND_POINT_ITEMS, (const char *)packet, TTALINK_MSG_ID_SEND_POINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN, TTALINK_MSG_ID_SEND_POINT_ITEMS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_send_point_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t mode, uint8_t num, const int32_t *longi, const int32_t *latit, const float *altit, float param1, float param2, float param3)
{
    _ttalink_send_point_items_send_buf(dst_addr, src_addr, msgbuf, chan, mode, num, longi, latit, altit, param1, param2, param3, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_send_point_items_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t mode, uint8_t num, const int32_t *longi, const int32_t *latit, const float *altit, float param1, float param2, float param3)
{
    _ttalink_send_point_items_send_buf(dst_addr, src_addr, msgbuf, chan, mode, num, longi, latit, altit, param1, param2, param3, true);
}
#endif

#endif

// MESSAGE SEND_POINT_ITEMS UNPACKING


/**
 * @brief Get field mode from send_point_items message
 *
 * @return  默认0 
 */
static inline uint8_t ttalink_send_point_items_get_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field num from send_point_items message
 *
 * @return  有效点数目 默认3个. 
 */
static inline uint8_t ttalink_send_point_items_get_num(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field longi from send_point_items message
 *
 * @return  1e-7. 
 */
static inline uint16_t ttalink_send_point_items_get_longi(const ttalink_message_t* msg, int32_t *longi)
{
    return _TTA_RETURN_int32_t_array(msg, longi, 5,  2);
}

/**
 * @brief Get field latit from send_point_items message
 *
 * @return  1e-7. 
 */
static inline uint16_t ttalink_send_point_items_get_latit(const ttalink_message_t* msg, int32_t *latit)
{
    return _TTA_RETURN_int32_t_array(msg, latit, 5,  22);
}

/**
 * @brief Get field altit from send_point_items message
 *
 * @return   . .
 */
static inline uint16_t ttalink_send_point_items_get_altit(const ttalink_message_t* msg, float *altit)
{
    return _TTA_RETURN_float_array(msg, altit, 5,  42);
}

/**
 * @brief Get field param1 from send_point_items message
 *
 * @return   . .
 */
static inline float ttalink_send_point_items_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  62);
}

/**
 * @brief Get field param2 from send_point_items message
 *
 * @return   . .
 */
static inline float ttalink_send_point_items_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  66);
}

/**
 * @brief Get field param3 from send_point_items message
 *
 * @return   . .
 */
static inline float ttalink_send_point_items_get_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  70);
}

/**
 * @brief Decode a send_point_items message into a struct
 *
 * @param msg The message to decode
 * @param send_point_items C-struct to decode the message contents into
 */
static inline void ttalink_send_point_items_decode(const ttalink_message_t* msg, ttalink_send_point_items_t* send_point_items)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    send_point_items->mode = ttalink_send_point_items_get_mode(msg);
    send_point_items->num = ttalink_send_point_items_get_num(msg);
    ttalink_send_point_items_get_longi(msg, send_point_items->longi);
    ttalink_send_point_items_get_latit(msg, send_point_items->latit);
    ttalink_send_point_items_get_altit(msg, send_point_items->altit);
    send_point_items->param1 = ttalink_send_point_items_get_param1(msg);
    send_point_items->param2 = ttalink_send_point_items_get_param2(msg);
    send_point_items->param3 = ttalink_send_point_items_get_param3(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN? msg->len : TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN;
        memset(send_point_items, 0, TTALINK_MSG_ID_SEND_POINT_ITEMS_LEN);
    memcpy(send_point_items, _TTA_PAYLOAD(msg), len);
#endif
}
