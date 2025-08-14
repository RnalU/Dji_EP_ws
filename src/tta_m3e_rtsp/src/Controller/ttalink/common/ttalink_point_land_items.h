#pragma once
// MESSAGE POINT_LAND_ITEMS PACKING

#define TTALINK_MSG_ID_POINT_LAND_ITEMS 2192

TTAPACKED(
typedef struct __ttalink_point_land_items_t {
 int32_t latit; /*<  .*/
 int32_t longi; /*<  .*/
 float altit; /*<  .*/
 float speed; /*<  .*/
 float acc_max; /*<  .*/
 int32_t delay_time; /*<  .*/
 uint8_t item_mode; /*<  .*/
 float point_r; /*<  .*/
 uint8_t yaw_cmd; /*<  .*/
 float params[4]; /*<  .*/
}) ttalink_point_land_items_t;

#define TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN 46
#define TTALINK_MSG_ID_POINT_LAND_ITEMS_MIN_LEN 46
#define TTALINK_MSG_ID_2192_LEN 46
#define TTALINK_MSG_ID_2192_MIN_LEN 46

#define TTALINK_MSG_ID_POINT_LAND_ITEMS_CRC 165
#define TTALINK_MSG_ID_2192_CRC 165

#define TTALINK_MSG_POINT_LAND_ITEMS_FIELD_PARAMS_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POINT_LAND_ITEMS { \
    2192, \
    "POINT_LAND_ITEMS", \
    10, \
    {  { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 0, offsetof(ttalink_point_land_items_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 4, offsetof(ttalink_point_land_items_t, longi) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_point_land_items_t, altit) }, \
         { "speed", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_point_land_items_t, speed) }, \
         { "acc_max", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_point_land_items_t, acc_max) }, \
         { "delay_time", NULL, TTALINK_TYPE_INT32_T, 0, 20, offsetof(ttalink_point_land_items_t, delay_time) }, \
         { "item_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 24, offsetof(ttalink_point_land_items_t, item_mode) }, \
         { "point_r", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_point_land_items_t, point_r) }, \
         { "yaw_cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 29, offsetof(ttalink_point_land_items_t, yaw_cmd) }, \
         { "params", NULL, TTALINK_TYPE_FLOAT, 4, 30, offsetof(ttalink_point_land_items_t, params) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POINT_LAND_ITEMS { \
    "POINT_LAND_ITEMS", \
    10, \
    {  { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 0, offsetof(ttalink_point_land_items_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 4, offsetof(ttalink_point_land_items_t, longi) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_point_land_items_t, altit) }, \
         { "speed", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_point_land_items_t, speed) }, \
         { "acc_max", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_point_land_items_t, acc_max) }, \
         { "delay_time", NULL, TTALINK_TYPE_INT32_T, 0, 20, offsetof(ttalink_point_land_items_t, delay_time) }, \
         { "item_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 24, offsetof(ttalink_point_land_items_t, item_mode) }, \
         { "point_r", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_point_land_items_t, point_r) }, \
         { "yaw_cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 29, offsetof(ttalink_point_land_items_t, yaw_cmd) }, \
         { "params", NULL, TTALINK_TYPE_FLOAT, 4, 30, offsetof(ttalink_point_land_items_t, params) }, \
         } \
}
#endif


static inline uint16_t _ttalink_point_land_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t yaw_cmd, const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN];
    _tta_put_int32_t(buf, 0, latit);
    _tta_put_int32_t(buf, 4, longi);
    _tta_put_float(buf, 8, altit);
    _tta_put_float(buf, 12, speed);
    _tta_put_float(buf, 16, acc_max);
    _tta_put_int32_t(buf, 20, delay_time);
    _tta_put_uint8_t(buf, 24, item_mode);
    _tta_put_float(buf, 25, point_r);
    _tta_put_uint8_t(buf, 29, yaw_cmd);
    _tta_put_float_array(buf, 30, params, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN);
#else
    ttalink_point_land_items_t packet;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;
    packet.speed = speed;
    packet.acc_max = acc_max;
    packet.delay_time = delay_time;
    packet.item_mode = item_mode;
    packet.point_r = point_r;
    packet.yaw_cmd = yaw_cmd;
    tta_array_memcpy(packet.params, params, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POINT_LAND_ITEMS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POINT_LAND_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN, TTALINK_MSG_ID_POINT_LAND_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a point_land_items message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param speed  .
 * @param acc_max  .
 * @param delay_time  .
 * @param item_mode  .
 * @param point_r  .
 * @param yaw_cmd  .
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_point_land_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t yaw_cmd, const float *params)
{
    return _ttalink_point_land_items_pack(dst_addr, src_addr, msg,  latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, yaw_cmd, params, false);
}

/**
 * @brief Pack a point_land_items message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param speed  .
 * @param acc_max  .
 * @param delay_time  .
 * @param item_mode  .
 * @param point_r  .
 * @param yaw_cmd  .
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_point_land_items_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t yaw_cmd, const float *params)
{
    return _ttalink_point_land_items_pack(dst_addr, src_addr, msg,  latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, yaw_cmd, params, true);
}


static inline uint16_t _ttalink_point_land_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int32_t latit,int32_t longi,float altit,float speed,float acc_max,int32_t delay_time,uint8_t item_mode,float point_r,uint8_t yaw_cmd,const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN];
    _tta_put_int32_t(buf, 0, latit);
    _tta_put_int32_t(buf, 4, longi);
    _tta_put_float(buf, 8, altit);
    _tta_put_float(buf, 12, speed);
    _tta_put_float(buf, 16, acc_max);
    _tta_put_int32_t(buf, 20, delay_time);
    _tta_put_uint8_t(buf, 24, item_mode);
    _tta_put_float(buf, 25, point_r);
    _tta_put_uint8_t(buf, 29, yaw_cmd);
    _tta_put_float_array(buf, 30, params, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN);
#else
    ttalink_point_land_items_t packet;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;
    packet.speed = speed;
    packet.acc_max = acc_max;
    packet.delay_time = delay_time;
    packet.item_mode = item_mode;
    packet.point_r = point_r;
    packet.yaw_cmd = yaw_cmd;
    tta_array_memcpy(packet.params, params, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POINT_LAND_ITEMS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POINT_LAND_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN, TTALINK_MSG_ID_POINT_LAND_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a point_land_items message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param speed  .
 * @param acc_max  .
 * @param delay_time  .
 * @param item_mode  .
 * @param point_r  .
 * @param yaw_cmd  .
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_point_land_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int32_t latit,int32_t longi,float altit,float speed,float acc_max,int32_t delay_time,uint8_t item_mode,float point_r,uint8_t yaw_cmd,const float *params)
{
    return _ttalink_point_land_items_pack_chan(dst_addr, src_addr, chan, msg,  latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, yaw_cmd, params, false);
}

/**
 * @brief Pack a point_land_items message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param speed  .
 * @param acc_max  .
 * @param delay_time  .
 * @param item_mode  .
 * @param point_r  .
 * @param yaw_cmd  .
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_point_land_items_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int32_t latit,int32_t longi,float altit,float speed,float acc_max,int32_t delay_time,uint8_t item_mode,float point_r,uint8_t yaw_cmd,const float *params)
{
    return _ttalink_point_land_items_pack_chan(dst_addr, src_addr, chan, msg,  latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, yaw_cmd, params, true);
}


static inline uint16_t _ttalink_point_land_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_point_land_items_t* point_land_items, bool nocrc)
{
    if(nocrc){
        return ttalink_point_land_items_pack_nocrc(dst_addr, src_addr, msg, point_land_items->latit, point_land_items->longi, point_land_items->altit, point_land_items->speed, point_land_items->acc_max, point_land_items->delay_time, point_land_items->item_mode, point_land_items->point_r, point_land_items->yaw_cmd, point_land_items->params);
    }else{
        return ttalink_point_land_items_pack(dst_addr, src_addr, msg, point_land_items->latit, point_land_items->longi, point_land_items->altit, point_land_items->speed, point_land_items->acc_max, point_land_items->delay_time, point_land_items->item_mode, point_land_items->point_r, point_land_items->yaw_cmd, point_land_items->params);
    }
    
}

/**
 * @brief Encode a point_land_items struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param point_land_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_point_land_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_point_land_items_t* point_land_items)
{
    return _ttalink_point_land_items_encode(dst_addr, src_addr, msg, point_land_items, false);
}

/**
 * @brief Encode a point_land_items struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param point_land_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_point_land_items_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_point_land_items_t* point_land_items)
{
    return _ttalink_point_land_items_encode(dst_addr, src_addr, msg, point_land_items, true);
}


static inline uint16_t _ttalink_point_land_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_point_land_items_t* point_land_items, bool nocrc)
{
    if(nocrc){
        return ttalink_point_land_items_pack_chan_nocrc(dst_addr, src_addr, chan, msg, point_land_items->latit, point_land_items->longi, point_land_items->altit, point_land_items->speed, point_land_items->acc_max, point_land_items->delay_time, point_land_items->item_mode, point_land_items->point_r, point_land_items->yaw_cmd, point_land_items->params);
    }else{
        return ttalink_point_land_items_pack_chan(dst_addr, src_addr, chan, msg, point_land_items->latit, point_land_items->longi, point_land_items->altit, point_land_items->speed, point_land_items->acc_max, point_land_items->delay_time, point_land_items->item_mode, point_land_items->point_r, point_land_items->yaw_cmd, point_land_items->params);
    }
}

/**
 * @brief Encode a point_land_items struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param point_land_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_point_land_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_point_land_items_t* point_land_items)
{
    return _ttalink_point_land_items_encode_chan(dst_addr, src_addr, chan, msg, point_land_items, false);
}

/**
 * @brief Encode a point_land_items struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param point_land_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_point_land_items_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_point_land_items_t* point_land_items)
{
    return _ttalink_point_land_items_encode_chan(dst_addr, src_addr, chan, msg, point_land_items, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_point_land_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t yaw_cmd, const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN];
    _tta_put_int32_t(buf, 0, latit);
    _tta_put_int32_t(buf, 4, longi);
    _tta_put_float(buf, 8, altit);
    _tta_put_float(buf, 12, speed);
    _tta_put_float(buf, 16, acc_max);
    _tta_put_int32_t(buf, 20, delay_time);
    _tta_put_uint8_t(buf, 24, item_mode);
    _tta_put_float(buf, 25, point_r);
    _tta_put_uint8_t(buf, 29, yaw_cmd);
    _tta_put_float_array(buf, 30, params, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POINT_LAND_ITEMS, buf, TTALINK_MSG_ID_POINT_LAND_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN, TTALINK_MSG_ID_POINT_LAND_ITEMS_CRC, nocrc);
#else
    ttalink_point_land_items_t packet;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;
    packet.speed = speed;
    packet.acc_max = acc_max;
    packet.delay_time = delay_time;
    packet.item_mode = item_mode;
    packet.point_r = point_r;
    packet.yaw_cmd = yaw_cmd;
    tta_array_memcpy(packet.params, params, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POINT_LAND_ITEMS, (const char *)&packet, TTALINK_MSG_ID_POINT_LAND_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN, TTALINK_MSG_ID_POINT_LAND_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a point_land_items message
 * @param chan TTAlink channel to send the message
 *
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param speed  .
 * @param acc_max  .
 * @param delay_time  .
 * @param item_mode  .
 * @param point_r  .
 * @param yaw_cmd  .
 * @param params  .
 */
static inline void ttalink_point_land_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t yaw_cmd, const float *params)
{
    _ttalink_point_land_items_send(dst_addr, src_addr, chan, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, yaw_cmd, params, false);
}

/**
 * @brief Send a point_land_items message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param speed  .
 * @param acc_max  .
 * @param delay_time  .
 * @param item_mode  .
 * @param point_r  .
 * @param yaw_cmd  .
 * @param params  .
 */
static inline void ttalink_point_land_items_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t yaw_cmd, const float *params)
{
    _ttalink_point_land_items_send(dst_addr, src_addr, chan, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, yaw_cmd, params, true);
}


static inline void _ttalink_point_land_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_point_land_items_t* point_land_items, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_point_land_items_send_nocrc(dst_addr, src_addr, chan, point_land_items->latit, point_land_items->longi, point_land_items->altit, point_land_items->speed, point_land_items->acc_max, point_land_items->delay_time, point_land_items->item_mode, point_land_items->point_r, point_land_items->yaw_cmd, point_land_items->params);
    }else{
        ttalink_point_land_items_send(dst_addr, src_addr, chan, point_land_items->latit, point_land_items->longi, point_land_items->altit, point_land_items->speed, point_land_items->acc_max, point_land_items->delay_time, point_land_items->item_mode, point_land_items->point_r, point_land_items->yaw_cmd, point_land_items->params);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POINT_LAND_ITEMS, (const char *)point_land_items, TTALINK_MSG_ID_POINT_LAND_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN, TTALINK_MSG_ID_POINT_LAND_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a point_land_items message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_point_land_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_point_land_items_t* point_land_items)
{
    _ttalink_point_land_items_send_struct(dst_addr, src_addr, chan, point_land_items, false);
}

/**
 * @brief Send a point_land_items message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_point_land_items_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_point_land_items_t* point_land_items)
{
    _ttalink_point_land_items_send_struct(dst_addr, src_addr, chan, point_land_items, true);
}

#if TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_point_land_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t yaw_cmd, const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_int32_t(buf, 0, latit);
    _tta_put_int32_t(buf, 4, longi);
    _tta_put_float(buf, 8, altit);
    _tta_put_float(buf, 12, speed);
    _tta_put_float(buf, 16, acc_max);
    _tta_put_int32_t(buf, 20, delay_time);
    _tta_put_uint8_t(buf, 24, item_mode);
    _tta_put_float(buf, 25, point_r);
    _tta_put_uint8_t(buf, 29, yaw_cmd);
    _tta_put_float_array(buf, 30, params, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POINT_LAND_ITEMS, buf, TTALINK_MSG_ID_POINT_LAND_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN, TTALINK_MSG_ID_POINT_LAND_ITEMS_CRC, nocrc);
#else
    ttalink_point_land_items_t *packet = (ttalink_point_land_items_t *)msgbuf;
    packet->latit = latit;
    packet->longi = longi;
    packet->altit = altit;
    packet->speed = speed;
    packet->acc_max = acc_max;
    packet->delay_time = delay_time;
    packet->item_mode = item_mode;
    packet->point_r = point_r;
    packet->yaw_cmd = yaw_cmd;
    tta_array_memcpy(packet->params, params, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POINT_LAND_ITEMS, (const char *)packet, TTALINK_MSG_ID_POINT_LAND_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN, TTALINK_MSG_ID_POINT_LAND_ITEMS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_point_land_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t yaw_cmd, const float *params)
{
    _ttalink_point_land_items_send_buf(dst_addr, src_addr, msgbuf, chan, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, yaw_cmd, params, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_point_land_items_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t yaw_cmd, const float *params)
{
    _ttalink_point_land_items_send_buf(dst_addr, src_addr, msgbuf, chan, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, yaw_cmd, params, true);
}
#endif

#endif

// MESSAGE POINT_LAND_ITEMS UNPACKING


/**
 * @brief Get field latit from point_land_items message
 *
 * @return  .
 */
static inline int32_t ttalink_point_land_items_get_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field longi from point_land_items message
 *
 * @return  .
 */
static inline int32_t ttalink_point_land_items_get_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field altit from point_land_items message
 *
 * @return  .
 */
static inline float ttalink_point_land_items_get_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  8);
}

/**
 * @brief Get field speed from point_land_items message
 *
 * @return  .
 */
static inline float ttalink_point_land_items_get_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  12);
}

/**
 * @brief Get field acc_max from point_land_items message
 *
 * @return  .
 */
static inline float ttalink_point_land_items_get_acc_max(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field delay_time from point_land_items message
 *
 * @return  .
 */
static inline int32_t ttalink_point_land_items_get_delay_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field item_mode from point_land_items message
 *
 * @return  .
 */
static inline uint8_t ttalink_point_land_items_get_item_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field point_r from point_land_items message
 *
 * @return  .
 */
static inline float ttalink_point_land_items_get_point_r(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  25);
}

/**
 * @brief Get field yaw_cmd from point_land_items message
 *
 * @return  .
 */
static inline uint8_t ttalink_point_land_items_get_yaw_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field params from point_land_items message
 *
 * @return  .
 */
static inline uint16_t ttalink_point_land_items_get_params(const ttalink_message_t* msg, float *params)
{
    return _TTA_RETURN_float_array(msg, params, 4,  30);
}

/**
 * @brief Decode a point_land_items message into a struct
 *
 * @param msg The message to decode
 * @param point_land_items C-struct to decode the message contents into
 */
static inline void ttalink_point_land_items_decode(const ttalink_message_t* msg, ttalink_point_land_items_t* point_land_items)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    point_land_items->latit = ttalink_point_land_items_get_latit(msg);
    point_land_items->longi = ttalink_point_land_items_get_longi(msg);
    point_land_items->altit = ttalink_point_land_items_get_altit(msg);
    point_land_items->speed = ttalink_point_land_items_get_speed(msg);
    point_land_items->acc_max = ttalink_point_land_items_get_acc_max(msg);
    point_land_items->delay_time = ttalink_point_land_items_get_delay_time(msg);
    point_land_items->item_mode = ttalink_point_land_items_get_item_mode(msg);
    point_land_items->point_r = ttalink_point_land_items_get_point_r(msg);
    point_land_items->yaw_cmd = ttalink_point_land_items_get_yaw_cmd(msg);
    ttalink_point_land_items_get_params(msg, point_land_items->params);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN? msg->len : TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN;
        memset(point_land_items, 0, TTALINK_MSG_ID_POINT_LAND_ITEMS_LEN);
    memcpy(point_land_items, _TTA_PAYLOAD(msg), len);
#endif
}
