#pragma once
// MESSAGE REQUEST_LAND_LOCATION PACKING

#define TTALINK_MSG_ID_REQUEST_LAND_LOCATION 2184

TTAPACKED(
typedef struct __ttalink_request_land_location_t {
 uint8_t land_location_replace; /*<  . */
 int32_t gps_lat; /*<  . */
 int32_t gps_lon; /*<  . */
 float alt; /*<   . .*/
 float yaw; /*<   . .*/
 int32_t delay_time; /*<   . .*/
 float param1; /*<   . .*/
 float param2; /*<   . .*/
 float param3; /*<   . .*/
 float param4; /*<   . .*/
}) ttalink_request_land_location_t;

#define TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN 37
#define TTALINK_MSG_ID_REQUEST_LAND_LOCATION_MIN_LEN 37
#define TTALINK_MSG_ID_2184_LEN 37
#define TTALINK_MSG_ID_2184_MIN_LEN 37

#define TTALINK_MSG_ID_REQUEST_LAND_LOCATION_CRC 167
#define TTALINK_MSG_ID_2184_CRC 167



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_REQUEST_LAND_LOCATION { \
    2184, \
    "REQUEST_LAND_LOCATION", \
    10, \
    {  { "land_location_replace", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_request_land_location_t, land_location_replace) }, \
         { "gps_lat", NULL, TTALINK_TYPE_INT32_T, 0, 1, offsetof(ttalink_request_land_location_t, gps_lat) }, \
         { "gps_lon", NULL, TTALINK_TYPE_INT32_T, 0, 5, offsetof(ttalink_request_land_location_t, gps_lon) }, \
         { "alt", NULL, TTALINK_TYPE_FLOAT, 0, 9, offsetof(ttalink_request_land_location_t, alt) }, \
         { "yaw", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_request_land_location_t, yaw) }, \
         { "delay_time", NULL, TTALINK_TYPE_INT32_T, 0, 17, offsetof(ttalink_request_land_location_t, delay_time) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 21, offsetof(ttalink_request_land_location_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_request_land_location_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 29, offsetof(ttalink_request_land_location_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 33, offsetof(ttalink_request_land_location_t, param4) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_REQUEST_LAND_LOCATION { \
    "REQUEST_LAND_LOCATION", \
    10, \
    {  { "land_location_replace", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_request_land_location_t, land_location_replace) }, \
         { "gps_lat", NULL, TTALINK_TYPE_INT32_T, 0, 1, offsetof(ttalink_request_land_location_t, gps_lat) }, \
         { "gps_lon", NULL, TTALINK_TYPE_INT32_T, 0, 5, offsetof(ttalink_request_land_location_t, gps_lon) }, \
         { "alt", NULL, TTALINK_TYPE_FLOAT, 0, 9, offsetof(ttalink_request_land_location_t, alt) }, \
         { "yaw", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_request_land_location_t, yaw) }, \
         { "delay_time", NULL, TTALINK_TYPE_INT32_T, 0, 17, offsetof(ttalink_request_land_location_t, delay_time) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 21, offsetof(ttalink_request_land_location_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_request_land_location_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 29, offsetof(ttalink_request_land_location_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 33, offsetof(ttalink_request_land_location_t, param4) }, \
         } \
}
#endif


static inline uint16_t _ttalink_request_land_location_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t land_location_replace, int32_t gps_lat, int32_t gps_lon, float alt, float yaw, int32_t delay_time, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN];
    _tta_put_uint8_t(buf, 0, land_location_replace);
    _tta_put_int32_t(buf, 1, gps_lat);
    _tta_put_int32_t(buf, 5, gps_lon);
    _tta_put_float(buf, 9, alt);
    _tta_put_float(buf, 13, yaw);
    _tta_put_int32_t(buf, 17, delay_time);
    _tta_put_float(buf, 21, param1);
    _tta_put_float(buf, 25, param2);
    _tta_put_float(buf, 29, param3);
    _tta_put_float(buf, 33, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN);
#else
    ttalink_request_land_location_t packet;
    packet.land_location_replace = land_location_replace;
    packet.gps_lat = gps_lat;
    packet.gps_lon = gps_lon;
    packet.alt = alt;
    packet.yaw = yaw;
    packet.delay_time = delay_time;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_LAND_LOCATION;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_MIN_LEN, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_CRC, nocrc);
}

/**
 * @brief Pack a request_land_location message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param land_location_replace  . 
 * @param gps_lat  . 
 * @param gps_lon  . 
 * @param alt   . .
 * @param yaw   . .
 * @param delay_time   . .
 * @param param1   . .
 * @param param2   . .
 * @param param3   . .
 * @param param4   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_land_location_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t land_location_replace, int32_t gps_lat, int32_t gps_lon, float alt, float yaw, int32_t delay_time, float param1, float param2, float param3, float param4)
{
    return _ttalink_request_land_location_pack(dst_addr, src_addr, msg,  land_location_replace, gps_lat, gps_lon, alt, yaw, delay_time, param1, param2, param3, param4, false);
}

/**
 * @brief Pack a request_land_location message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param land_location_replace  . 
 * @param gps_lat  . 
 * @param gps_lon  . 
 * @param alt   . .
 * @param yaw   . .
 * @param delay_time   . .
 * @param param1   . .
 * @param param2   . .
 * @param param3   . .
 * @param param4   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_land_location_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t land_location_replace, int32_t gps_lat, int32_t gps_lon, float alt, float yaw, int32_t delay_time, float param1, float param2, float param3, float param4)
{
    return _ttalink_request_land_location_pack(dst_addr, src_addr, msg,  land_location_replace, gps_lat, gps_lon, alt, yaw, delay_time, param1, param2, param3, param4, true);
}


static inline uint16_t _ttalink_request_land_location_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t land_location_replace,int32_t gps_lat,int32_t gps_lon,float alt,float yaw,int32_t delay_time,float param1,float param2,float param3,float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN];
    _tta_put_uint8_t(buf, 0, land_location_replace);
    _tta_put_int32_t(buf, 1, gps_lat);
    _tta_put_int32_t(buf, 5, gps_lon);
    _tta_put_float(buf, 9, alt);
    _tta_put_float(buf, 13, yaw);
    _tta_put_int32_t(buf, 17, delay_time);
    _tta_put_float(buf, 21, param1);
    _tta_put_float(buf, 25, param2);
    _tta_put_float(buf, 29, param3);
    _tta_put_float(buf, 33, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN);
#else
    ttalink_request_land_location_t packet;
    packet.land_location_replace = land_location_replace;
    packet.gps_lat = gps_lat;
    packet.gps_lon = gps_lon;
    packet.alt = alt;
    packet.yaw = yaw;
    packet.delay_time = delay_time;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_LAND_LOCATION;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_MIN_LEN, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_CRC, nocrc);
}

/**
 * @brief Pack a request_land_location message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param land_location_replace  . 
 * @param gps_lat  . 
 * @param gps_lon  . 
 * @param alt   . .
 * @param yaw   . .
 * @param delay_time   . .
 * @param param1   . .
 * @param param2   . .
 * @param param3   . .
 * @param param4   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_land_location_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t land_location_replace,int32_t gps_lat,int32_t gps_lon,float alt,float yaw,int32_t delay_time,float param1,float param2,float param3,float param4)
{
    return _ttalink_request_land_location_pack_chan(dst_addr, src_addr, chan, msg,  land_location_replace, gps_lat, gps_lon, alt, yaw, delay_time, param1, param2, param3, param4, false);
}

/**
 * @brief Pack a request_land_location message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param land_location_replace  . 
 * @param gps_lat  . 
 * @param gps_lon  . 
 * @param alt   . .
 * @param yaw   . .
 * @param delay_time   . .
 * @param param1   . .
 * @param param2   . .
 * @param param3   . .
 * @param param4   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_land_location_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t land_location_replace,int32_t gps_lat,int32_t gps_lon,float alt,float yaw,int32_t delay_time,float param1,float param2,float param3,float param4)
{
    return _ttalink_request_land_location_pack_chan(dst_addr, src_addr, chan, msg,  land_location_replace, gps_lat, gps_lon, alt, yaw, delay_time, param1, param2, param3, param4, true);
}


static inline uint16_t _ttalink_request_land_location_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_land_location_t* request_land_location, bool nocrc)
{
    if(nocrc){
        return ttalink_request_land_location_pack_nocrc(dst_addr, src_addr, msg, request_land_location->land_location_replace, request_land_location->gps_lat, request_land_location->gps_lon, request_land_location->alt, request_land_location->yaw, request_land_location->delay_time, request_land_location->param1, request_land_location->param2, request_land_location->param3, request_land_location->param4);
    }else{
        return ttalink_request_land_location_pack(dst_addr, src_addr, msg, request_land_location->land_location_replace, request_land_location->gps_lat, request_land_location->gps_lon, request_land_location->alt, request_land_location->yaw, request_land_location->delay_time, request_land_location->param1, request_land_location->param2, request_land_location->param3, request_land_location->param4);
    }
    
}

/**
 * @brief Encode a request_land_location struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_land_location C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_land_location_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_land_location_t* request_land_location)
{
    return _ttalink_request_land_location_encode(dst_addr, src_addr, msg, request_land_location, false);
}

/**
 * @brief Encode a request_land_location struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_land_location C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_land_location_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_land_location_t* request_land_location)
{
    return _ttalink_request_land_location_encode(dst_addr, src_addr, msg, request_land_location, true);
}


static inline uint16_t _ttalink_request_land_location_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_land_location_t* request_land_location, bool nocrc)
{
    if(nocrc){
        return ttalink_request_land_location_pack_chan_nocrc(dst_addr, src_addr, chan, msg, request_land_location->land_location_replace, request_land_location->gps_lat, request_land_location->gps_lon, request_land_location->alt, request_land_location->yaw, request_land_location->delay_time, request_land_location->param1, request_land_location->param2, request_land_location->param3, request_land_location->param4);
    }else{
        return ttalink_request_land_location_pack_chan(dst_addr, src_addr, chan, msg, request_land_location->land_location_replace, request_land_location->gps_lat, request_land_location->gps_lon, request_land_location->alt, request_land_location->yaw, request_land_location->delay_time, request_land_location->param1, request_land_location->param2, request_land_location->param3, request_land_location->param4);
    }
}

/**
 * @brief Encode a request_land_location struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_land_location C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_land_location_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_land_location_t* request_land_location)
{
    return _ttalink_request_land_location_encode_chan(dst_addr, src_addr, chan, msg, request_land_location, false);
}

/**
 * @brief Encode a request_land_location struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_land_location C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_land_location_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_land_location_t* request_land_location)
{
    return _ttalink_request_land_location_encode_chan(dst_addr, src_addr, chan, msg, request_land_location, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_request_land_location_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t land_location_replace, int32_t gps_lat, int32_t gps_lon, float alt, float yaw, int32_t delay_time, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN];
    _tta_put_uint8_t(buf, 0, land_location_replace);
    _tta_put_int32_t(buf, 1, gps_lat);
    _tta_put_int32_t(buf, 5, gps_lon);
    _tta_put_float(buf, 9, alt);
    _tta_put_float(buf, 13, yaw);
    _tta_put_int32_t(buf, 17, delay_time);
    _tta_put_float(buf, 21, param1);
    _tta_put_float(buf, 25, param2);
    _tta_put_float(buf, 29, param3);
    _tta_put_float(buf, 33, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_LAND_LOCATION, buf, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_MIN_LEN, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_CRC, nocrc);
#else
    ttalink_request_land_location_t packet;
    packet.land_location_replace = land_location_replace;
    packet.gps_lat = gps_lat;
    packet.gps_lon = gps_lon;
    packet.alt = alt;
    packet.yaw = yaw;
    packet.delay_time = delay_time;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_LAND_LOCATION, (const char *)&packet, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_MIN_LEN, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_land_location message
 * @param chan TTAlink channel to send the message
 *
 * @param land_location_replace  . 
 * @param gps_lat  . 
 * @param gps_lon  . 
 * @param alt   . .
 * @param yaw   . .
 * @param delay_time   . .
 * @param param1   . .
 * @param param2   . .
 * @param param3   . .
 * @param param4   . .
 */
static inline void ttalink_request_land_location_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t land_location_replace, int32_t gps_lat, int32_t gps_lon, float alt, float yaw, int32_t delay_time, float param1, float param2, float param3, float param4)
{
    _ttalink_request_land_location_send(dst_addr, src_addr, chan, land_location_replace, gps_lat, gps_lon, alt, yaw, delay_time, param1, param2, param3, param4, false);
}

/**
 * @brief Send a request_land_location message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param land_location_replace  . 
 * @param gps_lat  . 
 * @param gps_lon  . 
 * @param alt   . .
 * @param yaw   . .
 * @param delay_time   . .
 * @param param1   . .
 * @param param2   . .
 * @param param3   . .
 * @param param4   . .
 */
static inline void ttalink_request_land_location_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t land_location_replace, int32_t gps_lat, int32_t gps_lon, float alt, float yaw, int32_t delay_time, float param1, float param2, float param3, float param4)
{
    _ttalink_request_land_location_send(dst_addr, src_addr, chan, land_location_replace, gps_lat, gps_lon, alt, yaw, delay_time, param1, param2, param3, param4, true);
}


static inline void _ttalink_request_land_location_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_land_location_t* request_land_location, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_request_land_location_send_nocrc(dst_addr, src_addr, chan, request_land_location->land_location_replace, request_land_location->gps_lat, request_land_location->gps_lon, request_land_location->alt, request_land_location->yaw, request_land_location->delay_time, request_land_location->param1, request_land_location->param2, request_land_location->param3, request_land_location->param4);
    }else{
        ttalink_request_land_location_send(dst_addr, src_addr, chan, request_land_location->land_location_replace, request_land_location->gps_lat, request_land_location->gps_lon, request_land_location->alt, request_land_location->yaw, request_land_location->delay_time, request_land_location->param1, request_land_location->param2, request_land_location->param3, request_land_location->param4);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_LAND_LOCATION, (const char *)request_land_location, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_MIN_LEN, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_land_location message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_land_location_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_land_location_t* request_land_location)
{
    _ttalink_request_land_location_send_struct(dst_addr, src_addr, chan, request_land_location, false);
}

/**
 * @brief Send a request_land_location message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_land_location_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_land_location_t* request_land_location)
{
    _ttalink_request_land_location_send_struct(dst_addr, src_addr, chan, request_land_location, true);
}

#if TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_request_land_location_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t land_location_replace, int32_t gps_lat, int32_t gps_lon, float alt, float yaw, int32_t delay_time, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, land_location_replace);
    _tta_put_int32_t(buf, 1, gps_lat);
    _tta_put_int32_t(buf, 5, gps_lon);
    _tta_put_float(buf, 9, alt);
    _tta_put_float(buf, 13, yaw);
    _tta_put_int32_t(buf, 17, delay_time);
    _tta_put_float(buf, 21, param1);
    _tta_put_float(buf, 25, param2);
    _tta_put_float(buf, 29, param3);
    _tta_put_float(buf, 33, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_LAND_LOCATION, buf, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_MIN_LEN, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_CRC, nocrc);
#else
    ttalink_request_land_location_t *packet = (ttalink_request_land_location_t *)msgbuf;
    packet->land_location_replace = land_location_replace;
    packet->gps_lat = gps_lat;
    packet->gps_lon = gps_lon;
    packet->alt = alt;
    packet->yaw = yaw;
    packet->delay_time = delay_time;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_LAND_LOCATION, (const char *)packet, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_MIN_LEN, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_request_land_location_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t land_location_replace, int32_t gps_lat, int32_t gps_lon, float alt, float yaw, int32_t delay_time, float param1, float param2, float param3, float param4)
{
    _ttalink_request_land_location_send_buf(dst_addr, src_addr, msgbuf, chan, land_location_replace, gps_lat, gps_lon, alt, yaw, delay_time, param1, param2, param3, param4, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_request_land_location_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t land_location_replace, int32_t gps_lat, int32_t gps_lon, float alt, float yaw, int32_t delay_time, float param1, float param2, float param3, float param4)
{
    _ttalink_request_land_location_send_buf(dst_addr, src_addr, msgbuf, chan, land_location_replace, gps_lat, gps_lon, alt, yaw, delay_time, param1, param2, param3, param4, true);
}
#endif

#endif

// MESSAGE REQUEST_LAND_LOCATION UNPACKING


/**
 * @brief Get field land_location_replace from request_land_location message
 *
 * @return  . 
 */
static inline uint8_t ttalink_request_land_location_get_land_location_replace(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field gps_lat from request_land_location message
 *
 * @return  . 
 */
static inline int32_t ttalink_request_land_location_get_gps_lat(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  1);
}

/**
 * @brief Get field gps_lon from request_land_location message
 *
 * @return  . 
 */
static inline int32_t ttalink_request_land_location_get_gps_lon(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  5);
}

/**
 * @brief Get field alt from request_land_location message
 *
 * @return   . .
 */
static inline float ttalink_request_land_location_get_alt(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  9);
}

/**
 * @brief Get field yaw from request_land_location message
 *
 * @return   . .
 */
static inline float ttalink_request_land_location_get_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  13);
}

/**
 * @brief Get field delay_time from request_land_location message
 *
 * @return   . .
 */
static inline int32_t ttalink_request_land_location_get_delay_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  17);
}

/**
 * @brief Get field param1 from request_land_location message
 *
 * @return   . .
 */
static inline float ttalink_request_land_location_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  21);
}

/**
 * @brief Get field param2 from request_land_location message
 *
 * @return   . .
 */
static inline float ttalink_request_land_location_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  25);
}

/**
 * @brief Get field param3 from request_land_location message
 *
 * @return   . .
 */
static inline float ttalink_request_land_location_get_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  29);
}

/**
 * @brief Get field param4 from request_land_location message
 *
 * @return   . .
 */
static inline float ttalink_request_land_location_get_param4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  33);
}

/**
 * @brief Decode a request_land_location message into a struct
 *
 * @param msg The message to decode
 * @param request_land_location C-struct to decode the message contents into
 */
static inline void ttalink_request_land_location_decode(const ttalink_message_t* msg, ttalink_request_land_location_t* request_land_location)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    request_land_location->land_location_replace = ttalink_request_land_location_get_land_location_replace(msg);
    request_land_location->gps_lat = ttalink_request_land_location_get_gps_lat(msg);
    request_land_location->gps_lon = ttalink_request_land_location_get_gps_lon(msg);
    request_land_location->alt = ttalink_request_land_location_get_alt(msg);
    request_land_location->yaw = ttalink_request_land_location_get_yaw(msg);
    request_land_location->delay_time = ttalink_request_land_location_get_delay_time(msg);
    request_land_location->param1 = ttalink_request_land_location_get_param1(msg);
    request_land_location->param2 = ttalink_request_land_location_get_param2(msg);
    request_land_location->param3 = ttalink_request_land_location_get_param3(msg);
    request_land_location->param4 = ttalink_request_land_location_get_param4(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN? msg->len : TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN;
        memset(request_land_location, 0, TTALINK_MSG_ID_REQUEST_LAND_LOCATION_LEN);
    memcpy(request_land_location, _TTA_PAYLOAD(msg), len);
#endif
}
