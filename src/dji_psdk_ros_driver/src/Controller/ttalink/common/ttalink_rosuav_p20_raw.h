#pragma once
// MESSAGE ROSUAV_P20_RAW PACKING

#define TTALINK_MSG_ID_ROSUAV_P20_RAW 8197

TTAPACKED(
typedef struct __ttalink_rosuav_p20_raw_t {
 double latitude; /*<  .*/
 double longitude; /*<  .*/
 float altitude; /*<  .*/
 uint8_t satellites_num; /*<  .*/
 uint8_t location_state; /*<  .*/
 float groundSpeed; /*<  .*/
 float heading; /*<  .*/
 float reserve[4]; /*<  .*/
}) ttalink_rosuav_p20_raw_t;

#define TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN 46
#define TTALINK_MSG_ID_ROSUAV_P20_RAW_MIN_LEN 46
#define TTALINK_MSG_ID_8197_LEN 46
#define TTALINK_MSG_ID_8197_MIN_LEN 46

#define TTALINK_MSG_ID_ROSUAV_P20_RAW_CRC 36
#define TTALINK_MSG_ID_8197_CRC 36

#define TTALINK_MSG_ROSUAV_P20_RAW_FIELD_RESERVE_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_ROSUAV_P20_RAW { \
    8197, \
    "ROSUAV_P20_RAW", \
    8, \
    {  { "latitude", NULL, TTALINK_TYPE_DOUBLE, 0, 0, offsetof(ttalink_rosuav_p20_raw_t, latitude) }, \
         { "longitude", NULL, TTALINK_TYPE_DOUBLE, 0, 8, offsetof(ttalink_rosuav_p20_raw_t, longitude) }, \
         { "altitude", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_rosuav_p20_raw_t, altitude) }, \
         { "satellites_num", NULL, TTALINK_TYPE_UINT8_T, 0, 20, offsetof(ttalink_rosuav_p20_raw_t, satellites_num) }, \
         { "location_state", NULL, TTALINK_TYPE_UINT8_T, 0, 21, offsetof(ttalink_rosuav_p20_raw_t, location_state) }, \
         { "groundSpeed", NULL, TTALINK_TYPE_FLOAT, 0, 22, offsetof(ttalink_rosuav_p20_raw_t, groundSpeed) }, \
         { "heading", NULL, TTALINK_TYPE_FLOAT, 0, 26, offsetof(ttalink_rosuav_p20_raw_t, heading) }, \
         { "reserve", NULL, TTALINK_TYPE_FLOAT, 4, 30, offsetof(ttalink_rosuav_p20_raw_t, reserve) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_ROSUAV_P20_RAW { \
    "ROSUAV_P20_RAW", \
    8, \
    {  { "latitude", NULL, TTALINK_TYPE_DOUBLE, 0, 0, offsetof(ttalink_rosuav_p20_raw_t, latitude) }, \
         { "longitude", NULL, TTALINK_TYPE_DOUBLE, 0, 8, offsetof(ttalink_rosuav_p20_raw_t, longitude) }, \
         { "altitude", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_rosuav_p20_raw_t, altitude) }, \
         { "satellites_num", NULL, TTALINK_TYPE_UINT8_T, 0, 20, offsetof(ttalink_rosuav_p20_raw_t, satellites_num) }, \
         { "location_state", NULL, TTALINK_TYPE_UINT8_T, 0, 21, offsetof(ttalink_rosuav_p20_raw_t, location_state) }, \
         { "groundSpeed", NULL, TTALINK_TYPE_FLOAT, 0, 22, offsetof(ttalink_rosuav_p20_raw_t, groundSpeed) }, \
         { "heading", NULL, TTALINK_TYPE_FLOAT, 0, 26, offsetof(ttalink_rosuav_p20_raw_t, heading) }, \
         { "reserve", NULL, TTALINK_TYPE_FLOAT, 4, 30, offsetof(ttalink_rosuav_p20_raw_t, reserve) }, \
         } \
}
#endif


static inline uint16_t _ttalink_rosuav_p20_raw_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               double latitude, double longitude, float altitude, uint8_t satellites_num, uint8_t location_state, float groundSpeed, float heading, const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN];
    _tta_put_double(buf, 0, latitude);
    _tta_put_double(buf, 8, longitude);
    _tta_put_float(buf, 16, altitude);
    _tta_put_uint8_t(buf, 20, satellites_num);
    _tta_put_uint8_t(buf, 21, location_state);
    _tta_put_float(buf, 22, groundSpeed);
    _tta_put_float(buf, 26, heading);
    _tta_put_float_array(buf, 30, reserve, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN);
#else
    ttalink_rosuav_p20_raw_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.satellites_num = satellites_num;
    packet.location_state = location_state;
    packet.groundSpeed = groundSpeed;
    packet.heading = heading;
    tta_array_memcpy(packet.reserve, reserve, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ROSUAV_P20_RAW;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_ROSUAV_P20_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN, TTALINK_MSG_ID_ROSUAV_P20_RAW_CRC, nocrc);
}

/**
 * @brief Pack a rosuav_p20_raw message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param latitude  .
 * @param longitude  .
 * @param altitude  .
 * @param satellites_num  .
 * @param location_state  .
 * @param groundSpeed  .
 * @param heading  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_p20_raw_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               double latitude, double longitude, float altitude, uint8_t satellites_num, uint8_t location_state, float groundSpeed, float heading, const float *reserve)
{
    return _ttalink_rosuav_p20_raw_pack(dst_addr, src_addr, msg,  latitude, longitude, altitude, satellites_num, location_state, groundSpeed, heading, reserve, false);
}

/**
 * @brief Pack a rosuav_p20_raw message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param latitude  .
 * @param longitude  .
 * @param altitude  .
 * @param satellites_num  .
 * @param location_state  .
 * @param groundSpeed  .
 * @param heading  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_p20_raw_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               double latitude, double longitude, float altitude, uint8_t satellites_num, uint8_t location_state, float groundSpeed, float heading, const float *reserve)
{
    return _ttalink_rosuav_p20_raw_pack(dst_addr, src_addr, msg,  latitude, longitude, altitude, satellites_num, location_state, groundSpeed, heading, reserve, true);
}


static inline uint16_t _ttalink_rosuav_p20_raw_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   double latitude,double longitude,float altitude,uint8_t satellites_num,uint8_t location_state,float groundSpeed,float heading,const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN];
    _tta_put_double(buf, 0, latitude);
    _tta_put_double(buf, 8, longitude);
    _tta_put_float(buf, 16, altitude);
    _tta_put_uint8_t(buf, 20, satellites_num);
    _tta_put_uint8_t(buf, 21, location_state);
    _tta_put_float(buf, 22, groundSpeed);
    _tta_put_float(buf, 26, heading);
    _tta_put_float_array(buf, 30, reserve, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN);
#else
    ttalink_rosuav_p20_raw_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.satellites_num = satellites_num;
    packet.location_state = location_state;
    packet.groundSpeed = groundSpeed;
    packet.heading = heading;
    tta_array_memcpy(packet.reserve, reserve, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ROSUAV_P20_RAW;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_P20_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN, TTALINK_MSG_ID_ROSUAV_P20_RAW_CRC, nocrc);
}

/**
 * @brief Pack a rosuav_p20_raw message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param latitude  .
 * @param longitude  .
 * @param altitude  .
 * @param satellites_num  .
 * @param location_state  .
 * @param groundSpeed  .
 * @param heading  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_p20_raw_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   double latitude,double longitude,float altitude,uint8_t satellites_num,uint8_t location_state,float groundSpeed,float heading,const float *reserve)
{
    return _ttalink_rosuav_p20_raw_pack_chan(dst_addr, src_addr, chan, msg,  latitude, longitude, altitude, satellites_num, location_state, groundSpeed, heading, reserve, false);
}

/**
 * @brief Pack a rosuav_p20_raw message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param latitude  .
 * @param longitude  .
 * @param altitude  .
 * @param satellites_num  .
 * @param location_state  .
 * @param groundSpeed  .
 * @param heading  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_p20_raw_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   double latitude,double longitude,float altitude,uint8_t satellites_num,uint8_t location_state,float groundSpeed,float heading,const float *reserve)
{
    return _ttalink_rosuav_p20_raw_pack_chan(dst_addr, src_addr, chan, msg,  latitude, longitude, altitude, satellites_num, location_state, groundSpeed, heading, reserve, true);
}


static inline uint16_t _ttalink_rosuav_p20_raw_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_p20_raw_t* rosuav_p20_raw, bool nocrc)
{
    if(nocrc){
        return ttalink_rosuav_p20_raw_pack_nocrc(dst_addr, src_addr, msg, rosuav_p20_raw->latitude, rosuav_p20_raw->longitude, rosuav_p20_raw->altitude, rosuav_p20_raw->satellites_num, rosuav_p20_raw->location_state, rosuav_p20_raw->groundSpeed, rosuav_p20_raw->heading, rosuav_p20_raw->reserve);
    }else{
        return ttalink_rosuav_p20_raw_pack(dst_addr, src_addr, msg, rosuav_p20_raw->latitude, rosuav_p20_raw->longitude, rosuav_p20_raw->altitude, rosuav_p20_raw->satellites_num, rosuav_p20_raw->location_state, rosuav_p20_raw->groundSpeed, rosuav_p20_raw->heading, rosuav_p20_raw->reserve);
    }
    
}

/**
 * @brief Encode a rosuav_p20_raw struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_p20_raw C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_p20_raw_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_p20_raw_t* rosuav_p20_raw)
{
    return _ttalink_rosuav_p20_raw_encode(dst_addr, src_addr, msg, rosuav_p20_raw, false);
}

/**
 * @brief Encode a rosuav_p20_raw struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_p20_raw C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_p20_raw_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_p20_raw_t* rosuav_p20_raw)
{
    return _ttalink_rosuav_p20_raw_encode(dst_addr, src_addr, msg, rosuav_p20_raw, true);
}


static inline uint16_t _ttalink_rosuav_p20_raw_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_p20_raw_t* rosuav_p20_raw, bool nocrc)
{
    if(nocrc){
        return ttalink_rosuav_p20_raw_pack_chan_nocrc(dst_addr, src_addr, chan, msg, rosuav_p20_raw->latitude, rosuav_p20_raw->longitude, rosuav_p20_raw->altitude, rosuav_p20_raw->satellites_num, rosuav_p20_raw->location_state, rosuav_p20_raw->groundSpeed, rosuav_p20_raw->heading, rosuav_p20_raw->reserve);
    }else{
        return ttalink_rosuav_p20_raw_pack_chan(dst_addr, src_addr, chan, msg, rosuav_p20_raw->latitude, rosuav_p20_raw->longitude, rosuav_p20_raw->altitude, rosuav_p20_raw->satellites_num, rosuav_p20_raw->location_state, rosuav_p20_raw->groundSpeed, rosuav_p20_raw->heading, rosuav_p20_raw->reserve);
    }
}

/**
 * @brief Encode a rosuav_p20_raw struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_p20_raw C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_p20_raw_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_p20_raw_t* rosuav_p20_raw)
{
    return _ttalink_rosuav_p20_raw_encode_chan(dst_addr, src_addr, chan, msg, rosuav_p20_raw, false);
}

/**
 * @brief Encode a rosuav_p20_raw struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_p20_raw C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_p20_raw_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_p20_raw_t* rosuav_p20_raw)
{
    return _ttalink_rosuav_p20_raw_encode_chan(dst_addr, src_addr, chan, msg, rosuav_p20_raw, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_rosuav_p20_raw_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, double latitude, double longitude, float altitude, uint8_t satellites_num, uint8_t location_state, float groundSpeed, float heading, const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN];
    _tta_put_double(buf, 0, latitude);
    _tta_put_double(buf, 8, longitude);
    _tta_put_float(buf, 16, altitude);
    _tta_put_uint8_t(buf, 20, satellites_num);
    _tta_put_uint8_t(buf, 21, location_state);
    _tta_put_float(buf, 22, groundSpeed);
    _tta_put_float(buf, 26, heading);
    _tta_put_float_array(buf, 30, reserve, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_P20_RAW, buf, TTALINK_MSG_ID_ROSUAV_P20_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN, TTALINK_MSG_ID_ROSUAV_P20_RAW_CRC, nocrc);
#else
    ttalink_rosuav_p20_raw_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.satellites_num = satellites_num;
    packet.location_state = location_state;
    packet.groundSpeed = groundSpeed;
    packet.heading = heading;
    tta_array_memcpy(packet.reserve, reserve, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_P20_RAW, (const char *)&packet, TTALINK_MSG_ID_ROSUAV_P20_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN, TTALINK_MSG_ID_ROSUAV_P20_RAW_CRC, nocrc);
#endif
}

/**
 * @brief Send a rosuav_p20_raw message
 * @param chan TTAlink channel to send the message
 *
 * @param latitude  .
 * @param longitude  .
 * @param altitude  .
 * @param satellites_num  .
 * @param location_state  .
 * @param groundSpeed  .
 * @param heading  .
 * @param reserve  .
 */
static inline void ttalink_rosuav_p20_raw_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, double latitude, double longitude, float altitude, uint8_t satellites_num, uint8_t location_state, float groundSpeed, float heading, const float *reserve)
{
    _ttalink_rosuav_p20_raw_send(dst_addr, src_addr, chan, latitude, longitude, altitude, satellites_num, location_state, groundSpeed, heading, reserve, false);
}

/**
 * @brief Send a rosuav_p20_raw message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param latitude  .
 * @param longitude  .
 * @param altitude  .
 * @param satellites_num  .
 * @param location_state  .
 * @param groundSpeed  .
 * @param heading  .
 * @param reserve  .
 */
static inline void ttalink_rosuav_p20_raw_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, double latitude, double longitude, float altitude, uint8_t satellites_num, uint8_t location_state, float groundSpeed, float heading, const float *reserve)
{
    _ttalink_rosuav_p20_raw_send(dst_addr, src_addr, chan, latitude, longitude, altitude, satellites_num, location_state, groundSpeed, heading, reserve, true);
}


static inline void _ttalink_rosuav_p20_raw_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_p20_raw_t* rosuav_p20_raw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_rosuav_p20_raw_send_nocrc(dst_addr, src_addr, chan, rosuav_p20_raw->latitude, rosuav_p20_raw->longitude, rosuav_p20_raw->altitude, rosuav_p20_raw->satellites_num, rosuav_p20_raw->location_state, rosuav_p20_raw->groundSpeed, rosuav_p20_raw->heading, rosuav_p20_raw->reserve);
    }else{
        ttalink_rosuav_p20_raw_send(dst_addr, src_addr, chan, rosuav_p20_raw->latitude, rosuav_p20_raw->longitude, rosuav_p20_raw->altitude, rosuav_p20_raw->satellites_num, rosuav_p20_raw->location_state, rosuav_p20_raw->groundSpeed, rosuav_p20_raw->heading, rosuav_p20_raw->reserve);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_P20_RAW, (const char *)rosuav_p20_raw, TTALINK_MSG_ID_ROSUAV_P20_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN, TTALINK_MSG_ID_ROSUAV_P20_RAW_CRC, nocrc);
#endif
}

/**
 * @brief Send a rosuav_p20_raw message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rosuav_p20_raw_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_p20_raw_t* rosuav_p20_raw)
{
    _ttalink_rosuav_p20_raw_send_struct(dst_addr, src_addr, chan, rosuav_p20_raw, false);
}

/**
 * @brief Send a rosuav_p20_raw message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rosuav_p20_raw_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_p20_raw_t* rosuav_p20_raw)
{
    _ttalink_rosuav_p20_raw_send_struct(dst_addr, src_addr, chan, rosuav_p20_raw, true);
}

#if TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_rosuav_p20_raw_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  double latitude, double longitude, float altitude, uint8_t satellites_num, uint8_t location_state, float groundSpeed, float heading, const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_double(buf, 0, latitude);
    _tta_put_double(buf, 8, longitude);
    _tta_put_float(buf, 16, altitude);
    _tta_put_uint8_t(buf, 20, satellites_num);
    _tta_put_uint8_t(buf, 21, location_state);
    _tta_put_float(buf, 22, groundSpeed);
    _tta_put_float(buf, 26, heading);
    _tta_put_float_array(buf, 30, reserve, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_P20_RAW, buf, TTALINK_MSG_ID_ROSUAV_P20_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN, TTALINK_MSG_ID_ROSUAV_P20_RAW_CRC, nocrc);
#else
    ttalink_rosuav_p20_raw_t *packet = (ttalink_rosuav_p20_raw_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->altitude = altitude;
    packet->satellites_num = satellites_num;
    packet->location_state = location_state;
    packet->groundSpeed = groundSpeed;
    packet->heading = heading;
    tta_array_memcpy(packet->reserve, reserve, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_P20_RAW, (const char *)packet, TTALINK_MSG_ID_ROSUAV_P20_RAW_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN, TTALINK_MSG_ID_ROSUAV_P20_RAW_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_rosuav_p20_raw_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  double latitude, double longitude, float altitude, uint8_t satellites_num, uint8_t location_state, float groundSpeed, float heading, const float *reserve)
{
    _ttalink_rosuav_p20_raw_send_buf(dst_addr, src_addr, msgbuf, chan, latitude, longitude, altitude, satellites_num, location_state, groundSpeed, heading, reserve, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_rosuav_p20_raw_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  double latitude, double longitude, float altitude, uint8_t satellites_num, uint8_t location_state, float groundSpeed, float heading, const float *reserve)
{
    _ttalink_rosuav_p20_raw_send_buf(dst_addr, src_addr, msgbuf, chan, latitude, longitude, altitude, satellites_num, location_state, groundSpeed, heading, reserve, true);
}
#endif

#endif

// MESSAGE ROSUAV_P20_RAW UNPACKING


/**
 * @brief Get field latitude from rosuav_p20_raw message
 *
 * @return  .
 */
static inline double ttalink_rosuav_p20_raw_get_latitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  0);
}

/**
 * @brief Get field longitude from rosuav_p20_raw message
 *
 * @return  .
 */
static inline double ttalink_rosuav_p20_raw_get_longitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  8);
}

/**
 * @brief Get field altitude from rosuav_p20_raw message
 *
 * @return  .
 */
static inline float ttalink_rosuav_p20_raw_get_altitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field satellites_num from rosuav_p20_raw message
 *
 * @return  .
 */
static inline uint8_t ttalink_rosuav_p20_raw_get_satellites_num(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field location_state from rosuav_p20_raw message
 *
 * @return  .
 */
static inline uint8_t ttalink_rosuav_p20_raw_get_location_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field groundSpeed from rosuav_p20_raw message
 *
 * @return  .
 */
static inline float ttalink_rosuav_p20_raw_get_groundSpeed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  22);
}

/**
 * @brief Get field heading from rosuav_p20_raw message
 *
 * @return  .
 */
static inline float ttalink_rosuav_p20_raw_get_heading(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  26);
}

/**
 * @brief Get field reserve from rosuav_p20_raw message
 *
 * @return  .
 */
static inline uint16_t ttalink_rosuav_p20_raw_get_reserve(const ttalink_message_t* msg, float *reserve)
{
    return _TTA_RETURN_float_array(msg, reserve, 4,  30);
}

/**
 * @brief Decode a rosuav_p20_raw message into a struct
 *
 * @param msg The message to decode
 * @param rosuav_p20_raw C-struct to decode the message contents into
 */
static inline void ttalink_rosuav_p20_raw_decode(const ttalink_message_t* msg, ttalink_rosuav_p20_raw_t* rosuav_p20_raw)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    rosuav_p20_raw->latitude = ttalink_rosuav_p20_raw_get_latitude(msg);
    rosuav_p20_raw->longitude = ttalink_rosuav_p20_raw_get_longitude(msg);
    rosuav_p20_raw->altitude = ttalink_rosuav_p20_raw_get_altitude(msg);
    rosuav_p20_raw->satellites_num = ttalink_rosuav_p20_raw_get_satellites_num(msg);
    rosuav_p20_raw->location_state = ttalink_rosuav_p20_raw_get_location_state(msg);
    rosuav_p20_raw->groundSpeed = ttalink_rosuav_p20_raw_get_groundSpeed(msg);
    rosuav_p20_raw->heading = ttalink_rosuav_p20_raw_get_heading(msg);
    ttalink_rosuav_p20_raw_get_reserve(msg, rosuav_p20_raw->reserve);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN? msg->len : TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN;
        memset(rosuav_p20_raw, 0, TTALINK_MSG_ID_ROSUAV_P20_RAW_LEN);
    memcpy(rosuav_p20_raw, _TTA_PAYLOAD(msg), len);
#endif
}
