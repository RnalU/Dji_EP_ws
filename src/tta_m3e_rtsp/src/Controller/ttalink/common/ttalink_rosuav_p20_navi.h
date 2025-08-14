#pragma once
// MESSAGE ROSUAV_P20_NAVI PACKING

#define TTALINK_MSG_ID_ROSUAV_P20_NAVI 8196

TTAPACKED(
typedef struct __ttalink_rosuav_p20_navi_t {
 double latitude; /*<  .*/
 double longitude; /*<  .*/
 float altitude; /*<  .*/
 float vel_e; /*<  .*/
 float vel_n; /*<  .*/
 float vel_u; /*<  .*/
 float baseline_distance; /*<  .*/
 float reserve[4]; /*<  .*/
}) ttalink_rosuav_p20_navi_t;

#define TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN 52
#define TTALINK_MSG_ID_ROSUAV_P20_NAVI_MIN_LEN 52
#define TTALINK_MSG_ID_8196_LEN 52
#define TTALINK_MSG_ID_8196_MIN_LEN 52

#define TTALINK_MSG_ID_ROSUAV_P20_NAVI_CRC 134
#define TTALINK_MSG_ID_8196_CRC 134

#define TTALINK_MSG_ROSUAV_P20_NAVI_FIELD_RESERVE_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_ROSUAV_P20_NAVI { \
    8196, \
    "ROSUAV_P20_NAVI", \
    8, \
    {  { "latitude", NULL, TTALINK_TYPE_DOUBLE, 0, 0, offsetof(ttalink_rosuav_p20_navi_t, latitude) }, \
         { "longitude", NULL, TTALINK_TYPE_DOUBLE, 0, 8, offsetof(ttalink_rosuav_p20_navi_t, longitude) }, \
         { "altitude", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_rosuav_p20_navi_t, altitude) }, \
         { "vel_e", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_rosuav_p20_navi_t, vel_e) }, \
         { "vel_n", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_rosuav_p20_navi_t, vel_n) }, \
         { "vel_u", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_rosuav_p20_navi_t, vel_u) }, \
         { "baseline_distance", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_rosuav_p20_navi_t, baseline_distance) }, \
         { "reserve", NULL, TTALINK_TYPE_FLOAT, 4, 36, offsetof(ttalink_rosuav_p20_navi_t, reserve) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_ROSUAV_P20_NAVI { \
    "ROSUAV_P20_NAVI", \
    8, \
    {  { "latitude", NULL, TTALINK_TYPE_DOUBLE, 0, 0, offsetof(ttalink_rosuav_p20_navi_t, latitude) }, \
         { "longitude", NULL, TTALINK_TYPE_DOUBLE, 0, 8, offsetof(ttalink_rosuav_p20_navi_t, longitude) }, \
         { "altitude", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_rosuav_p20_navi_t, altitude) }, \
         { "vel_e", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_rosuav_p20_navi_t, vel_e) }, \
         { "vel_n", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_rosuav_p20_navi_t, vel_n) }, \
         { "vel_u", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_rosuav_p20_navi_t, vel_u) }, \
         { "baseline_distance", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_rosuav_p20_navi_t, baseline_distance) }, \
         { "reserve", NULL, TTALINK_TYPE_FLOAT, 4, 36, offsetof(ttalink_rosuav_p20_navi_t, reserve) }, \
         } \
}
#endif


static inline uint16_t _ttalink_rosuav_p20_navi_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               double latitude, double longitude, float altitude, float vel_e, float vel_n, float vel_u, float baseline_distance, const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN];
    _tta_put_double(buf, 0, latitude);
    _tta_put_double(buf, 8, longitude);
    _tta_put_float(buf, 16, altitude);
    _tta_put_float(buf, 20, vel_e);
    _tta_put_float(buf, 24, vel_n);
    _tta_put_float(buf, 28, vel_u);
    _tta_put_float(buf, 32, baseline_distance);
    _tta_put_float_array(buf, 36, reserve, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN);
#else
    ttalink_rosuav_p20_navi_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.vel_e = vel_e;
    packet.vel_n = vel_n;
    packet.vel_u = vel_u;
    packet.baseline_distance = baseline_distance;
    tta_array_memcpy(packet.reserve, reserve, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ROSUAV_P20_NAVI;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_ROSUAV_P20_NAVI_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN, TTALINK_MSG_ID_ROSUAV_P20_NAVI_CRC, nocrc);
}

/**
 * @brief Pack a rosuav_p20_navi message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param latitude  .
 * @param longitude  .
 * @param altitude  .
 * @param vel_e  .
 * @param vel_n  .
 * @param vel_u  .
 * @param baseline_distance  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_p20_navi_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               double latitude, double longitude, float altitude, float vel_e, float vel_n, float vel_u, float baseline_distance, const float *reserve)
{
    return _ttalink_rosuav_p20_navi_pack(dst_addr, src_addr, msg,  latitude, longitude, altitude, vel_e, vel_n, vel_u, baseline_distance, reserve, false);
}

/**
 * @brief Pack a rosuav_p20_navi message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param latitude  .
 * @param longitude  .
 * @param altitude  .
 * @param vel_e  .
 * @param vel_n  .
 * @param vel_u  .
 * @param baseline_distance  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_p20_navi_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               double latitude, double longitude, float altitude, float vel_e, float vel_n, float vel_u, float baseline_distance, const float *reserve)
{
    return _ttalink_rosuav_p20_navi_pack(dst_addr, src_addr, msg,  latitude, longitude, altitude, vel_e, vel_n, vel_u, baseline_distance, reserve, true);
}


static inline uint16_t _ttalink_rosuav_p20_navi_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   double latitude,double longitude,float altitude,float vel_e,float vel_n,float vel_u,float baseline_distance,const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN];
    _tta_put_double(buf, 0, latitude);
    _tta_put_double(buf, 8, longitude);
    _tta_put_float(buf, 16, altitude);
    _tta_put_float(buf, 20, vel_e);
    _tta_put_float(buf, 24, vel_n);
    _tta_put_float(buf, 28, vel_u);
    _tta_put_float(buf, 32, baseline_distance);
    _tta_put_float_array(buf, 36, reserve, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN);
#else
    ttalink_rosuav_p20_navi_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.vel_e = vel_e;
    packet.vel_n = vel_n;
    packet.vel_u = vel_u;
    packet.baseline_distance = baseline_distance;
    tta_array_memcpy(packet.reserve, reserve, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ROSUAV_P20_NAVI;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_P20_NAVI_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN, TTALINK_MSG_ID_ROSUAV_P20_NAVI_CRC, nocrc);
}

/**
 * @brief Pack a rosuav_p20_navi message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param latitude  .
 * @param longitude  .
 * @param altitude  .
 * @param vel_e  .
 * @param vel_n  .
 * @param vel_u  .
 * @param baseline_distance  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_p20_navi_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   double latitude,double longitude,float altitude,float vel_e,float vel_n,float vel_u,float baseline_distance,const float *reserve)
{
    return _ttalink_rosuav_p20_navi_pack_chan(dst_addr, src_addr, chan, msg,  latitude, longitude, altitude, vel_e, vel_n, vel_u, baseline_distance, reserve, false);
}

/**
 * @brief Pack a rosuav_p20_navi message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param latitude  .
 * @param longitude  .
 * @param altitude  .
 * @param vel_e  .
 * @param vel_n  .
 * @param vel_u  .
 * @param baseline_distance  .
 * @param reserve  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_p20_navi_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   double latitude,double longitude,float altitude,float vel_e,float vel_n,float vel_u,float baseline_distance,const float *reserve)
{
    return _ttalink_rosuav_p20_navi_pack_chan(dst_addr, src_addr, chan, msg,  latitude, longitude, altitude, vel_e, vel_n, vel_u, baseline_distance, reserve, true);
}


static inline uint16_t _ttalink_rosuav_p20_navi_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_p20_navi_t* rosuav_p20_navi, bool nocrc)
{
    if(nocrc){
        return ttalink_rosuav_p20_navi_pack_nocrc(dst_addr, src_addr, msg, rosuav_p20_navi->latitude, rosuav_p20_navi->longitude, rosuav_p20_navi->altitude, rosuav_p20_navi->vel_e, rosuav_p20_navi->vel_n, rosuav_p20_navi->vel_u, rosuav_p20_navi->baseline_distance, rosuav_p20_navi->reserve);
    }else{
        return ttalink_rosuav_p20_navi_pack(dst_addr, src_addr, msg, rosuav_p20_navi->latitude, rosuav_p20_navi->longitude, rosuav_p20_navi->altitude, rosuav_p20_navi->vel_e, rosuav_p20_navi->vel_n, rosuav_p20_navi->vel_u, rosuav_p20_navi->baseline_distance, rosuav_p20_navi->reserve);
    }
    
}

/**
 * @brief Encode a rosuav_p20_navi struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_p20_navi C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_p20_navi_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_p20_navi_t* rosuav_p20_navi)
{
    return _ttalink_rosuav_p20_navi_encode(dst_addr, src_addr, msg, rosuav_p20_navi, false);
}

/**
 * @brief Encode a rosuav_p20_navi struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_p20_navi C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_p20_navi_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_p20_navi_t* rosuav_p20_navi)
{
    return _ttalink_rosuav_p20_navi_encode(dst_addr, src_addr, msg, rosuav_p20_navi, true);
}


static inline uint16_t _ttalink_rosuav_p20_navi_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_p20_navi_t* rosuav_p20_navi, bool nocrc)
{
    if(nocrc){
        return ttalink_rosuav_p20_navi_pack_chan_nocrc(dst_addr, src_addr, chan, msg, rosuav_p20_navi->latitude, rosuav_p20_navi->longitude, rosuav_p20_navi->altitude, rosuav_p20_navi->vel_e, rosuav_p20_navi->vel_n, rosuav_p20_navi->vel_u, rosuav_p20_navi->baseline_distance, rosuav_p20_navi->reserve);
    }else{
        return ttalink_rosuav_p20_navi_pack_chan(dst_addr, src_addr, chan, msg, rosuav_p20_navi->latitude, rosuav_p20_navi->longitude, rosuav_p20_navi->altitude, rosuav_p20_navi->vel_e, rosuav_p20_navi->vel_n, rosuav_p20_navi->vel_u, rosuav_p20_navi->baseline_distance, rosuav_p20_navi->reserve);
    }
}

/**
 * @brief Encode a rosuav_p20_navi struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_p20_navi C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_p20_navi_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_p20_navi_t* rosuav_p20_navi)
{
    return _ttalink_rosuav_p20_navi_encode_chan(dst_addr, src_addr, chan, msg, rosuav_p20_navi, false);
}

/**
 * @brief Encode a rosuav_p20_navi struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_p20_navi C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_p20_navi_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_p20_navi_t* rosuav_p20_navi)
{
    return _ttalink_rosuav_p20_navi_encode_chan(dst_addr, src_addr, chan, msg, rosuav_p20_navi, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_rosuav_p20_navi_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, double latitude, double longitude, float altitude, float vel_e, float vel_n, float vel_u, float baseline_distance, const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN];
    _tta_put_double(buf, 0, latitude);
    _tta_put_double(buf, 8, longitude);
    _tta_put_float(buf, 16, altitude);
    _tta_put_float(buf, 20, vel_e);
    _tta_put_float(buf, 24, vel_n);
    _tta_put_float(buf, 28, vel_u);
    _tta_put_float(buf, 32, baseline_distance);
    _tta_put_float_array(buf, 36, reserve, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_P20_NAVI, buf, TTALINK_MSG_ID_ROSUAV_P20_NAVI_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN, TTALINK_MSG_ID_ROSUAV_P20_NAVI_CRC, nocrc);
#else
    ttalink_rosuav_p20_navi_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.vel_e = vel_e;
    packet.vel_n = vel_n;
    packet.vel_u = vel_u;
    packet.baseline_distance = baseline_distance;
    tta_array_memcpy(packet.reserve, reserve, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_P20_NAVI, (const char *)&packet, TTALINK_MSG_ID_ROSUAV_P20_NAVI_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN, TTALINK_MSG_ID_ROSUAV_P20_NAVI_CRC, nocrc);
#endif
}

/**
 * @brief Send a rosuav_p20_navi message
 * @param chan TTAlink channel to send the message
 *
 * @param latitude  .
 * @param longitude  .
 * @param altitude  .
 * @param vel_e  .
 * @param vel_n  .
 * @param vel_u  .
 * @param baseline_distance  .
 * @param reserve  .
 */
static inline void ttalink_rosuav_p20_navi_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, double latitude, double longitude, float altitude, float vel_e, float vel_n, float vel_u, float baseline_distance, const float *reserve)
{
    _ttalink_rosuav_p20_navi_send(dst_addr, src_addr, chan, latitude, longitude, altitude, vel_e, vel_n, vel_u, baseline_distance, reserve, false);
}

/**
 * @brief Send a rosuav_p20_navi message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param latitude  .
 * @param longitude  .
 * @param altitude  .
 * @param vel_e  .
 * @param vel_n  .
 * @param vel_u  .
 * @param baseline_distance  .
 * @param reserve  .
 */
static inline void ttalink_rosuav_p20_navi_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, double latitude, double longitude, float altitude, float vel_e, float vel_n, float vel_u, float baseline_distance, const float *reserve)
{
    _ttalink_rosuav_p20_navi_send(dst_addr, src_addr, chan, latitude, longitude, altitude, vel_e, vel_n, vel_u, baseline_distance, reserve, true);
}


static inline void _ttalink_rosuav_p20_navi_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_p20_navi_t* rosuav_p20_navi, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_rosuav_p20_navi_send_nocrc(dst_addr, src_addr, chan, rosuav_p20_navi->latitude, rosuav_p20_navi->longitude, rosuav_p20_navi->altitude, rosuav_p20_navi->vel_e, rosuav_p20_navi->vel_n, rosuav_p20_navi->vel_u, rosuav_p20_navi->baseline_distance, rosuav_p20_navi->reserve);
    }else{
        ttalink_rosuav_p20_navi_send(dst_addr, src_addr, chan, rosuav_p20_navi->latitude, rosuav_p20_navi->longitude, rosuav_p20_navi->altitude, rosuav_p20_navi->vel_e, rosuav_p20_navi->vel_n, rosuav_p20_navi->vel_u, rosuav_p20_navi->baseline_distance, rosuav_p20_navi->reserve);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_P20_NAVI, (const char *)rosuav_p20_navi, TTALINK_MSG_ID_ROSUAV_P20_NAVI_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN, TTALINK_MSG_ID_ROSUAV_P20_NAVI_CRC, nocrc);
#endif
}

/**
 * @brief Send a rosuav_p20_navi message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rosuav_p20_navi_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_p20_navi_t* rosuav_p20_navi)
{
    _ttalink_rosuav_p20_navi_send_struct(dst_addr, src_addr, chan, rosuav_p20_navi, false);
}

/**
 * @brief Send a rosuav_p20_navi message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rosuav_p20_navi_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_p20_navi_t* rosuav_p20_navi)
{
    _ttalink_rosuav_p20_navi_send_struct(dst_addr, src_addr, chan, rosuav_p20_navi, true);
}

#if TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_rosuav_p20_navi_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  double latitude, double longitude, float altitude, float vel_e, float vel_n, float vel_u, float baseline_distance, const float *reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_double(buf, 0, latitude);
    _tta_put_double(buf, 8, longitude);
    _tta_put_float(buf, 16, altitude);
    _tta_put_float(buf, 20, vel_e);
    _tta_put_float(buf, 24, vel_n);
    _tta_put_float(buf, 28, vel_u);
    _tta_put_float(buf, 32, baseline_distance);
    _tta_put_float_array(buf, 36, reserve, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_P20_NAVI, buf, TTALINK_MSG_ID_ROSUAV_P20_NAVI_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN, TTALINK_MSG_ID_ROSUAV_P20_NAVI_CRC, nocrc);
#else
    ttalink_rosuav_p20_navi_t *packet = (ttalink_rosuav_p20_navi_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->altitude = altitude;
    packet->vel_e = vel_e;
    packet->vel_n = vel_n;
    packet->vel_u = vel_u;
    packet->baseline_distance = baseline_distance;
    tta_array_memcpy(packet->reserve, reserve, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_P20_NAVI, (const char *)packet, TTALINK_MSG_ID_ROSUAV_P20_NAVI_MIN_LEN, TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN, TTALINK_MSG_ID_ROSUAV_P20_NAVI_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_rosuav_p20_navi_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  double latitude, double longitude, float altitude, float vel_e, float vel_n, float vel_u, float baseline_distance, const float *reserve)
{
    _ttalink_rosuav_p20_navi_send_buf(dst_addr, src_addr, msgbuf, chan, latitude, longitude, altitude, vel_e, vel_n, vel_u, baseline_distance, reserve, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_rosuav_p20_navi_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  double latitude, double longitude, float altitude, float vel_e, float vel_n, float vel_u, float baseline_distance, const float *reserve)
{
    _ttalink_rosuav_p20_navi_send_buf(dst_addr, src_addr, msgbuf, chan, latitude, longitude, altitude, vel_e, vel_n, vel_u, baseline_distance, reserve, true);
}
#endif

#endif

// MESSAGE ROSUAV_P20_NAVI UNPACKING


/**
 * @brief Get field latitude from rosuav_p20_navi message
 *
 * @return  .
 */
static inline double ttalink_rosuav_p20_navi_get_latitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  0);
}

/**
 * @brief Get field longitude from rosuav_p20_navi message
 *
 * @return  .
 */
static inline double ttalink_rosuav_p20_navi_get_longitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  8);
}

/**
 * @brief Get field altitude from rosuav_p20_navi message
 *
 * @return  .
 */
static inline float ttalink_rosuav_p20_navi_get_altitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field vel_e from rosuav_p20_navi message
 *
 * @return  .
 */
static inline float ttalink_rosuav_p20_navi_get_vel_e(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  20);
}

/**
 * @brief Get field vel_n from rosuav_p20_navi message
 *
 * @return  .
 */
static inline float ttalink_rosuav_p20_navi_get_vel_n(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Get field vel_u from rosuav_p20_navi message
 *
 * @return  .
 */
static inline float ttalink_rosuav_p20_navi_get_vel_u(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  28);
}

/**
 * @brief Get field baseline_distance from rosuav_p20_navi message
 *
 * @return  .
 */
static inline float ttalink_rosuav_p20_navi_get_baseline_distance(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Get field reserve from rosuav_p20_navi message
 *
 * @return  .
 */
static inline uint16_t ttalink_rosuav_p20_navi_get_reserve(const ttalink_message_t* msg, float *reserve)
{
    return _TTA_RETURN_float_array(msg, reserve, 4,  36);
}

/**
 * @brief Decode a rosuav_p20_navi message into a struct
 *
 * @param msg The message to decode
 * @param rosuav_p20_navi C-struct to decode the message contents into
 */
static inline void ttalink_rosuav_p20_navi_decode(const ttalink_message_t* msg, ttalink_rosuav_p20_navi_t* rosuav_p20_navi)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    rosuav_p20_navi->latitude = ttalink_rosuav_p20_navi_get_latitude(msg);
    rosuav_p20_navi->longitude = ttalink_rosuav_p20_navi_get_longitude(msg);
    rosuav_p20_navi->altitude = ttalink_rosuav_p20_navi_get_altitude(msg);
    rosuav_p20_navi->vel_e = ttalink_rosuav_p20_navi_get_vel_e(msg);
    rosuav_p20_navi->vel_n = ttalink_rosuav_p20_navi_get_vel_n(msg);
    rosuav_p20_navi->vel_u = ttalink_rosuav_p20_navi_get_vel_u(msg);
    rosuav_p20_navi->baseline_distance = ttalink_rosuav_p20_navi_get_baseline_distance(msg);
    ttalink_rosuav_p20_navi_get_reserve(msg, rosuav_p20_navi->reserve);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN? msg->len : TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN;
        memset(rosuav_p20_navi, 0, TTALINK_MSG_ID_ROSUAV_P20_NAVI_LEN);
    memcpy(rosuav_p20_navi, _TTA_PAYLOAD(msg), len);
#endif
}
