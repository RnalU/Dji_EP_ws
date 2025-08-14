#pragma once
// MESSAGE POS_LOCATION PACKING

#define TTALINK_MSG_ID_POS_LOCATION 2002

TTAPACKED(
typedef struct __ttalink_pos_location_t {
 uint8_t switch_data; /*<  */
 uint8_t nstar; /*<   .*/
 float gps_dop; /*<   .*/
 uint8_t rtk_nstar; /*<   .*/
 uint8_t rtk_pos_type; /*<   .*/
 uint8_t rtk_head_type; /*<   .*/
 int32_t latit; /*<   .*/
 int32_t longi; /*<   .*/
 float alt; /*<   .*/
 float target_alt; /*<   .*/
 float cur_alt; /*<   .*/
 float altitude; /*<   .*/
 float relative_altitude; /*<   .*/
 float target_speed; /*<   .*/
 float ground_speed; /*<   .*/
 float velN; /*<   .*/
 float velE; /*<   .*/
 float velD; /*<   .*/
 float diff_distance; /*<   .*/
 float flight_distance; /*<   .*/
}) ttalink_pos_location_t;

#define TTALINK_MSG_ID_POS_LOCATION_LEN 65
#define TTALINK_MSG_ID_POS_LOCATION_MIN_LEN 65
#define TTALINK_MSG_ID_2002_LEN 65
#define TTALINK_MSG_ID_2002_MIN_LEN 65

#define TTALINK_MSG_ID_POS_LOCATION_CRC 154
#define TTALINK_MSG_ID_2002_CRC 154



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POS_LOCATION { \
    2002, \
    "POS_LOCATION", \
    20, \
    {  { "switch_data", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pos_location_t, switch_data) }, \
         { "nstar", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pos_location_t, nstar) }, \
         { "gps_dop", NULL, TTALINK_TYPE_FLOAT, 0, 2, offsetof(ttalink_pos_location_t, gps_dop) }, \
         { "rtk_nstar", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_pos_location_t, rtk_nstar) }, \
         { "rtk_pos_type", NULL, TTALINK_TYPE_UINT8_T, 0, 7, offsetof(ttalink_pos_location_t, rtk_pos_type) }, \
         { "rtk_head_type", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_pos_location_t, rtk_head_type) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 9, offsetof(ttalink_pos_location_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 13, offsetof(ttalink_pos_location_t, longi) }, \
         { "alt", NULL, TTALINK_TYPE_FLOAT, 0, 17, offsetof(ttalink_pos_location_t, alt) }, \
         { "target_alt", NULL, TTALINK_TYPE_FLOAT, 0, 21, offsetof(ttalink_pos_location_t, target_alt) }, \
         { "cur_alt", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_pos_location_t, cur_alt) }, \
         { "altitude", NULL, TTALINK_TYPE_FLOAT, 0, 29, offsetof(ttalink_pos_location_t, altitude) }, \
         { "relative_altitude", NULL, TTALINK_TYPE_FLOAT, 0, 33, offsetof(ttalink_pos_location_t, relative_altitude) }, \
         { "target_speed", NULL, TTALINK_TYPE_FLOAT, 0, 37, offsetof(ttalink_pos_location_t, target_speed) }, \
         { "ground_speed", NULL, TTALINK_TYPE_FLOAT, 0, 41, offsetof(ttalink_pos_location_t, ground_speed) }, \
         { "velN", NULL, TTALINK_TYPE_FLOAT, 0, 45, offsetof(ttalink_pos_location_t, velN) }, \
         { "velE", NULL, TTALINK_TYPE_FLOAT, 0, 49, offsetof(ttalink_pos_location_t, velE) }, \
         { "velD", NULL, TTALINK_TYPE_FLOAT, 0, 53, offsetof(ttalink_pos_location_t, velD) }, \
         { "diff_distance", NULL, TTALINK_TYPE_FLOAT, 0, 57, offsetof(ttalink_pos_location_t, diff_distance) }, \
         { "flight_distance", NULL, TTALINK_TYPE_FLOAT, 0, 61, offsetof(ttalink_pos_location_t, flight_distance) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POS_LOCATION { \
    "POS_LOCATION", \
    20, \
    {  { "switch_data", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pos_location_t, switch_data) }, \
         { "nstar", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pos_location_t, nstar) }, \
         { "gps_dop", NULL, TTALINK_TYPE_FLOAT, 0, 2, offsetof(ttalink_pos_location_t, gps_dop) }, \
         { "rtk_nstar", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_pos_location_t, rtk_nstar) }, \
         { "rtk_pos_type", NULL, TTALINK_TYPE_UINT8_T, 0, 7, offsetof(ttalink_pos_location_t, rtk_pos_type) }, \
         { "rtk_head_type", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_pos_location_t, rtk_head_type) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 9, offsetof(ttalink_pos_location_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 13, offsetof(ttalink_pos_location_t, longi) }, \
         { "alt", NULL, TTALINK_TYPE_FLOAT, 0, 17, offsetof(ttalink_pos_location_t, alt) }, \
         { "target_alt", NULL, TTALINK_TYPE_FLOAT, 0, 21, offsetof(ttalink_pos_location_t, target_alt) }, \
         { "cur_alt", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_pos_location_t, cur_alt) }, \
         { "altitude", NULL, TTALINK_TYPE_FLOAT, 0, 29, offsetof(ttalink_pos_location_t, altitude) }, \
         { "relative_altitude", NULL, TTALINK_TYPE_FLOAT, 0, 33, offsetof(ttalink_pos_location_t, relative_altitude) }, \
         { "target_speed", NULL, TTALINK_TYPE_FLOAT, 0, 37, offsetof(ttalink_pos_location_t, target_speed) }, \
         { "ground_speed", NULL, TTALINK_TYPE_FLOAT, 0, 41, offsetof(ttalink_pos_location_t, ground_speed) }, \
         { "velN", NULL, TTALINK_TYPE_FLOAT, 0, 45, offsetof(ttalink_pos_location_t, velN) }, \
         { "velE", NULL, TTALINK_TYPE_FLOAT, 0, 49, offsetof(ttalink_pos_location_t, velE) }, \
         { "velD", NULL, TTALINK_TYPE_FLOAT, 0, 53, offsetof(ttalink_pos_location_t, velD) }, \
         { "diff_distance", NULL, TTALINK_TYPE_FLOAT, 0, 57, offsetof(ttalink_pos_location_t, diff_distance) }, \
         { "flight_distance", NULL, TTALINK_TYPE_FLOAT, 0, 61, offsetof(ttalink_pos_location_t, flight_distance) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pos_location_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t switch_data, uint8_t nstar, float gps_dop, uint8_t rtk_nstar, uint8_t rtk_pos_type, uint8_t rtk_head_type, int32_t latit, int32_t longi, float alt, float target_alt, float cur_alt, float altitude, float relative_altitude, float target_speed, float ground_speed, float velN, float velE, float velD, float diff_distance, float flight_distance, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POS_LOCATION_LEN];
    _tta_put_uint8_t(buf, 0, switch_data);
    _tta_put_uint8_t(buf, 1, nstar);
    _tta_put_float(buf, 2, gps_dop);
    _tta_put_uint8_t(buf, 6, rtk_nstar);
    _tta_put_uint8_t(buf, 7, rtk_pos_type);
    _tta_put_uint8_t(buf, 8, rtk_head_type);
    _tta_put_int32_t(buf, 9, latit);
    _tta_put_int32_t(buf, 13, longi);
    _tta_put_float(buf, 17, alt);
    _tta_put_float(buf, 21, target_alt);
    _tta_put_float(buf, 25, cur_alt);
    _tta_put_float(buf, 29, altitude);
    _tta_put_float(buf, 33, relative_altitude);
    _tta_put_float(buf, 37, target_speed);
    _tta_put_float(buf, 41, ground_speed);
    _tta_put_float(buf, 45, velN);
    _tta_put_float(buf, 49, velE);
    _tta_put_float(buf, 53, velD);
    _tta_put_float(buf, 57, diff_distance);
    _tta_put_float(buf, 61, flight_distance);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POS_LOCATION_LEN);
#else
    ttalink_pos_location_t packet;
    packet.switch_data = switch_data;
    packet.nstar = nstar;
    packet.gps_dop = gps_dop;
    packet.rtk_nstar = rtk_nstar;
    packet.rtk_pos_type = rtk_pos_type;
    packet.rtk_head_type = rtk_head_type;
    packet.latit = latit;
    packet.longi = longi;
    packet.alt = alt;
    packet.target_alt = target_alt;
    packet.cur_alt = cur_alt;
    packet.altitude = altitude;
    packet.relative_altitude = relative_altitude;
    packet.target_speed = target_speed;
    packet.ground_speed = ground_speed;
    packet.velN = velN;
    packet.velE = velE;
    packet.velD = velD;
    packet.diff_distance = diff_distance;
    packet.flight_distance = flight_distance;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POS_LOCATION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POS_LOCATION;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POS_LOCATION_MIN_LEN, TTALINK_MSG_ID_POS_LOCATION_LEN, TTALINK_MSG_ID_POS_LOCATION_CRC, nocrc);
}

/**
 * @brief Pack a pos_location message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param switch_data  
 * @param nstar   .
 * @param gps_dop   .
 * @param rtk_nstar   .
 * @param rtk_pos_type   .
 * @param rtk_head_type   .
 * @param latit   .
 * @param longi   .
 * @param alt   .
 * @param target_alt   .
 * @param cur_alt   .
 * @param altitude   .
 * @param relative_altitude   .
 * @param target_speed   .
 * @param ground_speed   .
 * @param velN   .
 * @param velE   .
 * @param velD   .
 * @param diff_distance   .
 * @param flight_distance   .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pos_location_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t switch_data, uint8_t nstar, float gps_dop, uint8_t rtk_nstar, uint8_t rtk_pos_type, uint8_t rtk_head_type, int32_t latit, int32_t longi, float alt, float target_alt, float cur_alt, float altitude, float relative_altitude, float target_speed, float ground_speed, float velN, float velE, float velD, float diff_distance, float flight_distance)
{
    return _ttalink_pos_location_pack(dst_addr, src_addr, msg,  switch_data, nstar, gps_dop, rtk_nstar, rtk_pos_type, rtk_head_type, latit, longi, alt, target_alt, cur_alt, altitude, relative_altitude, target_speed, ground_speed, velN, velE, velD, diff_distance, flight_distance, false);
}

/**
 * @brief Pack a pos_location message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param switch_data  
 * @param nstar   .
 * @param gps_dop   .
 * @param rtk_nstar   .
 * @param rtk_pos_type   .
 * @param rtk_head_type   .
 * @param latit   .
 * @param longi   .
 * @param alt   .
 * @param target_alt   .
 * @param cur_alt   .
 * @param altitude   .
 * @param relative_altitude   .
 * @param target_speed   .
 * @param ground_speed   .
 * @param velN   .
 * @param velE   .
 * @param velD   .
 * @param diff_distance   .
 * @param flight_distance   .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pos_location_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t switch_data, uint8_t nstar, float gps_dop, uint8_t rtk_nstar, uint8_t rtk_pos_type, uint8_t rtk_head_type, int32_t latit, int32_t longi, float alt, float target_alt, float cur_alt, float altitude, float relative_altitude, float target_speed, float ground_speed, float velN, float velE, float velD, float diff_distance, float flight_distance)
{
    return _ttalink_pos_location_pack(dst_addr, src_addr, msg,  switch_data, nstar, gps_dop, rtk_nstar, rtk_pos_type, rtk_head_type, latit, longi, alt, target_alt, cur_alt, altitude, relative_altitude, target_speed, ground_speed, velN, velE, velD, diff_distance, flight_distance, true);
}


static inline uint16_t _ttalink_pos_location_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t switch_data,uint8_t nstar,float gps_dop,uint8_t rtk_nstar,uint8_t rtk_pos_type,uint8_t rtk_head_type,int32_t latit,int32_t longi,float alt,float target_alt,float cur_alt,float altitude,float relative_altitude,float target_speed,float ground_speed,float velN,float velE,float velD,float diff_distance,float flight_distance, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POS_LOCATION_LEN];
    _tta_put_uint8_t(buf, 0, switch_data);
    _tta_put_uint8_t(buf, 1, nstar);
    _tta_put_float(buf, 2, gps_dop);
    _tta_put_uint8_t(buf, 6, rtk_nstar);
    _tta_put_uint8_t(buf, 7, rtk_pos_type);
    _tta_put_uint8_t(buf, 8, rtk_head_type);
    _tta_put_int32_t(buf, 9, latit);
    _tta_put_int32_t(buf, 13, longi);
    _tta_put_float(buf, 17, alt);
    _tta_put_float(buf, 21, target_alt);
    _tta_put_float(buf, 25, cur_alt);
    _tta_put_float(buf, 29, altitude);
    _tta_put_float(buf, 33, relative_altitude);
    _tta_put_float(buf, 37, target_speed);
    _tta_put_float(buf, 41, ground_speed);
    _tta_put_float(buf, 45, velN);
    _tta_put_float(buf, 49, velE);
    _tta_put_float(buf, 53, velD);
    _tta_put_float(buf, 57, diff_distance);
    _tta_put_float(buf, 61, flight_distance);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POS_LOCATION_LEN);
#else
    ttalink_pos_location_t packet;
    packet.switch_data = switch_data;
    packet.nstar = nstar;
    packet.gps_dop = gps_dop;
    packet.rtk_nstar = rtk_nstar;
    packet.rtk_pos_type = rtk_pos_type;
    packet.rtk_head_type = rtk_head_type;
    packet.latit = latit;
    packet.longi = longi;
    packet.alt = alt;
    packet.target_alt = target_alt;
    packet.cur_alt = cur_alt;
    packet.altitude = altitude;
    packet.relative_altitude = relative_altitude;
    packet.target_speed = target_speed;
    packet.ground_speed = ground_speed;
    packet.velN = velN;
    packet.velE = velE;
    packet.velD = velD;
    packet.diff_distance = diff_distance;
    packet.flight_distance = flight_distance;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POS_LOCATION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POS_LOCATION;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POS_LOCATION_MIN_LEN, TTALINK_MSG_ID_POS_LOCATION_LEN, TTALINK_MSG_ID_POS_LOCATION_CRC, nocrc);
}

/**
 * @brief Pack a pos_location message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param switch_data  
 * @param nstar   .
 * @param gps_dop   .
 * @param rtk_nstar   .
 * @param rtk_pos_type   .
 * @param rtk_head_type   .
 * @param latit   .
 * @param longi   .
 * @param alt   .
 * @param target_alt   .
 * @param cur_alt   .
 * @param altitude   .
 * @param relative_altitude   .
 * @param target_speed   .
 * @param ground_speed   .
 * @param velN   .
 * @param velE   .
 * @param velD   .
 * @param diff_distance   .
 * @param flight_distance   .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pos_location_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t switch_data,uint8_t nstar,float gps_dop,uint8_t rtk_nstar,uint8_t rtk_pos_type,uint8_t rtk_head_type,int32_t latit,int32_t longi,float alt,float target_alt,float cur_alt,float altitude,float relative_altitude,float target_speed,float ground_speed,float velN,float velE,float velD,float diff_distance,float flight_distance)
{
    return _ttalink_pos_location_pack_chan(dst_addr, src_addr, chan, msg,  switch_data, nstar, gps_dop, rtk_nstar, rtk_pos_type, rtk_head_type, latit, longi, alt, target_alt, cur_alt, altitude, relative_altitude, target_speed, ground_speed, velN, velE, velD, diff_distance, flight_distance, false);
}

/**
 * @brief Pack a pos_location message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param switch_data  
 * @param nstar   .
 * @param gps_dop   .
 * @param rtk_nstar   .
 * @param rtk_pos_type   .
 * @param rtk_head_type   .
 * @param latit   .
 * @param longi   .
 * @param alt   .
 * @param target_alt   .
 * @param cur_alt   .
 * @param altitude   .
 * @param relative_altitude   .
 * @param target_speed   .
 * @param ground_speed   .
 * @param velN   .
 * @param velE   .
 * @param velD   .
 * @param diff_distance   .
 * @param flight_distance   .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pos_location_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t switch_data,uint8_t nstar,float gps_dop,uint8_t rtk_nstar,uint8_t rtk_pos_type,uint8_t rtk_head_type,int32_t latit,int32_t longi,float alt,float target_alt,float cur_alt,float altitude,float relative_altitude,float target_speed,float ground_speed,float velN,float velE,float velD,float diff_distance,float flight_distance)
{
    return _ttalink_pos_location_pack_chan(dst_addr, src_addr, chan, msg,  switch_data, nstar, gps_dop, rtk_nstar, rtk_pos_type, rtk_head_type, latit, longi, alt, target_alt, cur_alt, altitude, relative_altitude, target_speed, ground_speed, velN, velE, velD, diff_distance, flight_distance, true);
}


static inline uint16_t _ttalink_pos_location_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pos_location_t* pos_location, bool nocrc)
{
    if(nocrc){
        return ttalink_pos_location_pack_nocrc(dst_addr, src_addr, msg, pos_location->switch_data, pos_location->nstar, pos_location->gps_dop, pos_location->rtk_nstar, pos_location->rtk_pos_type, pos_location->rtk_head_type, pos_location->latit, pos_location->longi, pos_location->alt, pos_location->target_alt, pos_location->cur_alt, pos_location->altitude, pos_location->relative_altitude, pos_location->target_speed, pos_location->ground_speed, pos_location->velN, pos_location->velE, pos_location->velD, pos_location->diff_distance, pos_location->flight_distance);
    }else{
        return ttalink_pos_location_pack(dst_addr, src_addr, msg, pos_location->switch_data, pos_location->nstar, pos_location->gps_dop, pos_location->rtk_nstar, pos_location->rtk_pos_type, pos_location->rtk_head_type, pos_location->latit, pos_location->longi, pos_location->alt, pos_location->target_alt, pos_location->cur_alt, pos_location->altitude, pos_location->relative_altitude, pos_location->target_speed, pos_location->ground_speed, pos_location->velN, pos_location->velE, pos_location->velD, pos_location->diff_distance, pos_location->flight_distance);
    }
    
}

/**
 * @brief Encode a pos_location struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pos_location C-struct to read the message contents from
 */
static inline uint16_t ttalink_pos_location_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pos_location_t* pos_location)
{
    return _ttalink_pos_location_encode(dst_addr, src_addr, msg, pos_location, false);
}

/**
 * @brief Encode a pos_location struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pos_location C-struct to read the message contents from
 */
static inline uint16_t ttalink_pos_location_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pos_location_t* pos_location)
{
    return _ttalink_pos_location_encode(dst_addr, src_addr, msg, pos_location, true);
}


static inline uint16_t _ttalink_pos_location_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pos_location_t* pos_location, bool nocrc)
{
    if(nocrc){
        return ttalink_pos_location_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pos_location->switch_data, pos_location->nstar, pos_location->gps_dop, pos_location->rtk_nstar, pos_location->rtk_pos_type, pos_location->rtk_head_type, pos_location->latit, pos_location->longi, pos_location->alt, pos_location->target_alt, pos_location->cur_alt, pos_location->altitude, pos_location->relative_altitude, pos_location->target_speed, pos_location->ground_speed, pos_location->velN, pos_location->velE, pos_location->velD, pos_location->diff_distance, pos_location->flight_distance);
    }else{
        return ttalink_pos_location_pack_chan(dst_addr, src_addr, chan, msg, pos_location->switch_data, pos_location->nstar, pos_location->gps_dop, pos_location->rtk_nstar, pos_location->rtk_pos_type, pos_location->rtk_head_type, pos_location->latit, pos_location->longi, pos_location->alt, pos_location->target_alt, pos_location->cur_alt, pos_location->altitude, pos_location->relative_altitude, pos_location->target_speed, pos_location->ground_speed, pos_location->velN, pos_location->velE, pos_location->velD, pos_location->diff_distance, pos_location->flight_distance);
    }
}

/**
 * @brief Encode a pos_location struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pos_location C-struct to read the message contents from
 */
static inline uint16_t ttalink_pos_location_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pos_location_t* pos_location)
{
    return _ttalink_pos_location_encode_chan(dst_addr, src_addr, chan, msg, pos_location, false);
}

/**
 * @brief Encode a pos_location struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pos_location C-struct to read the message contents from
 */
static inline uint16_t ttalink_pos_location_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pos_location_t* pos_location)
{
    return _ttalink_pos_location_encode_chan(dst_addr, src_addr, chan, msg, pos_location, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pos_location_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t switch_data, uint8_t nstar, float gps_dop, uint8_t rtk_nstar, uint8_t rtk_pos_type, uint8_t rtk_head_type, int32_t latit, int32_t longi, float alt, float target_alt, float cur_alt, float altitude, float relative_altitude, float target_speed, float ground_speed, float velN, float velE, float velD, float diff_distance, float flight_distance, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POS_LOCATION_LEN];
    _tta_put_uint8_t(buf, 0, switch_data);
    _tta_put_uint8_t(buf, 1, nstar);
    _tta_put_float(buf, 2, gps_dop);
    _tta_put_uint8_t(buf, 6, rtk_nstar);
    _tta_put_uint8_t(buf, 7, rtk_pos_type);
    _tta_put_uint8_t(buf, 8, rtk_head_type);
    _tta_put_int32_t(buf, 9, latit);
    _tta_put_int32_t(buf, 13, longi);
    _tta_put_float(buf, 17, alt);
    _tta_put_float(buf, 21, target_alt);
    _tta_put_float(buf, 25, cur_alt);
    _tta_put_float(buf, 29, altitude);
    _tta_put_float(buf, 33, relative_altitude);
    _tta_put_float(buf, 37, target_speed);
    _tta_put_float(buf, 41, ground_speed);
    _tta_put_float(buf, 45, velN);
    _tta_put_float(buf, 49, velE);
    _tta_put_float(buf, 53, velD);
    _tta_put_float(buf, 57, diff_distance);
    _tta_put_float(buf, 61, flight_distance);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POS_LOCATION, buf, TTALINK_MSG_ID_POS_LOCATION_MIN_LEN, TTALINK_MSG_ID_POS_LOCATION_LEN, TTALINK_MSG_ID_POS_LOCATION_CRC, nocrc);
#else
    ttalink_pos_location_t packet;
    packet.switch_data = switch_data;
    packet.nstar = nstar;
    packet.gps_dop = gps_dop;
    packet.rtk_nstar = rtk_nstar;
    packet.rtk_pos_type = rtk_pos_type;
    packet.rtk_head_type = rtk_head_type;
    packet.latit = latit;
    packet.longi = longi;
    packet.alt = alt;
    packet.target_alt = target_alt;
    packet.cur_alt = cur_alt;
    packet.altitude = altitude;
    packet.relative_altitude = relative_altitude;
    packet.target_speed = target_speed;
    packet.ground_speed = ground_speed;
    packet.velN = velN;
    packet.velE = velE;
    packet.velD = velD;
    packet.diff_distance = diff_distance;
    packet.flight_distance = flight_distance;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POS_LOCATION, (const char *)&packet, TTALINK_MSG_ID_POS_LOCATION_MIN_LEN, TTALINK_MSG_ID_POS_LOCATION_LEN, TTALINK_MSG_ID_POS_LOCATION_CRC, nocrc);
#endif
}

/**
 * @brief Send a pos_location message
 * @param chan TTAlink channel to send the message
 *
 * @param switch_data  
 * @param nstar   .
 * @param gps_dop   .
 * @param rtk_nstar   .
 * @param rtk_pos_type   .
 * @param rtk_head_type   .
 * @param latit   .
 * @param longi   .
 * @param alt   .
 * @param target_alt   .
 * @param cur_alt   .
 * @param altitude   .
 * @param relative_altitude   .
 * @param target_speed   .
 * @param ground_speed   .
 * @param velN   .
 * @param velE   .
 * @param velD   .
 * @param diff_distance   .
 * @param flight_distance   .
 */
static inline void ttalink_pos_location_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t switch_data, uint8_t nstar, float gps_dop, uint8_t rtk_nstar, uint8_t rtk_pos_type, uint8_t rtk_head_type, int32_t latit, int32_t longi, float alt, float target_alt, float cur_alt, float altitude, float relative_altitude, float target_speed, float ground_speed, float velN, float velE, float velD, float diff_distance, float flight_distance)
{
    _ttalink_pos_location_send(dst_addr, src_addr, chan, switch_data, nstar, gps_dop, rtk_nstar, rtk_pos_type, rtk_head_type, latit, longi, alt, target_alt, cur_alt, altitude, relative_altitude, target_speed, ground_speed, velN, velE, velD, diff_distance, flight_distance, false);
}

/**
 * @brief Send a pos_location message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param switch_data  
 * @param nstar   .
 * @param gps_dop   .
 * @param rtk_nstar   .
 * @param rtk_pos_type   .
 * @param rtk_head_type   .
 * @param latit   .
 * @param longi   .
 * @param alt   .
 * @param target_alt   .
 * @param cur_alt   .
 * @param altitude   .
 * @param relative_altitude   .
 * @param target_speed   .
 * @param ground_speed   .
 * @param velN   .
 * @param velE   .
 * @param velD   .
 * @param diff_distance   .
 * @param flight_distance   .
 */
static inline void ttalink_pos_location_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t switch_data, uint8_t nstar, float gps_dop, uint8_t rtk_nstar, uint8_t rtk_pos_type, uint8_t rtk_head_type, int32_t latit, int32_t longi, float alt, float target_alt, float cur_alt, float altitude, float relative_altitude, float target_speed, float ground_speed, float velN, float velE, float velD, float diff_distance, float flight_distance)
{
    _ttalink_pos_location_send(dst_addr, src_addr, chan, switch_data, nstar, gps_dop, rtk_nstar, rtk_pos_type, rtk_head_type, latit, longi, alt, target_alt, cur_alt, altitude, relative_altitude, target_speed, ground_speed, velN, velE, velD, diff_distance, flight_distance, true);
}


static inline void _ttalink_pos_location_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pos_location_t* pos_location, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pos_location_send_nocrc(dst_addr, src_addr, chan, pos_location->switch_data, pos_location->nstar, pos_location->gps_dop, pos_location->rtk_nstar, pos_location->rtk_pos_type, pos_location->rtk_head_type, pos_location->latit, pos_location->longi, pos_location->alt, pos_location->target_alt, pos_location->cur_alt, pos_location->altitude, pos_location->relative_altitude, pos_location->target_speed, pos_location->ground_speed, pos_location->velN, pos_location->velE, pos_location->velD, pos_location->diff_distance, pos_location->flight_distance);
    }else{
        ttalink_pos_location_send(dst_addr, src_addr, chan, pos_location->switch_data, pos_location->nstar, pos_location->gps_dop, pos_location->rtk_nstar, pos_location->rtk_pos_type, pos_location->rtk_head_type, pos_location->latit, pos_location->longi, pos_location->alt, pos_location->target_alt, pos_location->cur_alt, pos_location->altitude, pos_location->relative_altitude, pos_location->target_speed, pos_location->ground_speed, pos_location->velN, pos_location->velE, pos_location->velD, pos_location->diff_distance, pos_location->flight_distance);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POS_LOCATION, (const char *)pos_location, TTALINK_MSG_ID_POS_LOCATION_MIN_LEN, TTALINK_MSG_ID_POS_LOCATION_LEN, TTALINK_MSG_ID_POS_LOCATION_CRC, nocrc);
#endif
}

/**
 * @brief Send a pos_location message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pos_location_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pos_location_t* pos_location)
{
    _ttalink_pos_location_send_struct(dst_addr, src_addr, chan, pos_location, false);
}

/**
 * @brief Send a pos_location message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pos_location_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pos_location_t* pos_location)
{
    _ttalink_pos_location_send_struct(dst_addr, src_addr, chan, pos_location, true);
}

#if TTALINK_MSG_ID_POS_LOCATION_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pos_location_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t switch_data, uint8_t nstar, float gps_dop, uint8_t rtk_nstar, uint8_t rtk_pos_type, uint8_t rtk_head_type, int32_t latit, int32_t longi, float alt, float target_alt, float cur_alt, float altitude, float relative_altitude, float target_speed, float ground_speed, float velN, float velE, float velD, float diff_distance, float flight_distance, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, switch_data);
    _tta_put_uint8_t(buf, 1, nstar);
    _tta_put_float(buf, 2, gps_dop);
    _tta_put_uint8_t(buf, 6, rtk_nstar);
    _tta_put_uint8_t(buf, 7, rtk_pos_type);
    _tta_put_uint8_t(buf, 8, rtk_head_type);
    _tta_put_int32_t(buf, 9, latit);
    _tta_put_int32_t(buf, 13, longi);
    _tta_put_float(buf, 17, alt);
    _tta_put_float(buf, 21, target_alt);
    _tta_put_float(buf, 25, cur_alt);
    _tta_put_float(buf, 29, altitude);
    _tta_put_float(buf, 33, relative_altitude);
    _tta_put_float(buf, 37, target_speed);
    _tta_put_float(buf, 41, ground_speed);
    _tta_put_float(buf, 45, velN);
    _tta_put_float(buf, 49, velE);
    _tta_put_float(buf, 53, velD);
    _tta_put_float(buf, 57, diff_distance);
    _tta_put_float(buf, 61, flight_distance);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POS_LOCATION, buf, TTALINK_MSG_ID_POS_LOCATION_MIN_LEN, TTALINK_MSG_ID_POS_LOCATION_LEN, TTALINK_MSG_ID_POS_LOCATION_CRC, nocrc);
#else
    ttalink_pos_location_t *packet = (ttalink_pos_location_t *)msgbuf;
    packet->switch_data = switch_data;
    packet->nstar = nstar;
    packet->gps_dop = gps_dop;
    packet->rtk_nstar = rtk_nstar;
    packet->rtk_pos_type = rtk_pos_type;
    packet->rtk_head_type = rtk_head_type;
    packet->latit = latit;
    packet->longi = longi;
    packet->alt = alt;
    packet->target_alt = target_alt;
    packet->cur_alt = cur_alt;
    packet->altitude = altitude;
    packet->relative_altitude = relative_altitude;
    packet->target_speed = target_speed;
    packet->ground_speed = ground_speed;
    packet->velN = velN;
    packet->velE = velE;
    packet->velD = velD;
    packet->diff_distance = diff_distance;
    packet->flight_distance = flight_distance;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POS_LOCATION, (const char *)packet, TTALINK_MSG_ID_POS_LOCATION_MIN_LEN, TTALINK_MSG_ID_POS_LOCATION_LEN, TTALINK_MSG_ID_POS_LOCATION_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pos_location_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t switch_data, uint8_t nstar, float gps_dop, uint8_t rtk_nstar, uint8_t rtk_pos_type, uint8_t rtk_head_type, int32_t latit, int32_t longi, float alt, float target_alt, float cur_alt, float altitude, float relative_altitude, float target_speed, float ground_speed, float velN, float velE, float velD, float diff_distance, float flight_distance)
{
    _ttalink_pos_location_send_buf(dst_addr, src_addr, msgbuf, chan, switch_data, nstar, gps_dop, rtk_nstar, rtk_pos_type, rtk_head_type, latit, longi, alt, target_alt, cur_alt, altitude, relative_altitude, target_speed, ground_speed, velN, velE, velD, diff_distance, flight_distance, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pos_location_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t switch_data, uint8_t nstar, float gps_dop, uint8_t rtk_nstar, uint8_t rtk_pos_type, uint8_t rtk_head_type, int32_t latit, int32_t longi, float alt, float target_alt, float cur_alt, float altitude, float relative_altitude, float target_speed, float ground_speed, float velN, float velE, float velD, float diff_distance, float flight_distance)
{
    _ttalink_pos_location_send_buf(dst_addr, src_addr, msgbuf, chan, switch_data, nstar, gps_dop, rtk_nstar, rtk_pos_type, rtk_head_type, latit, longi, alt, target_alt, cur_alt, altitude, relative_altitude, target_speed, ground_speed, velN, velE, velD, diff_distance, flight_distance, true);
}
#endif

#endif

// MESSAGE POS_LOCATION UNPACKING


/**
 * @brief Get field switch_data from pos_location message
 *
 * @return  
 */
static inline uint8_t ttalink_pos_location_get_switch_data(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field nstar from pos_location message
 *
 * @return   .
 */
static inline uint8_t ttalink_pos_location_get_nstar(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field gps_dop from pos_location message
 *
 * @return   .
 */
static inline float ttalink_pos_location_get_gps_dop(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  2);
}

/**
 * @brief Get field rtk_nstar from pos_location message
 *
 * @return   .
 */
static inline uint8_t ttalink_pos_location_get_rtk_nstar(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field rtk_pos_type from pos_location message
 *
 * @return   .
 */
static inline uint8_t ttalink_pos_location_get_rtk_pos_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field rtk_head_type from pos_location message
 *
 * @return   .
 */
static inline uint8_t ttalink_pos_location_get_rtk_head_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field latit from pos_location message
 *
 * @return   .
 */
static inline int32_t ttalink_pos_location_get_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  9);
}

/**
 * @brief Get field longi from pos_location message
 *
 * @return   .
 */
static inline int32_t ttalink_pos_location_get_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  13);
}

/**
 * @brief Get field alt from pos_location message
 *
 * @return   .
 */
static inline float ttalink_pos_location_get_alt(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  17);
}

/**
 * @brief Get field target_alt from pos_location message
 *
 * @return   .
 */
static inline float ttalink_pos_location_get_target_alt(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  21);
}

/**
 * @brief Get field cur_alt from pos_location message
 *
 * @return   .
 */
static inline float ttalink_pos_location_get_cur_alt(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  25);
}

/**
 * @brief Get field altitude from pos_location message
 *
 * @return   .
 */
static inline float ttalink_pos_location_get_altitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  29);
}

/**
 * @brief Get field relative_altitude from pos_location message
 *
 * @return   .
 */
static inline float ttalink_pos_location_get_relative_altitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  33);
}

/**
 * @brief Get field target_speed from pos_location message
 *
 * @return   .
 */
static inline float ttalink_pos_location_get_target_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  37);
}

/**
 * @brief Get field ground_speed from pos_location message
 *
 * @return   .
 */
static inline float ttalink_pos_location_get_ground_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  41);
}

/**
 * @brief Get field velN from pos_location message
 *
 * @return   .
 */
static inline float ttalink_pos_location_get_velN(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  45);
}

/**
 * @brief Get field velE from pos_location message
 *
 * @return   .
 */
static inline float ttalink_pos_location_get_velE(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  49);
}

/**
 * @brief Get field velD from pos_location message
 *
 * @return   .
 */
static inline float ttalink_pos_location_get_velD(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  53);
}

/**
 * @brief Get field diff_distance from pos_location message
 *
 * @return   .
 */
static inline float ttalink_pos_location_get_diff_distance(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  57);
}

/**
 * @brief Get field flight_distance from pos_location message
 *
 * @return   .
 */
static inline float ttalink_pos_location_get_flight_distance(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  61);
}

/**
 * @brief Decode a pos_location message into a struct
 *
 * @param msg The message to decode
 * @param pos_location C-struct to decode the message contents into
 */
static inline void ttalink_pos_location_decode(const ttalink_message_t* msg, ttalink_pos_location_t* pos_location)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pos_location->switch_data = ttalink_pos_location_get_switch_data(msg);
    pos_location->nstar = ttalink_pos_location_get_nstar(msg);
    pos_location->gps_dop = ttalink_pos_location_get_gps_dop(msg);
    pos_location->rtk_nstar = ttalink_pos_location_get_rtk_nstar(msg);
    pos_location->rtk_pos_type = ttalink_pos_location_get_rtk_pos_type(msg);
    pos_location->rtk_head_type = ttalink_pos_location_get_rtk_head_type(msg);
    pos_location->latit = ttalink_pos_location_get_latit(msg);
    pos_location->longi = ttalink_pos_location_get_longi(msg);
    pos_location->alt = ttalink_pos_location_get_alt(msg);
    pos_location->target_alt = ttalink_pos_location_get_target_alt(msg);
    pos_location->cur_alt = ttalink_pos_location_get_cur_alt(msg);
    pos_location->altitude = ttalink_pos_location_get_altitude(msg);
    pos_location->relative_altitude = ttalink_pos_location_get_relative_altitude(msg);
    pos_location->target_speed = ttalink_pos_location_get_target_speed(msg);
    pos_location->ground_speed = ttalink_pos_location_get_ground_speed(msg);
    pos_location->velN = ttalink_pos_location_get_velN(msg);
    pos_location->velE = ttalink_pos_location_get_velE(msg);
    pos_location->velD = ttalink_pos_location_get_velD(msg);
    pos_location->diff_distance = ttalink_pos_location_get_diff_distance(msg);
    pos_location->flight_distance = ttalink_pos_location_get_flight_distance(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POS_LOCATION_LEN? msg->len : TTALINK_MSG_ID_POS_LOCATION_LEN;
        memset(pos_location, 0, TTALINK_MSG_ID_POS_LOCATION_LEN);
    memcpy(pos_location, _TTA_PAYLOAD(msg), len);
#endif
}
