#pragma once
// MESSAGE PTZ_POSITION PACKING

#define TTALINK_MSG_ID_PTZ_POSITION 2301

TTAPACKED(
typedef struct __ttalink_ptz_position_t {
 uint32_t update_time; /*<  update_time*/
 uint8_t sensor_index; /*<  .*/
 uint8_t gps_nstar; /*<  .*/
 float gps_dop; /*<  .*/
 uint8_t rtk_nstar; /*<  .*/
 uint8_t rtk_pos_type; /*<  .*/
 uint8_t rtk_head_type; /*<  .*/
 float rtk_heading; /*<  .*/
 int32_t longi; /*<  .*/
 int32_t latit; /*<  .*/
 float altit; /*<  .*/
 float velN; /*<  .*/
 float velE; /*<  .*/
 float velD; /*<  .*/
}) ttalink_ptz_position_t;

#define TTALINK_MSG_ID_PTZ_POSITION_LEN 41
#define TTALINK_MSG_ID_PTZ_POSITION_MIN_LEN 41
#define TTALINK_MSG_ID_2301_LEN 41
#define TTALINK_MSG_ID_2301_MIN_LEN 41

#define TTALINK_MSG_ID_PTZ_POSITION_CRC 93
#define TTALINK_MSG_ID_2301_CRC 93



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_PTZ_POSITION { \
    2301, \
    "PTZ_POSITION", \
    14, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_ptz_position_t, update_time) }, \
         { "sensor_index", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_ptz_position_t, sensor_index) }, \
         { "gps_nstar", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_ptz_position_t, gps_nstar) }, \
         { "gps_dop", NULL, TTALINK_TYPE_FLOAT, 0, 6, offsetof(ttalink_ptz_position_t, gps_dop) }, \
         { "rtk_nstar", NULL, TTALINK_TYPE_UINT8_T, 0, 10, offsetof(ttalink_ptz_position_t, rtk_nstar) }, \
         { "rtk_pos_type", NULL, TTALINK_TYPE_UINT8_T, 0, 11, offsetof(ttalink_ptz_position_t, rtk_pos_type) }, \
         { "rtk_head_type", NULL, TTALINK_TYPE_UINT8_T, 0, 12, offsetof(ttalink_ptz_position_t, rtk_head_type) }, \
         { "rtk_heading", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_ptz_position_t, rtk_heading) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 17, offsetof(ttalink_ptz_position_t, longi) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 21, offsetof(ttalink_ptz_position_t, latit) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_ptz_position_t, altit) }, \
         { "velN", NULL, TTALINK_TYPE_FLOAT, 0, 29, offsetof(ttalink_ptz_position_t, velN) }, \
         { "velE", NULL, TTALINK_TYPE_FLOAT, 0, 33, offsetof(ttalink_ptz_position_t, velE) }, \
         { "velD", NULL, TTALINK_TYPE_FLOAT, 0, 37, offsetof(ttalink_ptz_position_t, velD) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_PTZ_POSITION { \
    "PTZ_POSITION", \
    14, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_ptz_position_t, update_time) }, \
         { "sensor_index", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_ptz_position_t, sensor_index) }, \
         { "gps_nstar", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_ptz_position_t, gps_nstar) }, \
         { "gps_dop", NULL, TTALINK_TYPE_FLOAT, 0, 6, offsetof(ttalink_ptz_position_t, gps_dop) }, \
         { "rtk_nstar", NULL, TTALINK_TYPE_UINT8_T, 0, 10, offsetof(ttalink_ptz_position_t, rtk_nstar) }, \
         { "rtk_pos_type", NULL, TTALINK_TYPE_UINT8_T, 0, 11, offsetof(ttalink_ptz_position_t, rtk_pos_type) }, \
         { "rtk_head_type", NULL, TTALINK_TYPE_UINT8_T, 0, 12, offsetof(ttalink_ptz_position_t, rtk_head_type) }, \
         { "rtk_heading", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_ptz_position_t, rtk_heading) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 17, offsetof(ttalink_ptz_position_t, longi) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 21, offsetof(ttalink_ptz_position_t, latit) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_ptz_position_t, altit) }, \
         { "velN", NULL, TTALINK_TYPE_FLOAT, 0, 29, offsetof(ttalink_ptz_position_t, velN) }, \
         { "velE", NULL, TTALINK_TYPE_FLOAT, 0, 33, offsetof(ttalink_ptz_position_t, velE) }, \
         { "velD", NULL, TTALINK_TYPE_FLOAT, 0, 37, offsetof(ttalink_ptz_position_t, velD) }, \
         } \
}
#endif


static inline uint16_t _ttalink_ptz_position_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t sensor_index, uint8_t gps_nstar, float gps_dop, uint8_t rtk_nstar, uint8_t rtk_pos_type, uint8_t rtk_head_type, float rtk_heading, int32_t longi, int32_t latit, float altit, float velN, float velE, float velD, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PTZ_POSITION_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, sensor_index);
    _tta_put_uint8_t(buf, 5, gps_nstar);
    _tta_put_float(buf, 6, gps_dop);
    _tta_put_uint8_t(buf, 10, rtk_nstar);
    _tta_put_uint8_t(buf, 11, rtk_pos_type);
    _tta_put_uint8_t(buf, 12, rtk_head_type);
    _tta_put_float(buf, 13, rtk_heading);
    _tta_put_int32_t(buf, 17, longi);
    _tta_put_int32_t(buf, 21, latit);
    _tta_put_float(buf, 25, altit);
    _tta_put_float(buf, 29, velN);
    _tta_put_float(buf, 33, velE);
    _tta_put_float(buf, 37, velD);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PTZ_POSITION_LEN);
#else
    ttalink_ptz_position_t packet;
    packet.update_time = update_time;
    packet.sensor_index = sensor_index;
    packet.gps_nstar = gps_nstar;
    packet.gps_dop = gps_dop;
    packet.rtk_nstar = rtk_nstar;
    packet.rtk_pos_type = rtk_pos_type;
    packet.rtk_head_type = rtk_head_type;
    packet.rtk_heading = rtk_heading;
    packet.longi = longi;
    packet.latit = latit;
    packet.altit = altit;
    packet.velN = velN;
    packet.velE = velE;
    packet.velD = velD;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PTZ_POSITION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PTZ_POSITION;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_PTZ_POSITION_MIN_LEN, TTALINK_MSG_ID_PTZ_POSITION_LEN, TTALINK_MSG_ID_PTZ_POSITION_CRC, nocrc);
}

/**
 * @brief Pack a ptz_position message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param sensor_index  .
 * @param gps_nstar  .
 * @param gps_dop  .
 * @param rtk_nstar  .
 * @param rtk_pos_type  .
 * @param rtk_head_type  .
 * @param rtk_heading  .
 * @param longi  .
 * @param latit  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_position_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t sensor_index, uint8_t gps_nstar, float gps_dop, uint8_t rtk_nstar, uint8_t rtk_pos_type, uint8_t rtk_head_type, float rtk_heading, int32_t longi, int32_t latit, float altit, float velN, float velE, float velD)
{
    return _ttalink_ptz_position_pack(dst_addr, src_addr, msg,  update_time, sensor_index, gps_nstar, gps_dop, rtk_nstar, rtk_pos_type, rtk_head_type, rtk_heading, longi, latit, altit, velN, velE, velD, false);
}

/**
 * @brief Pack a ptz_position message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param sensor_index  .
 * @param gps_nstar  .
 * @param gps_dop  .
 * @param rtk_nstar  .
 * @param rtk_pos_type  .
 * @param rtk_head_type  .
 * @param rtk_heading  .
 * @param longi  .
 * @param latit  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_position_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t sensor_index, uint8_t gps_nstar, float gps_dop, uint8_t rtk_nstar, uint8_t rtk_pos_type, uint8_t rtk_head_type, float rtk_heading, int32_t longi, int32_t latit, float altit, float velN, float velE, float velD)
{
    return _ttalink_ptz_position_pack(dst_addr, src_addr, msg,  update_time, sensor_index, gps_nstar, gps_dop, rtk_nstar, rtk_pos_type, rtk_head_type, rtk_heading, longi, latit, altit, velN, velE, velD, true);
}


static inline uint16_t _ttalink_ptz_position_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t sensor_index,uint8_t gps_nstar,float gps_dop,uint8_t rtk_nstar,uint8_t rtk_pos_type,uint8_t rtk_head_type,float rtk_heading,int32_t longi,int32_t latit,float altit,float velN,float velE,float velD, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PTZ_POSITION_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, sensor_index);
    _tta_put_uint8_t(buf, 5, gps_nstar);
    _tta_put_float(buf, 6, gps_dop);
    _tta_put_uint8_t(buf, 10, rtk_nstar);
    _tta_put_uint8_t(buf, 11, rtk_pos_type);
    _tta_put_uint8_t(buf, 12, rtk_head_type);
    _tta_put_float(buf, 13, rtk_heading);
    _tta_put_int32_t(buf, 17, longi);
    _tta_put_int32_t(buf, 21, latit);
    _tta_put_float(buf, 25, altit);
    _tta_put_float(buf, 29, velN);
    _tta_put_float(buf, 33, velE);
    _tta_put_float(buf, 37, velD);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PTZ_POSITION_LEN);
#else
    ttalink_ptz_position_t packet;
    packet.update_time = update_time;
    packet.sensor_index = sensor_index;
    packet.gps_nstar = gps_nstar;
    packet.gps_dop = gps_dop;
    packet.rtk_nstar = rtk_nstar;
    packet.rtk_pos_type = rtk_pos_type;
    packet.rtk_head_type = rtk_head_type;
    packet.rtk_heading = rtk_heading;
    packet.longi = longi;
    packet.latit = latit;
    packet.altit = altit;
    packet.velN = velN;
    packet.velE = velE;
    packet.velD = velD;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PTZ_POSITION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PTZ_POSITION;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_POSITION_MIN_LEN, TTALINK_MSG_ID_PTZ_POSITION_LEN, TTALINK_MSG_ID_PTZ_POSITION_CRC, nocrc);
}

/**
 * @brief Pack a ptz_position message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param sensor_index  .
 * @param gps_nstar  .
 * @param gps_dop  .
 * @param rtk_nstar  .
 * @param rtk_pos_type  .
 * @param rtk_head_type  .
 * @param rtk_heading  .
 * @param longi  .
 * @param latit  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_position_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t sensor_index,uint8_t gps_nstar,float gps_dop,uint8_t rtk_nstar,uint8_t rtk_pos_type,uint8_t rtk_head_type,float rtk_heading,int32_t longi,int32_t latit,float altit,float velN,float velE,float velD)
{
    return _ttalink_ptz_position_pack_chan(dst_addr, src_addr, chan, msg,  update_time, sensor_index, gps_nstar, gps_dop, rtk_nstar, rtk_pos_type, rtk_head_type, rtk_heading, longi, latit, altit, velN, velE, velD, false);
}

/**
 * @brief Pack a ptz_position message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param sensor_index  .
 * @param gps_nstar  .
 * @param gps_dop  .
 * @param rtk_nstar  .
 * @param rtk_pos_type  .
 * @param rtk_head_type  .
 * @param rtk_heading  .
 * @param longi  .
 * @param latit  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_position_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t sensor_index,uint8_t gps_nstar,float gps_dop,uint8_t rtk_nstar,uint8_t rtk_pos_type,uint8_t rtk_head_type,float rtk_heading,int32_t longi,int32_t latit,float altit,float velN,float velE,float velD)
{
    return _ttalink_ptz_position_pack_chan(dst_addr, src_addr, chan, msg,  update_time, sensor_index, gps_nstar, gps_dop, rtk_nstar, rtk_pos_type, rtk_head_type, rtk_heading, longi, latit, altit, velN, velE, velD, true);
}


static inline uint16_t _ttalink_ptz_position_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_ptz_position_t* ptz_position, bool nocrc)
{
    if(nocrc){
        return ttalink_ptz_position_pack_nocrc(dst_addr, src_addr, msg, ptz_position->update_time, ptz_position->sensor_index, ptz_position->gps_nstar, ptz_position->gps_dop, ptz_position->rtk_nstar, ptz_position->rtk_pos_type, ptz_position->rtk_head_type, ptz_position->rtk_heading, ptz_position->longi, ptz_position->latit, ptz_position->altit, ptz_position->velN, ptz_position->velE, ptz_position->velD);
    }else{
        return ttalink_ptz_position_pack(dst_addr, src_addr, msg, ptz_position->update_time, ptz_position->sensor_index, ptz_position->gps_nstar, ptz_position->gps_dop, ptz_position->rtk_nstar, ptz_position->rtk_pos_type, ptz_position->rtk_head_type, ptz_position->rtk_heading, ptz_position->longi, ptz_position->latit, ptz_position->altit, ptz_position->velN, ptz_position->velE, ptz_position->velD);
    }
    
}

/**
 * @brief Encode a ptz_position struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param ptz_position C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_position_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_ptz_position_t* ptz_position)
{
    return _ttalink_ptz_position_encode(dst_addr, src_addr, msg, ptz_position, false);
}

/**
 * @brief Encode a ptz_position struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param ptz_position C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_position_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_ptz_position_t* ptz_position)
{
    return _ttalink_ptz_position_encode(dst_addr, src_addr, msg, ptz_position, true);
}


static inline uint16_t _ttalink_ptz_position_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_ptz_position_t* ptz_position, bool nocrc)
{
    if(nocrc){
        return ttalink_ptz_position_pack_chan_nocrc(dst_addr, src_addr, chan, msg, ptz_position->update_time, ptz_position->sensor_index, ptz_position->gps_nstar, ptz_position->gps_dop, ptz_position->rtk_nstar, ptz_position->rtk_pos_type, ptz_position->rtk_head_type, ptz_position->rtk_heading, ptz_position->longi, ptz_position->latit, ptz_position->altit, ptz_position->velN, ptz_position->velE, ptz_position->velD);
    }else{
        return ttalink_ptz_position_pack_chan(dst_addr, src_addr, chan, msg, ptz_position->update_time, ptz_position->sensor_index, ptz_position->gps_nstar, ptz_position->gps_dop, ptz_position->rtk_nstar, ptz_position->rtk_pos_type, ptz_position->rtk_head_type, ptz_position->rtk_heading, ptz_position->longi, ptz_position->latit, ptz_position->altit, ptz_position->velN, ptz_position->velE, ptz_position->velD);
    }
}

/**
 * @brief Encode a ptz_position struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ptz_position C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_position_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_ptz_position_t* ptz_position)
{
    return _ttalink_ptz_position_encode_chan(dst_addr, src_addr, chan, msg, ptz_position, false);
}

/**
 * @brief Encode a ptz_position struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ptz_position C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_position_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_ptz_position_t* ptz_position)
{
    return _ttalink_ptz_position_encode_chan(dst_addr, src_addr, chan, msg, ptz_position, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_ptz_position_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t sensor_index, uint8_t gps_nstar, float gps_dop, uint8_t rtk_nstar, uint8_t rtk_pos_type, uint8_t rtk_head_type, float rtk_heading, int32_t longi, int32_t latit, float altit, float velN, float velE, float velD, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PTZ_POSITION_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, sensor_index);
    _tta_put_uint8_t(buf, 5, gps_nstar);
    _tta_put_float(buf, 6, gps_dop);
    _tta_put_uint8_t(buf, 10, rtk_nstar);
    _tta_put_uint8_t(buf, 11, rtk_pos_type);
    _tta_put_uint8_t(buf, 12, rtk_head_type);
    _tta_put_float(buf, 13, rtk_heading);
    _tta_put_int32_t(buf, 17, longi);
    _tta_put_int32_t(buf, 21, latit);
    _tta_put_float(buf, 25, altit);
    _tta_put_float(buf, 29, velN);
    _tta_put_float(buf, 33, velE);
    _tta_put_float(buf, 37, velD);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_POSITION, buf, TTALINK_MSG_ID_PTZ_POSITION_MIN_LEN, TTALINK_MSG_ID_PTZ_POSITION_LEN, TTALINK_MSG_ID_PTZ_POSITION_CRC, nocrc);
#else
    ttalink_ptz_position_t packet;
    packet.update_time = update_time;
    packet.sensor_index = sensor_index;
    packet.gps_nstar = gps_nstar;
    packet.gps_dop = gps_dop;
    packet.rtk_nstar = rtk_nstar;
    packet.rtk_pos_type = rtk_pos_type;
    packet.rtk_head_type = rtk_head_type;
    packet.rtk_heading = rtk_heading;
    packet.longi = longi;
    packet.latit = latit;
    packet.altit = altit;
    packet.velN = velN;
    packet.velE = velE;
    packet.velD = velD;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_POSITION, (const char *)&packet, TTALINK_MSG_ID_PTZ_POSITION_MIN_LEN, TTALINK_MSG_ID_PTZ_POSITION_LEN, TTALINK_MSG_ID_PTZ_POSITION_CRC, nocrc);
#endif
}

/**
 * @brief Send a ptz_position message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param sensor_index  .
 * @param gps_nstar  .
 * @param gps_dop  .
 * @param rtk_nstar  .
 * @param rtk_pos_type  .
 * @param rtk_head_type  .
 * @param rtk_heading  .
 * @param longi  .
 * @param latit  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 */
static inline void ttalink_ptz_position_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t sensor_index, uint8_t gps_nstar, float gps_dop, uint8_t rtk_nstar, uint8_t rtk_pos_type, uint8_t rtk_head_type, float rtk_heading, int32_t longi, int32_t latit, float altit, float velN, float velE, float velD)
{
    _ttalink_ptz_position_send(dst_addr, src_addr, chan, update_time, sensor_index, gps_nstar, gps_dop, rtk_nstar, rtk_pos_type, rtk_head_type, rtk_heading, longi, latit, altit, velN, velE, velD, false);
}

/**
 * @brief Send a ptz_position message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param sensor_index  .
 * @param gps_nstar  .
 * @param gps_dop  .
 * @param rtk_nstar  .
 * @param rtk_pos_type  .
 * @param rtk_head_type  .
 * @param rtk_heading  .
 * @param longi  .
 * @param latit  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 */
static inline void ttalink_ptz_position_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t sensor_index, uint8_t gps_nstar, float gps_dop, uint8_t rtk_nstar, uint8_t rtk_pos_type, uint8_t rtk_head_type, float rtk_heading, int32_t longi, int32_t latit, float altit, float velN, float velE, float velD)
{
    _ttalink_ptz_position_send(dst_addr, src_addr, chan, update_time, sensor_index, gps_nstar, gps_dop, rtk_nstar, rtk_pos_type, rtk_head_type, rtk_heading, longi, latit, altit, velN, velE, velD, true);
}


static inline void _ttalink_ptz_position_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_ptz_position_t* ptz_position, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_ptz_position_send_nocrc(dst_addr, src_addr, chan, ptz_position->update_time, ptz_position->sensor_index, ptz_position->gps_nstar, ptz_position->gps_dop, ptz_position->rtk_nstar, ptz_position->rtk_pos_type, ptz_position->rtk_head_type, ptz_position->rtk_heading, ptz_position->longi, ptz_position->latit, ptz_position->altit, ptz_position->velN, ptz_position->velE, ptz_position->velD);
    }else{
        ttalink_ptz_position_send(dst_addr, src_addr, chan, ptz_position->update_time, ptz_position->sensor_index, ptz_position->gps_nstar, ptz_position->gps_dop, ptz_position->rtk_nstar, ptz_position->rtk_pos_type, ptz_position->rtk_head_type, ptz_position->rtk_heading, ptz_position->longi, ptz_position->latit, ptz_position->altit, ptz_position->velN, ptz_position->velE, ptz_position->velD);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_POSITION, (const char *)ptz_position, TTALINK_MSG_ID_PTZ_POSITION_MIN_LEN, TTALINK_MSG_ID_PTZ_POSITION_LEN, TTALINK_MSG_ID_PTZ_POSITION_CRC, nocrc);
#endif
}

/**
 * @brief Send a ptz_position message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_ptz_position_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_ptz_position_t* ptz_position)
{
    _ttalink_ptz_position_send_struct(dst_addr, src_addr, chan, ptz_position, false);
}

/**
 * @brief Send a ptz_position message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_ptz_position_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_ptz_position_t* ptz_position)
{
    _ttalink_ptz_position_send_struct(dst_addr, src_addr, chan, ptz_position, true);
}

#if TTALINK_MSG_ID_PTZ_POSITION_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_ptz_position_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t sensor_index, uint8_t gps_nstar, float gps_dop, uint8_t rtk_nstar, uint8_t rtk_pos_type, uint8_t rtk_head_type, float rtk_heading, int32_t longi, int32_t latit, float altit, float velN, float velE, float velD, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, sensor_index);
    _tta_put_uint8_t(buf, 5, gps_nstar);
    _tta_put_float(buf, 6, gps_dop);
    _tta_put_uint8_t(buf, 10, rtk_nstar);
    _tta_put_uint8_t(buf, 11, rtk_pos_type);
    _tta_put_uint8_t(buf, 12, rtk_head_type);
    _tta_put_float(buf, 13, rtk_heading);
    _tta_put_int32_t(buf, 17, longi);
    _tta_put_int32_t(buf, 21, latit);
    _tta_put_float(buf, 25, altit);
    _tta_put_float(buf, 29, velN);
    _tta_put_float(buf, 33, velE);
    _tta_put_float(buf, 37, velD);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_POSITION, buf, TTALINK_MSG_ID_PTZ_POSITION_MIN_LEN, TTALINK_MSG_ID_PTZ_POSITION_LEN, TTALINK_MSG_ID_PTZ_POSITION_CRC, nocrc);
#else
    ttalink_ptz_position_t *packet = (ttalink_ptz_position_t *)msgbuf;
    packet->update_time = update_time;
    packet->sensor_index = sensor_index;
    packet->gps_nstar = gps_nstar;
    packet->gps_dop = gps_dop;
    packet->rtk_nstar = rtk_nstar;
    packet->rtk_pos_type = rtk_pos_type;
    packet->rtk_head_type = rtk_head_type;
    packet->rtk_heading = rtk_heading;
    packet->longi = longi;
    packet->latit = latit;
    packet->altit = altit;
    packet->velN = velN;
    packet->velE = velE;
    packet->velD = velD;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_POSITION, (const char *)packet, TTALINK_MSG_ID_PTZ_POSITION_MIN_LEN, TTALINK_MSG_ID_PTZ_POSITION_LEN, TTALINK_MSG_ID_PTZ_POSITION_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_ptz_position_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t sensor_index, uint8_t gps_nstar, float gps_dop, uint8_t rtk_nstar, uint8_t rtk_pos_type, uint8_t rtk_head_type, float rtk_heading, int32_t longi, int32_t latit, float altit, float velN, float velE, float velD)
{
    _ttalink_ptz_position_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, sensor_index, gps_nstar, gps_dop, rtk_nstar, rtk_pos_type, rtk_head_type, rtk_heading, longi, latit, altit, velN, velE, velD, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_ptz_position_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t sensor_index, uint8_t gps_nstar, float gps_dop, uint8_t rtk_nstar, uint8_t rtk_pos_type, uint8_t rtk_head_type, float rtk_heading, int32_t longi, int32_t latit, float altit, float velN, float velE, float velD)
{
    _ttalink_ptz_position_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, sensor_index, gps_nstar, gps_dop, rtk_nstar, rtk_pos_type, rtk_head_type, rtk_heading, longi, latit, altit, velN, velE, velD, true);
}
#endif

#endif

// MESSAGE PTZ_POSITION UNPACKING


/**
 * @brief Get field update_time from ptz_position message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_ptz_position_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field sensor_index from ptz_position message
 *
 * @return  .
 */
static inline uint8_t ttalink_ptz_position_get_sensor_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field gps_nstar from ptz_position message
 *
 * @return  .
 */
static inline uint8_t ttalink_ptz_position_get_gps_nstar(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field gps_dop from ptz_position message
 *
 * @return  .
 */
static inline float ttalink_ptz_position_get_gps_dop(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  6);
}

/**
 * @brief Get field rtk_nstar from ptz_position message
 *
 * @return  .
 */
static inline uint8_t ttalink_ptz_position_get_rtk_nstar(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field rtk_pos_type from ptz_position message
 *
 * @return  .
 */
static inline uint8_t ttalink_ptz_position_get_rtk_pos_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field rtk_head_type from ptz_position message
 *
 * @return  .
 */
static inline uint8_t ttalink_ptz_position_get_rtk_head_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field rtk_heading from ptz_position message
 *
 * @return  .
 */
static inline float ttalink_ptz_position_get_rtk_heading(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  13);
}

/**
 * @brief Get field longi from ptz_position message
 *
 * @return  .
 */
static inline int32_t ttalink_ptz_position_get_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  17);
}

/**
 * @brief Get field latit from ptz_position message
 *
 * @return  .
 */
static inline int32_t ttalink_ptz_position_get_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  21);
}

/**
 * @brief Get field altit from ptz_position message
 *
 * @return  .
 */
static inline float ttalink_ptz_position_get_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  25);
}

/**
 * @brief Get field velN from ptz_position message
 *
 * @return  .
 */
static inline float ttalink_ptz_position_get_velN(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  29);
}

/**
 * @brief Get field velE from ptz_position message
 *
 * @return  .
 */
static inline float ttalink_ptz_position_get_velE(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  33);
}

/**
 * @brief Get field velD from ptz_position message
 *
 * @return  .
 */
static inline float ttalink_ptz_position_get_velD(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  37);
}

/**
 * @brief Decode a ptz_position message into a struct
 *
 * @param msg The message to decode
 * @param ptz_position C-struct to decode the message contents into
 */
static inline void ttalink_ptz_position_decode(const ttalink_message_t* msg, ttalink_ptz_position_t* ptz_position)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ptz_position->update_time = ttalink_ptz_position_get_update_time(msg);
    ptz_position->sensor_index = ttalink_ptz_position_get_sensor_index(msg);
    ptz_position->gps_nstar = ttalink_ptz_position_get_gps_nstar(msg);
    ptz_position->gps_dop = ttalink_ptz_position_get_gps_dop(msg);
    ptz_position->rtk_nstar = ttalink_ptz_position_get_rtk_nstar(msg);
    ptz_position->rtk_pos_type = ttalink_ptz_position_get_rtk_pos_type(msg);
    ptz_position->rtk_head_type = ttalink_ptz_position_get_rtk_head_type(msg);
    ptz_position->rtk_heading = ttalink_ptz_position_get_rtk_heading(msg);
    ptz_position->longi = ttalink_ptz_position_get_longi(msg);
    ptz_position->latit = ttalink_ptz_position_get_latit(msg);
    ptz_position->altit = ttalink_ptz_position_get_altit(msg);
    ptz_position->velN = ttalink_ptz_position_get_velN(msg);
    ptz_position->velE = ttalink_ptz_position_get_velE(msg);
    ptz_position->velD = ttalink_ptz_position_get_velD(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_PTZ_POSITION_LEN? msg->len : TTALINK_MSG_ID_PTZ_POSITION_LEN;
        memset(ptz_position, 0, TTALINK_MSG_ID_PTZ_POSITION_LEN);
    memcpy(ptz_position, _TTA_PAYLOAD(msg), len);
#endif
}
