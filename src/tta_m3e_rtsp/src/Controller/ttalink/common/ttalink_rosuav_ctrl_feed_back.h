#pragma once
// MESSAGE ROSUAV_CTRL_FEED_BACK PACKING

#define TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK 8201

TTAPACKED(
typedef struct __ttalink_rosuav_ctrl_feed_back_t {
 uint8_t navi_state; /*<  */
 uint8_t navi_mode; /*<  */
 uint8_t navi_atti_mode; /*<  */
 double latit; /*<  .*/
 double longi; /*<  .*/
 float altit; /*<  .*/
 float velN; /*<  .*/
 float velE; /*<  .*/
 float velD; /*<  .*/
 float atti_pitch; /*<  .*/
 float atti_roll; /*<  .*/
 float atti_yaw; /*<  .*/
 float gyro_pitch; /*<  .*/
 float gyro_roll; /*<  .*/
 float gyro_yaw; /*<  .*/
 float accN; /*<  .*/
 float accE; /*<  .*/
 float accD; /*<  .*/
 float quat[4]; /*<  四元数.*/
 float param[4]; /*<  .*/
}) ttalink_rosuav_ctrl_feed_back_t;

#define TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN 103
#define TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_MIN_LEN 103
#define TTALINK_MSG_ID_8201_LEN 103
#define TTALINK_MSG_ID_8201_MIN_LEN 103

#define TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_CRC 0
#define TTALINK_MSG_ID_8201_CRC 0

#define TTALINK_MSG_ROSUAV_CTRL_FEED_BACK_FIELD_QUAT_LEN 4
#define TTALINK_MSG_ROSUAV_CTRL_FEED_BACK_FIELD_PARAM_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_ROSUAV_CTRL_FEED_BACK { \
    8201, \
    "ROSUAV_CTRL_FEED_BACK", \
    20, \
    {  { "navi_state", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_rosuav_ctrl_feed_back_t, navi_state) }, \
         { "navi_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_rosuav_ctrl_feed_back_t, navi_mode) }, \
         { "navi_atti_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_rosuav_ctrl_feed_back_t, navi_atti_mode) }, \
         { "latit", NULL, TTALINK_TYPE_DOUBLE, 0, 3, offsetof(ttalink_rosuav_ctrl_feed_back_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_DOUBLE, 0, 11, offsetof(ttalink_rosuav_ctrl_feed_back_t, longi) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 19, offsetof(ttalink_rosuav_ctrl_feed_back_t, altit) }, \
         { "velN", NULL, TTALINK_TYPE_FLOAT, 0, 23, offsetof(ttalink_rosuav_ctrl_feed_back_t, velN) }, \
         { "velE", NULL, TTALINK_TYPE_FLOAT, 0, 27, offsetof(ttalink_rosuav_ctrl_feed_back_t, velE) }, \
         { "velD", NULL, TTALINK_TYPE_FLOAT, 0, 31, offsetof(ttalink_rosuav_ctrl_feed_back_t, velD) }, \
         { "atti_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 35, offsetof(ttalink_rosuav_ctrl_feed_back_t, atti_pitch) }, \
         { "atti_roll", NULL, TTALINK_TYPE_FLOAT, 0, 39, offsetof(ttalink_rosuav_ctrl_feed_back_t, atti_roll) }, \
         { "atti_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 43, offsetof(ttalink_rosuav_ctrl_feed_back_t, atti_yaw) }, \
         { "gyro_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 47, offsetof(ttalink_rosuav_ctrl_feed_back_t, gyro_pitch) }, \
         { "gyro_roll", NULL, TTALINK_TYPE_FLOAT, 0, 51, offsetof(ttalink_rosuav_ctrl_feed_back_t, gyro_roll) }, \
         { "gyro_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 55, offsetof(ttalink_rosuav_ctrl_feed_back_t, gyro_yaw) }, \
         { "accN", NULL, TTALINK_TYPE_FLOAT, 0, 59, offsetof(ttalink_rosuav_ctrl_feed_back_t, accN) }, \
         { "accE", NULL, TTALINK_TYPE_FLOAT, 0, 63, offsetof(ttalink_rosuav_ctrl_feed_back_t, accE) }, \
         { "accD", NULL, TTALINK_TYPE_FLOAT, 0, 67, offsetof(ttalink_rosuav_ctrl_feed_back_t, accD) }, \
         { "quat", NULL, TTALINK_TYPE_FLOAT, 4, 71, offsetof(ttalink_rosuav_ctrl_feed_back_t, quat) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 4, 87, offsetof(ttalink_rosuav_ctrl_feed_back_t, param) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_ROSUAV_CTRL_FEED_BACK { \
    "ROSUAV_CTRL_FEED_BACK", \
    20, \
    {  { "navi_state", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_rosuav_ctrl_feed_back_t, navi_state) }, \
         { "navi_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_rosuav_ctrl_feed_back_t, navi_mode) }, \
         { "navi_atti_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_rosuav_ctrl_feed_back_t, navi_atti_mode) }, \
         { "latit", NULL, TTALINK_TYPE_DOUBLE, 0, 3, offsetof(ttalink_rosuav_ctrl_feed_back_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_DOUBLE, 0, 11, offsetof(ttalink_rosuav_ctrl_feed_back_t, longi) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 19, offsetof(ttalink_rosuav_ctrl_feed_back_t, altit) }, \
         { "velN", NULL, TTALINK_TYPE_FLOAT, 0, 23, offsetof(ttalink_rosuav_ctrl_feed_back_t, velN) }, \
         { "velE", NULL, TTALINK_TYPE_FLOAT, 0, 27, offsetof(ttalink_rosuav_ctrl_feed_back_t, velE) }, \
         { "velD", NULL, TTALINK_TYPE_FLOAT, 0, 31, offsetof(ttalink_rosuav_ctrl_feed_back_t, velD) }, \
         { "atti_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 35, offsetof(ttalink_rosuav_ctrl_feed_back_t, atti_pitch) }, \
         { "atti_roll", NULL, TTALINK_TYPE_FLOAT, 0, 39, offsetof(ttalink_rosuav_ctrl_feed_back_t, atti_roll) }, \
         { "atti_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 43, offsetof(ttalink_rosuav_ctrl_feed_back_t, atti_yaw) }, \
         { "gyro_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 47, offsetof(ttalink_rosuav_ctrl_feed_back_t, gyro_pitch) }, \
         { "gyro_roll", NULL, TTALINK_TYPE_FLOAT, 0, 51, offsetof(ttalink_rosuav_ctrl_feed_back_t, gyro_roll) }, \
         { "gyro_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 55, offsetof(ttalink_rosuav_ctrl_feed_back_t, gyro_yaw) }, \
         { "accN", NULL, TTALINK_TYPE_FLOAT, 0, 59, offsetof(ttalink_rosuav_ctrl_feed_back_t, accN) }, \
         { "accE", NULL, TTALINK_TYPE_FLOAT, 0, 63, offsetof(ttalink_rosuav_ctrl_feed_back_t, accE) }, \
         { "accD", NULL, TTALINK_TYPE_FLOAT, 0, 67, offsetof(ttalink_rosuav_ctrl_feed_back_t, accD) }, \
         { "quat", NULL, TTALINK_TYPE_FLOAT, 4, 71, offsetof(ttalink_rosuav_ctrl_feed_back_t, quat) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 4, 87, offsetof(ttalink_rosuav_ctrl_feed_back_t, param) }, \
         } \
}
#endif


static inline uint16_t _ttalink_rosuav_ctrl_feed_back_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t navi_state, uint8_t navi_mode, uint8_t navi_atti_mode, double latit, double longi, float altit, float velN, float velE, float velD, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float accN, float accE, float accD, const float *quat, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN];
    _tta_put_uint8_t(buf, 0, navi_state);
    _tta_put_uint8_t(buf, 1, navi_mode);
    _tta_put_uint8_t(buf, 2, navi_atti_mode);
    _tta_put_double(buf, 3, latit);
    _tta_put_double(buf, 11, longi);
    _tta_put_float(buf, 19, altit);
    _tta_put_float(buf, 23, velN);
    _tta_put_float(buf, 27, velE);
    _tta_put_float(buf, 31, velD);
    _tta_put_float(buf, 35, atti_pitch);
    _tta_put_float(buf, 39, atti_roll);
    _tta_put_float(buf, 43, atti_yaw);
    _tta_put_float(buf, 47, gyro_pitch);
    _tta_put_float(buf, 51, gyro_roll);
    _tta_put_float(buf, 55, gyro_yaw);
    _tta_put_float(buf, 59, accN);
    _tta_put_float(buf, 63, accE);
    _tta_put_float(buf, 67, accD);
    _tta_put_float_array(buf, 71, quat, 4);
    _tta_put_float_array(buf, 87, param, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN);
#else
    ttalink_rosuav_ctrl_feed_back_t packet;
    packet.navi_state = navi_state;
    packet.navi_mode = navi_mode;
    packet.navi_atti_mode = navi_atti_mode;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;
    packet.velN = velN;
    packet.velE = velE;
    packet.velD = velD;
    packet.atti_pitch = atti_pitch;
    packet.atti_roll = atti_roll;
    packet.atti_yaw = atti_yaw;
    packet.gyro_pitch = gyro_pitch;
    packet.gyro_roll = gyro_roll;
    packet.gyro_yaw = gyro_yaw;
    packet.accN = accN;
    packet.accE = accE;
    packet.accD = accD;
    tta_array_memcpy(packet.quat, quat, sizeof(float)*4);
    tta_array_memcpy(packet.param, param, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_CRC, nocrc);
}

/**
 * @brief Pack a rosuav_ctrl_feed_back message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param navi_state  
 * @param navi_mode  
 * @param navi_atti_mode  
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @param atti_pitch  .
 * @param atti_roll  .
 * @param atti_yaw  .
 * @param gyro_pitch  .
 * @param gyro_roll  .
 * @param gyro_yaw  .
 * @param accN  .
 * @param accE  .
 * @param accD  .
 * @param quat  四元数.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_ctrl_feed_back_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t navi_state, uint8_t navi_mode, uint8_t navi_atti_mode, double latit, double longi, float altit, float velN, float velE, float velD, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float accN, float accE, float accD, const float *quat, const float *param)
{
    return _ttalink_rosuav_ctrl_feed_back_pack(dst_addr, src_addr, msg,  navi_state, navi_mode, navi_atti_mode, latit, longi, altit, velN, velE, velD, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, accN, accE, accD, quat, param, false);
}

/**
 * @brief Pack a rosuav_ctrl_feed_back message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param navi_state  
 * @param navi_mode  
 * @param navi_atti_mode  
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @param atti_pitch  .
 * @param atti_roll  .
 * @param atti_yaw  .
 * @param gyro_pitch  .
 * @param gyro_roll  .
 * @param gyro_yaw  .
 * @param accN  .
 * @param accE  .
 * @param accD  .
 * @param quat  四元数.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_ctrl_feed_back_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t navi_state, uint8_t navi_mode, uint8_t navi_atti_mode, double latit, double longi, float altit, float velN, float velE, float velD, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float accN, float accE, float accD, const float *quat, const float *param)
{
    return _ttalink_rosuav_ctrl_feed_back_pack(dst_addr, src_addr, msg,  navi_state, navi_mode, navi_atti_mode, latit, longi, altit, velN, velE, velD, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, accN, accE, accD, quat, param, true);
}


static inline uint16_t _ttalink_rosuav_ctrl_feed_back_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t navi_state,uint8_t navi_mode,uint8_t navi_atti_mode,double latit,double longi,float altit,float velN,float velE,float velD,float atti_pitch,float atti_roll,float atti_yaw,float gyro_pitch,float gyro_roll,float gyro_yaw,float accN,float accE,float accD,const float *quat,const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN];
    _tta_put_uint8_t(buf, 0, navi_state);
    _tta_put_uint8_t(buf, 1, navi_mode);
    _tta_put_uint8_t(buf, 2, navi_atti_mode);
    _tta_put_double(buf, 3, latit);
    _tta_put_double(buf, 11, longi);
    _tta_put_float(buf, 19, altit);
    _tta_put_float(buf, 23, velN);
    _tta_put_float(buf, 27, velE);
    _tta_put_float(buf, 31, velD);
    _tta_put_float(buf, 35, atti_pitch);
    _tta_put_float(buf, 39, atti_roll);
    _tta_put_float(buf, 43, atti_yaw);
    _tta_put_float(buf, 47, gyro_pitch);
    _tta_put_float(buf, 51, gyro_roll);
    _tta_put_float(buf, 55, gyro_yaw);
    _tta_put_float(buf, 59, accN);
    _tta_put_float(buf, 63, accE);
    _tta_put_float(buf, 67, accD);
    _tta_put_float_array(buf, 71, quat, 4);
    _tta_put_float_array(buf, 87, param, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN);
#else
    ttalink_rosuav_ctrl_feed_back_t packet;
    packet.navi_state = navi_state;
    packet.navi_mode = navi_mode;
    packet.navi_atti_mode = navi_atti_mode;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;
    packet.velN = velN;
    packet.velE = velE;
    packet.velD = velD;
    packet.atti_pitch = atti_pitch;
    packet.atti_roll = atti_roll;
    packet.atti_yaw = atti_yaw;
    packet.gyro_pitch = gyro_pitch;
    packet.gyro_roll = gyro_roll;
    packet.gyro_yaw = gyro_yaw;
    packet.accN = accN;
    packet.accE = accE;
    packet.accD = accD;
    tta_array_memcpy(packet.quat, quat, sizeof(float)*4);
    tta_array_memcpy(packet.param, param, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_CRC, nocrc);
}

/**
 * @brief Pack a rosuav_ctrl_feed_back message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param navi_state  
 * @param navi_mode  
 * @param navi_atti_mode  
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @param atti_pitch  .
 * @param atti_roll  .
 * @param atti_yaw  .
 * @param gyro_pitch  .
 * @param gyro_roll  .
 * @param gyro_yaw  .
 * @param accN  .
 * @param accE  .
 * @param accD  .
 * @param quat  四元数.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_ctrl_feed_back_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t navi_state,uint8_t navi_mode,uint8_t navi_atti_mode,double latit,double longi,float altit,float velN,float velE,float velD,float atti_pitch,float atti_roll,float atti_yaw,float gyro_pitch,float gyro_roll,float gyro_yaw,float accN,float accE,float accD,const float *quat,const float *param)
{
    return _ttalink_rosuav_ctrl_feed_back_pack_chan(dst_addr, src_addr, chan, msg,  navi_state, navi_mode, navi_atti_mode, latit, longi, altit, velN, velE, velD, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, accN, accE, accD, quat, param, false);
}

/**
 * @brief Pack a rosuav_ctrl_feed_back message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param navi_state  
 * @param navi_mode  
 * @param navi_atti_mode  
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @param atti_pitch  .
 * @param atti_roll  .
 * @param atti_yaw  .
 * @param gyro_pitch  .
 * @param gyro_roll  .
 * @param gyro_yaw  .
 * @param accN  .
 * @param accE  .
 * @param accD  .
 * @param quat  四元数.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_ctrl_feed_back_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t navi_state,uint8_t navi_mode,uint8_t navi_atti_mode,double latit,double longi,float altit,float velN,float velE,float velD,float atti_pitch,float atti_roll,float atti_yaw,float gyro_pitch,float gyro_roll,float gyro_yaw,float accN,float accE,float accD,const float *quat,const float *param)
{
    return _ttalink_rosuav_ctrl_feed_back_pack_chan(dst_addr, src_addr, chan, msg,  navi_state, navi_mode, navi_atti_mode, latit, longi, altit, velN, velE, velD, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, accN, accE, accD, quat, param, true);
}


static inline uint16_t _ttalink_rosuav_ctrl_feed_back_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_ctrl_feed_back_t* rosuav_ctrl_feed_back, bool nocrc)
{
    if(nocrc){
        return ttalink_rosuav_ctrl_feed_back_pack_nocrc(dst_addr, src_addr, msg, rosuav_ctrl_feed_back->navi_state, rosuav_ctrl_feed_back->navi_mode, rosuav_ctrl_feed_back->navi_atti_mode, rosuav_ctrl_feed_back->latit, rosuav_ctrl_feed_back->longi, rosuav_ctrl_feed_back->altit, rosuav_ctrl_feed_back->velN, rosuav_ctrl_feed_back->velE, rosuav_ctrl_feed_back->velD, rosuav_ctrl_feed_back->atti_pitch, rosuav_ctrl_feed_back->atti_roll, rosuav_ctrl_feed_back->atti_yaw, rosuav_ctrl_feed_back->gyro_pitch, rosuav_ctrl_feed_back->gyro_roll, rosuav_ctrl_feed_back->gyro_yaw, rosuav_ctrl_feed_back->accN, rosuav_ctrl_feed_back->accE, rosuav_ctrl_feed_back->accD, rosuav_ctrl_feed_back->quat, rosuav_ctrl_feed_back->param);
    }else{
        return ttalink_rosuav_ctrl_feed_back_pack(dst_addr, src_addr, msg, rosuav_ctrl_feed_back->navi_state, rosuav_ctrl_feed_back->navi_mode, rosuav_ctrl_feed_back->navi_atti_mode, rosuav_ctrl_feed_back->latit, rosuav_ctrl_feed_back->longi, rosuav_ctrl_feed_back->altit, rosuav_ctrl_feed_back->velN, rosuav_ctrl_feed_back->velE, rosuav_ctrl_feed_back->velD, rosuav_ctrl_feed_back->atti_pitch, rosuav_ctrl_feed_back->atti_roll, rosuav_ctrl_feed_back->atti_yaw, rosuav_ctrl_feed_back->gyro_pitch, rosuav_ctrl_feed_back->gyro_roll, rosuav_ctrl_feed_back->gyro_yaw, rosuav_ctrl_feed_back->accN, rosuav_ctrl_feed_back->accE, rosuav_ctrl_feed_back->accD, rosuav_ctrl_feed_back->quat, rosuav_ctrl_feed_back->param);
    }
    
}

/**
 * @brief Encode a rosuav_ctrl_feed_back struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_ctrl_feed_back C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_ctrl_feed_back_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_ctrl_feed_back_t* rosuav_ctrl_feed_back)
{
    return _ttalink_rosuav_ctrl_feed_back_encode(dst_addr, src_addr, msg, rosuav_ctrl_feed_back, false);
}

/**
 * @brief Encode a rosuav_ctrl_feed_back struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_ctrl_feed_back C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_ctrl_feed_back_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_ctrl_feed_back_t* rosuav_ctrl_feed_back)
{
    return _ttalink_rosuav_ctrl_feed_back_encode(dst_addr, src_addr, msg, rosuav_ctrl_feed_back, true);
}


static inline uint16_t _ttalink_rosuav_ctrl_feed_back_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_ctrl_feed_back_t* rosuav_ctrl_feed_back, bool nocrc)
{
    if(nocrc){
        return ttalink_rosuav_ctrl_feed_back_pack_chan_nocrc(dst_addr, src_addr, chan, msg, rosuav_ctrl_feed_back->navi_state, rosuav_ctrl_feed_back->navi_mode, rosuav_ctrl_feed_back->navi_atti_mode, rosuav_ctrl_feed_back->latit, rosuav_ctrl_feed_back->longi, rosuav_ctrl_feed_back->altit, rosuav_ctrl_feed_back->velN, rosuav_ctrl_feed_back->velE, rosuav_ctrl_feed_back->velD, rosuav_ctrl_feed_back->atti_pitch, rosuav_ctrl_feed_back->atti_roll, rosuav_ctrl_feed_back->atti_yaw, rosuav_ctrl_feed_back->gyro_pitch, rosuav_ctrl_feed_back->gyro_roll, rosuav_ctrl_feed_back->gyro_yaw, rosuav_ctrl_feed_back->accN, rosuav_ctrl_feed_back->accE, rosuav_ctrl_feed_back->accD, rosuav_ctrl_feed_back->quat, rosuav_ctrl_feed_back->param);
    }else{
        return ttalink_rosuav_ctrl_feed_back_pack_chan(dst_addr, src_addr, chan, msg, rosuav_ctrl_feed_back->navi_state, rosuav_ctrl_feed_back->navi_mode, rosuav_ctrl_feed_back->navi_atti_mode, rosuav_ctrl_feed_back->latit, rosuav_ctrl_feed_back->longi, rosuav_ctrl_feed_back->altit, rosuav_ctrl_feed_back->velN, rosuav_ctrl_feed_back->velE, rosuav_ctrl_feed_back->velD, rosuav_ctrl_feed_back->atti_pitch, rosuav_ctrl_feed_back->atti_roll, rosuav_ctrl_feed_back->atti_yaw, rosuav_ctrl_feed_back->gyro_pitch, rosuav_ctrl_feed_back->gyro_roll, rosuav_ctrl_feed_back->gyro_yaw, rosuav_ctrl_feed_back->accN, rosuav_ctrl_feed_back->accE, rosuav_ctrl_feed_back->accD, rosuav_ctrl_feed_back->quat, rosuav_ctrl_feed_back->param);
    }
}

/**
 * @brief Encode a rosuav_ctrl_feed_back struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_ctrl_feed_back C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_ctrl_feed_back_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_ctrl_feed_back_t* rosuav_ctrl_feed_back)
{
    return _ttalink_rosuav_ctrl_feed_back_encode_chan(dst_addr, src_addr, chan, msg, rosuav_ctrl_feed_back, false);
}

/**
 * @brief Encode a rosuav_ctrl_feed_back struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_ctrl_feed_back C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_ctrl_feed_back_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_ctrl_feed_back_t* rosuav_ctrl_feed_back)
{
    return _ttalink_rosuav_ctrl_feed_back_encode_chan(dst_addr, src_addr, chan, msg, rosuav_ctrl_feed_back, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_rosuav_ctrl_feed_back_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t navi_state, uint8_t navi_mode, uint8_t navi_atti_mode, double latit, double longi, float altit, float velN, float velE, float velD, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float accN, float accE, float accD, const float *quat, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN];
    _tta_put_uint8_t(buf, 0, navi_state);
    _tta_put_uint8_t(buf, 1, navi_mode);
    _tta_put_uint8_t(buf, 2, navi_atti_mode);
    _tta_put_double(buf, 3, latit);
    _tta_put_double(buf, 11, longi);
    _tta_put_float(buf, 19, altit);
    _tta_put_float(buf, 23, velN);
    _tta_put_float(buf, 27, velE);
    _tta_put_float(buf, 31, velD);
    _tta_put_float(buf, 35, atti_pitch);
    _tta_put_float(buf, 39, atti_roll);
    _tta_put_float(buf, 43, atti_yaw);
    _tta_put_float(buf, 47, gyro_pitch);
    _tta_put_float(buf, 51, gyro_roll);
    _tta_put_float(buf, 55, gyro_yaw);
    _tta_put_float(buf, 59, accN);
    _tta_put_float(buf, 63, accE);
    _tta_put_float(buf, 67, accD);
    _tta_put_float_array(buf, 71, quat, 4);
    _tta_put_float_array(buf, 87, param, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK, buf, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_CRC, nocrc);
#else
    ttalink_rosuav_ctrl_feed_back_t packet;
    packet.navi_state = navi_state;
    packet.navi_mode = navi_mode;
    packet.navi_atti_mode = navi_atti_mode;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;
    packet.velN = velN;
    packet.velE = velE;
    packet.velD = velD;
    packet.atti_pitch = atti_pitch;
    packet.atti_roll = atti_roll;
    packet.atti_yaw = atti_yaw;
    packet.gyro_pitch = gyro_pitch;
    packet.gyro_roll = gyro_roll;
    packet.gyro_yaw = gyro_yaw;
    packet.accN = accN;
    packet.accE = accE;
    packet.accD = accD;
    tta_array_memcpy(packet.quat, quat, sizeof(float)*4);
    tta_array_memcpy(packet.param, param, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK, (const char *)&packet, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a rosuav_ctrl_feed_back message
 * @param chan TTAlink channel to send the message
 *
 * @param navi_state  
 * @param navi_mode  
 * @param navi_atti_mode  
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @param atti_pitch  .
 * @param atti_roll  .
 * @param atti_yaw  .
 * @param gyro_pitch  .
 * @param gyro_roll  .
 * @param gyro_yaw  .
 * @param accN  .
 * @param accE  .
 * @param accD  .
 * @param quat  四元数.
 * @param param  .
 */
static inline void ttalink_rosuav_ctrl_feed_back_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t navi_state, uint8_t navi_mode, uint8_t navi_atti_mode, double latit, double longi, float altit, float velN, float velE, float velD, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float accN, float accE, float accD, const float *quat, const float *param)
{
    _ttalink_rosuav_ctrl_feed_back_send(dst_addr, src_addr, chan, navi_state, navi_mode, navi_atti_mode, latit, longi, altit, velN, velE, velD, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, accN, accE, accD, quat, param, false);
}

/**
 * @brief Send a rosuav_ctrl_feed_back message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param navi_state  
 * @param navi_mode  
 * @param navi_atti_mode  
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @param atti_pitch  .
 * @param atti_roll  .
 * @param atti_yaw  .
 * @param gyro_pitch  .
 * @param gyro_roll  .
 * @param gyro_yaw  .
 * @param accN  .
 * @param accE  .
 * @param accD  .
 * @param quat  四元数.
 * @param param  .
 */
static inline void ttalink_rosuav_ctrl_feed_back_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t navi_state, uint8_t navi_mode, uint8_t navi_atti_mode, double latit, double longi, float altit, float velN, float velE, float velD, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float accN, float accE, float accD, const float *quat, const float *param)
{
    _ttalink_rosuav_ctrl_feed_back_send(dst_addr, src_addr, chan, navi_state, navi_mode, navi_atti_mode, latit, longi, altit, velN, velE, velD, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, accN, accE, accD, quat, param, true);
}


static inline void _ttalink_rosuav_ctrl_feed_back_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_ctrl_feed_back_t* rosuav_ctrl_feed_back, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_rosuav_ctrl_feed_back_send_nocrc(dst_addr, src_addr, chan, rosuav_ctrl_feed_back->navi_state, rosuav_ctrl_feed_back->navi_mode, rosuav_ctrl_feed_back->navi_atti_mode, rosuav_ctrl_feed_back->latit, rosuav_ctrl_feed_back->longi, rosuav_ctrl_feed_back->altit, rosuav_ctrl_feed_back->velN, rosuav_ctrl_feed_back->velE, rosuav_ctrl_feed_back->velD, rosuav_ctrl_feed_back->atti_pitch, rosuav_ctrl_feed_back->atti_roll, rosuav_ctrl_feed_back->atti_yaw, rosuav_ctrl_feed_back->gyro_pitch, rosuav_ctrl_feed_back->gyro_roll, rosuav_ctrl_feed_back->gyro_yaw, rosuav_ctrl_feed_back->accN, rosuav_ctrl_feed_back->accE, rosuav_ctrl_feed_back->accD, rosuav_ctrl_feed_back->quat, rosuav_ctrl_feed_back->param);
    }else{
        ttalink_rosuav_ctrl_feed_back_send(dst_addr, src_addr, chan, rosuav_ctrl_feed_back->navi_state, rosuav_ctrl_feed_back->navi_mode, rosuav_ctrl_feed_back->navi_atti_mode, rosuav_ctrl_feed_back->latit, rosuav_ctrl_feed_back->longi, rosuav_ctrl_feed_back->altit, rosuav_ctrl_feed_back->velN, rosuav_ctrl_feed_back->velE, rosuav_ctrl_feed_back->velD, rosuav_ctrl_feed_back->atti_pitch, rosuav_ctrl_feed_back->atti_roll, rosuav_ctrl_feed_back->atti_yaw, rosuav_ctrl_feed_back->gyro_pitch, rosuav_ctrl_feed_back->gyro_roll, rosuav_ctrl_feed_back->gyro_yaw, rosuav_ctrl_feed_back->accN, rosuav_ctrl_feed_back->accE, rosuav_ctrl_feed_back->accD, rosuav_ctrl_feed_back->quat, rosuav_ctrl_feed_back->param);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK, (const char *)rosuav_ctrl_feed_back, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a rosuav_ctrl_feed_back message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rosuav_ctrl_feed_back_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_ctrl_feed_back_t* rosuav_ctrl_feed_back)
{
    _ttalink_rosuav_ctrl_feed_back_send_struct(dst_addr, src_addr, chan, rosuav_ctrl_feed_back, false);
}

/**
 * @brief Send a rosuav_ctrl_feed_back message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rosuav_ctrl_feed_back_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_ctrl_feed_back_t* rosuav_ctrl_feed_back)
{
    _ttalink_rosuav_ctrl_feed_back_send_struct(dst_addr, src_addr, chan, rosuav_ctrl_feed_back, true);
}

#if TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_rosuav_ctrl_feed_back_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t navi_state, uint8_t navi_mode, uint8_t navi_atti_mode, double latit, double longi, float altit, float velN, float velE, float velD, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float accN, float accE, float accD, const float *quat, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, navi_state);
    _tta_put_uint8_t(buf, 1, navi_mode);
    _tta_put_uint8_t(buf, 2, navi_atti_mode);
    _tta_put_double(buf, 3, latit);
    _tta_put_double(buf, 11, longi);
    _tta_put_float(buf, 19, altit);
    _tta_put_float(buf, 23, velN);
    _tta_put_float(buf, 27, velE);
    _tta_put_float(buf, 31, velD);
    _tta_put_float(buf, 35, atti_pitch);
    _tta_put_float(buf, 39, atti_roll);
    _tta_put_float(buf, 43, atti_yaw);
    _tta_put_float(buf, 47, gyro_pitch);
    _tta_put_float(buf, 51, gyro_roll);
    _tta_put_float(buf, 55, gyro_yaw);
    _tta_put_float(buf, 59, accN);
    _tta_put_float(buf, 63, accE);
    _tta_put_float(buf, 67, accD);
    _tta_put_float_array(buf, 71, quat, 4);
    _tta_put_float_array(buf, 87, param, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK, buf, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_CRC, nocrc);
#else
    ttalink_rosuav_ctrl_feed_back_t *packet = (ttalink_rosuav_ctrl_feed_back_t *)msgbuf;
    packet->navi_state = navi_state;
    packet->navi_mode = navi_mode;
    packet->navi_atti_mode = navi_atti_mode;
    packet->latit = latit;
    packet->longi = longi;
    packet->altit = altit;
    packet->velN = velN;
    packet->velE = velE;
    packet->velD = velD;
    packet->atti_pitch = atti_pitch;
    packet->atti_roll = atti_roll;
    packet->atti_yaw = atti_yaw;
    packet->gyro_pitch = gyro_pitch;
    packet->gyro_roll = gyro_roll;
    packet->gyro_yaw = gyro_yaw;
    packet->accN = accN;
    packet->accE = accE;
    packet->accD = accD;
    tta_array_memcpy(packet->quat, quat, sizeof(float)*4);
    tta_array_memcpy(packet->param, param, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK, (const char *)packet, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_rosuav_ctrl_feed_back_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t navi_state, uint8_t navi_mode, uint8_t navi_atti_mode, double latit, double longi, float altit, float velN, float velE, float velD, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float accN, float accE, float accD, const float *quat, const float *param)
{
    _ttalink_rosuav_ctrl_feed_back_send_buf(dst_addr, src_addr, msgbuf, chan, navi_state, navi_mode, navi_atti_mode, latit, longi, altit, velN, velE, velD, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, accN, accE, accD, quat, param, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_rosuav_ctrl_feed_back_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t navi_state, uint8_t navi_mode, uint8_t navi_atti_mode, double latit, double longi, float altit, float velN, float velE, float velD, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float accN, float accE, float accD, const float *quat, const float *param)
{
    _ttalink_rosuav_ctrl_feed_back_send_buf(dst_addr, src_addr, msgbuf, chan, navi_state, navi_mode, navi_atti_mode, latit, longi, altit, velN, velE, velD, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, accN, accE, accD, quat, param, true);
}
#endif

#endif

// MESSAGE ROSUAV_CTRL_FEED_BACK UNPACKING


/**
 * @brief Get field navi_state from rosuav_ctrl_feed_back message
 *
 * @return  
 */
static inline uint8_t ttalink_rosuav_ctrl_feed_back_get_navi_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field navi_mode from rosuav_ctrl_feed_back message
 *
 * @return  
 */
static inline uint8_t ttalink_rosuav_ctrl_feed_back_get_navi_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field navi_atti_mode from rosuav_ctrl_feed_back message
 *
 * @return  
 */
static inline uint8_t ttalink_rosuav_ctrl_feed_back_get_navi_atti_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field latit from rosuav_ctrl_feed_back message
 *
 * @return  .
 */
static inline double ttalink_rosuav_ctrl_feed_back_get_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  3);
}

/**
 * @brief Get field longi from rosuav_ctrl_feed_back message
 *
 * @return  .
 */
static inline double ttalink_rosuav_ctrl_feed_back_get_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  11);
}

/**
 * @brief Get field altit from rosuav_ctrl_feed_back message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_feed_back_get_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  19);
}

/**
 * @brief Get field velN from rosuav_ctrl_feed_back message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_feed_back_get_velN(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  23);
}

/**
 * @brief Get field velE from rosuav_ctrl_feed_back message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_feed_back_get_velE(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  27);
}

/**
 * @brief Get field velD from rosuav_ctrl_feed_back message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_feed_back_get_velD(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  31);
}

/**
 * @brief Get field atti_pitch from rosuav_ctrl_feed_back message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_feed_back_get_atti_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  35);
}

/**
 * @brief Get field atti_roll from rosuav_ctrl_feed_back message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_feed_back_get_atti_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  39);
}

/**
 * @brief Get field atti_yaw from rosuav_ctrl_feed_back message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_feed_back_get_atti_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  43);
}

/**
 * @brief Get field gyro_pitch from rosuav_ctrl_feed_back message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_feed_back_get_gyro_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  47);
}

/**
 * @brief Get field gyro_roll from rosuav_ctrl_feed_back message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_feed_back_get_gyro_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  51);
}

/**
 * @brief Get field gyro_yaw from rosuav_ctrl_feed_back message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_feed_back_get_gyro_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  55);
}

/**
 * @brief Get field accN from rosuav_ctrl_feed_back message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_feed_back_get_accN(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  59);
}

/**
 * @brief Get field accE from rosuav_ctrl_feed_back message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_feed_back_get_accE(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  63);
}

/**
 * @brief Get field accD from rosuav_ctrl_feed_back message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_feed_back_get_accD(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  67);
}

/**
 * @brief Get field quat from rosuav_ctrl_feed_back message
 *
 * @return  四元数.
 */
static inline uint16_t ttalink_rosuav_ctrl_feed_back_get_quat(const ttalink_message_t* msg, float *quat)
{
    return _TTA_RETURN_float_array(msg, quat, 4,  71);
}

/**
 * @brief Get field param from rosuav_ctrl_feed_back message
 *
 * @return  .
 */
static inline uint16_t ttalink_rosuav_ctrl_feed_back_get_param(const ttalink_message_t* msg, float *param)
{
    return _TTA_RETURN_float_array(msg, param, 4,  87);
}

/**
 * @brief Decode a rosuav_ctrl_feed_back message into a struct
 *
 * @param msg The message to decode
 * @param rosuav_ctrl_feed_back C-struct to decode the message contents into
 */
static inline void ttalink_rosuav_ctrl_feed_back_decode(const ttalink_message_t* msg, ttalink_rosuav_ctrl_feed_back_t* rosuav_ctrl_feed_back)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    rosuav_ctrl_feed_back->navi_state = ttalink_rosuav_ctrl_feed_back_get_navi_state(msg);
    rosuav_ctrl_feed_back->navi_mode = ttalink_rosuav_ctrl_feed_back_get_navi_mode(msg);
    rosuav_ctrl_feed_back->navi_atti_mode = ttalink_rosuav_ctrl_feed_back_get_navi_atti_mode(msg);
    rosuav_ctrl_feed_back->latit = ttalink_rosuav_ctrl_feed_back_get_latit(msg);
    rosuav_ctrl_feed_back->longi = ttalink_rosuav_ctrl_feed_back_get_longi(msg);
    rosuav_ctrl_feed_back->altit = ttalink_rosuav_ctrl_feed_back_get_altit(msg);
    rosuav_ctrl_feed_back->velN = ttalink_rosuav_ctrl_feed_back_get_velN(msg);
    rosuav_ctrl_feed_back->velE = ttalink_rosuav_ctrl_feed_back_get_velE(msg);
    rosuav_ctrl_feed_back->velD = ttalink_rosuav_ctrl_feed_back_get_velD(msg);
    rosuav_ctrl_feed_back->atti_pitch = ttalink_rosuav_ctrl_feed_back_get_atti_pitch(msg);
    rosuav_ctrl_feed_back->atti_roll = ttalink_rosuav_ctrl_feed_back_get_atti_roll(msg);
    rosuav_ctrl_feed_back->atti_yaw = ttalink_rosuav_ctrl_feed_back_get_atti_yaw(msg);
    rosuav_ctrl_feed_back->gyro_pitch = ttalink_rosuav_ctrl_feed_back_get_gyro_pitch(msg);
    rosuav_ctrl_feed_back->gyro_roll = ttalink_rosuav_ctrl_feed_back_get_gyro_roll(msg);
    rosuav_ctrl_feed_back->gyro_yaw = ttalink_rosuav_ctrl_feed_back_get_gyro_yaw(msg);
    rosuav_ctrl_feed_back->accN = ttalink_rosuav_ctrl_feed_back_get_accN(msg);
    rosuav_ctrl_feed_back->accE = ttalink_rosuav_ctrl_feed_back_get_accE(msg);
    rosuav_ctrl_feed_back->accD = ttalink_rosuav_ctrl_feed_back_get_accD(msg);
    ttalink_rosuav_ctrl_feed_back_get_quat(msg, rosuav_ctrl_feed_back->quat);
    ttalink_rosuav_ctrl_feed_back_get_param(msg, rosuav_ctrl_feed_back->param);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN? msg->len : TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN;
        memset(rosuav_ctrl_feed_back, 0, TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK_LEN);
    memcpy(rosuav_ctrl_feed_back, _TTA_PAYLOAD(msg), len);
#endif
}
