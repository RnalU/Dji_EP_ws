#pragma once
// MESSAGE RTK_SENSORS_AUGMENT_DATA PACKING

#define TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA 2177

TTAPACKED(
typedef struct __ttalink_rtk_sensors_augment_data_t {
 double psr_latitude; /*<  .*/
 double psr_longitude; /*<  .*/
 float psr_altitude; /*<  .*/
 uint8_t SVs; /*<  跟踪的卫星数.*/
 uint8_t solnSVs; /*<  使用的卫星数.*/
 int8_t stn_id[4]; /*<  基站ID.*/
 float diff_age; /*<  差分龄期.*/
 float sol_age; /*<  解龄期.*/
 float hor_spd; /*<  地速.*/
 float trk_gnd; /*< 地速与北向夹角.*/
 float vert_spd; /*< 垂向速度.*/
 uint16_t vel_std_g; /*<  速度水平误差.*/
 uint16_t vel_std_h; /*<  速度高程误差.*/
 uint32_t rtk_version; /*<  .*/
 float param2; /*<  .*/
 float param3; /*<  .*/
 float param4; /*<  .*/
}) ttalink_rtk_sensors_augment_data_t;

#define TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN 66
#define TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_MIN_LEN 66
#define TTALINK_MSG_ID_2177_LEN 66
#define TTALINK_MSG_ID_2177_MIN_LEN 66

#define TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_CRC 91
#define TTALINK_MSG_ID_2177_CRC 91

#define TTALINK_MSG_RTK_SENSORS_AUGMENT_DATA_FIELD_STN_ID_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_RTK_SENSORS_AUGMENT_DATA { \
    2177, \
    "RTK_SENSORS_AUGMENT_DATA", \
    17, \
    {  { "psr_latitude", NULL, TTALINK_TYPE_DOUBLE, 0, 0, offsetof(ttalink_rtk_sensors_augment_data_t, psr_latitude) }, \
         { "psr_longitude", NULL, TTALINK_TYPE_DOUBLE, 0, 8, offsetof(ttalink_rtk_sensors_augment_data_t, psr_longitude) }, \
         { "psr_altitude", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_rtk_sensors_augment_data_t, psr_altitude) }, \
         { "SVs", NULL, TTALINK_TYPE_UINT8_T, 0, 20, offsetof(ttalink_rtk_sensors_augment_data_t, SVs) }, \
         { "solnSVs", NULL, TTALINK_TYPE_UINT8_T, 0, 21, offsetof(ttalink_rtk_sensors_augment_data_t, solnSVs) }, \
         { "stn_id", NULL, TTALINK_TYPE_INT8_T, 4, 22, offsetof(ttalink_rtk_sensors_augment_data_t, stn_id) }, \
         { "diff_age", NULL, TTALINK_TYPE_FLOAT, 0, 26, offsetof(ttalink_rtk_sensors_augment_data_t, diff_age) }, \
         { "sol_age", NULL, TTALINK_TYPE_FLOAT, 0, 30, offsetof(ttalink_rtk_sensors_augment_data_t, sol_age) }, \
         { "hor_spd", NULL, TTALINK_TYPE_FLOAT, 0, 34, offsetof(ttalink_rtk_sensors_augment_data_t, hor_spd) }, \
         { "trk_gnd", NULL, TTALINK_TYPE_FLOAT, 0, 38, offsetof(ttalink_rtk_sensors_augment_data_t, trk_gnd) }, \
         { "vert_spd", NULL, TTALINK_TYPE_FLOAT, 0, 42, offsetof(ttalink_rtk_sensors_augment_data_t, vert_spd) }, \
         { "vel_std_g", NULL, TTALINK_TYPE_UINT16_T, 0, 46, offsetof(ttalink_rtk_sensors_augment_data_t, vel_std_g) }, \
         { "vel_std_h", NULL, TTALINK_TYPE_UINT16_T, 0, 48, offsetof(ttalink_rtk_sensors_augment_data_t, vel_std_h) }, \
         { "rtk_version", NULL, TTALINK_TYPE_UINT32_T, 0, 50, offsetof(ttalink_rtk_sensors_augment_data_t, rtk_version) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 54, offsetof(ttalink_rtk_sensors_augment_data_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 58, offsetof(ttalink_rtk_sensors_augment_data_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 62, offsetof(ttalink_rtk_sensors_augment_data_t, param4) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_RTK_SENSORS_AUGMENT_DATA { \
    "RTK_SENSORS_AUGMENT_DATA", \
    17, \
    {  { "psr_latitude", NULL, TTALINK_TYPE_DOUBLE, 0, 0, offsetof(ttalink_rtk_sensors_augment_data_t, psr_latitude) }, \
         { "psr_longitude", NULL, TTALINK_TYPE_DOUBLE, 0, 8, offsetof(ttalink_rtk_sensors_augment_data_t, psr_longitude) }, \
         { "psr_altitude", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_rtk_sensors_augment_data_t, psr_altitude) }, \
         { "SVs", NULL, TTALINK_TYPE_UINT8_T, 0, 20, offsetof(ttalink_rtk_sensors_augment_data_t, SVs) }, \
         { "solnSVs", NULL, TTALINK_TYPE_UINT8_T, 0, 21, offsetof(ttalink_rtk_sensors_augment_data_t, solnSVs) }, \
         { "stn_id", NULL, TTALINK_TYPE_INT8_T, 4, 22, offsetof(ttalink_rtk_sensors_augment_data_t, stn_id) }, \
         { "diff_age", NULL, TTALINK_TYPE_FLOAT, 0, 26, offsetof(ttalink_rtk_sensors_augment_data_t, diff_age) }, \
         { "sol_age", NULL, TTALINK_TYPE_FLOAT, 0, 30, offsetof(ttalink_rtk_sensors_augment_data_t, sol_age) }, \
         { "hor_spd", NULL, TTALINK_TYPE_FLOAT, 0, 34, offsetof(ttalink_rtk_sensors_augment_data_t, hor_spd) }, \
         { "trk_gnd", NULL, TTALINK_TYPE_FLOAT, 0, 38, offsetof(ttalink_rtk_sensors_augment_data_t, trk_gnd) }, \
         { "vert_spd", NULL, TTALINK_TYPE_FLOAT, 0, 42, offsetof(ttalink_rtk_sensors_augment_data_t, vert_spd) }, \
         { "vel_std_g", NULL, TTALINK_TYPE_UINT16_T, 0, 46, offsetof(ttalink_rtk_sensors_augment_data_t, vel_std_g) }, \
         { "vel_std_h", NULL, TTALINK_TYPE_UINT16_T, 0, 48, offsetof(ttalink_rtk_sensors_augment_data_t, vel_std_h) }, \
         { "rtk_version", NULL, TTALINK_TYPE_UINT32_T, 0, 50, offsetof(ttalink_rtk_sensors_augment_data_t, rtk_version) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 54, offsetof(ttalink_rtk_sensors_augment_data_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 58, offsetof(ttalink_rtk_sensors_augment_data_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 62, offsetof(ttalink_rtk_sensors_augment_data_t, param4) }, \
         } \
}
#endif


static inline uint16_t _ttalink_rtk_sensors_augment_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               double psr_latitude, double psr_longitude, float psr_altitude, uint8_t SVs, uint8_t solnSVs, const int8_t *stn_id, float diff_age, float sol_age, float hor_spd, float trk_gnd, float vert_spd, uint16_t vel_std_g, uint16_t vel_std_h, uint32_t rtk_version, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN];
    _tta_put_double(buf, 0, psr_latitude);
    _tta_put_double(buf, 8, psr_longitude);
    _tta_put_float(buf, 16, psr_altitude);
    _tta_put_uint8_t(buf, 20, SVs);
    _tta_put_uint8_t(buf, 21, solnSVs);
    _tta_put_float(buf, 26, diff_age);
    _tta_put_float(buf, 30, sol_age);
    _tta_put_float(buf, 34, hor_spd);
    _tta_put_float(buf, 38, trk_gnd);
    _tta_put_float(buf, 42, vert_spd);
    _tta_put_uint16_t(buf, 46, vel_std_g);
    _tta_put_uint16_t(buf, 48, vel_std_h);
    _tta_put_uint32_t(buf, 50, rtk_version);
    _tta_put_float(buf, 54, param2);
    _tta_put_float(buf, 58, param3);
    _tta_put_float(buf, 62, param4);
    _tta_put_int8_t_array(buf, 22, stn_id, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN);
#else
    ttalink_rtk_sensors_augment_data_t packet;
    packet.psr_latitude = psr_latitude;
    packet.psr_longitude = psr_longitude;
    packet.psr_altitude = psr_altitude;
    packet.SVs = SVs;
    packet.solnSVs = solnSVs;
    packet.diff_age = diff_age;
    packet.sol_age = sol_age;
    packet.hor_spd = hor_spd;
    packet.trk_gnd = trk_gnd;
    packet.vert_spd = vert_spd;
    packet.vel_std_g = vel_std_g;
    packet.vel_std_h = vel_std_h;
    packet.rtk_version = rtk_version;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    tta_array_memcpy(packet.stn_id, stn_id, sizeof(int8_t)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_CRC, nocrc);
}

/**
 * @brief Pack a rtk_sensors_augment_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param psr_latitude  .
 * @param psr_longitude  .
 * @param psr_altitude  .
 * @param SVs  跟踪的卫星数.
 * @param solnSVs  使用的卫星数.
 * @param stn_id  基站ID.
 * @param diff_age  差分龄期.
 * @param sol_age  解龄期.
 * @param hor_spd  地速.
 * @param trk_gnd 地速与北向夹角.
 * @param vert_spd 垂向速度.
 * @param vel_std_g  速度水平误差.
 * @param vel_std_h  速度高程误差.
 * @param rtk_version  .
 * @param param2  .
 * @param param3  .
 * @param param4  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rtk_sensors_augment_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               double psr_latitude, double psr_longitude, float psr_altitude, uint8_t SVs, uint8_t solnSVs, const int8_t *stn_id, float diff_age, float sol_age, float hor_spd, float trk_gnd, float vert_spd, uint16_t vel_std_g, uint16_t vel_std_h, uint32_t rtk_version, float param2, float param3, float param4)
{
    return _ttalink_rtk_sensors_augment_data_pack(dst_addr, src_addr, msg,  psr_latitude, psr_longitude, psr_altitude, SVs, solnSVs, stn_id, diff_age, sol_age, hor_spd, trk_gnd, vert_spd, vel_std_g, vel_std_h, rtk_version, param2, param3, param4, false);
}

/**
 * @brief Pack a rtk_sensors_augment_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param psr_latitude  .
 * @param psr_longitude  .
 * @param psr_altitude  .
 * @param SVs  跟踪的卫星数.
 * @param solnSVs  使用的卫星数.
 * @param stn_id  基站ID.
 * @param diff_age  差分龄期.
 * @param sol_age  解龄期.
 * @param hor_spd  地速.
 * @param trk_gnd 地速与北向夹角.
 * @param vert_spd 垂向速度.
 * @param vel_std_g  速度水平误差.
 * @param vel_std_h  速度高程误差.
 * @param rtk_version  .
 * @param param2  .
 * @param param3  .
 * @param param4  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rtk_sensors_augment_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               double psr_latitude, double psr_longitude, float psr_altitude, uint8_t SVs, uint8_t solnSVs, const int8_t *stn_id, float diff_age, float sol_age, float hor_spd, float trk_gnd, float vert_spd, uint16_t vel_std_g, uint16_t vel_std_h, uint32_t rtk_version, float param2, float param3, float param4)
{
    return _ttalink_rtk_sensors_augment_data_pack(dst_addr, src_addr, msg,  psr_latitude, psr_longitude, psr_altitude, SVs, solnSVs, stn_id, diff_age, sol_age, hor_spd, trk_gnd, vert_spd, vel_std_g, vel_std_h, rtk_version, param2, param3, param4, true);
}


static inline uint16_t _ttalink_rtk_sensors_augment_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   double psr_latitude,double psr_longitude,float psr_altitude,uint8_t SVs,uint8_t solnSVs,const int8_t *stn_id,float diff_age,float sol_age,float hor_spd,float trk_gnd,float vert_spd,uint16_t vel_std_g,uint16_t vel_std_h,uint32_t rtk_version,float param2,float param3,float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN];
    _tta_put_double(buf, 0, psr_latitude);
    _tta_put_double(buf, 8, psr_longitude);
    _tta_put_float(buf, 16, psr_altitude);
    _tta_put_uint8_t(buf, 20, SVs);
    _tta_put_uint8_t(buf, 21, solnSVs);
    _tta_put_float(buf, 26, diff_age);
    _tta_put_float(buf, 30, sol_age);
    _tta_put_float(buf, 34, hor_spd);
    _tta_put_float(buf, 38, trk_gnd);
    _tta_put_float(buf, 42, vert_spd);
    _tta_put_uint16_t(buf, 46, vel_std_g);
    _tta_put_uint16_t(buf, 48, vel_std_h);
    _tta_put_uint32_t(buf, 50, rtk_version);
    _tta_put_float(buf, 54, param2);
    _tta_put_float(buf, 58, param3);
    _tta_put_float(buf, 62, param4);
    _tta_put_int8_t_array(buf, 22, stn_id, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN);
#else
    ttalink_rtk_sensors_augment_data_t packet;
    packet.psr_latitude = psr_latitude;
    packet.psr_longitude = psr_longitude;
    packet.psr_altitude = psr_altitude;
    packet.SVs = SVs;
    packet.solnSVs = solnSVs;
    packet.diff_age = diff_age;
    packet.sol_age = sol_age;
    packet.hor_spd = hor_spd;
    packet.trk_gnd = trk_gnd;
    packet.vert_spd = vert_spd;
    packet.vel_std_g = vel_std_g;
    packet.vel_std_h = vel_std_h;
    packet.rtk_version = rtk_version;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    tta_array_memcpy(packet.stn_id, stn_id, sizeof(int8_t)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_CRC, nocrc);
}

/**
 * @brief Pack a rtk_sensors_augment_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param psr_latitude  .
 * @param psr_longitude  .
 * @param psr_altitude  .
 * @param SVs  跟踪的卫星数.
 * @param solnSVs  使用的卫星数.
 * @param stn_id  基站ID.
 * @param diff_age  差分龄期.
 * @param sol_age  解龄期.
 * @param hor_spd  地速.
 * @param trk_gnd 地速与北向夹角.
 * @param vert_spd 垂向速度.
 * @param vel_std_g  速度水平误差.
 * @param vel_std_h  速度高程误差.
 * @param rtk_version  .
 * @param param2  .
 * @param param3  .
 * @param param4  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rtk_sensors_augment_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   double psr_latitude,double psr_longitude,float psr_altitude,uint8_t SVs,uint8_t solnSVs,const int8_t *stn_id,float diff_age,float sol_age,float hor_spd,float trk_gnd,float vert_spd,uint16_t vel_std_g,uint16_t vel_std_h,uint32_t rtk_version,float param2,float param3,float param4)
{
    return _ttalink_rtk_sensors_augment_data_pack_chan(dst_addr, src_addr, chan, msg,  psr_latitude, psr_longitude, psr_altitude, SVs, solnSVs, stn_id, diff_age, sol_age, hor_spd, trk_gnd, vert_spd, vel_std_g, vel_std_h, rtk_version, param2, param3, param4, false);
}

/**
 * @brief Pack a rtk_sensors_augment_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param psr_latitude  .
 * @param psr_longitude  .
 * @param psr_altitude  .
 * @param SVs  跟踪的卫星数.
 * @param solnSVs  使用的卫星数.
 * @param stn_id  基站ID.
 * @param diff_age  差分龄期.
 * @param sol_age  解龄期.
 * @param hor_spd  地速.
 * @param trk_gnd 地速与北向夹角.
 * @param vert_spd 垂向速度.
 * @param vel_std_g  速度水平误差.
 * @param vel_std_h  速度高程误差.
 * @param rtk_version  .
 * @param param2  .
 * @param param3  .
 * @param param4  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rtk_sensors_augment_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   double psr_latitude,double psr_longitude,float psr_altitude,uint8_t SVs,uint8_t solnSVs,const int8_t *stn_id,float diff_age,float sol_age,float hor_spd,float trk_gnd,float vert_spd,uint16_t vel_std_g,uint16_t vel_std_h,uint32_t rtk_version,float param2,float param3,float param4)
{
    return _ttalink_rtk_sensors_augment_data_pack_chan(dst_addr, src_addr, chan, msg,  psr_latitude, psr_longitude, psr_altitude, SVs, solnSVs, stn_id, diff_age, sol_age, hor_spd, trk_gnd, vert_spd, vel_std_g, vel_std_h, rtk_version, param2, param3, param4, true);
}


static inline uint16_t _ttalink_rtk_sensors_augment_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rtk_sensors_augment_data_t* rtk_sensors_augment_data, bool nocrc)
{
    if(nocrc){
        return ttalink_rtk_sensors_augment_data_pack_nocrc(dst_addr, src_addr, msg, rtk_sensors_augment_data->psr_latitude, rtk_sensors_augment_data->psr_longitude, rtk_sensors_augment_data->psr_altitude, rtk_sensors_augment_data->SVs, rtk_sensors_augment_data->solnSVs, rtk_sensors_augment_data->stn_id, rtk_sensors_augment_data->diff_age, rtk_sensors_augment_data->sol_age, rtk_sensors_augment_data->hor_spd, rtk_sensors_augment_data->trk_gnd, rtk_sensors_augment_data->vert_spd, rtk_sensors_augment_data->vel_std_g, rtk_sensors_augment_data->vel_std_h, rtk_sensors_augment_data->rtk_version, rtk_sensors_augment_data->param2, rtk_sensors_augment_data->param3, rtk_sensors_augment_data->param4);
    }else{
        return ttalink_rtk_sensors_augment_data_pack(dst_addr, src_addr, msg, rtk_sensors_augment_data->psr_latitude, rtk_sensors_augment_data->psr_longitude, rtk_sensors_augment_data->psr_altitude, rtk_sensors_augment_data->SVs, rtk_sensors_augment_data->solnSVs, rtk_sensors_augment_data->stn_id, rtk_sensors_augment_data->diff_age, rtk_sensors_augment_data->sol_age, rtk_sensors_augment_data->hor_spd, rtk_sensors_augment_data->trk_gnd, rtk_sensors_augment_data->vert_spd, rtk_sensors_augment_data->vel_std_g, rtk_sensors_augment_data->vel_std_h, rtk_sensors_augment_data->rtk_version, rtk_sensors_augment_data->param2, rtk_sensors_augment_data->param3, rtk_sensors_augment_data->param4);
    }
    
}

/**
 * @brief Encode a rtk_sensors_augment_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rtk_sensors_augment_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_rtk_sensors_augment_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rtk_sensors_augment_data_t* rtk_sensors_augment_data)
{
    return _ttalink_rtk_sensors_augment_data_encode(dst_addr, src_addr, msg, rtk_sensors_augment_data, false);
}

/**
 * @brief Encode a rtk_sensors_augment_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rtk_sensors_augment_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_rtk_sensors_augment_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rtk_sensors_augment_data_t* rtk_sensors_augment_data)
{
    return _ttalink_rtk_sensors_augment_data_encode(dst_addr, src_addr, msg, rtk_sensors_augment_data, true);
}


static inline uint16_t _ttalink_rtk_sensors_augment_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rtk_sensors_augment_data_t* rtk_sensors_augment_data, bool nocrc)
{
    if(nocrc){
        return ttalink_rtk_sensors_augment_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, rtk_sensors_augment_data->psr_latitude, rtk_sensors_augment_data->psr_longitude, rtk_sensors_augment_data->psr_altitude, rtk_sensors_augment_data->SVs, rtk_sensors_augment_data->solnSVs, rtk_sensors_augment_data->stn_id, rtk_sensors_augment_data->diff_age, rtk_sensors_augment_data->sol_age, rtk_sensors_augment_data->hor_spd, rtk_sensors_augment_data->trk_gnd, rtk_sensors_augment_data->vert_spd, rtk_sensors_augment_data->vel_std_g, rtk_sensors_augment_data->vel_std_h, rtk_sensors_augment_data->rtk_version, rtk_sensors_augment_data->param2, rtk_sensors_augment_data->param3, rtk_sensors_augment_data->param4);
    }else{
        return ttalink_rtk_sensors_augment_data_pack_chan(dst_addr, src_addr, chan, msg, rtk_sensors_augment_data->psr_latitude, rtk_sensors_augment_data->psr_longitude, rtk_sensors_augment_data->psr_altitude, rtk_sensors_augment_data->SVs, rtk_sensors_augment_data->solnSVs, rtk_sensors_augment_data->stn_id, rtk_sensors_augment_data->diff_age, rtk_sensors_augment_data->sol_age, rtk_sensors_augment_data->hor_spd, rtk_sensors_augment_data->trk_gnd, rtk_sensors_augment_data->vert_spd, rtk_sensors_augment_data->vel_std_g, rtk_sensors_augment_data->vel_std_h, rtk_sensors_augment_data->rtk_version, rtk_sensors_augment_data->param2, rtk_sensors_augment_data->param3, rtk_sensors_augment_data->param4);
    }
}

/**
 * @brief Encode a rtk_sensors_augment_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rtk_sensors_augment_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_rtk_sensors_augment_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rtk_sensors_augment_data_t* rtk_sensors_augment_data)
{
    return _ttalink_rtk_sensors_augment_data_encode_chan(dst_addr, src_addr, chan, msg, rtk_sensors_augment_data, false);
}

/**
 * @brief Encode a rtk_sensors_augment_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rtk_sensors_augment_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_rtk_sensors_augment_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rtk_sensors_augment_data_t* rtk_sensors_augment_data)
{
    return _ttalink_rtk_sensors_augment_data_encode_chan(dst_addr, src_addr, chan, msg, rtk_sensors_augment_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_rtk_sensors_augment_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, double psr_latitude, double psr_longitude, float psr_altitude, uint8_t SVs, uint8_t solnSVs, const int8_t *stn_id, float diff_age, float sol_age, float hor_spd, float trk_gnd, float vert_spd, uint16_t vel_std_g, uint16_t vel_std_h, uint32_t rtk_version, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN];
    _tta_put_double(buf, 0, psr_latitude);
    _tta_put_double(buf, 8, psr_longitude);
    _tta_put_float(buf, 16, psr_altitude);
    _tta_put_uint8_t(buf, 20, SVs);
    _tta_put_uint8_t(buf, 21, solnSVs);
    _tta_put_float(buf, 26, diff_age);
    _tta_put_float(buf, 30, sol_age);
    _tta_put_float(buf, 34, hor_spd);
    _tta_put_float(buf, 38, trk_gnd);
    _tta_put_float(buf, 42, vert_spd);
    _tta_put_uint16_t(buf, 46, vel_std_g);
    _tta_put_uint16_t(buf, 48, vel_std_h);
    _tta_put_uint32_t(buf, 50, rtk_version);
    _tta_put_float(buf, 54, param2);
    _tta_put_float(buf, 58, param3);
    _tta_put_float(buf, 62, param4);
    _tta_put_int8_t_array(buf, 22, stn_id, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA, buf, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_CRC, nocrc);
#else
    ttalink_rtk_sensors_augment_data_t packet;
    packet.psr_latitude = psr_latitude;
    packet.psr_longitude = psr_longitude;
    packet.psr_altitude = psr_altitude;
    packet.SVs = SVs;
    packet.solnSVs = solnSVs;
    packet.diff_age = diff_age;
    packet.sol_age = sol_age;
    packet.hor_spd = hor_spd;
    packet.trk_gnd = trk_gnd;
    packet.vert_spd = vert_spd;
    packet.vel_std_g = vel_std_g;
    packet.vel_std_h = vel_std_h;
    packet.rtk_version = rtk_version;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    tta_array_memcpy(packet.stn_id, stn_id, sizeof(int8_t)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA, (const char *)&packet, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a rtk_sensors_augment_data message
 * @param chan TTAlink channel to send the message
 *
 * @param psr_latitude  .
 * @param psr_longitude  .
 * @param psr_altitude  .
 * @param SVs  跟踪的卫星数.
 * @param solnSVs  使用的卫星数.
 * @param stn_id  基站ID.
 * @param diff_age  差分龄期.
 * @param sol_age  解龄期.
 * @param hor_spd  地速.
 * @param trk_gnd 地速与北向夹角.
 * @param vert_spd 垂向速度.
 * @param vel_std_g  速度水平误差.
 * @param vel_std_h  速度高程误差.
 * @param rtk_version  .
 * @param param2  .
 * @param param3  .
 * @param param4  .
 */
static inline void ttalink_rtk_sensors_augment_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, double psr_latitude, double psr_longitude, float psr_altitude, uint8_t SVs, uint8_t solnSVs, const int8_t *stn_id, float diff_age, float sol_age, float hor_spd, float trk_gnd, float vert_spd, uint16_t vel_std_g, uint16_t vel_std_h, uint32_t rtk_version, float param2, float param3, float param4)
{
    _ttalink_rtk_sensors_augment_data_send(dst_addr, src_addr, chan, psr_latitude, psr_longitude, psr_altitude, SVs, solnSVs, stn_id, diff_age, sol_age, hor_spd, trk_gnd, vert_spd, vel_std_g, vel_std_h, rtk_version, param2, param3, param4, false);
}

/**
 * @brief Send a rtk_sensors_augment_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param psr_latitude  .
 * @param psr_longitude  .
 * @param psr_altitude  .
 * @param SVs  跟踪的卫星数.
 * @param solnSVs  使用的卫星数.
 * @param stn_id  基站ID.
 * @param diff_age  差分龄期.
 * @param sol_age  解龄期.
 * @param hor_spd  地速.
 * @param trk_gnd 地速与北向夹角.
 * @param vert_spd 垂向速度.
 * @param vel_std_g  速度水平误差.
 * @param vel_std_h  速度高程误差.
 * @param rtk_version  .
 * @param param2  .
 * @param param3  .
 * @param param4  .
 */
static inline void ttalink_rtk_sensors_augment_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, double psr_latitude, double psr_longitude, float psr_altitude, uint8_t SVs, uint8_t solnSVs, const int8_t *stn_id, float diff_age, float sol_age, float hor_spd, float trk_gnd, float vert_spd, uint16_t vel_std_g, uint16_t vel_std_h, uint32_t rtk_version, float param2, float param3, float param4)
{
    _ttalink_rtk_sensors_augment_data_send(dst_addr, src_addr, chan, psr_latitude, psr_longitude, psr_altitude, SVs, solnSVs, stn_id, diff_age, sol_age, hor_spd, trk_gnd, vert_spd, vel_std_g, vel_std_h, rtk_version, param2, param3, param4, true);
}


static inline void _ttalink_rtk_sensors_augment_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rtk_sensors_augment_data_t* rtk_sensors_augment_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_rtk_sensors_augment_data_send_nocrc(dst_addr, src_addr, chan, rtk_sensors_augment_data->psr_latitude, rtk_sensors_augment_data->psr_longitude, rtk_sensors_augment_data->psr_altitude, rtk_sensors_augment_data->SVs, rtk_sensors_augment_data->solnSVs, rtk_sensors_augment_data->stn_id, rtk_sensors_augment_data->diff_age, rtk_sensors_augment_data->sol_age, rtk_sensors_augment_data->hor_spd, rtk_sensors_augment_data->trk_gnd, rtk_sensors_augment_data->vert_spd, rtk_sensors_augment_data->vel_std_g, rtk_sensors_augment_data->vel_std_h, rtk_sensors_augment_data->rtk_version, rtk_sensors_augment_data->param2, rtk_sensors_augment_data->param3, rtk_sensors_augment_data->param4);
    }else{
        ttalink_rtk_sensors_augment_data_send(dst_addr, src_addr, chan, rtk_sensors_augment_data->psr_latitude, rtk_sensors_augment_data->psr_longitude, rtk_sensors_augment_data->psr_altitude, rtk_sensors_augment_data->SVs, rtk_sensors_augment_data->solnSVs, rtk_sensors_augment_data->stn_id, rtk_sensors_augment_data->diff_age, rtk_sensors_augment_data->sol_age, rtk_sensors_augment_data->hor_spd, rtk_sensors_augment_data->trk_gnd, rtk_sensors_augment_data->vert_spd, rtk_sensors_augment_data->vel_std_g, rtk_sensors_augment_data->vel_std_h, rtk_sensors_augment_data->rtk_version, rtk_sensors_augment_data->param2, rtk_sensors_augment_data->param3, rtk_sensors_augment_data->param4);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA, (const char *)rtk_sensors_augment_data, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a rtk_sensors_augment_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rtk_sensors_augment_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rtk_sensors_augment_data_t* rtk_sensors_augment_data)
{
    _ttalink_rtk_sensors_augment_data_send_struct(dst_addr, src_addr, chan, rtk_sensors_augment_data, false);
}

/**
 * @brief Send a rtk_sensors_augment_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rtk_sensors_augment_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rtk_sensors_augment_data_t* rtk_sensors_augment_data)
{
    _ttalink_rtk_sensors_augment_data_send_struct(dst_addr, src_addr, chan, rtk_sensors_augment_data, true);
}

#if TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_rtk_sensors_augment_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  double psr_latitude, double psr_longitude, float psr_altitude, uint8_t SVs, uint8_t solnSVs, const int8_t *stn_id, float diff_age, float sol_age, float hor_spd, float trk_gnd, float vert_spd, uint16_t vel_std_g, uint16_t vel_std_h, uint32_t rtk_version, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_double(buf, 0, psr_latitude);
    _tta_put_double(buf, 8, psr_longitude);
    _tta_put_float(buf, 16, psr_altitude);
    _tta_put_uint8_t(buf, 20, SVs);
    _tta_put_uint8_t(buf, 21, solnSVs);
    _tta_put_float(buf, 26, diff_age);
    _tta_put_float(buf, 30, sol_age);
    _tta_put_float(buf, 34, hor_spd);
    _tta_put_float(buf, 38, trk_gnd);
    _tta_put_float(buf, 42, vert_spd);
    _tta_put_uint16_t(buf, 46, vel_std_g);
    _tta_put_uint16_t(buf, 48, vel_std_h);
    _tta_put_uint32_t(buf, 50, rtk_version);
    _tta_put_float(buf, 54, param2);
    _tta_put_float(buf, 58, param3);
    _tta_put_float(buf, 62, param4);
    _tta_put_int8_t_array(buf, 22, stn_id, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA, buf, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_CRC, nocrc);
#else
    ttalink_rtk_sensors_augment_data_t *packet = (ttalink_rtk_sensors_augment_data_t *)msgbuf;
    packet->psr_latitude = psr_latitude;
    packet->psr_longitude = psr_longitude;
    packet->psr_altitude = psr_altitude;
    packet->SVs = SVs;
    packet->solnSVs = solnSVs;
    packet->diff_age = diff_age;
    packet->sol_age = sol_age;
    packet->hor_spd = hor_spd;
    packet->trk_gnd = trk_gnd;
    packet->vert_spd = vert_spd;
    packet->vel_std_g = vel_std_g;
    packet->vel_std_h = vel_std_h;
    packet->rtk_version = rtk_version;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;
    tta_array_memcpy(packet->stn_id, stn_id, sizeof(int8_t)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA, (const char *)packet, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_rtk_sensors_augment_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  double psr_latitude, double psr_longitude, float psr_altitude, uint8_t SVs, uint8_t solnSVs, const int8_t *stn_id, float diff_age, float sol_age, float hor_spd, float trk_gnd, float vert_spd, uint16_t vel_std_g, uint16_t vel_std_h, uint32_t rtk_version, float param2, float param3, float param4)
{
    _ttalink_rtk_sensors_augment_data_send_buf(dst_addr, src_addr, msgbuf, chan, psr_latitude, psr_longitude, psr_altitude, SVs, solnSVs, stn_id, diff_age, sol_age, hor_spd, trk_gnd, vert_spd, vel_std_g, vel_std_h, rtk_version, param2, param3, param4, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_rtk_sensors_augment_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  double psr_latitude, double psr_longitude, float psr_altitude, uint8_t SVs, uint8_t solnSVs, const int8_t *stn_id, float diff_age, float sol_age, float hor_spd, float trk_gnd, float vert_spd, uint16_t vel_std_g, uint16_t vel_std_h, uint32_t rtk_version, float param2, float param3, float param4)
{
    _ttalink_rtk_sensors_augment_data_send_buf(dst_addr, src_addr, msgbuf, chan, psr_latitude, psr_longitude, psr_altitude, SVs, solnSVs, stn_id, diff_age, sol_age, hor_spd, trk_gnd, vert_spd, vel_std_g, vel_std_h, rtk_version, param2, param3, param4, true);
}
#endif

#endif

// MESSAGE RTK_SENSORS_AUGMENT_DATA UNPACKING


/**
 * @brief Get field psr_latitude from rtk_sensors_augment_data message
 *
 * @return  .
 */
static inline double ttalink_rtk_sensors_augment_data_get_psr_latitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  0);
}

/**
 * @brief Get field psr_longitude from rtk_sensors_augment_data message
 *
 * @return  .
 */
static inline double ttalink_rtk_sensors_augment_data_get_psr_longitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  8);
}

/**
 * @brief Get field psr_altitude from rtk_sensors_augment_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_augment_data_get_psr_altitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field SVs from rtk_sensors_augment_data message
 *
 * @return  跟踪的卫星数.
 */
static inline uint8_t ttalink_rtk_sensors_augment_data_get_SVs(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field solnSVs from rtk_sensors_augment_data message
 *
 * @return  使用的卫星数.
 */
static inline uint8_t ttalink_rtk_sensors_augment_data_get_solnSVs(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field stn_id from rtk_sensors_augment_data message
 *
 * @return  基站ID.
 */
static inline uint16_t ttalink_rtk_sensors_augment_data_get_stn_id(const ttalink_message_t* msg, int8_t *stn_id)
{
    return _TTA_RETURN_int8_t_array(msg, stn_id, 4,  22);
}

/**
 * @brief Get field diff_age from rtk_sensors_augment_data message
 *
 * @return  差分龄期.
 */
static inline float ttalink_rtk_sensors_augment_data_get_diff_age(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  26);
}

/**
 * @brief Get field sol_age from rtk_sensors_augment_data message
 *
 * @return  解龄期.
 */
static inline float ttalink_rtk_sensors_augment_data_get_sol_age(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  30);
}

/**
 * @brief Get field hor_spd from rtk_sensors_augment_data message
 *
 * @return  地速.
 */
static inline float ttalink_rtk_sensors_augment_data_get_hor_spd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  34);
}

/**
 * @brief Get field trk_gnd from rtk_sensors_augment_data message
 *
 * @return 地速与北向夹角.
 */
static inline float ttalink_rtk_sensors_augment_data_get_trk_gnd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  38);
}

/**
 * @brief Get field vert_spd from rtk_sensors_augment_data message
 *
 * @return 垂向速度.
 */
static inline float ttalink_rtk_sensors_augment_data_get_vert_spd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  42);
}

/**
 * @brief Get field vel_std_g from rtk_sensors_augment_data message
 *
 * @return  速度水平误差.
 */
static inline uint16_t ttalink_rtk_sensors_augment_data_get_vel_std_g(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  46);
}

/**
 * @brief Get field vel_std_h from rtk_sensors_augment_data message
 *
 * @return  速度高程误差.
 */
static inline uint16_t ttalink_rtk_sensors_augment_data_get_vel_std_h(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  48);
}

/**
 * @brief Get field rtk_version from rtk_sensors_augment_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_rtk_sensors_augment_data_get_rtk_version(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  50);
}

/**
 * @brief Get field param2 from rtk_sensors_augment_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_augment_data_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  54);
}

/**
 * @brief Get field param3 from rtk_sensors_augment_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_augment_data_get_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  58);
}

/**
 * @brief Get field param4 from rtk_sensors_augment_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_augment_data_get_param4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  62);
}

/**
 * @brief Decode a rtk_sensors_augment_data message into a struct
 *
 * @param msg The message to decode
 * @param rtk_sensors_augment_data C-struct to decode the message contents into
 */
static inline void ttalink_rtk_sensors_augment_data_decode(const ttalink_message_t* msg, ttalink_rtk_sensors_augment_data_t* rtk_sensors_augment_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    rtk_sensors_augment_data->psr_latitude = ttalink_rtk_sensors_augment_data_get_psr_latitude(msg);
    rtk_sensors_augment_data->psr_longitude = ttalink_rtk_sensors_augment_data_get_psr_longitude(msg);
    rtk_sensors_augment_data->psr_altitude = ttalink_rtk_sensors_augment_data_get_psr_altitude(msg);
    rtk_sensors_augment_data->SVs = ttalink_rtk_sensors_augment_data_get_SVs(msg);
    rtk_sensors_augment_data->solnSVs = ttalink_rtk_sensors_augment_data_get_solnSVs(msg);
    ttalink_rtk_sensors_augment_data_get_stn_id(msg, rtk_sensors_augment_data->stn_id);
    rtk_sensors_augment_data->diff_age = ttalink_rtk_sensors_augment_data_get_diff_age(msg);
    rtk_sensors_augment_data->sol_age = ttalink_rtk_sensors_augment_data_get_sol_age(msg);
    rtk_sensors_augment_data->hor_spd = ttalink_rtk_sensors_augment_data_get_hor_spd(msg);
    rtk_sensors_augment_data->trk_gnd = ttalink_rtk_sensors_augment_data_get_trk_gnd(msg);
    rtk_sensors_augment_data->vert_spd = ttalink_rtk_sensors_augment_data_get_vert_spd(msg);
    rtk_sensors_augment_data->vel_std_g = ttalink_rtk_sensors_augment_data_get_vel_std_g(msg);
    rtk_sensors_augment_data->vel_std_h = ttalink_rtk_sensors_augment_data_get_vel_std_h(msg);
    rtk_sensors_augment_data->rtk_version = ttalink_rtk_sensors_augment_data_get_rtk_version(msg);
    rtk_sensors_augment_data->param2 = ttalink_rtk_sensors_augment_data_get_param2(msg);
    rtk_sensors_augment_data->param3 = ttalink_rtk_sensors_augment_data_get_param3(msg);
    rtk_sensors_augment_data->param4 = ttalink_rtk_sensors_augment_data_get_param4(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN? msg->len : TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN;
        memset(rtk_sensors_augment_data, 0, TTALINK_MSG_ID_RTK_SENSORS_AUGMENT_DATA_LEN);
    memcpy(rtk_sensors_augment_data, _TTA_PAYLOAD(msg), len);
#endif
}
