#pragma once
// MESSAGE UPDATE_VR_TO_FC PACKING

#define TTALINK_MSG_ID_UPDATE_VR_TO_FC 2228

TTAPACKED(
typedef struct __ttalink_update_vr_to_fc_t {
 double latit; /*<  （WGS84）.*/
 double longi; /*<  （WGS84）.*/
 float altit; /*<  （m）.*/
 float vel_n; /*<  （NED坐标系）（m/s）.*/
 float vel_e; /*<  （NED坐标系）（m/s）.*/
 float vel_d; /*<  （NED坐标系）（m/s）.*/
 float atti_pitch; /*<  （符合右手定责，前右下坐标系）（°）.*/
 float atti_roll; /*<  （符合右手定责，前右下坐标系）（°）.*/
 float atti_yaw; /*<  （符合右手定责，前右下坐标系）（°）.*/
 float param1; /*<  预留.*/
 float param2; /*<  预留.*/
 float param3; /*<  预留.*/
 float param4; /*<  预留.*/
}) ttalink_update_vr_to_fc_t;

#define TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN 60
#define TTALINK_MSG_ID_UPDATE_VR_TO_FC_MIN_LEN 60
#define TTALINK_MSG_ID_2228_LEN 60
#define TTALINK_MSG_ID_2228_MIN_LEN 60

#define TTALINK_MSG_ID_UPDATE_VR_TO_FC_CRC 142
#define TTALINK_MSG_ID_2228_CRC 142



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_UPDATE_VR_TO_FC { \
    2228, \
    "UPDATE_VR_TO_FC", \
    13, \
    {  { "latit", NULL, TTALINK_TYPE_DOUBLE, 0, 0, offsetof(ttalink_update_vr_to_fc_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_DOUBLE, 0, 8, offsetof(ttalink_update_vr_to_fc_t, longi) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_update_vr_to_fc_t, altit) }, \
         { "vel_n", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_update_vr_to_fc_t, vel_n) }, \
         { "vel_e", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_update_vr_to_fc_t, vel_e) }, \
         { "vel_d", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_update_vr_to_fc_t, vel_d) }, \
         { "atti_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_update_vr_to_fc_t, atti_pitch) }, \
         { "atti_roll", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_update_vr_to_fc_t, atti_roll) }, \
         { "atti_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_update_vr_to_fc_t, atti_yaw) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_update_vr_to_fc_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 48, offsetof(ttalink_update_vr_to_fc_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 52, offsetof(ttalink_update_vr_to_fc_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 56, offsetof(ttalink_update_vr_to_fc_t, param4) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_UPDATE_VR_TO_FC { \
    "UPDATE_VR_TO_FC", \
    13, \
    {  { "latit", NULL, TTALINK_TYPE_DOUBLE, 0, 0, offsetof(ttalink_update_vr_to_fc_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_DOUBLE, 0, 8, offsetof(ttalink_update_vr_to_fc_t, longi) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_update_vr_to_fc_t, altit) }, \
         { "vel_n", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_update_vr_to_fc_t, vel_n) }, \
         { "vel_e", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_update_vr_to_fc_t, vel_e) }, \
         { "vel_d", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_update_vr_to_fc_t, vel_d) }, \
         { "atti_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_update_vr_to_fc_t, atti_pitch) }, \
         { "atti_roll", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_update_vr_to_fc_t, atti_roll) }, \
         { "atti_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_update_vr_to_fc_t, atti_yaw) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_update_vr_to_fc_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 48, offsetof(ttalink_update_vr_to_fc_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 52, offsetof(ttalink_update_vr_to_fc_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 56, offsetof(ttalink_update_vr_to_fc_t, param4) }, \
         } \
}
#endif


static inline uint16_t _ttalink_update_vr_to_fc_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               double latit, double longi, float altit, float vel_n, float vel_e, float vel_d, float atti_pitch, float atti_roll, float atti_yaw, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN];
    _tta_put_double(buf, 0, latit);
    _tta_put_double(buf, 8, longi);
    _tta_put_float(buf, 16, altit);
    _tta_put_float(buf, 20, vel_n);
    _tta_put_float(buf, 24, vel_e);
    _tta_put_float(buf, 28, vel_d);
    _tta_put_float(buf, 32, atti_pitch);
    _tta_put_float(buf, 36, atti_roll);
    _tta_put_float(buf, 40, atti_yaw);
    _tta_put_float(buf, 44, param1);
    _tta_put_float(buf, 48, param2);
    _tta_put_float(buf, 52, param3);
    _tta_put_float(buf, 56, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN);
#else
    ttalink_update_vr_to_fc_t packet;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.atti_pitch = atti_pitch;
    packet.atti_roll = atti_roll;
    packet.atti_yaw = atti_yaw;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_UPDATE_VR_TO_FC;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_UPDATE_VR_TO_FC_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN, TTALINK_MSG_ID_UPDATE_VR_TO_FC_CRC, nocrc);
}

/**
 * @brief Pack a update_vr_to_fc message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param latit  （WGS84）.
 * @param longi  （WGS84）.
 * @param altit  （m）.
 * @param vel_n  （NED坐标系）（m/s）.
 * @param vel_e  （NED坐标系）（m/s）.
 * @param vel_d  （NED坐标系）（m/s）.
 * @param atti_pitch  （符合右手定责，前右下坐标系）（°）.
 * @param atti_roll  （符合右手定责，前右下坐标系）（°）.
 * @param atti_yaw  （符合右手定责，前右下坐标系）（°）.
 * @param param1  预留.
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_update_vr_to_fc_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               double latit, double longi, float altit, float vel_n, float vel_e, float vel_d, float atti_pitch, float atti_roll, float atti_yaw, float param1, float param2, float param3, float param4)
{
    return _ttalink_update_vr_to_fc_pack(dst_addr, src_addr, msg,  latit, longi, altit, vel_n, vel_e, vel_d, atti_pitch, atti_roll, atti_yaw, param1, param2, param3, param4, false);
}

/**
 * @brief Pack a update_vr_to_fc message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param latit  （WGS84）.
 * @param longi  （WGS84）.
 * @param altit  （m）.
 * @param vel_n  （NED坐标系）（m/s）.
 * @param vel_e  （NED坐标系）（m/s）.
 * @param vel_d  （NED坐标系）（m/s）.
 * @param atti_pitch  （符合右手定责，前右下坐标系）（°）.
 * @param atti_roll  （符合右手定责，前右下坐标系）（°）.
 * @param atti_yaw  （符合右手定责，前右下坐标系）（°）.
 * @param param1  预留.
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_update_vr_to_fc_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               double latit, double longi, float altit, float vel_n, float vel_e, float vel_d, float atti_pitch, float atti_roll, float atti_yaw, float param1, float param2, float param3, float param4)
{
    return _ttalink_update_vr_to_fc_pack(dst_addr, src_addr, msg,  latit, longi, altit, vel_n, vel_e, vel_d, atti_pitch, atti_roll, atti_yaw, param1, param2, param3, param4, true);
}


static inline uint16_t _ttalink_update_vr_to_fc_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   double latit,double longi,float altit,float vel_n,float vel_e,float vel_d,float atti_pitch,float atti_roll,float atti_yaw,float param1,float param2,float param3,float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN];
    _tta_put_double(buf, 0, latit);
    _tta_put_double(buf, 8, longi);
    _tta_put_float(buf, 16, altit);
    _tta_put_float(buf, 20, vel_n);
    _tta_put_float(buf, 24, vel_e);
    _tta_put_float(buf, 28, vel_d);
    _tta_put_float(buf, 32, atti_pitch);
    _tta_put_float(buf, 36, atti_roll);
    _tta_put_float(buf, 40, atti_yaw);
    _tta_put_float(buf, 44, param1);
    _tta_put_float(buf, 48, param2);
    _tta_put_float(buf, 52, param3);
    _tta_put_float(buf, 56, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN);
#else
    ttalink_update_vr_to_fc_t packet;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.atti_pitch = atti_pitch;
    packet.atti_roll = atti_roll;
    packet.atti_yaw = atti_yaw;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_UPDATE_VR_TO_FC;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_UPDATE_VR_TO_FC_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN, TTALINK_MSG_ID_UPDATE_VR_TO_FC_CRC, nocrc);
}

/**
 * @brief Pack a update_vr_to_fc message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param latit  （WGS84）.
 * @param longi  （WGS84）.
 * @param altit  （m）.
 * @param vel_n  （NED坐标系）（m/s）.
 * @param vel_e  （NED坐标系）（m/s）.
 * @param vel_d  （NED坐标系）（m/s）.
 * @param atti_pitch  （符合右手定责，前右下坐标系）（°）.
 * @param atti_roll  （符合右手定责，前右下坐标系）（°）.
 * @param atti_yaw  （符合右手定责，前右下坐标系）（°）.
 * @param param1  预留.
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_update_vr_to_fc_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   double latit,double longi,float altit,float vel_n,float vel_e,float vel_d,float atti_pitch,float atti_roll,float atti_yaw,float param1,float param2,float param3,float param4)
{
    return _ttalink_update_vr_to_fc_pack_chan(dst_addr, src_addr, chan, msg,  latit, longi, altit, vel_n, vel_e, vel_d, atti_pitch, atti_roll, atti_yaw, param1, param2, param3, param4, false);
}

/**
 * @brief Pack a update_vr_to_fc message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param latit  （WGS84）.
 * @param longi  （WGS84）.
 * @param altit  （m）.
 * @param vel_n  （NED坐标系）（m/s）.
 * @param vel_e  （NED坐标系）（m/s）.
 * @param vel_d  （NED坐标系）（m/s）.
 * @param atti_pitch  （符合右手定责，前右下坐标系）（°）.
 * @param atti_roll  （符合右手定责，前右下坐标系）（°）.
 * @param atti_yaw  （符合右手定责，前右下坐标系）（°）.
 * @param param1  预留.
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_update_vr_to_fc_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   double latit,double longi,float altit,float vel_n,float vel_e,float vel_d,float atti_pitch,float atti_roll,float atti_yaw,float param1,float param2,float param3,float param4)
{
    return _ttalink_update_vr_to_fc_pack_chan(dst_addr, src_addr, chan, msg,  latit, longi, altit, vel_n, vel_e, vel_d, atti_pitch, atti_roll, atti_yaw, param1, param2, param3, param4, true);
}


static inline uint16_t _ttalink_update_vr_to_fc_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_update_vr_to_fc_t* update_vr_to_fc, bool nocrc)
{
    if(nocrc){
        return ttalink_update_vr_to_fc_pack_nocrc(dst_addr, src_addr, msg, update_vr_to_fc->latit, update_vr_to_fc->longi, update_vr_to_fc->altit, update_vr_to_fc->vel_n, update_vr_to_fc->vel_e, update_vr_to_fc->vel_d, update_vr_to_fc->atti_pitch, update_vr_to_fc->atti_roll, update_vr_to_fc->atti_yaw, update_vr_to_fc->param1, update_vr_to_fc->param2, update_vr_to_fc->param3, update_vr_to_fc->param4);
    }else{
        return ttalink_update_vr_to_fc_pack(dst_addr, src_addr, msg, update_vr_to_fc->latit, update_vr_to_fc->longi, update_vr_to_fc->altit, update_vr_to_fc->vel_n, update_vr_to_fc->vel_e, update_vr_to_fc->vel_d, update_vr_to_fc->atti_pitch, update_vr_to_fc->atti_roll, update_vr_to_fc->atti_yaw, update_vr_to_fc->param1, update_vr_to_fc->param2, update_vr_to_fc->param3, update_vr_to_fc->param4);
    }
    
}

/**
 * @brief Encode a update_vr_to_fc struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param update_vr_to_fc C-struct to read the message contents from
 */
static inline uint16_t ttalink_update_vr_to_fc_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_update_vr_to_fc_t* update_vr_to_fc)
{
    return _ttalink_update_vr_to_fc_encode(dst_addr, src_addr, msg, update_vr_to_fc, false);
}

/**
 * @brief Encode a update_vr_to_fc struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param update_vr_to_fc C-struct to read the message contents from
 */
static inline uint16_t ttalink_update_vr_to_fc_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_update_vr_to_fc_t* update_vr_to_fc)
{
    return _ttalink_update_vr_to_fc_encode(dst_addr, src_addr, msg, update_vr_to_fc, true);
}


static inline uint16_t _ttalink_update_vr_to_fc_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_update_vr_to_fc_t* update_vr_to_fc, bool nocrc)
{
    if(nocrc){
        return ttalink_update_vr_to_fc_pack_chan_nocrc(dst_addr, src_addr, chan, msg, update_vr_to_fc->latit, update_vr_to_fc->longi, update_vr_to_fc->altit, update_vr_to_fc->vel_n, update_vr_to_fc->vel_e, update_vr_to_fc->vel_d, update_vr_to_fc->atti_pitch, update_vr_to_fc->atti_roll, update_vr_to_fc->atti_yaw, update_vr_to_fc->param1, update_vr_to_fc->param2, update_vr_to_fc->param3, update_vr_to_fc->param4);
    }else{
        return ttalink_update_vr_to_fc_pack_chan(dst_addr, src_addr, chan, msg, update_vr_to_fc->latit, update_vr_to_fc->longi, update_vr_to_fc->altit, update_vr_to_fc->vel_n, update_vr_to_fc->vel_e, update_vr_to_fc->vel_d, update_vr_to_fc->atti_pitch, update_vr_to_fc->atti_roll, update_vr_to_fc->atti_yaw, update_vr_to_fc->param1, update_vr_to_fc->param2, update_vr_to_fc->param3, update_vr_to_fc->param4);
    }
}

/**
 * @brief Encode a update_vr_to_fc struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_vr_to_fc C-struct to read the message contents from
 */
static inline uint16_t ttalink_update_vr_to_fc_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_update_vr_to_fc_t* update_vr_to_fc)
{
    return _ttalink_update_vr_to_fc_encode_chan(dst_addr, src_addr, chan, msg, update_vr_to_fc, false);
}

/**
 * @brief Encode a update_vr_to_fc struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_vr_to_fc C-struct to read the message contents from
 */
static inline uint16_t ttalink_update_vr_to_fc_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_update_vr_to_fc_t* update_vr_to_fc)
{
    return _ttalink_update_vr_to_fc_encode_chan(dst_addr, src_addr, chan, msg, update_vr_to_fc, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_update_vr_to_fc_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, double latit, double longi, float altit, float vel_n, float vel_e, float vel_d, float atti_pitch, float atti_roll, float atti_yaw, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN];
    _tta_put_double(buf, 0, latit);
    _tta_put_double(buf, 8, longi);
    _tta_put_float(buf, 16, altit);
    _tta_put_float(buf, 20, vel_n);
    _tta_put_float(buf, 24, vel_e);
    _tta_put_float(buf, 28, vel_d);
    _tta_put_float(buf, 32, atti_pitch);
    _tta_put_float(buf, 36, atti_roll);
    _tta_put_float(buf, 40, atti_yaw);
    _tta_put_float(buf, 44, param1);
    _tta_put_float(buf, 48, param2);
    _tta_put_float(buf, 52, param3);
    _tta_put_float(buf, 56, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPDATE_VR_TO_FC, buf, TTALINK_MSG_ID_UPDATE_VR_TO_FC_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN, TTALINK_MSG_ID_UPDATE_VR_TO_FC_CRC, nocrc);
#else
    ttalink_update_vr_to_fc_t packet;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.atti_pitch = atti_pitch;
    packet.atti_roll = atti_roll;
    packet.atti_yaw = atti_yaw;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPDATE_VR_TO_FC, (const char *)&packet, TTALINK_MSG_ID_UPDATE_VR_TO_FC_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN, TTALINK_MSG_ID_UPDATE_VR_TO_FC_CRC, nocrc);
#endif
}

/**
 * @brief Send a update_vr_to_fc message
 * @param chan TTAlink channel to send the message
 *
 * @param latit  （WGS84）.
 * @param longi  （WGS84）.
 * @param altit  （m）.
 * @param vel_n  （NED坐标系）（m/s）.
 * @param vel_e  （NED坐标系）（m/s）.
 * @param vel_d  （NED坐标系）（m/s）.
 * @param atti_pitch  （符合右手定责，前右下坐标系）（°）.
 * @param atti_roll  （符合右手定责，前右下坐标系）（°）.
 * @param atti_yaw  （符合右手定责，前右下坐标系）（°）.
 * @param param1  预留.
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 */
static inline void ttalink_update_vr_to_fc_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, double latit, double longi, float altit, float vel_n, float vel_e, float vel_d, float atti_pitch, float atti_roll, float atti_yaw, float param1, float param2, float param3, float param4)
{
    _ttalink_update_vr_to_fc_send(dst_addr, src_addr, chan, latit, longi, altit, vel_n, vel_e, vel_d, atti_pitch, atti_roll, atti_yaw, param1, param2, param3, param4, false);
}

/**
 * @brief Send a update_vr_to_fc message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param latit  （WGS84）.
 * @param longi  （WGS84）.
 * @param altit  （m）.
 * @param vel_n  （NED坐标系）（m/s）.
 * @param vel_e  （NED坐标系）（m/s）.
 * @param vel_d  （NED坐标系）（m/s）.
 * @param atti_pitch  （符合右手定责，前右下坐标系）（°）.
 * @param atti_roll  （符合右手定责，前右下坐标系）（°）.
 * @param atti_yaw  （符合右手定责，前右下坐标系）（°）.
 * @param param1  预留.
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 */
static inline void ttalink_update_vr_to_fc_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, double latit, double longi, float altit, float vel_n, float vel_e, float vel_d, float atti_pitch, float atti_roll, float atti_yaw, float param1, float param2, float param3, float param4)
{
    _ttalink_update_vr_to_fc_send(dst_addr, src_addr, chan, latit, longi, altit, vel_n, vel_e, vel_d, atti_pitch, atti_roll, atti_yaw, param1, param2, param3, param4, true);
}


static inline void _ttalink_update_vr_to_fc_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_update_vr_to_fc_t* update_vr_to_fc, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_update_vr_to_fc_send_nocrc(dst_addr, src_addr, chan, update_vr_to_fc->latit, update_vr_to_fc->longi, update_vr_to_fc->altit, update_vr_to_fc->vel_n, update_vr_to_fc->vel_e, update_vr_to_fc->vel_d, update_vr_to_fc->atti_pitch, update_vr_to_fc->atti_roll, update_vr_to_fc->atti_yaw, update_vr_to_fc->param1, update_vr_to_fc->param2, update_vr_to_fc->param3, update_vr_to_fc->param4);
    }else{
        ttalink_update_vr_to_fc_send(dst_addr, src_addr, chan, update_vr_to_fc->latit, update_vr_to_fc->longi, update_vr_to_fc->altit, update_vr_to_fc->vel_n, update_vr_to_fc->vel_e, update_vr_to_fc->vel_d, update_vr_to_fc->atti_pitch, update_vr_to_fc->atti_roll, update_vr_to_fc->atti_yaw, update_vr_to_fc->param1, update_vr_to_fc->param2, update_vr_to_fc->param3, update_vr_to_fc->param4);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPDATE_VR_TO_FC, (const char *)update_vr_to_fc, TTALINK_MSG_ID_UPDATE_VR_TO_FC_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN, TTALINK_MSG_ID_UPDATE_VR_TO_FC_CRC, nocrc);
#endif
}

/**
 * @brief Send a update_vr_to_fc message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_update_vr_to_fc_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_update_vr_to_fc_t* update_vr_to_fc)
{
    _ttalink_update_vr_to_fc_send_struct(dst_addr, src_addr, chan, update_vr_to_fc, false);
}

/**
 * @brief Send a update_vr_to_fc message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_update_vr_to_fc_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_update_vr_to_fc_t* update_vr_to_fc)
{
    _ttalink_update_vr_to_fc_send_struct(dst_addr, src_addr, chan, update_vr_to_fc, true);
}

#if TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_update_vr_to_fc_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  double latit, double longi, float altit, float vel_n, float vel_e, float vel_d, float atti_pitch, float atti_roll, float atti_yaw, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_double(buf, 0, latit);
    _tta_put_double(buf, 8, longi);
    _tta_put_float(buf, 16, altit);
    _tta_put_float(buf, 20, vel_n);
    _tta_put_float(buf, 24, vel_e);
    _tta_put_float(buf, 28, vel_d);
    _tta_put_float(buf, 32, atti_pitch);
    _tta_put_float(buf, 36, atti_roll);
    _tta_put_float(buf, 40, atti_yaw);
    _tta_put_float(buf, 44, param1);
    _tta_put_float(buf, 48, param2);
    _tta_put_float(buf, 52, param3);
    _tta_put_float(buf, 56, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPDATE_VR_TO_FC, buf, TTALINK_MSG_ID_UPDATE_VR_TO_FC_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN, TTALINK_MSG_ID_UPDATE_VR_TO_FC_CRC, nocrc);
#else
    ttalink_update_vr_to_fc_t *packet = (ttalink_update_vr_to_fc_t *)msgbuf;
    packet->latit = latit;
    packet->longi = longi;
    packet->altit = altit;
    packet->vel_n = vel_n;
    packet->vel_e = vel_e;
    packet->vel_d = vel_d;
    packet->atti_pitch = atti_pitch;
    packet->atti_roll = atti_roll;
    packet->atti_yaw = atti_yaw;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPDATE_VR_TO_FC, (const char *)packet, TTALINK_MSG_ID_UPDATE_VR_TO_FC_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN, TTALINK_MSG_ID_UPDATE_VR_TO_FC_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_update_vr_to_fc_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  double latit, double longi, float altit, float vel_n, float vel_e, float vel_d, float atti_pitch, float atti_roll, float atti_yaw, float param1, float param2, float param3, float param4)
{
    _ttalink_update_vr_to_fc_send_buf(dst_addr, src_addr, msgbuf, chan, latit, longi, altit, vel_n, vel_e, vel_d, atti_pitch, atti_roll, atti_yaw, param1, param2, param3, param4, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_update_vr_to_fc_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  double latit, double longi, float altit, float vel_n, float vel_e, float vel_d, float atti_pitch, float atti_roll, float atti_yaw, float param1, float param2, float param3, float param4)
{
    _ttalink_update_vr_to_fc_send_buf(dst_addr, src_addr, msgbuf, chan, latit, longi, altit, vel_n, vel_e, vel_d, atti_pitch, atti_roll, atti_yaw, param1, param2, param3, param4, true);
}
#endif

#endif

// MESSAGE UPDATE_VR_TO_FC UNPACKING


/**
 * @brief Get field latit from update_vr_to_fc message
 *
 * @return  （WGS84）.
 */
static inline double ttalink_update_vr_to_fc_get_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  0);
}

/**
 * @brief Get field longi from update_vr_to_fc message
 *
 * @return  （WGS84）.
 */
static inline double ttalink_update_vr_to_fc_get_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  8);
}

/**
 * @brief Get field altit from update_vr_to_fc message
 *
 * @return  （m）.
 */
static inline float ttalink_update_vr_to_fc_get_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field vel_n from update_vr_to_fc message
 *
 * @return  （NED坐标系）（m/s）.
 */
static inline float ttalink_update_vr_to_fc_get_vel_n(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  20);
}

/**
 * @brief Get field vel_e from update_vr_to_fc message
 *
 * @return  （NED坐标系）（m/s）.
 */
static inline float ttalink_update_vr_to_fc_get_vel_e(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Get field vel_d from update_vr_to_fc message
 *
 * @return  （NED坐标系）（m/s）.
 */
static inline float ttalink_update_vr_to_fc_get_vel_d(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  28);
}

/**
 * @brief Get field atti_pitch from update_vr_to_fc message
 *
 * @return  （符合右手定责，前右下坐标系）（°）.
 */
static inline float ttalink_update_vr_to_fc_get_atti_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Get field atti_roll from update_vr_to_fc message
 *
 * @return  （符合右手定责，前右下坐标系）（°）.
 */
static inline float ttalink_update_vr_to_fc_get_atti_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  36);
}

/**
 * @brief Get field atti_yaw from update_vr_to_fc message
 *
 * @return  （符合右手定责，前右下坐标系）（°）.
 */
static inline float ttalink_update_vr_to_fc_get_atti_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  40);
}

/**
 * @brief Get field param1 from update_vr_to_fc message
 *
 * @return  预留.
 */
static inline float ttalink_update_vr_to_fc_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  44);
}

/**
 * @brief Get field param2 from update_vr_to_fc message
 *
 * @return  预留.
 */
static inline float ttalink_update_vr_to_fc_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  48);
}

/**
 * @brief Get field param3 from update_vr_to_fc message
 *
 * @return  预留.
 */
static inline float ttalink_update_vr_to_fc_get_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  52);
}

/**
 * @brief Get field param4 from update_vr_to_fc message
 *
 * @return  预留.
 */
static inline float ttalink_update_vr_to_fc_get_param4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  56);
}

/**
 * @brief Decode a update_vr_to_fc message into a struct
 *
 * @param msg The message to decode
 * @param update_vr_to_fc C-struct to decode the message contents into
 */
static inline void ttalink_update_vr_to_fc_decode(const ttalink_message_t* msg, ttalink_update_vr_to_fc_t* update_vr_to_fc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    update_vr_to_fc->latit = ttalink_update_vr_to_fc_get_latit(msg);
    update_vr_to_fc->longi = ttalink_update_vr_to_fc_get_longi(msg);
    update_vr_to_fc->altit = ttalink_update_vr_to_fc_get_altit(msg);
    update_vr_to_fc->vel_n = ttalink_update_vr_to_fc_get_vel_n(msg);
    update_vr_to_fc->vel_e = ttalink_update_vr_to_fc_get_vel_e(msg);
    update_vr_to_fc->vel_d = ttalink_update_vr_to_fc_get_vel_d(msg);
    update_vr_to_fc->atti_pitch = ttalink_update_vr_to_fc_get_atti_pitch(msg);
    update_vr_to_fc->atti_roll = ttalink_update_vr_to_fc_get_atti_roll(msg);
    update_vr_to_fc->atti_yaw = ttalink_update_vr_to_fc_get_atti_yaw(msg);
    update_vr_to_fc->param1 = ttalink_update_vr_to_fc_get_param1(msg);
    update_vr_to_fc->param2 = ttalink_update_vr_to_fc_get_param2(msg);
    update_vr_to_fc->param3 = ttalink_update_vr_to_fc_get_param3(msg);
    update_vr_to_fc->param4 = ttalink_update_vr_to_fc_get_param4(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN? msg->len : TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN;
        memset(update_vr_to_fc, 0, TTALINK_MSG_ID_UPDATE_VR_TO_FC_LEN);
    memcpy(update_vr_to_fc, _TTA_PAYLOAD(msg), len);
#endif
}
