#pragma once
// MESSAGE MAV_HIL_SIM_STATE PACKING

#define TTALINK_MSG_ID_MAV_HIL_SIM_STATE 108

TTAPACKED(
typedef struct __ttalink_mav_hil_sim_state_t {
 float q1; /*< True attitude quaternion component 1, w (1 in null-rotation)*/
 float q2; /*< True attitude quaternion component 2, x (0 in null-rotation)*/
 float q3; /*< True attitude quaternion component 3, y (0 in null-rotation)*/
 float q4; /*< True attitude quaternion component 4, z (0 in null-rotation)*/
 float roll; /*< Attitude roll expressed as Euler angles, not recommended except for human-readable outputs*/
 float pitch; /*< Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs*/
 float yaw; /*< Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs*/
 float xacc; /*< X acceleration m/s/s*/
 float yacc; /*< Y acceleration m/s/s*/
 float zacc; /*< Z acceleration m/s/s*/
 float xgyro; /*< Angular speed around X axis rad/s*/
 float ygyro; /*< Angular speed around Y axis rad/s*/
 float zgyro; /*< Angular speed around Z axis rad/s*/
 float lat; /*< Latitude in degrees*/
 float lon; /*< Longitude in degrees*/
 float alt; /*< Altitude in meters*/
 float std_dev_horz; /*< Horizontal position standard deviation*/
 float std_dev_vert; /*< Vertical position standard deviation*/
 float vn; /*< True velocity in m/s in NORTH direction in earth-fixed NED frame*/
 float ve; /*< True velocity in m/s in EAST direction in earth-fixed NED frame*/
 float vd; /*< True velocity in m/s in DOWN direction in earth-fixed NED frame*/
}) ttalink_mav_hil_sim_state_t;

#define TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN 84
#define TTALINK_MSG_ID_MAV_HIL_SIM_STATE_MIN_LEN 84
#define TTALINK_MSG_ID_108_LEN 84
#define TTALINK_MSG_ID_108_MIN_LEN 84

#define TTALINK_MSG_ID_MAV_HIL_SIM_STATE_CRC 168
#define TTALINK_MSG_ID_108_CRC 168



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_MAV_HIL_SIM_STATE { \
    108, \
    "MAV_HIL_SIM_STATE", \
    21, \
    {  { "q1", NULL, TTALINK_TYPE_FLOAT, 0, 0, offsetof(ttalink_mav_hil_sim_state_t, q1) }, \
         { "q2", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_mav_hil_sim_state_t, q2) }, \
         { "q3", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_mav_hil_sim_state_t, q3) }, \
         { "q4", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_mav_hil_sim_state_t, q4) }, \
         { "roll", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_mav_hil_sim_state_t, roll) }, \
         { "pitch", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_mav_hil_sim_state_t, pitch) }, \
         { "yaw", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_mav_hil_sim_state_t, yaw) }, \
         { "xacc", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_mav_hil_sim_state_t, xacc) }, \
         { "yacc", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_mav_hil_sim_state_t, yacc) }, \
         { "zacc", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_mav_hil_sim_state_t, zacc) }, \
         { "xgyro", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_mav_hil_sim_state_t, xgyro) }, \
         { "ygyro", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_mav_hil_sim_state_t, ygyro) }, \
         { "zgyro", NULL, TTALINK_TYPE_FLOAT, 0, 48, offsetof(ttalink_mav_hil_sim_state_t, zgyro) }, \
         { "lat", NULL, TTALINK_TYPE_FLOAT, 0, 52, offsetof(ttalink_mav_hil_sim_state_t, lat) }, \
         { "lon", NULL, TTALINK_TYPE_FLOAT, 0, 56, offsetof(ttalink_mav_hil_sim_state_t, lon) }, \
         { "alt", NULL, TTALINK_TYPE_FLOAT, 0, 60, offsetof(ttalink_mav_hil_sim_state_t, alt) }, \
         { "std_dev_horz", NULL, TTALINK_TYPE_FLOAT, 0, 64, offsetof(ttalink_mav_hil_sim_state_t, std_dev_horz) }, \
         { "std_dev_vert", NULL, TTALINK_TYPE_FLOAT, 0, 68, offsetof(ttalink_mav_hil_sim_state_t, std_dev_vert) }, \
         { "vn", NULL, TTALINK_TYPE_FLOAT, 0, 72, offsetof(ttalink_mav_hil_sim_state_t, vn) }, \
         { "ve", NULL, TTALINK_TYPE_FLOAT, 0, 76, offsetof(ttalink_mav_hil_sim_state_t, ve) }, \
         { "vd", NULL, TTALINK_TYPE_FLOAT, 0, 80, offsetof(ttalink_mav_hil_sim_state_t, vd) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_MAV_HIL_SIM_STATE { \
    "MAV_HIL_SIM_STATE", \
    21, \
    {  { "q1", NULL, TTALINK_TYPE_FLOAT, 0, 0, offsetof(ttalink_mav_hil_sim_state_t, q1) }, \
         { "q2", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_mav_hil_sim_state_t, q2) }, \
         { "q3", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_mav_hil_sim_state_t, q3) }, \
         { "q4", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_mav_hil_sim_state_t, q4) }, \
         { "roll", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_mav_hil_sim_state_t, roll) }, \
         { "pitch", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_mav_hil_sim_state_t, pitch) }, \
         { "yaw", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_mav_hil_sim_state_t, yaw) }, \
         { "xacc", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_mav_hil_sim_state_t, xacc) }, \
         { "yacc", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_mav_hil_sim_state_t, yacc) }, \
         { "zacc", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_mav_hil_sim_state_t, zacc) }, \
         { "xgyro", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_mav_hil_sim_state_t, xgyro) }, \
         { "ygyro", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_mav_hil_sim_state_t, ygyro) }, \
         { "zgyro", NULL, TTALINK_TYPE_FLOAT, 0, 48, offsetof(ttalink_mav_hil_sim_state_t, zgyro) }, \
         { "lat", NULL, TTALINK_TYPE_FLOAT, 0, 52, offsetof(ttalink_mav_hil_sim_state_t, lat) }, \
         { "lon", NULL, TTALINK_TYPE_FLOAT, 0, 56, offsetof(ttalink_mav_hil_sim_state_t, lon) }, \
         { "alt", NULL, TTALINK_TYPE_FLOAT, 0, 60, offsetof(ttalink_mav_hil_sim_state_t, alt) }, \
         { "std_dev_horz", NULL, TTALINK_TYPE_FLOAT, 0, 64, offsetof(ttalink_mav_hil_sim_state_t, std_dev_horz) }, \
         { "std_dev_vert", NULL, TTALINK_TYPE_FLOAT, 0, 68, offsetof(ttalink_mav_hil_sim_state_t, std_dev_vert) }, \
         { "vn", NULL, TTALINK_TYPE_FLOAT, 0, 72, offsetof(ttalink_mav_hil_sim_state_t, vn) }, \
         { "ve", NULL, TTALINK_TYPE_FLOAT, 0, 76, offsetof(ttalink_mav_hil_sim_state_t, ve) }, \
         { "vd", NULL, TTALINK_TYPE_FLOAT, 0, 80, offsetof(ttalink_mav_hil_sim_state_t, vd) }, \
         } \
}
#endif


static inline uint16_t _ttalink_mav_hil_sim_state_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float lat, float lon, float alt, float std_dev_horz, float std_dev_vert, float vn, float ve, float vd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN];
    _tta_put_float(buf, 0, q1);
    _tta_put_float(buf, 4, q2);
    _tta_put_float(buf, 8, q3);
    _tta_put_float(buf, 12, q4);
    _tta_put_float(buf, 16, roll);
    _tta_put_float(buf, 20, pitch);
    _tta_put_float(buf, 24, yaw);
    _tta_put_float(buf, 28, xacc);
    _tta_put_float(buf, 32, yacc);
    _tta_put_float(buf, 36, zacc);
    _tta_put_float(buf, 40, xgyro);
    _tta_put_float(buf, 44, ygyro);
    _tta_put_float(buf, 48, zgyro);
    _tta_put_float(buf, 52, lat);
    _tta_put_float(buf, 56, lon);
    _tta_put_float(buf, 60, alt);
    _tta_put_float(buf, 64, std_dev_horz);
    _tta_put_float(buf, 68, std_dev_vert);
    _tta_put_float(buf, 72, vn);
    _tta_put_float(buf, 76, ve);
    _tta_put_float(buf, 80, vd);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN);
#else
    ttalink_mav_hil_sim_state_t packet;
    packet.q1 = q1;
    packet.q2 = q2;
    packet.q3 = q3;
    packet.q4 = q4;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.std_dev_horz = std_dev_horz;
    packet.std_dev_vert = std_dev_vert;
    packet.vn = vn;
    packet.ve = ve;
    packet.vd = vd;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MAV_HIL_SIM_STATE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_CRC, nocrc);
}

/**
 * @brief Pack a mav_hil_sim_state message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param q1 True attitude quaternion component 1, w (1 in null-rotation)
 * @param q2 True attitude quaternion component 2, x (0 in null-rotation)
 * @param q3 True attitude quaternion component 3, y (0 in null-rotation)
 * @param q4 True attitude quaternion component 4, z (0 in null-rotation)
 * @param roll Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
 * @param pitch Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
 * @param yaw Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
 * @param xacc X acceleration m/s/s
 * @param yacc Y acceleration m/s/s
 * @param zacc Z acceleration m/s/s
 * @param xgyro Angular speed around X axis rad/s
 * @param ygyro Angular speed around Y axis rad/s
 * @param zgyro Angular speed around Z axis rad/s
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param alt Altitude in meters
 * @param std_dev_horz Horizontal position standard deviation
 * @param std_dev_vert Vertical position standard deviation
 * @param vn True velocity in m/s in NORTH direction in earth-fixed NED frame
 * @param ve True velocity in m/s in EAST direction in earth-fixed NED frame
 * @param vd True velocity in m/s in DOWN direction in earth-fixed NED frame
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_sim_state_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float lat, float lon, float alt, float std_dev_horz, float std_dev_vert, float vn, float ve, float vd)
{
    return _ttalink_mav_hil_sim_state_pack(dst_addr, src_addr, msg,  q1, q2, q3, q4, roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lon, alt, std_dev_horz, std_dev_vert, vn, ve, vd, false);
}

/**
 * @brief Pack a mav_hil_sim_state message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param q1 True attitude quaternion component 1, w (1 in null-rotation)
 * @param q2 True attitude quaternion component 2, x (0 in null-rotation)
 * @param q3 True attitude quaternion component 3, y (0 in null-rotation)
 * @param q4 True attitude quaternion component 4, z (0 in null-rotation)
 * @param roll Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
 * @param pitch Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
 * @param yaw Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
 * @param xacc X acceleration m/s/s
 * @param yacc Y acceleration m/s/s
 * @param zacc Z acceleration m/s/s
 * @param xgyro Angular speed around X axis rad/s
 * @param ygyro Angular speed around Y axis rad/s
 * @param zgyro Angular speed around Z axis rad/s
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param alt Altitude in meters
 * @param std_dev_horz Horizontal position standard deviation
 * @param std_dev_vert Vertical position standard deviation
 * @param vn True velocity in m/s in NORTH direction in earth-fixed NED frame
 * @param ve True velocity in m/s in EAST direction in earth-fixed NED frame
 * @param vd True velocity in m/s in DOWN direction in earth-fixed NED frame
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_sim_state_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float lat, float lon, float alt, float std_dev_horz, float std_dev_vert, float vn, float ve, float vd)
{
    return _ttalink_mav_hil_sim_state_pack(dst_addr, src_addr, msg,  q1, q2, q3, q4, roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lon, alt, std_dev_horz, std_dev_vert, vn, ve, vd, true);
}


static inline uint16_t _ttalink_mav_hil_sim_state_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float q1,float q2,float q3,float q4,float roll,float pitch,float yaw,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float lat,float lon,float alt,float std_dev_horz,float std_dev_vert,float vn,float ve,float vd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN];
    _tta_put_float(buf, 0, q1);
    _tta_put_float(buf, 4, q2);
    _tta_put_float(buf, 8, q3);
    _tta_put_float(buf, 12, q4);
    _tta_put_float(buf, 16, roll);
    _tta_put_float(buf, 20, pitch);
    _tta_put_float(buf, 24, yaw);
    _tta_put_float(buf, 28, xacc);
    _tta_put_float(buf, 32, yacc);
    _tta_put_float(buf, 36, zacc);
    _tta_put_float(buf, 40, xgyro);
    _tta_put_float(buf, 44, ygyro);
    _tta_put_float(buf, 48, zgyro);
    _tta_put_float(buf, 52, lat);
    _tta_put_float(buf, 56, lon);
    _tta_put_float(buf, 60, alt);
    _tta_put_float(buf, 64, std_dev_horz);
    _tta_put_float(buf, 68, std_dev_vert);
    _tta_put_float(buf, 72, vn);
    _tta_put_float(buf, 76, ve);
    _tta_put_float(buf, 80, vd);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN);
#else
    ttalink_mav_hil_sim_state_t packet;
    packet.q1 = q1;
    packet.q2 = q2;
    packet.q3 = q3;
    packet.q4 = q4;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.std_dev_horz = std_dev_horz;
    packet.std_dev_vert = std_dev_vert;
    packet.vn = vn;
    packet.ve = ve;
    packet.vd = vd;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MAV_HIL_SIM_STATE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_CRC, nocrc);
}

/**
 * @brief Pack a mav_hil_sim_state message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param q1 True attitude quaternion component 1, w (1 in null-rotation)
 * @param q2 True attitude quaternion component 2, x (0 in null-rotation)
 * @param q3 True attitude quaternion component 3, y (0 in null-rotation)
 * @param q4 True attitude quaternion component 4, z (0 in null-rotation)
 * @param roll Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
 * @param pitch Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
 * @param yaw Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
 * @param xacc X acceleration m/s/s
 * @param yacc Y acceleration m/s/s
 * @param zacc Z acceleration m/s/s
 * @param xgyro Angular speed around X axis rad/s
 * @param ygyro Angular speed around Y axis rad/s
 * @param zgyro Angular speed around Z axis rad/s
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param alt Altitude in meters
 * @param std_dev_horz Horizontal position standard deviation
 * @param std_dev_vert Vertical position standard deviation
 * @param vn True velocity in m/s in NORTH direction in earth-fixed NED frame
 * @param ve True velocity in m/s in EAST direction in earth-fixed NED frame
 * @param vd True velocity in m/s in DOWN direction in earth-fixed NED frame
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_sim_state_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float q1,float q2,float q3,float q4,float roll,float pitch,float yaw,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float lat,float lon,float alt,float std_dev_horz,float std_dev_vert,float vn,float ve,float vd)
{
    return _ttalink_mav_hil_sim_state_pack_chan(dst_addr, src_addr, chan, msg,  q1, q2, q3, q4, roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lon, alt, std_dev_horz, std_dev_vert, vn, ve, vd, false);
}

/**
 * @brief Pack a mav_hil_sim_state message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param q1 True attitude quaternion component 1, w (1 in null-rotation)
 * @param q2 True attitude quaternion component 2, x (0 in null-rotation)
 * @param q3 True attitude quaternion component 3, y (0 in null-rotation)
 * @param q4 True attitude quaternion component 4, z (0 in null-rotation)
 * @param roll Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
 * @param pitch Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
 * @param yaw Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
 * @param xacc X acceleration m/s/s
 * @param yacc Y acceleration m/s/s
 * @param zacc Z acceleration m/s/s
 * @param xgyro Angular speed around X axis rad/s
 * @param ygyro Angular speed around Y axis rad/s
 * @param zgyro Angular speed around Z axis rad/s
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param alt Altitude in meters
 * @param std_dev_horz Horizontal position standard deviation
 * @param std_dev_vert Vertical position standard deviation
 * @param vn True velocity in m/s in NORTH direction in earth-fixed NED frame
 * @param ve True velocity in m/s in EAST direction in earth-fixed NED frame
 * @param vd True velocity in m/s in DOWN direction in earth-fixed NED frame
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_sim_state_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float q1,float q2,float q3,float q4,float roll,float pitch,float yaw,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float lat,float lon,float alt,float std_dev_horz,float std_dev_vert,float vn,float ve,float vd)
{
    return _ttalink_mav_hil_sim_state_pack_chan(dst_addr, src_addr, chan, msg,  q1, q2, q3, q4, roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lon, alt, std_dev_horz, std_dev_vert, vn, ve, vd, true);
}


static inline uint16_t _ttalink_mav_hil_sim_state_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_sim_state_t* mav_hil_sim_state, bool nocrc)
{
    if(nocrc){
        return ttalink_mav_hil_sim_state_pack_nocrc(dst_addr, src_addr, msg, mav_hil_sim_state->q1, mav_hil_sim_state->q2, mav_hil_sim_state->q3, mav_hil_sim_state->q4, mav_hil_sim_state->roll, mav_hil_sim_state->pitch, mav_hil_sim_state->yaw, mav_hil_sim_state->xacc, mav_hil_sim_state->yacc, mav_hil_sim_state->zacc, mav_hil_sim_state->xgyro, mav_hil_sim_state->ygyro, mav_hil_sim_state->zgyro, mav_hil_sim_state->lat, mav_hil_sim_state->lon, mav_hil_sim_state->alt, mav_hil_sim_state->std_dev_horz, mav_hil_sim_state->std_dev_vert, mav_hil_sim_state->vn, mav_hil_sim_state->ve, mav_hil_sim_state->vd);
    }else{
        return ttalink_mav_hil_sim_state_pack(dst_addr, src_addr, msg, mav_hil_sim_state->q1, mav_hil_sim_state->q2, mav_hil_sim_state->q3, mav_hil_sim_state->q4, mav_hil_sim_state->roll, mav_hil_sim_state->pitch, mav_hil_sim_state->yaw, mav_hil_sim_state->xacc, mav_hil_sim_state->yacc, mav_hil_sim_state->zacc, mav_hil_sim_state->xgyro, mav_hil_sim_state->ygyro, mav_hil_sim_state->zgyro, mav_hil_sim_state->lat, mav_hil_sim_state->lon, mav_hil_sim_state->alt, mav_hil_sim_state->std_dev_horz, mav_hil_sim_state->std_dev_vert, mav_hil_sim_state->vn, mav_hil_sim_state->ve, mav_hil_sim_state->vd);
    }
    
}

/**
 * @brief Encode a mav_hil_sim_state struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_sim_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_sim_state_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_sim_state_t* mav_hil_sim_state)
{
    return _ttalink_mav_hil_sim_state_encode(dst_addr, src_addr, msg, mav_hil_sim_state, false);
}

/**
 * @brief Encode a mav_hil_sim_state struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_sim_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_sim_state_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_sim_state_t* mav_hil_sim_state)
{
    return _ttalink_mav_hil_sim_state_encode(dst_addr, src_addr, msg, mav_hil_sim_state, true);
}


static inline uint16_t _ttalink_mav_hil_sim_state_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_sim_state_t* mav_hil_sim_state, bool nocrc)
{
    if(nocrc){
        return ttalink_mav_hil_sim_state_pack_chan_nocrc(dst_addr, src_addr, chan, msg, mav_hil_sim_state->q1, mav_hil_sim_state->q2, mav_hil_sim_state->q3, mav_hil_sim_state->q4, mav_hil_sim_state->roll, mav_hil_sim_state->pitch, mav_hil_sim_state->yaw, mav_hil_sim_state->xacc, mav_hil_sim_state->yacc, mav_hil_sim_state->zacc, mav_hil_sim_state->xgyro, mav_hil_sim_state->ygyro, mav_hil_sim_state->zgyro, mav_hil_sim_state->lat, mav_hil_sim_state->lon, mav_hil_sim_state->alt, mav_hil_sim_state->std_dev_horz, mav_hil_sim_state->std_dev_vert, mav_hil_sim_state->vn, mav_hil_sim_state->ve, mav_hil_sim_state->vd);
    }else{
        return ttalink_mav_hil_sim_state_pack_chan(dst_addr, src_addr, chan, msg, mav_hil_sim_state->q1, mav_hil_sim_state->q2, mav_hil_sim_state->q3, mav_hil_sim_state->q4, mav_hil_sim_state->roll, mav_hil_sim_state->pitch, mav_hil_sim_state->yaw, mav_hil_sim_state->xacc, mav_hil_sim_state->yacc, mav_hil_sim_state->zacc, mav_hil_sim_state->xgyro, mav_hil_sim_state->ygyro, mav_hil_sim_state->zgyro, mav_hil_sim_state->lat, mav_hil_sim_state->lon, mav_hil_sim_state->alt, mav_hil_sim_state->std_dev_horz, mav_hil_sim_state->std_dev_vert, mav_hil_sim_state->vn, mav_hil_sim_state->ve, mav_hil_sim_state->vd);
    }
}

/**
 * @brief Encode a mav_hil_sim_state struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_sim_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_sim_state_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_sim_state_t* mav_hil_sim_state)
{
    return _ttalink_mav_hil_sim_state_encode_chan(dst_addr, src_addr, chan, msg, mav_hil_sim_state, false);
}

/**
 * @brief Encode a mav_hil_sim_state struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_sim_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_sim_state_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_sim_state_t* mav_hil_sim_state)
{
    return _ttalink_mav_hil_sim_state_encode_chan(dst_addr, src_addr, chan, msg, mav_hil_sim_state, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_mav_hil_sim_state_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float lat, float lon, float alt, float std_dev_horz, float std_dev_vert, float vn, float ve, float vd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN];
    _tta_put_float(buf, 0, q1);
    _tta_put_float(buf, 4, q2);
    _tta_put_float(buf, 8, q3);
    _tta_put_float(buf, 12, q4);
    _tta_put_float(buf, 16, roll);
    _tta_put_float(buf, 20, pitch);
    _tta_put_float(buf, 24, yaw);
    _tta_put_float(buf, 28, xacc);
    _tta_put_float(buf, 32, yacc);
    _tta_put_float(buf, 36, zacc);
    _tta_put_float(buf, 40, xgyro);
    _tta_put_float(buf, 44, ygyro);
    _tta_put_float(buf, 48, zgyro);
    _tta_put_float(buf, 52, lat);
    _tta_put_float(buf, 56, lon);
    _tta_put_float(buf, 60, alt);
    _tta_put_float(buf, 64, std_dev_horz);
    _tta_put_float(buf, 68, std_dev_vert);
    _tta_put_float(buf, 72, vn);
    _tta_put_float(buf, 76, ve);
    _tta_put_float(buf, 80, vd);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_SIM_STATE, buf, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_CRC, nocrc);
#else
    ttalink_mav_hil_sim_state_t packet;
    packet.q1 = q1;
    packet.q2 = q2;
    packet.q3 = q3;
    packet.q4 = q4;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.std_dev_horz = std_dev_horz;
    packet.std_dev_vert = std_dev_vert;
    packet.vn = vn;
    packet.ve = ve;
    packet.vd = vd;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_SIM_STATE, (const char *)&packet, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_CRC, nocrc);
#endif
}

/**
 * @brief Send a mav_hil_sim_state message
 * @param chan TTAlink channel to send the message
 *
 * @param q1 True attitude quaternion component 1, w (1 in null-rotation)
 * @param q2 True attitude quaternion component 2, x (0 in null-rotation)
 * @param q3 True attitude quaternion component 3, y (0 in null-rotation)
 * @param q4 True attitude quaternion component 4, z (0 in null-rotation)
 * @param roll Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
 * @param pitch Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
 * @param yaw Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
 * @param xacc X acceleration m/s/s
 * @param yacc Y acceleration m/s/s
 * @param zacc Z acceleration m/s/s
 * @param xgyro Angular speed around X axis rad/s
 * @param ygyro Angular speed around Y axis rad/s
 * @param zgyro Angular speed around Z axis rad/s
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param alt Altitude in meters
 * @param std_dev_horz Horizontal position standard deviation
 * @param std_dev_vert Vertical position standard deviation
 * @param vn True velocity in m/s in NORTH direction in earth-fixed NED frame
 * @param ve True velocity in m/s in EAST direction in earth-fixed NED frame
 * @param vd True velocity in m/s in DOWN direction in earth-fixed NED frame
 */
static inline void ttalink_mav_hil_sim_state_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float lat, float lon, float alt, float std_dev_horz, float std_dev_vert, float vn, float ve, float vd)
{
    _ttalink_mav_hil_sim_state_send(dst_addr, src_addr, chan, q1, q2, q3, q4, roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lon, alt, std_dev_horz, std_dev_vert, vn, ve, vd, false);
}

/**
 * @brief Send a mav_hil_sim_state message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param q1 True attitude quaternion component 1, w (1 in null-rotation)
 * @param q2 True attitude quaternion component 2, x (0 in null-rotation)
 * @param q3 True attitude quaternion component 3, y (0 in null-rotation)
 * @param q4 True attitude quaternion component 4, z (0 in null-rotation)
 * @param roll Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
 * @param pitch Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
 * @param yaw Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
 * @param xacc X acceleration m/s/s
 * @param yacc Y acceleration m/s/s
 * @param zacc Z acceleration m/s/s
 * @param xgyro Angular speed around X axis rad/s
 * @param ygyro Angular speed around Y axis rad/s
 * @param zgyro Angular speed around Z axis rad/s
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param alt Altitude in meters
 * @param std_dev_horz Horizontal position standard deviation
 * @param std_dev_vert Vertical position standard deviation
 * @param vn True velocity in m/s in NORTH direction in earth-fixed NED frame
 * @param ve True velocity in m/s in EAST direction in earth-fixed NED frame
 * @param vd True velocity in m/s in DOWN direction in earth-fixed NED frame
 */
static inline void ttalink_mav_hil_sim_state_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float lat, float lon, float alt, float std_dev_horz, float std_dev_vert, float vn, float ve, float vd)
{
    _ttalink_mav_hil_sim_state_send(dst_addr, src_addr, chan, q1, q2, q3, q4, roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lon, alt, std_dev_horz, std_dev_vert, vn, ve, vd, true);
}


static inline void _ttalink_mav_hil_sim_state_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_sim_state_t* mav_hil_sim_state, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_mav_hil_sim_state_send_nocrc(dst_addr, src_addr, chan, mav_hil_sim_state->q1, mav_hil_sim_state->q2, mav_hil_sim_state->q3, mav_hil_sim_state->q4, mav_hil_sim_state->roll, mav_hil_sim_state->pitch, mav_hil_sim_state->yaw, mav_hil_sim_state->xacc, mav_hil_sim_state->yacc, mav_hil_sim_state->zacc, mav_hil_sim_state->xgyro, mav_hil_sim_state->ygyro, mav_hil_sim_state->zgyro, mav_hil_sim_state->lat, mav_hil_sim_state->lon, mav_hil_sim_state->alt, mav_hil_sim_state->std_dev_horz, mav_hil_sim_state->std_dev_vert, mav_hil_sim_state->vn, mav_hil_sim_state->ve, mav_hil_sim_state->vd);
    }else{
        ttalink_mav_hil_sim_state_send(dst_addr, src_addr, chan, mav_hil_sim_state->q1, mav_hil_sim_state->q2, mav_hil_sim_state->q3, mav_hil_sim_state->q4, mav_hil_sim_state->roll, mav_hil_sim_state->pitch, mav_hil_sim_state->yaw, mav_hil_sim_state->xacc, mav_hil_sim_state->yacc, mav_hil_sim_state->zacc, mav_hil_sim_state->xgyro, mav_hil_sim_state->ygyro, mav_hil_sim_state->zgyro, mav_hil_sim_state->lat, mav_hil_sim_state->lon, mav_hil_sim_state->alt, mav_hil_sim_state->std_dev_horz, mav_hil_sim_state->std_dev_vert, mav_hil_sim_state->vn, mav_hil_sim_state->ve, mav_hil_sim_state->vd);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_SIM_STATE, (const char *)mav_hil_sim_state, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_CRC, nocrc);
#endif
}

/**
 * @brief Send a mav_hil_sim_state message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_mav_hil_sim_state_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_sim_state_t* mav_hil_sim_state)
{
    _ttalink_mav_hil_sim_state_send_struct(dst_addr, src_addr, chan, mav_hil_sim_state, false);
}

/**
 * @brief Send a mav_hil_sim_state message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_mav_hil_sim_state_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_sim_state_t* mav_hil_sim_state)
{
    _ttalink_mav_hil_sim_state_send_struct(dst_addr, src_addr, chan, mav_hil_sim_state, true);
}

#if TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_mav_hil_sim_state_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float lat, float lon, float alt, float std_dev_horz, float std_dev_vert, float vn, float ve, float vd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_float(buf, 0, q1);
    _tta_put_float(buf, 4, q2);
    _tta_put_float(buf, 8, q3);
    _tta_put_float(buf, 12, q4);
    _tta_put_float(buf, 16, roll);
    _tta_put_float(buf, 20, pitch);
    _tta_put_float(buf, 24, yaw);
    _tta_put_float(buf, 28, xacc);
    _tta_put_float(buf, 32, yacc);
    _tta_put_float(buf, 36, zacc);
    _tta_put_float(buf, 40, xgyro);
    _tta_put_float(buf, 44, ygyro);
    _tta_put_float(buf, 48, zgyro);
    _tta_put_float(buf, 52, lat);
    _tta_put_float(buf, 56, lon);
    _tta_put_float(buf, 60, alt);
    _tta_put_float(buf, 64, std_dev_horz);
    _tta_put_float(buf, 68, std_dev_vert);
    _tta_put_float(buf, 72, vn);
    _tta_put_float(buf, 76, ve);
    _tta_put_float(buf, 80, vd);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_SIM_STATE, buf, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_CRC, nocrc);
#else
    ttalink_mav_hil_sim_state_t *packet = (ttalink_mav_hil_sim_state_t *)msgbuf;
    packet->q1 = q1;
    packet->q2 = q2;
    packet->q3 = q3;
    packet->q4 = q4;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->xacc = xacc;
    packet->yacc = yacc;
    packet->zacc = zacc;
    packet->xgyro = xgyro;
    packet->ygyro = ygyro;
    packet->zgyro = zgyro;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->std_dev_horz = std_dev_horz;
    packet->std_dev_vert = std_dev_vert;
    packet->vn = vn;
    packet->ve = ve;
    packet->vd = vd;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_SIM_STATE, (const char *)packet, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_mav_hil_sim_state_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float lat, float lon, float alt, float std_dev_horz, float std_dev_vert, float vn, float ve, float vd)
{
    _ttalink_mav_hil_sim_state_send_buf(dst_addr, src_addr, msgbuf, chan, q1, q2, q3, q4, roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lon, alt, std_dev_horz, std_dev_vert, vn, ve, vd, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_mav_hil_sim_state_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float lat, float lon, float alt, float std_dev_horz, float std_dev_vert, float vn, float ve, float vd)
{
    _ttalink_mav_hil_sim_state_send_buf(dst_addr, src_addr, msgbuf, chan, q1, q2, q3, q4, roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lon, alt, std_dev_horz, std_dev_vert, vn, ve, vd, true);
}
#endif

#endif

// MESSAGE MAV_HIL_SIM_STATE UNPACKING


/**
 * @brief Get field q1 from mav_hil_sim_state message
 *
 * @return True attitude quaternion component 1, w (1 in null-rotation)
 */
static inline float ttalink_mav_hil_sim_state_get_q1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  0);
}

/**
 * @brief Get field q2 from mav_hil_sim_state message
 *
 * @return True attitude quaternion component 2, x (0 in null-rotation)
 */
static inline float ttalink_mav_hil_sim_state_get_q2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  4);
}

/**
 * @brief Get field q3 from mav_hil_sim_state message
 *
 * @return True attitude quaternion component 3, y (0 in null-rotation)
 */
static inline float ttalink_mav_hil_sim_state_get_q3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  8);
}

/**
 * @brief Get field q4 from mav_hil_sim_state message
 *
 * @return True attitude quaternion component 4, z (0 in null-rotation)
 */
static inline float ttalink_mav_hil_sim_state_get_q4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  12);
}

/**
 * @brief Get field roll from mav_hil_sim_state message
 *
 * @return Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
 */
static inline float ttalink_mav_hil_sim_state_get_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field pitch from mav_hil_sim_state message
 *
 * @return Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
 */
static inline float ttalink_mav_hil_sim_state_get_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  20);
}

/**
 * @brief Get field yaw from mav_hil_sim_state message
 *
 * @return Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
 */
static inline float ttalink_mav_hil_sim_state_get_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Get field xacc from mav_hil_sim_state message
 *
 * @return X acceleration m/s/s
 */
static inline float ttalink_mav_hil_sim_state_get_xacc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  28);
}

/**
 * @brief Get field yacc from mav_hil_sim_state message
 *
 * @return Y acceleration m/s/s
 */
static inline float ttalink_mav_hil_sim_state_get_yacc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Get field zacc from mav_hil_sim_state message
 *
 * @return Z acceleration m/s/s
 */
static inline float ttalink_mav_hil_sim_state_get_zacc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  36);
}

/**
 * @brief Get field xgyro from mav_hil_sim_state message
 *
 * @return Angular speed around X axis rad/s
 */
static inline float ttalink_mav_hil_sim_state_get_xgyro(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  40);
}

/**
 * @brief Get field ygyro from mav_hil_sim_state message
 *
 * @return Angular speed around Y axis rad/s
 */
static inline float ttalink_mav_hil_sim_state_get_ygyro(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  44);
}

/**
 * @brief Get field zgyro from mav_hil_sim_state message
 *
 * @return Angular speed around Z axis rad/s
 */
static inline float ttalink_mav_hil_sim_state_get_zgyro(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  48);
}

/**
 * @brief Get field lat from mav_hil_sim_state message
 *
 * @return Latitude in degrees
 */
static inline float ttalink_mav_hil_sim_state_get_lat(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  52);
}

/**
 * @brief Get field lon from mav_hil_sim_state message
 *
 * @return Longitude in degrees
 */
static inline float ttalink_mav_hil_sim_state_get_lon(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  56);
}

/**
 * @brief Get field alt from mav_hil_sim_state message
 *
 * @return Altitude in meters
 */
static inline float ttalink_mav_hil_sim_state_get_alt(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  60);
}

/**
 * @brief Get field std_dev_horz from mav_hil_sim_state message
 *
 * @return Horizontal position standard deviation
 */
static inline float ttalink_mav_hil_sim_state_get_std_dev_horz(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  64);
}

/**
 * @brief Get field std_dev_vert from mav_hil_sim_state message
 *
 * @return Vertical position standard deviation
 */
static inline float ttalink_mav_hil_sim_state_get_std_dev_vert(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  68);
}

/**
 * @brief Get field vn from mav_hil_sim_state message
 *
 * @return True velocity in m/s in NORTH direction in earth-fixed NED frame
 */
static inline float ttalink_mav_hil_sim_state_get_vn(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  72);
}

/**
 * @brief Get field ve from mav_hil_sim_state message
 *
 * @return True velocity in m/s in EAST direction in earth-fixed NED frame
 */
static inline float ttalink_mav_hil_sim_state_get_ve(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  76);
}

/**
 * @brief Get field vd from mav_hil_sim_state message
 *
 * @return True velocity in m/s in DOWN direction in earth-fixed NED frame
 */
static inline float ttalink_mav_hil_sim_state_get_vd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  80);
}

/**
 * @brief Decode a mav_hil_sim_state message into a struct
 *
 * @param msg The message to decode
 * @param mav_hil_sim_state C-struct to decode the message contents into
 */
static inline void ttalink_mav_hil_sim_state_decode(const ttalink_message_t* msg, ttalink_mav_hil_sim_state_t* mav_hil_sim_state)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    mav_hil_sim_state->q1 = ttalink_mav_hil_sim_state_get_q1(msg);
    mav_hil_sim_state->q2 = ttalink_mav_hil_sim_state_get_q2(msg);
    mav_hil_sim_state->q3 = ttalink_mav_hil_sim_state_get_q3(msg);
    mav_hil_sim_state->q4 = ttalink_mav_hil_sim_state_get_q4(msg);
    mav_hil_sim_state->roll = ttalink_mav_hil_sim_state_get_roll(msg);
    mav_hil_sim_state->pitch = ttalink_mav_hil_sim_state_get_pitch(msg);
    mav_hil_sim_state->yaw = ttalink_mav_hil_sim_state_get_yaw(msg);
    mav_hil_sim_state->xacc = ttalink_mav_hil_sim_state_get_xacc(msg);
    mav_hil_sim_state->yacc = ttalink_mav_hil_sim_state_get_yacc(msg);
    mav_hil_sim_state->zacc = ttalink_mav_hil_sim_state_get_zacc(msg);
    mav_hil_sim_state->xgyro = ttalink_mav_hil_sim_state_get_xgyro(msg);
    mav_hil_sim_state->ygyro = ttalink_mav_hil_sim_state_get_ygyro(msg);
    mav_hil_sim_state->zgyro = ttalink_mav_hil_sim_state_get_zgyro(msg);
    mav_hil_sim_state->lat = ttalink_mav_hil_sim_state_get_lat(msg);
    mav_hil_sim_state->lon = ttalink_mav_hil_sim_state_get_lon(msg);
    mav_hil_sim_state->alt = ttalink_mav_hil_sim_state_get_alt(msg);
    mav_hil_sim_state->std_dev_horz = ttalink_mav_hil_sim_state_get_std_dev_horz(msg);
    mav_hil_sim_state->std_dev_vert = ttalink_mav_hil_sim_state_get_std_dev_vert(msg);
    mav_hil_sim_state->vn = ttalink_mav_hil_sim_state_get_vn(msg);
    mav_hil_sim_state->ve = ttalink_mav_hil_sim_state_get_ve(msg);
    mav_hil_sim_state->vd = ttalink_mav_hil_sim_state_get_vd(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN? msg->len : TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN;
        memset(mav_hil_sim_state, 0, TTALINK_MSG_ID_MAV_HIL_SIM_STATE_LEN);
    memcpy(mav_hil_sim_state, _TTA_PAYLOAD(msg), len);
#endif
}
