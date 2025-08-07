#pragma once
// MESSAGE MAV_HIL_STATE_QUATERNION PACKING

#define TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION 8212

TTAPACKED(
typedef struct __ttalink_mav_hil_state_quaternion_t {
 uint64_t time_usec; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
 float attitude_quaternion[4]; /*< Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)*/
 float rollspeed; /*< Body frame roll / phi angular speed (rad/s)*/
 float pitchspeed; /*< Body frame pitch / theta angular speed (rad/s)*/
 float yawspeed; /*< Body frame yaw / psi angular speed (rad/s)*/
 int32_t lat; /*< Latitude, expressed as * 1E7*/
 int32_t lon; /*< Longitude, expressed as * 1E7*/
 int32_t alt; /*< Altitude in meters, expressed as * 1000 (millimeters)*/
 int16_t vx; /*< Ground X Speed (Latitude), expressed as m/s * 100*/
 int16_t vy; /*< Ground Y Speed (Longitude), expressed as m/s * 100*/
 int16_t vz; /*< Ground Z Speed (Altitude), expressed as m/s * 100*/
 uint16_t ind_airspeed; /*< Indicated airspeed, expressed as m/s * 100*/
 uint16_t true_airspeed; /*< True airspeed, expressed as m/s * 100*/
 int16_t xacc; /*< X acceleration (mg)*/
 int16_t yacc; /*< Y acceleration (mg)*/
 int16_t zacc; /*< Z acceleration (mg)*/
}) ttalink_mav_hil_state_quaternion_t;

#define TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN 64
#define TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_MIN_LEN 64
#define TTALINK_MSG_ID_8212_LEN 64
#define TTALINK_MSG_ID_8212_MIN_LEN 64

#define TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_CRC 196
#define TTALINK_MSG_ID_8212_CRC 196

#define TTALINK_MSG_MAV_HIL_STATE_QUATERNION_FIELD_ATTITUDE_QUATERNION_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_MAV_HIL_STATE_QUATERNION { \
    8212, \
    "MAV_HIL_STATE_QUATERNION", \
    16, \
    {  { "time_usec", NULL, TTALINK_TYPE_UINT64_T, 0, 0, offsetof(ttalink_mav_hil_state_quaternion_t, time_usec) }, \
         { "attitude_quaternion", NULL, TTALINK_TYPE_FLOAT, 4, 8, offsetof(ttalink_mav_hil_state_quaternion_t, attitude_quaternion) }, \
         { "rollspeed", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_mav_hil_state_quaternion_t, rollspeed) }, \
         { "pitchspeed", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_mav_hil_state_quaternion_t, pitchspeed) }, \
         { "yawspeed", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_mav_hil_state_quaternion_t, yawspeed) }, \
         { "lat", NULL, TTALINK_TYPE_INT32_T, 0, 36, offsetof(ttalink_mav_hil_state_quaternion_t, lat) }, \
         { "lon", NULL, TTALINK_TYPE_INT32_T, 0, 40, offsetof(ttalink_mav_hil_state_quaternion_t, lon) }, \
         { "alt", NULL, TTALINK_TYPE_INT32_T, 0, 44, offsetof(ttalink_mav_hil_state_quaternion_t, alt) }, \
         { "vx", NULL, TTALINK_TYPE_INT16_T, 0, 48, offsetof(ttalink_mav_hil_state_quaternion_t, vx) }, \
         { "vy", NULL, TTALINK_TYPE_INT16_T, 0, 50, offsetof(ttalink_mav_hil_state_quaternion_t, vy) }, \
         { "vz", NULL, TTALINK_TYPE_INT16_T, 0, 52, offsetof(ttalink_mav_hil_state_quaternion_t, vz) }, \
         { "ind_airspeed", NULL, TTALINK_TYPE_UINT16_T, 0, 54, offsetof(ttalink_mav_hil_state_quaternion_t, ind_airspeed) }, \
         { "true_airspeed", NULL, TTALINK_TYPE_UINT16_T, 0, 56, offsetof(ttalink_mav_hil_state_quaternion_t, true_airspeed) }, \
         { "xacc", NULL, TTALINK_TYPE_INT16_T, 0, 58, offsetof(ttalink_mav_hil_state_quaternion_t, xacc) }, \
         { "yacc", NULL, TTALINK_TYPE_INT16_T, 0, 60, offsetof(ttalink_mav_hil_state_quaternion_t, yacc) }, \
         { "zacc", NULL, TTALINK_TYPE_INT16_T, 0, 62, offsetof(ttalink_mav_hil_state_quaternion_t, zacc) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_MAV_HIL_STATE_QUATERNION { \
    "MAV_HIL_STATE_QUATERNION", \
    16, \
    {  { "time_usec", NULL, TTALINK_TYPE_UINT64_T, 0, 0, offsetof(ttalink_mav_hil_state_quaternion_t, time_usec) }, \
         { "attitude_quaternion", NULL, TTALINK_TYPE_FLOAT, 4, 8, offsetof(ttalink_mav_hil_state_quaternion_t, attitude_quaternion) }, \
         { "rollspeed", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_mav_hil_state_quaternion_t, rollspeed) }, \
         { "pitchspeed", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_mav_hil_state_quaternion_t, pitchspeed) }, \
         { "yawspeed", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_mav_hil_state_quaternion_t, yawspeed) }, \
         { "lat", NULL, TTALINK_TYPE_INT32_T, 0, 36, offsetof(ttalink_mav_hil_state_quaternion_t, lat) }, \
         { "lon", NULL, TTALINK_TYPE_INT32_T, 0, 40, offsetof(ttalink_mav_hil_state_quaternion_t, lon) }, \
         { "alt", NULL, TTALINK_TYPE_INT32_T, 0, 44, offsetof(ttalink_mav_hil_state_quaternion_t, alt) }, \
         { "vx", NULL, TTALINK_TYPE_INT16_T, 0, 48, offsetof(ttalink_mav_hil_state_quaternion_t, vx) }, \
         { "vy", NULL, TTALINK_TYPE_INT16_T, 0, 50, offsetof(ttalink_mav_hil_state_quaternion_t, vy) }, \
         { "vz", NULL, TTALINK_TYPE_INT16_T, 0, 52, offsetof(ttalink_mav_hil_state_quaternion_t, vz) }, \
         { "ind_airspeed", NULL, TTALINK_TYPE_UINT16_T, 0, 54, offsetof(ttalink_mav_hil_state_quaternion_t, ind_airspeed) }, \
         { "true_airspeed", NULL, TTALINK_TYPE_UINT16_T, 0, 56, offsetof(ttalink_mav_hil_state_quaternion_t, true_airspeed) }, \
         { "xacc", NULL, TTALINK_TYPE_INT16_T, 0, 58, offsetof(ttalink_mav_hil_state_quaternion_t, xacc) }, \
         { "yacc", NULL, TTALINK_TYPE_INT16_T, 0, 60, offsetof(ttalink_mav_hil_state_quaternion_t, yacc) }, \
         { "zacc", NULL, TTALINK_TYPE_INT16_T, 0, 62, offsetof(ttalink_mav_hil_state_quaternion_t, zacc) }, \
         } \
}
#endif


static inline uint16_t _ttalink_mav_hil_state_quaternion_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time_usec, const float *attitude_quaternion, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN];
    _tta_put_uint64_t(buf, 0, time_usec);
    _tta_put_float(buf, 24, rollspeed);
    _tta_put_float(buf, 28, pitchspeed);
    _tta_put_float(buf, 32, yawspeed);
    _tta_put_int32_t(buf, 36, lat);
    _tta_put_int32_t(buf, 40, lon);
    _tta_put_int32_t(buf, 44, alt);
    _tta_put_int16_t(buf, 48, vx);
    _tta_put_int16_t(buf, 50, vy);
    _tta_put_int16_t(buf, 52, vz);
    _tta_put_uint16_t(buf, 54, ind_airspeed);
    _tta_put_uint16_t(buf, 56, true_airspeed);
    _tta_put_int16_t(buf, 58, xacc);
    _tta_put_int16_t(buf, 60, yacc);
    _tta_put_int16_t(buf, 62, zacc);
    _tta_put_float_array(buf, 8, attitude_quaternion, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN);
#else
    ttalink_mav_hil_state_quaternion_t packet;
    packet.time_usec = time_usec;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.ind_airspeed = ind_airspeed;
    packet.true_airspeed = true_airspeed;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    tta_array_memcpy(packet.attitude_quaternion, attitude_quaternion, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_CRC, nocrc);
}

/**
 * @brief Pack a mav_hil_state_quaternion message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param attitude_quaternion Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
 * @param rollspeed Body frame roll / phi angular speed (rad/s)
 * @param pitchspeed Body frame pitch / theta angular speed (rad/s)
 * @param yawspeed Body frame yaw / psi angular speed (rad/s)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param ind_airspeed Indicated airspeed, expressed as m/s * 100
 * @param true_airspeed True airspeed, expressed as m/s * 100
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_state_quaternion_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time_usec, const float *attitude_quaternion, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc)
{
    return _ttalink_mav_hil_state_quaternion_pack(dst_addr, src_addr, msg,  time_usec, attitude_quaternion, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, ind_airspeed, true_airspeed, xacc, yacc, zacc, false);
}

/**
 * @brief Pack a mav_hil_state_quaternion message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param attitude_quaternion Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
 * @param rollspeed Body frame roll / phi angular speed (rad/s)
 * @param pitchspeed Body frame pitch / theta angular speed (rad/s)
 * @param yawspeed Body frame yaw / psi angular speed (rad/s)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param ind_airspeed Indicated airspeed, expressed as m/s * 100
 * @param true_airspeed True airspeed, expressed as m/s * 100
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_state_quaternion_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time_usec, const float *attitude_quaternion, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc)
{
    return _ttalink_mav_hil_state_quaternion_pack(dst_addr, src_addr, msg,  time_usec, attitude_quaternion, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, ind_airspeed, true_airspeed, xacc, yacc, zacc, true);
}


static inline uint16_t _ttalink_mav_hil_state_quaternion_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time_usec,const float *attitude_quaternion,float rollspeed,float pitchspeed,float yawspeed,int32_t lat,int32_t lon,int32_t alt,int16_t vx,int16_t vy,int16_t vz,uint16_t ind_airspeed,uint16_t true_airspeed,int16_t xacc,int16_t yacc,int16_t zacc, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN];
    _tta_put_uint64_t(buf, 0, time_usec);
    _tta_put_float(buf, 24, rollspeed);
    _tta_put_float(buf, 28, pitchspeed);
    _tta_put_float(buf, 32, yawspeed);
    _tta_put_int32_t(buf, 36, lat);
    _tta_put_int32_t(buf, 40, lon);
    _tta_put_int32_t(buf, 44, alt);
    _tta_put_int16_t(buf, 48, vx);
    _tta_put_int16_t(buf, 50, vy);
    _tta_put_int16_t(buf, 52, vz);
    _tta_put_uint16_t(buf, 54, ind_airspeed);
    _tta_put_uint16_t(buf, 56, true_airspeed);
    _tta_put_int16_t(buf, 58, xacc);
    _tta_put_int16_t(buf, 60, yacc);
    _tta_put_int16_t(buf, 62, zacc);
    _tta_put_float_array(buf, 8, attitude_quaternion, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN);
#else
    ttalink_mav_hil_state_quaternion_t packet;
    packet.time_usec = time_usec;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.ind_airspeed = ind_airspeed;
    packet.true_airspeed = true_airspeed;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    tta_array_memcpy(packet.attitude_quaternion, attitude_quaternion, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_CRC, nocrc);
}

/**
 * @brief Pack a mav_hil_state_quaternion message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param attitude_quaternion Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
 * @param rollspeed Body frame roll / phi angular speed (rad/s)
 * @param pitchspeed Body frame pitch / theta angular speed (rad/s)
 * @param yawspeed Body frame yaw / psi angular speed (rad/s)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param ind_airspeed Indicated airspeed, expressed as m/s * 100
 * @param true_airspeed True airspeed, expressed as m/s * 100
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_state_quaternion_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time_usec,const float *attitude_quaternion,float rollspeed,float pitchspeed,float yawspeed,int32_t lat,int32_t lon,int32_t alt,int16_t vx,int16_t vy,int16_t vz,uint16_t ind_airspeed,uint16_t true_airspeed,int16_t xacc,int16_t yacc,int16_t zacc)
{
    return _ttalink_mav_hil_state_quaternion_pack_chan(dst_addr, src_addr, chan, msg,  time_usec, attitude_quaternion, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, ind_airspeed, true_airspeed, xacc, yacc, zacc, false);
}

/**
 * @brief Pack a mav_hil_state_quaternion message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param attitude_quaternion Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
 * @param rollspeed Body frame roll / phi angular speed (rad/s)
 * @param pitchspeed Body frame pitch / theta angular speed (rad/s)
 * @param yawspeed Body frame yaw / psi angular speed (rad/s)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param ind_airspeed Indicated airspeed, expressed as m/s * 100
 * @param true_airspeed True airspeed, expressed as m/s * 100
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_state_quaternion_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time_usec,const float *attitude_quaternion,float rollspeed,float pitchspeed,float yawspeed,int32_t lat,int32_t lon,int32_t alt,int16_t vx,int16_t vy,int16_t vz,uint16_t ind_airspeed,uint16_t true_airspeed,int16_t xacc,int16_t yacc,int16_t zacc)
{
    return _ttalink_mav_hil_state_quaternion_pack_chan(dst_addr, src_addr, chan, msg,  time_usec, attitude_quaternion, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, ind_airspeed, true_airspeed, xacc, yacc, zacc, true);
}


static inline uint16_t _ttalink_mav_hil_state_quaternion_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_state_quaternion_t* mav_hil_state_quaternion, bool nocrc)
{
    if(nocrc){
        return ttalink_mav_hil_state_quaternion_pack_nocrc(dst_addr, src_addr, msg, mav_hil_state_quaternion->time_usec, mav_hil_state_quaternion->attitude_quaternion, mav_hil_state_quaternion->rollspeed, mav_hil_state_quaternion->pitchspeed, mav_hil_state_quaternion->yawspeed, mav_hil_state_quaternion->lat, mav_hil_state_quaternion->lon, mav_hil_state_quaternion->alt, mav_hil_state_quaternion->vx, mav_hil_state_quaternion->vy, mav_hil_state_quaternion->vz, mav_hil_state_quaternion->ind_airspeed, mav_hil_state_quaternion->true_airspeed, mav_hil_state_quaternion->xacc, mav_hil_state_quaternion->yacc, mav_hil_state_quaternion->zacc);
    }else{
        return ttalink_mav_hil_state_quaternion_pack(dst_addr, src_addr, msg, mav_hil_state_quaternion->time_usec, mav_hil_state_quaternion->attitude_quaternion, mav_hil_state_quaternion->rollspeed, mav_hil_state_quaternion->pitchspeed, mav_hil_state_quaternion->yawspeed, mav_hil_state_quaternion->lat, mav_hil_state_quaternion->lon, mav_hil_state_quaternion->alt, mav_hil_state_quaternion->vx, mav_hil_state_quaternion->vy, mav_hil_state_quaternion->vz, mav_hil_state_quaternion->ind_airspeed, mav_hil_state_quaternion->true_airspeed, mav_hil_state_quaternion->xacc, mav_hil_state_quaternion->yacc, mav_hil_state_quaternion->zacc);
    }
    
}

/**
 * @brief Encode a mav_hil_state_quaternion struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_state_quaternion C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_state_quaternion_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_state_quaternion_t* mav_hil_state_quaternion)
{
    return _ttalink_mav_hil_state_quaternion_encode(dst_addr, src_addr, msg, mav_hil_state_quaternion, false);
}

/**
 * @brief Encode a mav_hil_state_quaternion struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_state_quaternion C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_state_quaternion_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_state_quaternion_t* mav_hil_state_quaternion)
{
    return _ttalink_mav_hil_state_quaternion_encode(dst_addr, src_addr, msg, mav_hil_state_quaternion, true);
}


static inline uint16_t _ttalink_mav_hil_state_quaternion_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_state_quaternion_t* mav_hil_state_quaternion, bool nocrc)
{
    if(nocrc){
        return ttalink_mav_hil_state_quaternion_pack_chan_nocrc(dst_addr, src_addr, chan, msg, mav_hil_state_quaternion->time_usec, mav_hil_state_quaternion->attitude_quaternion, mav_hil_state_quaternion->rollspeed, mav_hil_state_quaternion->pitchspeed, mav_hil_state_quaternion->yawspeed, mav_hil_state_quaternion->lat, mav_hil_state_quaternion->lon, mav_hil_state_quaternion->alt, mav_hil_state_quaternion->vx, mav_hil_state_quaternion->vy, mav_hil_state_quaternion->vz, mav_hil_state_quaternion->ind_airspeed, mav_hil_state_quaternion->true_airspeed, mav_hil_state_quaternion->xacc, mav_hil_state_quaternion->yacc, mav_hil_state_quaternion->zacc);
    }else{
        return ttalink_mav_hil_state_quaternion_pack_chan(dst_addr, src_addr, chan, msg, mav_hil_state_quaternion->time_usec, mav_hil_state_quaternion->attitude_quaternion, mav_hil_state_quaternion->rollspeed, mav_hil_state_quaternion->pitchspeed, mav_hil_state_quaternion->yawspeed, mav_hil_state_quaternion->lat, mav_hil_state_quaternion->lon, mav_hil_state_quaternion->alt, mav_hil_state_quaternion->vx, mav_hil_state_quaternion->vy, mav_hil_state_quaternion->vz, mav_hil_state_quaternion->ind_airspeed, mav_hil_state_quaternion->true_airspeed, mav_hil_state_quaternion->xacc, mav_hil_state_quaternion->yacc, mav_hil_state_quaternion->zacc);
    }
}

/**
 * @brief Encode a mav_hil_state_quaternion struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_state_quaternion C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_state_quaternion_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_state_quaternion_t* mav_hil_state_quaternion)
{
    return _ttalink_mav_hil_state_quaternion_encode_chan(dst_addr, src_addr, chan, msg, mav_hil_state_quaternion, false);
}

/**
 * @brief Encode a mav_hil_state_quaternion struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_state_quaternion C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_state_quaternion_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_state_quaternion_t* mav_hil_state_quaternion)
{
    return _ttalink_mav_hil_state_quaternion_encode_chan(dst_addr, src_addr, chan, msg, mav_hil_state_quaternion, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_mav_hil_state_quaternion_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time_usec, const float *attitude_quaternion, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN];
    _tta_put_uint64_t(buf, 0, time_usec);
    _tta_put_float(buf, 24, rollspeed);
    _tta_put_float(buf, 28, pitchspeed);
    _tta_put_float(buf, 32, yawspeed);
    _tta_put_int32_t(buf, 36, lat);
    _tta_put_int32_t(buf, 40, lon);
    _tta_put_int32_t(buf, 44, alt);
    _tta_put_int16_t(buf, 48, vx);
    _tta_put_int16_t(buf, 50, vy);
    _tta_put_int16_t(buf, 52, vz);
    _tta_put_uint16_t(buf, 54, ind_airspeed);
    _tta_put_uint16_t(buf, 56, true_airspeed);
    _tta_put_int16_t(buf, 58, xacc);
    _tta_put_int16_t(buf, 60, yacc);
    _tta_put_int16_t(buf, 62, zacc);
    _tta_put_float_array(buf, 8, attitude_quaternion, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION, buf, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_CRC, nocrc);
#else
    ttalink_mav_hil_state_quaternion_t packet;
    packet.time_usec = time_usec;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.ind_airspeed = ind_airspeed;
    packet.true_airspeed = true_airspeed;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    tta_array_memcpy(packet.attitude_quaternion, attitude_quaternion, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION, (const char *)&packet, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_CRC, nocrc);
#endif
}

/**
 * @brief Send a mav_hil_state_quaternion message
 * @param chan TTAlink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param attitude_quaternion Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
 * @param rollspeed Body frame roll / phi angular speed (rad/s)
 * @param pitchspeed Body frame pitch / theta angular speed (rad/s)
 * @param yawspeed Body frame yaw / psi angular speed (rad/s)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param ind_airspeed Indicated airspeed, expressed as m/s * 100
 * @param true_airspeed True airspeed, expressed as m/s * 100
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 */
static inline void ttalink_mav_hil_state_quaternion_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time_usec, const float *attitude_quaternion, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc)
{
    _ttalink_mav_hil_state_quaternion_send(dst_addr, src_addr, chan, time_usec, attitude_quaternion, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, ind_airspeed, true_airspeed, xacc, yacc, zacc, false);
}

/**
 * @brief Send a mav_hil_state_quaternion message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param attitude_quaternion Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
 * @param rollspeed Body frame roll / phi angular speed (rad/s)
 * @param pitchspeed Body frame pitch / theta angular speed (rad/s)
 * @param yawspeed Body frame yaw / psi angular speed (rad/s)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param ind_airspeed Indicated airspeed, expressed as m/s * 100
 * @param true_airspeed True airspeed, expressed as m/s * 100
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 */
static inline void ttalink_mav_hil_state_quaternion_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time_usec, const float *attitude_quaternion, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc)
{
    _ttalink_mav_hil_state_quaternion_send(dst_addr, src_addr, chan, time_usec, attitude_quaternion, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, ind_airspeed, true_airspeed, xacc, yacc, zacc, true);
}


static inline void _ttalink_mav_hil_state_quaternion_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_state_quaternion_t* mav_hil_state_quaternion, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_mav_hil_state_quaternion_send_nocrc(dst_addr, src_addr, chan, mav_hil_state_quaternion->time_usec, mav_hil_state_quaternion->attitude_quaternion, mav_hil_state_quaternion->rollspeed, mav_hil_state_quaternion->pitchspeed, mav_hil_state_quaternion->yawspeed, mav_hil_state_quaternion->lat, mav_hil_state_quaternion->lon, mav_hil_state_quaternion->alt, mav_hil_state_quaternion->vx, mav_hil_state_quaternion->vy, mav_hil_state_quaternion->vz, mav_hil_state_quaternion->ind_airspeed, mav_hil_state_quaternion->true_airspeed, mav_hil_state_quaternion->xacc, mav_hil_state_quaternion->yacc, mav_hil_state_quaternion->zacc);
    }else{
        ttalink_mav_hil_state_quaternion_send(dst_addr, src_addr, chan, mav_hil_state_quaternion->time_usec, mav_hil_state_quaternion->attitude_quaternion, mav_hil_state_quaternion->rollspeed, mav_hil_state_quaternion->pitchspeed, mav_hil_state_quaternion->yawspeed, mav_hil_state_quaternion->lat, mav_hil_state_quaternion->lon, mav_hil_state_quaternion->alt, mav_hil_state_quaternion->vx, mav_hil_state_quaternion->vy, mav_hil_state_quaternion->vz, mav_hil_state_quaternion->ind_airspeed, mav_hil_state_quaternion->true_airspeed, mav_hil_state_quaternion->xacc, mav_hil_state_quaternion->yacc, mav_hil_state_quaternion->zacc);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION, (const char *)mav_hil_state_quaternion, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_CRC, nocrc);
#endif
}

/**
 * @brief Send a mav_hil_state_quaternion message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_mav_hil_state_quaternion_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_state_quaternion_t* mav_hil_state_quaternion)
{
    _ttalink_mav_hil_state_quaternion_send_struct(dst_addr, src_addr, chan, mav_hil_state_quaternion, false);
}

/**
 * @brief Send a mav_hil_state_quaternion message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_mav_hil_state_quaternion_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_state_quaternion_t* mav_hil_state_quaternion)
{
    _ttalink_mav_hil_state_quaternion_send_struct(dst_addr, src_addr, chan, mav_hil_state_quaternion, true);
}

#if TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_mav_hil_state_quaternion_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time_usec, const float *attitude_quaternion, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint64_t(buf, 0, time_usec);
    _tta_put_float(buf, 24, rollspeed);
    _tta_put_float(buf, 28, pitchspeed);
    _tta_put_float(buf, 32, yawspeed);
    _tta_put_int32_t(buf, 36, lat);
    _tta_put_int32_t(buf, 40, lon);
    _tta_put_int32_t(buf, 44, alt);
    _tta_put_int16_t(buf, 48, vx);
    _tta_put_int16_t(buf, 50, vy);
    _tta_put_int16_t(buf, 52, vz);
    _tta_put_uint16_t(buf, 54, ind_airspeed);
    _tta_put_uint16_t(buf, 56, true_airspeed);
    _tta_put_int16_t(buf, 58, xacc);
    _tta_put_int16_t(buf, 60, yacc);
    _tta_put_int16_t(buf, 62, zacc);
    _tta_put_float_array(buf, 8, attitude_quaternion, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION, buf, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_CRC, nocrc);
#else
    ttalink_mav_hil_state_quaternion_t *packet = (ttalink_mav_hil_state_quaternion_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->rollspeed = rollspeed;
    packet->pitchspeed = pitchspeed;
    packet->yawspeed = yawspeed;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->ind_airspeed = ind_airspeed;
    packet->true_airspeed = true_airspeed;
    packet->xacc = xacc;
    packet->yacc = yacc;
    packet->zacc = zacc;
    tta_array_memcpy(packet->attitude_quaternion, attitude_quaternion, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION, (const char *)packet, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_mav_hil_state_quaternion_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time_usec, const float *attitude_quaternion, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc)
{
    _ttalink_mav_hil_state_quaternion_send_buf(dst_addr, src_addr, msgbuf, chan, time_usec, attitude_quaternion, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, ind_airspeed, true_airspeed, xacc, yacc, zacc, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_mav_hil_state_quaternion_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time_usec, const float *attitude_quaternion, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc)
{
    _ttalink_mav_hil_state_quaternion_send_buf(dst_addr, src_addr, msgbuf, chan, time_usec, attitude_quaternion, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, ind_airspeed, true_airspeed, xacc, yacc, zacc, true);
}
#endif

#endif

// MESSAGE MAV_HIL_STATE_QUATERNION UNPACKING


/**
 * @brief Get field time_usec from mav_hil_state_quaternion message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t ttalink_mav_hil_state_quaternion_get_time_usec(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field attitude_quaternion from mav_hil_state_quaternion message
 *
 * @return Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
 */
static inline uint16_t ttalink_mav_hil_state_quaternion_get_attitude_quaternion(const ttalink_message_t* msg, float *attitude_quaternion)
{
    return _TTA_RETURN_float_array(msg, attitude_quaternion, 4,  8);
}

/**
 * @brief Get field rollspeed from mav_hil_state_quaternion message
 *
 * @return Body frame roll / phi angular speed (rad/s)
 */
static inline float ttalink_mav_hil_state_quaternion_get_rollspeed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Get field pitchspeed from mav_hil_state_quaternion message
 *
 * @return Body frame pitch / theta angular speed (rad/s)
 */
static inline float ttalink_mav_hil_state_quaternion_get_pitchspeed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  28);
}

/**
 * @brief Get field yawspeed from mav_hil_state_quaternion message
 *
 * @return Body frame yaw / psi angular speed (rad/s)
 */
static inline float ttalink_mav_hil_state_quaternion_get_yawspeed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Get field lat from mav_hil_state_quaternion message
 *
 * @return Latitude, expressed as * 1E7
 */
static inline int32_t ttalink_mav_hil_state_quaternion_get_lat(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  36);
}

/**
 * @brief Get field lon from mav_hil_state_quaternion message
 *
 * @return Longitude, expressed as * 1E7
 */
static inline int32_t ttalink_mav_hil_state_quaternion_get_lon(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  40);
}

/**
 * @brief Get field alt from mav_hil_state_quaternion message
 *
 * @return Altitude in meters, expressed as * 1000 (millimeters)
 */
static inline int32_t ttalink_mav_hil_state_quaternion_get_alt(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  44);
}

/**
 * @brief Get field vx from mav_hil_state_quaternion message
 *
 * @return Ground X Speed (Latitude), expressed as m/s * 100
 */
static inline int16_t ttalink_mav_hil_state_quaternion_get_vx(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  48);
}

/**
 * @brief Get field vy from mav_hil_state_quaternion message
 *
 * @return Ground Y Speed (Longitude), expressed as m/s * 100
 */
static inline int16_t ttalink_mav_hil_state_quaternion_get_vy(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  50);
}

/**
 * @brief Get field vz from mav_hil_state_quaternion message
 *
 * @return Ground Z Speed (Altitude), expressed as m/s * 100
 */
static inline int16_t ttalink_mav_hil_state_quaternion_get_vz(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  52);
}

/**
 * @brief Get field ind_airspeed from mav_hil_state_quaternion message
 *
 * @return Indicated airspeed, expressed as m/s * 100
 */
static inline uint16_t ttalink_mav_hil_state_quaternion_get_ind_airspeed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  54);
}

/**
 * @brief Get field true_airspeed from mav_hil_state_quaternion message
 *
 * @return True airspeed, expressed as m/s * 100
 */
static inline uint16_t ttalink_mav_hil_state_quaternion_get_true_airspeed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  56);
}

/**
 * @brief Get field xacc from mav_hil_state_quaternion message
 *
 * @return X acceleration (mg)
 */
static inline int16_t ttalink_mav_hil_state_quaternion_get_xacc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  58);
}

/**
 * @brief Get field yacc from mav_hil_state_quaternion message
 *
 * @return Y acceleration (mg)
 */
static inline int16_t ttalink_mav_hil_state_quaternion_get_yacc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  60);
}

/**
 * @brief Get field zacc from mav_hil_state_quaternion message
 *
 * @return Z acceleration (mg)
 */
static inline int16_t ttalink_mav_hil_state_quaternion_get_zacc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  62);
}

/**
 * @brief Decode a mav_hil_state_quaternion message into a struct
 *
 * @param msg The message to decode
 * @param mav_hil_state_quaternion C-struct to decode the message contents into
 */
static inline void ttalink_mav_hil_state_quaternion_decode(const ttalink_message_t* msg, ttalink_mav_hil_state_quaternion_t* mav_hil_state_quaternion)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    mav_hil_state_quaternion->time_usec = ttalink_mav_hil_state_quaternion_get_time_usec(msg);
    ttalink_mav_hil_state_quaternion_get_attitude_quaternion(msg, mav_hil_state_quaternion->attitude_quaternion);
    mav_hil_state_quaternion->rollspeed = ttalink_mav_hil_state_quaternion_get_rollspeed(msg);
    mav_hil_state_quaternion->pitchspeed = ttalink_mav_hil_state_quaternion_get_pitchspeed(msg);
    mav_hil_state_quaternion->yawspeed = ttalink_mav_hil_state_quaternion_get_yawspeed(msg);
    mav_hil_state_quaternion->lat = ttalink_mav_hil_state_quaternion_get_lat(msg);
    mav_hil_state_quaternion->lon = ttalink_mav_hil_state_quaternion_get_lon(msg);
    mav_hil_state_quaternion->alt = ttalink_mav_hil_state_quaternion_get_alt(msg);
    mav_hil_state_quaternion->vx = ttalink_mav_hil_state_quaternion_get_vx(msg);
    mav_hil_state_quaternion->vy = ttalink_mav_hil_state_quaternion_get_vy(msg);
    mav_hil_state_quaternion->vz = ttalink_mav_hil_state_quaternion_get_vz(msg);
    mav_hil_state_quaternion->ind_airspeed = ttalink_mav_hil_state_quaternion_get_ind_airspeed(msg);
    mav_hil_state_quaternion->true_airspeed = ttalink_mav_hil_state_quaternion_get_true_airspeed(msg);
    mav_hil_state_quaternion->xacc = ttalink_mav_hil_state_quaternion_get_xacc(msg);
    mav_hil_state_quaternion->yacc = ttalink_mav_hil_state_quaternion_get_yacc(msg);
    mav_hil_state_quaternion->zacc = ttalink_mav_hil_state_quaternion_get_zacc(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN? msg->len : TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN;
        memset(mav_hil_state_quaternion, 0, TTALINK_MSG_ID_MAV_HIL_STATE_QUATERNION_LEN);
    memcpy(mav_hil_state_quaternion, _TTA_PAYLOAD(msg), len);
#endif
}
