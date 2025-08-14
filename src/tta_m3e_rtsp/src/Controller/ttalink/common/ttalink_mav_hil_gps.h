#pragma once
// MESSAGE MAV_HIL_GPS PACKING

#define TTALINK_MSG_ID_MAV_HIL_GPS 8210

TTAPACKED(
typedef struct __ttalink_mav_hil_gps_t {
 uint64_t time_usec; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
 uint8_t fix_type; /*< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.*/
 int32_t lat; /*< Latitude (WGS84), in degrees * 1E7*/
 int32_t lon; /*< Longitude (WGS84), in degrees * 1E7*/
 int32_t alt; /*< Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)*/
 uint16_t eph; /*< GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535*/
 uint16_t epv; /*< GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535*/
 uint16_t vel; /*< GPS ground speed (m/s * 100). If unknown, set to: 65535*/
 int16_t vn; /*< GPS velocity in cm/s in NORTH direction in earth-fixed NED frame*/
 int16_t ve; /*< GPS velocity in cm/s in EAST direction in earth-fixed NED frame*/
 int16_t vd; /*< GPS velocity in cm/s in DOWN direction in earth-fixed NED frame*/
 uint16_t cog; /*< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535*/
 uint8_t satellites_visible; /*< Number of satellites visible. If unknown, set to 255*/
}) ttalink_mav_hil_gps_t;

#define TTALINK_MSG_ID_MAV_HIL_GPS_LEN 36
#define TTALINK_MSG_ID_MAV_HIL_GPS_MIN_LEN 36
#define TTALINK_MSG_ID_8210_LEN 36
#define TTALINK_MSG_ID_8210_MIN_LEN 36

#define TTALINK_MSG_ID_MAV_HIL_GPS_CRC 28
#define TTALINK_MSG_ID_8210_CRC 28



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_MAV_HIL_GPS { \
    8210, \
    "MAV_HIL_GPS", \
    13, \
    {  { "time_usec", NULL, TTALINK_TYPE_UINT64_T, 0, 0, offsetof(ttalink_mav_hil_gps_t, time_usec) }, \
         { "fix_type", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_mav_hil_gps_t, fix_type) }, \
         { "lat", NULL, TTALINK_TYPE_INT32_T, 0, 9, offsetof(ttalink_mav_hil_gps_t, lat) }, \
         { "lon", NULL, TTALINK_TYPE_INT32_T, 0, 13, offsetof(ttalink_mav_hil_gps_t, lon) }, \
         { "alt", NULL, TTALINK_TYPE_INT32_T, 0, 17, offsetof(ttalink_mav_hil_gps_t, alt) }, \
         { "eph", NULL, TTALINK_TYPE_UINT16_T, 0, 21, offsetof(ttalink_mav_hil_gps_t, eph) }, \
         { "epv", NULL, TTALINK_TYPE_UINT16_T, 0, 23, offsetof(ttalink_mav_hil_gps_t, epv) }, \
         { "vel", NULL, TTALINK_TYPE_UINT16_T, 0, 25, offsetof(ttalink_mav_hil_gps_t, vel) }, \
         { "vn", NULL, TTALINK_TYPE_INT16_T, 0, 27, offsetof(ttalink_mav_hil_gps_t, vn) }, \
         { "ve", NULL, TTALINK_TYPE_INT16_T, 0, 29, offsetof(ttalink_mav_hil_gps_t, ve) }, \
         { "vd", NULL, TTALINK_TYPE_INT16_T, 0, 31, offsetof(ttalink_mav_hil_gps_t, vd) }, \
         { "cog", NULL, TTALINK_TYPE_UINT16_T, 0, 33, offsetof(ttalink_mav_hil_gps_t, cog) }, \
         { "satellites_visible", NULL, TTALINK_TYPE_UINT8_T, 0, 35, offsetof(ttalink_mav_hil_gps_t, satellites_visible) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_MAV_HIL_GPS { \
    "MAV_HIL_GPS", \
    13, \
    {  { "time_usec", NULL, TTALINK_TYPE_UINT64_T, 0, 0, offsetof(ttalink_mav_hil_gps_t, time_usec) }, \
         { "fix_type", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_mav_hil_gps_t, fix_type) }, \
         { "lat", NULL, TTALINK_TYPE_INT32_T, 0, 9, offsetof(ttalink_mav_hil_gps_t, lat) }, \
         { "lon", NULL, TTALINK_TYPE_INT32_T, 0, 13, offsetof(ttalink_mav_hil_gps_t, lon) }, \
         { "alt", NULL, TTALINK_TYPE_INT32_T, 0, 17, offsetof(ttalink_mav_hil_gps_t, alt) }, \
         { "eph", NULL, TTALINK_TYPE_UINT16_T, 0, 21, offsetof(ttalink_mav_hil_gps_t, eph) }, \
         { "epv", NULL, TTALINK_TYPE_UINT16_T, 0, 23, offsetof(ttalink_mav_hil_gps_t, epv) }, \
         { "vel", NULL, TTALINK_TYPE_UINT16_T, 0, 25, offsetof(ttalink_mav_hil_gps_t, vel) }, \
         { "vn", NULL, TTALINK_TYPE_INT16_T, 0, 27, offsetof(ttalink_mav_hil_gps_t, vn) }, \
         { "ve", NULL, TTALINK_TYPE_INT16_T, 0, 29, offsetof(ttalink_mav_hil_gps_t, ve) }, \
         { "vd", NULL, TTALINK_TYPE_INT16_T, 0, 31, offsetof(ttalink_mav_hil_gps_t, vd) }, \
         { "cog", NULL, TTALINK_TYPE_UINT16_T, 0, 33, offsetof(ttalink_mav_hil_gps_t, cog) }, \
         { "satellites_visible", NULL, TTALINK_TYPE_UINT8_T, 0, 35, offsetof(ttalink_mav_hil_gps_t, satellites_visible) }, \
         } \
}
#endif


static inline uint16_t _ttalink_mav_hil_gps_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satellites_visible, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_GPS_LEN];
    _tta_put_uint64_t(buf, 0, time_usec);
    _tta_put_uint8_t(buf, 8, fix_type);
    _tta_put_int32_t(buf, 9, lat);
    _tta_put_int32_t(buf, 13, lon);
    _tta_put_int32_t(buf, 17, alt);
    _tta_put_uint16_t(buf, 21, eph);
    _tta_put_uint16_t(buf, 23, epv);
    _tta_put_uint16_t(buf, 25, vel);
    _tta_put_int16_t(buf, 27, vn);
    _tta_put_int16_t(buf, 29, ve);
    _tta_put_int16_t(buf, 31, vd);
    _tta_put_uint16_t(buf, 33, cog);
    _tta_put_uint8_t(buf, 35, satellites_visible);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MAV_HIL_GPS_LEN);
#else
    ttalink_mav_hil_gps_t packet;
    packet.time_usec = time_usec;
    packet.fix_type = fix_type;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.vn = vn;
    packet.ve = ve;
    packet.vd = vd;
    packet.cog = cog;
    packet.satellites_visible = satellites_visible;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MAV_HIL_GPS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MAV_HIL_GPS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_MAV_HIL_GPS_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_GPS_LEN, TTALINK_MSG_ID_MAV_HIL_GPS_CRC, nocrc);
}

/**
 * @brief Pack a mav_hil_gps message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param epv GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: 65535
 * @param vn GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
 * @param ve GPS velocity in cm/s in EAST direction in earth-fixed NED frame
 * @param vd GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_gps_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satellites_visible)
{
    return _ttalink_mav_hil_gps_pack(dst_addr, src_addr, msg,  time_usec, fix_type, lat, lon, alt, eph, epv, vel, vn, ve, vd, cog, satellites_visible, false);
}

/**
 * @brief Pack a mav_hil_gps message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param epv GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: 65535
 * @param vn GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
 * @param ve GPS velocity in cm/s in EAST direction in earth-fixed NED frame
 * @param vd GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_gps_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satellites_visible)
{
    return _ttalink_mav_hil_gps_pack(dst_addr, src_addr, msg,  time_usec, fix_type, lat, lon, alt, eph, epv, vel, vn, ve, vd, cog, satellites_visible, true);
}


static inline uint16_t _ttalink_mav_hil_gps_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time_usec,uint8_t fix_type,int32_t lat,int32_t lon,int32_t alt,uint16_t eph,uint16_t epv,uint16_t vel,int16_t vn,int16_t ve,int16_t vd,uint16_t cog,uint8_t satellites_visible, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_GPS_LEN];
    _tta_put_uint64_t(buf, 0, time_usec);
    _tta_put_uint8_t(buf, 8, fix_type);
    _tta_put_int32_t(buf, 9, lat);
    _tta_put_int32_t(buf, 13, lon);
    _tta_put_int32_t(buf, 17, alt);
    _tta_put_uint16_t(buf, 21, eph);
    _tta_put_uint16_t(buf, 23, epv);
    _tta_put_uint16_t(buf, 25, vel);
    _tta_put_int16_t(buf, 27, vn);
    _tta_put_int16_t(buf, 29, ve);
    _tta_put_int16_t(buf, 31, vd);
    _tta_put_uint16_t(buf, 33, cog);
    _tta_put_uint8_t(buf, 35, satellites_visible);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MAV_HIL_GPS_LEN);
#else
    ttalink_mav_hil_gps_t packet;
    packet.time_usec = time_usec;
    packet.fix_type = fix_type;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.vn = vn;
    packet.ve = ve;
    packet.vd = vd;
    packet.cog = cog;
    packet.satellites_visible = satellites_visible;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MAV_HIL_GPS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MAV_HIL_GPS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_GPS_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_GPS_LEN, TTALINK_MSG_ID_MAV_HIL_GPS_CRC, nocrc);
}

/**
 * @brief Pack a mav_hil_gps message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param epv GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: 65535
 * @param vn GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
 * @param ve GPS velocity in cm/s in EAST direction in earth-fixed NED frame
 * @param vd GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_gps_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time_usec,uint8_t fix_type,int32_t lat,int32_t lon,int32_t alt,uint16_t eph,uint16_t epv,uint16_t vel,int16_t vn,int16_t ve,int16_t vd,uint16_t cog,uint8_t satellites_visible)
{
    return _ttalink_mav_hil_gps_pack_chan(dst_addr, src_addr, chan, msg,  time_usec, fix_type, lat, lon, alt, eph, epv, vel, vn, ve, vd, cog, satellites_visible, false);
}

/**
 * @brief Pack a mav_hil_gps message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param epv GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: 65535
 * @param vn GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
 * @param ve GPS velocity in cm/s in EAST direction in earth-fixed NED frame
 * @param vd GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_gps_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time_usec,uint8_t fix_type,int32_t lat,int32_t lon,int32_t alt,uint16_t eph,uint16_t epv,uint16_t vel,int16_t vn,int16_t ve,int16_t vd,uint16_t cog,uint8_t satellites_visible)
{
    return _ttalink_mav_hil_gps_pack_chan(dst_addr, src_addr, chan, msg,  time_usec, fix_type, lat, lon, alt, eph, epv, vel, vn, ve, vd, cog, satellites_visible, true);
}


static inline uint16_t _ttalink_mav_hil_gps_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_gps_t* mav_hil_gps, bool nocrc)
{
    if(nocrc){
        return ttalink_mav_hil_gps_pack_nocrc(dst_addr, src_addr, msg, mav_hil_gps->time_usec, mav_hil_gps->fix_type, mav_hil_gps->lat, mav_hil_gps->lon, mav_hil_gps->alt, mav_hil_gps->eph, mav_hil_gps->epv, mav_hil_gps->vel, mav_hil_gps->vn, mav_hil_gps->ve, mav_hil_gps->vd, mav_hil_gps->cog, mav_hil_gps->satellites_visible);
    }else{
        return ttalink_mav_hil_gps_pack(dst_addr, src_addr, msg, mav_hil_gps->time_usec, mav_hil_gps->fix_type, mav_hil_gps->lat, mav_hil_gps->lon, mav_hil_gps->alt, mav_hil_gps->eph, mav_hil_gps->epv, mav_hil_gps->vel, mav_hil_gps->vn, mav_hil_gps->ve, mav_hil_gps->vd, mav_hil_gps->cog, mav_hil_gps->satellites_visible);
    }
    
}

/**
 * @brief Encode a mav_hil_gps struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_gps C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_gps_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_gps_t* mav_hil_gps)
{
    return _ttalink_mav_hil_gps_encode(dst_addr, src_addr, msg, mav_hil_gps, false);
}

/**
 * @brief Encode a mav_hil_gps struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_gps C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_gps_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_gps_t* mav_hil_gps)
{
    return _ttalink_mav_hil_gps_encode(dst_addr, src_addr, msg, mav_hil_gps, true);
}


static inline uint16_t _ttalink_mav_hil_gps_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_gps_t* mav_hil_gps, bool nocrc)
{
    if(nocrc){
        return ttalink_mav_hil_gps_pack_chan_nocrc(dst_addr, src_addr, chan, msg, mav_hil_gps->time_usec, mav_hil_gps->fix_type, mav_hil_gps->lat, mav_hil_gps->lon, mav_hil_gps->alt, mav_hil_gps->eph, mav_hil_gps->epv, mav_hil_gps->vel, mav_hil_gps->vn, mav_hil_gps->ve, mav_hil_gps->vd, mav_hil_gps->cog, mav_hil_gps->satellites_visible);
    }else{
        return ttalink_mav_hil_gps_pack_chan(dst_addr, src_addr, chan, msg, mav_hil_gps->time_usec, mav_hil_gps->fix_type, mav_hil_gps->lat, mav_hil_gps->lon, mav_hil_gps->alt, mav_hil_gps->eph, mav_hil_gps->epv, mav_hil_gps->vel, mav_hil_gps->vn, mav_hil_gps->ve, mav_hil_gps->vd, mav_hil_gps->cog, mav_hil_gps->satellites_visible);
    }
}

/**
 * @brief Encode a mav_hil_gps struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_gps C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_gps_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_gps_t* mav_hil_gps)
{
    return _ttalink_mav_hil_gps_encode_chan(dst_addr, src_addr, chan, msg, mav_hil_gps, false);
}

/**
 * @brief Encode a mav_hil_gps struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_gps C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_gps_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_gps_t* mav_hil_gps)
{
    return _ttalink_mav_hil_gps_encode_chan(dst_addr, src_addr, chan, msg, mav_hil_gps, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_mav_hil_gps_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satellites_visible, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_GPS_LEN];
    _tta_put_uint64_t(buf, 0, time_usec);
    _tta_put_uint8_t(buf, 8, fix_type);
    _tta_put_int32_t(buf, 9, lat);
    _tta_put_int32_t(buf, 13, lon);
    _tta_put_int32_t(buf, 17, alt);
    _tta_put_uint16_t(buf, 21, eph);
    _tta_put_uint16_t(buf, 23, epv);
    _tta_put_uint16_t(buf, 25, vel);
    _tta_put_int16_t(buf, 27, vn);
    _tta_put_int16_t(buf, 29, ve);
    _tta_put_int16_t(buf, 31, vd);
    _tta_put_uint16_t(buf, 33, cog);
    _tta_put_uint8_t(buf, 35, satellites_visible);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_GPS, buf, TTALINK_MSG_ID_MAV_HIL_GPS_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_GPS_LEN, TTALINK_MSG_ID_MAV_HIL_GPS_CRC, nocrc);
#else
    ttalink_mav_hil_gps_t packet;
    packet.time_usec = time_usec;
    packet.fix_type = fix_type;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.vn = vn;
    packet.ve = ve;
    packet.vd = vd;
    packet.cog = cog;
    packet.satellites_visible = satellites_visible;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_GPS, (const char *)&packet, TTALINK_MSG_ID_MAV_HIL_GPS_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_GPS_LEN, TTALINK_MSG_ID_MAV_HIL_GPS_CRC, nocrc);
#endif
}

/**
 * @brief Send a mav_hil_gps message
 * @param chan TTAlink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param epv GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: 65535
 * @param vn GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
 * @param ve GPS velocity in cm/s in EAST direction in earth-fixed NED frame
 * @param vd GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 */
static inline void ttalink_mav_hil_gps_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satellites_visible)
{
    _ttalink_mav_hil_gps_send(dst_addr, src_addr, chan, time_usec, fix_type, lat, lon, alt, eph, epv, vel, vn, ve, vd, cog, satellites_visible, false);
}

/**
 * @brief Send a mav_hil_gps message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param epv GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: 65535
 * @param vn GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
 * @param ve GPS velocity in cm/s in EAST direction in earth-fixed NED frame
 * @param vd GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 */
static inline void ttalink_mav_hil_gps_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satellites_visible)
{
    _ttalink_mav_hil_gps_send(dst_addr, src_addr, chan, time_usec, fix_type, lat, lon, alt, eph, epv, vel, vn, ve, vd, cog, satellites_visible, true);
}


static inline void _ttalink_mav_hil_gps_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_gps_t* mav_hil_gps, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_mav_hil_gps_send_nocrc(dst_addr, src_addr, chan, mav_hil_gps->time_usec, mav_hil_gps->fix_type, mav_hil_gps->lat, mav_hil_gps->lon, mav_hil_gps->alt, mav_hil_gps->eph, mav_hil_gps->epv, mav_hil_gps->vel, mav_hil_gps->vn, mav_hil_gps->ve, mav_hil_gps->vd, mav_hil_gps->cog, mav_hil_gps->satellites_visible);
    }else{
        ttalink_mav_hil_gps_send(dst_addr, src_addr, chan, mav_hil_gps->time_usec, mav_hil_gps->fix_type, mav_hil_gps->lat, mav_hil_gps->lon, mav_hil_gps->alt, mav_hil_gps->eph, mav_hil_gps->epv, mav_hil_gps->vel, mav_hil_gps->vn, mav_hil_gps->ve, mav_hil_gps->vd, mav_hil_gps->cog, mav_hil_gps->satellites_visible);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_GPS, (const char *)mav_hil_gps, TTALINK_MSG_ID_MAV_HIL_GPS_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_GPS_LEN, TTALINK_MSG_ID_MAV_HIL_GPS_CRC, nocrc);
#endif
}

/**
 * @brief Send a mav_hil_gps message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_mav_hil_gps_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_gps_t* mav_hil_gps)
{
    _ttalink_mav_hil_gps_send_struct(dst_addr, src_addr, chan, mav_hil_gps, false);
}

/**
 * @brief Send a mav_hil_gps message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_mav_hil_gps_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_gps_t* mav_hil_gps)
{
    _ttalink_mav_hil_gps_send_struct(dst_addr, src_addr, chan, mav_hil_gps, true);
}

#if TTALINK_MSG_ID_MAV_HIL_GPS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_mav_hil_gps_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satellites_visible, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint64_t(buf, 0, time_usec);
    _tta_put_uint8_t(buf, 8, fix_type);
    _tta_put_int32_t(buf, 9, lat);
    _tta_put_int32_t(buf, 13, lon);
    _tta_put_int32_t(buf, 17, alt);
    _tta_put_uint16_t(buf, 21, eph);
    _tta_put_uint16_t(buf, 23, epv);
    _tta_put_uint16_t(buf, 25, vel);
    _tta_put_int16_t(buf, 27, vn);
    _tta_put_int16_t(buf, 29, ve);
    _tta_put_int16_t(buf, 31, vd);
    _tta_put_uint16_t(buf, 33, cog);
    _tta_put_uint8_t(buf, 35, satellites_visible);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_GPS, buf, TTALINK_MSG_ID_MAV_HIL_GPS_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_GPS_LEN, TTALINK_MSG_ID_MAV_HIL_GPS_CRC, nocrc);
#else
    ttalink_mav_hil_gps_t *packet = (ttalink_mav_hil_gps_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->fix_type = fix_type;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->eph = eph;
    packet->epv = epv;
    packet->vel = vel;
    packet->vn = vn;
    packet->ve = ve;
    packet->vd = vd;
    packet->cog = cog;
    packet->satellites_visible = satellites_visible;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_GPS, (const char *)packet, TTALINK_MSG_ID_MAV_HIL_GPS_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_GPS_LEN, TTALINK_MSG_ID_MAV_HIL_GPS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_mav_hil_gps_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satellites_visible)
{
    _ttalink_mav_hil_gps_send_buf(dst_addr, src_addr, msgbuf, chan, time_usec, fix_type, lat, lon, alt, eph, epv, vel, vn, ve, vd, cog, satellites_visible, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_mav_hil_gps_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satellites_visible)
{
    _ttalink_mav_hil_gps_send_buf(dst_addr, src_addr, msgbuf, chan, time_usec, fix_type, lat, lon, alt, eph, epv, vel, vn, ve, vd, cog, satellites_visible, true);
}
#endif

#endif

// MESSAGE MAV_HIL_GPS UNPACKING


/**
 * @brief Get field time_usec from mav_hil_gps message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t ttalink_mav_hil_gps_get_time_usec(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field fix_type from mav_hil_gps message
 *
 * @return 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 */
static inline uint8_t ttalink_mav_hil_gps_get_fix_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field lat from mav_hil_gps message
 *
 * @return Latitude (WGS84), in degrees * 1E7
 */
static inline int32_t ttalink_mav_hil_gps_get_lat(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  9);
}

/**
 * @brief Get field lon from mav_hil_gps message
 *
 * @return Longitude (WGS84), in degrees * 1E7
 */
static inline int32_t ttalink_mav_hil_gps_get_lon(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  13);
}

/**
 * @brief Get field alt from mav_hil_gps message
 *
 * @return Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
 */
static inline int32_t ttalink_mav_hil_gps_get_alt(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  17);
}

/**
 * @brief Get field eph from mav_hil_gps message
 *
 * @return GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 */
static inline uint16_t ttalink_mav_hil_gps_get_eph(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  21);
}

/**
 * @brief Get field epv from mav_hil_gps message
 *
 * @return GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
 */
static inline uint16_t ttalink_mav_hil_gps_get_epv(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  23);
}

/**
 * @brief Get field vel from mav_hil_gps message
 *
 * @return GPS ground speed (m/s * 100). If unknown, set to: 65535
 */
static inline uint16_t ttalink_mav_hil_gps_get_vel(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  25);
}

/**
 * @brief Get field vn from mav_hil_gps message
 *
 * @return GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
 */
static inline int16_t ttalink_mav_hil_gps_get_vn(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  27);
}

/**
 * @brief Get field ve from mav_hil_gps message
 *
 * @return GPS velocity in cm/s in EAST direction in earth-fixed NED frame
 */
static inline int16_t ttalink_mav_hil_gps_get_ve(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  29);
}

/**
 * @brief Get field vd from mav_hil_gps message
 *
 * @return GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
 */
static inline int16_t ttalink_mav_hil_gps_get_vd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  31);
}

/**
 * @brief Get field cog from mav_hil_gps message
 *
 * @return Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 */
static inline uint16_t ttalink_mav_hil_gps_get_cog(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  33);
}

/**
 * @brief Get field satellites_visible from mav_hil_gps message
 *
 * @return Number of satellites visible. If unknown, set to 255
 */
static inline uint8_t ttalink_mav_hil_gps_get_satellites_visible(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Decode a mav_hil_gps message into a struct
 *
 * @param msg The message to decode
 * @param mav_hil_gps C-struct to decode the message contents into
 */
static inline void ttalink_mav_hil_gps_decode(const ttalink_message_t* msg, ttalink_mav_hil_gps_t* mav_hil_gps)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    mav_hil_gps->time_usec = ttalink_mav_hil_gps_get_time_usec(msg);
    mav_hil_gps->fix_type = ttalink_mav_hil_gps_get_fix_type(msg);
    mav_hil_gps->lat = ttalink_mav_hil_gps_get_lat(msg);
    mav_hil_gps->lon = ttalink_mav_hil_gps_get_lon(msg);
    mav_hil_gps->alt = ttalink_mav_hil_gps_get_alt(msg);
    mav_hil_gps->eph = ttalink_mav_hil_gps_get_eph(msg);
    mav_hil_gps->epv = ttalink_mav_hil_gps_get_epv(msg);
    mav_hil_gps->vel = ttalink_mav_hil_gps_get_vel(msg);
    mav_hil_gps->vn = ttalink_mav_hil_gps_get_vn(msg);
    mav_hil_gps->ve = ttalink_mav_hil_gps_get_ve(msg);
    mav_hil_gps->vd = ttalink_mav_hil_gps_get_vd(msg);
    mav_hil_gps->cog = ttalink_mav_hil_gps_get_cog(msg);
    mav_hil_gps->satellites_visible = ttalink_mav_hil_gps_get_satellites_visible(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_MAV_HIL_GPS_LEN? msg->len : TTALINK_MSG_ID_MAV_HIL_GPS_LEN;
        memset(mav_hil_gps, 0, TTALINK_MSG_ID_MAV_HIL_GPS_LEN);
    memcpy(mav_hil_gps, _TTA_PAYLOAD(msg), len);
#endif
}
