#pragma once
// MESSAGE MAV_HIL_SENSOR PACKING

#define TTALINK_MSG_ID_MAV_HIL_SENSOR 8211

TTAPACKED(
typedef struct __ttalink_mav_hil_sensor_t {
 uint64_t time_usec; /*< Timestamp (microseconds, synced to UNIX time or since system boot)*/
 float xacc; /*< X acceleration (m/s^2)*/
 float yacc; /*< Y acceleration (m/s^2)*/
 float zacc; /*< Z acceleration (m/s^2)*/
 float xgyro; /*< Angular speed around X axis in body frame (rad / sec)*/
 float ygyro; /*< Angular speed around Y axis in body frame (rad / sec)*/
 float zgyro; /*< Angular speed around Z axis in body frame (rad / sec)*/
 float xmag; /*< X Magnetic field (Gauss)*/
 float ymag; /*< Y Magnetic field (Gauss)*/
 float zmag; /*< Z Magnetic field (Gauss)*/
 float abs_pressure; /*< Absolute pressure in millibar*/
 float diff_pressure; /*< Differential pressure (airspeed) in millibar*/
 float pressure_alt; /*< Altitude calculated from pressure*/
 float temperature; /*< Temperature in degrees celsius*/
 uint32_t fields_updated; /*< Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.*/
}) ttalink_mav_hil_sensor_t;

#define TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN 64
#define TTALINK_MSG_ID_MAV_HIL_SENSOR_MIN_LEN 64
#define TTALINK_MSG_ID_8211_LEN 64
#define TTALINK_MSG_ID_8211_MIN_LEN 64

#define TTALINK_MSG_ID_MAV_HIL_SENSOR_CRC 34
#define TTALINK_MSG_ID_8211_CRC 34



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_MAV_HIL_SENSOR { \
    8211, \
    "MAV_HIL_SENSOR", \
    15, \
    {  { "time_usec", NULL, TTALINK_TYPE_UINT64_T, 0, 0, offsetof(ttalink_mav_hil_sensor_t, time_usec) }, \
         { "xacc", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_mav_hil_sensor_t, xacc) }, \
         { "yacc", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_mav_hil_sensor_t, yacc) }, \
         { "zacc", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_mav_hil_sensor_t, zacc) }, \
         { "xgyro", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_mav_hil_sensor_t, xgyro) }, \
         { "ygyro", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_mav_hil_sensor_t, ygyro) }, \
         { "zgyro", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_mav_hil_sensor_t, zgyro) }, \
         { "xmag", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_mav_hil_sensor_t, xmag) }, \
         { "ymag", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_mav_hil_sensor_t, ymag) }, \
         { "zmag", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_mav_hil_sensor_t, zmag) }, \
         { "abs_pressure", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_mav_hil_sensor_t, abs_pressure) }, \
         { "diff_pressure", NULL, TTALINK_TYPE_FLOAT, 0, 48, offsetof(ttalink_mav_hil_sensor_t, diff_pressure) }, \
         { "pressure_alt", NULL, TTALINK_TYPE_FLOAT, 0, 52, offsetof(ttalink_mav_hil_sensor_t, pressure_alt) }, \
         { "temperature", NULL, TTALINK_TYPE_FLOAT, 0, 56, offsetof(ttalink_mav_hil_sensor_t, temperature) }, \
         { "fields_updated", NULL, TTALINK_TYPE_UINT32_T, 0, 60, offsetof(ttalink_mav_hil_sensor_t, fields_updated) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_MAV_HIL_SENSOR { \
    "MAV_HIL_SENSOR", \
    15, \
    {  { "time_usec", NULL, TTALINK_TYPE_UINT64_T, 0, 0, offsetof(ttalink_mav_hil_sensor_t, time_usec) }, \
         { "xacc", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_mav_hil_sensor_t, xacc) }, \
         { "yacc", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_mav_hil_sensor_t, yacc) }, \
         { "zacc", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_mav_hil_sensor_t, zacc) }, \
         { "xgyro", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_mav_hil_sensor_t, xgyro) }, \
         { "ygyro", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_mav_hil_sensor_t, ygyro) }, \
         { "zgyro", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_mav_hil_sensor_t, zgyro) }, \
         { "xmag", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_mav_hil_sensor_t, xmag) }, \
         { "ymag", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_mav_hil_sensor_t, ymag) }, \
         { "zmag", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_mav_hil_sensor_t, zmag) }, \
         { "abs_pressure", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_mav_hil_sensor_t, abs_pressure) }, \
         { "diff_pressure", NULL, TTALINK_TYPE_FLOAT, 0, 48, offsetof(ttalink_mav_hil_sensor_t, diff_pressure) }, \
         { "pressure_alt", NULL, TTALINK_TYPE_FLOAT, 0, 52, offsetof(ttalink_mav_hil_sensor_t, pressure_alt) }, \
         { "temperature", NULL, TTALINK_TYPE_FLOAT, 0, 56, offsetof(ttalink_mav_hil_sensor_t, temperature) }, \
         { "fields_updated", NULL, TTALINK_TYPE_UINT32_T, 0, 60, offsetof(ttalink_mav_hil_sensor_t, fields_updated) }, \
         } \
}
#endif


static inline uint16_t _ttalink_mav_hil_sensor_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint32_t fields_updated, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN];
    _tta_put_uint64_t(buf, 0, time_usec);
    _tta_put_float(buf, 8, xacc);
    _tta_put_float(buf, 12, yacc);
    _tta_put_float(buf, 16, zacc);
    _tta_put_float(buf, 20, xgyro);
    _tta_put_float(buf, 24, ygyro);
    _tta_put_float(buf, 28, zgyro);
    _tta_put_float(buf, 32, xmag);
    _tta_put_float(buf, 36, ymag);
    _tta_put_float(buf, 40, zmag);
    _tta_put_float(buf, 44, abs_pressure);
    _tta_put_float(buf, 48, diff_pressure);
    _tta_put_float(buf, 52, pressure_alt);
    _tta_put_float(buf, 56, temperature);
    _tta_put_uint32_t(buf, 60, fields_updated);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN);
#else
    ttalink_mav_hil_sensor_t packet;
    packet.time_usec = time_usec;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.xmag = xmag;
    packet.ymag = ymag;
    packet.zmag = zmag;
    packet.abs_pressure = abs_pressure;
    packet.diff_pressure = diff_pressure;
    packet.pressure_alt = pressure_alt;
    packet.temperature = temperature;
    packet.fields_updated = fields_updated;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MAV_HIL_SENSOR;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_MAV_HIL_SENSOR_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN, TTALINK_MSG_ID_MAV_HIL_SENSOR_CRC, nocrc);
}

/**
 * @brief Pack a mav_hil_sensor message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param xacc X acceleration (m/s^2)
 * @param yacc Y acceleration (m/s^2)
 * @param zacc Z acceleration (m/s^2)
 * @param xgyro Angular speed around X axis in body frame (rad / sec)
 * @param ygyro Angular speed around Y axis in body frame (rad / sec)
 * @param zgyro Angular speed around Z axis in body frame (rad / sec)
 * @param xmag X Magnetic field (Gauss)
 * @param ymag Y Magnetic field (Gauss)
 * @param zmag Z Magnetic field (Gauss)
 * @param abs_pressure Absolute pressure in millibar
 * @param diff_pressure Differential pressure (airspeed) in millibar
 * @param pressure_alt Altitude calculated from pressure
 * @param temperature Temperature in degrees celsius
 * @param fields_updated Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_sensor_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint32_t fields_updated)
{
    return _ttalink_mav_hil_sensor_pack(dst_addr, src_addr, msg,  time_usec, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, abs_pressure, diff_pressure, pressure_alt, temperature, fields_updated, false);
}

/**
 * @brief Pack a mav_hil_sensor message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param xacc X acceleration (m/s^2)
 * @param yacc Y acceleration (m/s^2)
 * @param zacc Z acceleration (m/s^2)
 * @param xgyro Angular speed around X axis in body frame (rad / sec)
 * @param ygyro Angular speed around Y axis in body frame (rad / sec)
 * @param zgyro Angular speed around Z axis in body frame (rad / sec)
 * @param xmag X Magnetic field (Gauss)
 * @param ymag Y Magnetic field (Gauss)
 * @param zmag Z Magnetic field (Gauss)
 * @param abs_pressure Absolute pressure in millibar
 * @param diff_pressure Differential pressure (airspeed) in millibar
 * @param pressure_alt Altitude calculated from pressure
 * @param temperature Temperature in degrees celsius
 * @param fields_updated Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_sensor_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint32_t fields_updated)
{
    return _ttalink_mav_hil_sensor_pack(dst_addr, src_addr, msg,  time_usec, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, abs_pressure, diff_pressure, pressure_alt, temperature, fields_updated, true);
}


static inline uint16_t _ttalink_mav_hil_sensor_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time_usec,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float xmag,float ymag,float zmag,float abs_pressure,float diff_pressure,float pressure_alt,float temperature,uint32_t fields_updated, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN];
    _tta_put_uint64_t(buf, 0, time_usec);
    _tta_put_float(buf, 8, xacc);
    _tta_put_float(buf, 12, yacc);
    _tta_put_float(buf, 16, zacc);
    _tta_put_float(buf, 20, xgyro);
    _tta_put_float(buf, 24, ygyro);
    _tta_put_float(buf, 28, zgyro);
    _tta_put_float(buf, 32, xmag);
    _tta_put_float(buf, 36, ymag);
    _tta_put_float(buf, 40, zmag);
    _tta_put_float(buf, 44, abs_pressure);
    _tta_put_float(buf, 48, diff_pressure);
    _tta_put_float(buf, 52, pressure_alt);
    _tta_put_float(buf, 56, temperature);
    _tta_put_uint32_t(buf, 60, fields_updated);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN);
#else
    ttalink_mav_hil_sensor_t packet;
    packet.time_usec = time_usec;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.xmag = xmag;
    packet.ymag = ymag;
    packet.zmag = zmag;
    packet.abs_pressure = abs_pressure;
    packet.diff_pressure = diff_pressure;
    packet.pressure_alt = pressure_alt;
    packet.temperature = temperature;
    packet.fields_updated = fields_updated;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MAV_HIL_SENSOR;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_SENSOR_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN, TTALINK_MSG_ID_MAV_HIL_SENSOR_CRC, nocrc);
}

/**
 * @brief Pack a mav_hil_sensor message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param xacc X acceleration (m/s^2)
 * @param yacc Y acceleration (m/s^2)
 * @param zacc Z acceleration (m/s^2)
 * @param xgyro Angular speed around X axis in body frame (rad / sec)
 * @param ygyro Angular speed around Y axis in body frame (rad / sec)
 * @param zgyro Angular speed around Z axis in body frame (rad / sec)
 * @param xmag X Magnetic field (Gauss)
 * @param ymag Y Magnetic field (Gauss)
 * @param zmag Z Magnetic field (Gauss)
 * @param abs_pressure Absolute pressure in millibar
 * @param diff_pressure Differential pressure (airspeed) in millibar
 * @param pressure_alt Altitude calculated from pressure
 * @param temperature Temperature in degrees celsius
 * @param fields_updated Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_sensor_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time_usec,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float xmag,float ymag,float zmag,float abs_pressure,float diff_pressure,float pressure_alt,float temperature,uint32_t fields_updated)
{
    return _ttalink_mav_hil_sensor_pack_chan(dst_addr, src_addr, chan, msg,  time_usec, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, abs_pressure, diff_pressure, pressure_alt, temperature, fields_updated, false);
}

/**
 * @brief Pack a mav_hil_sensor message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param xacc X acceleration (m/s^2)
 * @param yacc Y acceleration (m/s^2)
 * @param zacc Z acceleration (m/s^2)
 * @param xgyro Angular speed around X axis in body frame (rad / sec)
 * @param ygyro Angular speed around Y axis in body frame (rad / sec)
 * @param zgyro Angular speed around Z axis in body frame (rad / sec)
 * @param xmag X Magnetic field (Gauss)
 * @param ymag Y Magnetic field (Gauss)
 * @param zmag Z Magnetic field (Gauss)
 * @param abs_pressure Absolute pressure in millibar
 * @param diff_pressure Differential pressure (airspeed) in millibar
 * @param pressure_alt Altitude calculated from pressure
 * @param temperature Temperature in degrees celsius
 * @param fields_updated Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_sensor_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time_usec,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float xmag,float ymag,float zmag,float abs_pressure,float diff_pressure,float pressure_alt,float temperature,uint32_t fields_updated)
{
    return _ttalink_mav_hil_sensor_pack_chan(dst_addr, src_addr, chan, msg,  time_usec, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, abs_pressure, diff_pressure, pressure_alt, temperature, fields_updated, true);
}


static inline uint16_t _ttalink_mav_hil_sensor_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_sensor_t* mav_hil_sensor, bool nocrc)
{
    if(nocrc){
        return ttalink_mav_hil_sensor_pack_nocrc(dst_addr, src_addr, msg, mav_hil_sensor->time_usec, mav_hil_sensor->xacc, mav_hil_sensor->yacc, mav_hil_sensor->zacc, mav_hil_sensor->xgyro, mav_hil_sensor->ygyro, mav_hil_sensor->zgyro, mav_hil_sensor->xmag, mav_hil_sensor->ymag, mav_hil_sensor->zmag, mav_hil_sensor->abs_pressure, mav_hil_sensor->diff_pressure, mav_hil_sensor->pressure_alt, mav_hil_sensor->temperature, mav_hil_sensor->fields_updated);
    }else{
        return ttalink_mav_hil_sensor_pack(dst_addr, src_addr, msg, mav_hil_sensor->time_usec, mav_hil_sensor->xacc, mav_hil_sensor->yacc, mav_hil_sensor->zacc, mav_hil_sensor->xgyro, mav_hil_sensor->ygyro, mav_hil_sensor->zgyro, mav_hil_sensor->xmag, mav_hil_sensor->ymag, mav_hil_sensor->zmag, mav_hil_sensor->abs_pressure, mav_hil_sensor->diff_pressure, mav_hil_sensor->pressure_alt, mav_hil_sensor->temperature, mav_hil_sensor->fields_updated);
    }
    
}

/**
 * @brief Encode a mav_hil_sensor struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_sensor C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_sensor_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_sensor_t* mav_hil_sensor)
{
    return _ttalink_mav_hil_sensor_encode(dst_addr, src_addr, msg, mav_hil_sensor, false);
}

/**
 * @brief Encode a mav_hil_sensor struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_sensor C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_sensor_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_sensor_t* mav_hil_sensor)
{
    return _ttalink_mav_hil_sensor_encode(dst_addr, src_addr, msg, mav_hil_sensor, true);
}


static inline uint16_t _ttalink_mav_hil_sensor_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_sensor_t* mav_hil_sensor, bool nocrc)
{
    if(nocrc){
        return ttalink_mav_hil_sensor_pack_chan_nocrc(dst_addr, src_addr, chan, msg, mav_hil_sensor->time_usec, mav_hil_sensor->xacc, mav_hil_sensor->yacc, mav_hil_sensor->zacc, mav_hil_sensor->xgyro, mav_hil_sensor->ygyro, mav_hil_sensor->zgyro, mav_hil_sensor->xmag, mav_hil_sensor->ymag, mav_hil_sensor->zmag, mav_hil_sensor->abs_pressure, mav_hil_sensor->diff_pressure, mav_hil_sensor->pressure_alt, mav_hil_sensor->temperature, mav_hil_sensor->fields_updated);
    }else{
        return ttalink_mav_hil_sensor_pack_chan(dst_addr, src_addr, chan, msg, mav_hil_sensor->time_usec, mav_hil_sensor->xacc, mav_hil_sensor->yacc, mav_hil_sensor->zacc, mav_hil_sensor->xgyro, mav_hil_sensor->ygyro, mav_hil_sensor->zgyro, mav_hil_sensor->xmag, mav_hil_sensor->ymag, mav_hil_sensor->zmag, mav_hil_sensor->abs_pressure, mav_hil_sensor->diff_pressure, mav_hil_sensor->pressure_alt, mav_hil_sensor->temperature, mav_hil_sensor->fields_updated);
    }
}

/**
 * @brief Encode a mav_hil_sensor struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_sensor C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_sensor_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_sensor_t* mav_hil_sensor)
{
    return _ttalink_mav_hil_sensor_encode_chan(dst_addr, src_addr, chan, msg, mav_hil_sensor, false);
}

/**
 * @brief Encode a mav_hil_sensor struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_sensor C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_sensor_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_sensor_t* mav_hil_sensor)
{
    return _ttalink_mav_hil_sensor_encode_chan(dst_addr, src_addr, chan, msg, mav_hil_sensor, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_mav_hil_sensor_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint32_t fields_updated, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN];
    _tta_put_uint64_t(buf, 0, time_usec);
    _tta_put_float(buf, 8, xacc);
    _tta_put_float(buf, 12, yacc);
    _tta_put_float(buf, 16, zacc);
    _tta_put_float(buf, 20, xgyro);
    _tta_put_float(buf, 24, ygyro);
    _tta_put_float(buf, 28, zgyro);
    _tta_put_float(buf, 32, xmag);
    _tta_put_float(buf, 36, ymag);
    _tta_put_float(buf, 40, zmag);
    _tta_put_float(buf, 44, abs_pressure);
    _tta_put_float(buf, 48, diff_pressure);
    _tta_put_float(buf, 52, pressure_alt);
    _tta_put_float(buf, 56, temperature);
    _tta_put_uint32_t(buf, 60, fields_updated);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_SENSOR, buf, TTALINK_MSG_ID_MAV_HIL_SENSOR_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN, TTALINK_MSG_ID_MAV_HIL_SENSOR_CRC, nocrc);
#else
    ttalink_mav_hil_sensor_t packet;
    packet.time_usec = time_usec;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.xmag = xmag;
    packet.ymag = ymag;
    packet.zmag = zmag;
    packet.abs_pressure = abs_pressure;
    packet.diff_pressure = diff_pressure;
    packet.pressure_alt = pressure_alt;
    packet.temperature = temperature;
    packet.fields_updated = fields_updated;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_SENSOR, (const char *)&packet, TTALINK_MSG_ID_MAV_HIL_SENSOR_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN, TTALINK_MSG_ID_MAV_HIL_SENSOR_CRC, nocrc);
#endif
}

/**
 * @brief Send a mav_hil_sensor message
 * @param chan TTAlink channel to send the message
 *
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param xacc X acceleration (m/s^2)
 * @param yacc Y acceleration (m/s^2)
 * @param zacc Z acceleration (m/s^2)
 * @param xgyro Angular speed around X axis in body frame (rad / sec)
 * @param ygyro Angular speed around Y axis in body frame (rad / sec)
 * @param zgyro Angular speed around Z axis in body frame (rad / sec)
 * @param xmag X Magnetic field (Gauss)
 * @param ymag Y Magnetic field (Gauss)
 * @param zmag Z Magnetic field (Gauss)
 * @param abs_pressure Absolute pressure in millibar
 * @param diff_pressure Differential pressure (airspeed) in millibar
 * @param pressure_alt Altitude calculated from pressure
 * @param temperature Temperature in degrees celsius
 * @param fields_updated Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
 */
static inline void ttalink_mav_hil_sensor_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint32_t fields_updated)
{
    _ttalink_mav_hil_sensor_send(dst_addr, src_addr, chan, time_usec, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, abs_pressure, diff_pressure, pressure_alt, temperature, fields_updated, false);
}

/**
 * @brief Send a mav_hil_sensor message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param xacc X acceleration (m/s^2)
 * @param yacc Y acceleration (m/s^2)
 * @param zacc Z acceleration (m/s^2)
 * @param xgyro Angular speed around X axis in body frame (rad / sec)
 * @param ygyro Angular speed around Y axis in body frame (rad / sec)
 * @param zgyro Angular speed around Z axis in body frame (rad / sec)
 * @param xmag X Magnetic field (Gauss)
 * @param ymag Y Magnetic field (Gauss)
 * @param zmag Z Magnetic field (Gauss)
 * @param abs_pressure Absolute pressure in millibar
 * @param diff_pressure Differential pressure (airspeed) in millibar
 * @param pressure_alt Altitude calculated from pressure
 * @param temperature Temperature in degrees celsius
 * @param fields_updated Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
 */
static inline void ttalink_mav_hil_sensor_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint32_t fields_updated)
{
    _ttalink_mav_hil_sensor_send(dst_addr, src_addr, chan, time_usec, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, abs_pressure, diff_pressure, pressure_alt, temperature, fields_updated, true);
}


static inline void _ttalink_mav_hil_sensor_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_sensor_t* mav_hil_sensor, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_mav_hil_sensor_send_nocrc(dst_addr, src_addr, chan, mav_hil_sensor->time_usec, mav_hil_sensor->xacc, mav_hil_sensor->yacc, mav_hil_sensor->zacc, mav_hil_sensor->xgyro, mav_hil_sensor->ygyro, mav_hil_sensor->zgyro, mav_hil_sensor->xmag, mav_hil_sensor->ymag, mav_hil_sensor->zmag, mav_hil_sensor->abs_pressure, mav_hil_sensor->diff_pressure, mav_hil_sensor->pressure_alt, mav_hil_sensor->temperature, mav_hil_sensor->fields_updated);
    }else{
        ttalink_mav_hil_sensor_send(dst_addr, src_addr, chan, mav_hil_sensor->time_usec, mav_hil_sensor->xacc, mav_hil_sensor->yacc, mav_hil_sensor->zacc, mav_hil_sensor->xgyro, mav_hil_sensor->ygyro, mav_hil_sensor->zgyro, mav_hil_sensor->xmag, mav_hil_sensor->ymag, mav_hil_sensor->zmag, mav_hil_sensor->abs_pressure, mav_hil_sensor->diff_pressure, mav_hil_sensor->pressure_alt, mav_hil_sensor->temperature, mav_hil_sensor->fields_updated);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_SENSOR, (const char *)mav_hil_sensor, TTALINK_MSG_ID_MAV_HIL_SENSOR_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN, TTALINK_MSG_ID_MAV_HIL_SENSOR_CRC, nocrc);
#endif
}

/**
 * @brief Send a mav_hil_sensor message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_mav_hil_sensor_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_sensor_t* mav_hil_sensor)
{
    _ttalink_mav_hil_sensor_send_struct(dst_addr, src_addr, chan, mav_hil_sensor, false);
}

/**
 * @brief Send a mav_hil_sensor message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_mav_hil_sensor_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_sensor_t* mav_hil_sensor)
{
    _ttalink_mav_hil_sensor_send_struct(dst_addr, src_addr, chan, mav_hil_sensor, true);
}

#if TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_mav_hil_sensor_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint32_t fields_updated, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint64_t(buf, 0, time_usec);
    _tta_put_float(buf, 8, xacc);
    _tta_put_float(buf, 12, yacc);
    _tta_put_float(buf, 16, zacc);
    _tta_put_float(buf, 20, xgyro);
    _tta_put_float(buf, 24, ygyro);
    _tta_put_float(buf, 28, zgyro);
    _tta_put_float(buf, 32, xmag);
    _tta_put_float(buf, 36, ymag);
    _tta_put_float(buf, 40, zmag);
    _tta_put_float(buf, 44, abs_pressure);
    _tta_put_float(buf, 48, diff_pressure);
    _tta_put_float(buf, 52, pressure_alt);
    _tta_put_float(buf, 56, temperature);
    _tta_put_uint32_t(buf, 60, fields_updated);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_SENSOR, buf, TTALINK_MSG_ID_MAV_HIL_SENSOR_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN, TTALINK_MSG_ID_MAV_HIL_SENSOR_CRC, nocrc);
#else
    ttalink_mav_hil_sensor_t *packet = (ttalink_mav_hil_sensor_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->xacc = xacc;
    packet->yacc = yacc;
    packet->zacc = zacc;
    packet->xgyro = xgyro;
    packet->ygyro = ygyro;
    packet->zgyro = zgyro;
    packet->xmag = xmag;
    packet->ymag = ymag;
    packet->zmag = zmag;
    packet->abs_pressure = abs_pressure;
    packet->diff_pressure = diff_pressure;
    packet->pressure_alt = pressure_alt;
    packet->temperature = temperature;
    packet->fields_updated = fields_updated;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_SENSOR, (const char *)packet, TTALINK_MSG_ID_MAV_HIL_SENSOR_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN, TTALINK_MSG_ID_MAV_HIL_SENSOR_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_mav_hil_sensor_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint32_t fields_updated)
{
    _ttalink_mav_hil_sensor_send_buf(dst_addr, src_addr, msgbuf, chan, time_usec, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, abs_pressure, diff_pressure, pressure_alt, temperature, fields_updated, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_mav_hil_sensor_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint32_t fields_updated)
{
    _ttalink_mav_hil_sensor_send_buf(dst_addr, src_addr, msgbuf, chan, time_usec, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, abs_pressure, diff_pressure, pressure_alt, temperature, fields_updated, true);
}
#endif

#endif

// MESSAGE MAV_HIL_SENSOR UNPACKING


/**
 * @brief Get field time_usec from mav_hil_sensor message
 *
 * @return Timestamp (microseconds, synced to UNIX time or since system boot)
 */
static inline uint64_t ttalink_mav_hil_sensor_get_time_usec(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field xacc from mav_hil_sensor message
 *
 * @return X acceleration (m/s^2)
 */
static inline float ttalink_mav_hil_sensor_get_xacc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  8);
}

/**
 * @brief Get field yacc from mav_hil_sensor message
 *
 * @return Y acceleration (m/s^2)
 */
static inline float ttalink_mav_hil_sensor_get_yacc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  12);
}

/**
 * @brief Get field zacc from mav_hil_sensor message
 *
 * @return Z acceleration (m/s^2)
 */
static inline float ttalink_mav_hil_sensor_get_zacc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field xgyro from mav_hil_sensor message
 *
 * @return Angular speed around X axis in body frame (rad / sec)
 */
static inline float ttalink_mav_hil_sensor_get_xgyro(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  20);
}

/**
 * @brief Get field ygyro from mav_hil_sensor message
 *
 * @return Angular speed around Y axis in body frame (rad / sec)
 */
static inline float ttalink_mav_hil_sensor_get_ygyro(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Get field zgyro from mav_hil_sensor message
 *
 * @return Angular speed around Z axis in body frame (rad / sec)
 */
static inline float ttalink_mav_hil_sensor_get_zgyro(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  28);
}

/**
 * @brief Get field xmag from mav_hil_sensor message
 *
 * @return X Magnetic field (Gauss)
 */
static inline float ttalink_mav_hil_sensor_get_xmag(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Get field ymag from mav_hil_sensor message
 *
 * @return Y Magnetic field (Gauss)
 */
static inline float ttalink_mav_hil_sensor_get_ymag(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  36);
}

/**
 * @brief Get field zmag from mav_hil_sensor message
 *
 * @return Z Magnetic field (Gauss)
 */
static inline float ttalink_mav_hil_sensor_get_zmag(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  40);
}

/**
 * @brief Get field abs_pressure from mav_hil_sensor message
 *
 * @return Absolute pressure in millibar
 */
static inline float ttalink_mav_hil_sensor_get_abs_pressure(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  44);
}

/**
 * @brief Get field diff_pressure from mav_hil_sensor message
 *
 * @return Differential pressure (airspeed) in millibar
 */
static inline float ttalink_mav_hil_sensor_get_diff_pressure(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  48);
}

/**
 * @brief Get field pressure_alt from mav_hil_sensor message
 *
 * @return Altitude calculated from pressure
 */
static inline float ttalink_mav_hil_sensor_get_pressure_alt(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  52);
}

/**
 * @brief Get field temperature from mav_hil_sensor message
 *
 * @return Temperature in degrees celsius
 */
static inline float ttalink_mav_hil_sensor_get_temperature(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  56);
}

/**
 * @brief Get field fields_updated from mav_hil_sensor message
 *
 * @return Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
 */
static inline uint32_t ttalink_mav_hil_sensor_get_fields_updated(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  60);
}

/**
 * @brief Decode a mav_hil_sensor message into a struct
 *
 * @param msg The message to decode
 * @param mav_hil_sensor C-struct to decode the message contents into
 */
static inline void ttalink_mav_hil_sensor_decode(const ttalink_message_t* msg, ttalink_mav_hil_sensor_t* mav_hil_sensor)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    mav_hil_sensor->time_usec = ttalink_mav_hil_sensor_get_time_usec(msg);
    mav_hil_sensor->xacc = ttalink_mav_hil_sensor_get_xacc(msg);
    mav_hil_sensor->yacc = ttalink_mav_hil_sensor_get_yacc(msg);
    mav_hil_sensor->zacc = ttalink_mav_hil_sensor_get_zacc(msg);
    mav_hil_sensor->xgyro = ttalink_mav_hil_sensor_get_xgyro(msg);
    mav_hil_sensor->ygyro = ttalink_mav_hil_sensor_get_ygyro(msg);
    mav_hil_sensor->zgyro = ttalink_mav_hil_sensor_get_zgyro(msg);
    mav_hil_sensor->xmag = ttalink_mav_hil_sensor_get_xmag(msg);
    mav_hil_sensor->ymag = ttalink_mav_hil_sensor_get_ymag(msg);
    mav_hil_sensor->zmag = ttalink_mav_hil_sensor_get_zmag(msg);
    mav_hil_sensor->abs_pressure = ttalink_mav_hil_sensor_get_abs_pressure(msg);
    mav_hil_sensor->diff_pressure = ttalink_mav_hil_sensor_get_diff_pressure(msg);
    mav_hil_sensor->pressure_alt = ttalink_mav_hil_sensor_get_pressure_alt(msg);
    mav_hil_sensor->temperature = ttalink_mav_hil_sensor_get_temperature(msg);
    mav_hil_sensor->fields_updated = ttalink_mav_hil_sensor_get_fields_updated(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN? msg->len : TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN;
        memset(mav_hil_sensor, 0, TTALINK_MSG_ID_MAV_HIL_SENSOR_LEN);
    memcpy(mav_hil_sensor, _TTA_PAYLOAD(msg), len);
#endif
}
