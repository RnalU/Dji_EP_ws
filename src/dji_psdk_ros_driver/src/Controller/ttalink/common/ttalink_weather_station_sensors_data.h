#pragma once
// MESSAGE WEATHER_STATION_SENSORS_DATA PACKING

#define TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA 2169

TTAPACKED(
typedef struct __ttalink_weather_station_sensors_data_t {
 uint32_t update_time; /*<  update_time*/
 float wind_speed; /*<  风速.*/
 float wind_direction; /*<  风向.*/
 float temperature; /*<  温度.*/
 float humidity; /*<  湿度.*/
 float pressure; /*<  气压.*/
 float param1; /*<  预留.*/
 float param2; /*<  预留.*/
 float param3; /*<  预留.*/
 float param4; /*<  预留.*/
}) ttalink_weather_station_sensors_data_t;

#define TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN 40
#define TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_MIN_LEN 40
#define TTALINK_MSG_ID_2169_LEN 40
#define TTALINK_MSG_ID_2169_MIN_LEN 40

#define TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_CRC 198
#define TTALINK_MSG_ID_2169_CRC 198



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_WEATHER_STATION_SENSORS_DATA { \
    2169, \
    "WEATHER_STATION_SENSORS_DATA", \
    10, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_weather_station_sensors_data_t, update_time) }, \
         { "wind_speed", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_weather_station_sensors_data_t, wind_speed) }, \
         { "wind_direction", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_weather_station_sensors_data_t, wind_direction) }, \
         { "temperature", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_weather_station_sensors_data_t, temperature) }, \
         { "humidity", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_weather_station_sensors_data_t, humidity) }, \
         { "pressure", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_weather_station_sensors_data_t, pressure) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_weather_station_sensors_data_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_weather_station_sensors_data_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_weather_station_sensors_data_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_weather_station_sensors_data_t, param4) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_WEATHER_STATION_SENSORS_DATA { \
    "WEATHER_STATION_SENSORS_DATA", \
    10, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_weather_station_sensors_data_t, update_time) }, \
         { "wind_speed", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_weather_station_sensors_data_t, wind_speed) }, \
         { "wind_direction", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_weather_station_sensors_data_t, wind_direction) }, \
         { "temperature", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_weather_station_sensors_data_t, temperature) }, \
         { "humidity", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_weather_station_sensors_data_t, humidity) }, \
         { "pressure", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_weather_station_sensors_data_t, pressure) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_weather_station_sensors_data_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_weather_station_sensors_data_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_weather_station_sensors_data_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_weather_station_sensors_data_t, param4) }, \
         } \
}
#endif


static inline uint16_t _ttalink_weather_station_sensors_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, float wind_speed, float wind_direction, float temperature, float humidity, float pressure, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, wind_speed);
    _tta_put_float(buf, 8, wind_direction);
    _tta_put_float(buf, 12, temperature);
    _tta_put_float(buf, 16, humidity);
    _tta_put_float(buf, 20, pressure);
    _tta_put_float(buf, 24, param1);
    _tta_put_float(buf, 28, param2);
    _tta_put_float(buf, 32, param3);
    _tta_put_float(buf, 36, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN);
#else
    ttalink_weather_station_sensors_data_t packet;
    packet.update_time = update_time;
    packet.wind_speed = wind_speed;
    packet.wind_direction = wind_direction;
    packet.temperature = temperature;
    packet.humidity = humidity;
    packet.pressure = pressure;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_CRC, nocrc);
}

/**
 * @brief Pack a weather_station_sensors_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param wind_speed  风速.
 * @param wind_direction  风向.
 * @param temperature  温度.
 * @param humidity  湿度.
 * @param pressure  气压.
 * @param param1  预留.
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_weather_station_sensors_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, float wind_speed, float wind_direction, float temperature, float humidity, float pressure, float param1, float param2, float param3, float param4)
{
    return _ttalink_weather_station_sensors_data_pack(dst_addr, src_addr, msg,  update_time, wind_speed, wind_direction, temperature, humidity, pressure, param1, param2, param3, param4, false);
}

/**
 * @brief Pack a weather_station_sensors_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param wind_speed  风速.
 * @param wind_direction  风向.
 * @param temperature  温度.
 * @param humidity  湿度.
 * @param pressure  气压.
 * @param param1  预留.
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_weather_station_sensors_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, float wind_speed, float wind_direction, float temperature, float humidity, float pressure, float param1, float param2, float param3, float param4)
{
    return _ttalink_weather_station_sensors_data_pack(dst_addr, src_addr, msg,  update_time, wind_speed, wind_direction, temperature, humidity, pressure, param1, param2, param3, param4, true);
}


static inline uint16_t _ttalink_weather_station_sensors_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,float wind_speed,float wind_direction,float temperature,float humidity,float pressure,float param1,float param2,float param3,float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, wind_speed);
    _tta_put_float(buf, 8, wind_direction);
    _tta_put_float(buf, 12, temperature);
    _tta_put_float(buf, 16, humidity);
    _tta_put_float(buf, 20, pressure);
    _tta_put_float(buf, 24, param1);
    _tta_put_float(buf, 28, param2);
    _tta_put_float(buf, 32, param3);
    _tta_put_float(buf, 36, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN);
#else
    ttalink_weather_station_sensors_data_t packet;
    packet.update_time = update_time;
    packet.wind_speed = wind_speed;
    packet.wind_direction = wind_direction;
    packet.temperature = temperature;
    packet.humidity = humidity;
    packet.pressure = pressure;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_CRC, nocrc);
}

/**
 * @brief Pack a weather_station_sensors_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param wind_speed  风速.
 * @param wind_direction  风向.
 * @param temperature  温度.
 * @param humidity  湿度.
 * @param pressure  气压.
 * @param param1  预留.
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_weather_station_sensors_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,float wind_speed,float wind_direction,float temperature,float humidity,float pressure,float param1,float param2,float param3,float param4)
{
    return _ttalink_weather_station_sensors_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, wind_speed, wind_direction, temperature, humidity, pressure, param1, param2, param3, param4, false);
}

/**
 * @brief Pack a weather_station_sensors_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param wind_speed  风速.
 * @param wind_direction  风向.
 * @param temperature  温度.
 * @param humidity  湿度.
 * @param pressure  气压.
 * @param param1  预留.
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_weather_station_sensors_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,float wind_speed,float wind_direction,float temperature,float humidity,float pressure,float param1,float param2,float param3,float param4)
{
    return _ttalink_weather_station_sensors_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, wind_speed, wind_direction, temperature, humidity, pressure, param1, param2, param3, param4, true);
}


static inline uint16_t _ttalink_weather_station_sensors_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_weather_station_sensors_data_t* weather_station_sensors_data, bool nocrc)
{
    if(nocrc){
        return ttalink_weather_station_sensors_data_pack_nocrc(dst_addr, src_addr, msg, weather_station_sensors_data->update_time, weather_station_sensors_data->wind_speed, weather_station_sensors_data->wind_direction, weather_station_sensors_data->temperature, weather_station_sensors_data->humidity, weather_station_sensors_data->pressure, weather_station_sensors_data->param1, weather_station_sensors_data->param2, weather_station_sensors_data->param3, weather_station_sensors_data->param4);
    }else{
        return ttalink_weather_station_sensors_data_pack(dst_addr, src_addr, msg, weather_station_sensors_data->update_time, weather_station_sensors_data->wind_speed, weather_station_sensors_data->wind_direction, weather_station_sensors_data->temperature, weather_station_sensors_data->humidity, weather_station_sensors_data->pressure, weather_station_sensors_data->param1, weather_station_sensors_data->param2, weather_station_sensors_data->param3, weather_station_sensors_data->param4);
    }
    
}

/**
 * @brief Encode a weather_station_sensors_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param weather_station_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_weather_station_sensors_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_weather_station_sensors_data_t* weather_station_sensors_data)
{
    return _ttalink_weather_station_sensors_data_encode(dst_addr, src_addr, msg, weather_station_sensors_data, false);
}

/**
 * @brief Encode a weather_station_sensors_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param weather_station_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_weather_station_sensors_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_weather_station_sensors_data_t* weather_station_sensors_data)
{
    return _ttalink_weather_station_sensors_data_encode(dst_addr, src_addr, msg, weather_station_sensors_data, true);
}


static inline uint16_t _ttalink_weather_station_sensors_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_weather_station_sensors_data_t* weather_station_sensors_data, bool nocrc)
{
    if(nocrc){
        return ttalink_weather_station_sensors_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, weather_station_sensors_data->update_time, weather_station_sensors_data->wind_speed, weather_station_sensors_data->wind_direction, weather_station_sensors_data->temperature, weather_station_sensors_data->humidity, weather_station_sensors_data->pressure, weather_station_sensors_data->param1, weather_station_sensors_data->param2, weather_station_sensors_data->param3, weather_station_sensors_data->param4);
    }else{
        return ttalink_weather_station_sensors_data_pack_chan(dst_addr, src_addr, chan, msg, weather_station_sensors_data->update_time, weather_station_sensors_data->wind_speed, weather_station_sensors_data->wind_direction, weather_station_sensors_data->temperature, weather_station_sensors_data->humidity, weather_station_sensors_data->pressure, weather_station_sensors_data->param1, weather_station_sensors_data->param2, weather_station_sensors_data->param3, weather_station_sensors_data->param4);
    }
}

/**
 * @brief Encode a weather_station_sensors_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param weather_station_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_weather_station_sensors_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_weather_station_sensors_data_t* weather_station_sensors_data)
{
    return _ttalink_weather_station_sensors_data_encode_chan(dst_addr, src_addr, chan, msg, weather_station_sensors_data, false);
}

/**
 * @brief Encode a weather_station_sensors_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param weather_station_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_weather_station_sensors_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_weather_station_sensors_data_t* weather_station_sensors_data)
{
    return _ttalink_weather_station_sensors_data_encode_chan(dst_addr, src_addr, chan, msg, weather_station_sensors_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_weather_station_sensors_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, float wind_speed, float wind_direction, float temperature, float humidity, float pressure, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, wind_speed);
    _tta_put_float(buf, 8, wind_direction);
    _tta_put_float(buf, 12, temperature);
    _tta_put_float(buf, 16, humidity);
    _tta_put_float(buf, 20, pressure);
    _tta_put_float(buf, 24, param1);
    _tta_put_float(buf, 28, param2);
    _tta_put_float(buf, 32, param3);
    _tta_put_float(buf, 36, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA, buf, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_CRC, nocrc);
#else
    ttalink_weather_station_sensors_data_t packet;
    packet.update_time = update_time;
    packet.wind_speed = wind_speed;
    packet.wind_direction = wind_direction;
    packet.temperature = temperature;
    packet.humidity = humidity;
    packet.pressure = pressure;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA, (const char *)&packet, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a weather_station_sensors_data message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param wind_speed  风速.
 * @param wind_direction  风向.
 * @param temperature  温度.
 * @param humidity  湿度.
 * @param pressure  气压.
 * @param param1  预留.
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 */
static inline void ttalink_weather_station_sensors_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, float wind_speed, float wind_direction, float temperature, float humidity, float pressure, float param1, float param2, float param3, float param4)
{
    _ttalink_weather_station_sensors_data_send(dst_addr, src_addr, chan, update_time, wind_speed, wind_direction, temperature, humidity, pressure, param1, param2, param3, param4, false);
}

/**
 * @brief Send a weather_station_sensors_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param wind_speed  风速.
 * @param wind_direction  风向.
 * @param temperature  温度.
 * @param humidity  湿度.
 * @param pressure  气压.
 * @param param1  预留.
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 */
static inline void ttalink_weather_station_sensors_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, float wind_speed, float wind_direction, float temperature, float humidity, float pressure, float param1, float param2, float param3, float param4)
{
    _ttalink_weather_station_sensors_data_send(dst_addr, src_addr, chan, update_time, wind_speed, wind_direction, temperature, humidity, pressure, param1, param2, param3, param4, true);
}


static inline void _ttalink_weather_station_sensors_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_weather_station_sensors_data_t* weather_station_sensors_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_weather_station_sensors_data_send_nocrc(dst_addr, src_addr, chan, weather_station_sensors_data->update_time, weather_station_sensors_data->wind_speed, weather_station_sensors_data->wind_direction, weather_station_sensors_data->temperature, weather_station_sensors_data->humidity, weather_station_sensors_data->pressure, weather_station_sensors_data->param1, weather_station_sensors_data->param2, weather_station_sensors_data->param3, weather_station_sensors_data->param4);
    }else{
        ttalink_weather_station_sensors_data_send(dst_addr, src_addr, chan, weather_station_sensors_data->update_time, weather_station_sensors_data->wind_speed, weather_station_sensors_data->wind_direction, weather_station_sensors_data->temperature, weather_station_sensors_data->humidity, weather_station_sensors_data->pressure, weather_station_sensors_data->param1, weather_station_sensors_data->param2, weather_station_sensors_data->param3, weather_station_sensors_data->param4);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA, (const char *)weather_station_sensors_data, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a weather_station_sensors_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_weather_station_sensors_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_weather_station_sensors_data_t* weather_station_sensors_data)
{
    _ttalink_weather_station_sensors_data_send_struct(dst_addr, src_addr, chan, weather_station_sensors_data, false);
}

/**
 * @brief Send a weather_station_sensors_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_weather_station_sensors_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_weather_station_sensors_data_t* weather_station_sensors_data)
{
    _ttalink_weather_station_sensors_data_send_struct(dst_addr, src_addr, chan, weather_station_sensors_data, true);
}

#if TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_weather_station_sensors_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, float wind_speed, float wind_direction, float temperature, float humidity, float pressure, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, wind_speed);
    _tta_put_float(buf, 8, wind_direction);
    _tta_put_float(buf, 12, temperature);
    _tta_put_float(buf, 16, humidity);
    _tta_put_float(buf, 20, pressure);
    _tta_put_float(buf, 24, param1);
    _tta_put_float(buf, 28, param2);
    _tta_put_float(buf, 32, param3);
    _tta_put_float(buf, 36, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA, buf, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_CRC, nocrc);
#else
    ttalink_weather_station_sensors_data_t *packet = (ttalink_weather_station_sensors_data_t *)msgbuf;
    packet->update_time = update_time;
    packet->wind_speed = wind_speed;
    packet->wind_direction = wind_direction;
    packet->temperature = temperature;
    packet->humidity = humidity;
    packet->pressure = pressure;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA, (const char *)packet, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_weather_station_sensors_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, float wind_speed, float wind_direction, float temperature, float humidity, float pressure, float param1, float param2, float param3, float param4)
{
    _ttalink_weather_station_sensors_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, wind_speed, wind_direction, temperature, humidity, pressure, param1, param2, param3, param4, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_weather_station_sensors_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, float wind_speed, float wind_direction, float temperature, float humidity, float pressure, float param1, float param2, float param3, float param4)
{
    _ttalink_weather_station_sensors_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, wind_speed, wind_direction, temperature, humidity, pressure, param1, param2, param3, param4, true);
}
#endif

#endif

// MESSAGE WEATHER_STATION_SENSORS_DATA UNPACKING


/**
 * @brief Get field update_time from weather_station_sensors_data message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_weather_station_sensors_data_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field wind_speed from weather_station_sensors_data message
 *
 * @return  风速.
 */
static inline float ttalink_weather_station_sensors_data_get_wind_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  4);
}

/**
 * @brief Get field wind_direction from weather_station_sensors_data message
 *
 * @return  风向.
 */
static inline float ttalink_weather_station_sensors_data_get_wind_direction(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  8);
}

/**
 * @brief Get field temperature from weather_station_sensors_data message
 *
 * @return  温度.
 */
static inline float ttalink_weather_station_sensors_data_get_temperature(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  12);
}

/**
 * @brief Get field humidity from weather_station_sensors_data message
 *
 * @return  湿度.
 */
static inline float ttalink_weather_station_sensors_data_get_humidity(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field pressure from weather_station_sensors_data message
 *
 * @return  气压.
 */
static inline float ttalink_weather_station_sensors_data_get_pressure(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  20);
}

/**
 * @brief Get field param1 from weather_station_sensors_data message
 *
 * @return  预留.
 */
static inline float ttalink_weather_station_sensors_data_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Get field param2 from weather_station_sensors_data message
 *
 * @return  预留.
 */
static inline float ttalink_weather_station_sensors_data_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  28);
}

/**
 * @brief Get field param3 from weather_station_sensors_data message
 *
 * @return  预留.
 */
static inline float ttalink_weather_station_sensors_data_get_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Get field param4 from weather_station_sensors_data message
 *
 * @return  预留.
 */
static inline float ttalink_weather_station_sensors_data_get_param4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  36);
}

/**
 * @brief Decode a weather_station_sensors_data message into a struct
 *
 * @param msg The message to decode
 * @param weather_station_sensors_data C-struct to decode the message contents into
 */
static inline void ttalink_weather_station_sensors_data_decode(const ttalink_message_t* msg, ttalink_weather_station_sensors_data_t* weather_station_sensors_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    weather_station_sensors_data->update_time = ttalink_weather_station_sensors_data_get_update_time(msg);
    weather_station_sensors_data->wind_speed = ttalink_weather_station_sensors_data_get_wind_speed(msg);
    weather_station_sensors_data->wind_direction = ttalink_weather_station_sensors_data_get_wind_direction(msg);
    weather_station_sensors_data->temperature = ttalink_weather_station_sensors_data_get_temperature(msg);
    weather_station_sensors_data->humidity = ttalink_weather_station_sensors_data_get_humidity(msg);
    weather_station_sensors_data->pressure = ttalink_weather_station_sensors_data_get_pressure(msg);
    weather_station_sensors_data->param1 = ttalink_weather_station_sensors_data_get_param1(msg);
    weather_station_sensors_data->param2 = ttalink_weather_station_sensors_data_get_param2(msg);
    weather_station_sensors_data->param3 = ttalink_weather_station_sensors_data_get_param3(msg);
    weather_station_sensors_data->param4 = ttalink_weather_station_sensors_data_get_param4(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN? msg->len : TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN;
        memset(weather_station_sensors_data, 0, TTALINK_MSG_ID_WEATHER_STATION_SENSORS_DATA_LEN);
    memcpy(weather_station_sensors_data, _TTA_PAYLOAD(msg), len);
#endif
}
