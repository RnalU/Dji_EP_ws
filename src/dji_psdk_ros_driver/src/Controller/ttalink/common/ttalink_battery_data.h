#pragma once
// MESSAGE BATTERY_DATA PACKING

#define TTALINK_MSG_ID_BATTERY_DATA 2005

TTAPACKED(
typedef struct __ttalink_battery_data_t {
 uint8_t battery_num; /*< 总电池数.*/
 uint8_t battery_index; /*< This is battery number.*/
 uint8_t cell; /*< 电池节数.*/
 float temperature; /*< This is the battery temperature.*/
 float voltage_data; /*< This is mathematical expectation of voltage.*/
 float voltage[18]; /*< This is single core voltage.*/
 float current; /*< This is percentage of current.*/
 uint16_t remain; /*< This is percentage of Residual Electricity.*/
 uint32_t capacity; /*< This is Electricity.*/
 uint32_t full_capacity; /*< This is full charge capacity.*/
 uint16_t cycle; /*< This is cycle times.*/
 uint32_t design_capacity; /*< This is battery design capacity.*/
 uint32_t factory_data; /*< This is date of manufacture.*/
 uint32_t SN; /*< This is serial number.*/
 uint32_t battery_err_info; /*< This is battery infomation.*/
 uint32_t time_remain; /*< This is the remain time.*/
}) ttalink_battery_data_t;

#define TTALINK_MSG_ID_BATTERY_DATA_LEN 119
#define TTALINK_MSG_ID_BATTERY_DATA_MIN_LEN 119
#define TTALINK_MSG_ID_2005_LEN 119
#define TTALINK_MSG_ID_2005_MIN_LEN 119

#define TTALINK_MSG_ID_BATTERY_DATA_CRC 207
#define TTALINK_MSG_ID_2005_CRC 207

#define TTALINK_MSG_BATTERY_DATA_FIELD_VOLTAGE_LEN 18

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_BATTERY_DATA { \
    2005, \
    "BATTERY_DATA", \
    16, \
    {  { "battery_num", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_battery_data_t, battery_num) }, \
         { "battery_index", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_battery_data_t, battery_index) }, \
         { "cell", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_battery_data_t, cell) }, \
         { "temperature", NULL, TTALINK_TYPE_FLOAT, 0, 3, offsetof(ttalink_battery_data_t, temperature) }, \
         { "voltage_data", NULL, TTALINK_TYPE_FLOAT, 0, 7, offsetof(ttalink_battery_data_t, voltage_data) }, \
         { "voltage", NULL, TTALINK_TYPE_FLOAT, 18, 11, offsetof(ttalink_battery_data_t, voltage) }, \
         { "current", NULL, TTALINK_TYPE_FLOAT, 0, 83, offsetof(ttalink_battery_data_t, current) }, \
         { "remain", NULL, TTALINK_TYPE_UINT16_T, 0, 87, offsetof(ttalink_battery_data_t, remain) }, \
         { "capacity", NULL, TTALINK_TYPE_UINT32_T, 0, 89, offsetof(ttalink_battery_data_t, capacity) }, \
         { "full_capacity", NULL, TTALINK_TYPE_UINT32_T, 0, 93, offsetof(ttalink_battery_data_t, full_capacity) }, \
         { "cycle", NULL, TTALINK_TYPE_UINT16_T, 0, 97, offsetof(ttalink_battery_data_t, cycle) }, \
         { "design_capacity", NULL, TTALINK_TYPE_UINT32_T, 0, 99, offsetof(ttalink_battery_data_t, design_capacity) }, \
         { "factory_data", NULL, TTALINK_TYPE_UINT32_T, 0, 103, offsetof(ttalink_battery_data_t, factory_data) }, \
         { "SN", NULL, TTALINK_TYPE_UINT32_T, 0, 107, offsetof(ttalink_battery_data_t, SN) }, \
         { "battery_err_info", NULL, TTALINK_TYPE_UINT32_T, 0, 111, offsetof(ttalink_battery_data_t, battery_err_info) }, \
         { "time_remain", NULL, TTALINK_TYPE_UINT32_T, 0, 115, offsetof(ttalink_battery_data_t, time_remain) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_BATTERY_DATA { \
    "BATTERY_DATA", \
    16, \
    {  { "battery_num", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_battery_data_t, battery_num) }, \
         { "battery_index", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_battery_data_t, battery_index) }, \
         { "cell", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_battery_data_t, cell) }, \
         { "temperature", NULL, TTALINK_TYPE_FLOAT, 0, 3, offsetof(ttalink_battery_data_t, temperature) }, \
         { "voltage_data", NULL, TTALINK_TYPE_FLOAT, 0, 7, offsetof(ttalink_battery_data_t, voltage_data) }, \
         { "voltage", NULL, TTALINK_TYPE_FLOAT, 18, 11, offsetof(ttalink_battery_data_t, voltage) }, \
         { "current", NULL, TTALINK_TYPE_FLOAT, 0, 83, offsetof(ttalink_battery_data_t, current) }, \
         { "remain", NULL, TTALINK_TYPE_UINT16_T, 0, 87, offsetof(ttalink_battery_data_t, remain) }, \
         { "capacity", NULL, TTALINK_TYPE_UINT32_T, 0, 89, offsetof(ttalink_battery_data_t, capacity) }, \
         { "full_capacity", NULL, TTALINK_TYPE_UINT32_T, 0, 93, offsetof(ttalink_battery_data_t, full_capacity) }, \
         { "cycle", NULL, TTALINK_TYPE_UINT16_T, 0, 97, offsetof(ttalink_battery_data_t, cycle) }, \
         { "design_capacity", NULL, TTALINK_TYPE_UINT32_T, 0, 99, offsetof(ttalink_battery_data_t, design_capacity) }, \
         { "factory_data", NULL, TTALINK_TYPE_UINT32_T, 0, 103, offsetof(ttalink_battery_data_t, factory_data) }, \
         { "SN", NULL, TTALINK_TYPE_UINT32_T, 0, 107, offsetof(ttalink_battery_data_t, SN) }, \
         { "battery_err_info", NULL, TTALINK_TYPE_UINT32_T, 0, 111, offsetof(ttalink_battery_data_t, battery_err_info) }, \
         { "time_remain", NULL, TTALINK_TYPE_UINT32_T, 0, 115, offsetof(ttalink_battery_data_t, time_remain) }, \
         } \
}
#endif


static inline uint16_t _ttalink_battery_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t battery_num, uint8_t battery_index, uint8_t cell, float temperature, float voltage_data, const float *voltage, float current, uint16_t remain, uint32_t capacity, uint32_t full_capacity, uint16_t cycle, uint32_t design_capacity, uint32_t factory_data, uint32_t SN, uint32_t battery_err_info, uint32_t time_remain, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BATTERY_DATA_LEN];
    _tta_put_uint8_t(buf, 0, battery_num);
    _tta_put_uint8_t(buf, 1, battery_index);
    _tta_put_uint8_t(buf, 2, cell);
    _tta_put_float(buf, 3, temperature);
    _tta_put_float(buf, 7, voltage_data);
    _tta_put_float(buf, 83, current);
    _tta_put_uint16_t(buf, 87, remain);
    _tta_put_uint32_t(buf, 89, capacity);
    _tta_put_uint32_t(buf, 93, full_capacity);
    _tta_put_uint16_t(buf, 97, cycle);
    _tta_put_uint32_t(buf, 99, design_capacity);
    _tta_put_uint32_t(buf, 103, factory_data);
    _tta_put_uint32_t(buf, 107, SN);
    _tta_put_uint32_t(buf, 111, battery_err_info);
    _tta_put_uint32_t(buf, 115, time_remain);
    _tta_put_float_array(buf, 11, voltage, 18);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_BATTERY_DATA_LEN);
#else
    ttalink_battery_data_t packet;
    packet.battery_num = battery_num;
    packet.battery_index = battery_index;
    packet.cell = cell;
    packet.temperature = temperature;
    packet.voltage_data = voltage_data;
    packet.current = current;
    packet.remain = remain;
    packet.capacity = capacity;
    packet.full_capacity = full_capacity;
    packet.cycle = cycle;
    packet.design_capacity = design_capacity;
    packet.factory_data = factory_data;
    packet.SN = SN;
    packet.battery_err_info = battery_err_info;
    packet.time_remain = time_remain;
    tta_array_memcpy(packet.voltage, voltage, sizeof(float)*18);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_BATTERY_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_BATTERY_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_BATTERY_DATA_MIN_LEN, TTALINK_MSG_ID_BATTERY_DATA_LEN, TTALINK_MSG_ID_BATTERY_DATA_CRC, nocrc);
}

/**
 * @brief Pack a battery_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param battery_num 总电池数.
 * @param battery_index This is battery number.
 * @param cell 电池节数.
 * @param temperature This is the battery temperature.
 * @param voltage_data This is mathematical expectation of voltage.
 * @param voltage This is single core voltage.
 * @param current This is percentage of current.
 * @param remain This is percentage of Residual Electricity.
 * @param capacity This is Electricity.
 * @param full_capacity This is full charge capacity.
 * @param cycle This is cycle times.
 * @param design_capacity This is battery design capacity.
 * @param factory_data This is date of manufacture.
 * @param SN This is serial number.
 * @param battery_err_info This is battery infomation.
 * @param time_remain This is the remain time.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_battery_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t battery_num, uint8_t battery_index, uint8_t cell, float temperature, float voltage_data, const float *voltage, float current, uint16_t remain, uint32_t capacity, uint32_t full_capacity, uint16_t cycle, uint32_t design_capacity, uint32_t factory_data, uint32_t SN, uint32_t battery_err_info, uint32_t time_remain)
{
    return _ttalink_battery_data_pack(dst_addr, src_addr, msg,  battery_num, battery_index, cell, temperature, voltage_data, voltage, current, remain, capacity, full_capacity, cycle, design_capacity, factory_data, SN, battery_err_info, time_remain, false);
}

/**
 * @brief Pack a battery_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param battery_num 总电池数.
 * @param battery_index This is battery number.
 * @param cell 电池节数.
 * @param temperature This is the battery temperature.
 * @param voltage_data This is mathematical expectation of voltage.
 * @param voltage This is single core voltage.
 * @param current This is percentage of current.
 * @param remain This is percentage of Residual Electricity.
 * @param capacity This is Electricity.
 * @param full_capacity This is full charge capacity.
 * @param cycle This is cycle times.
 * @param design_capacity This is battery design capacity.
 * @param factory_data This is date of manufacture.
 * @param SN This is serial number.
 * @param battery_err_info This is battery infomation.
 * @param time_remain This is the remain time.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_battery_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t battery_num, uint8_t battery_index, uint8_t cell, float temperature, float voltage_data, const float *voltage, float current, uint16_t remain, uint32_t capacity, uint32_t full_capacity, uint16_t cycle, uint32_t design_capacity, uint32_t factory_data, uint32_t SN, uint32_t battery_err_info, uint32_t time_remain)
{
    return _ttalink_battery_data_pack(dst_addr, src_addr, msg,  battery_num, battery_index, cell, temperature, voltage_data, voltage, current, remain, capacity, full_capacity, cycle, design_capacity, factory_data, SN, battery_err_info, time_remain, true);
}


static inline uint16_t _ttalink_battery_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t battery_num,uint8_t battery_index,uint8_t cell,float temperature,float voltage_data,const float *voltage,float current,uint16_t remain,uint32_t capacity,uint32_t full_capacity,uint16_t cycle,uint32_t design_capacity,uint32_t factory_data,uint32_t SN,uint32_t battery_err_info,uint32_t time_remain, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BATTERY_DATA_LEN];
    _tta_put_uint8_t(buf, 0, battery_num);
    _tta_put_uint8_t(buf, 1, battery_index);
    _tta_put_uint8_t(buf, 2, cell);
    _tta_put_float(buf, 3, temperature);
    _tta_put_float(buf, 7, voltage_data);
    _tta_put_float(buf, 83, current);
    _tta_put_uint16_t(buf, 87, remain);
    _tta_put_uint32_t(buf, 89, capacity);
    _tta_put_uint32_t(buf, 93, full_capacity);
    _tta_put_uint16_t(buf, 97, cycle);
    _tta_put_uint32_t(buf, 99, design_capacity);
    _tta_put_uint32_t(buf, 103, factory_data);
    _tta_put_uint32_t(buf, 107, SN);
    _tta_put_uint32_t(buf, 111, battery_err_info);
    _tta_put_uint32_t(buf, 115, time_remain);
    _tta_put_float_array(buf, 11, voltage, 18);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_BATTERY_DATA_LEN);
#else
    ttalink_battery_data_t packet;
    packet.battery_num = battery_num;
    packet.battery_index = battery_index;
    packet.cell = cell;
    packet.temperature = temperature;
    packet.voltage_data = voltage_data;
    packet.current = current;
    packet.remain = remain;
    packet.capacity = capacity;
    packet.full_capacity = full_capacity;
    packet.cycle = cycle;
    packet.design_capacity = design_capacity;
    packet.factory_data = factory_data;
    packet.SN = SN;
    packet.battery_err_info = battery_err_info;
    packet.time_remain = time_remain;
    tta_array_memcpy(packet.voltage, voltage, sizeof(float)*18);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_BATTERY_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_BATTERY_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_BATTERY_DATA_MIN_LEN, TTALINK_MSG_ID_BATTERY_DATA_LEN, TTALINK_MSG_ID_BATTERY_DATA_CRC, nocrc);
}

/**
 * @brief Pack a battery_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param battery_num 总电池数.
 * @param battery_index This is battery number.
 * @param cell 电池节数.
 * @param temperature This is the battery temperature.
 * @param voltage_data This is mathematical expectation of voltage.
 * @param voltage This is single core voltage.
 * @param current This is percentage of current.
 * @param remain This is percentage of Residual Electricity.
 * @param capacity This is Electricity.
 * @param full_capacity This is full charge capacity.
 * @param cycle This is cycle times.
 * @param design_capacity This is battery design capacity.
 * @param factory_data This is date of manufacture.
 * @param SN This is serial number.
 * @param battery_err_info This is battery infomation.
 * @param time_remain This is the remain time.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_battery_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t battery_num,uint8_t battery_index,uint8_t cell,float temperature,float voltage_data,const float *voltage,float current,uint16_t remain,uint32_t capacity,uint32_t full_capacity,uint16_t cycle,uint32_t design_capacity,uint32_t factory_data,uint32_t SN,uint32_t battery_err_info,uint32_t time_remain)
{
    return _ttalink_battery_data_pack_chan(dst_addr, src_addr, chan, msg,  battery_num, battery_index, cell, temperature, voltage_data, voltage, current, remain, capacity, full_capacity, cycle, design_capacity, factory_data, SN, battery_err_info, time_remain, false);
}

/**
 * @brief Pack a battery_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param battery_num 总电池数.
 * @param battery_index This is battery number.
 * @param cell 电池节数.
 * @param temperature This is the battery temperature.
 * @param voltage_data This is mathematical expectation of voltage.
 * @param voltage This is single core voltage.
 * @param current This is percentage of current.
 * @param remain This is percentage of Residual Electricity.
 * @param capacity This is Electricity.
 * @param full_capacity This is full charge capacity.
 * @param cycle This is cycle times.
 * @param design_capacity This is battery design capacity.
 * @param factory_data This is date of manufacture.
 * @param SN This is serial number.
 * @param battery_err_info This is battery infomation.
 * @param time_remain This is the remain time.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_battery_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t battery_num,uint8_t battery_index,uint8_t cell,float temperature,float voltage_data,const float *voltage,float current,uint16_t remain,uint32_t capacity,uint32_t full_capacity,uint16_t cycle,uint32_t design_capacity,uint32_t factory_data,uint32_t SN,uint32_t battery_err_info,uint32_t time_remain)
{
    return _ttalink_battery_data_pack_chan(dst_addr, src_addr, chan, msg,  battery_num, battery_index, cell, temperature, voltage_data, voltage, current, remain, capacity, full_capacity, cycle, design_capacity, factory_data, SN, battery_err_info, time_remain, true);
}


static inline uint16_t _ttalink_battery_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_battery_data_t* battery_data, bool nocrc)
{
    if(nocrc){
        return ttalink_battery_data_pack_nocrc(dst_addr, src_addr, msg, battery_data->battery_num, battery_data->battery_index, battery_data->cell, battery_data->temperature, battery_data->voltage_data, battery_data->voltage, battery_data->current, battery_data->remain, battery_data->capacity, battery_data->full_capacity, battery_data->cycle, battery_data->design_capacity, battery_data->factory_data, battery_data->SN, battery_data->battery_err_info, battery_data->time_remain);
    }else{
        return ttalink_battery_data_pack(dst_addr, src_addr, msg, battery_data->battery_num, battery_data->battery_index, battery_data->cell, battery_data->temperature, battery_data->voltage_data, battery_data->voltage, battery_data->current, battery_data->remain, battery_data->capacity, battery_data->full_capacity, battery_data->cycle, battery_data->design_capacity, battery_data->factory_data, battery_data->SN, battery_data->battery_err_info, battery_data->time_remain);
    }
    
}

/**
 * @brief Encode a battery_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param battery_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_battery_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_battery_data_t* battery_data)
{
    return _ttalink_battery_data_encode(dst_addr, src_addr, msg, battery_data, false);
}

/**
 * @brief Encode a battery_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param battery_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_battery_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_battery_data_t* battery_data)
{
    return _ttalink_battery_data_encode(dst_addr, src_addr, msg, battery_data, true);
}


static inline uint16_t _ttalink_battery_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_battery_data_t* battery_data, bool nocrc)
{
    if(nocrc){
        return ttalink_battery_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, battery_data->battery_num, battery_data->battery_index, battery_data->cell, battery_data->temperature, battery_data->voltage_data, battery_data->voltage, battery_data->current, battery_data->remain, battery_data->capacity, battery_data->full_capacity, battery_data->cycle, battery_data->design_capacity, battery_data->factory_data, battery_data->SN, battery_data->battery_err_info, battery_data->time_remain);
    }else{
        return ttalink_battery_data_pack_chan(dst_addr, src_addr, chan, msg, battery_data->battery_num, battery_data->battery_index, battery_data->cell, battery_data->temperature, battery_data->voltage_data, battery_data->voltage, battery_data->current, battery_data->remain, battery_data->capacity, battery_data->full_capacity, battery_data->cycle, battery_data->design_capacity, battery_data->factory_data, battery_data->SN, battery_data->battery_err_info, battery_data->time_remain);
    }
}

/**
 * @brief Encode a battery_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param battery_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_battery_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_battery_data_t* battery_data)
{
    return _ttalink_battery_data_encode_chan(dst_addr, src_addr, chan, msg, battery_data, false);
}

/**
 * @brief Encode a battery_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param battery_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_battery_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_battery_data_t* battery_data)
{
    return _ttalink_battery_data_encode_chan(dst_addr, src_addr, chan, msg, battery_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_battery_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t battery_num, uint8_t battery_index, uint8_t cell, float temperature, float voltage_data, const float *voltage, float current, uint16_t remain, uint32_t capacity, uint32_t full_capacity, uint16_t cycle, uint32_t design_capacity, uint32_t factory_data, uint32_t SN, uint32_t battery_err_info, uint32_t time_remain, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BATTERY_DATA_LEN];
    _tta_put_uint8_t(buf, 0, battery_num);
    _tta_put_uint8_t(buf, 1, battery_index);
    _tta_put_uint8_t(buf, 2, cell);
    _tta_put_float(buf, 3, temperature);
    _tta_put_float(buf, 7, voltage_data);
    _tta_put_float(buf, 83, current);
    _tta_put_uint16_t(buf, 87, remain);
    _tta_put_uint32_t(buf, 89, capacity);
    _tta_put_uint32_t(buf, 93, full_capacity);
    _tta_put_uint16_t(buf, 97, cycle);
    _tta_put_uint32_t(buf, 99, design_capacity);
    _tta_put_uint32_t(buf, 103, factory_data);
    _tta_put_uint32_t(buf, 107, SN);
    _tta_put_uint32_t(buf, 111, battery_err_info);
    _tta_put_uint32_t(buf, 115, time_remain);
    _tta_put_float_array(buf, 11, voltage, 18);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BATTERY_DATA, buf, TTALINK_MSG_ID_BATTERY_DATA_MIN_LEN, TTALINK_MSG_ID_BATTERY_DATA_LEN, TTALINK_MSG_ID_BATTERY_DATA_CRC, nocrc);
#else
    ttalink_battery_data_t packet;
    packet.battery_num = battery_num;
    packet.battery_index = battery_index;
    packet.cell = cell;
    packet.temperature = temperature;
    packet.voltage_data = voltage_data;
    packet.current = current;
    packet.remain = remain;
    packet.capacity = capacity;
    packet.full_capacity = full_capacity;
    packet.cycle = cycle;
    packet.design_capacity = design_capacity;
    packet.factory_data = factory_data;
    packet.SN = SN;
    packet.battery_err_info = battery_err_info;
    packet.time_remain = time_remain;
    tta_array_memcpy(packet.voltage, voltage, sizeof(float)*18);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BATTERY_DATA, (const char *)&packet, TTALINK_MSG_ID_BATTERY_DATA_MIN_LEN, TTALINK_MSG_ID_BATTERY_DATA_LEN, TTALINK_MSG_ID_BATTERY_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a battery_data message
 * @param chan TTAlink channel to send the message
 *
 * @param battery_num 总电池数.
 * @param battery_index This is battery number.
 * @param cell 电池节数.
 * @param temperature This is the battery temperature.
 * @param voltage_data This is mathematical expectation of voltage.
 * @param voltage This is single core voltage.
 * @param current This is percentage of current.
 * @param remain This is percentage of Residual Electricity.
 * @param capacity This is Electricity.
 * @param full_capacity This is full charge capacity.
 * @param cycle This is cycle times.
 * @param design_capacity This is battery design capacity.
 * @param factory_data This is date of manufacture.
 * @param SN This is serial number.
 * @param battery_err_info This is battery infomation.
 * @param time_remain This is the remain time.
 */
static inline void ttalink_battery_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t battery_num, uint8_t battery_index, uint8_t cell, float temperature, float voltage_data, const float *voltage, float current, uint16_t remain, uint32_t capacity, uint32_t full_capacity, uint16_t cycle, uint32_t design_capacity, uint32_t factory_data, uint32_t SN, uint32_t battery_err_info, uint32_t time_remain)
{
    _ttalink_battery_data_send(dst_addr, src_addr, chan, battery_num, battery_index, cell, temperature, voltage_data, voltage, current, remain, capacity, full_capacity, cycle, design_capacity, factory_data, SN, battery_err_info, time_remain, false);
}

/**
 * @brief Send a battery_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param battery_num 总电池数.
 * @param battery_index This is battery number.
 * @param cell 电池节数.
 * @param temperature This is the battery temperature.
 * @param voltage_data This is mathematical expectation of voltage.
 * @param voltage This is single core voltage.
 * @param current This is percentage of current.
 * @param remain This is percentage of Residual Electricity.
 * @param capacity This is Electricity.
 * @param full_capacity This is full charge capacity.
 * @param cycle This is cycle times.
 * @param design_capacity This is battery design capacity.
 * @param factory_data This is date of manufacture.
 * @param SN This is serial number.
 * @param battery_err_info This is battery infomation.
 * @param time_remain This is the remain time.
 */
static inline void ttalink_battery_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t battery_num, uint8_t battery_index, uint8_t cell, float temperature, float voltage_data, const float *voltage, float current, uint16_t remain, uint32_t capacity, uint32_t full_capacity, uint16_t cycle, uint32_t design_capacity, uint32_t factory_data, uint32_t SN, uint32_t battery_err_info, uint32_t time_remain)
{
    _ttalink_battery_data_send(dst_addr, src_addr, chan, battery_num, battery_index, cell, temperature, voltage_data, voltage, current, remain, capacity, full_capacity, cycle, design_capacity, factory_data, SN, battery_err_info, time_remain, true);
}


static inline void _ttalink_battery_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_battery_data_t* battery_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_battery_data_send_nocrc(dst_addr, src_addr, chan, battery_data->battery_num, battery_data->battery_index, battery_data->cell, battery_data->temperature, battery_data->voltage_data, battery_data->voltage, battery_data->current, battery_data->remain, battery_data->capacity, battery_data->full_capacity, battery_data->cycle, battery_data->design_capacity, battery_data->factory_data, battery_data->SN, battery_data->battery_err_info, battery_data->time_remain);
    }else{
        ttalink_battery_data_send(dst_addr, src_addr, chan, battery_data->battery_num, battery_data->battery_index, battery_data->cell, battery_data->temperature, battery_data->voltage_data, battery_data->voltage, battery_data->current, battery_data->remain, battery_data->capacity, battery_data->full_capacity, battery_data->cycle, battery_data->design_capacity, battery_data->factory_data, battery_data->SN, battery_data->battery_err_info, battery_data->time_remain);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BATTERY_DATA, (const char *)battery_data, TTALINK_MSG_ID_BATTERY_DATA_MIN_LEN, TTALINK_MSG_ID_BATTERY_DATA_LEN, TTALINK_MSG_ID_BATTERY_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a battery_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_battery_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_battery_data_t* battery_data)
{
    _ttalink_battery_data_send_struct(dst_addr, src_addr, chan, battery_data, false);
}

/**
 * @brief Send a battery_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_battery_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_battery_data_t* battery_data)
{
    _ttalink_battery_data_send_struct(dst_addr, src_addr, chan, battery_data, true);
}

#if TTALINK_MSG_ID_BATTERY_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_battery_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t battery_num, uint8_t battery_index, uint8_t cell, float temperature, float voltage_data, const float *voltage, float current, uint16_t remain, uint32_t capacity, uint32_t full_capacity, uint16_t cycle, uint32_t design_capacity, uint32_t factory_data, uint32_t SN, uint32_t battery_err_info, uint32_t time_remain, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, battery_num);
    _tta_put_uint8_t(buf, 1, battery_index);
    _tta_put_uint8_t(buf, 2, cell);
    _tta_put_float(buf, 3, temperature);
    _tta_put_float(buf, 7, voltage_data);
    _tta_put_float(buf, 83, current);
    _tta_put_uint16_t(buf, 87, remain);
    _tta_put_uint32_t(buf, 89, capacity);
    _tta_put_uint32_t(buf, 93, full_capacity);
    _tta_put_uint16_t(buf, 97, cycle);
    _tta_put_uint32_t(buf, 99, design_capacity);
    _tta_put_uint32_t(buf, 103, factory_data);
    _tta_put_uint32_t(buf, 107, SN);
    _tta_put_uint32_t(buf, 111, battery_err_info);
    _tta_put_uint32_t(buf, 115, time_remain);
    _tta_put_float_array(buf, 11, voltage, 18);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BATTERY_DATA, buf, TTALINK_MSG_ID_BATTERY_DATA_MIN_LEN, TTALINK_MSG_ID_BATTERY_DATA_LEN, TTALINK_MSG_ID_BATTERY_DATA_CRC, nocrc);
#else
    ttalink_battery_data_t *packet = (ttalink_battery_data_t *)msgbuf;
    packet->battery_num = battery_num;
    packet->battery_index = battery_index;
    packet->cell = cell;
    packet->temperature = temperature;
    packet->voltage_data = voltage_data;
    packet->current = current;
    packet->remain = remain;
    packet->capacity = capacity;
    packet->full_capacity = full_capacity;
    packet->cycle = cycle;
    packet->design_capacity = design_capacity;
    packet->factory_data = factory_data;
    packet->SN = SN;
    packet->battery_err_info = battery_err_info;
    packet->time_remain = time_remain;
    tta_array_memcpy(packet->voltage, voltage, sizeof(float)*18);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BATTERY_DATA, (const char *)packet, TTALINK_MSG_ID_BATTERY_DATA_MIN_LEN, TTALINK_MSG_ID_BATTERY_DATA_LEN, TTALINK_MSG_ID_BATTERY_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_battery_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t battery_num, uint8_t battery_index, uint8_t cell, float temperature, float voltage_data, const float *voltage, float current, uint16_t remain, uint32_t capacity, uint32_t full_capacity, uint16_t cycle, uint32_t design_capacity, uint32_t factory_data, uint32_t SN, uint32_t battery_err_info, uint32_t time_remain)
{
    _ttalink_battery_data_send_buf(dst_addr, src_addr, msgbuf, chan, battery_num, battery_index, cell, temperature, voltage_data, voltage, current, remain, capacity, full_capacity, cycle, design_capacity, factory_data, SN, battery_err_info, time_remain, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_battery_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t battery_num, uint8_t battery_index, uint8_t cell, float temperature, float voltage_data, const float *voltage, float current, uint16_t remain, uint32_t capacity, uint32_t full_capacity, uint16_t cycle, uint32_t design_capacity, uint32_t factory_data, uint32_t SN, uint32_t battery_err_info, uint32_t time_remain)
{
    _ttalink_battery_data_send_buf(dst_addr, src_addr, msgbuf, chan, battery_num, battery_index, cell, temperature, voltage_data, voltage, current, remain, capacity, full_capacity, cycle, design_capacity, factory_data, SN, battery_err_info, time_remain, true);
}
#endif

#endif

// MESSAGE BATTERY_DATA UNPACKING


/**
 * @brief Get field battery_num from battery_data message
 *
 * @return 总电池数.
 */
static inline uint8_t ttalink_battery_data_get_battery_num(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field battery_index from battery_data message
 *
 * @return This is battery number.
 */
static inline uint8_t ttalink_battery_data_get_battery_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field cell from battery_data message
 *
 * @return 电池节数.
 */
static inline uint8_t ttalink_battery_data_get_cell(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field temperature from battery_data message
 *
 * @return This is the battery temperature.
 */
static inline float ttalink_battery_data_get_temperature(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  3);
}

/**
 * @brief Get field voltage_data from battery_data message
 *
 * @return This is mathematical expectation of voltage.
 */
static inline float ttalink_battery_data_get_voltage_data(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  7);
}

/**
 * @brief Get field voltage from battery_data message
 *
 * @return This is single core voltage.
 */
static inline uint16_t ttalink_battery_data_get_voltage(const ttalink_message_t* msg, float *voltage)
{
    return _TTA_RETURN_float_array(msg, voltage, 18,  11);
}

/**
 * @brief Get field current from battery_data message
 *
 * @return This is percentage of current.
 */
static inline float ttalink_battery_data_get_current(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  83);
}

/**
 * @brief Get field remain from battery_data message
 *
 * @return This is percentage of Residual Electricity.
 */
static inline uint16_t ttalink_battery_data_get_remain(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  87);
}

/**
 * @brief Get field capacity from battery_data message
 *
 * @return This is Electricity.
 */
static inline uint32_t ttalink_battery_data_get_capacity(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  89);
}

/**
 * @brief Get field full_capacity from battery_data message
 *
 * @return This is full charge capacity.
 */
static inline uint32_t ttalink_battery_data_get_full_capacity(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  93);
}

/**
 * @brief Get field cycle from battery_data message
 *
 * @return This is cycle times.
 */
static inline uint16_t ttalink_battery_data_get_cycle(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  97);
}

/**
 * @brief Get field design_capacity from battery_data message
 *
 * @return This is battery design capacity.
 */
static inline uint32_t ttalink_battery_data_get_design_capacity(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  99);
}

/**
 * @brief Get field factory_data from battery_data message
 *
 * @return This is date of manufacture.
 */
static inline uint32_t ttalink_battery_data_get_factory_data(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  103);
}

/**
 * @brief Get field SN from battery_data message
 *
 * @return This is serial number.
 */
static inline uint32_t ttalink_battery_data_get_SN(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  107);
}

/**
 * @brief Get field battery_err_info from battery_data message
 *
 * @return This is battery infomation.
 */
static inline uint32_t ttalink_battery_data_get_battery_err_info(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  111);
}

/**
 * @brief Get field time_remain from battery_data message
 *
 * @return This is the remain time.
 */
static inline uint32_t ttalink_battery_data_get_time_remain(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  115);
}

/**
 * @brief Decode a battery_data message into a struct
 *
 * @param msg The message to decode
 * @param battery_data C-struct to decode the message contents into
 */
static inline void ttalink_battery_data_decode(const ttalink_message_t* msg, ttalink_battery_data_t* battery_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    battery_data->battery_num = ttalink_battery_data_get_battery_num(msg);
    battery_data->battery_index = ttalink_battery_data_get_battery_index(msg);
    battery_data->cell = ttalink_battery_data_get_cell(msg);
    battery_data->temperature = ttalink_battery_data_get_temperature(msg);
    battery_data->voltage_data = ttalink_battery_data_get_voltage_data(msg);
    ttalink_battery_data_get_voltage(msg, battery_data->voltage);
    battery_data->current = ttalink_battery_data_get_current(msg);
    battery_data->remain = ttalink_battery_data_get_remain(msg);
    battery_data->capacity = ttalink_battery_data_get_capacity(msg);
    battery_data->full_capacity = ttalink_battery_data_get_full_capacity(msg);
    battery_data->cycle = ttalink_battery_data_get_cycle(msg);
    battery_data->design_capacity = ttalink_battery_data_get_design_capacity(msg);
    battery_data->factory_data = ttalink_battery_data_get_factory_data(msg);
    battery_data->SN = ttalink_battery_data_get_SN(msg);
    battery_data->battery_err_info = ttalink_battery_data_get_battery_err_info(msg);
    battery_data->time_remain = ttalink_battery_data_get_time_remain(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_BATTERY_DATA_LEN? msg->len : TTALINK_MSG_ID_BATTERY_DATA_LEN;
        memset(battery_data, 0, TTALINK_MSG_ID_BATTERY_DATA_LEN);
    memcpy(battery_data, _TTA_PAYLOAD(msg), len);
#endif
}
