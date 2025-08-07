#pragma once
// MESSAGE BMS_DATA PACKING

#define TTALINK_MSG_ID_BMS_DATA 2052

TTAPACKED(
typedef struct __ttalink_bms_data_t {
 uint32_t update_time; /*<  . .*/
 int16_t core_temperature; /*<  . .*/
 int16_t plate_temperature; /*<  . .*/
 uint8_t cell; /*<  . */
 uint16_t voltage; /*<  . */
 uint16_t voltage_cell[18]; /*<  . */
 int16_t current; /*<  . .*/
 uint8_t bms_state; /*<  . */
 uint8_t bms_addr; /*<  . */
 uint32_t remaining_capacity; /*<  . .*/
 uint32_t full_capacity; /*<  . .*/
 uint8_t charge_state; /*<  . */
 uint16_t charging_voltage; /*<  . .*/
 uint16_t charging_current[3]; /*<  . .*/
}) ttalink_bms_data_t;

#define TTALINK_MSG_ID_BMS_DATA_LEN 68
#define TTALINK_MSG_ID_BMS_DATA_MIN_LEN 68
#define TTALINK_MSG_ID_2052_LEN 68
#define TTALINK_MSG_ID_2052_MIN_LEN 68

#define TTALINK_MSG_ID_BMS_DATA_CRC 223
#define TTALINK_MSG_ID_2052_CRC 223

#define TTALINK_MSG_BMS_DATA_FIELD_VOLTAGE_CELL_LEN 18
#define TTALINK_MSG_BMS_DATA_FIELD_CHARGING_CURRENT_LEN 3

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_BMS_DATA { \
    2052, \
    "BMS_DATA", \
    14, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_bms_data_t, update_time) }, \
         { "core_temperature", NULL, TTALINK_TYPE_INT16_T, 0, 4, offsetof(ttalink_bms_data_t, core_temperature) }, \
         { "plate_temperature", NULL, TTALINK_TYPE_INT16_T, 0, 6, offsetof(ttalink_bms_data_t, plate_temperature) }, \
         { "cell", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_bms_data_t, cell) }, \
         { "voltage", NULL, TTALINK_TYPE_UINT16_T, 0, 9, offsetof(ttalink_bms_data_t, voltage) }, \
         { "voltage_cell", NULL, TTALINK_TYPE_UINT16_T, 18, 11, offsetof(ttalink_bms_data_t, voltage_cell) }, \
         { "current", NULL, TTALINK_TYPE_INT16_T, 0, 47, offsetof(ttalink_bms_data_t, current) }, \
         { "bms_state", NULL, TTALINK_TYPE_UINT8_T, 0, 49, offsetof(ttalink_bms_data_t, bms_state) }, \
         { "bms_addr", NULL, TTALINK_TYPE_UINT8_T, 0, 50, offsetof(ttalink_bms_data_t, bms_addr) }, \
         { "remaining_capacity", NULL, TTALINK_TYPE_UINT32_T, 0, 51, offsetof(ttalink_bms_data_t, remaining_capacity) }, \
         { "full_capacity", NULL, TTALINK_TYPE_UINT32_T, 0, 55, offsetof(ttalink_bms_data_t, full_capacity) }, \
         { "charge_state", NULL, TTALINK_TYPE_UINT8_T, 0, 59, offsetof(ttalink_bms_data_t, charge_state) }, \
         { "charging_voltage", NULL, TTALINK_TYPE_UINT16_T, 0, 60, offsetof(ttalink_bms_data_t, charging_voltage) }, \
         { "charging_current", NULL, TTALINK_TYPE_UINT16_T, 3, 62, offsetof(ttalink_bms_data_t, charging_current) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_BMS_DATA { \
    "BMS_DATA", \
    14, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_bms_data_t, update_time) }, \
         { "core_temperature", NULL, TTALINK_TYPE_INT16_T, 0, 4, offsetof(ttalink_bms_data_t, core_temperature) }, \
         { "plate_temperature", NULL, TTALINK_TYPE_INT16_T, 0, 6, offsetof(ttalink_bms_data_t, plate_temperature) }, \
         { "cell", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_bms_data_t, cell) }, \
         { "voltage", NULL, TTALINK_TYPE_UINT16_T, 0, 9, offsetof(ttalink_bms_data_t, voltage) }, \
         { "voltage_cell", NULL, TTALINK_TYPE_UINT16_T, 18, 11, offsetof(ttalink_bms_data_t, voltage_cell) }, \
         { "current", NULL, TTALINK_TYPE_INT16_T, 0, 47, offsetof(ttalink_bms_data_t, current) }, \
         { "bms_state", NULL, TTALINK_TYPE_UINT8_T, 0, 49, offsetof(ttalink_bms_data_t, bms_state) }, \
         { "bms_addr", NULL, TTALINK_TYPE_UINT8_T, 0, 50, offsetof(ttalink_bms_data_t, bms_addr) }, \
         { "remaining_capacity", NULL, TTALINK_TYPE_UINT32_T, 0, 51, offsetof(ttalink_bms_data_t, remaining_capacity) }, \
         { "full_capacity", NULL, TTALINK_TYPE_UINT32_T, 0, 55, offsetof(ttalink_bms_data_t, full_capacity) }, \
         { "charge_state", NULL, TTALINK_TYPE_UINT8_T, 0, 59, offsetof(ttalink_bms_data_t, charge_state) }, \
         { "charging_voltage", NULL, TTALINK_TYPE_UINT16_T, 0, 60, offsetof(ttalink_bms_data_t, charging_voltage) }, \
         { "charging_current", NULL, TTALINK_TYPE_UINT16_T, 3, 62, offsetof(ttalink_bms_data_t, charging_current) }, \
         } \
}
#endif


static inline uint16_t _ttalink_bms_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, int16_t core_temperature, int16_t plate_temperature, uint8_t cell, uint16_t voltage, const uint16_t *voltage_cell, int16_t current, uint8_t bms_state, uint8_t bms_addr, uint32_t remaining_capacity, uint32_t full_capacity, uint8_t charge_state, uint16_t charging_voltage, const uint16_t *charging_current, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BMS_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int16_t(buf, 4, core_temperature);
    _tta_put_int16_t(buf, 6, plate_temperature);
    _tta_put_uint8_t(buf, 8, cell);
    _tta_put_uint16_t(buf, 9, voltage);
    _tta_put_int16_t(buf, 47, current);
    _tta_put_uint8_t(buf, 49, bms_state);
    _tta_put_uint8_t(buf, 50, bms_addr);
    _tta_put_uint32_t(buf, 51, remaining_capacity);
    _tta_put_uint32_t(buf, 55, full_capacity);
    _tta_put_uint8_t(buf, 59, charge_state);
    _tta_put_uint16_t(buf, 60, charging_voltage);
    _tta_put_uint16_t_array(buf, 11, voltage_cell, 18);
    _tta_put_uint16_t_array(buf, 62, charging_current, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_BMS_DATA_LEN);
#else
    ttalink_bms_data_t packet;
    packet.update_time = update_time;
    packet.core_temperature = core_temperature;
    packet.plate_temperature = plate_temperature;
    packet.cell = cell;
    packet.voltage = voltage;
    packet.current = current;
    packet.bms_state = bms_state;
    packet.bms_addr = bms_addr;
    packet.remaining_capacity = remaining_capacity;
    packet.full_capacity = full_capacity;
    packet.charge_state = charge_state;
    packet.charging_voltage = charging_voltage;
    tta_array_memcpy(packet.voltage_cell, voltage_cell, sizeof(uint16_t)*18);
    tta_array_memcpy(packet.charging_current, charging_current, sizeof(uint16_t)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_BMS_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_BMS_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_BMS_DATA_MIN_LEN, TTALINK_MSG_ID_BMS_DATA_LEN, TTALINK_MSG_ID_BMS_DATA_CRC, nocrc);
}

/**
 * @brief Pack a bms_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  . .
 * @param core_temperature  . .
 * @param plate_temperature  . .
 * @param cell  . 
 * @param voltage  . 
 * @param voltage_cell  . 
 * @param current  . .
 * @param bms_state  . 
 * @param bms_addr  . 
 * @param remaining_capacity  . .
 * @param full_capacity  . .
 * @param charge_state  . 
 * @param charging_voltage  . .
 * @param charging_current  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, int16_t core_temperature, int16_t plate_temperature, uint8_t cell, uint16_t voltage, const uint16_t *voltage_cell, int16_t current, uint8_t bms_state, uint8_t bms_addr, uint32_t remaining_capacity, uint32_t full_capacity, uint8_t charge_state, uint16_t charging_voltage, const uint16_t *charging_current)
{
    return _ttalink_bms_data_pack(dst_addr, src_addr, msg,  update_time, core_temperature, plate_temperature, cell, voltage, voltage_cell, current, bms_state, bms_addr, remaining_capacity, full_capacity, charge_state, charging_voltage, charging_current, false);
}

/**
 * @brief Pack a bms_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  . .
 * @param core_temperature  . .
 * @param plate_temperature  . .
 * @param cell  . 
 * @param voltage  . 
 * @param voltage_cell  . 
 * @param current  . .
 * @param bms_state  . 
 * @param bms_addr  . 
 * @param remaining_capacity  . .
 * @param full_capacity  . .
 * @param charge_state  . 
 * @param charging_voltage  . .
 * @param charging_current  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, int16_t core_temperature, int16_t plate_temperature, uint8_t cell, uint16_t voltage, const uint16_t *voltage_cell, int16_t current, uint8_t bms_state, uint8_t bms_addr, uint32_t remaining_capacity, uint32_t full_capacity, uint8_t charge_state, uint16_t charging_voltage, const uint16_t *charging_current)
{
    return _ttalink_bms_data_pack(dst_addr, src_addr, msg,  update_time, core_temperature, plate_temperature, cell, voltage, voltage_cell, current, bms_state, bms_addr, remaining_capacity, full_capacity, charge_state, charging_voltage, charging_current, true);
}


static inline uint16_t _ttalink_bms_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,int16_t core_temperature,int16_t plate_temperature,uint8_t cell,uint16_t voltage,const uint16_t *voltage_cell,int16_t current,uint8_t bms_state,uint8_t bms_addr,uint32_t remaining_capacity,uint32_t full_capacity,uint8_t charge_state,uint16_t charging_voltage,const uint16_t *charging_current, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BMS_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int16_t(buf, 4, core_temperature);
    _tta_put_int16_t(buf, 6, plate_temperature);
    _tta_put_uint8_t(buf, 8, cell);
    _tta_put_uint16_t(buf, 9, voltage);
    _tta_put_int16_t(buf, 47, current);
    _tta_put_uint8_t(buf, 49, bms_state);
    _tta_put_uint8_t(buf, 50, bms_addr);
    _tta_put_uint32_t(buf, 51, remaining_capacity);
    _tta_put_uint32_t(buf, 55, full_capacity);
    _tta_put_uint8_t(buf, 59, charge_state);
    _tta_put_uint16_t(buf, 60, charging_voltage);
    _tta_put_uint16_t_array(buf, 11, voltage_cell, 18);
    _tta_put_uint16_t_array(buf, 62, charging_current, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_BMS_DATA_LEN);
#else
    ttalink_bms_data_t packet;
    packet.update_time = update_time;
    packet.core_temperature = core_temperature;
    packet.plate_temperature = plate_temperature;
    packet.cell = cell;
    packet.voltage = voltage;
    packet.current = current;
    packet.bms_state = bms_state;
    packet.bms_addr = bms_addr;
    packet.remaining_capacity = remaining_capacity;
    packet.full_capacity = full_capacity;
    packet.charge_state = charge_state;
    packet.charging_voltage = charging_voltage;
    tta_array_memcpy(packet.voltage_cell, voltage_cell, sizeof(uint16_t)*18);
    tta_array_memcpy(packet.charging_current, charging_current, sizeof(uint16_t)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_BMS_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_BMS_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_DATA_MIN_LEN, TTALINK_MSG_ID_BMS_DATA_LEN, TTALINK_MSG_ID_BMS_DATA_CRC, nocrc);
}

/**
 * @brief Pack a bms_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  . .
 * @param core_temperature  . .
 * @param plate_temperature  . .
 * @param cell  . 
 * @param voltage  . 
 * @param voltage_cell  . 
 * @param current  . .
 * @param bms_state  . 
 * @param bms_addr  . 
 * @param remaining_capacity  . .
 * @param full_capacity  . .
 * @param charge_state  . 
 * @param charging_voltage  . .
 * @param charging_current  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,int16_t core_temperature,int16_t plate_temperature,uint8_t cell,uint16_t voltage,const uint16_t *voltage_cell,int16_t current,uint8_t bms_state,uint8_t bms_addr,uint32_t remaining_capacity,uint32_t full_capacity,uint8_t charge_state,uint16_t charging_voltage,const uint16_t *charging_current)
{
    return _ttalink_bms_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, core_temperature, plate_temperature, cell, voltage, voltage_cell, current, bms_state, bms_addr, remaining_capacity, full_capacity, charge_state, charging_voltage, charging_current, false);
}

/**
 * @brief Pack a bms_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  . .
 * @param core_temperature  . .
 * @param plate_temperature  . .
 * @param cell  . 
 * @param voltage  . 
 * @param voltage_cell  . 
 * @param current  . .
 * @param bms_state  . 
 * @param bms_addr  . 
 * @param remaining_capacity  . .
 * @param full_capacity  . .
 * @param charge_state  . 
 * @param charging_voltage  . .
 * @param charging_current  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,int16_t core_temperature,int16_t plate_temperature,uint8_t cell,uint16_t voltage,const uint16_t *voltage_cell,int16_t current,uint8_t bms_state,uint8_t bms_addr,uint32_t remaining_capacity,uint32_t full_capacity,uint8_t charge_state,uint16_t charging_voltage,const uint16_t *charging_current)
{
    return _ttalink_bms_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, core_temperature, plate_temperature, cell, voltage, voltage_cell, current, bms_state, bms_addr, remaining_capacity, full_capacity, charge_state, charging_voltage, charging_current, true);
}


static inline uint16_t _ttalink_bms_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_bms_data_t* bms_data, bool nocrc)
{
    if(nocrc){
        return ttalink_bms_data_pack_nocrc(dst_addr, src_addr, msg, bms_data->update_time, bms_data->core_temperature, bms_data->plate_temperature, bms_data->cell, bms_data->voltage, bms_data->voltage_cell, bms_data->current, bms_data->bms_state, bms_data->bms_addr, bms_data->remaining_capacity, bms_data->full_capacity, bms_data->charge_state, bms_data->charging_voltage, bms_data->charging_current);
    }else{
        return ttalink_bms_data_pack(dst_addr, src_addr, msg, bms_data->update_time, bms_data->core_temperature, bms_data->plate_temperature, bms_data->cell, bms_data->voltage, bms_data->voltage_cell, bms_data->current, bms_data->bms_state, bms_data->bms_addr, bms_data->remaining_capacity, bms_data->full_capacity, bms_data->charge_state, bms_data->charging_voltage, bms_data->charging_current);
    }
    
}

/**
 * @brief Encode a bms_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param bms_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_bms_data_t* bms_data)
{
    return _ttalink_bms_data_encode(dst_addr, src_addr, msg, bms_data, false);
}

/**
 * @brief Encode a bms_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param bms_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_bms_data_t* bms_data)
{
    return _ttalink_bms_data_encode(dst_addr, src_addr, msg, bms_data, true);
}


static inline uint16_t _ttalink_bms_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_bms_data_t* bms_data, bool nocrc)
{
    if(nocrc){
        return ttalink_bms_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, bms_data->update_time, bms_data->core_temperature, bms_data->plate_temperature, bms_data->cell, bms_data->voltage, bms_data->voltage_cell, bms_data->current, bms_data->bms_state, bms_data->bms_addr, bms_data->remaining_capacity, bms_data->full_capacity, bms_data->charge_state, bms_data->charging_voltage, bms_data->charging_current);
    }else{
        return ttalink_bms_data_pack_chan(dst_addr, src_addr, chan, msg, bms_data->update_time, bms_data->core_temperature, bms_data->plate_temperature, bms_data->cell, bms_data->voltage, bms_data->voltage_cell, bms_data->current, bms_data->bms_state, bms_data->bms_addr, bms_data->remaining_capacity, bms_data->full_capacity, bms_data->charge_state, bms_data->charging_voltage, bms_data->charging_current);
    }
}

/**
 * @brief Encode a bms_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param bms_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_bms_data_t* bms_data)
{
    return _ttalink_bms_data_encode_chan(dst_addr, src_addr, chan, msg, bms_data, false);
}

/**
 * @brief Encode a bms_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param bms_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_bms_data_t* bms_data)
{
    return _ttalink_bms_data_encode_chan(dst_addr, src_addr, chan, msg, bms_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_bms_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, int16_t core_temperature, int16_t plate_temperature, uint8_t cell, uint16_t voltage, const uint16_t *voltage_cell, int16_t current, uint8_t bms_state, uint8_t bms_addr, uint32_t remaining_capacity, uint32_t full_capacity, uint8_t charge_state, uint16_t charging_voltage, const uint16_t *charging_current, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BMS_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int16_t(buf, 4, core_temperature);
    _tta_put_int16_t(buf, 6, plate_temperature);
    _tta_put_uint8_t(buf, 8, cell);
    _tta_put_uint16_t(buf, 9, voltage);
    _tta_put_int16_t(buf, 47, current);
    _tta_put_uint8_t(buf, 49, bms_state);
    _tta_put_uint8_t(buf, 50, bms_addr);
    _tta_put_uint32_t(buf, 51, remaining_capacity);
    _tta_put_uint32_t(buf, 55, full_capacity);
    _tta_put_uint8_t(buf, 59, charge_state);
    _tta_put_uint16_t(buf, 60, charging_voltage);
    _tta_put_uint16_t_array(buf, 11, voltage_cell, 18);
    _tta_put_uint16_t_array(buf, 62, charging_current, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_DATA, buf, TTALINK_MSG_ID_BMS_DATA_MIN_LEN, TTALINK_MSG_ID_BMS_DATA_LEN, TTALINK_MSG_ID_BMS_DATA_CRC, nocrc);
#else
    ttalink_bms_data_t packet;
    packet.update_time = update_time;
    packet.core_temperature = core_temperature;
    packet.plate_temperature = plate_temperature;
    packet.cell = cell;
    packet.voltage = voltage;
    packet.current = current;
    packet.bms_state = bms_state;
    packet.bms_addr = bms_addr;
    packet.remaining_capacity = remaining_capacity;
    packet.full_capacity = full_capacity;
    packet.charge_state = charge_state;
    packet.charging_voltage = charging_voltage;
    tta_array_memcpy(packet.voltage_cell, voltage_cell, sizeof(uint16_t)*18);
    tta_array_memcpy(packet.charging_current, charging_current, sizeof(uint16_t)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_DATA, (const char *)&packet, TTALINK_MSG_ID_BMS_DATA_MIN_LEN, TTALINK_MSG_ID_BMS_DATA_LEN, TTALINK_MSG_ID_BMS_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a bms_data message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  . .
 * @param core_temperature  . .
 * @param plate_temperature  . .
 * @param cell  . 
 * @param voltage  . 
 * @param voltage_cell  . 
 * @param current  . .
 * @param bms_state  . 
 * @param bms_addr  . 
 * @param remaining_capacity  . .
 * @param full_capacity  . .
 * @param charge_state  . 
 * @param charging_voltage  . .
 * @param charging_current  . .
 */
static inline void ttalink_bms_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, int16_t core_temperature, int16_t plate_temperature, uint8_t cell, uint16_t voltage, const uint16_t *voltage_cell, int16_t current, uint8_t bms_state, uint8_t bms_addr, uint32_t remaining_capacity, uint32_t full_capacity, uint8_t charge_state, uint16_t charging_voltage, const uint16_t *charging_current)
{
    _ttalink_bms_data_send(dst_addr, src_addr, chan, update_time, core_temperature, plate_temperature, cell, voltage, voltage_cell, current, bms_state, bms_addr, remaining_capacity, full_capacity, charge_state, charging_voltage, charging_current, false);
}

/**
 * @brief Send a bms_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  . .
 * @param core_temperature  . .
 * @param plate_temperature  . .
 * @param cell  . 
 * @param voltage  . 
 * @param voltage_cell  . 
 * @param current  . .
 * @param bms_state  . 
 * @param bms_addr  . 
 * @param remaining_capacity  . .
 * @param full_capacity  . .
 * @param charge_state  . 
 * @param charging_voltage  . .
 * @param charging_current  . .
 */
static inline void ttalink_bms_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, int16_t core_temperature, int16_t plate_temperature, uint8_t cell, uint16_t voltage, const uint16_t *voltage_cell, int16_t current, uint8_t bms_state, uint8_t bms_addr, uint32_t remaining_capacity, uint32_t full_capacity, uint8_t charge_state, uint16_t charging_voltage, const uint16_t *charging_current)
{
    _ttalink_bms_data_send(dst_addr, src_addr, chan, update_time, core_temperature, plate_temperature, cell, voltage, voltage_cell, current, bms_state, bms_addr, remaining_capacity, full_capacity, charge_state, charging_voltage, charging_current, true);
}


static inline void _ttalink_bms_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_bms_data_t* bms_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_bms_data_send_nocrc(dst_addr, src_addr, chan, bms_data->update_time, bms_data->core_temperature, bms_data->plate_temperature, bms_data->cell, bms_data->voltage, bms_data->voltage_cell, bms_data->current, bms_data->bms_state, bms_data->bms_addr, bms_data->remaining_capacity, bms_data->full_capacity, bms_data->charge_state, bms_data->charging_voltage, bms_data->charging_current);
    }else{
        ttalink_bms_data_send(dst_addr, src_addr, chan, bms_data->update_time, bms_data->core_temperature, bms_data->plate_temperature, bms_data->cell, bms_data->voltage, bms_data->voltage_cell, bms_data->current, bms_data->bms_state, bms_data->bms_addr, bms_data->remaining_capacity, bms_data->full_capacity, bms_data->charge_state, bms_data->charging_voltage, bms_data->charging_current);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_DATA, (const char *)bms_data, TTALINK_MSG_ID_BMS_DATA_MIN_LEN, TTALINK_MSG_ID_BMS_DATA_LEN, TTALINK_MSG_ID_BMS_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a bms_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_bms_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_bms_data_t* bms_data)
{
    _ttalink_bms_data_send_struct(dst_addr, src_addr, chan, bms_data, false);
}

/**
 * @brief Send a bms_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_bms_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_bms_data_t* bms_data)
{
    _ttalink_bms_data_send_struct(dst_addr, src_addr, chan, bms_data, true);
}

#if TTALINK_MSG_ID_BMS_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_bms_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, int16_t core_temperature, int16_t plate_temperature, uint8_t cell, uint16_t voltage, const uint16_t *voltage_cell, int16_t current, uint8_t bms_state, uint8_t bms_addr, uint32_t remaining_capacity, uint32_t full_capacity, uint8_t charge_state, uint16_t charging_voltage, const uint16_t *charging_current, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int16_t(buf, 4, core_temperature);
    _tta_put_int16_t(buf, 6, plate_temperature);
    _tta_put_uint8_t(buf, 8, cell);
    _tta_put_uint16_t(buf, 9, voltage);
    _tta_put_int16_t(buf, 47, current);
    _tta_put_uint8_t(buf, 49, bms_state);
    _tta_put_uint8_t(buf, 50, bms_addr);
    _tta_put_uint32_t(buf, 51, remaining_capacity);
    _tta_put_uint32_t(buf, 55, full_capacity);
    _tta_put_uint8_t(buf, 59, charge_state);
    _tta_put_uint16_t(buf, 60, charging_voltage);
    _tta_put_uint16_t_array(buf, 11, voltage_cell, 18);
    _tta_put_uint16_t_array(buf, 62, charging_current, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_DATA, buf, TTALINK_MSG_ID_BMS_DATA_MIN_LEN, TTALINK_MSG_ID_BMS_DATA_LEN, TTALINK_MSG_ID_BMS_DATA_CRC, nocrc);
#else
    ttalink_bms_data_t *packet = (ttalink_bms_data_t *)msgbuf;
    packet->update_time = update_time;
    packet->core_temperature = core_temperature;
    packet->plate_temperature = plate_temperature;
    packet->cell = cell;
    packet->voltage = voltage;
    packet->current = current;
    packet->bms_state = bms_state;
    packet->bms_addr = bms_addr;
    packet->remaining_capacity = remaining_capacity;
    packet->full_capacity = full_capacity;
    packet->charge_state = charge_state;
    packet->charging_voltage = charging_voltage;
    tta_array_memcpy(packet->voltage_cell, voltage_cell, sizeof(uint16_t)*18);
    tta_array_memcpy(packet->charging_current, charging_current, sizeof(uint16_t)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_DATA, (const char *)packet, TTALINK_MSG_ID_BMS_DATA_MIN_LEN, TTALINK_MSG_ID_BMS_DATA_LEN, TTALINK_MSG_ID_BMS_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_bms_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, int16_t core_temperature, int16_t plate_temperature, uint8_t cell, uint16_t voltage, const uint16_t *voltage_cell, int16_t current, uint8_t bms_state, uint8_t bms_addr, uint32_t remaining_capacity, uint32_t full_capacity, uint8_t charge_state, uint16_t charging_voltage, const uint16_t *charging_current)
{
    _ttalink_bms_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, core_temperature, plate_temperature, cell, voltage, voltage_cell, current, bms_state, bms_addr, remaining_capacity, full_capacity, charge_state, charging_voltage, charging_current, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_bms_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, int16_t core_temperature, int16_t plate_temperature, uint8_t cell, uint16_t voltage, const uint16_t *voltage_cell, int16_t current, uint8_t bms_state, uint8_t bms_addr, uint32_t remaining_capacity, uint32_t full_capacity, uint8_t charge_state, uint16_t charging_voltage, const uint16_t *charging_current)
{
    _ttalink_bms_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, core_temperature, plate_temperature, cell, voltage, voltage_cell, current, bms_state, bms_addr, remaining_capacity, full_capacity, charge_state, charging_voltage, charging_current, true);
}
#endif

#endif

// MESSAGE BMS_DATA UNPACKING


/**
 * @brief Get field update_time from bms_data message
 *
 * @return  . .
 */
static inline uint32_t ttalink_bms_data_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field core_temperature from bms_data message
 *
 * @return  . .
 */
static inline int16_t ttalink_bms_data_get_core_temperature(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field plate_temperature from bms_data message
 *
 * @return  . .
 */
static inline int16_t ttalink_bms_data_get_plate_temperature(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field cell from bms_data message
 *
 * @return  . 
 */
static inline uint8_t ttalink_bms_data_get_cell(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field voltage from bms_data message
 *
 * @return  . 
 */
static inline uint16_t ttalink_bms_data_get_voltage(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  9);
}

/**
 * @brief Get field voltage_cell from bms_data message
 *
 * @return  . 
 */
static inline uint16_t ttalink_bms_data_get_voltage_cell(const ttalink_message_t* msg, uint16_t *voltage_cell)
{
    return _TTA_RETURN_uint16_t_array(msg, voltage_cell, 18,  11);
}

/**
 * @brief Get field current from bms_data message
 *
 * @return  . .
 */
static inline int16_t ttalink_bms_data_get_current(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  47);
}

/**
 * @brief Get field bms_state from bms_data message
 *
 * @return  . 
 */
static inline uint8_t ttalink_bms_data_get_bms_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  49);
}

/**
 * @brief Get field bms_addr from bms_data message
 *
 * @return  . 
 */
static inline uint8_t ttalink_bms_data_get_bms_addr(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Get field remaining_capacity from bms_data message
 *
 * @return  . .
 */
static inline uint32_t ttalink_bms_data_get_remaining_capacity(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  51);
}

/**
 * @brief Get field full_capacity from bms_data message
 *
 * @return  . .
 */
static inline uint32_t ttalink_bms_data_get_full_capacity(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  55);
}

/**
 * @brief Get field charge_state from bms_data message
 *
 * @return  . 
 */
static inline uint8_t ttalink_bms_data_get_charge_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  59);
}

/**
 * @brief Get field charging_voltage from bms_data message
 *
 * @return  . .
 */
static inline uint16_t ttalink_bms_data_get_charging_voltage(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  60);
}

/**
 * @brief Get field charging_current from bms_data message
 *
 * @return  . .
 */
static inline uint16_t ttalink_bms_data_get_charging_current(const ttalink_message_t* msg, uint16_t *charging_current)
{
    return _TTA_RETURN_uint16_t_array(msg, charging_current, 3,  62);
}

/**
 * @brief Decode a bms_data message into a struct
 *
 * @param msg The message to decode
 * @param bms_data C-struct to decode the message contents into
 */
static inline void ttalink_bms_data_decode(const ttalink_message_t* msg, ttalink_bms_data_t* bms_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    bms_data->update_time = ttalink_bms_data_get_update_time(msg);
    bms_data->core_temperature = ttalink_bms_data_get_core_temperature(msg);
    bms_data->plate_temperature = ttalink_bms_data_get_plate_temperature(msg);
    bms_data->cell = ttalink_bms_data_get_cell(msg);
    bms_data->voltage = ttalink_bms_data_get_voltage(msg);
    ttalink_bms_data_get_voltage_cell(msg, bms_data->voltage_cell);
    bms_data->current = ttalink_bms_data_get_current(msg);
    bms_data->bms_state = ttalink_bms_data_get_bms_state(msg);
    bms_data->bms_addr = ttalink_bms_data_get_bms_addr(msg);
    bms_data->remaining_capacity = ttalink_bms_data_get_remaining_capacity(msg);
    bms_data->full_capacity = ttalink_bms_data_get_full_capacity(msg);
    bms_data->charge_state = ttalink_bms_data_get_charge_state(msg);
    bms_data->charging_voltage = ttalink_bms_data_get_charging_voltage(msg);
    ttalink_bms_data_get_charging_current(msg, bms_data->charging_current);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_BMS_DATA_LEN? msg->len : TTALINK_MSG_ID_BMS_DATA_LEN;
        memset(bms_data, 0, TTALINK_MSG_ID_BMS_DATA_LEN);
    memcpy(bms_data, _TTA_PAYLOAD(msg), len);
#endif
}
