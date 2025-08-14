#pragma once
// MESSAGE BMS_INFO PACKING

#define TTALINK_MSG_ID_BMS_INFO 2054

TTAPACKED(
typedef struct __ttalink_bms_info_t {
 uint32_t update_time; /*<  . .*/
 uint32_t cycle_count; /*<  .电池循环次数 .*/
 uint16_t overcharge_count; /*<  .过充次数 .*/
 uint16_t overdischarge_count; /*<  . 过放次数.*/
 uint16_t temperature_high_count; /*<  .温度过高次数 .*/
 uint16_t temperature_low_count; /*<  .温度过低次数 .*/
 uint16_t voltage_unbalanced; /*<  . 电压不均衡次数.*/
 uint16_t discharge_current_count; /*<  . 放电电流过大次数.*/
 uint16_t charge_current_count; /*<  .充电电流过大次数 .*/
 uint8_t charging_time; /*<  .电池安全充电时间(0.1h) .*/
 uint16_t storage_voltage; /*<  . 电池存储电压(0.01V).*/
 uint16_t discharge_current; /*<  . 自放电安全电流（0.01A）.*/
 uint32_t design_capacity; /*<  . 电池组的设计容量值（mAh）.*/
 uint16_t design_voltage; /*<  .电池组的设计电压值（0.01V） */
 uint32_t manufacture_date; /*<  电池出厂日期. */
 uint8_t serial_number[16]; /*<  出厂序列号. */
 uint8_t version_number[16]; /*<  出厂版本号. */
 uint8_t manufacturer_name[14]; /*<  .制造厂商名称 .*/
 uint8_t bms_addr; /*<  . */
 uint8_t bms_param; /*<  . */
 uint8_t model_name[8]; /*<  .型号名称 .*/
 uint8_t device_type; /*<  器件类型. */
}) ttalink_bms_info_t;

#define TTALINK_MSG_ID_BMS_INFO_LEN 94
#define TTALINK_MSG_ID_BMS_INFO_MIN_LEN 94
#define TTALINK_MSG_ID_2054_LEN 94
#define TTALINK_MSG_ID_2054_MIN_LEN 94

#define TTALINK_MSG_ID_BMS_INFO_CRC 143
#define TTALINK_MSG_ID_2054_CRC 143

#define TTALINK_MSG_BMS_INFO_FIELD_SERIAL_NUMBER_LEN 16
#define TTALINK_MSG_BMS_INFO_FIELD_VERSION_NUMBER_LEN 16
#define TTALINK_MSG_BMS_INFO_FIELD_MANUFACTURER_NAME_LEN 14
#define TTALINK_MSG_BMS_INFO_FIELD_MODEL_NAME_LEN 8

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_BMS_INFO { \
    2054, \
    "BMS_INFO", \
    22, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_bms_info_t, update_time) }, \
         { "cycle_count", NULL, TTALINK_TYPE_UINT32_T, 0, 4, offsetof(ttalink_bms_info_t, cycle_count) }, \
         { "overcharge_count", NULL, TTALINK_TYPE_UINT16_T, 0, 8, offsetof(ttalink_bms_info_t, overcharge_count) }, \
         { "overdischarge_count", NULL, TTALINK_TYPE_UINT16_T, 0, 10, offsetof(ttalink_bms_info_t, overdischarge_count) }, \
         { "temperature_high_count", NULL, TTALINK_TYPE_UINT16_T, 0, 12, offsetof(ttalink_bms_info_t, temperature_high_count) }, \
         { "temperature_low_count", NULL, TTALINK_TYPE_UINT16_T, 0, 14, offsetof(ttalink_bms_info_t, temperature_low_count) }, \
         { "voltage_unbalanced", NULL, TTALINK_TYPE_UINT16_T, 0, 16, offsetof(ttalink_bms_info_t, voltage_unbalanced) }, \
         { "discharge_current_count", NULL, TTALINK_TYPE_UINT16_T, 0, 18, offsetof(ttalink_bms_info_t, discharge_current_count) }, \
         { "charge_current_count", NULL, TTALINK_TYPE_UINT16_T, 0, 20, offsetof(ttalink_bms_info_t, charge_current_count) }, \
         { "charging_time", NULL, TTALINK_TYPE_UINT8_T, 0, 22, offsetof(ttalink_bms_info_t, charging_time) }, \
         { "storage_voltage", NULL, TTALINK_TYPE_UINT16_T, 0, 23, offsetof(ttalink_bms_info_t, storage_voltage) }, \
         { "discharge_current", NULL, TTALINK_TYPE_UINT16_T, 0, 25, offsetof(ttalink_bms_info_t, discharge_current) }, \
         { "design_capacity", NULL, TTALINK_TYPE_UINT32_T, 0, 27, offsetof(ttalink_bms_info_t, design_capacity) }, \
         { "design_voltage", NULL, TTALINK_TYPE_UINT16_T, 0, 31, offsetof(ttalink_bms_info_t, design_voltage) }, \
         { "manufacture_date", NULL, TTALINK_TYPE_UINT32_T, 0, 33, offsetof(ttalink_bms_info_t, manufacture_date) }, \
         { "serial_number", NULL, TTALINK_TYPE_UINT8_T, 16, 37, offsetof(ttalink_bms_info_t, serial_number) }, \
         { "version_number", NULL, TTALINK_TYPE_UINT8_T, 16, 53, offsetof(ttalink_bms_info_t, version_number) }, \
         { "manufacturer_name", NULL, TTALINK_TYPE_UINT8_T, 14, 69, offsetof(ttalink_bms_info_t, manufacturer_name) }, \
         { "bms_addr", NULL, TTALINK_TYPE_UINT8_T, 0, 83, offsetof(ttalink_bms_info_t, bms_addr) }, \
         { "bms_param", NULL, TTALINK_TYPE_UINT8_T, 0, 84, offsetof(ttalink_bms_info_t, bms_param) }, \
         { "model_name", NULL, TTALINK_TYPE_UINT8_T, 8, 85, offsetof(ttalink_bms_info_t, model_name) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 93, offsetof(ttalink_bms_info_t, device_type) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_BMS_INFO { \
    "BMS_INFO", \
    22, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_bms_info_t, update_time) }, \
         { "cycle_count", NULL, TTALINK_TYPE_UINT32_T, 0, 4, offsetof(ttalink_bms_info_t, cycle_count) }, \
         { "overcharge_count", NULL, TTALINK_TYPE_UINT16_T, 0, 8, offsetof(ttalink_bms_info_t, overcharge_count) }, \
         { "overdischarge_count", NULL, TTALINK_TYPE_UINT16_T, 0, 10, offsetof(ttalink_bms_info_t, overdischarge_count) }, \
         { "temperature_high_count", NULL, TTALINK_TYPE_UINT16_T, 0, 12, offsetof(ttalink_bms_info_t, temperature_high_count) }, \
         { "temperature_low_count", NULL, TTALINK_TYPE_UINT16_T, 0, 14, offsetof(ttalink_bms_info_t, temperature_low_count) }, \
         { "voltage_unbalanced", NULL, TTALINK_TYPE_UINT16_T, 0, 16, offsetof(ttalink_bms_info_t, voltage_unbalanced) }, \
         { "discharge_current_count", NULL, TTALINK_TYPE_UINT16_T, 0, 18, offsetof(ttalink_bms_info_t, discharge_current_count) }, \
         { "charge_current_count", NULL, TTALINK_TYPE_UINT16_T, 0, 20, offsetof(ttalink_bms_info_t, charge_current_count) }, \
         { "charging_time", NULL, TTALINK_TYPE_UINT8_T, 0, 22, offsetof(ttalink_bms_info_t, charging_time) }, \
         { "storage_voltage", NULL, TTALINK_TYPE_UINT16_T, 0, 23, offsetof(ttalink_bms_info_t, storage_voltage) }, \
         { "discharge_current", NULL, TTALINK_TYPE_UINT16_T, 0, 25, offsetof(ttalink_bms_info_t, discharge_current) }, \
         { "design_capacity", NULL, TTALINK_TYPE_UINT32_T, 0, 27, offsetof(ttalink_bms_info_t, design_capacity) }, \
         { "design_voltage", NULL, TTALINK_TYPE_UINT16_T, 0, 31, offsetof(ttalink_bms_info_t, design_voltage) }, \
         { "manufacture_date", NULL, TTALINK_TYPE_UINT32_T, 0, 33, offsetof(ttalink_bms_info_t, manufacture_date) }, \
         { "serial_number", NULL, TTALINK_TYPE_UINT8_T, 16, 37, offsetof(ttalink_bms_info_t, serial_number) }, \
         { "version_number", NULL, TTALINK_TYPE_UINT8_T, 16, 53, offsetof(ttalink_bms_info_t, version_number) }, \
         { "manufacturer_name", NULL, TTALINK_TYPE_UINT8_T, 14, 69, offsetof(ttalink_bms_info_t, manufacturer_name) }, \
         { "bms_addr", NULL, TTALINK_TYPE_UINT8_T, 0, 83, offsetof(ttalink_bms_info_t, bms_addr) }, \
         { "bms_param", NULL, TTALINK_TYPE_UINT8_T, 0, 84, offsetof(ttalink_bms_info_t, bms_param) }, \
         { "model_name", NULL, TTALINK_TYPE_UINT8_T, 8, 85, offsetof(ttalink_bms_info_t, model_name) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 93, offsetof(ttalink_bms_info_t, device_type) }, \
         } \
}
#endif


static inline uint16_t _ttalink_bms_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t cycle_count, uint16_t overcharge_count, uint16_t overdischarge_count, uint16_t temperature_high_count, uint16_t temperature_low_count, uint16_t voltage_unbalanced, uint16_t discharge_current_count, uint16_t charge_current_count, uint8_t charging_time, uint16_t storage_voltage, uint16_t discharge_current, uint32_t design_capacity, uint16_t design_voltage, uint32_t manufacture_date, const uint8_t *serial_number, const uint8_t *version_number, const uint8_t *manufacturer_name, uint8_t bms_addr, uint8_t bms_param, const uint8_t *model_name, uint8_t device_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BMS_INFO_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, cycle_count);
    _tta_put_uint16_t(buf, 8, overcharge_count);
    _tta_put_uint16_t(buf, 10, overdischarge_count);
    _tta_put_uint16_t(buf, 12, temperature_high_count);
    _tta_put_uint16_t(buf, 14, temperature_low_count);
    _tta_put_uint16_t(buf, 16, voltage_unbalanced);
    _tta_put_uint16_t(buf, 18, discharge_current_count);
    _tta_put_uint16_t(buf, 20, charge_current_count);
    _tta_put_uint8_t(buf, 22, charging_time);
    _tta_put_uint16_t(buf, 23, storage_voltage);
    _tta_put_uint16_t(buf, 25, discharge_current);
    _tta_put_uint32_t(buf, 27, design_capacity);
    _tta_put_uint16_t(buf, 31, design_voltage);
    _tta_put_uint32_t(buf, 33, manufacture_date);
    _tta_put_uint8_t(buf, 83, bms_addr);
    _tta_put_uint8_t(buf, 84, bms_param);
    _tta_put_uint8_t(buf, 93, device_type);
    _tta_put_uint8_t_array(buf, 37, serial_number, 16);
    _tta_put_uint8_t_array(buf, 53, version_number, 16);
    _tta_put_uint8_t_array(buf, 69, manufacturer_name, 14);
    _tta_put_uint8_t_array(buf, 85, model_name, 8);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_BMS_INFO_LEN);
#else
    ttalink_bms_info_t packet;
    packet.update_time = update_time;
    packet.cycle_count = cycle_count;
    packet.overcharge_count = overcharge_count;
    packet.overdischarge_count = overdischarge_count;
    packet.temperature_high_count = temperature_high_count;
    packet.temperature_low_count = temperature_low_count;
    packet.voltage_unbalanced = voltage_unbalanced;
    packet.discharge_current_count = discharge_current_count;
    packet.charge_current_count = charge_current_count;
    packet.charging_time = charging_time;
    packet.storage_voltage = storage_voltage;
    packet.discharge_current = discharge_current;
    packet.design_capacity = design_capacity;
    packet.design_voltage = design_voltage;
    packet.manufacture_date = manufacture_date;
    packet.bms_addr = bms_addr;
    packet.bms_param = bms_param;
    packet.device_type = device_type;
    tta_array_memcpy(packet.serial_number, serial_number, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.version_number, version_number, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.manufacturer_name, manufacturer_name, sizeof(uint8_t)*14);
    tta_array_memcpy(packet.model_name, model_name, sizeof(uint8_t)*8);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_BMS_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_BMS_INFO;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_BMS_INFO_MIN_LEN, TTALINK_MSG_ID_BMS_INFO_LEN, TTALINK_MSG_ID_BMS_INFO_CRC, nocrc);
}

/**
 * @brief Pack a bms_info message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  . .
 * @param cycle_count  .电池循环次数 .
 * @param overcharge_count  .过充次数 .
 * @param overdischarge_count  . 过放次数.
 * @param temperature_high_count  .温度过高次数 .
 * @param temperature_low_count  .温度过低次数 .
 * @param voltage_unbalanced  . 电压不均衡次数.
 * @param discharge_current_count  . 放电电流过大次数.
 * @param charge_current_count  .充电电流过大次数 .
 * @param charging_time  .电池安全充电时间(0.1h) .
 * @param storage_voltage  . 电池存储电压(0.01V).
 * @param discharge_current  . 自放电安全电流（0.01A）.
 * @param design_capacity  . 电池组的设计容量值（mAh）.
 * @param design_voltage  .电池组的设计电压值（0.01V） 
 * @param manufacture_date  电池出厂日期. 
 * @param serial_number  出厂序列号. 
 * @param version_number  出厂版本号. 
 * @param manufacturer_name  .制造厂商名称 .
 * @param bms_addr  . 
 * @param bms_param  . 
 * @param model_name  .型号名称 .
 * @param device_type  器件类型. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t cycle_count, uint16_t overcharge_count, uint16_t overdischarge_count, uint16_t temperature_high_count, uint16_t temperature_low_count, uint16_t voltage_unbalanced, uint16_t discharge_current_count, uint16_t charge_current_count, uint8_t charging_time, uint16_t storage_voltage, uint16_t discharge_current, uint32_t design_capacity, uint16_t design_voltage, uint32_t manufacture_date, const uint8_t *serial_number, const uint8_t *version_number, const uint8_t *manufacturer_name, uint8_t bms_addr, uint8_t bms_param, const uint8_t *model_name, uint8_t device_type)
{
    return _ttalink_bms_info_pack(dst_addr, src_addr, msg,  update_time, cycle_count, overcharge_count, overdischarge_count, temperature_high_count, temperature_low_count, voltage_unbalanced, discharge_current_count, charge_current_count, charging_time, storage_voltage, discharge_current, design_capacity, design_voltage, manufacture_date, serial_number, version_number, manufacturer_name, bms_addr, bms_param, model_name, device_type, false);
}

/**
 * @brief Pack a bms_info message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  . .
 * @param cycle_count  .电池循环次数 .
 * @param overcharge_count  .过充次数 .
 * @param overdischarge_count  . 过放次数.
 * @param temperature_high_count  .温度过高次数 .
 * @param temperature_low_count  .温度过低次数 .
 * @param voltage_unbalanced  . 电压不均衡次数.
 * @param discharge_current_count  . 放电电流过大次数.
 * @param charge_current_count  .充电电流过大次数 .
 * @param charging_time  .电池安全充电时间(0.1h) .
 * @param storage_voltage  . 电池存储电压(0.01V).
 * @param discharge_current  . 自放电安全电流（0.01A）.
 * @param design_capacity  . 电池组的设计容量值（mAh）.
 * @param design_voltage  .电池组的设计电压值（0.01V） 
 * @param manufacture_date  电池出厂日期. 
 * @param serial_number  出厂序列号. 
 * @param version_number  出厂版本号. 
 * @param manufacturer_name  .制造厂商名称 .
 * @param bms_addr  . 
 * @param bms_param  . 
 * @param model_name  .型号名称 .
 * @param device_type  器件类型. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_info_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t cycle_count, uint16_t overcharge_count, uint16_t overdischarge_count, uint16_t temperature_high_count, uint16_t temperature_low_count, uint16_t voltage_unbalanced, uint16_t discharge_current_count, uint16_t charge_current_count, uint8_t charging_time, uint16_t storage_voltage, uint16_t discharge_current, uint32_t design_capacity, uint16_t design_voltage, uint32_t manufacture_date, const uint8_t *serial_number, const uint8_t *version_number, const uint8_t *manufacturer_name, uint8_t bms_addr, uint8_t bms_param, const uint8_t *model_name, uint8_t device_type)
{
    return _ttalink_bms_info_pack(dst_addr, src_addr, msg,  update_time, cycle_count, overcharge_count, overdischarge_count, temperature_high_count, temperature_low_count, voltage_unbalanced, discharge_current_count, charge_current_count, charging_time, storage_voltage, discharge_current, design_capacity, design_voltage, manufacture_date, serial_number, version_number, manufacturer_name, bms_addr, bms_param, model_name, device_type, true);
}


static inline uint16_t _ttalink_bms_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t cycle_count,uint16_t overcharge_count,uint16_t overdischarge_count,uint16_t temperature_high_count,uint16_t temperature_low_count,uint16_t voltage_unbalanced,uint16_t discharge_current_count,uint16_t charge_current_count,uint8_t charging_time,uint16_t storage_voltage,uint16_t discharge_current,uint32_t design_capacity,uint16_t design_voltage,uint32_t manufacture_date,const uint8_t *serial_number,const uint8_t *version_number,const uint8_t *manufacturer_name,uint8_t bms_addr,uint8_t bms_param,const uint8_t *model_name,uint8_t device_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BMS_INFO_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, cycle_count);
    _tta_put_uint16_t(buf, 8, overcharge_count);
    _tta_put_uint16_t(buf, 10, overdischarge_count);
    _tta_put_uint16_t(buf, 12, temperature_high_count);
    _tta_put_uint16_t(buf, 14, temperature_low_count);
    _tta_put_uint16_t(buf, 16, voltage_unbalanced);
    _tta_put_uint16_t(buf, 18, discharge_current_count);
    _tta_put_uint16_t(buf, 20, charge_current_count);
    _tta_put_uint8_t(buf, 22, charging_time);
    _tta_put_uint16_t(buf, 23, storage_voltage);
    _tta_put_uint16_t(buf, 25, discharge_current);
    _tta_put_uint32_t(buf, 27, design_capacity);
    _tta_put_uint16_t(buf, 31, design_voltage);
    _tta_put_uint32_t(buf, 33, manufacture_date);
    _tta_put_uint8_t(buf, 83, bms_addr);
    _tta_put_uint8_t(buf, 84, bms_param);
    _tta_put_uint8_t(buf, 93, device_type);
    _tta_put_uint8_t_array(buf, 37, serial_number, 16);
    _tta_put_uint8_t_array(buf, 53, version_number, 16);
    _tta_put_uint8_t_array(buf, 69, manufacturer_name, 14);
    _tta_put_uint8_t_array(buf, 85, model_name, 8);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_BMS_INFO_LEN);
#else
    ttalink_bms_info_t packet;
    packet.update_time = update_time;
    packet.cycle_count = cycle_count;
    packet.overcharge_count = overcharge_count;
    packet.overdischarge_count = overdischarge_count;
    packet.temperature_high_count = temperature_high_count;
    packet.temperature_low_count = temperature_low_count;
    packet.voltage_unbalanced = voltage_unbalanced;
    packet.discharge_current_count = discharge_current_count;
    packet.charge_current_count = charge_current_count;
    packet.charging_time = charging_time;
    packet.storage_voltage = storage_voltage;
    packet.discharge_current = discharge_current;
    packet.design_capacity = design_capacity;
    packet.design_voltage = design_voltage;
    packet.manufacture_date = manufacture_date;
    packet.bms_addr = bms_addr;
    packet.bms_param = bms_param;
    packet.device_type = device_type;
    tta_array_memcpy(packet.serial_number, serial_number, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.version_number, version_number, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.manufacturer_name, manufacturer_name, sizeof(uint8_t)*14);
    tta_array_memcpy(packet.model_name, model_name, sizeof(uint8_t)*8);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_BMS_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_BMS_INFO;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_INFO_MIN_LEN, TTALINK_MSG_ID_BMS_INFO_LEN, TTALINK_MSG_ID_BMS_INFO_CRC, nocrc);
}

/**
 * @brief Pack a bms_info message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  . .
 * @param cycle_count  .电池循环次数 .
 * @param overcharge_count  .过充次数 .
 * @param overdischarge_count  . 过放次数.
 * @param temperature_high_count  .温度过高次数 .
 * @param temperature_low_count  .温度过低次数 .
 * @param voltage_unbalanced  . 电压不均衡次数.
 * @param discharge_current_count  . 放电电流过大次数.
 * @param charge_current_count  .充电电流过大次数 .
 * @param charging_time  .电池安全充电时间(0.1h) .
 * @param storage_voltage  . 电池存储电压(0.01V).
 * @param discharge_current  . 自放电安全电流（0.01A）.
 * @param design_capacity  . 电池组的设计容量值（mAh）.
 * @param design_voltage  .电池组的设计电压值（0.01V） 
 * @param manufacture_date  电池出厂日期. 
 * @param serial_number  出厂序列号. 
 * @param version_number  出厂版本号. 
 * @param manufacturer_name  .制造厂商名称 .
 * @param bms_addr  . 
 * @param bms_param  . 
 * @param model_name  .型号名称 .
 * @param device_type  器件类型. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t cycle_count,uint16_t overcharge_count,uint16_t overdischarge_count,uint16_t temperature_high_count,uint16_t temperature_low_count,uint16_t voltage_unbalanced,uint16_t discharge_current_count,uint16_t charge_current_count,uint8_t charging_time,uint16_t storage_voltage,uint16_t discharge_current,uint32_t design_capacity,uint16_t design_voltage,uint32_t manufacture_date,const uint8_t *serial_number,const uint8_t *version_number,const uint8_t *manufacturer_name,uint8_t bms_addr,uint8_t bms_param,const uint8_t *model_name,uint8_t device_type)
{
    return _ttalink_bms_info_pack_chan(dst_addr, src_addr, chan, msg,  update_time, cycle_count, overcharge_count, overdischarge_count, temperature_high_count, temperature_low_count, voltage_unbalanced, discharge_current_count, charge_current_count, charging_time, storage_voltage, discharge_current, design_capacity, design_voltage, manufacture_date, serial_number, version_number, manufacturer_name, bms_addr, bms_param, model_name, device_type, false);
}

/**
 * @brief Pack a bms_info message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  . .
 * @param cycle_count  .电池循环次数 .
 * @param overcharge_count  .过充次数 .
 * @param overdischarge_count  . 过放次数.
 * @param temperature_high_count  .温度过高次数 .
 * @param temperature_low_count  .温度过低次数 .
 * @param voltage_unbalanced  . 电压不均衡次数.
 * @param discharge_current_count  . 放电电流过大次数.
 * @param charge_current_count  .充电电流过大次数 .
 * @param charging_time  .电池安全充电时间(0.1h) .
 * @param storage_voltage  . 电池存储电压(0.01V).
 * @param discharge_current  . 自放电安全电流（0.01A）.
 * @param design_capacity  . 电池组的设计容量值（mAh）.
 * @param design_voltage  .电池组的设计电压值（0.01V） 
 * @param manufacture_date  电池出厂日期. 
 * @param serial_number  出厂序列号. 
 * @param version_number  出厂版本号. 
 * @param manufacturer_name  .制造厂商名称 .
 * @param bms_addr  . 
 * @param bms_param  . 
 * @param model_name  .型号名称 .
 * @param device_type  器件类型. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_info_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t cycle_count,uint16_t overcharge_count,uint16_t overdischarge_count,uint16_t temperature_high_count,uint16_t temperature_low_count,uint16_t voltage_unbalanced,uint16_t discharge_current_count,uint16_t charge_current_count,uint8_t charging_time,uint16_t storage_voltage,uint16_t discharge_current,uint32_t design_capacity,uint16_t design_voltage,uint32_t manufacture_date,const uint8_t *serial_number,const uint8_t *version_number,const uint8_t *manufacturer_name,uint8_t bms_addr,uint8_t bms_param,const uint8_t *model_name,uint8_t device_type)
{
    return _ttalink_bms_info_pack_chan(dst_addr, src_addr, chan, msg,  update_time, cycle_count, overcharge_count, overdischarge_count, temperature_high_count, temperature_low_count, voltage_unbalanced, discharge_current_count, charge_current_count, charging_time, storage_voltage, discharge_current, design_capacity, design_voltage, manufacture_date, serial_number, version_number, manufacturer_name, bms_addr, bms_param, model_name, device_type, true);
}


static inline uint16_t _ttalink_bms_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_bms_info_t* bms_info, bool nocrc)
{
    if(nocrc){
        return ttalink_bms_info_pack_nocrc(dst_addr, src_addr, msg, bms_info->update_time, bms_info->cycle_count, bms_info->overcharge_count, bms_info->overdischarge_count, bms_info->temperature_high_count, bms_info->temperature_low_count, bms_info->voltage_unbalanced, bms_info->discharge_current_count, bms_info->charge_current_count, bms_info->charging_time, bms_info->storage_voltage, bms_info->discharge_current, bms_info->design_capacity, bms_info->design_voltage, bms_info->manufacture_date, bms_info->serial_number, bms_info->version_number, bms_info->manufacturer_name, bms_info->bms_addr, bms_info->bms_param, bms_info->model_name, bms_info->device_type);
    }else{
        return ttalink_bms_info_pack(dst_addr, src_addr, msg, bms_info->update_time, bms_info->cycle_count, bms_info->overcharge_count, bms_info->overdischarge_count, bms_info->temperature_high_count, bms_info->temperature_low_count, bms_info->voltage_unbalanced, bms_info->discharge_current_count, bms_info->charge_current_count, bms_info->charging_time, bms_info->storage_voltage, bms_info->discharge_current, bms_info->design_capacity, bms_info->design_voltage, bms_info->manufacture_date, bms_info->serial_number, bms_info->version_number, bms_info->manufacturer_name, bms_info->bms_addr, bms_info->bms_param, bms_info->model_name, bms_info->device_type);
    }
    
}

/**
 * @brief Encode a bms_info struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param bms_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_bms_info_t* bms_info)
{
    return _ttalink_bms_info_encode(dst_addr, src_addr, msg, bms_info, false);
}

/**
 * @brief Encode a bms_info struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param bms_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_info_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_bms_info_t* bms_info)
{
    return _ttalink_bms_info_encode(dst_addr, src_addr, msg, bms_info, true);
}


static inline uint16_t _ttalink_bms_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_bms_info_t* bms_info, bool nocrc)
{
    if(nocrc){
        return ttalink_bms_info_pack_chan_nocrc(dst_addr, src_addr, chan, msg, bms_info->update_time, bms_info->cycle_count, bms_info->overcharge_count, bms_info->overdischarge_count, bms_info->temperature_high_count, bms_info->temperature_low_count, bms_info->voltage_unbalanced, bms_info->discharge_current_count, bms_info->charge_current_count, bms_info->charging_time, bms_info->storage_voltage, bms_info->discharge_current, bms_info->design_capacity, bms_info->design_voltage, bms_info->manufacture_date, bms_info->serial_number, bms_info->version_number, bms_info->manufacturer_name, bms_info->bms_addr, bms_info->bms_param, bms_info->model_name, bms_info->device_type);
    }else{
        return ttalink_bms_info_pack_chan(dst_addr, src_addr, chan, msg, bms_info->update_time, bms_info->cycle_count, bms_info->overcharge_count, bms_info->overdischarge_count, bms_info->temperature_high_count, bms_info->temperature_low_count, bms_info->voltage_unbalanced, bms_info->discharge_current_count, bms_info->charge_current_count, bms_info->charging_time, bms_info->storage_voltage, bms_info->discharge_current, bms_info->design_capacity, bms_info->design_voltage, bms_info->manufacture_date, bms_info->serial_number, bms_info->version_number, bms_info->manufacturer_name, bms_info->bms_addr, bms_info->bms_param, bms_info->model_name, bms_info->device_type);
    }
}

/**
 * @brief Encode a bms_info struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param bms_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_bms_info_t* bms_info)
{
    return _ttalink_bms_info_encode_chan(dst_addr, src_addr, chan, msg, bms_info, false);
}

/**
 * @brief Encode a bms_info struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param bms_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_info_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_bms_info_t* bms_info)
{
    return _ttalink_bms_info_encode_chan(dst_addr, src_addr, chan, msg, bms_info, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_bms_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t cycle_count, uint16_t overcharge_count, uint16_t overdischarge_count, uint16_t temperature_high_count, uint16_t temperature_low_count, uint16_t voltage_unbalanced, uint16_t discharge_current_count, uint16_t charge_current_count, uint8_t charging_time, uint16_t storage_voltage, uint16_t discharge_current, uint32_t design_capacity, uint16_t design_voltage, uint32_t manufacture_date, const uint8_t *serial_number, const uint8_t *version_number, const uint8_t *manufacturer_name, uint8_t bms_addr, uint8_t bms_param, const uint8_t *model_name, uint8_t device_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BMS_INFO_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, cycle_count);
    _tta_put_uint16_t(buf, 8, overcharge_count);
    _tta_put_uint16_t(buf, 10, overdischarge_count);
    _tta_put_uint16_t(buf, 12, temperature_high_count);
    _tta_put_uint16_t(buf, 14, temperature_low_count);
    _tta_put_uint16_t(buf, 16, voltage_unbalanced);
    _tta_put_uint16_t(buf, 18, discharge_current_count);
    _tta_put_uint16_t(buf, 20, charge_current_count);
    _tta_put_uint8_t(buf, 22, charging_time);
    _tta_put_uint16_t(buf, 23, storage_voltage);
    _tta_put_uint16_t(buf, 25, discharge_current);
    _tta_put_uint32_t(buf, 27, design_capacity);
    _tta_put_uint16_t(buf, 31, design_voltage);
    _tta_put_uint32_t(buf, 33, manufacture_date);
    _tta_put_uint8_t(buf, 83, bms_addr);
    _tta_put_uint8_t(buf, 84, bms_param);
    _tta_put_uint8_t(buf, 93, device_type);
    _tta_put_uint8_t_array(buf, 37, serial_number, 16);
    _tta_put_uint8_t_array(buf, 53, version_number, 16);
    _tta_put_uint8_t_array(buf, 69, manufacturer_name, 14);
    _tta_put_uint8_t_array(buf, 85, model_name, 8);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_INFO, buf, TTALINK_MSG_ID_BMS_INFO_MIN_LEN, TTALINK_MSG_ID_BMS_INFO_LEN, TTALINK_MSG_ID_BMS_INFO_CRC, nocrc);
#else
    ttalink_bms_info_t packet;
    packet.update_time = update_time;
    packet.cycle_count = cycle_count;
    packet.overcharge_count = overcharge_count;
    packet.overdischarge_count = overdischarge_count;
    packet.temperature_high_count = temperature_high_count;
    packet.temperature_low_count = temperature_low_count;
    packet.voltage_unbalanced = voltage_unbalanced;
    packet.discharge_current_count = discharge_current_count;
    packet.charge_current_count = charge_current_count;
    packet.charging_time = charging_time;
    packet.storage_voltage = storage_voltage;
    packet.discharge_current = discharge_current;
    packet.design_capacity = design_capacity;
    packet.design_voltage = design_voltage;
    packet.manufacture_date = manufacture_date;
    packet.bms_addr = bms_addr;
    packet.bms_param = bms_param;
    packet.device_type = device_type;
    tta_array_memcpy(packet.serial_number, serial_number, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.version_number, version_number, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.manufacturer_name, manufacturer_name, sizeof(uint8_t)*14);
    tta_array_memcpy(packet.model_name, model_name, sizeof(uint8_t)*8);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_INFO, (const char *)&packet, TTALINK_MSG_ID_BMS_INFO_MIN_LEN, TTALINK_MSG_ID_BMS_INFO_LEN, TTALINK_MSG_ID_BMS_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a bms_info message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  . .
 * @param cycle_count  .电池循环次数 .
 * @param overcharge_count  .过充次数 .
 * @param overdischarge_count  . 过放次数.
 * @param temperature_high_count  .温度过高次数 .
 * @param temperature_low_count  .温度过低次数 .
 * @param voltage_unbalanced  . 电压不均衡次数.
 * @param discharge_current_count  . 放电电流过大次数.
 * @param charge_current_count  .充电电流过大次数 .
 * @param charging_time  .电池安全充电时间(0.1h) .
 * @param storage_voltage  . 电池存储电压(0.01V).
 * @param discharge_current  . 自放电安全电流（0.01A）.
 * @param design_capacity  . 电池组的设计容量值（mAh）.
 * @param design_voltage  .电池组的设计电压值（0.01V） 
 * @param manufacture_date  电池出厂日期. 
 * @param serial_number  出厂序列号. 
 * @param version_number  出厂版本号. 
 * @param manufacturer_name  .制造厂商名称 .
 * @param bms_addr  . 
 * @param bms_param  . 
 * @param model_name  .型号名称 .
 * @param device_type  器件类型. 
 */
static inline void ttalink_bms_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t cycle_count, uint16_t overcharge_count, uint16_t overdischarge_count, uint16_t temperature_high_count, uint16_t temperature_low_count, uint16_t voltage_unbalanced, uint16_t discharge_current_count, uint16_t charge_current_count, uint8_t charging_time, uint16_t storage_voltage, uint16_t discharge_current, uint32_t design_capacity, uint16_t design_voltage, uint32_t manufacture_date, const uint8_t *serial_number, const uint8_t *version_number, const uint8_t *manufacturer_name, uint8_t bms_addr, uint8_t bms_param, const uint8_t *model_name, uint8_t device_type)
{
    _ttalink_bms_info_send(dst_addr, src_addr, chan, update_time, cycle_count, overcharge_count, overdischarge_count, temperature_high_count, temperature_low_count, voltage_unbalanced, discharge_current_count, charge_current_count, charging_time, storage_voltage, discharge_current, design_capacity, design_voltage, manufacture_date, serial_number, version_number, manufacturer_name, bms_addr, bms_param, model_name, device_type, false);
}

/**
 * @brief Send a bms_info message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  . .
 * @param cycle_count  .电池循环次数 .
 * @param overcharge_count  .过充次数 .
 * @param overdischarge_count  . 过放次数.
 * @param temperature_high_count  .温度过高次数 .
 * @param temperature_low_count  .温度过低次数 .
 * @param voltage_unbalanced  . 电压不均衡次数.
 * @param discharge_current_count  . 放电电流过大次数.
 * @param charge_current_count  .充电电流过大次数 .
 * @param charging_time  .电池安全充电时间(0.1h) .
 * @param storage_voltage  . 电池存储电压(0.01V).
 * @param discharge_current  . 自放电安全电流（0.01A）.
 * @param design_capacity  . 电池组的设计容量值（mAh）.
 * @param design_voltage  .电池组的设计电压值（0.01V） 
 * @param manufacture_date  电池出厂日期. 
 * @param serial_number  出厂序列号. 
 * @param version_number  出厂版本号. 
 * @param manufacturer_name  .制造厂商名称 .
 * @param bms_addr  . 
 * @param bms_param  . 
 * @param model_name  .型号名称 .
 * @param device_type  器件类型. 
 */
static inline void ttalink_bms_info_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t cycle_count, uint16_t overcharge_count, uint16_t overdischarge_count, uint16_t temperature_high_count, uint16_t temperature_low_count, uint16_t voltage_unbalanced, uint16_t discharge_current_count, uint16_t charge_current_count, uint8_t charging_time, uint16_t storage_voltage, uint16_t discharge_current, uint32_t design_capacity, uint16_t design_voltage, uint32_t manufacture_date, const uint8_t *serial_number, const uint8_t *version_number, const uint8_t *manufacturer_name, uint8_t bms_addr, uint8_t bms_param, const uint8_t *model_name, uint8_t device_type)
{
    _ttalink_bms_info_send(dst_addr, src_addr, chan, update_time, cycle_count, overcharge_count, overdischarge_count, temperature_high_count, temperature_low_count, voltage_unbalanced, discharge_current_count, charge_current_count, charging_time, storage_voltage, discharge_current, design_capacity, design_voltage, manufacture_date, serial_number, version_number, manufacturer_name, bms_addr, bms_param, model_name, device_type, true);
}


static inline void _ttalink_bms_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_bms_info_t* bms_info, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_bms_info_send_nocrc(dst_addr, src_addr, chan, bms_info->update_time, bms_info->cycle_count, bms_info->overcharge_count, bms_info->overdischarge_count, bms_info->temperature_high_count, bms_info->temperature_low_count, bms_info->voltage_unbalanced, bms_info->discharge_current_count, bms_info->charge_current_count, bms_info->charging_time, bms_info->storage_voltage, bms_info->discharge_current, bms_info->design_capacity, bms_info->design_voltage, bms_info->manufacture_date, bms_info->serial_number, bms_info->version_number, bms_info->manufacturer_name, bms_info->bms_addr, bms_info->bms_param, bms_info->model_name, bms_info->device_type);
    }else{
        ttalink_bms_info_send(dst_addr, src_addr, chan, bms_info->update_time, bms_info->cycle_count, bms_info->overcharge_count, bms_info->overdischarge_count, bms_info->temperature_high_count, bms_info->temperature_low_count, bms_info->voltage_unbalanced, bms_info->discharge_current_count, bms_info->charge_current_count, bms_info->charging_time, bms_info->storage_voltage, bms_info->discharge_current, bms_info->design_capacity, bms_info->design_voltage, bms_info->manufacture_date, bms_info->serial_number, bms_info->version_number, bms_info->manufacturer_name, bms_info->bms_addr, bms_info->bms_param, bms_info->model_name, bms_info->device_type);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_INFO, (const char *)bms_info, TTALINK_MSG_ID_BMS_INFO_MIN_LEN, TTALINK_MSG_ID_BMS_INFO_LEN, TTALINK_MSG_ID_BMS_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a bms_info message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_bms_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_bms_info_t* bms_info)
{
    _ttalink_bms_info_send_struct(dst_addr, src_addr, chan, bms_info, false);
}

/**
 * @brief Send a bms_info message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_bms_info_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_bms_info_t* bms_info)
{
    _ttalink_bms_info_send_struct(dst_addr, src_addr, chan, bms_info, true);
}

#if TTALINK_MSG_ID_BMS_INFO_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_bms_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t cycle_count, uint16_t overcharge_count, uint16_t overdischarge_count, uint16_t temperature_high_count, uint16_t temperature_low_count, uint16_t voltage_unbalanced, uint16_t discharge_current_count, uint16_t charge_current_count, uint8_t charging_time, uint16_t storage_voltage, uint16_t discharge_current, uint32_t design_capacity, uint16_t design_voltage, uint32_t manufacture_date, const uint8_t *serial_number, const uint8_t *version_number, const uint8_t *manufacturer_name, uint8_t bms_addr, uint8_t bms_param, const uint8_t *model_name, uint8_t device_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, cycle_count);
    _tta_put_uint16_t(buf, 8, overcharge_count);
    _tta_put_uint16_t(buf, 10, overdischarge_count);
    _tta_put_uint16_t(buf, 12, temperature_high_count);
    _tta_put_uint16_t(buf, 14, temperature_low_count);
    _tta_put_uint16_t(buf, 16, voltage_unbalanced);
    _tta_put_uint16_t(buf, 18, discharge_current_count);
    _tta_put_uint16_t(buf, 20, charge_current_count);
    _tta_put_uint8_t(buf, 22, charging_time);
    _tta_put_uint16_t(buf, 23, storage_voltage);
    _tta_put_uint16_t(buf, 25, discharge_current);
    _tta_put_uint32_t(buf, 27, design_capacity);
    _tta_put_uint16_t(buf, 31, design_voltage);
    _tta_put_uint32_t(buf, 33, manufacture_date);
    _tta_put_uint8_t(buf, 83, bms_addr);
    _tta_put_uint8_t(buf, 84, bms_param);
    _tta_put_uint8_t(buf, 93, device_type);
    _tta_put_uint8_t_array(buf, 37, serial_number, 16);
    _tta_put_uint8_t_array(buf, 53, version_number, 16);
    _tta_put_uint8_t_array(buf, 69, manufacturer_name, 14);
    _tta_put_uint8_t_array(buf, 85, model_name, 8);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_INFO, buf, TTALINK_MSG_ID_BMS_INFO_MIN_LEN, TTALINK_MSG_ID_BMS_INFO_LEN, TTALINK_MSG_ID_BMS_INFO_CRC, nocrc);
#else
    ttalink_bms_info_t *packet = (ttalink_bms_info_t *)msgbuf;
    packet->update_time = update_time;
    packet->cycle_count = cycle_count;
    packet->overcharge_count = overcharge_count;
    packet->overdischarge_count = overdischarge_count;
    packet->temperature_high_count = temperature_high_count;
    packet->temperature_low_count = temperature_low_count;
    packet->voltage_unbalanced = voltage_unbalanced;
    packet->discharge_current_count = discharge_current_count;
    packet->charge_current_count = charge_current_count;
    packet->charging_time = charging_time;
    packet->storage_voltage = storage_voltage;
    packet->discharge_current = discharge_current;
    packet->design_capacity = design_capacity;
    packet->design_voltage = design_voltage;
    packet->manufacture_date = manufacture_date;
    packet->bms_addr = bms_addr;
    packet->bms_param = bms_param;
    packet->device_type = device_type;
    tta_array_memcpy(packet->serial_number, serial_number, sizeof(uint8_t)*16);
    tta_array_memcpy(packet->version_number, version_number, sizeof(uint8_t)*16);
    tta_array_memcpy(packet->manufacturer_name, manufacturer_name, sizeof(uint8_t)*14);
    tta_array_memcpy(packet->model_name, model_name, sizeof(uint8_t)*8);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_INFO, (const char *)packet, TTALINK_MSG_ID_BMS_INFO_MIN_LEN, TTALINK_MSG_ID_BMS_INFO_LEN, TTALINK_MSG_ID_BMS_INFO_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_bms_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t cycle_count, uint16_t overcharge_count, uint16_t overdischarge_count, uint16_t temperature_high_count, uint16_t temperature_low_count, uint16_t voltage_unbalanced, uint16_t discharge_current_count, uint16_t charge_current_count, uint8_t charging_time, uint16_t storage_voltage, uint16_t discharge_current, uint32_t design_capacity, uint16_t design_voltage, uint32_t manufacture_date, const uint8_t *serial_number, const uint8_t *version_number, const uint8_t *manufacturer_name, uint8_t bms_addr, uint8_t bms_param, const uint8_t *model_name, uint8_t device_type)
{
    _ttalink_bms_info_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, cycle_count, overcharge_count, overdischarge_count, temperature_high_count, temperature_low_count, voltage_unbalanced, discharge_current_count, charge_current_count, charging_time, storage_voltage, discharge_current, design_capacity, design_voltage, manufacture_date, serial_number, version_number, manufacturer_name, bms_addr, bms_param, model_name, device_type, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_bms_info_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t cycle_count, uint16_t overcharge_count, uint16_t overdischarge_count, uint16_t temperature_high_count, uint16_t temperature_low_count, uint16_t voltage_unbalanced, uint16_t discharge_current_count, uint16_t charge_current_count, uint8_t charging_time, uint16_t storage_voltage, uint16_t discharge_current, uint32_t design_capacity, uint16_t design_voltage, uint32_t manufacture_date, const uint8_t *serial_number, const uint8_t *version_number, const uint8_t *manufacturer_name, uint8_t bms_addr, uint8_t bms_param, const uint8_t *model_name, uint8_t device_type)
{
    _ttalink_bms_info_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, cycle_count, overcharge_count, overdischarge_count, temperature_high_count, temperature_low_count, voltage_unbalanced, discharge_current_count, charge_current_count, charging_time, storage_voltage, discharge_current, design_capacity, design_voltage, manufacture_date, serial_number, version_number, manufacturer_name, bms_addr, bms_param, model_name, device_type, true);
}
#endif

#endif

// MESSAGE BMS_INFO UNPACKING


/**
 * @brief Get field update_time from bms_info message
 *
 * @return  . .
 */
static inline uint32_t ttalink_bms_info_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field cycle_count from bms_info message
 *
 * @return  .电池循环次数 .
 */
static inline uint32_t ttalink_bms_info_get_cycle_count(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field overcharge_count from bms_info message
 *
 * @return  .过充次数 .
 */
static inline uint16_t ttalink_bms_info_get_overcharge_count(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field overdischarge_count from bms_info message
 *
 * @return  . 过放次数.
 */
static inline uint16_t ttalink_bms_info_get_overdischarge_count(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field temperature_high_count from bms_info message
 *
 * @return  .温度过高次数 .
 */
static inline uint16_t ttalink_bms_info_get_temperature_high_count(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field temperature_low_count from bms_info message
 *
 * @return  .温度过低次数 .
 */
static inline uint16_t ttalink_bms_info_get_temperature_low_count(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field voltage_unbalanced from bms_info message
 *
 * @return  . 电压不均衡次数.
 */
static inline uint16_t ttalink_bms_info_get_voltage_unbalanced(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field discharge_current_count from bms_info message
 *
 * @return  . 放电电流过大次数.
 */
static inline uint16_t ttalink_bms_info_get_discharge_current_count(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field charge_current_count from bms_info message
 *
 * @return  .充电电流过大次数 .
 */
static inline uint16_t ttalink_bms_info_get_charge_current_count(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field charging_time from bms_info message
 *
 * @return  .电池安全充电时间(0.1h) .
 */
static inline uint8_t ttalink_bms_info_get_charging_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field storage_voltage from bms_info message
 *
 * @return  . 电池存储电压(0.01V).
 */
static inline uint16_t ttalink_bms_info_get_storage_voltage(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  23);
}

/**
 * @brief Get field discharge_current from bms_info message
 *
 * @return  . 自放电安全电流（0.01A）.
 */
static inline uint16_t ttalink_bms_info_get_discharge_current(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  25);
}

/**
 * @brief Get field design_capacity from bms_info message
 *
 * @return  . 电池组的设计容量值（mAh）.
 */
static inline uint32_t ttalink_bms_info_get_design_capacity(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  27);
}

/**
 * @brief Get field design_voltage from bms_info message
 *
 * @return  .电池组的设计电压值（0.01V） 
 */
static inline uint16_t ttalink_bms_info_get_design_voltage(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  31);
}

/**
 * @brief Get field manufacture_date from bms_info message
 *
 * @return  电池出厂日期. 
 */
static inline uint32_t ttalink_bms_info_get_manufacture_date(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  33);
}

/**
 * @brief Get field serial_number from bms_info message
 *
 * @return  出厂序列号. 
 */
static inline uint16_t ttalink_bms_info_get_serial_number(const ttalink_message_t* msg, uint8_t *serial_number)
{
    return _TTA_RETURN_uint8_t_array(msg, serial_number, 16,  37);
}

/**
 * @brief Get field version_number from bms_info message
 *
 * @return  出厂版本号. 
 */
static inline uint16_t ttalink_bms_info_get_version_number(const ttalink_message_t* msg, uint8_t *version_number)
{
    return _TTA_RETURN_uint8_t_array(msg, version_number, 16,  53);
}

/**
 * @brief Get field manufacturer_name from bms_info message
 *
 * @return  .制造厂商名称 .
 */
static inline uint16_t ttalink_bms_info_get_manufacturer_name(const ttalink_message_t* msg, uint8_t *manufacturer_name)
{
    return _TTA_RETURN_uint8_t_array(msg, manufacturer_name, 14,  69);
}

/**
 * @brief Get field bms_addr from bms_info message
 *
 * @return  . 
 */
static inline uint8_t ttalink_bms_info_get_bms_addr(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  83);
}

/**
 * @brief Get field bms_param from bms_info message
 *
 * @return  . 
 */
static inline uint8_t ttalink_bms_info_get_bms_param(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  84);
}

/**
 * @brief Get field model_name from bms_info message
 *
 * @return  .型号名称 .
 */
static inline uint16_t ttalink_bms_info_get_model_name(const ttalink_message_t* msg, uint8_t *model_name)
{
    return _TTA_RETURN_uint8_t_array(msg, model_name, 8,  85);
}

/**
 * @brief Get field device_type from bms_info message
 *
 * @return  器件类型. 
 */
static inline uint8_t ttalink_bms_info_get_device_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  93);
}

/**
 * @brief Decode a bms_info message into a struct
 *
 * @param msg The message to decode
 * @param bms_info C-struct to decode the message contents into
 */
static inline void ttalink_bms_info_decode(const ttalink_message_t* msg, ttalink_bms_info_t* bms_info)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    bms_info->update_time = ttalink_bms_info_get_update_time(msg);
    bms_info->cycle_count = ttalink_bms_info_get_cycle_count(msg);
    bms_info->overcharge_count = ttalink_bms_info_get_overcharge_count(msg);
    bms_info->overdischarge_count = ttalink_bms_info_get_overdischarge_count(msg);
    bms_info->temperature_high_count = ttalink_bms_info_get_temperature_high_count(msg);
    bms_info->temperature_low_count = ttalink_bms_info_get_temperature_low_count(msg);
    bms_info->voltage_unbalanced = ttalink_bms_info_get_voltage_unbalanced(msg);
    bms_info->discharge_current_count = ttalink_bms_info_get_discharge_current_count(msg);
    bms_info->charge_current_count = ttalink_bms_info_get_charge_current_count(msg);
    bms_info->charging_time = ttalink_bms_info_get_charging_time(msg);
    bms_info->storage_voltage = ttalink_bms_info_get_storage_voltage(msg);
    bms_info->discharge_current = ttalink_bms_info_get_discharge_current(msg);
    bms_info->design_capacity = ttalink_bms_info_get_design_capacity(msg);
    bms_info->design_voltage = ttalink_bms_info_get_design_voltage(msg);
    bms_info->manufacture_date = ttalink_bms_info_get_manufacture_date(msg);
    ttalink_bms_info_get_serial_number(msg, bms_info->serial_number);
    ttalink_bms_info_get_version_number(msg, bms_info->version_number);
    ttalink_bms_info_get_manufacturer_name(msg, bms_info->manufacturer_name);
    bms_info->bms_addr = ttalink_bms_info_get_bms_addr(msg);
    bms_info->bms_param = ttalink_bms_info_get_bms_param(msg);
    ttalink_bms_info_get_model_name(msg, bms_info->model_name);
    bms_info->device_type = ttalink_bms_info_get_device_type(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_BMS_INFO_LEN? msg->len : TTALINK_MSG_ID_BMS_INFO_LEN;
        memset(bms_info, 0, TTALINK_MSG_ID_BMS_INFO_LEN);
    memcpy(bms_info, _TTA_PAYLOAD(msg), len);
#endif
}
