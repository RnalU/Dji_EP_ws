#pragma once
// MESSAGE FOC_REPORT_BASIC_INFO PACKING

#define TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO 2011

TTAPACKED(
typedef struct __ttalink_foc_report_basic_info_t {
 uint32_t update_time; /*< system time*/
 uint8_t motor_addr; /*< This is the motor addr.*/
 uint8_t foc_state_machine; /*< Running State Machine*/
 uint16_t st_foc_error_code; /*< error code*/
 uint16_t ti_foc_error_code; /*< error code*/
 int16_t current_rpm; /*< Current speed RPM*/
 int16_t current_torque; /*< Current Torque*/
 uint16_t current_power; /*< Current power Watt*/
 int16_t mosfet_temperature; /*< Current mosfet temperature*/
 int16_t motor_temperature; /*< Current motor temperature*/
 int16_t current_I_Q; /*< q axis current*/
 int16_t current_I_D; /*< d axis current*/
 uint16_t current_V; /*< voltage*/
 uint16_t Rx_Thro_Value; /*< Rx*/
 uint16_t Actual_Thro_Value; /*< Action*/
}) ttalink_foc_report_basic_info_t;

#define TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN 30
#define TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_MIN_LEN 30
#define TTALINK_MSG_ID_2011_LEN 30
#define TTALINK_MSG_ID_2011_MIN_LEN 30

#define TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_CRC 198
#define TTALINK_MSG_ID_2011_CRC 198



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_FOC_REPORT_BASIC_INFO { \
    2011, \
    "FOC_REPORT_BASIC_INFO", \
    15, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_foc_report_basic_info_t, update_time) }, \
         { "motor_addr", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_foc_report_basic_info_t, motor_addr) }, \
         { "foc_state_machine", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_foc_report_basic_info_t, foc_state_machine) }, \
         { "st_foc_error_code", NULL, TTALINK_TYPE_UINT16_T, 0, 6, offsetof(ttalink_foc_report_basic_info_t, st_foc_error_code) }, \
         { "ti_foc_error_code", NULL, TTALINK_TYPE_UINT16_T, 0, 8, offsetof(ttalink_foc_report_basic_info_t, ti_foc_error_code) }, \
         { "current_rpm", NULL, TTALINK_TYPE_INT16_T, 0, 10, offsetof(ttalink_foc_report_basic_info_t, current_rpm) }, \
         { "current_torque", NULL, TTALINK_TYPE_INT16_T, 0, 12, offsetof(ttalink_foc_report_basic_info_t, current_torque) }, \
         { "current_power", NULL, TTALINK_TYPE_UINT16_T, 0, 14, offsetof(ttalink_foc_report_basic_info_t, current_power) }, \
         { "mosfet_temperature", NULL, TTALINK_TYPE_INT16_T, 0, 16, offsetof(ttalink_foc_report_basic_info_t, mosfet_temperature) }, \
         { "motor_temperature", NULL, TTALINK_TYPE_INT16_T, 0, 18, offsetof(ttalink_foc_report_basic_info_t, motor_temperature) }, \
         { "current_I_Q", NULL, TTALINK_TYPE_INT16_T, 0, 20, offsetof(ttalink_foc_report_basic_info_t, current_I_Q) }, \
         { "current_I_D", NULL, TTALINK_TYPE_INT16_T, 0, 22, offsetof(ttalink_foc_report_basic_info_t, current_I_D) }, \
         { "current_V", NULL, TTALINK_TYPE_UINT16_T, 0, 24, offsetof(ttalink_foc_report_basic_info_t, current_V) }, \
         { "Rx_Thro_Value", NULL, TTALINK_TYPE_UINT16_T, 0, 26, offsetof(ttalink_foc_report_basic_info_t, Rx_Thro_Value) }, \
         { "Actual_Thro_Value", NULL, TTALINK_TYPE_UINT16_T, 0, 28, offsetof(ttalink_foc_report_basic_info_t, Actual_Thro_Value) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_FOC_REPORT_BASIC_INFO { \
    "FOC_REPORT_BASIC_INFO", \
    15, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_foc_report_basic_info_t, update_time) }, \
         { "motor_addr", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_foc_report_basic_info_t, motor_addr) }, \
         { "foc_state_machine", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_foc_report_basic_info_t, foc_state_machine) }, \
         { "st_foc_error_code", NULL, TTALINK_TYPE_UINT16_T, 0, 6, offsetof(ttalink_foc_report_basic_info_t, st_foc_error_code) }, \
         { "ti_foc_error_code", NULL, TTALINK_TYPE_UINT16_T, 0, 8, offsetof(ttalink_foc_report_basic_info_t, ti_foc_error_code) }, \
         { "current_rpm", NULL, TTALINK_TYPE_INT16_T, 0, 10, offsetof(ttalink_foc_report_basic_info_t, current_rpm) }, \
         { "current_torque", NULL, TTALINK_TYPE_INT16_T, 0, 12, offsetof(ttalink_foc_report_basic_info_t, current_torque) }, \
         { "current_power", NULL, TTALINK_TYPE_UINT16_T, 0, 14, offsetof(ttalink_foc_report_basic_info_t, current_power) }, \
         { "mosfet_temperature", NULL, TTALINK_TYPE_INT16_T, 0, 16, offsetof(ttalink_foc_report_basic_info_t, mosfet_temperature) }, \
         { "motor_temperature", NULL, TTALINK_TYPE_INT16_T, 0, 18, offsetof(ttalink_foc_report_basic_info_t, motor_temperature) }, \
         { "current_I_Q", NULL, TTALINK_TYPE_INT16_T, 0, 20, offsetof(ttalink_foc_report_basic_info_t, current_I_Q) }, \
         { "current_I_D", NULL, TTALINK_TYPE_INT16_T, 0, 22, offsetof(ttalink_foc_report_basic_info_t, current_I_D) }, \
         { "current_V", NULL, TTALINK_TYPE_UINT16_T, 0, 24, offsetof(ttalink_foc_report_basic_info_t, current_V) }, \
         { "Rx_Thro_Value", NULL, TTALINK_TYPE_UINT16_T, 0, 26, offsetof(ttalink_foc_report_basic_info_t, Rx_Thro_Value) }, \
         { "Actual_Thro_Value", NULL, TTALINK_TYPE_UINT16_T, 0, 28, offsetof(ttalink_foc_report_basic_info_t, Actual_Thro_Value) }, \
         } \
}
#endif


static inline uint16_t _ttalink_foc_report_basic_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t motor_addr, uint8_t foc_state_machine, uint16_t st_foc_error_code, uint16_t ti_foc_error_code, int16_t current_rpm, int16_t current_torque, uint16_t current_power, int16_t mosfet_temperature, int16_t motor_temperature, int16_t current_I_Q, int16_t current_I_D, uint16_t current_V, uint16_t Rx_Thro_Value, uint16_t Actual_Thro_Value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, motor_addr);
    _tta_put_uint8_t(buf, 5, foc_state_machine);
    _tta_put_uint16_t(buf, 6, st_foc_error_code);
    _tta_put_uint16_t(buf, 8, ti_foc_error_code);
    _tta_put_int16_t(buf, 10, current_rpm);
    _tta_put_int16_t(buf, 12, current_torque);
    _tta_put_uint16_t(buf, 14, current_power);
    _tta_put_int16_t(buf, 16, mosfet_temperature);
    _tta_put_int16_t(buf, 18, motor_temperature);
    _tta_put_int16_t(buf, 20, current_I_Q);
    _tta_put_int16_t(buf, 22, current_I_D);
    _tta_put_uint16_t(buf, 24, current_V);
    _tta_put_uint16_t(buf, 26, Rx_Thro_Value);
    _tta_put_uint16_t(buf, 28, Actual_Thro_Value);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN);
#else
    ttalink_foc_report_basic_info_t packet;
    packet.update_time = update_time;
    packet.motor_addr = motor_addr;
    packet.foc_state_machine = foc_state_machine;
    packet.st_foc_error_code = st_foc_error_code;
    packet.ti_foc_error_code = ti_foc_error_code;
    packet.current_rpm = current_rpm;
    packet.current_torque = current_torque;
    packet.current_power = current_power;
    packet.mosfet_temperature = mosfet_temperature;
    packet.motor_temperature = motor_temperature;
    packet.current_I_Q = current_I_Q;
    packet.current_I_D = current_I_D;
    packet.current_V = current_V;
    packet.Rx_Thro_Value = Rx_Thro_Value;
    packet.Actual_Thro_Value = Actual_Thro_Value;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_MIN_LEN, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_CRC, nocrc);
}

/**
 * @brief Pack a foc_report_basic_info message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time system time
 * @param motor_addr This is the motor addr.
 * @param foc_state_machine Running State Machine
 * @param st_foc_error_code error code
 * @param ti_foc_error_code error code
 * @param current_rpm Current speed RPM
 * @param current_torque Current Torque
 * @param current_power Current power Watt
 * @param mosfet_temperature Current mosfet temperature
 * @param motor_temperature Current motor temperature
 * @param current_I_Q q axis current
 * @param current_I_D d axis current
 * @param current_V voltage
 * @param Rx_Thro_Value Rx
 * @param Actual_Thro_Value Action
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_report_basic_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t motor_addr, uint8_t foc_state_machine, uint16_t st_foc_error_code, uint16_t ti_foc_error_code, int16_t current_rpm, int16_t current_torque, uint16_t current_power, int16_t mosfet_temperature, int16_t motor_temperature, int16_t current_I_Q, int16_t current_I_D, uint16_t current_V, uint16_t Rx_Thro_Value, uint16_t Actual_Thro_Value)
{
    return _ttalink_foc_report_basic_info_pack(dst_addr, src_addr, msg,  update_time, motor_addr, foc_state_machine, st_foc_error_code, ti_foc_error_code, current_rpm, current_torque, current_power, mosfet_temperature, motor_temperature, current_I_Q, current_I_D, current_V, Rx_Thro_Value, Actual_Thro_Value, false);
}

/**
 * @brief Pack a foc_report_basic_info message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time system time
 * @param motor_addr This is the motor addr.
 * @param foc_state_machine Running State Machine
 * @param st_foc_error_code error code
 * @param ti_foc_error_code error code
 * @param current_rpm Current speed RPM
 * @param current_torque Current Torque
 * @param current_power Current power Watt
 * @param mosfet_temperature Current mosfet temperature
 * @param motor_temperature Current motor temperature
 * @param current_I_Q q axis current
 * @param current_I_D d axis current
 * @param current_V voltage
 * @param Rx_Thro_Value Rx
 * @param Actual_Thro_Value Action
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_report_basic_info_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t motor_addr, uint8_t foc_state_machine, uint16_t st_foc_error_code, uint16_t ti_foc_error_code, int16_t current_rpm, int16_t current_torque, uint16_t current_power, int16_t mosfet_temperature, int16_t motor_temperature, int16_t current_I_Q, int16_t current_I_D, uint16_t current_V, uint16_t Rx_Thro_Value, uint16_t Actual_Thro_Value)
{
    return _ttalink_foc_report_basic_info_pack(dst_addr, src_addr, msg,  update_time, motor_addr, foc_state_machine, st_foc_error_code, ti_foc_error_code, current_rpm, current_torque, current_power, mosfet_temperature, motor_temperature, current_I_Q, current_I_D, current_V, Rx_Thro_Value, Actual_Thro_Value, true);
}


static inline uint16_t _ttalink_foc_report_basic_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t motor_addr,uint8_t foc_state_machine,uint16_t st_foc_error_code,uint16_t ti_foc_error_code,int16_t current_rpm,int16_t current_torque,uint16_t current_power,int16_t mosfet_temperature,int16_t motor_temperature,int16_t current_I_Q,int16_t current_I_D,uint16_t current_V,uint16_t Rx_Thro_Value,uint16_t Actual_Thro_Value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, motor_addr);
    _tta_put_uint8_t(buf, 5, foc_state_machine);
    _tta_put_uint16_t(buf, 6, st_foc_error_code);
    _tta_put_uint16_t(buf, 8, ti_foc_error_code);
    _tta_put_int16_t(buf, 10, current_rpm);
    _tta_put_int16_t(buf, 12, current_torque);
    _tta_put_uint16_t(buf, 14, current_power);
    _tta_put_int16_t(buf, 16, mosfet_temperature);
    _tta_put_int16_t(buf, 18, motor_temperature);
    _tta_put_int16_t(buf, 20, current_I_Q);
    _tta_put_int16_t(buf, 22, current_I_D);
    _tta_put_uint16_t(buf, 24, current_V);
    _tta_put_uint16_t(buf, 26, Rx_Thro_Value);
    _tta_put_uint16_t(buf, 28, Actual_Thro_Value);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN);
#else
    ttalink_foc_report_basic_info_t packet;
    packet.update_time = update_time;
    packet.motor_addr = motor_addr;
    packet.foc_state_machine = foc_state_machine;
    packet.st_foc_error_code = st_foc_error_code;
    packet.ti_foc_error_code = ti_foc_error_code;
    packet.current_rpm = current_rpm;
    packet.current_torque = current_torque;
    packet.current_power = current_power;
    packet.mosfet_temperature = mosfet_temperature;
    packet.motor_temperature = motor_temperature;
    packet.current_I_Q = current_I_Q;
    packet.current_I_D = current_I_D;
    packet.current_V = current_V;
    packet.Rx_Thro_Value = Rx_Thro_Value;
    packet.Actual_Thro_Value = Actual_Thro_Value;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_MIN_LEN, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_CRC, nocrc);
}

/**
 * @brief Pack a foc_report_basic_info message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time system time
 * @param motor_addr This is the motor addr.
 * @param foc_state_machine Running State Machine
 * @param st_foc_error_code error code
 * @param ti_foc_error_code error code
 * @param current_rpm Current speed RPM
 * @param current_torque Current Torque
 * @param current_power Current power Watt
 * @param mosfet_temperature Current mosfet temperature
 * @param motor_temperature Current motor temperature
 * @param current_I_Q q axis current
 * @param current_I_D d axis current
 * @param current_V voltage
 * @param Rx_Thro_Value Rx
 * @param Actual_Thro_Value Action
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_report_basic_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t motor_addr,uint8_t foc_state_machine,uint16_t st_foc_error_code,uint16_t ti_foc_error_code,int16_t current_rpm,int16_t current_torque,uint16_t current_power,int16_t mosfet_temperature,int16_t motor_temperature,int16_t current_I_Q,int16_t current_I_D,uint16_t current_V,uint16_t Rx_Thro_Value,uint16_t Actual_Thro_Value)
{
    return _ttalink_foc_report_basic_info_pack_chan(dst_addr, src_addr, chan, msg,  update_time, motor_addr, foc_state_machine, st_foc_error_code, ti_foc_error_code, current_rpm, current_torque, current_power, mosfet_temperature, motor_temperature, current_I_Q, current_I_D, current_V, Rx_Thro_Value, Actual_Thro_Value, false);
}

/**
 * @brief Pack a foc_report_basic_info message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time system time
 * @param motor_addr This is the motor addr.
 * @param foc_state_machine Running State Machine
 * @param st_foc_error_code error code
 * @param ti_foc_error_code error code
 * @param current_rpm Current speed RPM
 * @param current_torque Current Torque
 * @param current_power Current power Watt
 * @param mosfet_temperature Current mosfet temperature
 * @param motor_temperature Current motor temperature
 * @param current_I_Q q axis current
 * @param current_I_D d axis current
 * @param current_V voltage
 * @param Rx_Thro_Value Rx
 * @param Actual_Thro_Value Action
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_report_basic_info_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t motor_addr,uint8_t foc_state_machine,uint16_t st_foc_error_code,uint16_t ti_foc_error_code,int16_t current_rpm,int16_t current_torque,uint16_t current_power,int16_t mosfet_temperature,int16_t motor_temperature,int16_t current_I_Q,int16_t current_I_D,uint16_t current_V,uint16_t Rx_Thro_Value,uint16_t Actual_Thro_Value)
{
    return _ttalink_foc_report_basic_info_pack_chan(dst_addr, src_addr, chan, msg,  update_time, motor_addr, foc_state_machine, st_foc_error_code, ti_foc_error_code, current_rpm, current_torque, current_power, mosfet_temperature, motor_temperature, current_I_Q, current_I_D, current_V, Rx_Thro_Value, Actual_Thro_Value, true);
}


static inline uint16_t _ttalink_foc_report_basic_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_report_basic_info_t* foc_report_basic_info, bool nocrc)
{
    if(nocrc){
        return ttalink_foc_report_basic_info_pack_nocrc(dst_addr, src_addr, msg, foc_report_basic_info->update_time, foc_report_basic_info->motor_addr, foc_report_basic_info->foc_state_machine, foc_report_basic_info->st_foc_error_code, foc_report_basic_info->ti_foc_error_code, foc_report_basic_info->current_rpm, foc_report_basic_info->current_torque, foc_report_basic_info->current_power, foc_report_basic_info->mosfet_temperature, foc_report_basic_info->motor_temperature, foc_report_basic_info->current_I_Q, foc_report_basic_info->current_I_D, foc_report_basic_info->current_V, foc_report_basic_info->Rx_Thro_Value, foc_report_basic_info->Actual_Thro_Value);
    }else{
        return ttalink_foc_report_basic_info_pack(dst_addr, src_addr, msg, foc_report_basic_info->update_time, foc_report_basic_info->motor_addr, foc_report_basic_info->foc_state_machine, foc_report_basic_info->st_foc_error_code, foc_report_basic_info->ti_foc_error_code, foc_report_basic_info->current_rpm, foc_report_basic_info->current_torque, foc_report_basic_info->current_power, foc_report_basic_info->mosfet_temperature, foc_report_basic_info->motor_temperature, foc_report_basic_info->current_I_Q, foc_report_basic_info->current_I_D, foc_report_basic_info->current_V, foc_report_basic_info->Rx_Thro_Value, foc_report_basic_info->Actual_Thro_Value);
    }
    
}

/**
 * @brief Encode a foc_report_basic_info struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param foc_report_basic_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_report_basic_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_report_basic_info_t* foc_report_basic_info)
{
    return _ttalink_foc_report_basic_info_encode(dst_addr, src_addr, msg, foc_report_basic_info, false);
}

/**
 * @brief Encode a foc_report_basic_info struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param foc_report_basic_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_report_basic_info_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_report_basic_info_t* foc_report_basic_info)
{
    return _ttalink_foc_report_basic_info_encode(dst_addr, src_addr, msg, foc_report_basic_info, true);
}


static inline uint16_t _ttalink_foc_report_basic_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_report_basic_info_t* foc_report_basic_info, bool nocrc)
{
    if(nocrc){
        return ttalink_foc_report_basic_info_pack_chan_nocrc(dst_addr, src_addr, chan, msg, foc_report_basic_info->update_time, foc_report_basic_info->motor_addr, foc_report_basic_info->foc_state_machine, foc_report_basic_info->st_foc_error_code, foc_report_basic_info->ti_foc_error_code, foc_report_basic_info->current_rpm, foc_report_basic_info->current_torque, foc_report_basic_info->current_power, foc_report_basic_info->mosfet_temperature, foc_report_basic_info->motor_temperature, foc_report_basic_info->current_I_Q, foc_report_basic_info->current_I_D, foc_report_basic_info->current_V, foc_report_basic_info->Rx_Thro_Value, foc_report_basic_info->Actual_Thro_Value);
    }else{
        return ttalink_foc_report_basic_info_pack_chan(dst_addr, src_addr, chan, msg, foc_report_basic_info->update_time, foc_report_basic_info->motor_addr, foc_report_basic_info->foc_state_machine, foc_report_basic_info->st_foc_error_code, foc_report_basic_info->ti_foc_error_code, foc_report_basic_info->current_rpm, foc_report_basic_info->current_torque, foc_report_basic_info->current_power, foc_report_basic_info->mosfet_temperature, foc_report_basic_info->motor_temperature, foc_report_basic_info->current_I_Q, foc_report_basic_info->current_I_D, foc_report_basic_info->current_V, foc_report_basic_info->Rx_Thro_Value, foc_report_basic_info->Actual_Thro_Value);
    }
}

/**
 * @brief Encode a foc_report_basic_info struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param foc_report_basic_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_report_basic_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_report_basic_info_t* foc_report_basic_info)
{
    return _ttalink_foc_report_basic_info_encode_chan(dst_addr, src_addr, chan, msg, foc_report_basic_info, false);
}

/**
 * @brief Encode a foc_report_basic_info struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param foc_report_basic_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_report_basic_info_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_report_basic_info_t* foc_report_basic_info)
{
    return _ttalink_foc_report_basic_info_encode_chan(dst_addr, src_addr, chan, msg, foc_report_basic_info, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_foc_report_basic_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t motor_addr, uint8_t foc_state_machine, uint16_t st_foc_error_code, uint16_t ti_foc_error_code, int16_t current_rpm, int16_t current_torque, uint16_t current_power, int16_t mosfet_temperature, int16_t motor_temperature, int16_t current_I_Q, int16_t current_I_D, uint16_t current_V, uint16_t Rx_Thro_Value, uint16_t Actual_Thro_Value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, motor_addr);
    _tta_put_uint8_t(buf, 5, foc_state_machine);
    _tta_put_uint16_t(buf, 6, st_foc_error_code);
    _tta_put_uint16_t(buf, 8, ti_foc_error_code);
    _tta_put_int16_t(buf, 10, current_rpm);
    _tta_put_int16_t(buf, 12, current_torque);
    _tta_put_uint16_t(buf, 14, current_power);
    _tta_put_int16_t(buf, 16, mosfet_temperature);
    _tta_put_int16_t(buf, 18, motor_temperature);
    _tta_put_int16_t(buf, 20, current_I_Q);
    _tta_put_int16_t(buf, 22, current_I_D);
    _tta_put_uint16_t(buf, 24, current_V);
    _tta_put_uint16_t(buf, 26, Rx_Thro_Value);
    _tta_put_uint16_t(buf, 28, Actual_Thro_Value);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO, buf, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_MIN_LEN, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_CRC, nocrc);
#else
    ttalink_foc_report_basic_info_t packet;
    packet.update_time = update_time;
    packet.motor_addr = motor_addr;
    packet.foc_state_machine = foc_state_machine;
    packet.st_foc_error_code = st_foc_error_code;
    packet.ti_foc_error_code = ti_foc_error_code;
    packet.current_rpm = current_rpm;
    packet.current_torque = current_torque;
    packet.current_power = current_power;
    packet.mosfet_temperature = mosfet_temperature;
    packet.motor_temperature = motor_temperature;
    packet.current_I_Q = current_I_Q;
    packet.current_I_D = current_I_D;
    packet.current_V = current_V;
    packet.Rx_Thro_Value = Rx_Thro_Value;
    packet.Actual_Thro_Value = Actual_Thro_Value;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO, (const char *)&packet, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_MIN_LEN, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a foc_report_basic_info message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time system time
 * @param motor_addr This is the motor addr.
 * @param foc_state_machine Running State Machine
 * @param st_foc_error_code error code
 * @param ti_foc_error_code error code
 * @param current_rpm Current speed RPM
 * @param current_torque Current Torque
 * @param current_power Current power Watt
 * @param mosfet_temperature Current mosfet temperature
 * @param motor_temperature Current motor temperature
 * @param current_I_Q q axis current
 * @param current_I_D d axis current
 * @param current_V voltage
 * @param Rx_Thro_Value Rx
 * @param Actual_Thro_Value Action
 */
static inline void ttalink_foc_report_basic_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t motor_addr, uint8_t foc_state_machine, uint16_t st_foc_error_code, uint16_t ti_foc_error_code, int16_t current_rpm, int16_t current_torque, uint16_t current_power, int16_t mosfet_temperature, int16_t motor_temperature, int16_t current_I_Q, int16_t current_I_D, uint16_t current_V, uint16_t Rx_Thro_Value, uint16_t Actual_Thro_Value)
{
    _ttalink_foc_report_basic_info_send(dst_addr, src_addr, chan, update_time, motor_addr, foc_state_machine, st_foc_error_code, ti_foc_error_code, current_rpm, current_torque, current_power, mosfet_temperature, motor_temperature, current_I_Q, current_I_D, current_V, Rx_Thro_Value, Actual_Thro_Value, false);
}

/**
 * @brief Send a foc_report_basic_info message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time system time
 * @param motor_addr This is the motor addr.
 * @param foc_state_machine Running State Machine
 * @param st_foc_error_code error code
 * @param ti_foc_error_code error code
 * @param current_rpm Current speed RPM
 * @param current_torque Current Torque
 * @param current_power Current power Watt
 * @param mosfet_temperature Current mosfet temperature
 * @param motor_temperature Current motor temperature
 * @param current_I_Q q axis current
 * @param current_I_D d axis current
 * @param current_V voltage
 * @param Rx_Thro_Value Rx
 * @param Actual_Thro_Value Action
 */
static inline void ttalink_foc_report_basic_info_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t motor_addr, uint8_t foc_state_machine, uint16_t st_foc_error_code, uint16_t ti_foc_error_code, int16_t current_rpm, int16_t current_torque, uint16_t current_power, int16_t mosfet_temperature, int16_t motor_temperature, int16_t current_I_Q, int16_t current_I_D, uint16_t current_V, uint16_t Rx_Thro_Value, uint16_t Actual_Thro_Value)
{
    _ttalink_foc_report_basic_info_send(dst_addr, src_addr, chan, update_time, motor_addr, foc_state_machine, st_foc_error_code, ti_foc_error_code, current_rpm, current_torque, current_power, mosfet_temperature, motor_temperature, current_I_Q, current_I_D, current_V, Rx_Thro_Value, Actual_Thro_Value, true);
}


static inline void _ttalink_foc_report_basic_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_report_basic_info_t* foc_report_basic_info, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_foc_report_basic_info_send_nocrc(dst_addr, src_addr, chan, foc_report_basic_info->update_time, foc_report_basic_info->motor_addr, foc_report_basic_info->foc_state_machine, foc_report_basic_info->st_foc_error_code, foc_report_basic_info->ti_foc_error_code, foc_report_basic_info->current_rpm, foc_report_basic_info->current_torque, foc_report_basic_info->current_power, foc_report_basic_info->mosfet_temperature, foc_report_basic_info->motor_temperature, foc_report_basic_info->current_I_Q, foc_report_basic_info->current_I_D, foc_report_basic_info->current_V, foc_report_basic_info->Rx_Thro_Value, foc_report_basic_info->Actual_Thro_Value);
    }else{
        ttalink_foc_report_basic_info_send(dst_addr, src_addr, chan, foc_report_basic_info->update_time, foc_report_basic_info->motor_addr, foc_report_basic_info->foc_state_machine, foc_report_basic_info->st_foc_error_code, foc_report_basic_info->ti_foc_error_code, foc_report_basic_info->current_rpm, foc_report_basic_info->current_torque, foc_report_basic_info->current_power, foc_report_basic_info->mosfet_temperature, foc_report_basic_info->motor_temperature, foc_report_basic_info->current_I_Q, foc_report_basic_info->current_I_D, foc_report_basic_info->current_V, foc_report_basic_info->Rx_Thro_Value, foc_report_basic_info->Actual_Thro_Value);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO, (const char *)foc_report_basic_info, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_MIN_LEN, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a foc_report_basic_info message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_foc_report_basic_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_report_basic_info_t* foc_report_basic_info)
{
    _ttalink_foc_report_basic_info_send_struct(dst_addr, src_addr, chan, foc_report_basic_info, false);
}

/**
 * @brief Send a foc_report_basic_info message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_foc_report_basic_info_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_report_basic_info_t* foc_report_basic_info)
{
    _ttalink_foc_report_basic_info_send_struct(dst_addr, src_addr, chan, foc_report_basic_info, true);
}

#if TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_foc_report_basic_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t motor_addr, uint8_t foc_state_machine, uint16_t st_foc_error_code, uint16_t ti_foc_error_code, int16_t current_rpm, int16_t current_torque, uint16_t current_power, int16_t mosfet_temperature, int16_t motor_temperature, int16_t current_I_Q, int16_t current_I_D, uint16_t current_V, uint16_t Rx_Thro_Value, uint16_t Actual_Thro_Value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, motor_addr);
    _tta_put_uint8_t(buf, 5, foc_state_machine);
    _tta_put_uint16_t(buf, 6, st_foc_error_code);
    _tta_put_uint16_t(buf, 8, ti_foc_error_code);
    _tta_put_int16_t(buf, 10, current_rpm);
    _tta_put_int16_t(buf, 12, current_torque);
    _tta_put_uint16_t(buf, 14, current_power);
    _tta_put_int16_t(buf, 16, mosfet_temperature);
    _tta_put_int16_t(buf, 18, motor_temperature);
    _tta_put_int16_t(buf, 20, current_I_Q);
    _tta_put_int16_t(buf, 22, current_I_D);
    _tta_put_uint16_t(buf, 24, current_V);
    _tta_put_uint16_t(buf, 26, Rx_Thro_Value);
    _tta_put_uint16_t(buf, 28, Actual_Thro_Value);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO, buf, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_MIN_LEN, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_CRC, nocrc);
#else
    ttalink_foc_report_basic_info_t *packet = (ttalink_foc_report_basic_info_t *)msgbuf;
    packet->update_time = update_time;
    packet->motor_addr = motor_addr;
    packet->foc_state_machine = foc_state_machine;
    packet->st_foc_error_code = st_foc_error_code;
    packet->ti_foc_error_code = ti_foc_error_code;
    packet->current_rpm = current_rpm;
    packet->current_torque = current_torque;
    packet->current_power = current_power;
    packet->mosfet_temperature = mosfet_temperature;
    packet->motor_temperature = motor_temperature;
    packet->current_I_Q = current_I_Q;
    packet->current_I_D = current_I_D;
    packet->current_V = current_V;
    packet->Rx_Thro_Value = Rx_Thro_Value;
    packet->Actual_Thro_Value = Actual_Thro_Value;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO, (const char *)packet, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_MIN_LEN, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_foc_report_basic_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t motor_addr, uint8_t foc_state_machine, uint16_t st_foc_error_code, uint16_t ti_foc_error_code, int16_t current_rpm, int16_t current_torque, uint16_t current_power, int16_t mosfet_temperature, int16_t motor_temperature, int16_t current_I_Q, int16_t current_I_D, uint16_t current_V, uint16_t Rx_Thro_Value, uint16_t Actual_Thro_Value)
{
    _ttalink_foc_report_basic_info_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, motor_addr, foc_state_machine, st_foc_error_code, ti_foc_error_code, current_rpm, current_torque, current_power, mosfet_temperature, motor_temperature, current_I_Q, current_I_D, current_V, Rx_Thro_Value, Actual_Thro_Value, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_foc_report_basic_info_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t motor_addr, uint8_t foc_state_machine, uint16_t st_foc_error_code, uint16_t ti_foc_error_code, int16_t current_rpm, int16_t current_torque, uint16_t current_power, int16_t mosfet_temperature, int16_t motor_temperature, int16_t current_I_Q, int16_t current_I_D, uint16_t current_V, uint16_t Rx_Thro_Value, uint16_t Actual_Thro_Value)
{
    _ttalink_foc_report_basic_info_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, motor_addr, foc_state_machine, st_foc_error_code, ti_foc_error_code, current_rpm, current_torque, current_power, mosfet_temperature, motor_temperature, current_I_Q, current_I_D, current_V, Rx_Thro_Value, Actual_Thro_Value, true);
}
#endif

#endif

// MESSAGE FOC_REPORT_BASIC_INFO UNPACKING


/**
 * @brief Get field update_time from foc_report_basic_info message
 *
 * @return system time
 */
static inline uint32_t ttalink_foc_report_basic_info_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field motor_addr from foc_report_basic_info message
 *
 * @return This is the motor addr.
 */
static inline uint8_t ttalink_foc_report_basic_info_get_motor_addr(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field foc_state_machine from foc_report_basic_info message
 *
 * @return Running State Machine
 */
static inline uint8_t ttalink_foc_report_basic_info_get_foc_state_machine(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field st_foc_error_code from foc_report_basic_info message
 *
 * @return error code
 */
static inline uint16_t ttalink_foc_report_basic_info_get_st_foc_error_code(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field ti_foc_error_code from foc_report_basic_info message
 *
 * @return error code
 */
static inline uint16_t ttalink_foc_report_basic_info_get_ti_foc_error_code(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field current_rpm from foc_report_basic_info message
 *
 * @return Current speed RPM
 */
static inline int16_t ttalink_foc_report_basic_info_get_current_rpm(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field current_torque from foc_report_basic_info message
 *
 * @return Current Torque
 */
static inline int16_t ttalink_foc_report_basic_info_get_current_torque(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field current_power from foc_report_basic_info message
 *
 * @return Current power Watt
 */
static inline uint16_t ttalink_foc_report_basic_info_get_current_power(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field mosfet_temperature from foc_report_basic_info message
 *
 * @return Current mosfet temperature
 */
static inline int16_t ttalink_foc_report_basic_info_get_mosfet_temperature(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field motor_temperature from foc_report_basic_info message
 *
 * @return Current motor temperature
 */
static inline int16_t ttalink_foc_report_basic_info_get_motor_temperature(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field current_I_Q from foc_report_basic_info message
 *
 * @return q axis current
 */
static inline int16_t ttalink_foc_report_basic_info_get_current_I_Q(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field current_I_D from foc_report_basic_info message
 *
 * @return d axis current
 */
static inline int16_t ttalink_foc_report_basic_info_get_current_I_D(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field current_V from foc_report_basic_info message
 *
 * @return voltage
 */
static inline uint16_t ttalink_foc_report_basic_info_get_current_V(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field Rx_Thro_Value from foc_report_basic_info message
 *
 * @return Rx
 */
static inline uint16_t ttalink_foc_report_basic_info_get_Rx_Thro_Value(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field Actual_Thro_Value from foc_report_basic_info message
 *
 * @return Action
 */
static inline uint16_t ttalink_foc_report_basic_info_get_Actual_Thro_Value(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Decode a foc_report_basic_info message into a struct
 *
 * @param msg The message to decode
 * @param foc_report_basic_info C-struct to decode the message contents into
 */
static inline void ttalink_foc_report_basic_info_decode(const ttalink_message_t* msg, ttalink_foc_report_basic_info_t* foc_report_basic_info)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    foc_report_basic_info->update_time = ttalink_foc_report_basic_info_get_update_time(msg);
    foc_report_basic_info->motor_addr = ttalink_foc_report_basic_info_get_motor_addr(msg);
    foc_report_basic_info->foc_state_machine = ttalink_foc_report_basic_info_get_foc_state_machine(msg);
    foc_report_basic_info->st_foc_error_code = ttalink_foc_report_basic_info_get_st_foc_error_code(msg);
    foc_report_basic_info->ti_foc_error_code = ttalink_foc_report_basic_info_get_ti_foc_error_code(msg);
    foc_report_basic_info->current_rpm = ttalink_foc_report_basic_info_get_current_rpm(msg);
    foc_report_basic_info->current_torque = ttalink_foc_report_basic_info_get_current_torque(msg);
    foc_report_basic_info->current_power = ttalink_foc_report_basic_info_get_current_power(msg);
    foc_report_basic_info->mosfet_temperature = ttalink_foc_report_basic_info_get_mosfet_temperature(msg);
    foc_report_basic_info->motor_temperature = ttalink_foc_report_basic_info_get_motor_temperature(msg);
    foc_report_basic_info->current_I_Q = ttalink_foc_report_basic_info_get_current_I_Q(msg);
    foc_report_basic_info->current_I_D = ttalink_foc_report_basic_info_get_current_I_D(msg);
    foc_report_basic_info->current_V = ttalink_foc_report_basic_info_get_current_V(msg);
    foc_report_basic_info->Rx_Thro_Value = ttalink_foc_report_basic_info_get_Rx_Thro_Value(msg);
    foc_report_basic_info->Actual_Thro_Value = ttalink_foc_report_basic_info_get_Actual_Thro_Value(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN? msg->len : TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN;
        memset(foc_report_basic_info, 0, TTALINK_MSG_ID_FOC_REPORT_BASIC_INFO_LEN);
    memcpy(foc_report_basic_info, _TTA_PAYLOAD(msg), len);
#endif
}
