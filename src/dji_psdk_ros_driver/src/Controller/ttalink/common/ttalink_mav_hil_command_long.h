#pragma once
// MESSAGE MAV_HIL_COMMAND_LONG PACKING

#define TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG 8276

TTAPACKED(
typedef struct __ttalink_mav_hil_command_long_t {
 uint8_t target_system; /*< System which should execute the command*/
 uint8_t target_component; /*< Component which should execute the command, 0 for all components*/
 uint16_t command; /*< Command ID, as defined by MAV_CMD enum.*/
 uint8_t confirmation; /*< 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)*/
 float param1; /*< Parameter 1, as defined by MAV_CMD enum.*/
 float param2; /*< Parameter 2, as defined by MAV_CMD enum.*/
 float param3; /*< Parameter 3, as defined by MAV_CMD enum.*/
 float param4; /*< Parameter 4, as defined by MAV_CMD enum.*/
 float param5; /*< Parameter 5, as defined by MAV_CMD enum.*/
 float param6; /*< Parameter 6, as defined by MAV_CMD enum.*/
 float param7; /*< Parameter 7, as defined by MAV_CMD enum.*/
}) ttalink_mav_hil_command_long_t;

#define TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN 33
#define TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_MIN_LEN 33
#define TTALINK_MSG_ID_8276_LEN 33
#define TTALINK_MSG_ID_8276_MIN_LEN 33

#define TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_CRC 193
#define TTALINK_MSG_ID_8276_CRC 193



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_MAV_HIL_COMMAND_LONG { \
    8276, \
    "MAV_HIL_COMMAND_LONG", \
    11, \
    {  { "target_system", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_mav_hil_command_long_t, target_system) }, \
         { "target_component", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_mav_hil_command_long_t, target_component) }, \
         { "command", NULL, TTALINK_TYPE_UINT16_T, 0, 2, offsetof(ttalink_mav_hil_command_long_t, command) }, \
         { "confirmation", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_mav_hil_command_long_t, confirmation) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 5, offsetof(ttalink_mav_hil_command_long_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 9, offsetof(ttalink_mav_hil_command_long_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_mav_hil_command_long_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 17, offsetof(ttalink_mav_hil_command_long_t, param4) }, \
         { "param5", NULL, TTALINK_TYPE_FLOAT, 0, 21, offsetof(ttalink_mav_hil_command_long_t, param5) }, \
         { "param6", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_mav_hil_command_long_t, param6) }, \
         { "param7", NULL, TTALINK_TYPE_FLOAT, 0, 29, offsetof(ttalink_mav_hil_command_long_t, param7) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_MAV_HIL_COMMAND_LONG { \
    "MAV_HIL_COMMAND_LONG", \
    11, \
    {  { "target_system", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_mav_hil_command_long_t, target_system) }, \
         { "target_component", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_mav_hil_command_long_t, target_component) }, \
         { "command", NULL, TTALINK_TYPE_UINT16_T, 0, 2, offsetof(ttalink_mav_hil_command_long_t, command) }, \
         { "confirmation", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_mav_hil_command_long_t, confirmation) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 5, offsetof(ttalink_mav_hil_command_long_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 9, offsetof(ttalink_mav_hil_command_long_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_mav_hil_command_long_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 17, offsetof(ttalink_mav_hil_command_long_t, param4) }, \
         { "param5", NULL, TTALINK_TYPE_FLOAT, 0, 21, offsetof(ttalink_mav_hil_command_long_t, param5) }, \
         { "param6", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_mav_hil_command_long_t, param6) }, \
         { "param7", NULL, TTALINK_TYPE_FLOAT, 0, 29, offsetof(ttalink_mav_hil_command_long_t, param7) }, \
         } \
}
#endif


static inline uint16_t _ttalink_mav_hil_command_long_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN];
    _tta_put_uint8_t(buf, 0, target_system);
    _tta_put_uint8_t(buf, 1, target_component);
    _tta_put_uint16_t(buf, 2, command);
    _tta_put_uint8_t(buf, 4, confirmation);
    _tta_put_float(buf, 5, param1);
    _tta_put_float(buf, 9, param2);
    _tta_put_float(buf, 13, param3);
    _tta_put_float(buf, 17, param4);
    _tta_put_float(buf, 21, param5);
    _tta_put_float(buf, 25, param6);
    _tta_put_float(buf, 29, param7);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN);
#else
    ttalink_mav_hil_command_long_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.command = command;
    packet.confirmation = confirmation;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_CRC, nocrc);
}

/**
 * @brief Pack a mav_hil_command_long message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @param param5 Parameter 5, as defined by MAV_CMD enum.
 * @param param6 Parameter 6, as defined by MAV_CMD enum.
 * @param param7 Parameter 7, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_command_long_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    return _ttalink_mav_hil_command_long_pack(dst_addr, src_addr, msg,  target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7, false);
}

/**
 * @brief Pack a mav_hil_command_long message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @param param5 Parameter 5, as defined by MAV_CMD enum.
 * @param param6 Parameter 6, as defined by MAV_CMD enum.
 * @param param7 Parameter 7, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_command_long_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    return _ttalink_mav_hil_command_long_pack(dst_addr, src_addr, msg,  target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7, true);
}


static inline uint16_t _ttalink_mav_hil_command_long_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint16_t command,uint8_t confirmation,float param1,float param2,float param3,float param4,float param5,float param6,float param7, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN];
    _tta_put_uint8_t(buf, 0, target_system);
    _tta_put_uint8_t(buf, 1, target_component);
    _tta_put_uint16_t(buf, 2, command);
    _tta_put_uint8_t(buf, 4, confirmation);
    _tta_put_float(buf, 5, param1);
    _tta_put_float(buf, 9, param2);
    _tta_put_float(buf, 13, param3);
    _tta_put_float(buf, 17, param4);
    _tta_put_float(buf, 21, param5);
    _tta_put_float(buf, 25, param6);
    _tta_put_float(buf, 29, param7);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN);
#else
    ttalink_mav_hil_command_long_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.command = command;
    packet.confirmation = confirmation;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_CRC, nocrc);
}

/**
 * @brief Pack a mav_hil_command_long message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @param param5 Parameter 5, as defined by MAV_CMD enum.
 * @param param6 Parameter 6, as defined by MAV_CMD enum.
 * @param param7 Parameter 7, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_command_long_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint16_t command,uint8_t confirmation,float param1,float param2,float param3,float param4,float param5,float param6,float param7)
{
    return _ttalink_mav_hil_command_long_pack_chan(dst_addr, src_addr, chan, msg,  target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7, false);
}

/**
 * @brief Pack a mav_hil_command_long message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @param param5 Parameter 5, as defined by MAV_CMD enum.
 * @param param6 Parameter 6, as defined by MAV_CMD enum.
 * @param param7 Parameter 7, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_command_long_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint16_t command,uint8_t confirmation,float param1,float param2,float param3,float param4,float param5,float param6,float param7)
{
    return _ttalink_mav_hil_command_long_pack_chan(dst_addr, src_addr, chan, msg,  target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7, true);
}


static inline uint16_t _ttalink_mav_hil_command_long_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_command_long_t* mav_hil_command_long, bool nocrc)
{
    if(nocrc){
        return ttalink_mav_hil_command_long_pack_nocrc(dst_addr, src_addr, msg, mav_hil_command_long->target_system, mav_hil_command_long->target_component, mav_hil_command_long->command, mav_hil_command_long->confirmation, mav_hil_command_long->param1, mav_hil_command_long->param2, mav_hil_command_long->param3, mav_hil_command_long->param4, mav_hil_command_long->param5, mav_hil_command_long->param6, mav_hil_command_long->param7);
    }else{
        return ttalink_mav_hil_command_long_pack(dst_addr, src_addr, msg, mav_hil_command_long->target_system, mav_hil_command_long->target_component, mav_hil_command_long->command, mav_hil_command_long->confirmation, mav_hil_command_long->param1, mav_hil_command_long->param2, mav_hil_command_long->param3, mav_hil_command_long->param4, mav_hil_command_long->param5, mav_hil_command_long->param6, mav_hil_command_long->param7);
    }
    
}

/**
 * @brief Encode a mav_hil_command_long struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_command_long C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_command_long_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_command_long_t* mav_hil_command_long)
{
    return _ttalink_mav_hil_command_long_encode(dst_addr, src_addr, msg, mav_hil_command_long, false);
}

/**
 * @brief Encode a mav_hil_command_long struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_command_long C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_command_long_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_command_long_t* mav_hil_command_long)
{
    return _ttalink_mav_hil_command_long_encode(dst_addr, src_addr, msg, mav_hil_command_long, true);
}


static inline uint16_t _ttalink_mav_hil_command_long_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_command_long_t* mav_hil_command_long, bool nocrc)
{
    if(nocrc){
        return ttalink_mav_hil_command_long_pack_chan_nocrc(dst_addr, src_addr, chan, msg, mav_hil_command_long->target_system, mav_hil_command_long->target_component, mav_hil_command_long->command, mav_hil_command_long->confirmation, mav_hil_command_long->param1, mav_hil_command_long->param2, mav_hil_command_long->param3, mav_hil_command_long->param4, mav_hil_command_long->param5, mav_hil_command_long->param6, mav_hil_command_long->param7);
    }else{
        return ttalink_mav_hil_command_long_pack_chan(dst_addr, src_addr, chan, msg, mav_hil_command_long->target_system, mav_hil_command_long->target_component, mav_hil_command_long->command, mav_hil_command_long->confirmation, mav_hil_command_long->param1, mav_hil_command_long->param2, mav_hil_command_long->param3, mav_hil_command_long->param4, mav_hil_command_long->param5, mav_hil_command_long->param6, mav_hil_command_long->param7);
    }
}

/**
 * @brief Encode a mav_hil_command_long struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_command_long C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_command_long_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_command_long_t* mav_hil_command_long)
{
    return _ttalink_mav_hil_command_long_encode_chan(dst_addr, src_addr, chan, msg, mav_hil_command_long, false);
}

/**
 * @brief Encode a mav_hil_command_long struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_command_long C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_command_long_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_command_long_t* mav_hil_command_long)
{
    return _ttalink_mav_hil_command_long_encode_chan(dst_addr, src_addr, chan, msg, mav_hil_command_long, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_mav_hil_command_long_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN];
    _tta_put_uint8_t(buf, 0, target_system);
    _tta_put_uint8_t(buf, 1, target_component);
    _tta_put_uint16_t(buf, 2, command);
    _tta_put_uint8_t(buf, 4, confirmation);
    _tta_put_float(buf, 5, param1);
    _tta_put_float(buf, 9, param2);
    _tta_put_float(buf, 13, param3);
    _tta_put_float(buf, 17, param4);
    _tta_put_float(buf, 21, param5);
    _tta_put_float(buf, 25, param6);
    _tta_put_float(buf, 29, param7);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG, buf, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_CRC, nocrc);
#else
    ttalink_mav_hil_command_long_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.command = command;
    packet.confirmation = confirmation;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG, (const char *)&packet, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_CRC, nocrc);
#endif
}

/**
 * @brief Send a mav_hil_command_long message
 * @param chan TTAlink channel to send the message
 *
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @param param5 Parameter 5, as defined by MAV_CMD enum.
 * @param param6 Parameter 6, as defined by MAV_CMD enum.
 * @param param7 Parameter 7, as defined by MAV_CMD enum.
 */
static inline void ttalink_mav_hil_command_long_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    _ttalink_mav_hil_command_long_send(dst_addr, src_addr, chan, target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7, false);
}

/**
 * @brief Send a mav_hil_command_long message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @param param5 Parameter 5, as defined by MAV_CMD enum.
 * @param param6 Parameter 6, as defined by MAV_CMD enum.
 * @param param7 Parameter 7, as defined by MAV_CMD enum.
 */
static inline void ttalink_mav_hil_command_long_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    _ttalink_mav_hil_command_long_send(dst_addr, src_addr, chan, target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7, true);
}


static inline void _ttalink_mav_hil_command_long_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_command_long_t* mav_hil_command_long, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_mav_hil_command_long_send_nocrc(dst_addr, src_addr, chan, mav_hil_command_long->target_system, mav_hil_command_long->target_component, mav_hil_command_long->command, mav_hil_command_long->confirmation, mav_hil_command_long->param1, mav_hil_command_long->param2, mav_hil_command_long->param3, mav_hil_command_long->param4, mav_hil_command_long->param5, mav_hil_command_long->param6, mav_hil_command_long->param7);
    }else{
        ttalink_mav_hil_command_long_send(dst_addr, src_addr, chan, mav_hil_command_long->target_system, mav_hil_command_long->target_component, mav_hil_command_long->command, mav_hil_command_long->confirmation, mav_hil_command_long->param1, mav_hil_command_long->param2, mav_hil_command_long->param3, mav_hil_command_long->param4, mav_hil_command_long->param5, mav_hil_command_long->param6, mav_hil_command_long->param7);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG, (const char *)mav_hil_command_long, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_CRC, nocrc);
#endif
}

/**
 * @brief Send a mav_hil_command_long message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_mav_hil_command_long_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_command_long_t* mav_hil_command_long)
{
    _ttalink_mav_hil_command_long_send_struct(dst_addr, src_addr, chan, mav_hil_command_long, false);
}

/**
 * @brief Send a mav_hil_command_long message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_mav_hil_command_long_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_command_long_t* mav_hil_command_long)
{
    _ttalink_mav_hil_command_long_send_struct(dst_addr, src_addr, chan, mav_hil_command_long, true);
}

#if TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_mav_hil_command_long_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, target_system);
    _tta_put_uint8_t(buf, 1, target_component);
    _tta_put_uint16_t(buf, 2, command);
    _tta_put_uint8_t(buf, 4, confirmation);
    _tta_put_float(buf, 5, param1);
    _tta_put_float(buf, 9, param2);
    _tta_put_float(buf, 13, param3);
    _tta_put_float(buf, 17, param4);
    _tta_put_float(buf, 21, param5);
    _tta_put_float(buf, 25, param6);
    _tta_put_float(buf, 29, param7);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG, buf, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_CRC, nocrc);
#else
    ttalink_mav_hil_command_long_t *packet = (ttalink_mav_hil_command_long_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->command = command;
    packet->confirmation = confirmation;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;
    packet->param5 = param5;
    packet->param6 = param6;
    packet->param7 = param7;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG, (const char *)packet, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_mav_hil_command_long_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    _ttalink_mav_hil_command_long_send_buf(dst_addr, src_addr, msgbuf, chan, target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_mav_hil_command_long_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    _ttalink_mav_hil_command_long_send_buf(dst_addr, src_addr, msgbuf, chan, target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7, true);
}
#endif

#endif

// MESSAGE MAV_HIL_COMMAND_LONG UNPACKING


/**
 * @brief Get field target_system from mav_hil_command_long message
 *
 * @return System which should execute the command
 */
static inline uint8_t ttalink_mav_hil_command_long_get_target_system(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from mav_hil_command_long message
 *
 * @return Component which should execute the command, 0 for all components
 */
static inline uint8_t ttalink_mav_hil_command_long_get_target_component(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field command from mav_hil_command_long message
 *
 * @return Command ID, as defined by MAV_CMD enum.
 */
static inline uint16_t ttalink_mav_hil_command_long_get_command(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field confirmation from mav_hil_command_long message
 *
 * @return 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 */
static inline uint8_t ttalink_mav_hil_command_long_get_confirmation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field param1 from mav_hil_command_long message
 *
 * @return Parameter 1, as defined by MAV_CMD enum.
 */
static inline float ttalink_mav_hil_command_long_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  5);
}

/**
 * @brief Get field param2 from mav_hil_command_long message
 *
 * @return Parameter 2, as defined by MAV_CMD enum.
 */
static inline float ttalink_mav_hil_command_long_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  9);
}

/**
 * @brief Get field param3 from mav_hil_command_long message
 *
 * @return Parameter 3, as defined by MAV_CMD enum.
 */
static inline float ttalink_mav_hil_command_long_get_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  13);
}

/**
 * @brief Get field param4 from mav_hil_command_long message
 *
 * @return Parameter 4, as defined by MAV_CMD enum.
 */
static inline float ttalink_mav_hil_command_long_get_param4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  17);
}

/**
 * @brief Get field param5 from mav_hil_command_long message
 *
 * @return Parameter 5, as defined by MAV_CMD enum.
 */
static inline float ttalink_mav_hil_command_long_get_param5(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  21);
}

/**
 * @brief Get field param6 from mav_hil_command_long message
 *
 * @return Parameter 6, as defined by MAV_CMD enum.
 */
static inline float ttalink_mav_hil_command_long_get_param6(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  25);
}

/**
 * @brief Get field param7 from mav_hil_command_long message
 *
 * @return Parameter 7, as defined by MAV_CMD enum.
 */
static inline float ttalink_mav_hil_command_long_get_param7(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  29);
}

/**
 * @brief Decode a mav_hil_command_long message into a struct
 *
 * @param msg The message to decode
 * @param mav_hil_command_long C-struct to decode the message contents into
 */
static inline void ttalink_mav_hil_command_long_decode(const ttalink_message_t* msg, ttalink_mav_hil_command_long_t* mav_hil_command_long)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    mav_hil_command_long->target_system = ttalink_mav_hil_command_long_get_target_system(msg);
    mav_hil_command_long->target_component = ttalink_mav_hil_command_long_get_target_component(msg);
    mav_hil_command_long->command = ttalink_mav_hil_command_long_get_command(msg);
    mav_hil_command_long->confirmation = ttalink_mav_hil_command_long_get_confirmation(msg);
    mav_hil_command_long->param1 = ttalink_mav_hil_command_long_get_param1(msg);
    mav_hil_command_long->param2 = ttalink_mav_hil_command_long_get_param2(msg);
    mav_hil_command_long->param3 = ttalink_mav_hil_command_long_get_param3(msg);
    mav_hil_command_long->param4 = ttalink_mav_hil_command_long_get_param4(msg);
    mav_hil_command_long->param5 = ttalink_mav_hil_command_long_get_param5(msg);
    mav_hil_command_long->param6 = ttalink_mav_hil_command_long_get_param6(msg);
    mav_hil_command_long->param7 = ttalink_mav_hil_command_long_get_param7(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN? msg->len : TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN;
        memset(mav_hil_command_long, 0, TTALINK_MSG_ID_MAV_HIL_COMMAND_LONG_LEN);
    memcpy(mav_hil_command_long, _TTA_PAYLOAD(msg), len);
#endif
}
