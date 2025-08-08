#pragma once
// MESSAGE RADAR_SENSORS_DATA PACKING

#define TTALINK_MSG_ID_RADAR_SENSORS_DATA 2174

TTAPACKED(
typedef struct __ttalink_radar_sensors_data_t {
 uint8_t type; /*< 0:NUll 1：定高雷达 2 :前雷达 3：后雷达  4：旋转雷达.*/
 uint8_t target_num; /*< This is number of target.*/
 uint32_t update_time; /*< This is the update time.*/
 float distance_E; /*< This is the distance_E.*/
 float distance_Var; /*< This is the distance_Var.*/
 float SNR_E; /*< This is the SNR_E.*/
 float SNR_Var; /*< This is the SNR_Var.*/
 uint32_t distance1; /*< This is the distance1.*/
 uint32_t SNR1; /*< This is the SNR1 of distance1.*/
 uint32_t distance2; /*< This is the distance2.*/
 uint32_t SNR2; /*< This is the SNR2 of distance2.*/
 uint32_t distance3; /*< This is the distance3.*/
 uint32_t SNR3; /*< This is the SNR3 of distance3.*/
 uint32_t distance4; /*< This is the distance4.*/
 uint32_t SNR4; /*< This is the SNR4 of distance4.*/
 uint32_t distance5; /*< This is the distance5.*/
 uint32_t SNR5; /*< This is the SNR5 of distance5.*/
}) ttalink_radar_sensors_data_t;

#define TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN 62
#define TTALINK_MSG_ID_RADAR_SENSORS_DATA_MIN_LEN 62
#define TTALINK_MSG_ID_2174_LEN 62
#define TTALINK_MSG_ID_2174_MIN_LEN 62

#define TTALINK_MSG_ID_RADAR_SENSORS_DATA_CRC 67
#define TTALINK_MSG_ID_2174_CRC 67



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_RADAR_SENSORS_DATA { \
    2174, \
    "RADAR_SENSORS_DATA", \
    17, \
    {  { "type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_radar_sensors_data_t, type) }, \
         { "target_num", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_radar_sensors_data_t, target_num) }, \
         { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 2, offsetof(ttalink_radar_sensors_data_t, update_time) }, \
         { "distance_E", NULL, TTALINK_TYPE_FLOAT, 0, 6, offsetof(ttalink_radar_sensors_data_t, distance_E) }, \
         { "distance_Var", NULL, TTALINK_TYPE_FLOAT, 0, 10, offsetof(ttalink_radar_sensors_data_t, distance_Var) }, \
         { "SNR_E", NULL, TTALINK_TYPE_FLOAT, 0, 14, offsetof(ttalink_radar_sensors_data_t, SNR_E) }, \
         { "SNR_Var", NULL, TTALINK_TYPE_FLOAT, 0, 18, offsetof(ttalink_radar_sensors_data_t, SNR_Var) }, \
         { "distance1", NULL, TTALINK_TYPE_UINT32_T, 0, 22, offsetof(ttalink_radar_sensors_data_t, distance1) }, \
         { "SNR1", NULL, TTALINK_TYPE_UINT32_T, 0, 26, offsetof(ttalink_radar_sensors_data_t, SNR1) }, \
         { "distance2", NULL, TTALINK_TYPE_UINT32_T, 0, 30, offsetof(ttalink_radar_sensors_data_t, distance2) }, \
         { "SNR2", NULL, TTALINK_TYPE_UINT32_T, 0, 34, offsetof(ttalink_radar_sensors_data_t, SNR2) }, \
         { "distance3", NULL, TTALINK_TYPE_UINT32_T, 0, 38, offsetof(ttalink_radar_sensors_data_t, distance3) }, \
         { "SNR3", NULL, TTALINK_TYPE_UINT32_T, 0, 42, offsetof(ttalink_radar_sensors_data_t, SNR3) }, \
         { "distance4", NULL, TTALINK_TYPE_UINT32_T, 0, 46, offsetof(ttalink_radar_sensors_data_t, distance4) }, \
         { "SNR4", NULL, TTALINK_TYPE_UINT32_T, 0, 50, offsetof(ttalink_radar_sensors_data_t, SNR4) }, \
         { "distance5", NULL, TTALINK_TYPE_UINT32_T, 0, 54, offsetof(ttalink_radar_sensors_data_t, distance5) }, \
         { "SNR5", NULL, TTALINK_TYPE_UINT32_T, 0, 58, offsetof(ttalink_radar_sensors_data_t, SNR5) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_RADAR_SENSORS_DATA { \
    "RADAR_SENSORS_DATA", \
    17, \
    {  { "type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_radar_sensors_data_t, type) }, \
         { "target_num", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_radar_sensors_data_t, target_num) }, \
         { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 2, offsetof(ttalink_radar_sensors_data_t, update_time) }, \
         { "distance_E", NULL, TTALINK_TYPE_FLOAT, 0, 6, offsetof(ttalink_radar_sensors_data_t, distance_E) }, \
         { "distance_Var", NULL, TTALINK_TYPE_FLOAT, 0, 10, offsetof(ttalink_radar_sensors_data_t, distance_Var) }, \
         { "SNR_E", NULL, TTALINK_TYPE_FLOAT, 0, 14, offsetof(ttalink_radar_sensors_data_t, SNR_E) }, \
         { "SNR_Var", NULL, TTALINK_TYPE_FLOAT, 0, 18, offsetof(ttalink_radar_sensors_data_t, SNR_Var) }, \
         { "distance1", NULL, TTALINK_TYPE_UINT32_T, 0, 22, offsetof(ttalink_radar_sensors_data_t, distance1) }, \
         { "SNR1", NULL, TTALINK_TYPE_UINT32_T, 0, 26, offsetof(ttalink_radar_sensors_data_t, SNR1) }, \
         { "distance2", NULL, TTALINK_TYPE_UINT32_T, 0, 30, offsetof(ttalink_radar_sensors_data_t, distance2) }, \
         { "SNR2", NULL, TTALINK_TYPE_UINT32_T, 0, 34, offsetof(ttalink_radar_sensors_data_t, SNR2) }, \
         { "distance3", NULL, TTALINK_TYPE_UINT32_T, 0, 38, offsetof(ttalink_radar_sensors_data_t, distance3) }, \
         { "SNR3", NULL, TTALINK_TYPE_UINT32_T, 0, 42, offsetof(ttalink_radar_sensors_data_t, SNR3) }, \
         { "distance4", NULL, TTALINK_TYPE_UINT32_T, 0, 46, offsetof(ttalink_radar_sensors_data_t, distance4) }, \
         { "SNR4", NULL, TTALINK_TYPE_UINT32_T, 0, 50, offsetof(ttalink_radar_sensors_data_t, SNR4) }, \
         { "distance5", NULL, TTALINK_TYPE_UINT32_T, 0, 54, offsetof(ttalink_radar_sensors_data_t, distance5) }, \
         { "SNR5", NULL, TTALINK_TYPE_UINT32_T, 0, 58, offsetof(ttalink_radar_sensors_data_t, SNR5) }, \
         } \
}
#endif


static inline uint16_t _ttalink_radar_sensors_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t type, uint8_t target_num, uint32_t update_time, float distance_E, float distance_Var, float SNR_E, float SNR_Var, uint32_t distance1, uint32_t SNR1, uint32_t distance2, uint32_t SNR2, uint32_t distance3, uint32_t SNR3, uint32_t distance4, uint32_t SNR4, uint32_t distance5, uint32_t SNR5, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN];
    _tta_put_uint8_t(buf, 0, type);
    _tta_put_uint8_t(buf, 1, target_num);
    _tta_put_uint32_t(buf, 2, update_time);
    _tta_put_float(buf, 6, distance_E);
    _tta_put_float(buf, 10, distance_Var);
    _tta_put_float(buf, 14, SNR_E);
    _tta_put_float(buf, 18, SNR_Var);
    _tta_put_uint32_t(buf, 22, distance1);
    _tta_put_uint32_t(buf, 26, SNR1);
    _tta_put_uint32_t(buf, 30, distance2);
    _tta_put_uint32_t(buf, 34, SNR2);
    _tta_put_uint32_t(buf, 38, distance3);
    _tta_put_uint32_t(buf, 42, SNR3);
    _tta_put_uint32_t(buf, 46, distance4);
    _tta_put_uint32_t(buf, 50, SNR4);
    _tta_put_uint32_t(buf, 54, distance5);
    _tta_put_uint32_t(buf, 58, SNR5);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN);
#else
    ttalink_radar_sensors_data_t packet;
    packet.type = type;
    packet.target_num = target_num;
    packet.update_time = update_time;
    packet.distance_E = distance_E;
    packet.distance_Var = distance_Var;
    packet.SNR_E = SNR_E;
    packet.SNR_Var = SNR_Var;
    packet.distance1 = distance1;
    packet.SNR1 = SNR1;
    packet.distance2 = distance2;
    packet.SNR2 = SNR2;
    packet.distance3 = distance3;
    packet.SNR3 = SNR3;
    packet.distance4 = distance4;
    packet.SNR4 = SNR4;
    packet.distance5 = distance5;
    packet.SNR5 = SNR5;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_RADAR_SENSORS_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_RADAR_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN, TTALINK_MSG_ID_RADAR_SENSORS_DATA_CRC, nocrc);
}

/**
 * @brief Pack a radar_sensors_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param type 0:NUll 1：定高雷达 2 :前雷达 3：后雷达  4：旋转雷达.
 * @param target_num This is number of target.
 * @param update_time This is the update time.
 * @param distance_E This is the distance_E.
 * @param distance_Var This is the distance_Var.
 * @param SNR_E This is the SNR_E.
 * @param SNR_Var This is the SNR_Var.
 * @param distance1 This is the distance1.
 * @param SNR1 This is the SNR1 of distance1.
 * @param distance2 This is the distance2.
 * @param SNR2 This is the SNR2 of distance2.
 * @param distance3 This is the distance3.
 * @param SNR3 This is the SNR3 of distance3.
 * @param distance4 This is the distance4.
 * @param SNR4 This is the SNR4 of distance4.
 * @param distance5 This is the distance5.
 * @param SNR5 This is the SNR5 of distance5.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_radar_sensors_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t type, uint8_t target_num, uint32_t update_time, float distance_E, float distance_Var, float SNR_E, float SNR_Var, uint32_t distance1, uint32_t SNR1, uint32_t distance2, uint32_t SNR2, uint32_t distance3, uint32_t SNR3, uint32_t distance4, uint32_t SNR4, uint32_t distance5, uint32_t SNR5)
{
    return _ttalink_radar_sensors_data_pack(dst_addr, src_addr, msg,  type, target_num, update_time, distance_E, distance_Var, SNR_E, SNR_Var, distance1, SNR1, distance2, SNR2, distance3, SNR3, distance4, SNR4, distance5, SNR5, false);
}

/**
 * @brief Pack a radar_sensors_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param type 0:NUll 1：定高雷达 2 :前雷达 3：后雷达  4：旋转雷达.
 * @param target_num This is number of target.
 * @param update_time This is the update time.
 * @param distance_E This is the distance_E.
 * @param distance_Var This is the distance_Var.
 * @param SNR_E This is the SNR_E.
 * @param SNR_Var This is the SNR_Var.
 * @param distance1 This is the distance1.
 * @param SNR1 This is the SNR1 of distance1.
 * @param distance2 This is the distance2.
 * @param SNR2 This is the SNR2 of distance2.
 * @param distance3 This is the distance3.
 * @param SNR3 This is the SNR3 of distance3.
 * @param distance4 This is the distance4.
 * @param SNR4 This is the SNR4 of distance4.
 * @param distance5 This is the distance5.
 * @param SNR5 This is the SNR5 of distance5.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_radar_sensors_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t type, uint8_t target_num, uint32_t update_time, float distance_E, float distance_Var, float SNR_E, float SNR_Var, uint32_t distance1, uint32_t SNR1, uint32_t distance2, uint32_t SNR2, uint32_t distance3, uint32_t SNR3, uint32_t distance4, uint32_t SNR4, uint32_t distance5, uint32_t SNR5)
{
    return _ttalink_radar_sensors_data_pack(dst_addr, src_addr, msg,  type, target_num, update_time, distance_E, distance_Var, SNR_E, SNR_Var, distance1, SNR1, distance2, SNR2, distance3, SNR3, distance4, SNR4, distance5, SNR5, true);
}


static inline uint16_t _ttalink_radar_sensors_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t type,uint8_t target_num,uint32_t update_time,float distance_E,float distance_Var,float SNR_E,float SNR_Var,uint32_t distance1,uint32_t SNR1,uint32_t distance2,uint32_t SNR2,uint32_t distance3,uint32_t SNR3,uint32_t distance4,uint32_t SNR4,uint32_t distance5,uint32_t SNR5, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN];
    _tta_put_uint8_t(buf, 0, type);
    _tta_put_uint8_t(buf, 1, target_num);
    _tta_put_uint32_t(buf, 2, update_time);
    _tta_put_float(buf, 6, distance_E);
    _tta_put_float(buf, 10, distance_Var);
    _tta_put_float(buf, 14, SNR_E);
    _tta_put_float(buf, 18, SNR_Var);
    _tta_put_uint32_t(buf, 22, distance1);
    _tta_put_uint32_t(buf, 26, SNR1);
    _tta_put_uint32_t(buf, 30, distance2);
    _tta_put_uint32_t(buf, 34, SNR2);
    _tta_put_uint32_t(buf, 38, distance3);
    _tta_put_uint32_t(buf, 42, SNR3);
    _tta_put_uint32_t(buf, 46, distance4);
    _tta_put_uint32_t(buf, 50, SNR4);
    _tta_put_uint32_t(buf, 54, distance5);
    _tta_put_uint32_t(buf, 58, SNR5);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN);
#else
    ttalink_radar_sensors_data_t packet;
    packet.type = type;
    packet.target_num = target_num;
    packet.update_time = update_time;
    packet.distance_E = distance_E;
    packet.distance_Var = distance_Var;
    packet.SNR_E = SNR_E;
    packet.SNR_Var = SNR_Var;
    packet.distance1 = distance1;
    packet.SNR1 = SNR1;
    packet.distance2 = distance2;
    packet.SNR2 = SNR2;
    packet.distance3 = distance3;
    packet.SNR3 = SNR3;
    packet.distance4 = distance4;
    packet.SNR4 = SNR4;
    packet.distance5 = distance5;
    packet.SNR5 = SNR5;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_RADAR_SENSORS_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_RADAR_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN, TTALINK_MSG_ID_RADAR_SENSORS_DATA_CRC, nocrc);
}

/**
 * @brief Pack a radar_sensors_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param type 0:NUll 1：定高雷达 2 :前雷达 3：后雷达  4：旋转雷达.
 * @param target_num This is number of target.
 * @param update_time This is the update time.
 * @param distance_E This is the distance_E.
 * @param distance_Var This is the distance_Var.
 * @param SNR_E This is the SNR_E.
 * @param SNR_Var This is the SNR_Var.
 * @param distance1 This is the distance1.
 * @param SNR1 This is the SNR1 of distance1.
 * @param distance2 This is the distance2.
 * @param SNR2 This is the SNR2 of distance2.
 * @param distance3 This is the distance3.
 * @param SNR3 This is the SNR3 of distance3.
 * @param distance4 This is the distance4.
 * @param SNR4 This is the SNR4 of distance4.
 * @param distance5 This is the distance5.
 * @param SNR5 This is the SNR5 of distance5.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_radar_sensors_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t type,uint8_t target_num,uint32_t update_time,float distance_E,float distance_Var,float SNR_E,float SNR_Var,uint32_t distance1,uint32_t SNR1,uint32_t distance2,uint32_t SNR2,uint32_t distance3,uint32_t SNR3,uint32_t distance4,uint32_t SNR4,uint32_t distance5,uint32_t SNR5)
{
    return _ttalink_radar_sensors_data_pack_chan(dst_addr, src_addr, chan, msg,  type, target_num, update_time, distance_E, distance_Var, SNR_E, SNR_Var, distance1, SNR1, distance2, SNR2, distance3, SNR3, distance4, SNR4, distance5, SNR5, false);
}

/**
 * @brief Pack a radar_sensors_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param type 0:NUll 1：定高雷达 2 :前雷达 3：后雷达  4：旋转雷达.
 * @param target_num This is number of target.
 * @param update_time This is the update time.
 * @param distance_E This is the distance_E.
 * @param distance_Var This is the distance_Var.
 * @param SNR_E This is the SNR_E.
 * @param SNR_Var This is the SNR_Var.
 * @param distance1 This is the distance1.
 * @param SNR1 This is the SNR1 of distance1.
 * @param distance2 This is the distance2.
 * @param SNR2 This is the SNR2 of distance2.
 * @param distance3 This is the distance3.
 * @param SNR3 This is the SNR3 of distance3.
 * @param distance4 This is the distance4.
 * @param SNR4 This is the SNR4 of distance4.
 * @param distance5 This is the distance5.
 * @param SNR5 This is the SNR5 of distance5.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_radar_sensors_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t type,uint8_t target_num,uint32_t update_time,float distance_E,float distance_Var,float SNR_E,float SNR_Var,uint32_t distance1,uint32_t SNR1,uint32_t distance2,uint32_t SNR2,uint32_t distance3,uint32_t SNR3,uint32_t distance4,uint32_t SNR4,uint32_t distance5,uint32_t SNR5)
{
    return _ttalink_radar_sensors_data_pack_chan(dst_addr, src_addr, chan, msg,  type, target_num, update_time, distance_E, distance_Var, SNR_E, SNR_Var, distance1, SNR1, distance2, SNR2, distance3, SNR3, distance4, SNR4, distance5, SNR5, true);
}


static inline uint16_t _ttalink_radar_sensors_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_radar_sensors_data_t* radar_sensors_data, bool nocrc)
{
    if(nocrc){
        return ttalink_radar_sensors_data_pack_nocrc(dst_addr, src_addr, msg, radar_sensors_data->type, radar_sensors_data->target_num, radar_sensors_data->update_time, radar_sensors_data->distance_E, radar_sensors_data->distance_Var, radar_sensors_data->SNR_E, radar_sensors_data->SNR_Var, radar_sensors_data->distance1, radar_sensors_data->SNR1, radar_sensors_data->distance2, radar_sensors_data->SNR2, radar_sensors_data->distance3, radar_sensors_data->SNR3, radar_sensors_data->distance4, radar_sensors_data->SNR4, radar_sensors_data->distance5, radar_sensors_data->SNR5);
    }else{
        return ttalink_radar_sensors_data_pack(dst_addr, src_addr, msg, radar_sensors_data->type, radar_sensors_data->target_num, radar_sensors_data->update_time, radar_sensors_data->distance_E, radar_sensors_data->distance_Var, radar_sensors_data->SNR_E, radar_sensors_data->SNR_Var, radar_sensors_data->distance1, radar_sensors_data->SNR1, radar_sensors_data->distance2, radar_sensors_data->SNR2, radar_sensors_data->distance3, radar_sensors_data->SNR3, radar_sensors_data->distance4, radar_sensors_data->SNR4, radar_sensors_data->distance5, radar_sensors_data->SNR5);
    }
    
}

/**
 * @brief Encode a radar_sensors_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param radar_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_radar_sensors_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_radar_sensors_data_t* radar_sensors_data)
{
    return _ttalink_radar_sensors_data_encode(dst_addr, src_addr, msg, radar_sensors_data, false);
}

/**
 * @brief Encode a radar_sensors_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param radar_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_radar_sensors_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_radar_sensors_data_t* radar_sensors_data)
{
    return _ttalink_radar_sensors_data_encode(dst_addr, src_addr, msg, radar_sensors_data, true);
}


static inline uint16_t _ttalink_radar_sensors_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_radar_sensors_data_t* radar_sensors_data, bool nocrc)
{
    if(nocrc){
        return ttalink_radar_sensors_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, radar_sensors_data->type, radar_sensors_data->target_num, radar_sensors_data->update_time, radar_sensors_data->distance_E, radar_sensors_data->distance_Var, radar_sensors_data->SNR_E, radar_sensors_data->SNR_Var, radar_sensors_data->distance1, radar_sensors_data->SNR1, radar_sensors_data->distance2, radar_sensors_data->SNR2, radar_sensors_data->distance3, radar_sensors_data->SNR3, radar_sensors_data->distance4, radar_sensors_data->SNR4, radar_sensors_data->distance5, radar_sensors_data->SNR5);
    }else{
        return ttalink_radar_sensors_data_pack_chan(dst_addr, src_addr, chan, msg, radar_sensors_data->type, radar_sensors_data->target_num, radar_sensors_data->update_time, radar_sensors_data->distance_E, radar_sensors_data->distance_Var, radar_sensors_data->SNR_E, radar_sensors_data->SNR_Var, radar_sensors_data->distance1, radar_sensors_data->SNR1, radar_sensors_data->distance2, radar_sensors_data->SNR2, radar_sensors_data->distance3, radar_sensors_data->SNR3, radar_sensors_data->distance4, radar_sensors_data->SNR4, radar_sensors_data->distance5, radar_sensors_data->SNR5);
    }
}

/**
 * @brief Encode a radar_sensors_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param radar_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_radar_sensors_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_radar_sensors_data_t* radar_sensors_data)
{
    return _ttalink_radar_sensors_data_encode_chan(dst_addr, src_addr, chan, msg, radar_sensors_data, false);
}

/**
 * @brief Encode a radar_sensors_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param radar_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_radar_sensors_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_radar_sensors_data_t* radar_sensors_data)
{
    return _ttalink_radar_sensors_data_encode_chan(dst_addr, src_addr, chan, msg, radar_sensors_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_radar_sensors_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t type, uint8_t target_num, uint32_t update_time, float distance_E, float distance_Var, float SNR_E, float SNR_Var, uint32_t distance1, uint32_t SNR1, uint32_t distance2, uint32_t SNR2, uint32_t distance3, uint32_t SNR3, uint32_t distance4, uint32_t SNR4, uint32_t distance5, uint32_t SNR5, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN];
    _tta_put_uint8_t(buf, 0, type);
    _tta_put_uint8_t(buf, 1, target_num);
    _tta_put_uint32_t(buf, 2, update_time);
    _tta_put_float(buf, 6, distance_E);
    _tta_put_float(buf, 10, distance_Var);
    _tta_put_float(buf, 14, SNR_E);
    _tta_put_float(buf, 18, SNR_Var);
    _tta_put_uint32_t(buf, 22, distance1);
    _tta_put_uint32_t(buf, 26, SNR1);
    _tta_put_uint32_t(buf, 30, distance2);
    _tta_put_uint32_t(buf, 34, SNR2);
    _tta_put_uint32_t(buf, 38, distance3);
    _tta_put_uint32_t(buf, 42, SNR3);
    _tta_put_uint32_t(buf, 46, distance4);
    _tta_put_uint32_t(buf, 50, SNR4);
    _tta_put_uint32_t(buf, 54, distance5);
    _tta_put_uint32_t(buf, 58, SNR5);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RADAR_SENSORS_DATA, buf, TTALINK_MSG_ID_RADAR_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN, TTALINK_MSG_ID_RADAR_SENSORS_DATA_CRC, nocrc);
#else
    ttalink_radar_sensors_data_t packet;
    packet.type = type;
    packet.target_num = target_num;
    packet.update_time = update_time;
    packet.distance_E = distance_E;
    packet.distance_Var = distance_Var;
    packet.SNR_E = SNR_E;
    packet.SNR_Var = SNR_Var;
    packet.distance1 = distance1;
    packet.SNR1 = SNR1;
    packet.distance2 = distance2;
    packet.SNR2 = SNR2;
    packet.distance3 = distance3;
    packet.SNR3 = SNR3;
    packet.distance4 = distance4;
    packet.SNR4 = SNR4;
    packet.distance5 = distance5;
    packet.SNR5 = SNR5;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RADAR_SENSORS_DATA, (const char *)&packet, TTALINK_MSG_ID_RADAR_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN, TTALINK_MSG_ID_RADAR_SENSORS_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a radar_sensors_data message
 * @param chan TTAlink channel to send the message
 *
 * @param type 0:NUll 1：定高雷达 2 :前雷达 3：后雷达  4：旋转雷达.
 * @param target_num This is number of target.
 * @param update_time This is the update time.
 * @param distance_E This is the distance_E.
 * @param distance_Var This is the distance_Var.
 * @param SNR_E This is the SNR_E.
 * @param SNR_Var This is the SNR_Var.
 * @param distance1 This is the distance1.
 * @param SNR1 This is the SNR1 of distance1.
 * @param distance2 This is the distance2.
 * @param SNR2 This is the SNR2 of distance2.
 * @param distance3 This is the distance3.
 * @param SNR3 This is the SNR3 of distance3.
 * @param distance4 This is the distance4.
 * @param SNR4 This is the SNR4 of distance4.
 * @param distance5 This is the distance5.
 * @param SNR5 This is the SNR5 of distance5.
 */
static inline void ttalink_radar_sensors_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t type, uint8_t target_num, uint32_t update_time, float distance_E, float distance_Var, float SNR_E, float SNR_Var, uint32_t distance1, uint32_t SNR1, uint32_t distance2, uint32_t SNR2, uint32_t distance3, uint32_t SNR3, uint32_t distance4, uint32_t SNR4, uint32_t distance5, uint32_t SNR5)
{
    _ttalink_radar_sensors_data_send(dst_addr, src_addr, chan, type, target_num, update_time, distance_E, distance_Var, SNR_E, SNR_Var, distance1, SNR1, distance2, SNR2, distance3, SNR3, distance4, SNR4, distance5, SNR5, false);
}

/**
 * @brief Send a radar_sensors_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param type 0:NUll 1：定高雷达 2 :前雷达 3：后雷达  4：旋转雷达.
 * @param target_num This is number of target.
 * @param update_time This is the update time.
 * @param distance_E This is the distance_E.
 * @param distance_Var This is the distance_Var.
 * @param SNR_E This is the SNR_E.
 * @param SNR_Var This is the SNR_Var.
 * @param distance1 This is the distance1.
 * @param SNR1 This is the SNR1 of distance1.
 * @param distance2 This is the distance2.
 * @param SNR2 This is the SNR2 of distance2.
 * @param distance3 This is the distance3.
 * @param SNR3 This is the SNR3 of distance3.
 * @param distance4 This is the distance4.
 * @param SNR4 This is the SNR4 of distance4.
 * @param distance5 This is the distance5.
 * @param SNR5 This is the SNR5 of distance5.
 */
static inline void ttalink_radar_sensors_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t type, uint8_t target_num, uint32_t update_time, float distance_E, float distance_Var, float SNR_E, float SNR_Var, uint32_t distance1, uint32_t SNR1, uint32_t distance2, uint32_t SNR2, uint32_t distance3, uint32_t SNR3, uint32_t distance4, uint32_t SNR4, uint32_t distance5, uint32_t SNR5)
{
    _ttalink_radar_sensors_data_send(dst_addr, src_addr, chan, type, target_num, update_time, distance_E, distance_Var, SNR_E, SNR_Var, distance1, SNR1, distance2, SNR2, distance3, SNR3, distance4, SNR4, distance5, SNR5, true);
}


static inline void _ttalink_radar_sensors_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_radar_sensors_data_t* radar_sensors_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_radar_sensors_data_send_nocrc(dst_addr, src_addr, chan, radar_sensors_data->type, radar_sensors_data->target_num, radar_sensors_data->update_time, radar_sensors_data->distance_E, radar_sensors_data->distance_Var, radar_sensors_data->SNR_E, radar_sensors_data->SNR_Var, radar_sensors_data->distance1, radar_sensors_data->SNR1, radar_sensors_data->distance2, radar_sensors_data->SNR2, radar_sensors_data->distance3, radar_sensors_data->SNR3, radar_sensors_data->distance4, radar_sensors_data->SNR4, radar_sensors_data->distance5, radar_sensors_data->SNR5);
    }else{
        ttalink_radar_sensors_data_send(dst_addr, src_addr, chan, radar_sensors_data->type, radar_sensors_data->target_num, radar_sensors_data->update_time, radar_sensors_data->distance_E, radar_sensors_data->distance_Var, radar_sensors_data->SNR_E, radar_sensors_data->SNR_Var, radar_sensors_data->distance1, radar_sensors_data->SNR1, radar_sensors_data->distance2, radar_sensors_data->SNR2, radar_sensors_data->distance3, radar_sensors_data->SNR3, radar_sensors_data->distance4, radar_sensors_data->SNR4, radar_sensors_data->distance5, radar_sensors_data->SNR5);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RADAR_SENSORS_DATA, (const char *)radar_sensors_data, TTALINK_MSG_ID_RADAR_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN, TTALINK_MSG_ID_RADAR_SENSORS_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a radar_sensors_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_radar_sensors_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_radar_sensors_data_t* radar_sensors_data)
{
    _ttalink_radar_sensors_data_send_struct(dst_addr, src_addr, chan, radar_sensors_data, false);
}

/**
 * @brief Send a radar_sensors_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_radar_sensors_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_radar_sensors_data_t* radar_sensors_data)
{
    _ttalink_radar_sensors_data_send_struct(dst_addr, src_addr, chan, radar_sensors_data, true);
}

#if TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_radar_sensors_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t type, uint8_t target_num, uint32_t update_time, float distance_E, float distance_Var, float SNR_E, float SNR_Var, uint32_t distance1, uint32_t SNR1, uint32_t distance2, uint32_t SNR2, uint32_t distance3, uint32_t SNR3, uint32_t distance4, uint32_t SNR4, uint32_t distance5, uint32_t SNR5, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, type);
    _tta_put_uint8_t(buf, 1, target_num);
    _tta_put_uint32_t(buf, 2, update_time);
    _tta_put_float(buf, 6, distance_E);
    _tta_put_float(buf, 10, distance_Var);
    _tta_put_float(buf, 14, SNR_E);
    _tta_put_float(buf, 18, SNR_Var);
    _tta_put_uint32_t(buf, 22, distance1);
    _tta_put_uint32_t(buf, 26, SNR1);
    _tta_put_uint32_t(buf, 30, distance2);
    _tta_put_uint32_t(buf, 34, SNR2);
    _tta_put_uint32_t(buf, 38, distance3);
    _tta_put_uint32_t(buf, 42, SNR3);
    _tta_put_uint32_t(buf, 46, distance4);
    _tta_put_uint32_t(buf, 50, SNR4);
    _tta_put_uint32_t(buf, 54, distance5);
    _tta_put_uint32_t(buf, 58, SNR5);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RADAR_SENSORS_DATA, buf, TTALINK_MSG_ID_RADAR_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN, TTALINK_MSG_ID_RADAR_SENSORS_DATA_CRC, nocrc);
#else
    ttalink_radar_sensors_data_t *packet = (ttalink_radar_sensors_data_t *)msgbuf;
    packet->type = type;
    packet->target_num = target_num;
    packet->update_time = update_time;
    packet->distance_E = distance_E;
    packet->distance_Var = distance_Var;
    packet->SNR_E = SNR_E;
    packet->SNR_Var = SNR_Var;
    packet->distance1 = distance1;
    packet->SNR1 = SNR1;
    packet->distance2 = distance2;
    packet->SNR2 = SNR2;
    packet->distance3 = distance3;
    packet->SNR3 = SNR3;
    packet->distance4 = distance4;
    packet->SNR4 = SNR4;
    packet->distance5 = distance5;
    packet->SNR5 = SNR5;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RADAR_SENSORS_DATA, (const char *)packet, TTALINK_MSG_ID_RADAR_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN, TTALINK_MSG_ID_RADAR_SENSORS_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_radar_sensors_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t type, uint8_t target_num, uint32_t update_time, float distance_E, float distance_Var, float SNR_E, float SNR_Var, uint32_t distance1, uint32_t SNR1, uint32_t distance2, uint32_t SNR2, uint32_t distance3, uint32_t SNR3, uint32_t distance4, uint32_t SNR4, uint32_t distance5, uint32_t SNR5)
{
    _ttalink_radar_sensors_data_send_buf(dst_addr, src_addr, msgbuf, chan, type, target_num, update_time, distance_E, distance_Var, SNR_E, SNR_Var, distance1, SNR1, distance2, SNR2, distance3, SNR3, distance4, SNR4, distance5, SNR5, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_radar_sensors_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t type, uint8_t target_num, uint32_t update_time, float distance_E, float distance_Var, float SNR_E, float SNR_Var, uint32_t distance1, uint32_t SNR1, uint32_t distance2, uint32_t SNR2, uint32_t distance3, uint32_t SNR3, uint32_t distance4, uint32_t SNR4, uint32_t distance5, uint32_t SNR5)
{
    _ttalink_radar_sensors_data_send_buf(dst_addr, src_addr, msgbuf, chan, type, target_num, update_time, distance_E, distance_Var, SNR_E, SNR_Var, distance1, SNR1, distance2, SNR2, distance3, SNR3, distance4, SNR4, distance5, SNR5, true);
}
#endif

#endif

// MESSAGE RADAR_SENSORS_DATA UNPACKING


/**
 * @brief Get field type from radar_sensors_data message
 *
 * @return 0:NUll 1：定高雷达 2 :前雷达 3：后雷达  4：旋转雷达.
 */
static inline uint8_t ttalink_radar_sensors_data_get_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_num from radar_sensors_data message
 *
 * @return This is number of target.
 */
static inline uint8_t ttalink_radar_sensors_data_get_target_num(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field update_time from radar_sensors_data message
 *
 * @return This is the update time.
 */
static inline uint32_t ttalink_radar_sensors_data_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  2);
}

/**
 * @brief Get field distance_E from radar_sensors_data message
 *
 * @return This is the distance_E.
 */
static inline float ttalink_radar_sensors_data_get_distance_E(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  6);
}

/**
 * @brief Get field distance_Var from radar_sensors_data message
 *
 * @return This is the distance_Var.
 */
static inline float ttalink_radar_sensors_data_get_distance_Var(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  10);
}

/**
 * @brief Get field SNR_E from radar_sensors_data message
 *
 * @return This is the SNR_E.
 */
static inline float ttalink_radar_sensors_data_get_SNR_E(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  14);
}

/**
 * @brief Get field SNR_Var from radar_sensors_data message
 *
 * @return This is the SNR_Var.
 */
static inline float ttalink_radar_sensors_data_get_SNR_Var(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  18);
}

/**
 * @brief Get field distance1 from radar_sensors_data message
 *
 * @return This is the distance1.
 */
static inline uint32_t ttalink_radar_sensors_data_get_distance1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  22);
}

/**
 * @brief Get field SNR1 from radar_sensors_data message
 *
 * @return This is the SNR1 of distance1.
 */
static inline uint32_t ttalink_radar_sensors_data_get_SNR1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  26);
}

/**
 * @brief Get field distance2 from radar_sensors_data message
 *
 * @return This is the distance2.
 */
static inline uint32_t ttalink_radar_sensors_data_get_distance2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  30);
}

/**
 * @brief Get field SNR2 from radar_sensors_data message
 *
 * @return This is the SNR2 of distance2.
 */
static inline uint32_t ttalink_radar_sensors_data_get_SNR2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  34);
}

/**
 * @brief Get field distance3 from radar_sensors_data message
 *
 * @return This is the distance3.
 */
static inline uint32_t ttalink_radar_sensors_data_get_distance3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  38);
}

/**
 * @brief Get field SNR3 from radar_sensors_data message
 *
 * @return This is the SNR3 of distance3.
 */
static inline uint32_t ttalink_radar_sensors_data_get_SNR3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  42);
}

/**
 * @brief Get field distance4 from radar_sensors_data message
 *
 * @return This is the distance4.
 */
static inline uint32_t ttalink_radar_sensors_data_get_distance4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  46);
}

/**
 * @brief Get field SNR4 from radar_sensors_data message
 *
 * @return This is the SNR4 of distance4.
 */
static inline uint32_t ttalink_radar_sensors_data_get_SNR4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  50);
}

/**
 * @brief Get field distance5 from radar_sensors_data message
 *
 * @return This is the distance5.
 */
static inline uint32_t ttalink_radar_sensors_data_get_distance5(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  54);
}

/**
 * @brief Get field SNR5 from radar_sensors_data message
 *
 * @return This is the SNR5 of distance5.
 */
static inline uint32_t ttalink_radar_sensors_data_get_SNR5(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  58);
}

/**
 * @brief Decode a radar_sensors_data message into a struct
 *
 * @param msg The message to decode
 * @param radar_sensors_data C-struct to decode the message contents into
 */
static inline void ttalink_radar_sensors_data_decode(const ttalink_message_t* msg, ttalink_radar_sensors_data_t* radar_sensors_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    radar_sensors_data->type = ttalink_radar_sensors_data_get_type(msg);
    radar_sensors_data->target_num = ttalink_radar_sensors_data_get_target_num(msg);
    radar_sensors_data->update_time = ttalink_radar_sensors_data_get_update_time(msg);
    radar_sensors_data->distance_E = ttalink_radar_sensors_data_get_distance_E(msg);
    radar_sensors_data->distance_Var = ttalink_radar_sensors_data_get_distance_Var(msg);
    radar_sensors_data->SNR_E = ttalink_radar_sensors_data_get_SNR_E(msg);
    radar_sensors_data->SNR_Var = ttalink_radar_sensors_data_get_SNR_Var(msg);
    radar_sensors_data->distance1 = ttalink_radar_sensors_data_get_distance1(msg);
    radar_sensors_data->SNR1 = ttalink_radar_sensors_data_get_SNR1(msg);
    radar_sensors_data->distance2 = ttalink_radar_sensors_data_get_distance2(msg);
    radar_sensors_data->SNR2 = ttalink_radar_sensors_data_get_SNR2(msg);
    radar_sensors_data->distance3 = ttalink_radar_sensors_data_get_distance3(msg);
    radar_sensors_data->SNR3 = ttalink_radar_sensors_data_get_SNR3(msg);
    radar_sensors_data->distance4 = ttalink_radar_sensors_data_get_distance4(msg);
    radar_sensors_data->SNR4 = ttalink_radar_sensors_data_get_SNR4(msg);
    radar_sensors_data->distance5 = ttalink_radar_sensors_data_get_distance5(msg);
    radar_sensors_data->SNR5 = ttalink_radar_sensors_data_get_SNR5(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN? msg->len : TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN;
        memset(radar_sensors_data, 0, TTALINK_MSG_ID_RADAR_SENSORS_DATA_LEN);
    memcpy(radar_sensors_data, _TTA_PAYLOAD(msg), len);
#endif
}
