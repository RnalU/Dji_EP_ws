#pragma once
// MESSAGE EXAM_STATE PACKING

#define TTALINK_MSG_ID_EXAM_STATE 190

TTAPACKED(
typedef struct __ttalink_exam_state_t {
 uint64_t time; /*<  utc_time(MS)*/
 uint64_t uav_id; /*<  uav_id.*/
 int32_t longi; /*<  [-180,180] : longi*10000000.*/
 int32_t latit; /*<  [-90,90] : latit*10000000.*/
 int32_t altitude; /*<  altit*1000(M).*/
 int32_t height; /*<  Relative altitude*1000(M).*/
 uint16_t voltage; /*<  Battery voltage*100(V).*/
 int16_t angularVelocity; /*<  angularVelocity*10.*/
 int16_t guoundspeed; /*<  guoundspeed*10.*/
 int16_t heading; /*<  [0,360]*/
 int8_t yaw_min; /*<  [-120,120].*/
 int8_t yaw_max; /*<  [-120,120].*/
 int8_t pitch_min; /*<  [-45,45].*/
 int8_t pitch_max; /*<  [-45,45].*/
 int8_t roll_min; /*<  [-45,45].*/
 int8_t roll_max; /*<  [-45,45].*/
 uint8_t flight_mode; /*<  flight_mode.*/
 uint8_t state; /*<  state : 0-lock 1-unlock.*/
 uint8_t gps_stat_num; /*<  star.*/
 uint8_t fix_type; /*<  .*/
}) ttalink_exam_state_t;

#define TTALINK_MSG_ID_EXAM_STATE_LEN 50
#define TTALINK_MSG_ID_EXAM_STATE_MIN_LEN 50
#define TTALINK_MSG_ID_190_LEN 50
#define TTALINK_MSG_ID_190_MIN_LEN 50

#define TTALINK_MSG_ID_EXAM_STATE_CRC 155
#define TTALINK_MSG_ID_190_CRC 155



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_EXAM_STATE { \
    190, \
    "EXAM_STATE", \
    20, \
    {  { "time", NULL, TTALINK_TYPE_UINT64_T, 0, 0, offsetof(ttalink_exam_state_t, time) }, \
         { "uav_id", NULL, TTALINK_TYPE_UINT64_T, 0, 8, offsetof(ttalink_exam_state_t, uav_id) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 16, offsetof(ttalink_exam_state_t, longi) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 20, offsetof(ttalink_exam_state_t, latit) }, \
         { "altitude", NULL, TTALINK_TYPE_INT32_T, 0, 24, offsetof(ttalink_exam_state_t, altitude) }, \
         { "height", NULL, TTALINK_TYPE_INT32_T, 0, 28, offsetof(ttalink_exam_state_t, height) }, \
         { "voltage", NULL, TTALINK_TYPE_UINT16_T, 0, 32, offsetof(ttalink_exam_state_t, voltage) }, \
         { "angularVelocity", NULL, TTALINK_TYPE_INT16_T, 0, 34, offsetof(ttalink_exam_state_t, angularVelocity) }, \
         { "guoundspeed", NULL, TTALINK_TYPE_INT16_T, 0, 36, offsetof(ttalink_exam_state_t, guoundspeed) }, \
         { "heading", NULL, TTALINK_TYPE_INT16_T, 0, 38, offsetof(ttalink_exam_state_t, heading) }, \
         { "yaw_min", NULL, TTALINK_TYPE_INT8_T, 0, 40, offsetof(ttalink_exam_state_t, yaw_min) }, \
         { "yaw_max", NULL, TTALINK_TYPE_INT8_T, 0, 41, offsetof(ttalink_exam_state_t, yaw_max) }, \
         { "pitch_min", NULL, TTALINK_TYPE_INT8_T, 0, 42, offsetof(ttalink_exam_state_t, pitch_min) }, \
         { "pitch_max", NULL, TTALINK_TYPE_INT8_T, 0, 43, offsetof(ttalink_exam_state_t, pitch_max) }, \
         { "roll_min", NULL, TTALINK_TYPE_INT8_T, 0, 44, offsetof(ttalink_exam_state_t, roll_min) }, \
         { "roll_max", NULL, TTALINK_TYPE_INT8_T, 0, 45, offsetof(ttalink_exam_state_t, roll_max) }, \
         { "flight_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 46, offsetof(ttalink_exam_state_t, flight_mode) }, \
         { "state", NULL, TTALINK_TYPE_UINT8_T, 0, 47, offsetof(ttalink_exam_state_t, state) }, \
         { "gps_stat_num", NULL, TTALINK_TYPE_UINT8_T, 0, 48, offsetof(ttalink_exam_state_t, gps_stat_num) }, \
         { "fix_type", NULL, TTALINK_TYPE_UINT8_T, 0, 49, offsetof(ttalink_exam_state_t, fix_type) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_EXAM_STATE { \
    "EXAM_STATE", \
    20, \
    {  { "time", NULL, TTALINK_TYPE_UINT64_T, 0, 0, offsetof(ttalink_exam_state_t, time) }, \
         { "uav_id", NULL, TTALINK_TYPE_UINT64_T, 0, 8, offsetof(ttalink_exam_state_t, uav_id) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 16, offsetof(ttalink_exam_state_t, longi) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 20, offsetof(ttalink_exam_state_t, latit) }, \
         { "altitude", NULL, TTALINK_TYPE_INT32_T, 0, 24, offsetof(ttalink_exam_state_t, altitude) }, \
         { "height", NULL, TTALINK_TYPE_INT32_T, 0, 28, offsetof(ttalink_exam_state_t, height) }, \
         { "voltage", NULL, TTALINK_TYPE_UINT16_T, 0, 32, offsetof(ttalink_exam_state_t, voltage) }, \
         { "angularVelocity", NULL, TTALINK_TYPE_INT16_T, 0, 34, offsetof(ttalink_exam_state_t, angularVelocity) }, \
         { "guoundspeed", NULL, TTALINK_TYPE_INT16_T, 0, 36, offsetof(ttalink_exam_state_t, guoundspeed) }, \
         { "heading", NULL, TTALINK_TYPE_INT16_T, 0, 38, offsetof(ttalink_exam_state_t, heading) }, \
         { "yaw_min", NULL, TTALINK_TYPE_INT8_T, 0, 40, offsetof(ttalink_exam_state_t, yaw_min) }, \
         { "yaw_max", NULL, TTALINK_TYPE_INT8_T, 0, 41, offsetof(ttalink_exam_state_t, yaw_max) }, \
         { "pitch_min", NULL, TTALINK_TYPE_INT8_T, 0, 42, offsetof(ttalink_exam_state_t, pitch_min) }, \
         { "pitch_max", NULL, TTALINK_TYPE_INT8_T, 0, 43, offsetof(ttalink_exam_state_t, pitch_max) }, \
         { "roll_min", NULL, TTALINK_TYPE_INT8_T, 0, 44, offsetof(ttalink_exam_state_t, roll_min) }, \
         { "roll_max", NULL, TTALINK_TYPE_INT8_T, 0, 45, offsetof(ttalink_exam_state_t, roll_max) }, \
         { "flight_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 46, offsetof(ttalink_exam_state_t, flight_mode) }, \
         { "state", NULL, TTALINK_TYPE_UINT8_T, 0, 47, offsetof(ttalink_exam_state_t, state) }, \
         { "gps_stat_num", NULL, TTALINK_TYPE_UINT8_T, 0, 48, offsetof(ttalink_exam_state_t, gps_stat_num) }, \
         { "fix_type", NULL, TTALINK_TYPE_UINT8_T, 0, 49, offsetof(ttalink_exam_state_t, fix_type) }, \
         } \
}
#endif


static inline uint16_t _ttalink_exam_state_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time, uint64_t uav_id, int32_t longi, int32_t latit, int32_t altitude, int32_t height, uint16_t voltage, int16_t angularVelocity, int16_t guoundspeed, int16_t heading, int8_t yaw_min, int8_t yaw_max, int8_t pitch_min, int8_t pitch_max, int8_t roll_min, int8_t roll_max, uint8_t flight_mode, uint8_t state, uint8_t gps_stat_num, uint8_t fix_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_EXAM_STATE_LEN];
    _tta_put_uint64_t(buf, 0, time);
    _tta_put_uint64_t(buf, 8, uav_id);
    _tta_put_int32_t(buf, 16, longi);
    _tta_put_int32_t(buf, 20, latit);
    _tta_put_int32_t(buf, 24, altitude);
    _tta_put_int32_t(buf, 28, height);
    _tta_put_uint16_t(buf, 32, voltage);
    _tta_put_int16_t(buf, 34, angularVelocity);
    _tta_put_int16_t(buf, 36, guoundspeed);
    _tta_put_int16_t(buf, 38, heading);
    _tta_put_int8_t(buf, 40, yaw_min);
    _tta_put_int8_t(buf, 41, yaw_max);
    _tta_put_int8_t(buf, 42, pitch_min);
    _tta_put_int8_t(buf, 43, pitch_max);
    _tta_put_int8_t(buf, 44, roll_min);
    _tta_put_int8_t(buf, 45, roll_max);
    _tta_put_uint8_t(buf, 46, flight_mode);
    _tta_put_uint8_t(buf, 47, state);
    _tta_put_uint8_t(buf, 48, gps_stat_num);
    _tta_put_uint8_t(buf, 49, fix_type);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_EXAM_STATE_LEN);
#else
    ttalink_exam_state_t packet;
    packet.time = time;
    packet.uav_id = uav_id;
    packet.longi = longi;
    packet.latit = latit;
    packet.altitude = altitude;
    packet.height = height;
    packet.voltage = voltage;
    packet.angularVelocity = angularVelocity;
    packet.guoundspeed = guoundspeed;
    packet.heading = heading;
    packet.yaw_min = yaw_min;
    packet.yaw_max = yaw_max;
    packet.pitch_min = pitch_min;
    packet.pitch_max = pitch_max;
    packet.roll_min = roll_min;
    packet.roll_max = roll_max;
    packet.flight_mode = flight_mode;
    packet.state = state;
    packet.gps_stat_num = gps_stat_num;
    packet.fix_type = fix_type;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_EXAM_STATE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_EXAM_STATE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_EXAM_STATE_MIN_LEN, TTALINK_MSG_ID_EXAM_STATE_LEN, TTALINK_MSG_ID_EXAM_STATE_CRC, nocrc);
}

/**
 * @brief Pack a exam_state message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param time  utc_time(MS)
 * @param uav_id  uav_id.
 * @param longi  [-180,180] : longi*10000000.
 * @param latit  [-90,90] : latit*10000000.
 * @param altitude  altit*1000(M).
 * @param height  Relative altitude*1000(M).
 * @param voltage  Battery voltage*100(V).
 * @param angularVelocity  angularVelocity*10.
 * @param guoundspeed  guoundspeed*10.
 * @param heading  [0,360]
 * @param yaw_min  [-120,120].
 * @param yaw_max  [-120,120].
 * @param pitch_min  [-45,45].
 * @param pitch_max  [-45,45].
 * @param roll_min  [-45,45].
 * @param roll_max  [-45,45].
 * @param flight_mode  flight_mode.
 * @param state  state : 0-lock 1-unlock.
 * @param gps_stat_num  star.
 * @param fix_type  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_exam_state_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time, uint64_t uav_id, int32_t longi, int32_t latit, int32_t altitude, int32_t height, uint16_t voltage, int16_t angularVelocity, int16_t guoundspeed, int16_t heading, int8_t yaw_min, int8_t yaw_max, int8_t pitch_min, int8_t pitch_max, int8_t roll_min, int8_t roll_max, uint8_t flight_mode, uint8_t state, uint8_t gps_stat_num, uint8_t fix_type)
{
    return _ttalink_exam_state_pack(dst_addr, src_addr, msg,  time, uav_id, longi, latit, altitude, height, voltage, angularVelocity, guoundspeed, heading, yaw_min, yaw_max, pitch_min, pitch_max, roll_min, roll_max, flight_mode, state, gps_stat_num, fix_type, false);
}

/**
 * @brief Pack a exam_state message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param time  utc_time(MS)
 * @param uav_id  uav_id.
 * @param longi  [-180,180] : longi*10000000.
 * @param latit  [-90,90] : latit*10000000.
 * @param altitude  altit*1000(M).
 * @param height  Relative altitude*1000(M).
 * @param voltage  Battery voltage*100(V).
 * @param angularVelocity  angularVelocity*10.
 * @param guoundspeed  guoundspeed*10.
 * @param heading  [0,360]
 * @param yaw_min  [-120,120].
 * @param yaw_max  [-120,120].
 * @param pitch_min  [-45,45].
 * @param pitch_max  [-45,45].
 * @param roll_min  [-45,45].
 * @param roll_max  [-45,45].
 * @param flight_mode  flight_mode.
 * @param state  state : 0-lock 1-unlock.
 * @param gps_stat_num  star.
 * @param fix_type  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_exam_state_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time, uint64_t uav_id, int32_t longi, int32_t latit, int32_t altitude, int32_t height, uint16_t voltage, int16_t angularVelocity, int16_t guoundspeed, int16_t heading, int8_t yaw_min, int8_t yaw_max, int8_t pitch_min, int8_t pitch_max, int8_t roll_min, int8_t roll_max, uint8_t flight_mode, uint8_t state, uint8_t gps_stat_num, uint8_t fix_type)
{
    return _ttalink_exam_state_pack(dst_addr, src_addr, msg,  time, uav_id, longi, latit, altitude, height, voltage, angularVelocity, guoundspeed, heading, yaw_min, yaw_max, pitch_min, pitch_max, roll_min, roll_max, flight_mode, state, gps_stat_num, fix_type, true);
}


static inline uint16_t _ttalink_exam_state_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time,uint64_t uav_id,int32_t longi,int32_t latit,int32_t altitude,int32_t height,uint16_t voltage,int16_t angularVelocity,int16_t guoundspeed,int16_t heading,int8_t yaw_min,int8_t yaw_max,int8_t pitch_min,int8_t pitch_max,int8_t roll_min,int8_t roll_max,uint8_t flight_mode,uint8_t state,uint8_t gps_stat_num,uint8_t fix_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_EXAM_STATE_LEN];
    _tta_put_uint64_t(buf, 0, time);
    _tta_put_uint64_t(buf, 8, uav_id);
    _tta_put_int32_t(buf, 16, longi);
    _tta_put_int32_t(buf, 20, latit);
    _tta_put_int32_t(buf, 24, altitude);
    _tta_put_int32_t(buf, 28, height);
    _tta_put_uint16_t(buf, 32, voltage);
    _tta_put_int16_t(buf, 34, angularVelocity);
    _tta_put_int16_t(buf, 36, guoundspeed);
    _tta_put_int16_t(buf, 38, heading);
    _tta_put_int8_t(buf, 40, yaw_min);
    _tta_put_int8_t(buf, 41, yaw_max);
    _tta_put_int8_t(buf, 42, pitch_min);
    _tta_put_int8_t(buf, 43, pitch_max);
    _tta_put_int8_t(buf, 44, roll_min);
    _tta_put_int8_t(buf, 45, roll_max);
    _tta_put_uint8_t(buf, 46, flight_mode);
    _tta_put_uint8_t(buf, 47, state);
    _tta_put_uint8_t(buf, 48, gps_stat_num);
    _tta_put_uint8_t(buf, 49, fix_type);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_EXAM_STATE_LEN);
#else
    ttalink_exam_state_t packet;
    packet.time = time;
    packet.uav_id = uav_id;
    packet.longi = longi;
    packet.latit = latit;
    packet.altitude = altitude;
    packet.height = height;
    packet.voltage = voltage;
    packet.angularVelocity = angularVelocity;
    packet.guoundspeed = guoundspeed;
    packet.heading = heading;
    packet.yaw_min = yaw_min;
    packet.yaw_max = yaw_max;
    packet.pitch_min = pitch_min;
    packet.pitch_max = pitch_max;
    packet.roll_min = roll_min;
    packet.roll_max = roll_max;
    packet.flight_mode = flight_mode;
    packet.state = state;
    packet.gps_stat_num = gps_stat_num;
    packet.fix_type = fix_type;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_EXAM_STATE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_EXAM_STATE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_EXAM_STATE_MIN_LEN, TTALINK_MSG_ID_EXAM_STATE_LEN, TTALINK_MSG_ID_EXAM_STATE_CRC, nocrc);
}

/**
 * @brief Pack a exam_state message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param time  utc_time(MS)
 * @param uav_id  uav_id.
 * @param longi  [-180,180] : longi*10000000.
 * @param latit  [-90,90] : latit*10000000.
 * @param altitude  altit*1000(M).
 * @param height  Relative altitude*1000(M).
 * @param voltage  Battery voltage*100(V).
 * @param angularVelocity  angularVelocity*10.
 * @param guoundspeed  guoundspeed*10.
 * @param heading  [0,360]
 * @param yaw_min  [-120,120].
 * @param yaw_max  [-120,120].
 * @param pitch_min  [-45,45].
 * @param pitch_max  [-45,45].
 * @param roll_min  [-45,45].
 * @param roll_max  [-45,45].
 * @param flight_mode  flight_mode.
 * @param state  state : 0-lock 1-unlock.
 * @param gps_stat_num  star.
 * @param fix_type  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_exam_state_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time,uint64_t uav_id,int32_t longi,int32_t latit,int32_t altitude,int32_t height,uint16_t voltage,int16_t angularVelocity,int16_t guoundspeed,int16_t heading,int8_t yaw_min,int8_t yaw_max,int8_t pitch_min,int8_t pitch_max,int8_t roll_min,int8_t roll_max,uint8_t flight_mode,uint8_t state,uint8_t gps_stat_num,uint8_t fix_type)
{
    return _ttalink_exam_state_pack_chan(dst_addr, src_addr, chan, msg,  time, uav_id, longi, latit, altitude, height, voltage, angularVelocity, guoundspeed, heading, yaw_min, yaw_max, pitch_min, pitch_max, roll_min, roll_max, flight_mode, state, gps_stat_num, fix_type, false);
}

/**
 * @brief Pack a exam_state message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param time  utc_time(MS)
 * @param uav_id  uav_id.
 * @param longi  [-180,180] : longi*10000000.
 * @param latit  [-90,90] : latit*10000000.
 * @param altitude  altit*1000(M).
 * @param height  Relative altitude*1000(M).
 * @param voltage  Battery voltage*100(V).
 * @param angularVelocity  angularVelocity*10.
 * @param guoundspeed  guoundspeed*10.
 * @param heading  [0,360]
 * @param yaw_min  [-120,120].
 * @param yaw_max  [-120,120].
 * @param pitch_min  [-45,45].
 * @param pitch_max  [-45,45].
 * @param roll_min  [-45,45].
 * @param roll_max  [-45,45].
 * @param flight_mode  flight_mode.
 * @param state  state : 0-lock 1-unlock.
 * @param gps_stat_num  star.
 * @param fix_type  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_exam_state_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time,uint64_t uav_id,int32_t longi,int32_t latit,int32_t altitude,int32_t height,uint16_t voltage,int16_t angularVelocity,int16_t guoundspeed,int16_t heading,int8_t yaw_min,int8_t yaw_max,int8_t pitch_min,int8_t pitch_max,int8_t roll_min,int8_t roll_max,uint8_t flight_mode,uint8_t state,uint8_t gps_stat_num,uint8_t fix_type)
{
    return _ttalink_exam_state_pack_chan(dst_addr, src_addr, chan, msg,  time, uav_id, longi, latit, altitude, height, voltage, angularVelocity, guoundspeed, heading, yaw_min, yaw_max, pitch_min, pitch_max, roll_min, roll_max, flight_mode, state, gps_stat_num, fix_type, true);
}


static inline uint16_t _ttalink_exam_state_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_exam_state_t* exam_state, bool nocrc)
{
    if(nocrc){
        return ttalink_exam_state_pack_nocrc(dst_addr, src_addr, msg, exam_state->time, exam_state->uav_id, exam_state->longi, exam_state->latit, exam_state->altitude, exam_state->height, exam_state->voltage, exam_state->angularVelocity, exam_state->guoundspeed, exam_state->heading, exam_state->yaw_min, exam_state->yaw_max, exam_state->pitch_min, exam_state->pitch_max, exam_state->roll_min, exam_state->roll_max, exam_state->flight_mode, exam_state->state, exam_state->gps_stat_num, exam_state->fix_type);
    }else{
        return ttalink_exam_state_pack(dst_addr, src_addr, msg, exam_state->time, exam_state->uav_id, exam_state->longi, exam_state->latit, exam_state->altitude, exam_state->height, exam_state->voltage, exam_state->angularVelocity, exam_state->guoundspeed, exam_state->heading, exam_state->yaw_min, exam_state->yaw_max, exam_state->pitch_min, exam_state->pitch_max, exam_state->roll_min, exam_state->roll_max, exam_state->flight_mode, exam_state->state, exam_state->gps_stat_num, exam_state->fix_type);
    }
    
}

/**
 * @brief Encode a exam_state struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param exam_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_exam_state_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_exam_state_t* exam_state)
{
    return _ttalink_exam_state_encode(dst_addr, src_addr, msg, exam_state, false);
}

/**
 * @brief Encode a exam_state struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param exam_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_exam_state_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_exam_state_t* exam_state)
{
    return _ttalink_exam_state_encode(dst_addr, src_addr, msg, exam_state, true);
}


static inline uint16_t _ttalink_exam_state_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_exam_state_t* exam_state, bool nocrc)
{
    if(nocrc){
        return ttalink_exam_state_pack_chan_nocrc(dst_addr, src_addr, chan, msg, exam_state->time, exam_state->uav_id, exam_state->longi, exam_state->latit, exam_state->altitude, exam_state->height, exam_state->voltage, exam_state->angularVelocity, exam_state->guoundspeed, exam_state->heading, exam_state->yaw_min, exam_state->yaw_max, exam_state->pitch_min, exam_state->pitch_max, exam_state->roll_min, exam_state->roll_max, exam_state->flight_mode, exam_state->state, exam_state->gps_stat_num, exam_state->fix_type);
    }else{
        return ttalink_exam_state_pack_chan(dst_addr, src_addr, chan, msg, exam_state->time, exam_state->uav_id, exam_state->longi, exam_state->latit, exam_state->altitude, exam_state->height, exam_state->voltage, exam_state->angularVelocity, exam_state->guoundspeed, exam_state->heading, exam_state->yaw_min, exam_state->yaw_max, exam_state->pitch_min, exam_state->pitch_max, exam_state->roll_min, exam_state->roll_max, exam_state->flight_mode, exam_state->state, exam_state->gps_stat_num, exam_state->fix_type);
    }
}

/**
 * @brief Encode a exam_state struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param exam_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_exam_state_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_exam_state_t* exam_state)
{
    return _ttalink_exam_state_encode_chan(dst_addr, src_addr, chan, msg, exam_state, false);
}

/**
 * @brief Encode a exam_state struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param exam_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_exam_state_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_exam_state_t* exam_state)
{
    return _ttalink_exam_state_encode_chan(dst_addr, src_addr, chan, msg, exam_state, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_exam_state_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time, uint64_t uav_id, int32_t longi, int32_t latit, int32_t altitude, int32_t height, uint16_t voltage, int16_t angularVelocity, int16_t guoundspeed, int16_t heading, int8_t yaw_min, int8_t yaw_max, int8_t pitch_min, int8_t pitch_max, int8_t roll_min, int8_t roll_max, uint8_t flight_mode, uint8_t state, uint8_t gps_stat_num, uint8_t fix_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_EXAM_STATE_LEN];
    _tta_put_uint64_t(buf, 0, time);
    _tta_put_uint64_t(buf, 8, uav_id);
    _tta_put_int32_t(buf, 16, longi);
    _tta_put_int32_t(buf, 20, latit);
    _tta_put_int32_t(buf, 24, altitude);
    _tta_put_int32_t(buf, 28, height);
    _tta_put_uint16_t(buf, 32, voltage);
    _tta_put_int16_t(buf, 34, angularVelocity);
    _tta_put_int16_t(buf, 36, guoundspeed);
    _tta_put_int16_t(buf, 38, heading);
    _tta_put_int8_t(buf, 40, yaw_min);
    _tta_put_int8_t(buf, 41, yaw_max);
    _tta_put_int8_t(buf, 42, pitch_min);
    _tta_put_int8_t(buf, 43, pitch_max);
    _tta_put_int8_t(buf, 44, roll_min);
    _tta_put_int8_t(buf, 45, roll_max);
    _tta_put_uint8_t(buf, 46, flight_mode);
    _tta_put_uint8_t(buf, 47, state);
    _tta_put_uint8_t(buf, 48, gps_stat_num);
    _tta_put_uint8_t(buf, 49, fix_type);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EXAM_STATE, buf, TTALINK_MSG_ID_EXAM_STATE_MIN_LEN, TTALINK_MSG_ID_EXAM_STATE_LEN, TTALINK_MSG_ID_EXAM_STATE_CRC, nocrc);
#else
    ttalink_exam_state_t packet;
    packet.time = time;
    packet.uav_id = uav_id;
    packet.longi = longi;
    packet.latit = latit;
    packet.altitude = altitude;
    packet.height = height;
    packet.voltage = voltage;
    packet.angularVelocity = angularVelocity;
    packet.guoundspeed = guoundspeed;
    packet.heading = heading;
    packet.yaw_min = yaw_min;
    packet.yaw_max = yaw_max;
    packet.pitch_min = pitch_min;
    packet.pitch_max = pitch_max;
    packet.roll_min = roll_min;
    packet.roll_max = roll_max;
    packet.flight_mode = flight_mode;
    packet.state = state;
    packet.gps_stat_num = gps_stat_num;
    packet.fix_type = fix_type;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EXAM_STATE, (const char *)&packet, TTALINK_MSG_ID_EXAM_STATE_MIN_LEN, TTALINK_MSG_ID_EXAM_STATE_LEN, TTALINK_MSG_ID_EXAM_STATE_CRC, nocrc);
#endif
}

/**
 * @brief Send a exam_state message
 * @param chan TTAlink channel to send the message
 *
 * @param time  utc_time(MS)
 * @param uav_id  uav_id.
 * @param longi  [-180,180] : longi*10000000.
 * @param latit  [-90,90] : latit*10000000.
 * @param altitude  altit*1000(M).
 * @param height  Relative altitude*1000(M).
 * @param voltage  Battery voltage*100(V).
 * @param angularVelocity  angularVelocity*10.
 * @param guoundspeed  guoundspeed*10.
 * @param heading  [0,360]
 * @param yaw_min  [-120,120].
 * @param yaw_max  [-120,120].
 * @param pitch_min  [-45,45].
 * @param pitch_max  [-45,45].
 * @param roll_min  [-45,45].
 * @param roll_max  [-45,45].
 * @param flight_mode  flight_mode.
 * @param state  state : 0-lock 1-unlock.
 * @param gps_stat_num  star.
 * @param fix_type  .
 */
static inline void ttalink_exam_state_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time, uint64_t uav_id, int32_t longi, int32_t latit, int32_t altitude, int32_t height, uint16_t voltage, int16_t angularVelocity, int16_t guoundspeed, int16_t heading, int8_t yaw_min, int8_t yaw_max, int8_t pitch_min, int8_t pitch_max, int8_t roll_min, int8_t roll_max, uint8_t flight_mode, uint8_t state, uint8_t gps_stat_num, uint8_t fix_type)
{
    _ttalink_exam_state_send(dst_addr, src_addr, chan, time, uav_id, longi, latit, altitude, height, voltage, angularVelocity, guoundspeed, heading, yaw_min, yaw_max, pitch_min, pitch_max, roll_min, roll_max, flight_mode, state, gps_stat_num, fix_type, false);
}

/**
 * @brief Send a exam_state message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param time  utc_time(MS)
 * @param uav_id  uav_id.
 * @param longi  [-180,180] : longi*10000000.
 * @param latit  [-90,90] : latit*10000000.
 * @param altitude  altit*1000(M).
 * @param height  Relative altitude*1000(M).
 * @param voltage  Battery voltage*100(V).
 * @param angularVelocity  angularVelocity*10.
 * @param guoundspeed  guoundspeed*10.
 * @param heading  [0,360]
 * @param yaw_min  [-120,120].
 * @param yaw_max  [-120,120].
 * @param pitch_min  [-45,45].
 * @param pitch_max  [-45,45].
 * @param roll_min  [-45,45].
 * @param roll_max  [-45,45].
 * @param flight_mode  flight_mode.
 * @param state  state : 0-lock 1-unlock.
 * @param gps_stat_num  star.
 * @param fix_type  .
 */
static inline void ttalink_exam_state_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time, uint64_t uav_id, int32_t longi, int32_t latit, int32_t altitude, int32_t height, uint16_t voltage, int16_t angularVelocity, int16_t guoundspeed, int16_t heading, int8_t yaw_min, int8_t yaw_max, int8_t pitch_min, int8_t pitch_max, int8_t roll_min, int8_t roll_max, uint8_t flight_mode, uint8_t state, uint8_t gps_stat_num, uint8_t fix_type)
{
    _ttalink_exam_state_send(dst_addr, src_addr, chan, time, uav_id, longi, latit, altitude, height, voltage, angularVelocity, guoundspeed, heading, yaw_min, yaw_max, pitch_min, pitch_max, roll_min, roll_max, flight_mode, state, gps_stat_num, fix_type, true);
}


static inline void _ttalink_exam_state_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_exam_state_t* exam_state, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_exam_state_send_nocrc(dst_addr, src_addr, chan, exam_state->time, exam_state->uav_id, exam_state->longi, exam_state->latit, exam_state->altitude, exam_state->height, exam_state->voltage, exam_state->angularVelocity, exam_state->guoundspeed, exam_state->heading, exam_state->yaw_min, exam_state->yaw_max, exam_state->pitch_min, exam_state->pitch_max, exam_state->roll_min, exam_state->roll_max, exam_state->flight_mode, exam_state->state, exam_state->gps_stat_num, exam_state->fix_type);
    }else{
        ttalink_exam_state_send(dst_addr, src_addr, chan, exam_state->time, exam_state->uav_id, exam_state->longi, exam_state->latit, exam_state->altitude, exam_state->height, exam_state->voltage, exam_state->angularVelocity, exam_state->guoundspeed, exam_state->heading, exam_state->yaw_min, exam_state->yaw_max, exam_state->pitch_min, exam_state->pitch_max, exam_state->roll_min, exam_state->roll_max, exam_state->flight_mode, exam_state->state, exam_state->gps_stat_num, exam_state->fix_type);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EXAM_STATE, (const char *)exam_state, TTALINK_MSG_ID_EXAM_STATE_MIN_LEN, TTALINK_MSG_ID_EXAM_STATE_LEN, TTALINK_MSG_ID_EXAM_STATE_CRC, nocrc);
#endif
}

/**
 * @brief Send a exam_state message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_exam_state_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_exam_state_t* exam_state)
{
    _ttalink_exam_state_send_struct(dst_addr, src_addr, chan, exam_state, false);
}

/**
 * @brief Send a exam_state message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_exam_state_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_exam_state_t* exam_state)
{
    _ttalink_exam_state_send_struct(dst_addr, src_addr, chan, exam_state, true);
}

#if TTALINK_MSG_ID_EXAM_STATE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_exam_state_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time, uint64_t uav_id, int32_t longi, int32_t latit, int32_t altitude, int32_t height, uint16_t voltage, int16_t angularVelocity, int16_t guoundspeed, int16_t heading, int8_t yaw_min, int8_t yaw_max, int8_t pitch_min, int8_t pitch_max, int8_t roll_min, int8_t roll_max, uint8_t flight_mode, uint8_t state, uint8_t gps_stat_num, uint8_t fix_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint64_t(buf, 0, time);
    _tta_put_uint64_t(buf, 8, uav_id);
    _tta_put_int32_t(buf, 16, longi);
    _tta_put_int32_t(buf, 20, latit);
    _tta_put_int32_t(buf, 24, altitude);
    _tta_put_int32_t(buf, 28, height);
    _tta_put_uint16_t(buf, 32, voltage);
    _tta_put_int16_t(buf, 34, angularVelocity);
    _tta_put_int16_t(buf, 36, guoundspeed);
    _tta_put_int16_t(buf, 38, heading);
    _tta_put_int8_t(buf, 40, yaw_min);
    _tta_put_int8_t(buf, 41, yaw_max);
    _tta_put_int8_t(buf, 42, pitch_min);
    _tta_put_int8_t(buf, 43, pitch_max);
    _tta_put_int8_t(buf, 44, roll_min);
    _tta_put_int8_t(buf, 45, roll_max);
    _tta_put_uint8_t(buf, 46, flight_mode);
    _tta_put_uint8_t(buf, 47, state);
    _tta_put_uint8_t(buf, 48, gps_stat_num);
    _tta_put_uint8_t(buf, 49, fix_type);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EXAM_STATE, buf, TTALINK_MSG_ID_EXAM_STATE_MIN_LEN, TTALINK_MSG_ID_EXAM_STATE_LEN, TTALINK_MSG_ID_EXAM_STATE_CRC, nocrc);
#else
    ttalink_exam_state_t *packet = (ttalink_exam_state_t *)msgbuf;
    packet->time = time;
    packet->uav_id = uav_id;
    packet->longi = longi;
    packet->latit = latit;
    packet->altitude = altitude;
    packet->height = height;
    packet->voltage = voltage;
    packet->angularVelocity = angularVelocity;
    packet->guoundspeed = guoundspeed;
    packet->heading = heading;
    packet->yaw_min = yaw_min;
    packet->yaw_max = yaw_max;
    packet->pitch_min = pitch_min;
    packet->pitch_max = pitch_max;
    packet->roll_min = roll_min;
    packet->roll_max = roll_max;
    packet->flight_mode = flight_mode;
    packet->state = state;
    packet->gps_stat_num = gps_stat_num;
    packet->fix_type = fix_type;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EXAM_STATE, (const char *)packet, TTALINK_MSG_ID_EXAM_STATE_MIN_LEN, TTALINK_MSG_ID_EXAM_STATE_LEN, TTALINK_MSG_ID_EXAM_STATE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_exam_state_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time, uint64_t uav_id, int32_t longi, int32_t latit, int32_t altitude, int32_t height, uint16_t voltage, int16_t angularVelocity, int16_t guoundspeed, int16_t heading, int8_t yaw_min, int8_t yaw_max, int8_t pitch_min, int8_t pitch_max, int8_t roll_min, int8_t roll_max, uint8_t flight_mode, uint8_t state, uint8_t gps_stat_num, uint8_t fix_type)
{
    _ttalink_exam_state_send_buf(dst_addr, src_addr, msgbuf, chan, time, uav_id, longi, latit, altitude, height, voltage, angularVelocity, guoundspeed, heading, yaw_min, yaw_max, pitch_min, pitch_max, roll_min, roll_max, flight_mode, state, gps_stat_num, fix_type, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_exam_state_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time, uint64_t uav_id, int32_t longi, int32_t latit, int32_t altitude, int32_t height, uint16_t voltage, int16_t angularVelocity, int16_t guoundspeed, int16_t heading, int8_t yaw_min, int8_t yaw_max, int8_t pitch_min, int8_t pitch_max, int8_t roll_min, int8_t roll_max, uint8_t flight_mode, uint8_t state, uint8_t gps_stat_num, uint8_t fix_type)
{
    _ttalink_exam_state_send_buf(dst_addr, src_addr, msgbuf, chan, time, uav_id, longi, latit, altitude, height, voltage, angularVelocity, guoundspeed, heading, yaw_min, yaw_max, pitch_min, pitch_max, roll_min, roll_max, flight_mode, state, gps_stat_num, fix_type, true);
}
#endif

#endif

// MESSAGE EXAM_STATE UNPACKING


/**
 * @brief Get field time from exam_state message
 *
 * @return  utc_time(MS)
 */
static inline uint64_t ttalink_exam_state_get_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field uav_id from exam_state message
 *
 * @return  uav_id.
 */
static inline uint64_t ttalink_exam_state_get_uav_id(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field longi from exam_state message
 *
 * @return  [-180,180] : longi*10000000.
 */
static inline int32_t ttalink_exam_state_get_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field latit from exam_state message
 *
 * @return  [-90,90] : latit*10000000.
 */
static inline int32_t ttalink_exam_state_get_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field altitude from exam_state message
 *
 * @return  altit*1000(M).
 */
static inline int32_t ttalink_exam_state_get_altitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field height from exam_state message
 *
 * @return  Relative altitude*1000(M).
 */
static inline int32_t ttalink_exam_state_get_height(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  28);
}

/**
 * @brief Get field voltage from exam_state message
 *
 * @return  Battery voltage*100(V).
 */
static inline uint16_t ttalink_exam_state_get_voltage(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field angularVelocity from exam_state message
 *
 * @return  angularVelocity*10.
 */
static inline int16_t ttalink_exam_state_get_angularVelocity(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  34);
}

/**
 * @brief Get field guoundspeed from exam_state message
 *
 * @return  guoundspeed*10.
 */
static inline int16_t ttalink_exam_state_get_guoundspeed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  36);
}

/**
 * @brief Get field heading from exam_state message
 *
 * @return  [0,360]
 */
static inline int16_t ttalink_exam_state_get_heading(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  38);
}

/**
 * @brief Get field yaw_min from exam_state message
 *
 * @return  [-120,120].
 */
static inline int8_t ttalink_exam_state_get_yaw_min(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int8_t(msg,  40);
}

/**
 * @brief Get field yaw_max from exam_state message
 *
 * @return  [-120,120].
 */
static inline int8_t ttalink_exam_state_get_yaw_max(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int8_t(msg,  41);
}

/**
 * @brief Get field pitch_min from exam_state message
 *
 * @return  [-45,45].
 */
static inline int8_t ttalink_exam_state_get_pitch_min(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int8_t(msg,  42);
}

/**
 * @brief Get field pitch_max from exam_state message
 *
 * @return  [-45,45].
 */
static inline int8_t ttalink_exam_state_get_pitch_max(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int8_t(msg,  43);
}

/**
 * @brief Get field roll_min from exam_state message
 *
 * @return  [-45,45].
 */
static inline int8_t ttalink_exam_state_get_roll_min(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int8_t(msg,  44);
}

/**
 * @brief Get field roll_max from exam_state message
 *
 * @return  [-45,45].
 */
static inline int8_t ttalink_exam_state_get_roll_max(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int8_t(msg,  45);
}

/**
 * @brief Get field flight_mode from exam_state message
 *
 * @return  flight_mode.
 */
static inline uint8_t ttalink_exam_state_get_flight_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  46);
}

/**
 * @brief Get field state from exam_state message
 *
 * @return  state : 0-lock 1-unlock.
 */
static inline uint8_t ttalink_exam_state_get_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  47);
}

/**
 * @brief Get field gps_stat_num from exam_state message
 *
 * @return  star.
 */
static inline uint8_t ttalink_exam_state_get_gps_stat_num(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field fix_type from exam_state message
 *
 * @return  .
 */
static inline uint8_t ttalink_exam_state_get_fix_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  49);
}

/**
 * @brief Decode a exam_state message into a struct
 *
 * @param msg The message to decode
 * @param exam_state C-struct to decode the message contents into
 */
static inline void ttalink_exam_state_decode(const ttalink_message_t* msg, ttalink_exam_state_t* exam_state)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    exam_state->time = ttalink_exam_state_get_time(msg);
    exam_state->uav_id = ttalink_exam_state_get_uav_id(msg);
    exam_state->longi = ttalink_exam_state_get_longi(msg);
    exam_state->latit = ttalink_exam_state_get_latit(msg);
    exam_state->altitude = ttalink_exam_state_get_altitude(msg);
    exam_state->height = ttalink_exam_state_get_height(msg);
    exam_state->voltage = ttalink_exam_state_get_voltage(msg);
    exam_state->angularVelocity = ttalink_exam_state_get_angularVelocity(msg);
    exam_state->guoundspeed = ttalink_exam_state_get_guoundspeed(msg);
    exam_state->heading = ttalink_exam_state_get_heading(msg);
    exam_state->yaw_min = ttalink_exam_state_get_yaw_min(msg);
    exam_state->yaw_max = ttalink_exam_state_get_yaw_max(msg);
    exam_state->pitch_min = ttalink_exam_state_get_pitch_min(msg);
    exam_state->pitch_max = ttalink_exam_state_get_pitch_max(msg);
    exam_state->roll_min = ttalink_exam_state_get_roll_min(msg);
    exam_state->roll_max = ttalink_exam_state_get_roll_max(msg);
    exam_state->flight_mode = ttalink_exam_state_get_flight_mode(msg);
    exam_state->state = ttalink_exam_state_get_state(msg);
    exam_state->gps_stat_num = ttalink_exam_state_get_gps_stat_num(msg);
    exam_state->fix_type = ttalink_exam_state_get_fix_type(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_EXAM_STATE_LEN? msg->len : TTALINK_MSG_ID_EXAM_STATE_LEN;
        memset(exam_state, 0, TTALINK_MSG_ID_EXAM_STATE_LEN);
    memcpy(exam_state, _TTA_PAYLOAD(msg), len);
#endif
}
