#pragma once
// MESSAGE STREAM_DRONE_STATES PACKING

#define TTALINK_MSG_ID_STREAM_DRONE_STATES 2143

TTAPACKED(
typedef struct __ttalink_stream_drone_states_t {
 float mill_alt; /*<  .*/
 uint8_t camera_state; /*<  .*/
 uint8_t parachute_state; /*<  .*/
 uint8_t main1_sensor_state; /*<  .*/
 uint8_t main2_sensor_state; /*<  .*/
 uint8_t backup_sensor_state; /*<  .*/
 uint8_t main1_sensor_health; /*<  .*/
 uint8_t main2_sensor_health; /*<  .*/
 uint8_t backup_sensor_health; /*<  .*/
 float takeoff_distance; /*<  .*/
 float taget_distance; /*<  .*/
 int16_t tx2_tempera; /*<  .*/
 int16_t drone_tempera; /*<  .*/
 uint8_t flight_mode; /*<  .*/
}) ttalink_stream_drone_states_t;

#define TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN 25
#define TTALINK_MSG_ID_STREAM_DRONE_STATES_MIN_LEN 25
#define TTALINK_MSG_ID_2143_LEN 25
#define TTALINK_MSG_ID_2143_MIN_LEN 25

#define TTALINK_MSG_ID_STREAM_DRONE_STATES_CRC 24
#define TTALINK_MSG_ID_2143_CRC 24



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_STREAM_DRONE_STATES { \
    2143, \
    "STREAM_DRONE_STATES", \
    14, \
    {  { "mill_alt", NULL, TTALINK_TYPE_FLOAT, 0, 0, offsetof(ttalink_stream_drone_states_t, mill_alt) }, \
         { "camera_state", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_stream_drone_states_t, camera_state) }, \
         { "parachute_state", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_stream_drone_states_t, parachute_state) }, \
         { "main1_sensor_state", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_stream_drone_states_t, main1_sensor_state) }, \
         { "main2_sensor_state", NULL, TTALINK_TYPE_UINT8_T, 0, 7, offsetof(ttalink_stream_drone_states_t, main2_sensor_state) }, \
         { "backup_sensor_state", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_stream_drone_states_t, backup_sensor_state) }, \
         { "main1_sensor_health", NULL, TTALINK_TYPE_UINT8_T, 0, 9, offsetof(ttalink_stream_drone_states_t, main1_sensor_health) }, \
         { "main2_sensor_health", NULL, TTALINK_TYPE_UINT8_T, 0, 10, offsetof(ttalink_stream_drone_states_t, main2_sensor_health) }, \
         { "backup_sensor_health", NULL, TTALINK_TYPE_UINT8_T, 0, 11, offsetof(ttalink_stream_drone_states_t, backup_sensor_health) }, \
         { "takeoff_distance", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_stream_drone_states_t, takeoff_distance) }, \
         { "taget_distance", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_stream_drone_states_t, taget_distance) }, \
         { "tx2_tempera", NULL, TTALINK_TYPE_INT16_T, 0, 20, offsetof(ttalink_stream_drone_states_t, tx2_tempera) }, \
         { "drone_tempera", NULL, TTALINK_TYPE_INT16_T, 0, 22, offsetof(ttalink_stream_drone_states_t, drone_tempera) }, \
         { "flight_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 24, offsetof(ttalink_stream_drone_states_t, flight_mode) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_STREAM_DRONE_STATES { \
    "STREAM_DRONE_STATES", \
    14, \
    {  { "mill_alt", NULL, TTALINK_TYPE_FLOAT, 0, 0, offsetof(ttalink_stream_drone_states_t, mill_alt) }, \
         { "camera_state", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_stream_drone_states_t, camera_state) }, \
         { "parachute_state", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_stream_drone_states_t, parachute_state) }, \
         { "main1_sensor_state", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_stream_drone_states_t, main1_sensor_state) }, \
         { "main2_sensor_state", NULL, TTALINK_TYPE_UINT8_T, 0, 7, offsetof(ttalink_stream_drone_states_t, main2_sensor_state) }, \
         { "backup_sensor_state", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_stream_drone_states_t, backup_sensor_state) }, \
         { "main1_sensor_health", NULL, TTALINK_TYPE_UINT8_T, 0, 9, offsetof(ttalink_stream_drone_states_t, main1_sensor_health) }, \
         { "main2_sensor_health", NULL, TTALINK_TYPE_UINT8_T, 0, 10, offsetof(ttalink_stream_drone_states_t, main2_sensor_health) }, \
         { "backup_sensor_health", NULL, TTALINK_TYPE_UINT8_T, 0, 11, offsetof(ttalink_stream_drone_states_t, backup_sensor_health) }, \
         { "takeoff_distance", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_stream_drone_states_t, takeoff_distance) }, \
         { "taget_distance", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_stream_drone_states_t, taget_distance) }, \
         { "tx2_tempera", NULL, TTALINK_TYPE_INT16_T, 0, 20, offsetof(ttalink_stream_drone_states_t, tx2_tempera) }, \
         { "drone_tempera", NULL, TTALINK_TYPE_INT16_T, 0, 22, offsetof(ttalink_stream_drone_states_t, drone_tempera) }, \
         { "flight_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 24, offsetof(ttalink_stream_drone_states_t, flight_mode) }, \
         } \
}
#endif


static inline uint16_t _ttalink_stream_drone_states_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float mill_alt, uint8_t camera_state, uint8_t parachute_state, uint8_t main1_sensor_state, uint8_t main2_sensor_state, uint8_t backup_sensor_state, uint8_t main1_sensor_health, uint8_t main2_sensor_health, uint8_t backup_sensor_health, float takeoff_distance, float taget_distance, int16_t tx2_tempera, int16_t drone_tempera, uint8_t flight_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN];
    _tta_put_float(buf, 0, mill_alt);
    _tta_put_uint8_t(buf, 4, camera_state);
    _tta_put_uint8_t(buf, 5, parachute_state);
    _tta_put_uint8_t(buf, 6, main1_sensor_state);
    _tta_put_uint8_t(buf, 7, main2_sensor_state);
    _tta_put_uint8_t(buf, 8, backup_sensor_state);
    _tta_put_uint8_t(buf, 9, main1_sensor_health);
    _tta_put_uint8_t(buf, 10, main2_sensor_health);
    _tta_put_uint8_t(buf, 11, backup_sensor_health);
    _tta_put_float(buf, 12, takeoff_distance);
    _tta_put_float(buf, 16, taget_distance);
    _tta_put_int16_t(buf, 20, tx2_tempera);
    _tta_put_int16_t(buf, 22, drone_tempera);
    _tta_put_uint8_t(buf, 24, flight_mode);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN);
#else
    ttalink_stream_drone_states_t packet;
    packet.mill_alt = mill_alt;
    packet.camera_state = camera_state;
    packet.parachute_state = parachute_state;
    packet.main1_sensor_state = main1_sensor_state;
    packet.main2_sensor_state = main2_sensor_state;
    packet.backup_sensor_state = backup_sensor_state;
    packet.main1_sensor_health = main1_sensor_health;
    packet.main2_sensor_health = main2_sensor_health;
    packet.backup_sensor_health = backup_sensor_health;
    packet.takeoff_distance = takeoff_distance;
    packet.taget_distance = taget_distance;
    packet.tx2_tempera = tx2_tempera;
    packet.drone_tempera = drone_tempera;
    packet.flight_mode = flight_mode;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_STREAM_DRONE_STATES;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_STREAM_DRONE_STATES_MIN_LEN, TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN, TTALINK_MSG_ID_STREAM_DRONE_STATES_CRC, nocrc);
}

/**
 * @brief Pack a stream_drone_states message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param mill_alt  .
 * @param camera_state  .
 * @param parachute_state  .
 * @param main1_sensor_state  .
 * @param main2_sensor_state  .
 * @param backup_sensor_state  .
 * @param main1_sensor_health  .
 * @param main2_sensor_health  .
 * @param backup_sensor_health  .
 * @param takeoff_distance  .
 * @param taget_distance  .
 * @param tx2_tempera  .
 * @param drone_tempera  .
 * @param flight_mode  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_drone_states_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float mill_alt, uint8_t camera_state, uint8_t parachute_state, uint8_t main1_sensor_state, uint8_t main2_sensor_state, uint8_t backup_sensor_state, uint8_t main1_sensor_health, uint8_t main2_sensor_health, uint8_t backup_sensor_health, float takeoff_distance, float taget_distance, int16_t tx2_tempera, int16_t drone_tempera, uint8_t flight_mode)
{
    return _ttalink_stream_drone_states_pack(dst_addr, src_addr, msg,  mill_alt, camera_state, parachute_state, main1_sensor_state, main2_sensor_state, backup_sensor_state, main1_sensor_health, main2_sensor_health, backup_sensor_health, takeoff_distance, taget_distance, tx2_tempera, drone_tempera, flight_mode, false);
}

/**
 * @brief Pack a stream_drone_states message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param mill_alt  .
 * @param camera_state  .
 * @param parachute_state  .
 * @param main1_sensor_state  .
 * @param main2_sensor_state  .
 * @param backup_sensor_state  .
 * @param main1_sensor_health  .
 * @param main2_sensor_health  .
 * @param backup_sensor_health  .
 * @param takeoff_distance  .
 * @param taget_distance  .
 * @param tx2_tempera  .
 * @param drone_tempera  .
 * @param flight_mode  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_drone_states_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float mill_alt, uint8_t camera_state, uint8_t parachute_state, uint8_t main1_sensor_state, uint8_t main2_sensor_state, uint8_t backup_sensor_state, uint8_t main1_sensor_health, uint8_t main2_sensor_health, uint8_t backup_sensor_health, float takeoff_distance, float taget_distance, int16_t tx2_tempera, int16_t drone_tempera, uint8_t flight_mode)
{
    return _ttalink_stream_drone_states_pack(dst_addr, src_addr, msg,  mill_alt, camera_state, parachute_state, main1_sensor_state, main2_sensor_state, backup_sensor_state, main1_sensor_health, main2_sensor_health, backup_sensor_health, takeoff_distance, taget_distance, tx2_tempera, drone_tempera, flight_mode, true);
}


static inline uint16_t _ttalink_stream_drone_states_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float mill_alt,uint8_t camera_state,uint8_t parachute_state,uint8_t main1_sensor_state,uint8_t main2_sensor_state,uint8_t backup_sensor_state,uint8_t main1_sensor_health,uint8_t main2_sensor_health,uint8_t backup_sensor_health,float takeoff_distance,float taget_distance,int16_t tx2_tempera,int16_t drone_tempera,uint8_t flight_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN];
    _tta_put_float(buf, 0, mill_alt);
    _tta_put_uint8_t(buf, 4, camera_state);
    _tta_put_uint8_t(buf, 5, parachute_state);
    _tta_put_uint8_t(buf, 6, main1_sensor_state);
    _tta_put_uint8_t(buf, 7, main2_sensor_state);
    _tta_put_uint8_t(buf, 8, backup_sensor_state);
    _tta_put_uint8_t(buf, 9, main1_sensor_health);
    _tta_put_uint8_t(buf, 10, main2_sensor_health);
    _tta_put_uint8_t(buf, 11, backup_sensor_health);
    _tta_put_float(buf, 12, takeoff_distance);
    _tta_put_float(buf, 16, taget_distance);
    _tta_put_int16_t(buf, 20, tx2_tempera);
    _tta_put_int16_t(buf, 22, drone_tempera);
    _tta_put_uint8_t(buf, 24, flight_mode);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN);
#else
    ttalink_stream_drone_states_t packet;
    packet.mill_alt = mill_alt;
    packet.camera_state = camera_state;
    packet.parachute_state = parachute_state;
    packet.main1_sensor_state = main1_sensor_state;
    packet.main2_sensor_state = main2_sensor_state;
    packet.backup_sensor_state = backup_sensor_state;
    packet.main1_sensor_health = main1_sensor_health;
    packet.main2_sensor_health = main2_sensor_health;
    packet.backup_sensor_health = backup_sensor_health;
    packet.takeoff_distance = takeoff_distance;
    packet.taget_distance = taget_distance;
    packet.tx2_tempera = tx2_tempera;
    packet.drone_tempera = drone_tempera;
    packet.flight_mode = flight_mode;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_STREAM_DRONE_STATES;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_DRONE_STATES_MIN_LEN, TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN, TTALINK_MSG_ID_STREAM_DRONE_STATES_CRC, nocrc);
}

/**
 * @brief Pack a stream_drone_states message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mill_alt  .
 * @param camera_state  .
 * @param parachute_state  .
 * @param main1_sensor_state  .
 * @param main2_sensor_state  .
 * @param backup_sensor_state  .
 * @param main1_sensor_health  .
 * @param main2_sensor_health  .
 * @param backup_sensor_health  .
 * @param takeoff_distance  .
 * @param taget_distance  .
 * @param tx2_tempera  .
 * @param drone_tempera  .
 * @param flight_mode  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_drone_states_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float mill_alt,uint8_t camera_state,uint8_t parachute_state,uint8_t main1_sensor_state,uint8_t main2_sensor_state,uint8_t backup_sensor_state,uint8_t main1_sensor_health,uint8_t main2_sensor_health,uint8_t backup_sensor_health,float takeoff_distance,float taget_distance,int16_t tx2_tempera,int16_t drone_tempera,uint8_t flight_mode)
{
    return _ttalink_stream_drone_states_pack_chan(dst_addr, src_addr, chan, msg,  mill_alt, camera_state, parachute_state, main1_sensor_state, main2_sensor_state, backup_sensor_state, main1_sensor_health, main2_sensor_health, backup_sensor_health, takeoff_distance, taget_distance, tx2_tempera, drone_tempera, flight_mode, false);
}

/**
 * @brief Pack a stream_drone_states message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mill_alt  .
 * @param camera_state  .
 * @param parachute_state  .
 * @param main1_sensor_state  .
 * @param main2_sensor_state  .
 * @param backup_sensor_state  .
 * @param main1_sensor_health  .
 * @param main2_sensor_health  .
 * @param backup_sensor_health  .
 * @param takeoff_distance  .
 * @param taget_distance  .
 * @param tx2_tempera  .
 * @param drone_tempera  .
 * @param flight_mode  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_drone_states_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float mill_alt,uint8_t camera_state,uint8_t parachute_state,uint8_t main1_sensor_state,uint8_t main2_sensor_state,uint8_t backup_sensor_state,uint8_t main1_sensor_health,uint8_t main2_sensor_health,uint8_t backup_sensor_health,float takeoff_distance,float taget_distance,int16_t tx2_tempera,int16_t drone_tempera,uint8_t flight_mode)
{
    return _ttalink_stream_drone_states_pack_chan(dst_addr, src_addr, chan, msg,  mill_alt, camera_state, parachute_state, main1_sensor_state, main2_sensor_state, backup_sensor_state, main1_sensor_health, main2_sensor_health, backup_sensor_health, takeoff_distance, taget_distance, tx2_tempera, drone_tempera, flight_mode, true);
}


static inline uint16_t _ttalink_stream_drone_states_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_stream_drone_states_t* stream_drone_states, bool nocrc)
{
    if(nocrc){
        return ttalink_stream_drone_states_pack_nocrc(dst_addr, src_addr, msg, stream_drone_states->mill_alt, stream_drone_states->camera_state, stream_drone_states->parachute_state, stream_drone_states->main1_sensor_state, stream_drone_states->main2_sensor_state, stream_drone_states->backup_sensor_state, stream_drone_states->main1_sensor_health, stream_drone_states->main2_sensor_health, stream_drone_states->backup_sensor_health, stream_drone_states->takeoff_distance, stream_drone_states->taget_distance, stream_drone_states->tx2_tempera, stream_drone_states->drone_tempera, stream_drone_states->flight_mode);
    }else{
        return ttalink_stream_drone_states_pack(dst_addr, src_addr, msg, stream_drone_states->mill_alt, stream_drone_states->camera_state, stream_drone_states->parachute_state, stream_drone_states->main1_sensor_state, stream_drone_states->main2_sensor_state, stream_drone_states->backup_sensor_state, stream_drone_states->main1_sensor_health, stream_drone_states->main2_sensor_health, stream_drone_states->backup_sensor_health, stream_drone_states->takeoff_distance, stream_drone_states->taget_distance, stream_drone_states->tx2_tempera, stream_drone_states->drone_tempera, stream_drone_states->flight_mode);
    }
    
}

/**
 * @brief Encode a stream_drone_states struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param stream_drone_states C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_drone_states_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_stream_drone_states_t* stream_drone_states)
{
    return _ttalink_stream_drone_states_encode(dst_addr, src_addr, msg, stream_drone_states, false);
}

/**
 * @brief Encode a stream_drone_states struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param stream_drone_states C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_drone_states_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_stream_drone_states_t* stream_drone_states)
{
    return _ttalink_stream_drone_states_encode(dst_addr, src_addr, msg, stream_drone_states, true);
}


static inline uint16_t _ttalink_stream_drone_states_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_stream_drone_states_t* stream_drone_states, bool nocrc)
{
    if(nocrc){
        return ttalink_stream_drone_states_pack_chan_nocrc(dst_addr, src_addr, chan, msg, stream_drone_states->mill_alt, stream_drone_states->camera_state, stream_drone_states->parachute_state, stream_drone_states->main1_sensor_state, stream_drone_states->main2_sensor_state, stream_drone_states->backup_sensor_state, stream_drone_states->main1_sensor_health, stream_drone_states->main2_sensor_health, stream_drone_states->backup_sensor_health, stream_drone_states->takeoff_distance, stream_drone_states->taget_distance, stream_drone_states->tx2_tempera, stream_drone_states->drone_tempera, stream_drone_states->flight_mode);
    }else{
        return ttalink_stream_drone_states_pack_chan(dst_addr, src_addr, chan, msg, stream_drone_states->mill_alt, stream_drone_states->camera_state, stream_drone_states->parachute_state, stream_drone_states->main1_sensor_state, stream_drone_states->main2_sensor_state, stream_drone_states->backup_sensor_state, stream_drone_states->main1_sensor_health, stream_drone_states->main2_sensor_health, stream_drone_states->backup_sensor_health, stream_drone_states->takeoff_distance, stream_drone_states->taget_distance, stream_drone_states->tx2_tempera, stream_drone_states->drone_tempera, stream_drone_states->flight_mode);
    }
}

/**
 * @brief Encode a stream_drone_states struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param stream_drone_states C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_drone_states_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_stream_drone_states_t* stream_drone_states)
{
    return _ttalink_stream_drone_states_encode_chan(dst_addr, src_addr, chan, msg, stream_drone_states, false);
}

/**
 * @brief Encode a stream_drone_states struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param stream_drone_states C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_drone_states_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_stream_drone_states_t* stream_drone_states)
{
    return _ttalink_stream_drone_states_encode_chan(dst_addr, src_addr, chan, msg, stream_drone_states, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_stream_drone_states_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float mill_alt, uint8_t camera_state, uint8_t parachute_state, uint8_t main1_sensor_state, uint8_t main2_sensor_state, uint8_t backup_sensor_state, uint8_t main1_sensor_health, uint8_t main2_sensor_health, uint8_t backup_sensor_health, float takeoff_distance, float taget_distance, int16_t tx2_tempera, int16_t drone_tempera, uint8_t flight_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN];
    _tta_put_float(buf, 0, mill_alt);
    _tta_put_uint8_t(buf, 4, camera_state);
    _tta_put_uint8_t(buf, 5, parachute_state);
    _tta_put_uint8_t(buf, 6, main1_sensor_state);
    _tta_put_uint8_t(buf, 7, main2_sensor_state);
    _tta_put_uint8_t(buf, 8, backup_sensor_state);
    _tta_put_uint8_t(buf, 9, main1_sensor_health);
    _tta_put_uint8_t(buf, 10, main2_sensor_health);
    _tta_put_uint8_t(buf, 11, backup_sensor_health);
    _tta_put_float(buf, 12, takeoff_distance);
    _tta_put_float(buf, 16, taget_distance);
    _tta_put_int16_t(buf, 20, tx2_tempera);
    _tta_put_int16_t(buf, 22, drone_tempera);
    _tta_put_uint8_t(buf, 24, flight_mode);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_DRONE_STATES, buf, TTALINK_MSG_ID_STREAM_DRONE_STATES_MIN_LEN, TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN, TTALINK_MSG_ID_STREAM_DRONE_STATES_CRC, nocrc);
#else
    ttalink_stream_drone_states_t packet;
    packet.mill_alt = mill_alt;
    packet.camera_state = camera_state;
    packet.parachute_state = parachute_state;
    packet.main1_sensor_state = main1_sensor_state;
    packet.main2_sensor_state = main2_sensor_state;
    packet.backup_sensor_state = backup_sensor_state;
    packet.main1_sensor_health = main1_sensor_health;
    packet.main2_sensor_health = main2_sensor_health;
    packet.backup_sensor_health = backup_sensor_health;
    packet.takeoff_distance = takeoff_distance;
    packet.taget_distance = taget_distance;
    packet.tx2_tempera = tx2_tempera;
    packet.drone_tempera = drone_tempera;
    packet.flight_mode = flight_mode;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_DRONE_STATES, (const char *)&packet, TTALINK_MSG_ID_STREAM_DRONE_STATES_MIN_LEN, TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN, TTALINK_MSG_ID_STREAM_DRONE_STATES_CRC, nocrc);
#endif
}

/**
 * @brief Send a stream_drone_states message
 * @param chan TTAlink channel to send the message
 *
 * @param mill_alt  .
 * @param camera_state  .
 * @param parachute_state  .
 * @param main1_sensor_state  .
 * @param main2_sensor_state  .
 * @param backup_sensor_state  .
 * @param main1_sensor_health  .
 * @param main2_sensor_health  .
 * @param backup_sensor_health  .
 * @param takeoff_distance  .
 * @param taget_distance  .
 * @param tx2_tempera  .
 * @param drone_tempera  .
 * @param flight_mode  .
 */
static inline void ttalink_stream_drone_states_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float mill_alt, uint8_t camera_state, uint8_t parachute_state, uint8_t main1_sensor_state, uint8_t main2_sensor_state, uint8_t backup_sensor_state, uint8_t main1_sensor_health, uint8_t main2_sensor_health, uint8_t backup_sensor_health, float takeoff_distance, float taget_distance, int16_t tx2_tempera, int16_t drone_tempera, uint8_t flight_mode)
{
    _ttalink_stream_drone_states_send(dst_addr, src_addr, chan, mill_alt, camera_state, parachute_state, main1_sensor_state, main2_sensor_state, backup_sensor_state, main1_sensor_health, main2_sensor_health, backup_sensor_health, takeoff_distance, taget_distance, tx2_tempera, drone_tempera, flight_mode, false);
}

/**
 * @brief Send a stream_drone_states message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param mill_alt  .
 * @param camera_state  .
 * @param parachute_state  .
 * @param main1_sensor_state  .
 * @param main2_sensor_state  .
 * @param backup_sensor_state  .
 * @param main1_sensor_health  .
 * @param main2_sensor_health  .
 * @param backup_sensor_health  .
 * @param takeoff_distance  .
 * @param taget_distance  .
 * @param tx2_tempera  .
 * @param drone_tempera  .
 * @param flight_mode  .
 */
static inline void ttalink_stream_drone_states_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float mill_alt, uint8_t camera_state, uint8_t parachute_state, uint8_t main1_sensor_state, uint8_t main2_sensor_state, uint8_t backup_sensor_state, uint8_t main1_sensor_health, uint8_t main2_sensor_health, uint8_t backup_sensor_health, float takeoff_distance, float taget_distance, int16_t tx2_tempera, int16_t drone_tempera, uint8_t flight_mode)
{
    _ttalink_stream_drone_states_send(dst_addr, src_addr, chan, mill_alt, camera_state, parachute_state, main1_sensor_state, main2_sensor_state, backup_sensor_state, main1_sensor_health, main2_sensor_health, backup_sensor_health, takeoff_distance, taget_distance, tx2_tempera, drone_tempera, flight_mode, true);
}


static inline void _ttalink_stream_drone_states_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_stream_drone_states_t* stream_drone_states, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_stream_drone_states_send_nocrc(dst_addr, src_addr, chan, stream_drone_states->mill_alt, stream_drone_states->camera_state, stream_drone_states->parachute_state, stream_drone_states->main1_sensor_state, stream_drone_states->main2_sensor_state, stream_drone_states->backup_sensor_state, stream_drone_states->main1_sensor_health, stream_drone_states->main2_sensor_health, stream_drone_states->backup_sensor_health, stream_drone_states->takeoff_distance, stream_drone_states->taget_distance, stream_drone_states->tx2_tempera, stream_drone_states->drone_tempera, stream_drone_states->flight_mode);
    }else{
        ttalink_stream_drone_states_send(dst_addr, src_addr, chan, stream_drone_states->mill_alt, stream_drone_states->camera_state, stream_drone_states->parachute_state, stream_drone_states->main1_sensor_state, stream_drone_states->main2_sensor_state, stream_drone_states->backup_sensor_state, stream_drone_states->main1_sensor_health, stream_drone_states->main2_sensor_health, stream_drone_states->backup_sensor_health, stream_drone_states->takeoff_distance, stream_drone_states->taget_distance, stream_drone_states->tx2_tempera, stream_drone_states->drone_tempera, stream_drone_states->flight_mode);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_DRONE_STATES, (const char *)stream_drone_states, TTALINK_MSG_ID_STREAM_DRONE_STATES_MIN_LEN, TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN, TTALINK_MSG_ID_STREAM_DRONE_STATES_CRC, nocrc);
#endif
}

/**
 * @brief Send a stream_drone_states message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_stream_drone_states_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_stream_drone_states_t* stream_drone_states)
{
    _ttalink_stream_drone_states_send_struct(dst_addr, src_addr, chan, stream_drone_states, false);
}

/**
 * @brief Send a stream_drone_states message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_stream_drone_states_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_stream_drone_states_t* stream_drone_states)
{
    _ttalink_stream_drone_states_send_struct(dst_addr, src_addr, chan, stream_drone_states, true);
}

#if TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_stream_drone_states_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float mill_alt, uint8_t camera_state, uint8_t parachute_state, uint8_t main1_sensor_state, uint8_t main2_sensor_state, uint8_t backup_sensor_state, uint8_t main1_sensor_health, uint8_t main2_sensor_health, uint8_t backup_sensor_health, float takeoff_distance, float taget_distance, int16_t tx2_tempera, int16_t drone_tempera, uint8_t flight_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_float(buf, 0, mill_alt);
    _tta_put_uint8_t(buf, 4, camera_state);
    _tta_put_uint8_t(buf, 5, parachute_state);
    _tta_put_uint8_t(buf, 6, main1_sensor_state);
    _tta_put_uint8_t(buf, 7, main2_sensor_state);
    _tta_put_uint8_t(buf, 8, backup_sensor_state);
    _tta_put_uint8_t(buf, 9, main1_sensor_health);
    _tta_put_uint8_t(buf, 10, main2_sensor_health);
    _tta_put_uint8_t(buf, 11, backup_sensor_health);
    _tta_put_float(buf, 12, takeoff_distance);
    _tta_put_float(buf, 16, taget_distance);
    _tta_put_int16_t(buf, 20, tx2_tempera);
    _tta_put_int16_t(buf, 22, drone_tempera);
    _tta_put_uint8_t(buf, 24, flight_mode);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_DRONE_STATES, buf, TTALINK_MSG_ID_STREAM_DRONE_STATES_MIN_LEN, TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN, TTALINK_MSG_ID_STREAM_DRONE_STATES_CRC, nocrc);
#else
    ttalink_stream_drone_states_t *packet = (ttalink_stream_drone_states_t *)msgbuf;
    packet->mill_alt = mill_alt;
    packet->camera_state = camera_state;
    packet->parachute_state = parachute_state;
    packet->main1_sensor_state = main1_sensor_state;
    packet->main2_sensor_state = main2_sensor_state;
    packet->backup_sensor_state = backup_sensor_state;
    packet->main1_sensor_health = main1_sensor_health;
    packet->main2_sensor_health = main2_sensor_health;
    packet->backup_sensor_health = backup_sensor_health;
    packet->takeoff_distance = takeoff_distance;
    packet->taget_distance = taget_distance;
    packet->tx2_tempera = tx2_tempera;
    packet->drone_tempera = drone_tempera;
    packet->flight_mode = flight_mode;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_DRONE_STATES, (const char *)packet, TTALINK_MSG_ID_STREAM_DRONE_STATES_MIN_LEN, TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN, TTALINK_MSG_ID_STREAM_DRONE_STATES_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_stream_drone_states_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float mill_alt, uint8_t camera_state, uint8_t parachute_state, uint8_t main1_sensor_state, uint8_t main2_sensor_state, uint8_t backup_sensor_state, uint8_t main1_sensor_health, uint8_t main2_sensor_health, uint8_t backup_sensor_health, float takeoff_distance, float taget_distance, int16_t tx2_tempera, int16_t drone_tempera, uint8_t flight_mode)
{
    _ttalink_stream_drone_states_send_buf(dst_addr, src_addr, msgbuf, chan, mill_alt, camera_state, parachute_state, main1_sensor_state, main2_sensor_state, backup_sensor_state, main1_sensor_health, main2_sensor_health, backup_sensor_health, takeoff_distance, taget_distance, tx2_tempera, drone_tempera, flight_mode, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_stream_drone_states_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float mill_alt, uint8_t camera_state, uint8_t parachute_state, uint8_t main1_sensor_state, uint8_t main2_sensor_state, uint8_t backup_sensor_state, uint8_t main1_sensor_health, uint8_t main2_sensor_health, uint8_t backup_sensor_health, float takeoff_distance, float taget_distance, int16_t tx2_tempera, int16_t drone_tempera, uint8_t flight_mode)
{
    _ttalink_stream_drone_states_send_buf(dst_addr, src_addr, msgbuf, chan, mill_alt, camera_state, parachute_state, main1_sensor_state, main2_sensor_state, backup_sensor_state, main1_sensor_health, main2_sensor_health, backup_sensor_health, takeoff_distance, taget_distance, tx2_tempera, drone_tempera, flight_mode, true);
}
#endif

#endif

// MESSAGE STREAM_DRONE_STATES UNPACKING


/**
 * @brief Get field mill_alt from stream_drone_states message
 *
 * @return  .
 */
static inline float ttalink_stream_drone_states_get_mill_alt(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  0);
}

/**
 * @brief Get field camera_state from stream_drone_states message
 *
 * @return  .
 */
static inline uint8_t ttalink_stream_drone_states_get_camera_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field parachute_state from stream_drone_states message
 *
 * @return  .
 */
static inline uint8_t ttalink_stream_drone_states_get_parachute_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field main1_sensor_state from stream_drone_states message
 *
 * @return  .
 */
static inline uint8_t ttalink_stream_drone_states_get_main1_sensor_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field main2_sensor_state from stream_drone_states message
 *
 * @return  .
 */
static inline uint8_t ttalink_stream_drone_states_get_main2_sensor_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field backup_sensor_state from stream_drone_states message
 *
 * @return  .
 */
static inline uint8_t ttalink_stream_drone_states_get_backup_sensor_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field main1_sensor_health from stream_drone_states message
 *
 * @return  .
 */
static inline uint8_t ttalink_stream_drone_states_get_main1_sensor_health(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field main2_sensor_health from stream_drone_states message
 *
 * @return  .
 */
static inline uint8_t ttalink_stream_drone_states_get_main2_sensor_health(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field backup_sensor_health from stream_drone_states message
 *
 * @return  .
 */
static inline uint8_t ttalink_stream_drone_states_get_backup_sensor_health(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field takeoff_distance from stream_drone_states message
 *
 * @return  .
 */
static inline float ttalink_stream_drone_states_get_takeoff_distance(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  12);
}

/**
 * @brief Get field taget_distance from stream_drone_states message
 *
 * @return  .
 */
static inline float ttalink_stream_drone_states_get_taget_distance(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field tx2_tempera from stream_drone_states message
 *
 * @return  .
 */
static inline int16_t ttalink_stream_drone_states_get_tx2_tempera(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field drone_tempera from stream_drone_states message
 *
 * @return  .
 */
static inline int16_t ttalink_stream_drone_states_get_drone_tempera(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field flight_mode from stream_drone_states message
 *
 * @return  .
 */
static inline uint8_t ttalink_stream_drone_states_get_flight_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Decode a stream_drone_states message into a struct
 *
 * @param msg The message to decode
 * @param stream_drone_states C-struct to decode the message contents into
 */
static inline void ttalink_stream_drone_states_decode(const ttalink_message_t* msg, ttalink_stream_drone_states_t* stream_drone_states)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    stream_drone_states->mill_alt = ttalink_stream_drone_states_get_mill_alt(msg);
    stream_drone_states->camera_state = ttalink_stream_drone_states_get_camera_state(msg);
    stream_drone_states->parachute_state = ttalink_stream_drone_states_get_parachute_state(msg);
    stream_drone_states->main1_sensor_state = ttalink_stream_drone_states_get_main1_sensor_state(msg);
    stream_drone_states->main2_sensor_state = ttalink_stream_drone_states_get_main2_sensor_state(msg);
    stream_drone_states->backup_sensor_state = ttalink_stream_drone_states_get_backup_sensor_state(msg);
    stream_drone_states->main1_sensor_health = ttalink_stream_drone_states_get_main1_sensor_health(msg);
    stream_drone_states->main2_sensor_health = ttalink_stream_drone_states_get_main2_sensor_health(msg);
    stream_drone_states->backup_sensor_health = ttalink_stream_drone_states_get_backup_sensor_health(msg);
    stream_drone_states->takeoff_distance = ttalink_stream_drone_states_get_takeoff_distance(msg);
    stream_drone_states->taget_distance = ttalink_stream_drone_states_get_taget_distance(msg);
    stream_drone_states->tx2_tempera = ttalink_stream_drone_states_get_tx2_tempera(msg);
    stream_drone_states->drone_tempera = ttalink_stream_drone_states_get_drone_tempera(msg);
    stream_drone_states->flight_mode = ttalink_stream_drone_states_get_flight_mode(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN? msg->len : TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN;
        memset(stream_drone_states, 0, TTALINK_MSG_ID_STREAM_DRONE_STATES_LEN);
    memcpy(stream_drone_states, _TTA_PAYLOAD(msg), len);
#endif
}
