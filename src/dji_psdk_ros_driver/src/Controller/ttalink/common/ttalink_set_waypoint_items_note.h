#pragma once
// MESSAGE SET_WAYPOINT_ITEMS_NOTE PACKING

#define TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE 2206

TTAPACKED(
typedef struct __ttalink_set_waypoint_items_note_t {
 uint8_t size; /*<  .*/
 uint8_t wp_name[8]; /*<  .*/
 int8_t note[400]; /*<  .*/
 float param1; /*<  .*/
 float param2; /*<  .*/
}) ttalink_set_waypoint_items_note_t;

#define TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN 417
#define TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_MIN_LEN 417
#define TTALINK_MSG_ID_2206_LEN 417
#define TTALINK_MSG_ID_2206_MIN_LEN 417

#define TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_CRC 134
#define TTALINK_MSG_ID_2206_CRC 134

#define TTALINK_MSG_SET_WAYPOINT_ITEMS_NOTE_FIELD_WP_NAME_LEN 8
#define TTALINK_MSG_SET_WAYPOINT_ITEMS_NOTE_FIELD_NOTE_LEN 400

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SET_WAYPOINT_ITEMS_NOTE { \
    2206, \
    "SET_WAYPOINT_ITEMS_NOTE", \
    5, \
    {  { "size", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_set_waypoint_items_note_t, size) }, \
         { "wp_name", NULL, TTALINK_TYPE_UINT8_T, 8, 1, offsetof(ttalink_set_waypoint_items_note_t, wp_name) }, \
         { "note", NULL, TTALINK_TYPE_INT8_T, 400, 9, offsetof(ttalink_set_waypoint_items_note_t, note) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 409, offsetof(ttalink_set_waypoint_items_note_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 413, offsetof(ttalink_set_waypoint_items_note_t, param2) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SET_WAYPOINT_ITEMS_NOTE { \
    "SET_WAYPOINT_ITEMS_NOTE", \
    5, \
    {  { "size", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_set_waypoint_items_note_t, size) }, \
         { "wp_name", NULL, TTALINK_TYPE_UINT8_T, 8, 1, offsetof(ttalink_set_waypoint_items_note_t, wp_name) }, \
         { "note", NULL, TTALINK_TYPE_INT8_T, 400, 9, offsetof(ttalink_set_waypoint_items_note_t, note) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 409, offsetof(ttalink_set_waypoint_items_note_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 413, offsetof(ttalink_set_waypoint_items_note_t, param2) }, \
         } \
}
#endif


static inline uint16_t _ttalink_set_waypoint_items_note_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t size, const uint8_t *wp_name, const int8_t *note, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN];
    _tta_put_uint8_t(buf, 0, size);
    _tta_put_float(buf, 409, param1);
    _tta_put_float(buf, 413, param2);
    _tta_put_uint8_t_array(buf, 1, wp_name, 8);
    _tta_put_int8_t_array(buf, 9, note, 400);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN);
#else
    ttalink_set_waypoint_items_note_t packet;
    packet.size = size;
    packet.param1 = param1;
    packet.param2 = param2;
    tta_array_memcpy(packet.wp_name, wp_name, sizeof(uint8_t)*8);
    tta_array_memcpy(packet.note, note, sizeof(int8_t)*400);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_CRC, nocrc);
}

/**
 * @brief Pack a set_waypoint_items_note message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param size  .
 * @param wp_name  .
 * @param note  .
 * @param param1  .
 * @param param2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_items_note_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t size, const uint8_t *wp_name, const int8_t *note, float param1, float param2)
{
    return _ttalink_set_waypoint_items_note_pack(dst_addr, src_addr, msg,  size, wp_name, note, param1, param2, false);
}

/**
 * @brief Pack a set_waypoint_items_note message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param size  .
 * @param wp_name  .
 * @param note  .
 * @param param1  .
 * @param param2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_items_note_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t size, const uint8_t *wp_name, const int8_t *note, float param1, float param2)
{
    return _ttalink_set_waypoint_items_note_pack(dst_addr, src_addr, msg,  size, wp_name, note, param1, param2, true);
}


static inline uint16_t _ttalink_set_waypoint_items_note_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t size,const uint8_t *wp_name,const int8_t *note,float param1,float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN];
    _tta_put_uint8_t(buf, 0, size);
    _tta_put_float(buf, 409, param1);
    _tta_put_float(buf, 413, param2);
    _tta_put_uint8_t_array(buf, 1, wp_name, 8);
    _tta_put_int8_t_array(buf, 9, note, 400);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN);
#else
    ttalink_set_waypoint_items_note_t packet;
    packet.size = size;
    packet.param1 = param1;
    packet.param2 = param2;
    tta_array_memcpy(packet.wp_name, wp_name, sizeof(uint8_t)*8);
    tta_array_memcpy(packet.note, note, sizeof(int8_t)*400);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_CRC, nocrc);
}

/**
 * @brief Pack a set_waypoint_items_note message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param size  .
 * @param wp_name  .
 * @param note  .
 * @param param1  .
 * @param param2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_items_note_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t size,const uint8_t *wp_name,const int8_t *note,float param1,float param2)
{
    return _ttalink_set_waypoint_items_note_pack_chan(dst_addr, src_addr, chan, msg,  size, wp_name, note, param1, param2, false);
}

/**
 * @brief Pack a set_waypoint_items_note message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param size  .
 * @param wp_name  .
 * @param note  .
 * @param param1  .
 * @param param2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_items_note_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t size,const uint8_t *wp_name,const int8_t *note,float param1,float param2)
{
    return _ttalink_set_waypoint_items_note_pack_chan(dst_addr, src_addr, chan, msg,  size, wp_name, note, param1, param2, true);
}


static inline uint16_t _ttalink_set_waypoint_items_note_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_waypoint_items_note_t* set_waypoint_items_note, bool nocrc)
{
    if(nocrc){
        return ttalink_set_waypoint_items_note_pack_nocrc(dst_addr, src_addr, msg, set_waypoint_items_note->size, set_waypoint_items_note->wp_name, set_waypoint_items_note->note, set_waypoint_items_note->param1, set_waypoint_items_note->param2);
    }else{
        return ttalink_set_waypoint_items_note_pack(dst_addr, src_addr, msg, set_waypoint_items_note->size, set_waypoint_items_note->wp_name, set_waypoint_items_note->note, set_waypoint_items_note->param1, set_waypoint_items_note->param2);
    }
    
}

/**
 * @brief Encode a set_waypoint_items_note struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_items_note C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_items_note_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_waypoint_items_note_t* set_waypoint_items_note)
{
    return _ttalink_set_waypoint_items_note_encode(dst_addr, src_addr, msg, set_waypoint_items_note, false);
}

/**
 * @brief Encode a set_waypoint_items_note struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_items_note C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_items_note_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_waypoint_items_note_t* set_waypoint_items_note)
{
    return _ttalink_set_waypoint_items_note_encode(dst_addr, src_addr, msg, set_waypoint_items_note, true);
}


static inline uint16_t _ttalink_set_waypoint_items_note_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_waypoint_items_note_t* set_waypoint_items_note, bool nocrc)
{
    if(nocrc){
        return ttalink_set_waypoint_items_note_pack_chan_nocrc(dst_addr, src_addr, chan, msg, set_waypoint_items_note->size, set_waypoint_items_note->wp_name, set_waypoint_items_note->note, set_waypoint_items_note->param1, set_waypoint_items_note->param2);
    }else{
        return ttalink_set_waypoint_items_note_pack_chan(dst_addr, src_addr, chan, msg, set_waypoint_items_note->size, set_waypoint_items_note->wp_name, set_waypoint_items_note->note, set_waypoint_items_note->param1, set_waypoint_items_note->param2);
    }
}

/**
 * @brief Encode a set_waypoint_items_note struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_items_note C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_items_note_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_waypoint_items_note_t* set_waypoint_items_note)
{
    return _ttalink_set_waypoint_items_note_encode_chan(dst_addr, src_addr, chan, msg, set_waypoint_items_note, false);
}

/**
 * @brief Encode a set_waypoint_items_note struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_items_note C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_items_note_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_waypoint_items_note_t* set_waypoint_items_note)
{
    return _ttalink_set_waypoint_items_note_encode_chan(dst_addr, src_addr, chan, msg, set_waypoint_items_note, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_set_waypoint_items_note_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t size, const uint8_t *wp_name, const int8_t *note, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN];
    _tta_put_uint8_t(buf, 0, size);
    _tta_put_float(buf, 409, param1);
    _tta_put_float(buf, 413, param2);
    _tta_put_uint8_t_array(buf, 1, wp_name, 8);
    _tta_put_int8_t_array(buf, 9, note, 400);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE, buf, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_CRC, nocrc);
#else
    ttalink_set_waypoint_items_note_t packet;
    packet.size = size;
    packet.param1 = param1;
    packet.param2 = param2;
    tta_array_memcpy(packet.wp_name, wp_name, sizeof(uint8_t)*8);
    tta_array_memcpy(packet.note, note, sizeof(int8_t)*400);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE, (const char *)&packet, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_CRC, nocrc);
#endif
}

/**
 * @brief Send a set_waypoint_items_note message
 * @param chan TTAlink channel to send the message
 *
 * @param size  .
 * @param wp_name  .
 * @param note  .
 * @param param1  .
 * @param param2  .
 */
static inline void ttalink_set_waypoint_items_note_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t size, const uint8_t *wp_name, const int8_t *note, float param1, float param2)
{
    _ttalink_set_waypoint_items_note_send(dst_addr, src_addr, chan, size, wp_name, note, param1, param2, false);
}

/**
 * @brief Send a set_waypoint_items_note message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param size  .
 * @param wp_name  .
 * @param note  .
 * @param param1  .
 * @param param2  .
 */
static inline void ttalink_set_waypoint_items_note_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t size, const uint8_t *wp_name, const int8_t *note, float param1, float param2)
{
    _ttalink_set_waypoint_items_note_send(dst_addr, src_addr, chan, size, wp_name, note, param1, param2, true);
}


static inline void _ttalink_set_waypoint_items_note_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_waypoint_items_note_t* set_waypoint_items_note, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_set_waypoint_items_note_send_nocrc(dst_addr, src_addr, chan, set_waypoint_items_note->size, set_waypoint_items_note->wp_name, set_waypoint_items_note->note, set_waypoint_items_note->param1, set_waypoint_items_note->param2);
    }else{
        ttalink_set_waypoint_items_note_send(dst_addr, src_addr, chan, set_waypoint_items_note->size, set_waypoint_items_note->wp_name, set_waypoint_items_note->note, set_waypoint_items_note->param1, set_waypoint_items_note->param2);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE, (const char *)set_waypoint_items_note, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_CRC, nocrc);
#endif
}

/**
 * @brief Send a set_waypoint_items_note message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_set_waypoint_items_note_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_waypoint_items_note_t* set_waypoint_items_note)
{
    _ttalink_set_waypoint_items_note_send_struct(dst_addr, src_addr, chan, set_waypoint_items_note, false);
}

/**
 * @brief Send a set_waypoint_items_note message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_set_waypoint_items_note_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_waypoint_items_note_t* set_waypoint_items_note)
{
    _ttalink_set_waypoint_items_note_send_struct(dst_addr, src_addr, chan, set_waypoint_items_note, true);
}

#if TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_set_waypoint_items_note_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t size, const uint8_t *wp_name, const int8_t *note, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, size);
    _tta_put_float(buf, 409, param1);
    _tta_put_float(buf, 413, param2);
    _tta_put_uint8_t_array(buf, 1, wp_name, 8);
    _tta_put_int8_t_array(buf, 9, note, 400);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE, buf, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_CRC, nocrc);
#else
    ttalink_set_waypoint_items_note_t *packet = (ttalink_set_waypoint_items_note_t *)msgbuf;
    packet->size = size;
    packet->param1 = param1;
    packet->param2 = param2;
    tta_array_memcpy(packet->wp_name, wp_name, sizeof(uint8_t)*8);
    tta_array_memcpy(packet->note, note, sizeof(int8_t)*400);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE, (const char *)packet, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_set_waypoint_items_note_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t size, const uint8_t *wp_name, const int8_t *note, float param1, float param2)
{
    _ttalink_set_waypoint_items_note_send_buf(dst_addr, src_addr, msgbuf, chan, size, wp_name, note, param1, param2, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_set_waypoint_items_note_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t size, const uint8_t *wp_name, const int8_t *note, float param1, float param2)
{
    _ttalink_set_waypoint_items_note_send_buf(dst_addr, src_addr, msgbuf, chan, size, wp_name, note, param1, param2, true);
}
#endif

#endif

// MESSAGE SET_WAYPOINT_ITEMS_NOTE UNPACKING


/**
 * @brief Get field size from set_waypoint_items_note message
 *
 * @return  .
 */
static inline uint8_t ttalink_set_waypoint_items_note_get_size(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field wp_name from set_waypoint_items_note message
 *
 * @return  .
 */
static inline uint16_t ttalink_set_waypoint_items_note_get_wp_name(const ttalink_message_t* msg, uint8_t *wp_name)
{
    return _TTA_RETURN_uint8_t_array(msg, wp_name, 8,  1);
}

/**
 * @brief Get field note from set_waypoint_items_note message
 *
 * @return  .
 */
static inline uint16_t ttalink_set_waypoint_items_note_get_note(const ttalink_message_t* msg, int8_t *note)
{
    return _TTA_RETURN_int8_t_array(msg, note, 400,  9);
}

/**
 * @brief Get field param1 from set_waypoint_items_note message
 *
 * @return  .
 */
static inline float ttalink_set_waypoint_items_note_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  409);
}

/**
 * @brief Get field param2 from set_waypoint_items_note message
 *
 * @return  .
 */
static inline float ttalink_set_waypoint_items_note_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  413);
}

/**
 * @brief Decode a set_waypoint_items_note message into a struct
 *
 * @param msg The message to decode
 * @param set_waypoint_items_note C-struct to decode the message contents into
 */
static inline void ttalink_set_waypoint_items_note_decode(const ttalink_message_t* msg, ttalink_set_waypoint_items_note_t* set_waypoint_items_note)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    set_waypoint_items_note->size = ttalink_set_waypoint_items_note_get_size(msg);
    ttalink_set_waypoint_items_note_get_wp_name(msg, set_waypoint_items_note->wp_name);
    ttalink_set_waypoint_items_note_get_note(msg, set_waypoint_items_note->note);
    set_waypoint_items_note->param1 = ttalink_set_waypoint_items_note_get_param1(msg);
    set_waypoint_items_note->param2 = ttalink_set_waypoint_items_note_get_param2(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN? msg->len : TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN;
        memset(set_waypoint_items_note, 0, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_NOTE_LEN);
    memcpy(set_waypoint_items_note, _TTA_PAYLOAD(msg), len);
#endif
}
