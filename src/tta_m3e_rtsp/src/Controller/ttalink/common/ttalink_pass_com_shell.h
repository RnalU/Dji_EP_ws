#pragma once
// MESSAGE PASS_COM_SHELL PACKING

#define TTALINK_MSG_ID_PASS_COM_SHELL 2245

TTAPACKED(
typedef struct __ttalink_pass_com_shell_t {
 uint16_t size; /*<  .*/
 uint16_t index; /*<  .*/
 int8_t payload[250]; /*<  .*/
}) ttalink_pass_com_shell_t;

#define TTALINK_MSG_ID_PASS_COM_SHELL_LEN 254
#define TTALINK_MSG_ID_PASS_COM_SHELL_MIN_LEN 254
#define TTALINK_MSG_ID_2245_LEN 254
#define TTALINK_MSG_ID_2245_MIN_LEN 254

#define TTALINK_MSG_ID_PASS_COM_SHELL_CRC 74
#define TTALINK_MSG_ID_2245_CRC 74

#define TTALINK_MSG_PASS_COM_SHELL_FIELD_PAYLOAD_LEN 250

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_PASS_COM_SHELL { \
    2245, \
    "PASS_COM_SHELL", \
    3, \
    {  { "size", NULL, TTALINK_TYPE_UINT16_T, 0, 0, offsetof(ttalink_pass_com_shell_t, size) }, \
         { "index", NULL, TTALINK_TYPE_UINT16_T, 0, 2, offsetof(ttalink_pass_com_shell_t, index) }, \
         { "payload", NULL, TTALINK_TYPE_INT8_T, 250, 4, offsetof(ttalink_pass_com_shell_t, payload) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_PASS_COM_SHELL { \
    "PASS_COM_SHELL", \
    3, \
    {  { "size", NULL, TTALINK_TYPE_UINT16_T, 0, 0, offsetof(ttalink_pass_com_shell_t, size) }, \
         { "index", NULL, TTALINK_TYPE_UINT16_T, 0, 2, offsetof(ttalink_pass_com_shell_t, index) }, \
         { "payload", NULL, TTALINK_TYPE_INT8_T, 250, 4, offsetof(ttalink_pass_com_shell_t, payload) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pass_com_shell_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t size, uint16_t index, const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PASS_COM_SHELL_LEN];
    _tta_put_uint16_t(buf, 0, size);
    _tta_put_uint16_t(buf, 2, index);
    _tta_put_int8_t_array(buf, 4, payload, 250);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PASS_COM_SHELL_LEN);
#else
    ttalink_pass_com_shell_t packet;
    packet.size = size;
    packet.index = index;
    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*250);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PASS_COM_SHELL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PASS_COM_SHELL;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_PASS_COM_SHELL_MIN_LEN, TTALINK_MSG_ID_PASS_COM_SHELL_LEN, TTALINK_MSG_ID_PASS_COM_SHELL_CRC, nocrc);
}

/**
 * @brief Pack a pass_com_shell message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param size  .
 * @param index  .
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pass_com_shell_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t size, uint16_t index, const int8_t *payload)
{
    return _ttalink_pass_com_shell_pack(dst_addr, src_addr, msg,  size, index, payload, false);
}

/**
 * @brief Pack a pass_com_shell message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param size  .
 * @param index  .
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pass_com_shell_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t size, uint16_t index, const int8_t *payload)
{
    return _ttalink_pass_com_shell_pack(dst_addr, src_addr, msg,  size, index, payload, true);
}


static inline uint16_t _ttalink_pass_com_shell_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t size,uint16_t index,const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PASS_COM_SHELL_LEN];
    _tta_put_uint16_t(buf, 0, size);
    _tta_put_uint16_t(buf, 2, index);
    _tta_put_int8_t_array(buf, 4, payload, 250);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PASS_COM_SHELL_LEN);
#else
    ttalink_pass_com_shell_t packet;
    packet.size = size;
    packet.index = index;
    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*250);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PASS_COM_SHELL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PASS_COM_SHELL;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_COM_SHELL_MIN_LEN, TTALINK_MSG_ID_PASS_COM_SHELL_LEN, TTALINK_MSG_ID_PASS_COM_SHELL_CRC, nocrc);
}

/**
 * @brief Pack a pass_com_shell message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param size  .
 * @param index  .
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pass_com_shell_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t size,uint16_t index,const int8_t *payload)
{
    return _ttalink_pass_com_shell_pack_chan(dst_addr, src_addr, chan, msg,  size, index, payload, false);
}

/**
 * @brief Pack a pass_com_shell message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param size  .
 * @param index  .
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pass_com_shell_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t size,uint16_t index,const int8_t *payload)
{
    return _ttalink_pass_com_shell_pack_chan(dst_addr, src_addr, chan, msg,  size, index, payload, true);
}


static inline uint16_t _ttalink_pass_com_shell_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pass_com_shell_t* pass_com_shell, bool nocrc)
{
    if(nocrc){
        return ttalink_pass_com_shell_pack_nocrc(dst_addr, src_addr, msg, pass_com_shell->size, pass_com_shell->index, pass_com_shell->payload);
    }else{
        return ttalink_pass_com_shell_pack(dst_addr, src_addr, msg, pass_com_shell->size, pass_com_shell->index, pass_com_shell->payload);
    }
    
}

/**
 * @brief Encode a pass_com_shell struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pass_com_shell C-struct to read the message contents from
 */
static inline uint16_t ttalink_pass_com_shell_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pass_com_shell_t* pass_com_shell)
{
    return _ttalink_pass_com_shell_encode(dst_addr, src_addr, msg, pass_com_shell, false);
}

/**
 * @brief Encode a pass_com_shell struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pass_com_shell C-struct to read the message contents from
 */
static inline uint16_t ttalink_pass_com_shell_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pass_com_shell_t* pass_com_shell)
{
    return _ttalink_pass_com_shell_encode(dst_addr, src_addr, msg, pass_com_shell, true);
}


static inline uint16_t _ttalink_pass_com_shell_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pass_com_shell_t* pass_com_shell, bool nocrc)
{
    if(nocrc){
        return ttalink_pass_com_shell_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pass_com_shell->size, pass_com_shell->index, pass_com_shell->payload);
    }else{
        return ttalink_pass_com_shell_pack_chan(dst_addr, src_addr, chan, msg, pass_com_shell->size, pass_com_shell->index, pass_com_shell->payload);
    }
}

/**
 * @brief Encode a pass_com_shell struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pass_com_shell C-struct to read the message contents from
 */
static inline uint16_t ttalink_pass_com_shell_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pass_com_shell_t* pass_com_shell)
{
    return _ttalink_pass_com_shell_encode_chan(dst_addr, src_addr, chan, msg, pass_com_shell, false);
}

/**
 * @brief Encode a pass_com_shell struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pass_com_shell C-struct to read the message contents from
 */
static inline uint16_t ttalink_pass_com_shell_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pass_com_shell_t* pass_com_shell)
{
    return _ttalink_pass_com_shell_encode_chan(dst_addr, src_addr, chan, msg, pass_com_shell, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pass_com_shell_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t size, uint16_t index, const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PASS_COM_SHELL_LEN];
    _tta_put_uint16_t(buf, 0, size);
    _tta_put_uint16_t(buf, 2, index);
    _tta_put_int8_t_array(buf, 4, payload, 250);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_COM_SHELL, buf, TTALINK_MSG_ID_PASS_COM_SHELL_MIN_LEN, TTALINK_MSG_ID_PASS_COM_SHELL_LEN, TTALINK_MSG_ID_PASS_COM_SHELL_CRC, nocrc);
#else
    ttalink_pass_com_shell_t packet;
    packet.size = size;
    packet.index = index;
    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*250);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_COM_SHELL, (const char *)&packet, TTALINK_MSG_ID_PASS_COM_SHELL_MIN_LEN, TTALINK_MSG_ID_PASS_COM_SHELL_LEN, TTALINK_MSG_ID_PASS_COM_SHELL_CRC, nocrc);
#endif
}

/**
 * @brief Send a pass_com_shell message
 * @param chan TTAlink channel to send the message
 *
 * @param size  .
 * @param index  .
 * @param payload  .
 */
static inline void ttalink_pass_com_shell_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t size, uint16_t index, const int8_t *payload)
{
    _ttalink_pass_com_shell_send(dst_addr, src_addr, chan, size, index, payload, false);
}

/**
 * @brief Send a pass_com_shell message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param size  .
 * @param index  .
 * @param payload  .
 */
static inline void ttalink_pass_com_shell_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t size, uint16_t index, const int8_t *payload)
{
    _ttalink_pass_com_shell_send(dst_addr, src_addr, chan, size, index, payload, true);
}


static inline void _ttalink_pass_com_shell_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pass_com_shell_t* pass_com_shell, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pass_com_shell_send_nocrc(dst_addr, src_addr, chan, pass_com_shell->size, pass_com_shell->index, pass_com_shell->payload);
    }else{
        ttalink_pass_com_shell_send(dst_addr, src_addr, chan, pass_com_shell->size, pass_com_shell->index, pass_com_shell->payload);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_COM_SHELL, (const char *)pass_com_shell, TTALINK_MSG_ID_PASS_COM_SHELL_MIN_LEN, TTALINK_MSG_ID_PASS_COM_SHELL_LEN, TTALINK_MSG_ID_PASS_COM_SHELL_CRC, nocrc);
#endif
}

/**
 * @brief Send a pass_com_shell message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pass_com_shell_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pass_com_shell_t* pass_com_shell)
{
    _ttalink_pass_com_shell_send_struct(dst_addr, src_addr, chan, pass_com_shell, false);
}

/**
 * @brief Send a pass_com_shell message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pass_com_shell_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pass_com_shell_t* pass_com_shell)
{
    _ttalink_pass_com_shell_send_struct(dst_addr, src_addr, chan, pass_com_shell, true);
}

#if TTALINK_MSG_ID_PASS_COM_SHELL_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pass_com_shell_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t size, uint16_t index, const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint16_t(buf, 0, size);
    _tta_put_uint16_t(buf, 2, index);
    _tta_put_int8_t_array(buf, 4, payload, 250);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_COM_SHELL, buf, TTALINK_MSG_ID_PASS_COM_SHELL_MIN_LEN, TTALINK_MSG_ID_PASS_COM_SHELL_LEN, TTALINK_MSG_ID_PASS_COM_SHELL_CRC, nocrc);
#else
    ttalink_pass_com_shell_t *packet = (ttalink_pass_com_shell_t *)msgbuf;
    packet->size = size;
    packet->index = index;
    tta_array_memcpy(packet->payload, payload, sizeof(int8_t)*250);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_COM_SHELL, (const char *)packet, TTALINK_MSG_ID_PASS_COM_SHELL_MIN_LEN, TTALINK_MSG_ID_PASS_COM_SHELL_LEN, TTALINK_MSG_ID_PASS_COM_SHELL_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pass_com_shell_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t size, uint16_t index, const int8_t *payload)
{
    _ttalink_pass_com_shell_send_buf(dst_addr, src_addr, msgbuf, chan, size, index, payload, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pass_com_shell_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t size, uint16_t index, const int8_t *payload)
{
    _ttalink_pass_com_shell_send_buf(dst_addr, src_addr, msgbuf, chan, size, index, payload, true);
}
#endif

#endif

// MESSAGE PASS_COM_SHELL UNPACKING


/**
 * @brief Get field size from pass_com_shell message
 *
 * @return  .
 */
static inline uint16_t ttalink_pass_com_shell_get_size(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field index from pass_com_shell message
 *
 * @return  .
 */
static inline uint16_t ttalink_pass_com_shell_get_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field payload from pass_com_shell message
 *
 * @return  .
 */
static inline uint16_t ttalink_pass_com_shell_get_payload(const ttalink_message_t* msg, int8_t *payload)
{
    return _TTA_RETURN_int8_t_array(msg, payload, 250,  4);
}

/**
 * @brief Decode a pass_com_shell message into a struct
 *
 * @param msg The message to decode
 * @param pass_com_shell C-struct to decode the message contents into
 */
static inline void ttalink_pass_com_shell_decode(const ttalink_message_t* msg, ttalink_pass_com_shell_t* pass_com_shell)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pass_com_shell->size = ttalink_pass_com_shell_get_size(msg);
    pass_com_shell->index = ttalink_pass_com_shell_get_index(msg);
    ttalink_pass_com_shell_get_payload(msg, pass_com_shell->payload);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_PASS_COM_SHELL_LEN? msg->len : TTALINK_MSG_ID_PASS_COM_SHELL_LEN;
        memset(pass_com_shell, 0, TTALINK_MSG_ID_PASS_COM_SHELL_LEN);
    memcpy(pass_com_shell, _TTA_PAYLOAD(msg), len);
#endif
}
