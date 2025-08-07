#pragma once
// MESSAGE TEST_PWM_OUTPUT PACKING

#define TTALINK_MSG_ID_TEST_PWM_OUTPUT 2262

TTAPACKED(
typedef struct __ttalink_test_pwm_output_t {
 uint8_t idx; /*< 通道索引*/
 uint16_t value; /*< 通道值*/
}) ttalink_test_pwm_output_t;

#define TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN 3
#define TTALINK_MSG_ID_TEST_PWM_OUTPUT_MIN_LEN 3
#define TTALINK_MSG_ID_2262_LEN 3
#define TTALINK_MSG_ID_2262_MIN_LEN 3

#define TTALINK_MSG_ID_TEST_PWM_OUTPUT_CRC 119
#define TTALINK_MSG_ID_2262_CRC 119



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_TEST_PWM_OUTPUT { \
    2262, \
    "TEST_PWM_OUTPUT", \
    2, \
    {  { "idx", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_test_pwm_output_t, idx) }, \
         { "value", NULL, TTALINK_TYPE_UINT16_T, 0, 1, offsetof(ttalink_test_pwm_output_t, value) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_TEST_PWM_OUTPUT { \
    "TEST_PWM_OUTPUT", \
    2, \
    {  { "idx", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_test_pwm_output_t, idx) }, \
         { "value", NULL, TTALINK_TYPE_UINT16_T, 0, 1, offsetof(ttalink_test_pwm_output_t, value) }, \
         } \
}
#endif


static inline uint16_t _ttalink_test_pwm_output_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t idx, uint16_t value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN];
    _tta_put_uint8_t(buf, 0, idx);
    _tta_put_uint16_t(buf, 1, value);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN);
#else
    ttalink_test_pwm_output_t packet;
    packet.idx = idx;
    packet.value = value;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TEST_PWM_OUTPUT;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_TEST_PWM_OUTPUT_MIN_LEN, TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN, TTALINK_MSG_ID_TEST_PWM_OUTPUT_CRC, nocrc);
}

/**
 * @brief Pack a test_pwm_output message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param idx 通道索引
 * @param value 通道值
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_test_pwm_output_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t idx, uint16_t value)
{
    return _ttalink_test_pwm_output_pack(dst_addr, src_addr, msg,  idx, value, false);
}

/**
 * @brief Pack a test_pwm_output message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param idx 通道索引
 * @param value 通道值
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_test_pwm_output_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t idx, uint16_t value)
{
    return _ttalink_test_pwm_output_pack(dst_addr, src_addr, msg,  idx, value, true);
}


static inline uint16_t _ttalink_test_pwm_output_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t idx,uint16_t value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN];
    _tta_put_uint8_t(buf, 0, idx);
    _tta_put_uint16_t(buf, 1, value);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN);
#else
    ttalink_test_pwm_output_t packet;
    packet.idx = idx;
    packet.value = value;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TEST_PWM_OUTPUT;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_TEST_PWM_OUTPUT_MIN_LEN, TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN, TTALINK_MSG_ID_TEST_PWM_OUTPUT_CRC, nocrc);
}

/**
 * @brief Pack a test_pwm_output message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param idx 通道索引
 * @param value 通道值
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_test_pwm_output_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t idx,uint16_t value)
{
    return _ttalink_test_pwm_output_pack_chan(dst_addr, src_addr, chan, msg,  idx, value, false);
}

/**
 * @brief Pack a test_pwm_output message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param idx 通道索引
 * @param value 通道值
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_test_pwm_output_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t idx,uint16_t value)
{
    return _ttalink_test_pwm_output_pack_chan(dst_addr, src_addr, chan, msg,  idx, value, true);
}


static inline uint16_t _ttalink_test_pwm_output_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_test_pwm_output_t* test_pwm_output, bool nocrc)
{
    if(nocrc){
        return ttalink_test_pwm_output_pack_nocrc(dst_addr, src_addr, msg, test_pwm_output->idx, test_pwm_output->value);
    }else{
        return ttalink_test_pwm_output_pack(dst_addr, src_addr, msg, test_pwm_output->idx, test_pwm_output->value);
    }
    
}

/**
 * @brief Encode a test_pwm_output struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param test_pwm_output C-struct to read the message contents from
 */
static inline uint16_t ttalink_test_pwm_output_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_test_pwm_output_t* test_pwm_output)
{
    return _ttalink_test_pwm_output_encode(dst_addr, src_addr, msg, test_pwm_output, false);
}

/**
 * @brief Encode a test_pwm_output struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param test_pwm_output C-struct to read the message contents from
 */
static inline uint16_t ttalink_test_pwm_output_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_test_pwm_output_t* test_pwm_output)
{
    return _ttalink_test_pwm_output_encode(dst_addr, src_addr, msg, test_pwm_output, true);
}


static inline uint16_t _ttalink_test_pwm_output_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_test_pwm_output_t* test_pwm_output, bool nocrc)
{
    if(nocrc){
        return ttalink_test_pwm_output_pack_chan_nocrc(dst_addr, src_addr, chan, msg, test_pwm_output->idx, test_pwm_output->value);
    }else{
        return ttalink_test_pwm_output_pack_chan(dst_addr, src_addr, chan, msg, test_pwm_output->idx, test_pwm_output->value);
    }
}

/**
 * @brief Encode a test_pwm_output struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param test_pwm_output C-struct to read the message contents from
 */
static inline uint16_t ttalink_test_pwm_output_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_test_pwm_output_t* test_pwm_output)
{
    return _ttalink_test_pwm_output_encode_chan(dst_addr, src_addr, chan, msg, test_pwm_output, false);
}

/**
 * @brief Encode a test_pwm_output struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param test_pwm_output C-struct to read the message contents from
 */
static inline uint16_t ttalink_test_pwm_output_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_test_pwm_output_t* test_pwm_output)
{
    return _ttalink_test_pwm_output_encode_chan(dst_addr, src_addr, chan, msg, test_pwm_output, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_test_pwm_output_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t idx, uint16_t value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN];
    _tta_put_uint8_t(buf, 0, idx);
    _tta_put_uint16_t(buf, 1, value);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TEST_PWM_OUTPUT, buf, TTALINK_MSG_ID_TEST_PWM_OUTPUT_MIN_LEN, TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN, TTALINK_MSG_ID_TEST_PWM_OUTPUT_CRC, nocrc);
#else
    ttalink_test_pwm_output_t packet;
    packet.idx = idx;
    packet.value = value;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TEST_PWM_OUTPUT, (const char *)&packet, TTALINK_MSG_ID_TEST_PWM_OUTPUT_MIN_LEN, TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN, TTALINK_MSG_ID_TEST_PWM_OUTPUT_CRC, nocrc);
#endif
}

/**
 * @brief Send a test_pwm_output message
 * @param chan TTAlink channel to send the message
 *
 * @param idx 通道索引
 * @param value 通道值
 */
static inline void ttalink_test_pwm_output_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t idx, uint16_t value)
{
    _ttalink_test_pwm_output_send(dst_addr, src_addr, chan, idx, value, false);
}

/**
 * @brief Send a test_pwm_output message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param idx 通道索引
 * @param value 通道值
 */
static inline void ttalink_test_pwm_output_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t idx, uint16_t value)
{
    _ttalink_test_pwm_output_send(dst_addr, src_addr, chan, idx, value, true);
}


static inline void _ttalink_test_pwm_output_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_test_pwm_output_t* test_pwm_output, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_test_pwm_output_send_nocrc(dst_addr, src_addr, chan, test_pwm_output->idx, test_pwm_output->value);
    }else{
        ttalink_test_pwm_output_send(dst_addr, src_addr, chan, test_pwm_output->idx, test_pwm_output->value);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TEST_PWM_OUTPUT, (const char *)test_pwm_output, TTALINK_MSG_ID_TEST_PWM_OUTPUT_MIN_LEN, TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN, TTALINK_MSG_ID_TEST_PWM_OUTPUT_CRC, nocrc);
#endif
}

/**
 * @brief Send a test_pwm_output message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_test_pwm_output_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_test_pwm_output_t* test_pwm_output)
{
    _ttalink_test_pwm_output_send_struct(dst_addr, src_addr, chan, test_pwm_output, false);
}

/**
 * @brief Send a test_pwm_output message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_test_pwm_output_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_test_pwm_output_t* test_pwm_output)
{
    _ttalink_test_pwm_output_send_struct(dst_addr, src_addr, chan, test_pwm_output, true);
}

#if TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_test_pwm_output_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t idx, uint16_t value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, idx);
    _tta_put_uint16_t(buf, 1, value);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TEST_PWM_OUTPUT, buf, TTALINK_MSG_ID_TEST_PWM_OUTPUT_MIN_LEN, TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN, TTALINK_MSG_ID_TEST_PWM_OUTPUT_CRC, nocrc);
#else
    ttalink_test_pwm_output_t *packet = (ttalink_test_pwm_output_t *)msgbuf;
    packet->idx = idx;
    packet->value = value;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TEST_PWM_OUTPUT, (const char *)packet, TTALINK_MSG_ID_TEST_PWM_OUTPUT_MIN_LEN, TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN, TTALINK_MSG_ID_TEST_PWM_OUTPUT_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_test_pwm_output_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t idx, uint16_t value)
{
    _ttalink_test_pwm_output_send_buf(dst_addr, src_addr, msgbuf, chan, idx, value, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_test_pwm_output_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t idx, uint16_t value)
{
    _ttalink_test_pwm_output_send_buf(dst_addr, src_addr, msgbuf, chan, idx, value, true);
}
#endif

#endif

// MESSAGE TEST_PWM_OUTPUT UNPACKING


/**
 * @brief Get field idx from test_pwm_output message
 *
 * @return 通道索引
 */
static inline uint8_t ttalink_test_pwm_output_get_idx(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field value from test_pwm_output message
 *
 * @return 通道值
 */
static inline uint16_t ttalink_test_pwm_output_get_value(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  1);
}

/**
 * @brief Decode a test_pwm_output message into a struct
 *
 * @param msg The message to decode
 * @param test_pwm_output C-struct to decode the message contents into
 */
static inline void ttalink_test_pwm_output_decode(const ttalink_message_t* msg, ttalink_test_pwm_output_t* test_pwm_output)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    test_pwm_output->idx = ttalink_test_pwm_output_get_idx(msg);
    test_pwm_output->value = ttalink_test_pwm_output_get_value(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN? msg->len : TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN;
        memset(test_pwm_output, 0, TTALINK_MSG_ID_TEST_PWM_OUTPUT_LEN);
    memcpy(test_pwm_output, _TTA_PAYLOAD(msg), len);
#endif
}
