#pragma once
// MESSAGE TEST_SPI_DATA PACKING

#define TTALINK_MSG_ID_TEST_SPI_DATA 2179

TTAPACKED(
typedef struct __ttalink_test_spi_data_t {
 uint8_t test_payload[512]; /*< .*/
}) ttalink_test_spi_data_t;

#define TTALINK_MSG_ID_TEST_SPI_DATA_LEN 512
#define TTALINK_MSG_ID_TEST_SPI_DATA_MIN_LEN 512
#define TTALINK_MSG_ID_2179_LEN 512
#define TTALINK_MSG_ID_2179_MIN_LEN 512

#define TTALINK_MSG_ID_TEST_SPI_DATA_CRC 223
#define TTALINK_MSG_ID_2179_CRC 223

#define TTALINK_MSG_TEST_SPI_DATA_FIELD_TEST_PAYLOAD_LEN 512

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_TEST_SPI_DATA { \
    2179, \
    "TEST_SPI_DATA", \
    1, \
    {  { "test_payload", NULL, TTALINK_TYPE_UINT8_T, 512, 0, offsetof(ttalink_test_spi_data_t, test_payload) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_TEST_SPI_DATA { \
    "TEST_SPI_DATA", \
    1, \
    {  { "test_payload", NULL, TTALINK_TYPE_UINT8_T, 512, 0, offsetof(ttalink_test_spi_data_t, test_payload) }, \
         } \
}
#endif


static inline uint16_t _ttalink_test_spi_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint8_t *test_payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TEST_SPI_DATA_LEN];

    _tta_put_uint8_t_array(buf, 0, test_payload, 512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TEST_SPI_DATA_LEN);
#else
    ttalink_test_spi_data_t packet;

    tta_array_memcpy(packet.test_payload, test_payload, sizeof(uint8_t)*512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TEST_SPI_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TEST_SPI_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_TEST_SPI_DATA_MIN_LEN, TTALINK_MSG_ID_TEST_SPI_DATA_LEN, TTALINK_MSG_ID_TEST_SPI_DATA_CRC, nocrc);
}

/**
 * @brief Pack a test_spi_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param test_payload .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_test_spi_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint8_t *test_payload)
{
    return _ttalink_test_spi_data_pack(dst_addr, src_addr, msg,  test_payload, false);
}

/**
 * @brief Pack a test_spi_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param test_payload .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_test_spi_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint8_t *test_payload)
{
    return _ttalink_test_spi_data_pack(dst_addr, src_addr, msg,  test_payload, true);
}


static inline uint16_t _ttalink_test_spi_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint8_t *test_payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TEST_SPI_DATA_LEN];

    _tta_put_uint8_t_array(buf, 0, test_payload, 512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TEST_SPI_DATA_LEN);
#else
    ttalink_test_spi_data_t packet;

    tta_array_memcpy(packet.test_payload, test_payload, sizeof(uint8_t)*512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TEST_SPI_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TEST_SPI_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_TEST_SPI_DATA_MIN_LEN, TTALINK_MSG_ID_TEST_SPI_DATA_LEN, TTALINK_MSG_ID_TEST_SPI_DATA_CRC, nocrc);
}

/**
 * @brief Pack a test_spi_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param test_payload .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_test_spi_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint8_t *test_payload)
{
    return _ttalink_test_spi_data_pack_chan(dst_addr, src_addr, chan, msg,  test_payload, false);
}

/**
 * @brief Pack a test_spi_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param test_payload .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_test_spi_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint8_t *test_payload)
{
    return _ttalink_test_spi_data_pack_chan(dst_addr, src_addr, chan, msg,  test_payload, true);
}


static inline uint16_t _ttalink_test_spi_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_test_spi_data_t* test_spi_data, bool nocrc)
{
    if(nocrc){
        return ttalink_test_spi_data_pack_nocrc(dst_addr, src_addr, msg, test_spi_data->test_payload);
    }else{
        return ttalink_test_spi_data_pack(dst_addr, src_addr, msg, test_spi_data->test_payload);
    }
    
}

/**
 * @brief Encode a test_spi_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param test_spi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_test_spi_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_test_spi_data_t* test_spi_data)
{
    return _ttalink_test_spi_data_encode(dst_addr, src_addr, msg, test_spi_data, false);
}

/**
 * @brief Encode a test_spi_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param test_spi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_test_spi_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_test_spi_data_t* test_spi_data)
{
    return _ttalink_test_spi_data_encode(dst_addr, src_addr, msg, test_spi_data, true);
}


static inline uint16_t _ttalink_test_spi_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_test_spi_data_t* test_spi_data, bool nocrc)
{
    if(nocrc){
        return ttalink_test_spi_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, test_spi_data->test_payload);
    }else{
        return ttalink_test_spi_data_pack_chan(dst_addr, src_addr, chan, msg, test_spi_data->test_payload);
    }
}

/**
 * @brief Encode a test_spi_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param test_spi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_test_spi_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_test_spi_data_t* test_spi_data)
{
    return _ttalink_test_spi_data_encode_chan(dst_addr, src_addr, chan, msg, test_spi_data, false);
}

/**
 * @brief Encode a test_spi_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param test_spi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_test_spi_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_test_spi_data_t* test_spi_data)
{
    return _ttalink_test_spi_data_encode_chan(dst_addr, src_addr, chan, msg, test_spi_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_test_spi_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint8_t *test_payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TEST_SPI_DATA_LEN];

    _tta_put_uint8_t_array(buf, 0, test_payload, 512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TEST_SPI_DATA, buf, TTALINK_MSG_ID_TEST_SPI_DATA_MIN_LEN, TTALINK_MSG_ID_TEST_SPI_DATA_LEN, TTALINK_MSG_ID_TEST_SPI_DATA_CRC, nocrc);
#else
    ttalink_test_spi_data_t packet;

    tta_array_memcpy(packet.test_payload, test_payload, sizeof(uint8_t)*512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TEST_SPI_DATA, (const char *)&packet, TTALINK_MSG_ID_TEST_SPI_DATA_MIN_LEN, TTALINK_MSG_ID_TEST_SPI_DATA_LEN, TTALINK_MSG_ID_TEST_SPI_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a test_spi_data message
 * @param chan TTAlink channel to send the message
 *
 * @param test_payload .
 */
static inline void ttalink_test_spi_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint8_t *test_payload)
{
    _ttalink_test_spi_data_send(dst_addr, src_addr, chan, test_payload, false);
}

/**
 * @brief Send a test_spi_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param test_payload .
 */
static inline void ttalink_test_spi_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint8_t *test_payload)
{
    _ttalink_test_spi_data_send(dst_addr, src_addr, chan, test_payload, true);
}


static inline void _ttalink_test_spi_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_test_spi_data_t* test_spi_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_test_spi_data_send_nocrc(dst_addr, src_addr, chan, test_spi_data->test_payload);
    }else{
        ttalink_test_spi_data_send(dst_addr, src_addr, chan, test_spi_data->test_payload);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TEST_SPI_DATA, (const char *)test_spi_data, TTALINK_MSG_ID_TEST_SPI_DATA_MIN_LEN, TTALINK_MSG_ID_TEST_SPI_DATA_LEN, TTALINK_MSG_ID_TEST_SPI_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a test_spi_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_test_spi_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_test_spi_data_t* test_spi_data)
{
    _ttalink_test_spi_data_send_struct(dst_addr, src_addr, chan, test_spi_data, false);
}

/**
 * @brief Send a test_spi_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_test_spi_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_test_spi_data_t* test_spi_data)
{
    _ttalink_test_spi_data_send_struct(dst_addr, src_addr, chan, test_spi_data, true);
}

#if TTALINK_MSG_ID_TEST_SPI_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_test_spi_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint8_t *test_payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _tta_put_uint8_t_array(buf, 0, test_payload, 512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TEST_SPI_DATA, buf, TTALINK_MSG_ID_TEST_SPI_DATA_MIN_LEN, TTALINK_MSG_ID_TEST_SPI_DATA_LEN, TTALINK_MSG_ID_TEST_SPI_DATA_CRC, nocrc);
#else
    ttalink_test_spi_data_t *packet = (ttalink_test_spi_data_t *)msgbuf;

    tta_array_memcpy(packet->test_payload, test_payload, sizeof(uint8_t)*512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TEST_SPI_DATA, (const char *)packet, TTALINK_MSG_ID_TEST_SPI_DATA_MIN_LEN, TTALINK_MSG_ID_TEST_SPI_DATA_LEN, TTALINK_MSG_ID_TEST_SPI_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_test_spi_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint8_t *test_payload)
{
    _ttalink_test_spi_data_send_buf(dst_addr, src_addr, msgbuf, chan, test_payload, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_test_spi_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint8_t *test_payload)
{
    _ttalink_test_spi_data_send_buf(dst_addr, src_addr, msgbuf, chan, test_payload, true);
}
#endif

#endif

// MESSAGE TEST_SPI_DATA UNPACKING


/**
 * @brief Get field test_payload from test_spi_data message
 *
 * @return .
 */
static inline uint16_t ttalink_test_spi_data_get_test_payload(const ttalink_message_t* msg, uint8_t *test_payload)
{
    return _TTA_RETURN_uint8_t_array(msg, test_payload, 512,  0);
}

/**
 * @brief Decode a test_spi_data message into a struct
 *
 * @param msg The message to decode
 * @param test_spi_data C-struct to decode the message contents into
 */
static inline void ttalink_test_spi_data_decode(const ttalink_message_t* msg, ttalink_test_spi_data_t* test_spi_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_test_spi_data_get_test_payload(msg, test_spi_data->test_payload);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_TEST_SPI_DATA_LEN? msg->len : TTALINK_MSG_ID_TEST_SPI_DATA_LEN;
        memset(test_spi_data, 0, TTALINK_MSG_ID_TEST_SPI_DATA_LEN);
    memcpy(test_spi_data, _TTA_PAYLOAD(msg), len);
#endif
}
