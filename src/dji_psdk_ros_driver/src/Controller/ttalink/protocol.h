#ifndef  _TTALINK_PROTOCOL_H_
#define  _TTALINK_PROTOCOL_H_

#include "string.h"
#include "ttalink_types.h"

/* 
   If you want TTAlink on a system that is native big-endian,
   you need to define NATIVE_BIG_ENDIAN
*/
#ifdef NATIVE_BIG_ENDIAN
# define TTALINK_NEED_BYTE_SWAP (TTALINK_ENDIAN == TTALINK_LITTLE_ENDIAN)
#else
# define TTALINK_NEED_BYTE_SWAP (TTALINK_ENDIAN != TTALINK_LITTLE_ENDIAN)
#endif

#ifndef TTALINK_STACK_BUFFER
#define TTALINK_STACK_BUFFER 0
#endif

#ifndef TTALINK_AVOID_GCC_STACK_BUG
# define TTALINK_AVOID_GCC_STACK_BUG defined(__GNUC__)
#endif

#ifndef TTALINK_ASSERT
#define TTALINK_ASSERT(x)
#endif


/* option to provide alternative implementation of ttalink_helpers.h */
#ifdef TTALINK_SEPARATE_HELPERS

    #define TTALINK_HELPER

    /* decls in sync with those in ttalink_helpers.h */
    #ifndef TTALINK_GET_CHANNEL_STATUS
    TTALINK_HELPER ttalink_status_t* ttalink_get_channel_status(uint8_t chan);
    #endif
    TTALINK_HELPER void ttalink_reset_channel_status(uint8_t chan);
    #if TTALINK_CRC_EXTRA
    TTALINK_HELPER uint16_t ttalink_finalize_message_chan(ttalink_message_t* msg, uint8_t dst_addr, uint8_t src_addr,
                                                          uint8_t chan, uint16_t min_length, uint16_t length, uint8_t crc_extra);
    TTALINK_HELPER uint16_t ttalink_finalize_message(ttalink_message_t* msg, uint8_t dst_addr, uint8_t src_addr,
                                                     uint16_t min_length, uint16_t length, uint8_t crc_extra);
    #ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS
    TTALINK_HELPER void _tta_finalize_message_chan_send(ttalink_channel_t chan, uint16_t msgid, const char *packet,
                                                        uint16_t min_length, uint16_t length, uint8_t crc_extra);
    #endif
    #else
    TTALINK_HELPER uint16_t ttalink_finalize_message_chan(ttalink_message_t* msg, uint8_t dst_addr, uint8_t src_addr,
                                                          uint8_t chan, uint16_t length);
    TTALINK_HELPER uint16_t ttalink_finalize_message(ttalink_message_t* msg, uint8_t dst_addr, uint8_t src_addr,
                                                     uint16_t length);
    #ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS
    TTALINK_HELPER void _tta_finalize_message_chan_send(ttalink_channel_t chan, uint16_t msgid, const char *packet, uint16_t length);
    #endif
    #endif // TTALINK_CRC_EXTRA
    TTALINK_HELPER uint16_t ttalink_to_send_buffer(uint8_t *buffer, const ttalink_message_t *msg);
    TTALINK_HELPER void ttalink_start_checksum(ttalink_message_t* msg);
    TTALINK_HELPER void ttalink_update_checksum(ttalink_message_t* msg, uint8_t c);
    TTALINK_HELPER uint8_t ttalink_frame_char_buffer(ttalink_message_t* rxmsg, 
						     ttalink_status_t* status,
						     uint8_t c, 
						     ttalink_message_t* r_message, 
						     ttalink_status_t* r_ttalink_status);
    TTALINK_HELPER uint8_t ttalink_frame_char(uint8_t chan, uint8_t c, ttalink_message_t* r_message, ttalink_status_t* r_ttalink_status);
    TTALINK_HELPER uint8_t ttalink_parse_char(uint8_t chan, uint8_t c, ttalink_message_t* r_message, ttalink_status_t* r_ttalink_status);
    TTALINK_HELPER uint8_t put_bitfield_n_by_index(int32_t b, uint8_t bits, uint8_t packet_index, uint8_t bit_index,
                               uint8_t* r_bit_index, uint8_t* buffer);
    #ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS
    TTALINK_HELPER void _ttalink_send_uart(ttalink_channel_t chan, const char *buf, uint16_t len);
    TTALINK_HELPER void _ttalink_resend_uart(ttalink_channel_t chan, const ttalink_message_t *msg);
    #endif

#else

    #define TTALINK_HELPER static inline
    #include "ttalink_helpers.h"

#endif // TTALINK_SEPARATE_HELPERS

/**
 * @brief Get the required buffer size for this message
 */
static inline uint16_t ttalink_get_send_buffer_length(const ttalink_message_t* msg)
{
	return msg->len + TTALINK_NUM_NON_PAYLOAD_BYTES;
}

#if TTALINK_NEED_BYTE_SWAP
static inline void byte_swap_2(char *dst, const char *src)
{
	dst[0] = src[1];
	dst[1] = src[0];
}
static inline void byte_swap_4(char *dst, const char *src)
{
	dst[0] = src[3];
	dst[1] = src[2];
	dst[2] = src[1];
	dst[3] = src[0];
}
static inline void byte_swap_8(char *dst, const char *src)
{
	dst[0] = src[7];
	dst[1] = src[6];
	dst[2] = src[5];
	dst[3] = src[4];
	dst[4] = src[3];
	dst[5] = src[2];
	dst[6] = src[1];
	dst[7] = src[0];
}
#elif !TTALINK_ALIGNED_FIELDS
static inline void byte_copy_2(char *dst, const char *src)
{
	dst[0] = src[0];
	dst[1] = src[1];
}
static inline void byte_copy_4(char *dst, const char *src)
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
	dst[3] = src[3];
}
static inline void byte_copy_8(char *dst, const char *src)
{
	memcpy(dst, src, 8);
}
#endif

#define _tta_put_uint8_t(buf, wire_offset, b) buf[wire_offset] = (uint8_t)b
#define _tta_put_int8_t(buf, wire_offset, b)  buf[wire_offset] = (int8_t)b
#define _tta_put_char(buf, wire_offset, b)    buf[wire_offset] = b

#if TTALINK_NEED_BYTE_SWAP
#define _tta_put_uint16_t(buf, wire_offset, b) byte_swap_2(&buf[wire_offset], (const char *)&b)
#define _tta_put_int16_t(buf, wire_offset, b)  byte_swap_2(&buf[wire_offset], (const char *)&b)
#define _tta_put_uint32_t(buf, wire_offset, b) byte_swap_4(&buf[wire_offset], (const char *)&b)
#define _tta_put_int32_t(buf, wire_offset, b)  byte_swap_4(&buf[wire_offset], (const char *)&b)
#define _tta_put_uint64_t(buf, wire_offset, b) byte_swap_8(&buf[wire_offset], (const char *)&b)
#define _tta_put_int64_t(buf, wire_offset, b)  byte_swap_8(&buf[wire_offset], (const char *)&b)
#define _tta_put_float(buf, wire_offset, b)    byte_swap_4(&buf[wire_offset], (const char *)&b)
#define _tta_put_double(buf, wire_offset, b)   byte_swap_8(&buf[wire_offset], (const char *)&b)
#elif !TTALINK_ALIGNED_FIELDS
#define _tta_put_uint16_t(buf, wire_offset, b) byte_copy_2(&buf[wire_offset], (const char *)&b)
#define _tta_put_int16_t(buf, wire_offset, b)  byte_copy_2(&buf[wire_offset], (const char *)&b)
#define _tta_put_uint32_t(buf, wire_offset, b) byte_copy_4(&buf[wire_offset], (const char *)&b)
#define _tta_put_int32_t(buf, wire_offset, b)  byte_copy_4(&buf[wire_offset], (const char *)&b)
#define _tta_put_uint64_t(buf, wire_offset, b) byte_copy_8(&buf[wire_offset], (const char *)&b)
#define _tta_put_int64_t(buf, wire_offset, b)  byte_copy_8(&buf[wire_offset], (const char *)&b)
#define _tta_put_float(buf, wire_offset, b)    byte_copy_4(&buf[wire_offset], (const char *)&b)
#define _tta_put_double(buf, wire_offset, b)   byte_copy_8(&buf[wire_offset], (const char *)&b)
#else
#define _tta_put_uint16_t(buf, wire_offset, b) *(uint16_t *)&buf[wire_offset] = b
#define _tta_put_int16_t(buf, wire_offset, b)  *(int16_t *)&buf[wire_offset] = b
#define _tta_put_uint32_t(buf, wire_offset, b) *(uint32_t *)&buf[wire_offset] = b
#define _tta_put_int32_t(buf, wire_offset, b)  *(int32_t *)&buf[wire_offset] = b
#define _tta_put_uint64_t(buf, wire_offset, b) *(uint64_t *)&buf[wire_offset] = b
#define _tta_put_int64_t(buf, wire_offset, b)  *(int64_t *)&buf[wire_offset] = b
#define _tta_put_float(buf, wire_offset, b)    *(float *)&buf[wire_offset] = b
#define _tta_put_double(buf, wire_offset, b)   *(double *)&buf[wire_offset] = b
#endif

/*
  like memcpy(), but if src is NULL, do a memset to zero
*/
static inline void tta_array_memcpy(void *dest, const void *src, size_t n)
{
	if (src == NULL) {
		memset(dest, 0, n);
	} else {
		memcpy(dest, src, n);
	}
}

/*
 * Place a char array into a buffer
 */
static inline void _tta_put_char_array(char *buf, uint16_t wire_offset, const char *b, uint16_t array_length)
{
	tta_array_memcpy(&buf[wire_offset], b, array_length);

}

/*
 * Place a uint8_t array into a buffer
 */
static inline void _tta_put_uint8_t_array(char *buf, uint16_t wire_offset, const uint8_t *b, uint16_t array_length)
{
	tta_array_memcpy(&buf[wire_offset], b, array_length);

}

/*
 * Place a int8_t array into a buffer
 */
static inline void _tta_put_int8_t_array(char *buf, uint16_t wire_offset, const int8_t *b, uint16_t array_length)
{
	tta_array_memcpy(&buf[wire_offset], b, array_length);

}

#if TTALINK_NEED_BYTE_SWAP
#define _TTA_PUT_ARRAY(TYPE, V) \
static inline void _tta_put_ ## TYPE ##_array(char *buf, uint16_t wire_offset, const TYPE *b, uint16_t array_length) \
{ \
	if (b == NULL) { \
		memset(&buf[wire_offset], 0, array_length*sizeof(TYPE)); \
	} else { \
		uint16_t i; \
		for (i=0; i<array_length; i++) { \
			_tta_put_## TYPE (buf, wire_offset+(i*sizeof(TYPE)), b[i]); \
		} \
	} \
}
#else
#define _TTA_PUT_ARRAY(TYPE, V)					\
static inline void _tta_put_ ## TYPE ##_array(char *buf, uint16_t wire_offset, const TYPE *b, uint16_t array_length) \
{ \
	tta_array_memcpy(&buf[wire_offset], b, array_length*sizeof(TYPE)); \
}
#endif

_TTA_PUT_ARRAY(uint16_t, u16)
_TTA_PUT_ARRAY(uint32_t, u32)
_TTA_PUT_ARRAY(uint64_t, u64)
_TTA_PUT_ARRAY(int16_t,  i16)
_TTA_PUT_ARRAY(int32_t,  i32)
_TTA_PUT_ARRAY(int64_t,  i64)
_TTA_PUT_ARRAY(float,    f)
_TTA_PUT_ARRAY(double,   d)

#define _TTA_RETURN_char(msg, wire_offset)             (char)_TTA_PAYLOAD(msg)[wire_offset]
#define _TTA_RETURN_int8_t(msg, wire_offset)   (int8_t)_TTA_PAYLOAD(msg)[wire_offset]
#define _TTA_RETURN_uint8_t(msg, wire_offset) (uint8_t)_TTA_PAYLOAD(msg)[wire_offset]

#if TTALINK_NEED_BYTE_SWAP
#define _TTA_MSG_RETURN_TYPE(TYPE, SIZE) \
static inline TYPE _TTA_RETURN_## TYPE(const ttalink_message_t *msg, uint16_t ofs) \
{ TYPE r; byte_swap_## SIZE((char*)&r, &_TTA_PAYLOAD(msg)[ofs]); return r; }

_TTA_MSG_RETURN_TYPE(uint16_t, 2)
_TTA_MSG_RETURN_TYPE(int16_t,  2)
_TTA_MSG_RETURN_TYPE(uint32_t, 4)
_TTA_MSG_RETURN_TYPE(int32_t,  4)
_TTA_MSG_RETURN_TYPE(uint64_t, 8)
_TTA_MSG_RETURN_TYPE(int64_t,  8)
_TTA_MSG_RETURN_TYPE(float,    4)
_TTA_MSG_RETURN_TYPE(double,   8)

#elif !TTALINK_ALIGNED_FIELDS
#define _TTA_MSG_RETURN_TYPE(TYPE, SIZE) \
static inline TYPE _TTA_RETURN_## TYPE(const ttalink_message_t *msg, uint16_t ofs) \
{ TYPE r; byte_copy_## SIZE((char*)&r, &_TTA_PAYLOAD(msg)[ofs]); return r; }

_TTA_MSG_RETURN_TYPE(uint16_t, 2)
_TTA_MSG_RETURN_TYPE(int16_t,  2)
_TTA_MSG_RETURN_TYPE(uint32_t, 4)
_TTA_MSG_RETURN_TYPE(int32_t,  4)
_TTA_MSG_RETURN_TYPE(uint64_t, 8)
_TTA_MSG_RETURN_TYPE(int64_t,  8)
_TTA_MSG_RETURN_TYPE(float,    4)
_TTA_MSG_RETURN_TYPE(double,   8)
#else // nicely aligned, no swap
#define _TTA_MSG_RETURN_TYPE(TYPE) \
static inline TYPE _TTA_RETURN_## TYPE(const ttalink_message_t *msg, uint16_t ofs) \
{ return *(const TYPE *)(&_TTA_PAYLOAD(msg)[ofs]);}

_TTA_MSG_RETURN_TYPE(uint16_t)
_TTA_MSG_RETURN_TYPE(int16_t)
_TTA_MSG_RETURN_TYPE(uint32_t)
_TTA_MSG_RETURN_TYPE(int32_t)
_TTA_MSG_RETURN_TYPE(uint64_t)
_TTA_MSG_RETURN_TYPE(int64_t)
_TTA_MSG_RETURN_TYPE(float)
_TTA_MSG_RETURN_TYPE(double)
#endif // TTALINK_NEED_BYTE_SWAP

static inline uint16_t _TTA_RETURN_char_array(const ttalink_message_t *msg, char *value, 
						     uint16_t array_length, uint16_t wire_offset)
{
	memcpy(value, &_TTA_PAYLOAD(msg)[wire_offset], array_length);
	return array_length;
}

static inline uint16_t _TTA_RETURN_uint8_t_array(const ttalink_message_t *msg, uint8_t *value, 
							uint16_t array_length, uint16_t wire_offset)
{
	memcpy(value, &_TTA_PAYLOAD(msg)[wire_offset], array_length);
	return array_length;
}

static inline uint16_t _TTA_RETURN_int8_t_array(const ttalink_message_t *msg, int8_t *value, 
						       uint16_t array_length, uint16_t wire_offset)
{
	memcpy(value, &_TTA_PAYLOAD(msg)[wire_offset], array_length);
	return array_length;
}

#if TTALINK_NEED_BYTE_SWAP
#define _TTA_RETURN_ARRAY(TYPE, V) \
static inline uint16_t _TTA_RETURN_## TYPE ##_array(const ttalink_message_t *msg, TYPE *value, \
							 uint16_t array_length, uint16_t wire_offset) \
{ \
	uint16_t i; \
	for (i=0; i<array_length; i++) { \
		value[i] = _TTA_RETURN_## TYPE (msg, wire_offset+(i*sizeof(value[0]))); \
	} \
	return array_length*sizeof(value[0]); \
}
#else
#define _TTA_RETURN_ARRAY(TYPE, V)					\
static inline uint16_t _TTA_RETURN_## TYPE ##_array(const ttalink_message_t *msg, TYPE *value, \
							 uint16_t array_length, uint16_t wire_offset) \
{ \
	memcpy(value, &_TTA_PAYLOAD(msg)[wire_offset], array_length*sizeof(TYPE)); \
	return array_length*sizeof(TYPE); \
}
#endif

_TTA_RETURN_ARRAY(uint16_t, u16)
_TTA_RETURN_ARRAY(uint32_t, u32)
_TTA_RETURN_ARRAY(uint64_t, u64)
_TTA_RETURN_ARRAY(int16_t,  i16)
_TTA_RETURN_ARRAY(int32_t,  i32)
_TTA_RETURN_ARRAY(int64_t,  i64)
_TTA_RETURN_ARRAY(float,    f)
_TTA_RETURN_ARRAY(double,   d)

#endif // _TTALINK_PROTOCOL_H_
