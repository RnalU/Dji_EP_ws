#ifndef TTALINK_TYPES_H_
#define TTALINK_TYPES_H_

// Visual Studio versions before 2010 don't have stdint.h, so we just error out.
#if (defined _MSC_VER) && (_MSC_VER < 1600)
#error "The C-TTAlink implementation requires Visual Studio 2010 or greater"
#endif

#include <stdbool.h>
#include <stdint.h>

// Macro to define packed structures
#pragma anon_unions
#ifdef __GNUC__
  #define TTAPACKED( __Declaration__ ) __Declaration__ __attribute__((packed))
#else
  //#define TTAPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
  #define TTAPACKED( __Declaration__ ) __Declaration__
#endif

#ifndef TTALINK_MAX_PAYLOAD_LEN
// it is possible to override this, but be careful!
#define TTALINK_MAX_PAYLOAD_LEN 1028 ///< Maximum payload length
#endif

#define TTALINK_CORE_HEADER_LEN 8 ///< Length of core header (of the comm. layer): message length (2 byte) + message sequence (1 byte) + message version major (1 byte) + message dst address (1 byte) + message src address (1 byte) + message type id (2 byte)
#define TTALINK_NUM_HEADER_BYTES (TTALINK_CORE_HEADER_LEN + 2) ///< Length of all header bytes, including core and checksum
#define TTALINK_NUM_CHECKSUM_BYTES 2
#define TTALINK_NUM_NON_PAYLOAD_BYTES (TTALINK_NUM_HEADER_BYTES + TTALINK_NUM_CHECKSUM_BYTES)

#define TTALINK_MAX_PACKET_LEN (TTALINK_MAX_PAYLOAD_LEN + TTALINK_NUM_NON_PAYLOAD_BYTES) ///< Maximum packet length

#define TTALINK_MSG_ID_EXTENDED_MESSAGE 255
#define TTALINK_EXTENDED_HEADER_LEN 14

#if (defined _MSC_VER) || ((defined __APPLE__) && (defined __MACH__)) || (defined __linux__)
  /* full fledged 32bit++ OS */
  #define TTALINK_MAX_EXTENDED_PACKET_LEN 65507
#else
  /* small microcontrollers */
  #define TTALINK_MAX_EXTENDED_PACKET_LEN 2048
#endif

#define TTALINK_MAX_EXTENDED_PAYLOAD_LEN (TTALINK_MAX_EXTENDED_PACKET_LEN - TTALINK_EXTENDED_HEADER_LEN - TTALINK_NUM_NON_PAYLOAD_BYTES)


/**
 * Old-style 4 byte param union
 *
 * This struct is the data format to be used when sending
 * parameters. The parameter should be copied to the native
 * type (without type conversion)
 * and re-instanted on the receiving side using the
 * native type as well.
 */
TTAPACKED(
typedef struct param_union {
	union {
		float param_float;
		int32_t param_int32;
		uint32_t param_uint32;
		int16_t param_int16;
		uint16_t param_uint16;
		int8_t param_int8;
		uint8_t param_uint8;
		uint8_t bytes[4];
	};
	uint8_t type;
}) ttalink_param_union_t;


/**
 * New-style 8 byte param union
 * ttalink_param_union_double_t will be 8 bytes long, and treated as needing 8 byte alignment for the purposes of TTAlink 1.0 field ordering.
 * The ttalink_param_union_double_t will be treated as a little-endian structure.
 *
 * If is_double is 1 then the type is a double, and the remaining 63 bits are the double, with the lowest bit of the mantissa zero.
 * The intention is that by replacing the is_double bit with 0 the type can be directly used as a double (as the is_double bit corresponds to the
 * lowest mantissa bit of a double). If is_double is 0 then ttalink_type gives the type in the union.
 * The ttalink_types.h header will also need to have shifts/masks to define the bit boundaries in the above,
 * as bitfield ordering isn’t consistent between platforms. The above is intended to be for gcc on x86,
 * which should be the same as gcc on little-endian arm. When using shifts/masks the value will be treated as a 64 bit unsigned number,
 * and the bits pulled out using the shifts/masks.
*/
TTAPACKED(
typedef struct param_union_extended {
    union {
    struct {
        uint8_t is_double:1;
        uint8_t ttalink_type:7;
        union {
            char c;
            uint8_t uint8;
            int8_t int8;
            uint16_t uint16;
            int16_t int16;
            uint32_t uint32;
            int32_t int32;
            float f;
            uint8_t align[7];
        };
    };
    uint8_t data[8];
    };
}) ttalink_param_union_double_t;

TTAPACKED(
typedef struct __ttalink_message {
	uint16_t checksum; ///< sent at end of packet
	uint8_t magic;   ///< protocol magic marker 1/2
	uint8_t magic2;  ///< protocol magic marker 2/2
	uint16_t len;     ///< Length of payload                      ***warning: attention endian
	uint8_t seq;     ///< Sequence of packet
	uint8_t ver_major;  /// Version major
	uint8_t dst_addr;   ///< to address
	uint8_t src_addr;   ///< from address
	uint16_t msgid;   ///< ID of message in payload               ***warning: attention endian
	uint64_t payload64[(TTALINK_MAX_PAYLOAD_LEN+TTALINK_NUM_CHECKSUM_BYTES+10)/8];
}) ttalink_message_t;

TTAPACKED(
typedef struct __ttalink_extended_message {
       ttalink_message_t base_msg;
       int32_t extended_payload_len;   ///< Length of extended payload if any
       uint8_t extended_payload[TTALINK_MAX_EXTENDED_PAYLOAD_LEN];
}) ttalink_extended_message_t;

typedef enum {
	TTALINK_TYPE_CHAR     = 0,
	TTALINK_TYPE_UINT8_T  = 1,
	TTALINK_TYPE_INT8_T   = 2,
	TTALINK_TYPE_UINT16_T = 3,
	TTALINK_TYPE_INT16_T  = 4,
	TTALINK_TYPE_UINT32_T = 5,
	TTALINK_TYPE_INT32_T  = 6,
	TTALINK_TYPE_UINT64_T = 7,
	TTALINK_TYPE_INT64_T  = 8,
	TTALINK_TYPE_FLOAT    = 9,
	TTALINK_TYPE_DOUBLE   = 10
} ttalink_message_type_t;

#define TTALINK_MAX_FIELDS 64

typedef struct __ttalink_field_info {
        const char *name;                 // name of this field
        const char *print_format;         // printing format hint, or NULL
        ttalink_message_type_t type;      // type of this field
        unsigned int array_length;        // if non-zero, field is an array
        unsigned int wire_offset;         // offset of each field in the payload
        unsigned int structure_offset;    // offset in a C structure
} ttalink_field_info_t;

// note that in this structure the order of fields is the order
// in the XML file, not necessary the wire order
typedef struct __ttalink_message_info {
	const char *name;                                      // name of the message
	unsigned num_fields;                                   // how many fields in this message
	ttalink_field_info_t fields[TTALINK_MAX_FIELDS];       // field information
} ttalink_message_info_t;

#define _TTA_PAYLOAD(msg)     ((const char *)(&((msg)->payload64[0])))
#define _TTA_PAYLOAD_NON_CONST(msg) ((char *)(&((msg)->payload64[0])))

// checksum is immediately after the payload bytes
#define ttalink_ck_a(msg) *((msg)->len + (uint8_t *)_TTA_PAYLOAD_NON_CONST(msg))
#define ttalink_ck_b(msg) *(((msg)->len+(uint16_t)1) + (uint8_t *)_TTA_PAYLOAD_NON_CONST(msg))

typedef enum {
    TTALINK_PARSE_STATE_UNINIT=0,
    TTALINK_PARSE_STATE_IDLE,
    TTALINK_PARSE_STATE_GOT_STX,
    TTALINK_PARSE_STATE_GOT_STX2,
    TTALINK_PARSE_STATE_GOT_STX2_NOCRC,
    TTALINK_PARSE_STATE_GOT_LENGTH,
    TTALINK_PARSE_STATE_GOT_LENGTH2,
    TTALINK_PARSE_STATE_GOT_SEQ,
    TTALINK_PARSE_STATE_GOT_VERSION_MAJOR,
    TTALINK_PARSE_STATE_GOT_DST_ADDR,
    TTALINK_PARSE_STATE_GOT_SRC_ADDR,
    TTALINK_PARSE_STATE_GOT_MSGID,
    TTALINK_PARSE_STATE_GOT_MSGID2,
    TTALINK_PARSE_STATE_GOT_PAYLOAD,
    TTALINK_PARSE_STATE_GOT_CRC1,
    TTALINK_PARSE_STATE_GOT_BAD_CRC1
} ttalink_parse_state_t; ///< The state machine for the comm parser

typedef enum {
    TTALINK_FRAMING_INCOMPLETE=0,
    TTALINK_FRAMING_OK=1,
    TTALINK_FRAMING_BAD_CRC=2
} ttalink_framing_t;

typedef struct __ttalink_status {
    uint16_t msg_received;               ///< Number of received messages
    uint16_t buffer_overrun;             ///< Number of buffer overruns
    uint8_t parse_error;                ///< Number of parse errors
    ttalink_parse_state_t parse_state;  ///< Parsing state machine
    uint16_t packet_idx;                 ///< Index in current packet
    uint16_t current_rx_seq;             ///< Sequence number of last packet received
    uint16_t current_tx_seq;             ///< Sequence number of last packet sent
    uint16_t packet_rx_success_count;   ///< Received packets
    uint16_t packet_rx_drop_count;      ///< Number of packet drops
    uint8_t use_crc;
} ttalink_status_t;

#define TTALINK_BIG_ENDIAN 0
#define TTALINK_LITTLE_ENDIAN 1

#endif /* TTALINK_TYPES_H_ */
