#ifndef  _TTALINK_HELPERS_H_
#define  _TTALINK_HELPERS_H_

#include "string.h"
#include "checksum.h"
#include "common/version.h"
#include "common/channel.h"
#include "ttalink_types.h"
#include "ttalink_conversions.h"

#ifndef TTALINK_HELPER
#define TTALINK_HELPER
#endif

/*
 * Internal function to give access to the channel status for each channel
 */
#ifndef TTALINK_GET_CHANNEL_STATUS
TTALINK_HELPER ttalink_status_t* ttalink_get_channel_status(uint8_t chan)
{
#ifdef TTALINK_EXTERNAL_RX_STATUS
	// No m_ttalink_status array defined in function,
	// has to be defined externally
#else
	static ttalink_status_t m_ttalink_status[TTALINK_COMM_NUM_BUFFERS];
#endif
	return &m_ttalink_status[chan];
}
#endif

/*
 * Internal function to give access to the channel buffer for each channel
 */
#ifndef TTALINK_GET_CHANNEL_BUFFER
TTALINK_HELPER ttalink_message_t* ttalink_get_channel_buffer(uint8_t chan)
{
	
#ifdef TTALINK_EXTERNAL_RX_BUFFER
	// No m_ttalink_buffer array defined in function,
	// has to be defined externally
#else
	static ttalink_message_t m_ttalink_buffer[TTALINK_COMM_NUM_BUFFERS];
#endif
	return &m_ttalink_buffer[chan];
}
#endif

/**
 * @brief Reset the status of a channel.
 */
TTALINK_HELPER void ttalink_reset_channel_status(uint8_t chan)
{
	ttalink_status_t *status = ttalink_get_channel_status(chan);
	status->parse_state = TTALINK_PARSE_STATE_IDLE;
}


/**
 * @brief Finalize a TTAlink message with channel assignment
 *
 * This function calculates the checksum and sets length and aircraft id correctly.
 * It assumes that the message id and the payload are already correctly set. This function
 * can also be used if the message header has already been written before (as in ttalink_msg_xxx_pack
 * instead of ttalink_msg_xxx_pack_headerless), it just introduces little extra overhead.
 *
 * @param msg Message to finalize
 * @param dst_addr Id of the sending (this) system, 1-127
 * @param length Message length
 */
#if TTALINK_CRC_EXTRA
TTALINK_HELPER uint16_t ttalink_finalize_message_chan(ttalink_message_t* msg, uint8_t dst_addr, uint8_t src_addr, 
						      uint8_t chan, uint16_t min_length, uint16_t length, uint8_t crc_extra, bool nocrc)
#else
TTALINK_HELPER uint16_t ttalink_finalize_message_chan(ttalink_message_t* msg, uint8_t dst_addr, uint8_t src_addr, 
						      uint8_t chan, uint16_t length, bool nocrc)
#endif
{
	// This is only used for the v2 protocol and we silence it here
	(void)min_length;
	// This code part is the same for all messages;
	msg->magic = TTALINK_STX;
	if(nocrc){
		msg->magic2 = TTALINK_STX2_NOCRC;
	}else{
		msg->magic2 = TTALINK_STX2;
	}
	msg->len = length;
	msg->ver_major = TTALINK_VERSION_MAJOR;
	msg->dst_addr = dst_addr;
	msg->src_addr = src_addr;
	// One sequence number per channel
	msg->seq = ttalink_get_channel_status(chan)->current_tx_seq;
	ttalink_get_channel_status(chan)->current_tx_seq = ttalink_get_channel_status(chan)->current_tx_seq+1;
	if(nocrc){
		msg->checksum = 0;
	}else{
		msg->checksum = crc_calculate(((const uint8_t*)(msg)) + 4, TTALINK_CORE_HEADER_LEN);
		crc_accumulate_buffer(&msg->checksum, _TTA_PAYLOAD(msg), msg->len);
//#if TTALINK_CRC_EXTRA
//		crc_accumulate(crc_extra, &msg->checksum);
//#endif
	}

	ttalink_ck_a(msg) = (uint8_t)(msg->checksum & 0xFF);
	ttalink_ck_b(msg) = (uint8_t)(msg->checksum >> 8);

	return length + TTALINK_NUM_NON_PAYLOAD_BYTES;
}

/**
 * @brief Finalize a TTAlink message with TTALINK_CHANNEL_DEFAULT as default channel
 */
#if TTALINK_CRC_EXTRA
TTALINK_HELPER uint16_t ttalink_finalize_message(ttalink_message_t* msg, uint8_t dst_addr, uint8_t src_addr, 
						 uint16_t min_length, uint16_t length, uint8_t crc_extra, bool nocrc)
{
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, TTALINK_CHANNEL_DEFAULT, min_length, length, crc_extra, nocrc);
}
#else
TTALINK_HELPER uint16_t ttalink_finalize_message(ttalink_message_t* msg, uint8_t dst_addr, uint8_t src_addr, 
						 uint16_t length, bool nocrc)
{
	return ttalink_finalize_message_chan(msg, dst_addr, src_addr, TTALINK_CHANNEL_DEFAULT, length, nocrc);
}
#endif

#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS
TTALINK_HELPER void _ttalink_send_uart(ttalink_channel_t chan, const char *buf, uint16_t len);

#ifdef TTALINK_START_UART_SEND
extern void TTALINK_START_UART_SEND(ttalink_channel_t chan, uint16_t len);
#endif

#ifdef TTALINK_END_UART_SEND
extern void TTALINK_END_UART_SEND(ttalink_channel_t chan, uint16_t len);
#endif

/**
 * @brief Finalize a TTAlink message with channel assignment and send
 */
#if TTALINK_CRC_EXTRA
TTALINK_HELPER void _tta_finalize_message_chan_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t msgid, const char *packet, 
						    uint16_t min_length, uint16_t length, uint8_t crc_extra, bool nocrc)
#else
TTALINK_HELPER void _tta_finalize_message_chan_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t msgid, const char *packet, uint16_t length, bool nocrc)
#endif
{
	uint16_t checksum;
	uint8_t buf[TTALINK_NUM_HEADER_BYTES];
	uint8_t ck[2];
	ttalink_status_t *status = ttalink_get_channel_status(chan);
	buf[0] = TTALINK_STX;
	if(nocrc){
		buf[1] = TTALINK_STX2_NOCRC;
	}else{
		buf[1] = TTALINK_STX2;
	}
	buf[2] = (uint8_t)(length & 0xFF);
	buf[3] = (uint8_t)(length >> 8);
	buf[4] = status->current_tx_seq;
	buf[5] = TTALINK_VERSION_MAJOR;
	buf[6] = dst_addr;
	buf[7] = src_addr;
	buf[8] = (uint8_t)(msgid & 0xFF);
	buf[9] = (uint8_t)(msgid >> 8);
	status->current_tx_seq++;
	if(nocrc){
		checksum = 0;
	}else{
		checksum = crc_calculate((const uint8_t*)&buf[2], TTALINK_CORE_HEADER_LEN);
		crc_accumulate_buffer(&checksum, packet, length);
#if TTALINK_CRC_EXTRA
//		crc_accumulate(crc_extra, &checksum);
#endif
	}

	ck[0] = (uint8_t)(checksum & 0xFF);
	ck[1] = (uint8_t)(checksum >> 8);

#ifdef TTALINK_START_UART_SEND
	TTALINK_START_UART_SEND(chan, TTALINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)length);
#endif
	_ttalink_send_uart(chan, (const char *)buf, TTALINK_NUM_HEADER_BYTES);
	_ttalink_send_uart(chan, packet, length);
	_ttalink_send_uart(chan, (const char *)ck, 2);
#ifdef TTALINK_END_UART_SEND
	TTALINK_END_UART_SEND(chan, TTALINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)length);
#endif
}

/**
 * @brief re-send a message over a uart channel
 * this is more stack efficient than re-marshalling the message
 */
TTALINK_HELPER void _ttalink_resend_uart(ttalink_channel_t chan, const ttalink_message_t *msg)
{
	uint8_t ck[2];

	ck[0] = (uint8_t)(msg->checksum & 0xFF);
	ck[1] = (uint8_t)(msg->checksum >> 8);
	// XXX use the right sequence here

#ifdef TTALINK_START_UART_SEND
	TTALINK_START_UART_SEND(chan, TTALINK_NUM_NON_PAYLOAD_BYTES + msg->len);
#endif
	_ttalink_send_uart(chan, (const char *)&msg->magic, TTALINK_NUM_HEADER_BYTES);
	_ttalink_send_uart(chan, _TTA_PAYLOAD(msg), msg->len);
	_ttalink_send_uart(chan, (const char *)ck, 2);
#ifdef TTALINK_END_UART_SEND
	TTALINK_END_UART_SEND(chan, TTALINK_NUM_NON_PAYLOAD_BYTES + msg->len);
#endif
}
#endif // TTALINK_USE_CONVENIENCE_FUNCTIONS

/**
 * @brief Pack a message to send it over a serial byte stream
 */
TTALINK_HELPER uint16_t ttalink_to_send_buffer(uint8_t *buffer, const ttalink_message_t *msg)
{
	memcpy(buffer, (const uint8_t *)&msg->magic, TTALINK_NUM_HEADER_BYTES);
	memcpy(buffer + TTALINK_NUM_HEADER_BYTES, (const uint8_t *)&msg->payload64, (uint16_t)msg->len);

	uint8_t *ck = buffer + (TTALINK_NUM_HEADER_BYTES + (uint16_t)msg->len);

	ck[0] = (uint8_t)(msg->checksum & 0xFF);
	ck[1] = (uint8_t)(msg->checksum >> 8);

	return TTALINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)msg->len;
}

union __ttalink_bitfield {
	uint8_t uint8;
	int8_t int8;
	uint16_t uint16;
	int16_t int16;
	uint32_t uint32;
	int32_t int32;
};


TTALINK_HELPER void ttalink_start_checksum(ttalink_message_t* msg)
{
	crc_init(&msg->checksum);
}

TTALINK_HELPER void ttalink_update_checksum(ttalink_message_t* msg, uint8_t c)
{
	crc_accumulate(c, &msg->checksum);
}

/**
 * This is a varient of ttalink_frame_char() but with caller supplied
 * parsing buffers. It is useful when you want to create a TTAlink
 * parser in a library that doesn't use any global variables
 *
 * @param rxmsg    parsing message buffer
 * @param status   parsing starus buffer 
 * @param c        The char to parse
 *
 * @param returnMsg NULL if no message could be decoded, the message data else
 * @param returnStats if a message was decoded, this is filled with the channel's stats
 * @return 0 if no message could be decoded, 1 on good message and CRC, 2 on bad CRC
 *
 * A typical use scenario of this function call is:
 *
 * @code
 * #include <ttalink.h>
 *
 * ttalink_message_t msg;
 * int chan = 0;
 *
 *
 * while(serial.bytesAvailable > 0)
 * {
 *   uint8_t byte = serial.getNextByte();
 *   if (ttalink_frame_char(chan, byte, &msg) != TTALINK_FRAMING_INCOMPLETE)
 *     {
 *     printf("Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid, msg.seq, msg.src_addr, msg.dst_addr);
 *     }
 * }
 *
 *
 * @endcode
 */
TTALINK_HELPER uint8_t ttalink_frame_char_buffer(ttalink_message_t* rxmsg, 
                                                 ttalink_status_t* status,
                                                 uint8_t c, 
                                                 ttalink_message_t* r_message, 
                                                 ttalink_status_t* r_ttalink_status)
{
        /*
	  default message crc function. You can override this per-system to
	  put this data in a different memory segment
	*/
#if TTALINK_CRC_EXTRA
//#ifndef TTALINK_MESSAGE_CRC
//	static const uint8_t ttalink_message_crcs[256] = TTALINK_MESSAGE_CRCS;
//#define TTALINK_MESSAGE_CRC(msgid) ttalink_message_crcs[msgid]
//#endif
#endif

	/* Enable this option to check the length of each message.
	   This allows invalid messages to be caught much sooner. Use if the transmission
	   medium is prone to missing (or extra) characters (e.g. a radio that fades in
	   and out). Only use if the channel will only contain messages types listed in
	   the headers.
	*/
#ifdef TTALINK_CHECK_MESSAGE_LENGTH
#ifndef TTALINK_MESSAGE_LENGTH
	static const uint8_t ttalink_message_lengths[256] = TTALINK_MESSAGE_LENGTHS;
#define TTALINK_MESSAGE_LENGTH(msgid) ttalink_message_lengths[msgid]
#endif
#endif

	int bufferIndex = 0;

	status->msg_received = TTALINK_FRAMING_INCOMPLETE;
	switch (status->parse_state)
	{
	case TTALINK_PARSE_STATE_UNINIT:
	case TTALINK_PARSE_STATE_IDLE:
		if (c == TTALINK_STX)
		{
			status->parse_state = TTALINK_PARSE_STATE_GOT_STX;
			rxmsg->len = 0;
			rxmsg->magic = c;
		}
		break;
	case TTALINK_PARSE_STATE_GOT_STX:
		if (c == TTALINK_STX2)
		{
			status->parse_state = TTALINK_PARSE_STATE_GOT_STX2;
			status->use_crc = 1;
			rxmsg->magic2 = c;
			ttalink_start_checksum(rxmsg);
		}else{
			status->buffer_overrun++;
			status->parse_error++;
			status->msg_received = 0;
			status->parse_state = TTALINK_PARSE_STATE_IDLE;
		}
		break;
	case TTALINK_PARSE_STATE_GOT_STX2:
		rxmsg->len = c;
		if(status->use_crc){
			ttalink_update_checksum(rxmsg, c);
		}
		status->parse_state = TTALINK_PARSE_STATE_GOT_LENGTH;
		break;

	case TTALINK_PARSE_STATE_GOT_LENGTH:
		rxmsg->len |= ((uint16_t)c) << 8;
		if (status->msg_received || rxmsg->len > TTALINK_MAX_PAYLOAD_LEN){
			status->buffer_overrun++;
			status->parse_error++;
			status->parse_state = TTALINK_PARSE_STATE_IDLE;
			status->msg_received = TTALINK_FRAMING_INCOMPLETE;
		}else{
			// NOT counting STX, LENGTH, SEQ, DST_ADDR, SRC_ADDR, MSGID, CRC1 and CRC2
			status->packet_idx = 0;
			if(status->use_crc){
				ttalink_update_checksum(rxmsg, c);
			}
			status->parse_state = TTALINK_PARSE_STATE_GOT_LENGTH2;
		}
		break;
	case TTALINK_PARSE_STATE_GOT_LENGTH2:
		rxmsg->seq = c;
		if(status->use_crc){
			ttalink_update_checksum(rxmsg, c);
		}
		status->parse_state = TTALINK_PARSE_STATE_GOT_SEQ;
		break;

	case TTALINK_PARSE_STATE_GOT_SEQ:
		rxmsg->ver_major = c;
		if(status->use_crc){
			ttalink_update_checksum(rxmsg, c);
		}
		status->parse_state = TTALINK_PARSE_STATE_GOT_VERSION_MAJOR;
		break;

	case TTALINK_PARSE_STATE_GOT_VERSION_MAJOR:
		rxmsg->dst_addr = c;
		if(status->use_crc){
			ttalink_update_checksum(rxmsg, c);
		}
		status->parse_state = TTALINK_PARSE_STATE_GOT_DST_ADDR;
		break;

	case TTALINK_PARSE_STATE_GOT_DST_ADDR:
		rxmsg->src_addr = c;
		if(status->use_crc){
			ttalink_update_checksum(rxmsg, c);
		}
		status->parse_state = TTALINK_PARSE_STATE_GOT_SRC_ADDR;
		break;

	case TTALINK_PARSE_STATE_GOT_SRC_ADDR:
		rxmsg->msgid = c;
		if(status->use_crc){
			ttalink_update_checksum(rxmsg, c);
		}
		status->parse_state = TTALINK_PARSE_STATE_GOT_MSGID;
		break;
	case TTALINK_PARSE_STATE_GOT_MSGID:
		rxmsg->msgid |= ((uint16_t)c) << 8;
#ifdef TTALINK_CHECK_MESSAGE_LENGTH
	        if ((rxmsg->len != TTALINK_MESSAGE_LENGTH(rxmsg->msgid))
		{
			status->parse_error++;
			status->parse_state = TTALINK_PARSE_STATE_IDLE;
			break;
	    }
#endif
		if(status->use_crc){
			ttalink_update_checksum(rxmsg, c);
		}
		if (rxmsg->len == 0)
		{
			status->parse_state = TTALINK_PARSE_STATE_GOT_PAYLOAD;
		}
		else
		{
			status->parse_state = TTALINK_PARSE_STATE_GOT_MSGID2;
		}
		break;

	case TTALINK_PARSE_STATE_GOT_MSGID2:
		_TTA_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx++] = (char)c;
		if(status->use_crc){
			ttalink_update_checksum(rxmsg, c);
		}
		if (status->packet_idx == rxmsg->len)
		{
			status->parse_state = TTALINK_PARSE_STATE_GOT_PAYLOAD;
		}
		break;

	case TTALINK_PARSE_STATE_GOT_PAYLOAD:
//#if TTALINK_CRC_EXTRA
//		if(status->use_crc){
//			ttalink_update_checksum(rxmsg, TTALINK_MESSAGE_CRC(rxmsg->msgid));
//		}
//#endif
		if(status->use_crc){
			if (c != (rxmsg->checksum & 0xFF)) {
				status->parse_state = TTALINK_PARSE_STATE_GOT_BAD_CRC1;
			} else {
				status->parse_state = TTALINK_PARSE_STATE_GOT_CRC1;
			}
		}else{
			status->parse_state = TTALINK_PARSE_STATE_GOT_CRC1;
		}
        _TTA_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx] = (char)c;
		break;

	case TTALINK_PARSE_STATE_GOT_CRC1:
	case TTALINK_PARSE_STATE_GOT_BAD_CRC1:
		if(status->use_crc){
			if (status->parse_state == TTALINK_PARSE_STATE_GOT_BAD_CRC1 || c != (rxmsg->checksum >> 8)) {
				// got a bad CRC message
				status->msg_received = TTALINK_FRAMING_BAD_CRC;
			} else {
				// Successfully got message
				status->msg_received = TTALINK_FRAMING_OK;
			}
		}else{
			// Successfully got message
				status->msg_received = TTALINK_FRAMING_OK;
		}
        status->parse_state = TTALINK_PARSE_STATE_IDLE;
        _TTA_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx+1] = (char)c;
        memcpy(r_message, rxmsg, sizeof(ttalink_message_t));
		break;
	}

	bufferIndex++;
	// If a message has been sucessfully decoded, check index
	if (status->msg_received == TTALINK_FRAMING_OK)
	{
		//while(status->current_seq != rxmsg->seq)
		//{
		//	status->packet_rx_drop_count++;
		//               status->current_seq++;
		//}
		status->current_rx_seq = rxmsg->seq;
		// Initial condition: If no packet has been received so far, drop count is undefined
		if (status->packet_rx_success_count == 0) status->packet_rx_drop_count = 0;
		// Count this packet as received
		status->packet_rx_success_count++;
	}

	r_message->len = rxmsg->len; // Provide visibility on how far we are into current msg
	r_ttalink_status->parse_state = status->parse_state;
	r_ttalink_status->packet_idx = status->packet_idx;
	r_ttalink_status->current_rx_seq = status->current_rx_seq+1;
	r_ttalink_status->packet_rx_success_count = status->packet_rx_success_count;
	r_ttalink_status->packet_rx_drop_count = status->parse_error;
	status->parse_error = 0;

	if (status->msg_received == TTALINK_FRAMING_BAD_CRC) {
		/*
		  the CRC came out wrong. We now need to overwrite the
		  msg CRC with the one on the wire so that if the
		  caller decides to forward the message anyway that
		  ttalink_to_send_buffer() won't overwrite the
		  checksum
		 */
		r_message->checksum = _TTA_PAYLOAD(rxmsg)[status->packet_idx] | (_TTA_PAYLOAD(rxmsg)[status->packet_idx+1]<<8);
	}

	return status->msg_received;
}

/**
 * This is a convenience function which handles the complete TTAlink parsing.
 * the function will parse one byte at a time and return the complete packet once
 * it could be successfully decoded. This function will return 0, 1 or
 * 2 (TTALINK_FRAMING_INCOMPLETE, TTALINK_FRAMING_OK or TTALINK_FRAMING_BAD_CRC)
 *
 * Messages are parsed into an internal buffer (one for each channel). When a complete
 * message is received it is copies into *returnMsg and the channel's status is
 * copied into *returnStats.
 *
 * @param chan     ID of the current channel. This allows to parse different channels with this function.
 *                 a channel is not a physical message channel like a serial port, but a logic partition of
 *                 the communication streams in this case. COMM_NB is the limit for the number of channels
 *                 on MCU (e.g. ARM7), while COMM_NB_HIGH is the limit for the number of channels in Linux/Windows
 * @param c        The char to parse
 *
 * @param returnMsg NULL if no message could be decoded, the message data else
 * @param returnStats if a message was decoded, this is filled with the channel's stats
 * @return 0 if no message could be decoded, 1 on good message and CRC, 2 on bad CRC
 *
 * A typical use scenario of this function call is:
 *
 * @code
 * #include <ttalink.h>
 *
 * ttalink_message_t msg;
 * int chan = 0;
 *
 *
 * while(serial.bytesAvailable > 0)
 * {
 *   uint8_t byte = serial.getNextByte();
 *   if (ttalink_frame_char(chan, byte, &msg) != TTALINK_FRAMING_INCOMPLETE)
 *     {
 *     printf("Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid, msg.seq, msg.src_addr, msg.dst_addr);
 *     }
 * }
 *
 *
 * @endcode
 */
TTALINK_HELPER uint8_t ttalink_frame_char(uint8_t chan, uint8_t c, ttalink_message_t* r_message, ttalink_status_t* r_ttalink_status)
{
	return ttalink_frame_char_buffer(ttalink_get_channel_buffer(chan),
					 ttalink_get_channel_status(chan),
					 c,
					 r_message,
					 r_ttalink_status);
}


/**
 * This is a convenience function which handles the complete TTAlink parsing.
 * the function will parse one byte at a time and return the complete packet once
 * it could be successfully decoded. This function will return 0 or 1.
 *
 * Messages are parsed into an internal buffer (one for each channel). When a complete
 * message is received it is copies into *returnMsg and the channel's status is
 * copied into *returnStats.
 *
 * @param chan     ID of the current channel. This allows to parse different channels with this function.
 *                 a channel is not a physical message channel like a serial port, but a logic partition of
 *                 the communication streams in this case. COMM_NB is the limit for the number of channels
 *                 on MCU (e.g. ARM7), while COMM_NB_HIGH is the limit for the number of channels in Linux/Windows
 * @param c        The char to parse
 *
 * @param returnMsg NULL if no message could be decoded, the message data else
 * @param returnStats if a message was decoded, this is filled with the channel's stats
 * @return 0 if no message could be decoded or bad CRC, 1 on good message and CRC
 *
 * A typical use scenario of this function call is:
 *
 * @code
 * #include <ttalink.h>
 *
 * ttalink_message_t msg;
 * int chan = 0;
 *
 *
 * while(serial.bytesAvailable > 0)
 * {
 *   uint8_t byte = serial.getNextByte();
 *   if (ttalink_parse_char(chan, byte, &msg))
 *     {
 *     printf("Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid, msg.seq, msg.src_addr, msg.dst_addr);
 *     }
 * }
 *
 *
 * @endcode
 */
TTALINK_HELPER uint8_t ttalink_parse_char(uint8_t chan, uint8_t c, ttalink_message_t* r_message, ttalink_status_t* r_ttalink_status)
{
    uint8_t msg_received = ttalink_frame_char(chan, c, r_message, r_ttalink_status);
    if (msg_received == TTALINK_FRAMING_BAD_CRC) {
	    // we got a bad CRC. Treat as a parse failure
	    ttalink_message_t* rxmsg = ttalink_get_channel_buffer(chan);
	    ttalink_status_t* status = ttalink_get_channel_status(chan);
	    status->parse_error++;
	    status->msg_received = TTALINK_FRAMING_INCOMPLETE;
	    status->parse_state = TTALINK_PARSE_STATE_IDLE;
	    if (c == TTALINK_STX)
	    {
		    status->parse_state = TTALINK_PARSE_STATE_GOT_STX;
		    rxmsg->len = 0;
		    ttalink_start_checksum(rxmsg);
	    }
	    return 0;
    }
    return msg_received;
}

/**
 * @brief Put a bitfield of length 1-32 bit into the buffer
 *
 * @param b the value to add, will be encoded in the bitfield
 * @param bits number of bits to use to encode b, e.g. 1 for boolean, 2, 3, etc.
 * @param packet_index the position in the packet (the index of the first byte to use)
 * @param bit_index the position in the byte (the index of the first bit to use)
 * @param buffer packet buffer to write into
 * @return new position of the last used byte in the buffer
 */
TTALINK_HELPER uint8_t put_bitfield_n_by_index(int32_t b, uint8_t bits, uint8_t packet_index, uint8_t bit_index, uint8_t* r_bit_index, uint8_t* buffer)
{
	uint16_t bits_remain = bits;
	// Transform number into network order
	int32_t v;
	uint8_t i_bit_index, i_byte_index, curr_bits_n;
#if TTALINK_NEED_BYTE_SWAP
	union {
		int32_t i;
		uint8_t b[4];
	} bin, bout;
	bin.i = b;
	bout.b[0] = bin.b[3];
	bout.b[1] = bin.b[2];
	bout.b[2] = bin.b[1];
	bout.b[3] = bin.b[0];
	v = bout.i;
#else
	v = b;
#endif

	// buffer in
	// 01100000 01000000 00000000 11110001
	// buffer out
	// 11110001 00000000 01000000 01100000

	// Existing partly filled byte (four free slots)
	// 0111xxxx

	// Mask n free bits
	// 00001111 = 2^0 + 2^1 + 2^2 + 2^3 = 2^n - 1
	// = ((uint32_t)(1 << n)) - 1; // = 2^n - 1

	// Shift n bits into the right position
	// out = in >> n;

	// Mask and shift bytes
	i_bit_index = bit_index;
	i_byte_index = packet_index;
	if (bit_index > 0)
	{
		// If bits were available at start, they were available
		// in the byte before the current index
		i_byte_index--;
	}

	// While bits have not been packed yet
	while (bits_remain > 0)
	{
		// Bits still have to be packed
		// there can be more than 8 bits, so
		// we might have to pack them into more than one byte

		// First pack everything we can into the current 'open' byte
		//curr_bits_n = bits_remain << 3; // Equals  bits_remain mod 8
		//FIXME
		if (bits_remain <= (uint8_t)(8 - i_bit_index))
		{
			// Enough space
			curr_bits_n = (uint8_t)bits_remain;
		}
		else
		{
			curr_bits_n = (8 - i_bit_index);
		}
		
		// Pack these n bits into the current byte
		// Mask out whatever was at that position with ones (xxx11111)
		buffer[i_byte_index] &= (0xFF >> (8 - curr_bits_n));
		// Put content to this position, by masking out the non-used part
		buffer[i_byte_index] |= ((0x00 << curr_bits_n) & v);
		
		// Increment the bit index
		i_bit_index += curr_bits_n;

		// Now proceed to the next byte, if necessary
		bits_remain -= curr_bits_n;
		if (bits_remain > 0)
		{
			// Offer another 8 bits / one byte
			i_byte_index++;
			i_bit_index = 0;
		}
	}
	
	*r_bit_index = i_bit_index;
	// If a partly filled byte is present, mark this as consumed
	if (i_bit_index != 7) i_byte_index++;
	return i_byte_index - packet_index;
}

#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

// To make TTAlink work on your MCU, define comm_send_ch() if you wish
// to send 1 byte at a time, or TTALINK_SEND_UART_BYTES() to send a
// whole packet at a time


#ifdef TTALINK_SEND_UART_BYTES
/*
void TTALINK_SEND_UART_BYTES(ttalink_channel_t chan, const char *buf, uint16_t len)
{
	printf("TTALINK_SEND_UART_BYTES chan=%d, ", chan);
	for(int i=0; i<len; i++){
		comm_buf[comm_count++] = buf[i];
		printf("%02x ", buf[i] & 0xff);
	}
	printf("\n");
}
 */
extern void TTALINK_SEND_UART_BYTES(ttalink_channel_t chan, const char *buf, uint16_t len);
#else
/*
#include "ttalink_types.h"
void comm_send_ch(ttalink_channel_t chan, uint8_t ch)
{
    if (chan == TTALINK_CHANNEL_DEFAULT)
    {
        uart0_transmit(ch);
    }
}
 */
extern void comm_send_ch(ttalink_channel_t chan, uint8_t ch);
#endif
TTALINK_HELPER void _ttalink_send_uart(ttalink_channel_t chan, const char *buf, uint16_t len)
{
#ifdef TTALINK_SEND_UART_BYTES
	/* this is the more efficient approach, if the platform
	   defines it */
	TTALINK_SEND_UART_BYTES(chan, buf, len);
#else
	/* fallback to one byte at a time */
	uint16_t i;
	for (i = 0; i < len; i++) {
		comm_send_ch(chan, (uint8_t)buf[i]);
	}
#endif
}
#endif // TTALINK_USE_CONVENIENCE_FUNCTIONS

#endif /* _TTALINK_HELPERS_H_ */
