/**
  ******************************************************************************
  * @file           : ubx.h
  * @author         : Jack Bauer
  * @version        : Pre-production v0.0
  * @date           : Jan 18, 2026
  * @brief          : Definition of NEO-M8U UBX serial communication protocol
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
**/

#ifndef INC_UBX_H_
#define INC_UBX_H_

#include <stdint.h>
#include "main.h"

#define SYNC_CHAR_1 0xB5
#define SYNC_CHAR_2 0x62

#define FRAME_PREAMBLE { .preamble = (SYNC_CHAR_2 << 1 | SYNC_CHAR_1) }


#define initialize_ubx_frame(FRAME, TYPE, ...) 								  		\
							  _Generic((TYPE), 						  			  	\
							  byte *		: _initialize_ubx_frame_from_array,		\
							  byte 			: _initialize_ubx_frame_from_fields, 	\
							  word			: _initialize_ubx_frame_from_fields,	\
							  uint32_t		: _initialize_ubx_frame_from_fields,	\
							  int			: _initialize_ubx_frame_from_fields 	\
							  )(FRAME, TYPE, __VA_ARGS__)


 // Defined in the receiver description section 32.6 UBX Class IDs
 enum
 {
    NAV = 0x01,
    RXM = 0x02,
    INF = 0x04,
    ACK = 0x05,
    CFG = 0x06,
    UPD = 0x09,
    MON = 0x0A,
    AID = 0x0B,
    TIM = 0x0D,
    ESF = 0x10,
    MGA = 0x13,
    LOG = 0x21,
    SEC = 0x27,
    HNR = 0x28
 } typedef UBXClass;


 enum
 {
	UBX_OK = 0,
    UBX_ERROR_SYNC = 1,
    UBX_ERROR_CLASS = 2,
    UBX_ERROR_ID = 3,
    UBX_ERROR_LENGTH = 4,
	UBX_ERROR_PAYLOAD = 5,
    UBX_ERROR_CHECKSUM = 6
 }typedef UBXStatus;


 // There are hundreds of different IDs that can be requested, and they typically overlap.
 // I wrote down a few here for testing and future implementation.
 // 32.8.xx ACK messages
 // 32.10.xx Messages for configuring the receiver
 // 32.13.xx Informational messages in ASCII format
 // 32.14.xx Log messages
 // 32.17.xx Navigation results
 // 32.19.xx Security details
 // Additional note: The checksums are pre-computed 0-length, 0-payload fetcher checksums using a MATLAB script provided in the repo.
 //                  The fetcher algorithm itself is described in the receiver description section 32.4 UBX Checksum


 /*
 * 	                USB Frame Structure: As defined in Receiver Description 32.1
    [ Preamble ]    [SYNC CHAR 1] | [SYNC CHAR 1] | [CLASS] | [ID] [LENGTH] | [PAYLOAD] [CHEHCKSUM_A] | [CHECKSUM_B]
 */
static const byte ubx_tx_poll_id[8]  = { SYNC_CHAR_1, SYNC_CHAR_2, SEC, 0x03, 0x00, 0x00, 0x2A, 0xA5 };
static const byte ubx_tx_poll_pvt[8] = { SYNC_CHAR_1, SYNC_CHAR_2, NAV, 0x07, 0x00, 0x00, 0x08, 0x19 };
static const byte ubx_tx_poll_pvt_hnr[8] = { SYNC_CHAR_1, SYNC_CHAR_2, HNR, 0x00, 0x00, 0x00, 0x28, 0xA0 };
static const byte ubx_tx_poll_att[8] = { SYNC_CHAR_1, SYNC_CHAR_2, NAV, 0x05, 0x00, 0x00, 0x06, 0x13 };

 struct
 {
    word preamble;
    byte class;
    byte id;
    word length;
    byte *payload;
    byte checksum_a;
    byte checksum_b;
 }typedef UBXFrame_Typedef; // Needs renaming


 // Global rx buffer. Must be global since this buffer acquires data under an interrupt!
extern byte UART4_rxBuffer[256];


 // TODO: Figure out how to make these static without gcc complaining to me.
UBXStatus _initialize_ubx_frame_from_array(UBXFrame_Typedef *ubx_frame, byte *ubx_frame_array, ...);
UBXStatus _initialize_ubx_frame_from_fields(UBXFrame_Typedef *ubx_frame,
 								  	  	  	                     byte class, byte id,
											                     word length, byte *payload,
											                     byte checksum_a, byte checksum_b);
void clear_buffer(byte *buffer, word size);
UBXStatus parse_rx_buffer_to_ubx_frame(UBXFrame_Typedef *ubx_frame);


#endif /* INC_UBX_H_ */
