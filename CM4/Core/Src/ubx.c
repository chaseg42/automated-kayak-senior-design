/**
  ******************************************************************************
  * @file           : ubx.c
  * @author         : Jack Bauer
  * @version        : Pre-production v0.0
  * @date           : Jan 18, 2026
  * @brief          : This source file contains the necessary functions for interpreting, decoding, and encoding ubx protocol
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
**/

#include "ubx.h"
#include "gps.h"
#include <string.h>

extern byte UART4_rxBuffer[256]; // Defined in main.c
extern GPS_Data_Struct GPS_Data; // Defined in main.c


// Helper functions to ensure our ubx frame structures are valid
static inline UBXStatus ubx_frame_valid( word preamble, byte class )
{
  UBXStatus _status = preamble == ((SYNC_CHAR_2 << 1) | SYNC_CHAR_1) ? UBX_OK : UBX_ERROR_SYNC;
  if(_status != UBX_OK) { return _status; }

  switch(class)
  {
    case NAV:
    case RXM:
    case INF:
    case ACK:
    case CFG:
    case UPD:
    case MON:
    case AID:
    case TIM:
    case ESF:
    case MGA:
    case LOG:
    case SEC:
    case HNR:
      return UBX_OK;
    default:
      return UBX_ERROR_CLASS;
  }
}

/**
  * @brief  Helper function to initialize a UBX message struct with a compiled UBX message array.
  * @param  class: ..
  * @retval UBX Status
  */
static inline UBXStatus __initialize_ubx_frame_from_array(UBXFrame_Typedef *ubx_frame, byte *ubx_message_array)
{
	  byte *_ubxm = ubx_message_array;
	  UBXStatus status = ubx_frame_valid((_ubxm[1] << 1) | _ubxm[0], _ubxm[2]);
	  if( status != UBX_OK ) { return status; }

	  byte s1 = _ubxm[0];
	  byte s2 = _ubxm[1];
	  byte cl = _ubxm[2];
	  byte id = _ubxm[3];
	  word l_ = (_ubxm[5] << 1) | _ubxm[4];

	  byte _tempPayload[l_];
	  memset(_tempPayload, 0, sizeof(_tempPayload));

	  byte offset = 6;
	  int i;

	  for(i = 0; i < l_; i++)
	  {
		  _tempPayload[i] = _ubxm[i + offset];
	  }

	  byte ca = _ubxm[i + offset];
	  byte cb = _ubxm[i + offset + 1];

	  ubx_frame->preamble = (s2 << 1) | s1;
	  ubx_frame->class = cl;
	  ubx_frame->id = id;
	  ubx_frame->length = l_;
	  ubx_frame->payload = _tempPayload;
	  ubx_frame->checksum_a = ca;
	  ubx_frame->checksum_b = cb;

	  return UBX_OK;
}

/**
  * @brief  Helper function to initialize a UBX message struct with a compiled UBX message array
  * 	    Use the Generic Macro to call the function.
  * @param  class: ..
  * @retval UBX Status
  */
UBXStatus _initialize_ubx_frame_from_array(UBXFrame_Typedef *ubx_frame, byte *ubx_message_array, ...)
{
	return __initialize_ubx_frame_from_array(ubx_frame, ubx_message_array);
}
/**
  * @brief  Helper function to initialize a UBX message struct with a custom field.
  *         Use the Generic Macro to call the function.
  * @param  class: ..
  * @retval UBX Status
  */
UBXStatus _initialize_ubx_frame_from_fields(UBXFrame_Typedef *ubx_frame,
								  byte class, byte id,
								  word length, byte *payload,
								  byte checksum_a, byte checksum_b)
{
  UBXStatus status = ubx_frame_valid(ubx_frame->preamble, class);
  if( status != UBX_OK ) { return status; }

  // Preamble should always be constant, but it is stored for brevity and debug testing purposes.
  // ubx_frame->preamble = (ubx_frame[1] << 1) | ubx_frame[0];
  ubx_frame->class = class;
  ubx_frame->id = id;
  ubx_frame->length = length;
  ubx_frame->payload = payload;
  ubx_frame->checksum_a = checksum_a;
  ubx_frame->checksum_b = checksum_b;

  return UBX_OK;
}


// TODO: Research if clearing the buffer is actually necessary.
void clear_buffer(byte *buffer, word size)
{
	memset(buffer, 0, size);
	return;
}

static inline UBXStatus decode_nav(UBXFrame_Typedef *ubx_frame)
{
	byte *_p = ubx_frame->payload;

	// TODO: status checking here or before just using the checksum.
//	UBXStatus status;

	// TODO: Automate the offset. There are also other useful navigation solutions that may be useful in the future.
	memcpy(&GPS_Data.iTOW, &_p[0], sizeof(uint32_t));
	memcpy(&GPS_Data.year, &_p[4], sizeof(word));
	memcpy(&GPS_Data.month, &_p[6], sizeof(byte));
	memcpy(&GPS_Data.day, &_p[7], sizeof(byte));
	memcpy(&GPS_Data.hour, &_p[8], sizeof(byte));
	memcpy(&GPS_Data.min, &_p[9], sizeof(byte));
	memcpy(&GPS_Data.sec, &_p[10], sizeof(byte));
	memcpy(&GPS_Data.valid, &_p[11], sizeof(byte));
	memcpy(&GPS_Data.tAcc, &_p[12], sizeof(uint32_t));
	memcpy(&GPS_Data.nano, &_p[16], sizeof(int));
	memcpy(&GPS_Data.fixType, &_p[20], sizeof(byte));
	memcpy(&GPS_Data.flags, &_p[21], sizeof(byte));
	memcpy(&GPS_Data.flags2, &_p[22], sizeof(byte));
	memcpy(&GPS_Data.numSV, &_p[23], sizeof(byte));
	memcpy(&GPS_Data.longitude, &_p[24], sizeof(int));
	memcpy(&GPS_Data.latitude, &_p[28], sizeof(int));
	memcpy(&GPS_Data.height, &_p[32], sizeof(int));
	memcpy(&GPS_Data.hMSL, &_p[36], sizeof(int));
	memcpy(&GPS_Data.hAcc, &_p[40], sizeof(uint32_t));
	memcpy(&GPS_Data.vAcc, &_p[44], sizeof(uint32_t));
	memcpy(&GPS_Data.velN, &_p[48], sizeof(int));
	memcpy(&GPS_Data.velE, &_p[52], sizeof(int));
	memcpy(&GPS_Data.velD, &_p[56], sizeof(int));
	memcpy(&GPS_Data.gSpeed, &_p[60], sizeof(int));
	memcpy(&GPS_Data.headMot, &_p[64], sizeof(int));
	memcpy(&GPS_Data.sAcc, &_p[68], sizeof(uint32_t));
	memcpy(&GPS_Data.headAcc, &_p[72], sizeof(uint32_t));
	memcpy(&GPS_Data.pDOP, &_p[76], sizeof(word));
	// Reserved - 6 bytes
	memcpy(&GPS_Data.headVeh, &_p[84], sizeof(int));
	memcpy(&GPS_Data.magDec, &_p[88], sizeof(short));
	memcpy(&GPS_Data.magAcc, &_p[90], sizeof(word));

	return UBX_OK;
}
static inline UBXStatus decode_sec(UBXFrame_Typedef *ubx_frame)
{
	// TODO: status checking here or before just using the checksum.
//	UBXStatus status;

	byte _id[5] = { ubx_frame->payload[4], ubx_frame->payload[5], ubx_frame->payload[6], ubx_frame->payload[7], ubx_frame->payload[8] };
	memcpy(GPS_Data.device_id, _id, sizeof(GPS_Data.device_id));
	return UBX_OK;
}

// This function assumes we know the command ID! Please do not call commands not explicitly listed in ubx.h.
UBXStatus decode_rx_buffer_to_ubx_message(UBXFrame_Typedef *ubx_frame)
{

	byte *_buffer = UART4_rxBuffer;
	_initialize_ubx_frame_from_array(ubx_frame, _buffer); // ugh
	clear_buffer(UART4_rxBuffer, sizeof(UART4_rxBuffer));
	UBXStatus status;

	// UBX frame actually has data and has been soft-verified
	// TODO: Implement Fletcher's Algorithm to check the checksum on the received data

	  switch(ubx_frame->class)
	  {
	    case NAV:
	    	// Assign a GPS data struct here
	    	status = decode_nav(ubx_frame);
	    	break;
	    case RXM:
	    case INF:
	    case ACK:
	    case CFG:
	    case UPD:
	    case MON:
	    case AID:
	    case TIM:
	    case ESF:
	    case MGA:
	    case LOG:
	    	assert("Not implemented");
	    case SEC:
	    	// Assign a GPS data struct here
	    	status = decode_sec(ubx_frame);
	    	break;
	    case HNR:
	    	assert("Not implemented");
	    default:
	    	assert("Not implemented");
	  }

	  return status;
}

