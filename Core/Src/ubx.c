/**
  ******************************************************************************
  * @file           : ubx.c
  * @author         : Jack Bauer
  * @version        : Pre-production v0.0
  * @date           : Jan 18, 2026
  * @brief          : 
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
*/

#include "ubx.h"
#include <string.h>

extern byte UART4_rxBuffer[256];
GPS_Data_Struct GPS_Data;

//static inline void decode_nav(UBXFrame_Typedef *ubx_frame);
//static inline void decode_sec(UBXFrame_Typedef *ubx_frame);

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
	int n = 0;

	for(int i = 0; i < size; i++)
	{
		if(n > 10) { break; }

		if(n != 0x00)
		{
			buffer[i] = 0;
		}
		else
		{
			n++;
		}
	}

	return;
}

static inline void decode_nav(UBXFrame_Typedef *ubx_frame)
{
	byte *_p = ubx_frame->payload;
	GPS_Data_Struct _gps;

	// Does this work?
	memcpy(&_gps, _p, sizeof(_gps));


//	GPS_Data->iTOW = _p[3] << 3 | _p[2] << 2 | _p[1] << 1 | p[0];
//	GPS_Data->year = _p[5] << 1 | p[4];
//	GPS_Data->month = _p[6];
//	GPS_Data->day = _p[7];
//	GPS_Data->hour = _p[8];
//	GPS_Data->min = _p[9];
//	GPS_Data->sec = _p[10];
//
//	// Skipping a bunch of unnecessary data //
//
//	GPS_Data->numSV = _p[23];
//	GPS_Data->longitude = _p[27] << 3 | _p[26] << 2 | _p[25] << 1 | _p[24];
//	GPS_Data->latitude = _p[31] << 3 | _p[30] << 2 | _p[29] << 1 | _p[28];


}
static inline void decode_sec(UBXFrame_Typedef *ubx_frame)
{
	byte _id[5] = { ubx_frame->payload[4], ubx_frame->payload[5], ubx_frame->payload[6], ubx_frame->payload[7], ubx_frame->payload[8] };
	memcpy(GPS_Data.device_id, _id, sizeof(GPS_Data.device_id));
	return;
}

// This function assumes we know the command ID! Please do not call commands not explicitly listed in ubx.h.
void decode_rx_buffer_to_ubx_message(UBXFrame_Typedef *ubx_frame)
{

	byte *_buffer = UART4_rxBuffer;
	_initialize_ubx_frame_from_array(ubx_frame, _buffer); // ugh
	clear_buffer(UART4_rxBuffer, sizeof(UART4_rxBuffer));

	// UBX frame actually has data and has been soft-verified
	// TODO: Implement Fletcher's Algorithm to check the checksum on the received data

	  switch(ubx_frame->class)
	  {
	    case NAV:
	    	// Assign a GPS data struct here
	    	decode_nav(ubx_frame);
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
	    	decode_sec(ubx_frame);
	    	break;
	    case HNR:
	    	assert("Not implemented");
	    default:
	    	assert("Not implemented");
	  }
}

