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

// Helper functions to ensure our message structures are valid
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
	  UBXStatus status = ubx_frame_valid(ubx_frame->preamble, _ubxm[2]);
	  if( status != UBX_OK ) { return status; }

	  // byte s1 = _ubxm[0];
	  // byte s2 = _ubxm[1]; // Skipped, these are constants
	  byte cl = _ubxm[2];
	  byte id = _ubxm[3];
	  word l_ = _ubxm[4];
	  byte *p_ = (byte *)_ubxm[5];
	  byte ca = _ubxm[6];
	  byte cb = _ubxm[7];

	  ubx_frame->class = cl;
	  ubx_frame->id = id;
	  ubx_frame->length = l_;
	  ubx_frame->payload = p_;
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

  // Preamble is always constant!!
  ubx_frame->class = class;
  ubx_frame->id = id;
  ubx_frame->length = length;
  ubx_frame->payload = payload;
  ubx_frame->checksum_a = checksum_a;
  ubx_frame->checksum_b = checksum_b;

  return UBX_OK;
}
