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
#include "stdbool.h"

// Helper functions to ensure our message structures are valid
UBXStatus ubx_message_valid( word preamble, byte class )
{

  preamble == (SYNC_CHAR_2 << 1) | SYNC_CHAR_1 ? return UBX_OK : return UBX_ERROR_SYNC;

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

// Helper function to initialize UBX message structs
UBXStatus initialize_ubx_messages(UBXFrame_Typedef *ubx_frame, const byte *ubx_message_array)
{
  byte _ubxm = *ubx_message_array;
  UBXStatus status = ubx_message_valid(_ubxm[0], _ubxm[1], _ubxm[2]);
  if( status != UBX_OK ) { return status; }

  // byte s1 = _ubxm[0];
  // byte s2 = _ubxm[1]; // Skipped, these are constants
  byte cl = _ubxm[2];
  byte id = _ubxm[3];
  word l_ = _ubxm[4];
  byte *p_ = (byte *)_ubxm[5];
  byte ca = _ubxm[5];
  byte cb = _ubxm[6];
  
  ubx_frame->class = cl;
  ubx_frame->id = id;
  ubx_frame->length = l_;
  ubx_frame->payload = p_;
  ubx_frame->checksum_a = ca;
  ubx_frame->checksum_b = cb;


  return UBX_OK;
}


