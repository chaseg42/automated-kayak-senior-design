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


extern byte UART4_rxBuffer[256]; // Defined in freertos.c
extern GPSParsedDataStruct GPS_Parsed_Data; // Defined in freertos.c


/**
  * @brief  Helper function to ensure our UBX Frame structures are v
  * @param  class: ..
  * @retval UBX Status
  */
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
  * @brief Calculate a Fletcher checksum from the incoming UBX frame.
  * @param UBXFrame_Typedef
  * @retval UBX Status
  */
static inline UBXStatus fletcher_checksum(UBXFrame_Typedef *ubx_frame)
{
	byte class = ubx_frame->class;
	byte id = ubx_frame->id;
	word length = ubx_frame->length;
	byte *payload = ubx_frame->payload;
	byte ck_a, ck_b= 0;

	ck_a = class;
	ck_b = ck_a;

	ck_a += id;
	ck_b += ck_a;

	ck_a += length;
	ck_b += ck_a;

	for(int i = 0; i < length; i++)
	{
		ck_a += payload[i];
		ck_b += ck_a;
	}

	if(ck_a == ubx_frame->checksum_a && ck_b == ubx_frame->checksum_b)
	{
		return UBX_OK;
	}
	else
	{
		return UBX_ERROR_CHECKSUM;
	}
}


/**
  * @brief  Helper function to initialize a UBX message struct with a compiled UBX message array.
  * @param  class: ..
  * @retval UBX Status
  */
static UBXStatus __initialize_ubx_frame_from_array(UBXFrame_Typedef *ubx_frame, byte *ubx_message_array)
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

	// status = fletcher_checksum(ubx_frame); Simple algorithm does not work... Why ublox?

	return status;
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


/**
  * @brief  Parses the UBX NAV solution
  * @param  UBXFrame_Typedef
  * @retval UBX Status
  */
static UBXStatus parse_nav(UBXFrame_Typedef *ubx_frame)
{
	byte *_p = ubx_frame->payload;

	switch(ubx_frame->id)
	{
		// Attitude solution
		case 0x05:
			memcpy(&GPS_Parsed_Data.iTOW, 			&_p[0], 		sizeof(uint32_t));
			memcpy(&GPS_Parsed_Data.version, 		&_p[4], 		sizeof(byte));
			// Reserved - 3 bytes //	// Reserved - 3 bytes // 	// Reserved - 3 bytes //
			memcpy(&GPS_Parsed_Data.roll, 			&_p[8], 		sizeof(int));
			memcpy(&GPS_Parsed_Data.pitch, 			&_p[12], 		sizeof(int));
			memcpy(&GPS_Parsed_Data.yaw, 			&_p[16], 		sizeof(int));
			memcpy(&GPS_Parsed_Data.accRoll, 		&_p[20], 		sizeof(uint32_t));
			memcpy(&GPS_Parsed_Data.accPitch, 		&_p[24], 		sizeof(uint32_t));
			memcpy(&GPS_Parsed_Data.accYaw, 		&_p[28], 		sizeof(uint32_t));
		break;

		// Navigation solution
		case 0x07:
			// TODO: Automate the offset. There are also other useful navigation solutions that may be useful in the future.
			memcpy(&GPS_Parsed_Data.iTOW, 			&_p[0], 		sizeof(uint32_t));
			memcpy(&GPS_Parsed_Data.year, 			&_p[4], 		sizeof(word));
			memcpy(&GPS_Parsed_Data.month, 			&_p[6], 		sizeof(byte));
			memcpy(&GPS_Parsed_Data.day, 			&_p[7], 		sizeof(byte));
			memcpy(&GPS_Parsed_Data.hour, 			&_p[8], 		sizeof(byte));
			memcpy(&GPS_Parsed_Data.min, 			&_p[9], 		sizeof(byte));
			memcpy(&GPS_Parsed_Data.sec, 			&_p[10], 		sizeof(byte));
			memcpy(&GPS_Parsed_Data.valid, 			&_p[11], 		sizeof(byte));
			memcpy(&GPS_Parsed_Data.tAcc, 			&_p[12], 		sizeof(uint32_t));
			memcpy(&GPS_Parsed_Data.nano, 			&_p[16], 		sizeof(int));
			memcpy(&GPS_Parsed_Data.fixType, 		&_p[20], 		sizeof(byte));
			memcpy(&GPS_Parsed_Data.flags, 			&_p[21], 		sizeof(byte));
			memcpy(&GPS_Parsed_Data.flags2, 		&_p[22], 		sizeof(byte));
			memcpy(&GPS_Parsed_Data.numSV, 			&_p[23], 		sizeof(byte));
			memcpy(&GPS_Parsed_Data.longitude, 		&_p[24], 		sizeof(int));
			memcpy(&GPS_Parsed_Data.latitude, 		&_p[28], 		sizeof(int));
			memcpy(&GPS_Parsed_Data.height, 		&_p[32], 		sizeof(int));
			memcpy(&GPS_Parsed_Data.hMSL, 			&_p[36], 		sizeof(int));
			memcpy(&GPS_Parsed_Data.hAcc, 			&_p[40], 		sizeof(uint32_t));
			memcpy(&GPS_Parsed_Data.vAcc, 			&_p[44], 		sizeof(uint32_t));
			memcpy(&GPS_Parsed_Data.velN, 			&_p[48], 		sizeof(int));
			memcpy(&GPS_Parsed_Data.velE, 			&_p[52], 		sizeof(int));
			memcpy(&GPS_Parsed_Data.velD, 			&_p[56], 		sizeof(int));
			memcpy(&GPS_Parsed_Data.gSpeed, 		&_p[60], 		sizeof(int));
			memcpy(&GPS_Parsed_Data.headMot, 		&_p[64], 		sizeof(int));
			memcpy(&GPS_Parsed_Data.sAcc, 			&_p[68], 		sizeof(uint32_t));
			memcpy(&GPS_Parsed_Data.headAcc, 		&_p[72], 		sizeof(uint32_t));
			memcpy(&GPS_Parsed_Data.pDOP, 			&_p[76], 		sizeof(word));
			// Reserved - 6 bytes //	// Reserved - 6 bytes // 	// Reserved - 6 bytes //
			memcpy(&GPS_Parsed_Data.headVeh, 		&_p[84], 		sizeof(int));
			memcpy(&GPS_Parsed_Data.magDec, 		&_p[88], 		sizeof(short));
			memcpy(&GPS_Parsed_Data.magAcc, 		&_p[90], 		sizeof(word));
		break;
	}

	return UBX_OK;
}

// TODO: parse_nav_hnr
static UBXStatus parse_nav_hnr(UBXFrame_Typedef *ubx_frame)
{
	byte *_p = ubx_frame->payload;
	// TODO: Automate the offset. There are also other useful navigation solutions that may be useful in the future.
	memcpy(&GPS_Parsed_Data.iTOW, 			&_p[0], 		sizeof(uint32_t));
	memcpy(&GPS_Parsed_Data.year, 			&_p[4], 		sizeof(word));
	memcpy(&GPS_Parsed_Data.month, 			&_p[6], 		sizeof(byte));
	memcpy(&GPS_Parsed_Data.day, 			&_p[7], 		sizeof(byte));
	memcpy(&GPS_Parsed_Data.hour, 			&_p[8], 		sizeof(byte));
	memcpy(&GPS_Parsed_Data.min, 			&_p[9], 		sizeof(byte));
	memcpy(&GPS_Parsed_Data.sec, 			&_p[10], 		sizeof(byte));
	memcpy(&GPS_Parsed_Data.valid, 			&_p[11], 		sizeof(byte));
	memcpy(&GPS_Parsed_Data.nano, 			&_p[12], 		sizeof(int));
	memcpy(&GPS_Parsed_Data.fixType, 		&_p[16], 		sizeof(byte));
	memcpy(&GPS_Parsed_Data.flags, 			&_p[17], 		sizeof(byte));
	// Reserved - 2 bytes // 	// Reserved - 2 bytes // 	// Reserved - 2 bytes //
	memcpy(&GPS_Parsed_Data.longitude, 		&_p[20], 		sizeof(int));
	memcpy(&GPS_Parsed_Data.latitude, 		&_p[24], 		sizeof(int));
	memcpy(&GPS_Parsed_Data.height, 		&_p[28], 		sizeof(int));
	memcpy(&GPS_Parsed_Data.hMSL, 			&_p[32], 		sizeof(int));
	memcpy(&GPS_Parsed_Data.gSpeed, 		&_p[36], 		sizeof(int));
	memcpy(&GPS_Parsed_Data.speed, 			&_p[40], 		sizeof(int));
	memcpy(&GPS_Parsed_Data.headMot, 		&_p[44], 		sizeof(int));
	memcpy(&GPS_Parsed_Data.headVeh, 		&_p[48], 		sizeof(int));
	memcpy(&GPS_Parsed_Data.hAcc, 			&_p[52], 		sizeof(uint32_t));
	memcpy(&GPS_Parsed_Data.vAcc, 			&_p[56], 		sizeof(uint32_t));
	memcpy(&GPS_Parsed_Data.sAcc, 			&_p[60], 		sizeof(uint32_t));
	memcpy(&GPS_Parsed_Data.headAcc, 		&_p[64], 		sizeof(short));
	// Reserved - 4 bytes //	// Reserved - 4 bytes //	// Reserved - 4 bytes //
	return UBX_OK;
}

/**
  * @brief  Parses the UBX ID solution
  * @param  UBXFrame_Typedef
  * @retval UBX Status
  */
static inline UBXStatus parse_sec(UBXFrame_Typedef *ubx_frame)
{
	byte _id[5] = { ubx_frame->payload[4], ubx_frame->payload[5], ubx_frame->payload[6], ubx_frame->payload[7], ubx_frame->payload[8] };
	memcpy(GPS_Parsed_Data.device_id, _id, sizeof(GPS_Parsed_Data.device_id));
	return UBX_OK;
}


/**
  * @brief  Parses a transmitted UBX command into a GPS data structure.
  *         ATTENTION: This function assumes we know the command ID! Please do not call commands not explicitly listed in ubx.h.
  * @param  UBXFrame_Typedef
  * @retval UBX Status
  */
UBXStatus parse_rx_buffer_to_ubx_frame(UBXFrame_Typedef *ubx_frame)
{

	byte *_buffer = UART4_rxBuffer;

	UBXStatus status = _initialize_ubx_frame_from_array(ubx_frame, _buffer); // ugh
	clear_buffer(UART4_rxBuffer, sizeof(UART4_rxBuffer));

	if(status == UBX_OK)
	{
		// UBX frame actually has data and has been soft-verified
		// TODO: Implement Fletcher's Algorithm to check the checksum on the received data

		  switch(ubx_frame->class)
		  {
			case NAV:
				// Assign a GPS data struct here
				status = parse_nav(ubx_frame);
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
				assert("Received class ID is not valid or not implemented!");
				break;
			case SEC:
				// Assign a GPS data struct here
				status = parse_sec(ubx_frame);
				break;
			case HNR:
				status = parse_nav_hnr(ubx_frame);
				break;
			default:
				assert("Received class ID is not valid or not implemented!");
				break;
		  }
	}
	return status;
}

