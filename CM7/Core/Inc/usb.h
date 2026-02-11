/**
  ******************************************************************************
  * @file           : usb.h
  * @author         : Jack Bauer
  * @version        : 
  * @date           : Feb 11, 2026
  * @brief          : 
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
**/
#ifndef INC_USB_H_
#define INC_USB_H_

#include "common.h"
#include "cmsis_os.h"
#include <stddef.h>

enum
{
	TYPE_BYTE,
	TYPE_CHAR,
	TYPE_WORD,
	TYPE_INT,
	TYPE_FLOAT,
	TYPE_DOUBLE

}typedef ValueType;

enum
{
	FORMAT_GENERIC,
	FORMAT_DATE,
	FORMAT_TIME,
	FORMAT_VECTOR
}typedef FormatType;

struct
{
	ValueType type;
	FormatType format;
	size_t data_count;

	union
	{
		byte *b;
		char *c;
		word *w;
		float *f;
		double *d;
	}data;

}typedef ValueTypeDef;


void usb_tx(char *buffer, const int buffer_size, ValueTypeDef *data, const char *message);


#endif /* INC_USB_H_ */
