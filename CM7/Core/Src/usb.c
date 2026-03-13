/**
  ******************************************************************************
  * @file           : usb.c
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

#include "usb.h"
#include "ubx.h" // Move the clear buffer out of this header

static int construct_message(char *buffer, const int buffer_size, ValueTypeDef *data, const char *message);
static int construct_message_byte(ValueTypeDef *data, int message_size, char *buffer, const int buffer_size, size_t i);
static int construct_message_byte_as_hex(ValueTypeDef *data, int message_size, char *buffer, const int buffer_size, size_t i);
static int construct_message_word(ValueTypeDef *data, int message_size, char *buffer, const int buffer_size, size_t i);
static int construct_message_double(ValueTypeDef *data, int message_size, char *buffer, const int buffer_size, size_t i);

void usb_tx(char *buffer, const int buffer_size, ValueTypeDef *data, const char *message)
{
	// Construct the message
	int message_size = construct_message(buffer, buffer_size, data, message);

	// Transmit the constructed message
	CDC_Transmit_FS((char *)buffer, message_size);

	// Clear the buffer
	clear_buffer((byte *)buffer, buffer_size);

	osDelay(25); // Avoid saturating the USB buffer
}

static int construct_message(char *buffer, const int buffer_size, ValueTypeDef *data, const char *message)
{
	// Construct the message
	int size = snprintf(buffer, buffer_size, "%s ", message);

	for(size_t i = 0; i < data->data_count; i++)
	{
		switch(data->format)
		{
			case FORMAT_GENERIC:
				// TODO: Implement generic functionality
				if(data->type == TYPE_BYTE)
				{
					size += construct_message_byte_as_hex(data, size, buffer, buffer_size, i);
					size += snprintf(buffer + size, buffer_size - size, " ");
				}
				break;

			case FORMAT_DATE:
				size += construct_message_word(data, size, buffer, buffer_size, i);
				if(i < (data->data_count - 1))
				{
					size += snprintf(buffer + size, buffer_size - size, "/");
				}
				break;

			case FORMAT_TIME:
				size += construct_message_byte(data, size, buffer, buffer_size, i);
				if(i < (data->data_count - 1))
				{
					size += snprintf(buffer + size, buffer_size - size, ":");
				}
				break;

			case FORMAT_VECTOR:
				if(i == 0)
					size += snprintf(buffer + size, buffer_size - size, "{");

				size += construct_message_double(data, size, buffer, buffer_size, i);

				if(i < data->data_count - 1)
					size += snprintf(buffer + size, buffer_size - size, ", ");

				if (i == data->data_count - 1)
					size += snprintf(buffer + size, buffer_size - size, "}");
				break;

			default:
				break;
		}
	}

	size += snprintf(buffer + size, buffer_size - size, "\r\n");

	return size;
}


static int construct_message_byte(ValueTypeDef *data, int message_size, char *buffer, const int buffer_size, size_t i)
{
	return snprintf(buffer + message_size, buffer_size - message_size, "%u", data->data.b[i]);
}


static int construct_message_byte_as_hex(ValueTypeDef *data, int message_size, char *buffer, const int buffer_size, size_t i)
{
	return snprintf(buffer + message_size, buffer_size - message_size, "0x%x", data->data.b[i]);
}

static int construct_message_word(ValueTypeDef *data, int message_size, char *buffer, const int buffer_size, size_t i)
{
	return snprintf(buffer + message_size, buffer_size - message_size, "%u", data->data.w[i]);
}

static int construct_message_double(ValueTypeDef *data, int message_size, char *buffer, const int buffer_size, size_t i)
{
	return snprintf(buffer + message_size, buffer_size - message_size, "%.2lf", data->data.d[i]);
}



