/**
  ******************************************************************************
  * @file           : main.c
  * @author         : Jack Bauer
  * @version        : Pre-production v0.0
  * @date           : Jan 18, 2026
  * @brief          : Testing application & entry point for the NEO-M8U STM32 software driver
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "hal_config.h"
#include "system_config.h"
#include "ubx.h"
#include "gps.h"
#include <stdbool.h>
#include <stdlib.h>

UART_HandleTypeDef huart4;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel0;
byte UART4_rxBuffer[256] = {0};
GPS_Data_Struct GPS_Data;
volatile bool bTogglePage = false;
volatile bool bFireOnce = false;
extern LCD_DrawPropTypeDef DrawProp;

// Temporary demo functions
static inline bool is_first_decimal_zero(int num, int scale);
static void demo_print_floating_ints(int val, int scale, byte row, byte col);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_System_Init();
  system_initialize();


//  UBXStatus status;
//  UBXFrame_Typedef UBXFrame = FRAME_PREAMBLE;

  // Note: DMA transferring does not currently work. This does, however.
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UART4_rxBuffer, 256); // Initialize UART4 to use the RX interrupt
  // HAL_UART_Transmit_DMA(&huart4, ubx_tx_poll_id, sizeof(ubx_tx_poll_id));
  UBXFrame_Typedef UBXFrame;

  // LCD Setup //

  // Row 	| 	Text
  // ===================================
  // 0	| 	GPS Test for Senior Design
  // 1	|	Author: Jack Bauer
  // 2	| 	--------------------------
  // 3	|	Date: {date} [utc]
  // 4	|	Time: {time} [utc]
  // 5	| 	--------------------------
  // 6	|	latd: {latd} [deg] long: {long} [deg]
  // 7	|	HAE:  {HAE}  [mm] | HAMSL: {HAMSL} [mm]
  // 8	|   VEL.N: {N} [mm/s]
  // 9	|   VEL.E: {E} [mm/s]
  // 10	| 	VEL.D: {D} [mm/s]
  // 9	|	gSpeed: {gSpeed} [mm/s]
  // 10	|	headMot: {headMot} [deg]
  // 11	|	DOP:  {DOP}
  // 12	|	headVeh: {headVeh} [deg]
  // 13	|	magDec:	{magDec} [deg]
  // 14	|	magAcc: {magAcc} [deg]
  LCD_print_str(0, 0, (unsigned char *)"GPS Test | ELE4902");
  LCD_print_str(1, 0, (unsigned char *)"Author: Jack Bauer");
  LCD_print_str(2, 0, (unsigned char *)"------------------");
  LCD_print_str(3, 0, (unsigned char *)"Date:    /   /   ");
  LCD_print_str(4, 0, (unsigned char *)"Time:    :   :   ");
  LCD_print_str(5, 0, (unsigned char *)"------------------");

  // PAGE 1 Only
  LCD_print_str(6, 0, (unsigned char *)"LAT:    .         ");
  LCD_print_str(7, 0, (unsigned char *)"LONG:    .        ");
  LCD_print_str(8, 0, (unsigned char *)"VEL.N:    .       ");
  LCD_print_str(9, 0, (unsigned char *)"ID:               ");


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // NAVIGATION SOLUTIONS ARE CURRENTLY AT 10 HZ (default), but we are requesting data every 2 Hz

	  // Using the DMA to transmit is currently not functioning! Using a CPU blocking call instead.
	  // 										  Division is byte - omitted.
	  HAL_UART_Transmit(&huart4, ubx_tx_poll_id, sizeof(ubx_tx_poll_id), 100);
	  decode_rx_buffer_to_ubx_message(&UBXFrame);
	  HAL_Delay(600);
	  HAL_UART_Transmit(&huart4, ubx_tx_poll_pvt, sizeof(ubx_tx_poll_id), 100);
	  decode_rx_buffer_to_ubx_message(&UBXFrame);
	  HAL_Delay(600);

	  /* ======================= */
	  /* LCD printing for a DEMO */
	  /* ======================= */

	  // Date
	  LCD_print_udec3(3, 6, GPS_Data.month); LCD_print_udec3(3, 10, GPS_Data.day); LCD_print_udec5(3, 14, GPS_Data.year);
	  // Time
	  LCD_print_udec3(4, 6, GPS_Data.hour); LCD_print_udec3(4, 10, GPS_Data.min); LCD_print_udec3(4, 14, GPS_Data.sec);

	  if(!bTogglePage)
	  {
		  if(bFireOnce)
		  {
			  LCD_print_str(5, 0, (unsigned char *)"-----[Page 1]-----");
			  LCD_print_str(6, 0, (unsigned char *)"LAT:    .         ");
			  LCD_print_str(7, 0, (unsigned char *)"LONG:    .        ");
			  LCD_print_str(8, 0, (unsigned char *)"VELN:    .        ");
			  LCD_print_str(9, 0, (unsigned char *)"ID:               ");
			  bFireOnce = false;
		  }
		  demo_print_floating_ints(GPS_Data.longitude, 1e+7, 6, 5);
		  demo_print_floating_ints(GPS_Data.latitude, 1e+7, 7, 5);
		  demo_print_floating_ints(GPS_Data.velN, 1e+3, 8, 6);
		  LCD_print_hex8(9, 4, GPS_Data.device_id[0]);
		  LCD_print_hex8(9, 7, GPS_Data.device_id[1]);
		  LCD_print_hex8(9, 10, GPS_Data.device_id[2]);
		  LCD_print_hex8(9, 13, GPS_Data.device_id[3]);
		  LCD_print_hex8(9, 16, GPS_Data.device_id[4]);

	  }
	  else
	  {
		  if(bFireOnce)
		  {
			  LCD_print_str(5, 0, (unsigned char *)"-----[Page 2]-----");
			  LCD_print_str(6, 0, (unsigned char *)"gSpd:    .        ");
			  LCD_print_str(7, 0, (unsigned char *)"DOP:     .        ");
			  LCD_print_str(8, 0, (unsigned char *)"hVeh:    .        ");
			  LCD_print_str(9, 0, (unsigned char *)"magD:    .        ");
			  bFireOnce = false;
		  }
		  demo_print_floating_ints(GPS_Data.gSpeed, 1e+3, 6, 5);
		  demo_print_floating_ints(GPS_Data.pDOP, 1e+2, 7, 6);
		  demo_print_floating_ints(GPS_Data.headVeh, 1e+5, 8, 5);
		  demo_print_floating_ints(GPS_Data.magDec, 1e+2, 9, 5);
	  }
  }
}

// Temporary demo functions
static inline bool is_first_decimal_zero(int num, int scale)
{
	int _num = num;
	int _scale = scale / 10;

	num = (num / scale) * 10; // 430
	_num = _num / _scale; // 431

	return (_num - num > 0);
}

static void demo_print_floating_ints(int val, int scale, byte row, byte col)
{
	bool _b;
	_b = is_first_decimal_zero(val, scale);

	int _int = (int)(val / scale);
	int _float = (val - _int * scale);

	if(!_b)
	{
		LCD_print_dec5(row, col, _int); LCD_print_dec5(row, col + 7, abs((_float / (scale / 1000))));
	}
	else
	{
		LCD_print_dec5(row, col, _int); LCD_print_char(row, col + 7, '0'); LCD_print_dec5(row, col + 8, abs((_float / (scale / 1000))));
	}
	return;
}



/**
  * @brief System Clock Configuration
  * @retval None
  */

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
