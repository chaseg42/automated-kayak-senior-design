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
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "hal_config.h"
#include "system_config.h"
#include "ubx.h"

UART_HandleTypeDef huart4;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel0;
byte UART4_rxBuffer[256] = {0};


//void clear_buffer(byte *buffer, word size);
//void SystemClock_Config(void);
//static void SystemPower_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_GPDMA1_Init(void);
//static void MX_ICACHE_Init(void);
//static void MX_UART4_Init(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_System_Init();

//  /* Configure the System Power */
//  SystemPower_Config();
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  MX_GPDMA1_Init();UART4_rxBuffer
//  MX_GPIO_Init();
//  MX_ICACHE_Init();
//  MX_UART4_Init();

  system_initialize();

//  UBXStatus status;
//  UBXFrame_Typedef UBXFrame = FRAME_PREAMBLE;

  // Note: DMA transferring does not currently work. This does, however.
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UART4_rxBuffer, 256); // Initialize UART4 to use the RX interrupt
  // HAL_UART_Transmit_DMA(&huart4, ubx_tx_poll_id, sizeof(ubx_tx_poll_id));
  UBXFrame_Typedef UBXFrame;

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // 										  Division is byte - omitted.
	  HAL_UART_Transmit(&huart4, ubx_tx_poll_id, sizeof(ubx_tx_poll_id), 100);
	  HAL_Delay(250);
	  decode_rx_buffer_to_ubx_message(&UBXFrame);
//	  HAL_UART_Transmit(&huart4, ubx_tx_query_pvt, sizeof(ubx_tx_query_pvt), 100);
  }

  /* USER CODE END 3 */
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
