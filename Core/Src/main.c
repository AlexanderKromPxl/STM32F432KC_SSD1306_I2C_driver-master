/* USER CODE BEGIN Header */
/**
  ****************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ****************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ****************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "ssd1306.h"
#include <ssd1306_tests.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */
int vliegtuig();
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SSD1306_USE_I2C
#define __DEBUG 1
#define BUFFERSIZE 100
#define I2CBUF	12
#define debug_print(x) 	do { if ( __DEBUG ) { strcpy(uartBuffer, x); HAL_UART_Transmit(&huart2, (unsigned char*) uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY); }} while (0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const uint8_t SSD1306_ADDRESS = 0x3C << 1;
const uint8_t RANDOM_REG = 0x0F;

char uartBuffer[BUFFERSIZE] = "";
uint8_t I2CBuffer[I2CBUF] = {0};
HAL_StatusTypeDef returnValue = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

void init() {
	ssd1306_TestAll();
}
int x =0;

int main(void)
{
  HAL_Init();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  debug_print("Hello from STM32L432KC dev board\r\n");
  ssd1306_Init();

  ssd1306_Fill(Black);
  ssd1306_SetCursor(5, 18);
  ssd1306_WriteString("8===D >>", Font_6x8, White);
  ssd1306_SetCursor(50, 26+18);
  ssd1306_UpdateScreen();
 // init();
  int x = 10;
  int y1 =40;
  while (1)
  {

vliegtuig(y1);
wolk();
    if (y1 == 80)
    {
    	y1 =40;
    }
	  ssd1306_UpdateScreen();
	 HAL_Delay(500);
	  ssd1306_Fill(Black);
	 y1 = y1 +20;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
int wolk()
{
	int y1= 90;
	ssd1306_DrawPixel(12, 6+y1 , White);
	ssd1306_DrawPixel(13, 6+y1 , White);
	ssd1306_DrawPixel(14, 6+y1 , White);
	ssd1306_DrawPixel(15, 6+y1 , White);
	//2de lijn ***__***
	ssd1306_DrawPixel(10, 7+y1 , White);
	ssd1306_DrawPixel(11, 7+y1 , White);
	ssd1306_DrawPixel(12, 7+y1 , White);
	ssd1306_DrawPixel(15, 7+y1 , White);
	ssd1306_DrawPixel(16, 7+y1 , White);
	ssd1306_DrawPixel(17, 7+y1 , White);
	//3de lijn **_______*
	ssd1306_DrawPixel(9, 8+y1 , White);
	ssd1306_DrawPixel(10, 8+y1 , White);
	ssd1306_DrawPixel(18, 8+y1 , White);
	//4de lijn ***_________*****
	ssd1306_DrawPixel(7, 9+y1 , White);
	ssd1306_DrawPixel(8, 9+y1 , White);
	ssd1306_DrawPixel(9, 9+y1 , White);
	ssd1306_DrawPixel(19, 9+y1 , White);
	ssd1306_DrawPixel(20, 9+y1 , White);
	ssd1306_DrawPixel(21, 9+y1 , White);
	ssd1306_DrawPixel(22, 9+y1 , White);
	ssd1306_DrawPixel(23, 9+y1 , White);
	//5de lijn *_*_________*___**
	ssd1306_DrawPixel(7, 10+y1 , White);
	ssd1306_DrawPixel(9, 10+y1 , White);
	ssd1306_DrawPixel(19, 10+y1 , White);
	ssd1306_DrawPixel(23, 10+y1 , White);
	ssd1306_DrawPixel(24, 10+y1 , White);
	//6de lijn **___**_______*_____*
	ssd1306_DrawPixel(5, 11+y1 , White);
	ssd1306_DrawPixel(6, 11+y1 , White);
	ssd1306_DrawPixel(10, 11+y1 , White);
	ssd1306_DrawPixel(11, 11+y1 , White);
	ssd1306_DrawPixel(19, 11+y1 , White);
	ssd1306_DrawPixel(25, 11+y1 , White);
	//7de lijn *_______*___*
	ssd1306_DrawPixel(4, 12+y1 , White);
	ssd1306_DrawPixel(12, 12+y1 , White);
	ssd1306_DrawPixel(26, 12+y1 , White);
	//8ste lijn *______________________*
	ssd1306_DrawPixel(3, 13+y1 , White);
	ssd1306_DrawPixel(26, 13+y1 , White);
	//9de lijn *_________________*****
	ssd1306_DrawPixel(3, 14+y1 , White);
	ssd1306_DrawPixel(21, 14+y1 , White);
	ssd1306_DrawPixel(22, 14+y1 , White);
	ssd1306_DrawPixel(23, 14+y1 , White);
	ssd1306_DrawPixel(24, 14+y1 , White);
	ssd1306_DrawPixel(25, 14+y1 , White);
	//10de lijn ******************
	ssd1306_DrawPixel(4, 15+y1 , White);
	ssd1306_DrawPixel(5, 15+y1 , White);
	ssd1306_DrawPixel(6, 15+y1 , White);
	ssd1306_DrawPixel(7, 15+y1 , White);
	ssd1306_DrawPixel(8, 15+y1 , White);
	ssd1306_DrawPixel(9, 15+y1 , White);
	ssd1306_DrawPixel(10, 15+y1 , White);
	ssd1306_DrawPixel(11, 15+y1 , White);
	ssd1306_DrawPixel(12, 15+y1 , White);
	ssd1306_DrawPixel(13, 15+y1 , White);
	ssd1306_DrawPixel(14, 15+y1 , White);
	ssd1306_DrawPixel(15, 15+y1 , White);
	ssd1306_DrawPixel(16, 15+y1 , White);
	ssd1306_DrawPixel(17, 15+y1 , White);
	ssd1306_DrawPixel(18, 15+y1 , White);
	ssd1306_DrawPixel(19, 15+y1 , White);
	ssd1306_DrawPixel(20, 15+y1 , White);
	ssd1306_DrawPixel(21, 15+y1 , White);
	return 0;
}

int vliegtuig(int y1)
{
	//1ste lijn = **
	ssd1306_DrawPixel(14, 5+y1 , White);
	ssd1306_DrawPixel(15, 5+y1 , White);
	ssd1306_DrawPixel(14, 5+y1 , White);
	//2de lijn = *****
	ssd1306_DrawPixel(13, 6+y1 , White);
	ssd1306_DrawPixel(14, 6+y1 , White);
	ssd1306_DrawPixel(15, 6+y1 , White);
	ssd1306_DrawPixel(16, 6+y1 , White);
	ssd1306_DrawPixel(17, 6+y1 , White);
	//3de lijn = *___**
	ssd1306_DrawPixel(13, 7+y1 , White);
	ssd1306_DrawPixel(17, 7+y1 , White);
	ssd1306_DrawPixel(18, 7+y1 , White);
	//4de lijn = **___***____****
	ssd1306_DrawPixel(6, 8+y1 , White);
	ssd1306_DrawPixel(7, 8+y1 , White);
	ssd1306_DrawPixel(11, 8+y1 , White);
	ssd1306_DrawPixel(12, 8+y1 , White);
	ssd1306_DrawPixel(13, 8+y1 , White);
	ssd1306_DrawPixel(18, 8+y1 , White);
	ssd1306_DrawPixel(19, 8+y1 , White);
	ssd1306_DrawPixel(20, 8+y1 , White);
	ssd1306_DrawPixel(21, 8+y1 , White);
	//5de lijn = ***___*_____**
	ssd1306_DrawPixel(6, 9+y1 , White);
	ssd1306_DrawPixel(7, 9+y1 , White);
	ssd1306_DrawPixel(8, 9+y1 , White);
	ssd1306_DrawPixel(12, 9+y1 , White);
	ssd1306_DrawPixel(19, 9+y1 , White);
	ssd1306_DrawPixel(20, 9+y1 , White);
	//6de lijn = ***_**______**
	ssd1306_DrawPixel(7, 10+y1 , White);
	ssd1306_DrawPixel(8, 10+y1 , White);
	ssd1306_DrawPixel(9, 10+y1 , White);
	ssd1306_DrawPixel(11, 10+y1 , White);
	ssd1306_DrawPixel(12, 10+y1 , White);
	ssd1306_DrawPixel(21, 10+y1 , White);
	ssd1306_DrawPixel(21, 10+y1 , White);
	//7de lijn = *****************
	ssd1306_DrawPixel(9, 11+y1 , White);
	ssd1306_DrawPixel(10, 11+y1 , White);
	ssd1306_DrawPixel(11, 11+y1 , White);
	ssd1306_DrawPixel(12, 11+y1 , White);
	ssd1306_DrawPixel(13, 11+y1 , White);
	ssd1306_DrawPixel(14, 11+y1 , White);
	ssd1306_DrawPixel(15, 11+y1 , White);
	ssd1306_DrawPixel(16, 11+y1 , White);
	ssd1306_DrawPixel(17, 11+y1 , White);
	ssd1306_DrawPixel(18, 11+y1 , White);
	ssd1306_DrawPixel(19, 11+y1 , White);
	ssd1306_DrawPixel(20, 11+y1 , White);
	ssd1306_DrawPixel(21, 11+y1 , White);
	ssd1306_DrawPixel(22, 11+y1 , White);
	ssd1306_DrawPixel(23, 11+y1 , White);
	ssd1306_DrawPixel(24, 11+y1 , White);
	ssd1306_DrawPixel(25, 11+y1 , White);
	//8ste lijn = *****************(Replica lijn 7)
	ssd1306_DrawPixel(9, 12+y1 , White);
	ssd1306_DrawPixel(10, 12+y1 , White);
	ssd1306_DrawPixel(11, 12+y1 , White);
	ssd1306_DrawPixel(12, 12+y1 , White);
	ssd1306_DrawPixel(13, 12+y1 , White);
	ssd1306_DrawPixel(14, 12+y1 , White);
	ssd1306_DrawPixel(15, 12+y1 , White);
	ssd1306_DrawPixel(16, 12+y1 , White);
	ssd1306_DrawPixel(17, 12+y1 , White);
	ssd1306_DrawPixel(18, 12+y1 , White);
	ssd1306_DrawPixel(19, 12+y1 , White);
	ssd1306_DrawPixel(20, 12+y1 , White);
	ssd1306_DrawPixel(21, 12+y1 , White);
	ssd1306_DrawPixel(22, 12+y1 , White);
	ssd1306_DrawPixel(23, 12+y1 , White);
	ssd1306_DrawPixel(24, 12+y1 , White);
	ssd1306_DrawPixel(25, 12+y1 , White);
	//9de lijn = ***_**______**		(Replica lijn 6)
	ssd1306_DrawPixel(7, 13+y1 , White);
	ssd1306_DrawPixel(8, 13+y1 , White);
	ssd1306_DrawPixel(9, 13+y1 , White);
	ssd1306_DrawPixel(11, 13+y1 , White);
	ssd1306_DrawPixel(12, 13+y1 , White);
	ssd1306_DrawPixel(21, 13+y1 , White);
	ssd1306_DrawPixel(21, 13+y1, White);
	//10de lijn = ***___*_____**	(Replica lijn 5)
	ssd1306_DrawPixel(6, 14+y1 , White);
	ssd1306_DrawPixel(7, 14+y1 , White);
	ssd1306_DrawPixel(8, 14+y1 , White);
	ssd1306_DrawPixel(12, 14+y1 , White);
	ssd1306_DrawPixel(19, 14+y1 , White);
	ssd1306_DrawPixel(20, 14+y1 , White);
	//11de lijn = **___***____****	(Replica lijn 4)
	ssd1306_DrawPixel(6, 15+y1 , White);
	ssd1306_DrawPixel(7, 15+y1 , White);
	ssd1306_DrawPixel(11, 15+y1 , White);
	ssd1306_DrawPixel(12, 15+y1 , White);
	ssd1306_DrawPixel(13, 15+y1 , White);
	ssd1306_DrawPixel(18, 15+y1 , White);
	ssd1306_DrawPixel(19, 15+y1 , White);
	ssd1306_DrawPixel(20, 15+y1 , White);
	ssd1306_DrawPixel(21, 15+y1 , White);
	//12de lijn = *___**			(Replica lijn 3)
	ssd1306_DrawPixel(13, 16+y1 , White);
	ssd1306_DrawPixel(17, 16+y1 , White);
	ssd1306_DrawPixel(18, 16+y1 , White);
	//13de lijn = *****				(Replica lijn 2)
	ssd1306_DrawPixel(13, 17+y1 , White);
	ssd1306_DrawPixel(14, 17+y1 , White);
	ssd1306_DrawPixel(15, 17+y1 , White);
	ssd1306_DrawPixel(16, 17+y1 , White);
	ssd1306_DrawPixel(17, 17+y1 , White);
	//14de lijn = **				(Replica lijn 1)
	ssd1306_DrawPixel(14, 18+y1 , White);
	ssd1306_DrawPixel(15, 18+y1 , White);
	ssd1306_DrawPixel(14, 18+y1 , White);
	return 0;
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
