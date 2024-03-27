/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// tao mang truyen du lieu
uint8_t TX[4],RX[4], RX1[4] ,RX2[4] ;
volatile uint8_t run_flag, start_complted, meas_complted;
#define DUMMY                     0x00

/**
 * @brief Max stop number.
 * @details Definition of max stop number.
 */
#define DEV_MAX_NUM_STOPS         5


/**
 * @brief Calibration2.
 * @details Number of measuring clock periods, one of ( 2, 10, 20, 40 ).
 */
uint8_t  m_cal_2_periods;

/**
 * @brief CONFIG1 register value.
 * @details CONFIG1 register value to start measurement.
 */
uint8_t  m_config1;

/**
 * @brief Measurement mode.
 * @details Measurement mode ( 1, 2 ).
 */
uint8_t  m_mode;

/**
 * @brief Cached Norm LSB.
 * @details Cached norm_lsb value for tof calculation.
 */
uint32_t  m_norm_lsb;

/**
 * @brief Clock period.
 * @details Clock period in ( ps ).
 */
uint32_t m_clk_period_ps;

/**
 * @brief Overflow time.
 * @details Overflow time in ( ps ).
 */
uint32_t m_overflow_ps;

volatile uint8_t cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */


void delay_1us(uint32_t N)
{
	for (int n = 0; n< N; n ++)
		for (int m = 0; m < 60; m ++);

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

//  // tdc_int
//
  run_flag = 0;
  start_complted = 0;
  HAL_GPIO_WritePin(START_TDC_GPIO_Port,START_TDC_Pin, 0);
  HAL_GPIO_WritePin(STOP_TDC_GPIO_Port,STOP_TDC_Pin, 0);
  HAL_GPIO_WritePin(enable_spi_GPIO_Port, enable_spi_Pin, 0);
//
//  // tdc_default
//  HAL_GPIO_WritePin(enable_spi_GPIO_Port, enable_spi_Pin, 0);
//  HAL_Delay(10);
//  HAL_GPIO_WritePin(enable_spi_GPIO_Port, enable_spi_Pin, 1);
//  HAL_Delay(10);
//  m_clk_period_ps = PS_PER_K_SEC / ( TDC_RING_OSC_FREQ_KHZ * 2 );
//  m_overflow_ps = 0;
//
//  TX[0]=TDC_REG_ADR_INT_MASK + 64;
//  TX[1]= 0x04 | 0x02 | 0x01;
//  HAL_SPI_TransmitReceive(&hspi2, TX, RX, 16, 100);


//  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 1);
//  HAL_GPIO_WritePin(enable_spi_GPIO_Port, enable_spi_Pin, 0);
//  HAL_Delay(100);
//  HAL_GPIO_WritePin(enable_spi_GPIO_Port, enable_spi_Pin, 1);
//
//
//  HAL_GPIO_WritePin(START_TDC_GPIO_Port, START_TDC_Pin, 0);
//  HAL_Delay(1);
//  HAL_GPIO_WritePin(STOP_TDC_GPIO_Port, STOP_TDC_Pin, 0);
//  HAL_Delay(5);
//
//	TX[0]=0x00 + 64;
//	TX[1]=0x03;
//	  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 0);
//	  HAL_SPI_TransmitReceive(&hspi2, TX, RX, 2, 100);
//	  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 1);
//	  HAL_Delay(50);
//
//  HAL_GPIO_WritePin(START_TDC_GPIO_Port, START_TDC_Pin, 1);
////  HAL_Delay(10);
////  HAL_GPIO_WritePin(START_TDC_GPIO_Port, START_TDC_Pin, 0);
//  HAL_Delay(5);
//  HAL_GPIO_WritePin(STOP_TDC_GPIO_Port, STOP_TDC_Pin, 1);
//  HAL_Delay(10);
//  HAL_GPIO_WritePin(STOP_TDC_GPIO_Port, STOP_TDC_Pin, 0);
//  HAL_GPIO_WritePin(START_TDC_GPIO_Port, START_TDC_Pin, 0);
//  //HAL_Delay(10);
//	TX[0]=0x10;
//	TX[1]=0x00;
//	TX[2]=0x00;
//	TX[3]=0x00;
//
//	RX[0]=0x00;
//	RX[1]=0x00;
//	RX[2]=0x00;
//	RX[3]=0x00;
//
//	  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 0);
//	  HAL_SPI_TransmitReceive(&hspi2, TX, RX, 4, 100);
//
//	  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	cnt++;
//	HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 1);
//	  HAL_GPIO_WritePin(enable_spi_GPIO_Port, enable_spi_Pin, 0);
//	  HAL_Delay(100);
//	  HAL_GPIO_WritePin(enable_spi_GPIO_Port, enable_spi_Pin, 1);

	if (run_flag == 1)
	{

			  TX[0]=0x02 +64;
			  TX[1]=0xFF;
		      TX[2]=0x00;
			  TX[3]=0x00;

			  RX2[0]=0x00;
		      RX2[1]=0x00;
			  RX2[2]=0x00;
			  RX2[3]=0x00;

			  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 0);
			  HAL_SPI_TransmitReceive(&hspi2, TX, RX2, 2, 100);

			  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 1);
			  HAL_Delay(100);

	  HAL_GPIO_WritePin(START_TDC_GPIO_Port, START_TDC_Pin, 0);
	  HAL_Delay(1);
	  HAL_GPIO_WritePin(STOP_TDC_GPIO_Port, STOP_TDC_Pin, 0);
	  HAL_Delay(1);

	  TX[0]=0x00 + 64; // + 64 tương ứng với việc write data vào thanh ghi 0h
	  TX[1]=0x03;       // ghi giá trị 03h

	  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 0);
	  HAL_SPI_TransmitReceive(&hspi2, TX, RX, 2, 100);
	  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 1);
	  HAL_Delay(500);





	  // tao xung
	  HAL_GPIO_WritePin(START_TDC_GPIO_Port, START_TDC_Pin, 1);
	  delay_1us(1);
	  HAL_GPIO_WritePin(START_TDC_GPIO_Port, START_TDC_Pin, 0);

	  delay_1us(1000);

	  HAL_GPIO_WritePin(STOP_TDC_GPIO_Port, STOP_TDC_Pin, 1);
	  delay_1us(1);
	  HAL_GPIO_WritePin(STOP_TDC_GPIO_Port, STOP_TDC_Pin, 0);

	  // end tao xung


	  //kiem tra Start completed
	  start_complted = 0;
	  while (start_complted == 0)
	  {
	  	  TX[0]=0x02;	// đ�?c thanh ghi tại địa chỉ 11h; 00 la doc, 64 la ghi
	  	  TX[1]=0x00;

	  	  RX1[0]=0x00;
	      RX1[1]=0x00;


	  	  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 0);
	  	  HAL_SPI_TransmitReceive(&hspi2, TX, RX1, 2, 100);
	  	  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 1);

	  	start_complted =   RX1[1] & 0x08;
	  }

	  //kiem tra Start completed
	  meas_complted = 0;
	  while (meas_complted == 0)
	  {
	  	  TX[0]=0x02;	// đ�?c thanh ghi tại địa chỉ 11h; 00 la doc, 64 la ghi
	  	  TX[1]=0x00;

	  	  RX1[0]=0x00;
	      RX1[1]=0x00;


	  	  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 0);
	  	  HAL_SPI_TransmitReceive(&hspi2, TX, RX1, 2, 100);
	  	  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 1);

	  	meas_complted =   RX1[1] & 0x10;
	  }


	  HAL_Delay(50);
	  TX[0]=0x11;	// đ�?c thanh ghi tại địa chỉ 11h; 00 la doc, 64 la ghi
	  TX[1]=0x00;
      TX[2]=0x00;
	  TX[3]=0x00;

	  RX2[0]=0x00;
      RX2[1]=0x00;
	  RX2[2]=0x00;
	  RX2[3]=0x00;

	  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 0);
	  HAL_SPI_TransmitReceive(&hspi2, TX, RX2, 4, 100);
	  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 1);
	  HAL_Delay(50);


//	  TX[0]=0x02;
//	  TX[1]=0x00;
//      TX[2]=0x00;
//	  TX[3]=0x00;
//
//	  RX2[0]=0x00;
//      RX2[1]=0x00;
//	  RX2[2]=0x00;
//	  RX2[3]=0x00;
//
//	  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 0);
//	  HAL_SPI_TransmitReceive(&hspi2, TX, RX2, 2, 100);
//
//	  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 1);
////	  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 0);
////	  HAL_SPI_TransmitReceive(&hspi2, TX, RX, 2, 100);
//
//	  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, 1);
	  run_flag = 0;
	}

	  HAL_Delay(100);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED0_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, START_TDC_Pin|STOP_TDC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSn_spi_GPIO_Port, CSn_spi_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(enable_spi_GPIO_Port, enable_spi_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED0_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : START_TDC_Pin STOP_TDC_Pin */
  GPIO_InitStruct.Pin = START_TDC_Pin|STOP_TDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY3_Pin KEY2_Pin KEY1_Pin GPIO_EXTI0_Pin */
  GPIO_InitStruct.Pin = KEY3_Pin|KEY2_Pin|KEY1_Pin|GPIO_EXTI0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : CSn_spi_Pin enable_spi_Pin */
  GPIO_InitStruct.Pin = CSn_spi_Pin|enable_spi_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

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
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
