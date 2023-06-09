/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <main.hpp>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SpiPort.hpp"

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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define TIM_PERIOD_MS 5

uint8_t uartRx0[53] = { 0 };
uint8_t uartRx1[53] = { 0 };
uint8_t uartRxSaved[53] = { 0 };

uint8_t uartResMs = 0;
uint16_t uartTxMs = 0;

uint8_t uartTx[55] = {0x55,0xAA,  // ����������� ����� (0x55,0xAA)
					  0x32,       // ������ �������
					  0xFF,0xFF,  // ���. ���������(0-���� �����, 1-��� �����)

					  0,0,  // ���� 1
					  0,0,
					  0,0,  // ���� 2
					  0,0,
					  0,0,  // ���� 3
					  0,0,
					  0,0,  // ���� 4
					  0,0,
					  0,0,  // ���� 5
					  0,0,
					  0,0,  // ���� 6
					  0,0,
					  0,0,  // ���� 7
					  0,0,
					  0,0,  // ���� 8
					  0,0,
					  0,0,  // ���� 9
					  0,0,
					  0,0,  // ���� 10
					  0,0,
					  0,0,  // ���� 11
					  0,0,
					  0,0,  // ���� 12
					  0,0,

					  0,0}; // ����������� �����
uint8_t uartTxSaved[55] = { 0 };

uint8_t uartRxState0 = 0;
uint8_t uartRxState1 = 0;

uint8_t uartTxState = 0;

uint8_t uartRxFlag = 0;
uint8_t uartTxFlag = 0;

uint16_t uartTxSumm = 0;

SpiPort spiPort[12];
uint8_t spiState = 0;
uint8_t port = 0;

uint8_t timFlag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (uartRxState0 = 1) {
		uartRxState0 = 2;
		uartRxState1 = 1;
		HAL_UART_Receive_IT(&huart2, uartRx1, 53);

	} else if (uartRxState1 = 1) {
		uartRxState1 = 2;
		uartRxState0 = 1;
		HAL_UART_Receive_IT(&huart2, uartRx0, 53);

	}
	uartResMs = 0;
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart)
{
	uartTxState = 0;
}

void HAL_SPI_TxRxCpltCallback (SPI_HandleTypeDef * hspi)
{
	spiState = 2;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	++uartResMs;
	++uartTxMs;
}

void calculateRxSumm(uint8_t *uartRxAddr) {
	uint16_t uartRxSumm = 0;
	uint8_t uartRxSummArr[2] = { 0 };

	for (uint8_t i = 0; i < 51; i++) {
		uartRxSumm += *(uartRxAddr + i);
	}
	uartRxSummArr[0] = (uint8_t) uartRxSumm;
	uartRxSummArr[1] = (uint8_t) (uartRxSumm >> 8);

	if (uartRxSummArr[0] == *(uartRxAddr + 51) && uartRxSummArr[0] == *(uartRxAddr + 51)) {
		for (uint8_t i = 0; i < 53; i++) {
			uartRxSaved[i] = uartRx0[i];
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	spiPort[0].setCS(GPIOC, 9);
	spiPort[1].setCS(GPIOC, 8);
	spiPort[2].setCS(GPIOC, 7);
	spiPort[3].setCS(GPIOC, 6);
	spiPort[4].setCS(GPIOB, 15);
	spiPort[5].setCS(GPIOB, 14);
	spiPort[6].setCS(GPIOB, 13);
	spiPort[7].setCS(GPIOB, 12);
	spiPort[8].setCS(GPIOB, 2);
	spiPort[9].setCS(GPIOB, 1);
	spiPort[10].setCS(GPIOB, 0);
	spiPort[11].setCS(GPIOC, 5);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);

  HAL_UART_Receive_IT(&huart2, uartRx0, 53);
  uartRxState0 = 1;
  uartResMs = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (uartRxState0 == 1 && uartResMs > 2 && huart2.RxXferCount < 0x0035 && huart2.RxXferCount > 0) {
			HAL_UART_AbortReceive_IT(&huart2);
			HAL_UART_Receive_IT(&huart2, uartRx0, 53);
			uartRxState0 = 0;
			uartResMs = 0;
		} else if (uartRxState1 == 1 && uartResMs > 2 && huart2.RxXferCount < 0x0035 && huart2.RxXferCount > 0) {
				HAL_UART_AbortReceive_IT(&huart2);
				HAL_UART_Receive_IT(&huart2, uartRx1, 53);
				uartRxState1 = 0;
				uartResMs = 0;
		} else if (uartRxState0 == 2) {
			if (spiState == 0) {
				calculateRxSumm(uartRx0);
			}
		} else if (uartRxState1 == 2) {
			if (spiState == 0) {
				calculateRxSumm(uartRx1);
			}
		}


		if (spiState == 0) {
			if (port == 0) {
				spiState = 1;
				spiPort[12].unSelect();

				spiPort[port].setTx(&uartRxSaved[4 * port + 3], &uartRxSaved[4 * port + 4], &uartRxSaved[4 * port + 5], &uartRxSaved[4 * port + 6]);
				spiPort[port].select();
				HAL_SPI_TransmitReceive_IT(&hspi1, spiPort[port].getTx(), spiPort[port].getRx(), 6);
			}

		} else if (spiState == 2) {
			if (port == 0) {
				spiState = 0;
				if (spiPort[port].rxCheck()) {
					++port;
					if (spiPort[port].prevCheck()) {
						uartTxFlag = 3;

						uartTx[4 * port + 5] = *(spiPort[port].getRx());
						uartTx[4 * port + 6] = *(spiPort[port].getRx() + 1);
						uartTx[4 * port + 7] = *(spiPort[port].getRx() + 2);
						uartTx[4 * port + 8] = *(spiPort[port].getRx() + 3);
						*(spiPort[0].getRx() + 4) ? uartTx[3] &= ~(1 << port) : uartTx[3] |= (1 << port);
					}
				}
			}
		}





		if (uartTxMs > 300) {
			uartTxFlag = 1;
		}

		if (uartTxFlag != 0 && uartTxState == 0 && uartTxMs > 1) {
			uartTxState = 1;
			--uartTxFlag;

			for (int i = 0; i < 55; i++) {
				uartTxSaved[i] = uartTx[i];
			}

			uartTxSumm = 0;
			for (uint8_t i = 0; i < 53; i++) {
				uartTxSumm += uartTxSaved[i];
			}
			uartTxSaved[53] = (uint8_t) uartTxSumm;
			uartTxSaved[54] = (uint8_t) (uartTxSumm >> 8);

			uartTxMs = 0;
			HAL_UART_Transmit_IT(&huart2, (uint8_t*) uartTxSaved, 55);
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 23999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = TIM_PERIOD_MS * 2 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pins : PC5 PC6 PC7 PC8
                           PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB12
                           PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
