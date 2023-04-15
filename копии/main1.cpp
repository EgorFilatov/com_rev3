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
#define UART_RX_PERIOD_MS 2

uint8_t uartRx0[53] = { 0 };
uint8_t uartRx1[53] = { 0 };
uint8_t uartRxSaved[53] = { 0 };

uint8_t uartRxMs = 0;
uint16_t uartTxMs = 0;

uint8_t uartTx[55] = {0x55,0xAA,  // Контрольные байты (0x55,0xAA)
					  0x32,       // Размер массива
					  0xFF,0xFF,  // Тех. состояние(0-есть плата, 1-нет платы)

					  0,0,  // Порт 1
					  0,0,
					  0,0,  // Порт 2
					  0,0,
					  0,0,  // Порт 3
					  0,0,
					  0,0,  // Порт 4
					  0,0,
					  0,0,  // Порт 5
					  0,0,
					  0,0,  // Порт 6
					  0,0,
					  0,0,  // Порт 7
					  0,0,
					  0,0,  // Порт 8
					  0,0,
					  0,0,  // Порт 9
					  0,0,
					  0,0,  // Порт 10
					  0,0,
					  0,0,  // Порт 11
					  0,0,
					  0,0,  // Порт 12
					  0,0,

					  0,0}; // Контрольная сумма
uint8_t uartTxSaved[55] = { 0 };

uint8_t uartRxState0 = 0;
uint8_t uartRxState1 = 0;

uint8_t uartTxState = 0;

uint8_t uartRxFlag = 0;
uint8_t uartRxFlagOn = 0;
uint8_t uartTxFlag = 0;

uint16_t uartTxSumm = 0;

uint8_t spiState = 0;
uint8_t port = 0;

uint8_t timFlag = 0;


uint8_t spiRx[6] = { 0 }; // Massiv prinimaemyi po SPI
uint8_t spiTx[6] = { 0 }; // Massiv otpravlyaemyi po SPI

/* Byte s tipom platy -----------------------------------------------------------------------------------------------------------*/
uint8_t board_1_type  = 0; // Port 1
uint8_t board_2_type  = 0; // Port 2
uint8_t board_3_type  = 0; // Port 3
uint8_t board_4_type  = 0; // Port 4
uint8_t board_5_type  = 0; // Port 5
uint8_t board_6_type  = 0; // Port 6
uint8_t board_7_type  = 0; // Port 7
uint8_t board_8_type  = 0; // Port 8
uint8_t board_9_type  = 0; // Port 9
uint8_t board_10_type = 0; // Port 10
uint8_t board_11_type = 0; // Port 11
uint8_t board_12_type = 0; // Port 12
/* ------------------------------------------------------------------------------------------------------------------------------*/

/* Poslednee sostoyanie prinyatogo signala --------------------------------------------------------------------------------------*/
uint8_t last_sig_p_1  [4] = {0}; // Port 1
uint8_t last_sig_p_2  [4] = {0}; // Port 2
uint8_t last_sig_p_3  [4] = {0}; // Port 3
uint8_t last_sig_p_4  [4] = {0}; // Port 4
uint8_t last_sig_p_5  [4] = {0}; // Port 5
uint8_t last_sig_p_6  [4] = {0}; // Port 6
uint8_t last_sig_p_7  [4] = {0}; // Port 7
uint8_t last_sig_p_8  [4] = {0}; // Port 8
uint8_t last_sig_p_9  [4] = {0}; // Port 9
uint8_t last_sig_p_10 [4] = {0}; // Port 10
uint8_t last_sig_p_11 [4] = {0}; // Port 11
uint8_t last_sig_p_12 [4] = {0}; // Port 12
/* ------------------------------------------------------------------------------------------------------------------------------*/


uint16_t last_port_state = 0;
uint8_t spi_rx_ctrl_summ = 0;
uint8_t flag = 0;
uint8_t flag1 = 0;
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
	if (uartRxState0 == 1) {
		uartRxState0 = 2;
		uartRxState1 = 1;
		HAL_UART_Receive_IT(&huart2, uartRx1, 53);

	} else if (uartRxState1 == 1) {
		uartRxState1 = 2;
		uartRxState0 = 1;
		HAL_UART_Receive_IT(&huart2, uartRx0, 53);

	}
//	uartRxMs = 0;
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart)
{
	uartTxState = 0;
}

void HAL_SPI_TxRxCpltCallback (SPI_HandleTypeDef * hspi)
{
	GPIOC->BSRR = (1 << 9);
	GPIOC->BSRR = (1 << 8);
	GPIOC->BSRR = (1 << 7);
	GPIOC->BSRR = (1 << 6);
	GPIOB->BSRR = (1 << 15);
	GPIOB->BSRR = (1 << 14);
	GPIOB->BSRR = (1 << 13);
	GPIOB->BSRR = (1 << 12);
	GPIOB->BSRR = (1 << 2);
	GPIOB->BSRR = (1 << 1);
	GPIOB->BSRR = (1 << 0);
	GPIOC->BSRR = (1 << 5);

	spiState = 2;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	++uartRxMs;
	++uartTxMs;
}

void calculateRxSumm(uint8_t *uartRxAddr) {
	if (*(uartRxAddr + 0) != 0x55 || *(uartRxAddr + 1) != 0xAA || *(uartRxAddr + 2) != 0x30) {
		return;
	}

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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (uartRxState0 == 1 && huart2.RxXferCount < 0x0035 && huart2.RxXferCount > 0) {
			if (uartRxFlagOn == 0) {
				uartRxMs = 0;
				uartRxFlagOn = 1;
			} else if (uartRxFlagOn == 1) {
				if (uartRxMs > UART_RX_PERIOD_MS) {
					HAL_UART_AbortReceive_IT(&huart2);
					HAL_UART_Receive_IT(&huart2, uartRx0, 53);
					uartRxState0 = 1;
					uartRxFlagOn = 0;
					++flag1;
				}
			}
		} else if (uartRxState1 == 1 && huart2.RxXferCount < 0x0035 && huart2.RxXferCount > 0) {
			if (uartRxFlagOn == 0) {
				uartRxMs = 0;
				uartRxFlagOn = 1;
			} else if (uartRxFlagOn == 1) {
				if (uartRxMs > UART_RX_PERIOD_MS) {
					HAL_UART_AbortReceive_IT(&huart2);
					HAL_UART_Receive_IT(&huart2, uartRx1, 53);
					uartRxState1 = 1;
					uartRxFlagOn = 0;
					++flag;
				}
			}
		} else if (uartRxState0 == 2) {
			uartRxFlagOn = 0;
			if (spiState == 0) {
				calculateRxSumm(uartRx0);
			}
		} else if (uartRxState1 == 2) {
			uartRxFlagOn = 0;
			if (spiState == 0) {
				calculateRxSumm(uartRx1);
			}
		}












		/* *****PORT_1*****--------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if (spiState == 0 && port == 1) {
			spiState = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spiTx[0] = uartRxSaved[3];
			spiTx[1] = uartRxSaved[4];
			spiTx[2] = uartRxSaved[5];
			spiTx[3] = uartRxSaved[6];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spiTx[5] = spiTx[0] + spiTx[1] + spiTx[2] + spiTx[3]
					+ spiTx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOC->BRR = (1 << 9);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) spiTx,
					(uint8_t*) spiRx, 6);

			/* Platy vstavili v port 12----------------------------------------------------------------------------------------------------*/
			if (board_12_type != 0
					&& (last_port_state & (1 << 11)) == 0) {
				uartTx[0x04] &= ~(1 << 3); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 11); 	 // Poslednee sostoyanie ravno 1
				uartTxFlag = 3;		      // Razreshaem peredachy
			}

			/* Platy dostali iz porta 12---------------------------------------------------------------------------------------------------*/
			if (board_12_type == 0
					&& (last_port_state & (1 << 11)) != 0) {
				uartTx[0x04] |= (1 << 3); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~(1 << 11); // Poslednee sostoyanie ravno 0
				uartTxFlag = 3;	      // Razreshaem peredachy
			}

			/* Otpravka massiva pri izmenenii v signale------------------------------------------------------------------------------------*/
			if (last_sig_p_12[0] != uartTx[49]) {
				last_sig_p_12[0] = uartTx[49];
				uartTxFlag = 3;
			}
			if (last_sig_p_12[1] != uartTx[50]) {
				last_sig_p_12[1] = uartTx[50];
				uartTxFlag = 3;
			}
			if (last_sig_p_12[2] != uartTx[51]) {
				last_sig_p_12[2] = uartTx[51];
				uartTxFlag = 3;
			}
			if (last_sig_p_12[3] != uartTx[52]) {
				last_sig_p_12[3] = uartTx[52];
				uartTxFlag = 3;
			}
		}

		if (spiState == 2 && port == 1) {
			spi_rx_ctrl_summ = spiRx[0] + spiRx[1] + spiRx[2] + spiRx[3]
					+ spiRx[4];

			if (spi_rx_ctrl_summ == spiRx[5]) {
				port = 2;

				board_1_type = spiRx[4];
				uartTx[5] = spiRx[0];
				uartTx[6] = spiRx[1];
				uartTx[7] = spiRx[2];
				uartTx[8] = spiRx[3];
			}
			spiState = 0;

		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/

		/* *****PORT_2*****--------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if (spiState == 0 && port == 2) {
			spiState = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spiTx[0] = uartRxSaved[7];
			spiTx[1] = uartRxSaved[8];
			spiTx[2] = uartRxSaved[9];
			spiTx[3] = uartRxSaved[10];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spiTx[5] = spiTx[0] + spiTx[1] + spiTx[2] + spiTx[3]
					+ spiTx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOC->BRR = (1 << 8);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) spiTx,
					(uint8_t*) spiRx, 6);

			/* Platy vstavili v port 1-----------------------------------------------------------------------------------------------------*/
			if (board_1_type != 0 && (last_port_state & (1 << 0)) == 0) {
				uartTx[0x03] &= ~(1 << 0); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 0);  	// Sostoyanie porta ravno 1
				uartTxFlag = 3;		      // Razreshaem peredachy po UART
			}

			/* Platy dostali iz porta 1----------------------------------------------------------------------------------------------------*/
			if (board_1_type == 0 && (last_port_state & (1 << 0)) != 0) {
				uartTx[0x03] |= (1 << 0); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~(1 << 0);   // Sostoyanie ravno 0
				uartTxFlag = 3;
			}

			/* Otpravka massiva pri izmenenii v signale -----------------------------------------------------------------------------------*/
			if (last_sig_p_1[0] != uartTx[5]) {
				last_sig_p_1[0] = uartTx[5];
				uartTxFlag = 3;
			}
			if (last_sig_p_1[1] != uartTx[6]) {
				last_sig_p_1[1] = uartTx[6];
				uartTxFlag = 3;
			}
			if (last_sig_p_1[2] != uartTx[7]) {
				last_sig_p_1[2] = uartTx[7];
				uartTxFlag = 3;
			}
			if (last_sig_p_1[3] != uartTx[8]) {
				last_sig_p_1[3] = uartTx[8];
				uartTxFlag = 3;
			}
		}

		if (spiState == 2 && port == 2) {
			spi_rx_ctrl_summ = spiRx[0] + spiRx[1] + spiRx[2] + spiRx[3]
					+ spiRx[4];

			if (spi_rx_ctrl_summ == spiRx[5]) {
				port = 3;

				board_2_type = spiRx[4];
				uartTx[9] = spiRx[0];
				uartTx[10] = spiRx[1];
				uartTx[11] = spiRx[2];
				uartTx[12] = spiRx[3];
			}

			spiState = 0;
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/

		/* *****PORT_3*****--------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if (spiState == 0 && port == 3) {
			spiState = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spiTx[0] = uartRxSaved[11];
			spiTx[1] = uartRxSaved[12];
			spiTx[2] = uartRxSaved[13];
			spiTx[3] = uartRxSaved[14];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spiTx[5] = spiTx[0] + spiTx[1] + spiTx[2] + spiTx[3]
					+ spiTx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOC->BRR = (1 << 7);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) spiTx,
					(uint8_t*) spiRx, 6);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if (board_2_type != 0 && (last_port_state & (1 << 1)) == 0) {
				uartTx[0x03] &= ~(1 << 1); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 1);  	 // Poslednee sostoyanie ravno 1
				uartTxFlag = 3;		 			// Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if (board_2_type == 0 && (last_port_state & (1 << 1)) != 0) {
				uartTx[0x03] |= (1 << 1); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~(1 << 1);    // Poslednee sostoyanie ravno 0
				uartTxFlag = 3;	      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -----------------------------------------------------------------------------------*/
			if (last_sig_p_2[0] != uartTx[9]) {
				last_sig_p_2[0] = uartTx[9];
				uartTxFlag = 3;
			}
			if (last_sig_p_2[1] != uartTx[10]) {
				last_sig_p_2[1] = uartTx[10];
				uartTxFlag = 3;
			}
			if (last_sig_p_2[2] != uartTx[11]) {
				last_sig_p_2[2] = uartTx[11];
				uartTxFlag = 3;
			}
			if (last_sig_p_2[3] != uartTx[12]) {
				last_sig_p_2[3] = uartTx[12];
				uartTxFlag = 3;
			}
		}

		if (spiState == 2 && port == 3) {
			spi_rx_ctrl_summ = spiRx[0] + spiRx[1] + spiRx[2] + spiRx[3]
					+ spiRx[4];

			if (spi_rx_ctrl_summ == spiRx[5]) {
				port = 4;

				board_3_type = spiRx[4];
				uartTx[13] = spiRx[0];
				uartTx[14] = spiRx[1];
				uartTx[15] = spiRx[2];
				uartTx[16] = spiRx[3];
			}

			spiState = 0;
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/

		/* *****PORT_4*****--------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if (spiState == 0 && port == 4) {
			spiState = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spiTx[0] = uartRxSaved[15];
			spiTx[1] = uartRxSaved[16];
			spiTx[2] = uartRxSaved[17];
			spiTx[3] = uartRxSaved[18];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spiTx[5] = spiTx[0] + spiTx[1] + spiTx[2] + spiTx[3]
					+ spiTx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOC->BRR = (1 << 6);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) spiTx,
					(uint8_t*) spiRx, 6);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if (board_3_type != 0 && (last_port_state & (1 << 2)) == 0) {
				uartTx[0x03] &= ~(1 << 2); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 2);  	 // Poslednee sostoyanie ravno 1
				uartTxFlag = 3;		      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if (board_3_type == 0 && (last_port_state & (1 << 2)) != 0) {
				uartTx[0x03] |= (1 << 2); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~(1 << 2);    // Poslednee sostoyanie ravno 0
				uartTxFlag = 3;	      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -----------------------------------------------------------------------------------*/
			if (last_sig_p_3[0] != uartTx[13]) {
				last_sig_p_3[0] = uartTx[13];
				uartTxFlag = 3;
			}
			if (last_sig_p_3[1] != uartTx[14]) {
				last_sig_p_3[1] = uartTx[14];
				uartTxFlag = 3;
			}
			if (last_sig_p_3[2] != uartTx[15]) {
				last_sig_p_3[2] = uartTx[15];
				uartTxFlag = 3;
			}
			if (last_sig_p_3[3] != uartTx[16]) {
				last_sig_p_3[3] = uartTx[16];
				uartTxFlag = 3;
			}
		}

		if (spiState == 2 && port == 4) {
			spi_rx_ctrl_summ = spiRx[0] + spiRx[1] + spiRx[2] + spiRx[3]
					+ spiRx[4];

			if (spi_rx_ctrl_summ == spiRx[5]) {
				port = 5;

				board_4_type = spiRx[4];
				uartTx[17] = spiRx[0];
				uartTx[18] = spiRx[1];
				uartTx[19] = spiRx[2];
				uartTx[20] = spiRx[3];
			}

			spiState = 0;
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/

		/* *****PORT_5*****--------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if (spiState == 0 && port == 5) {
			spiState = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spiTx[0] = uartRxSaved[19];
			spiTx[1] = uartRxSaved[20];
			spiTx[2] = uartRxSaved[21];
			spiTx[3] = uartRxSaved[22];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spiTx[5] = spiTx[0] + spiTx[1] + spiTx[2] + spiTx[3]
					+ spiTx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOB->BRR = (1 << 15);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) spiTx,
					(uint8_t*) spiRx, 6);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if (board_4_type != 0 && (last_port_state & (1 << 3)) == 0) {
				uartTx[0x03] &= ~(1 << 3); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 3); 	 // Poslednee sostoyanie ravno 1
				uartTxFlag = 3;		      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if (board_4_type == 0 && (last_port_state & (1 << 3)) != 0) {
				uartTx[0x03] |= (1 << 3); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~(1 << 3);    // Poslednee sostoyanie ravno 0
				uartTxFlag = 3;	      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -----------------------------------------------------------------------------------*/
			if (last_sig_p_4[0] != uartTx[17]) {
				last_sig_p_4[0] = uartTx[17];
				uartTxFlag = 3;
			}
			if (last_sig_p_4[1] != uartTx[18]) {
				last_sig_p_4[1] = uartTx[18];
				uartTxFlag = 3;
			}
			if (last_sig_p_4[2] != uartTx[19]) {
				last_sig_p_4[2] = uartTx[19];
				uartTxFlag = 3;
			}
			if (last_sig_p_4[3] != uartTx[20]) {
				last_sig_p_4[3] = uartTx[20];
				uartTxFlag = 3;
			}
		}

		if (spiState == 2 && port == 5) {
			spi_rx_ctrl_summ = spiRx[0] + spiRx[1] + spiRx[2] + spiRx[3]
					+ spiRx[4];

			if (spi_rx_ctrl_summ == spiRx[5]) {
				port = 6;

				board_5_type = spiRx[4];
				uartTx[21] = spiRx[0];
				uartTx[22] = spiRx[1];
				uartTx[23] = spiRx[2];
				uartTx[24] = spiRx[3];
			}

			spiState = 0;
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/

		/* *****PORT_6*****--------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if (spiState == 0 && port == 6) {
			spiState = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spiTx[0] = uartRxSaved[23];
			spiTx[1] = uartRxSaved[24];
			spiTx[2] = uartRxSaved[25];
			spiTx[3] = uartRxSaved[26];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spiTx[5] = spiTx[0] + spiTx[1] + spiTx[2] + spiTx[3]
					+ spiTx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOB->BRR = (1 << 14);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) spiTx,
					(uint8_t*) spiRx, 6);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if (board_5_type != 0 && (last_port_state & (1 << 4)) == 0) {
				uartTx[0x03] &= ~(1 << 4); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 4);  	 // Poslednee sostoyanie ravno 1
				uartTxFlag = 3;		      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if (board_5_type == 0 && (last_port_state & (1 << 4)) != 0) {
				uartTx[0x03] |= (1 << 4); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~(1 << 4);    // Poslednee sostoyanie ravno 0
				uartTxFlag = 3;	      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -----------------------------------------------------------------------------------*/
			if (last_sig_p_5[0] != uartTx[21]) {
				last_sig_p_5[0] = uartTx[21];
				uartTxFlag = 3;
			}
			if (last_sig_p_5[1] != uartTx[22]) {
				last_sig_p_5[1] = uartTx[22];
				uartTxFlag = 3;
			}
			if (last_sig_p_5[2] != uartTx[23]) {
				last_sig_p_5[2] = uartTx[23];
				uartTxFlag = 3;
			}
			if (last_sig_p_5[3] != uartTx[24]) {
				last_sig_p_5[3] = uartTx[24];
				uartTxFlag = 3;
			}
		}

		if (spiState == 2 && port == 6) {
			spi_rx_ctrl_summ = spiRx[0] + spiRx[1] + spiRx[2] + spiRx[3]
					+ spiRx[4];

			if (spi_rx_ctrl_summ == spiRx[5]) {
				port = 7;

				board_6_type = spiRx[4];
				uartTx[25] = spiRx[0];
				uartTx[26] = spiRx[1];
				uartTx[27] = spiRx[2];
				uartTx[28] = spiRx[3];
			}

			spiState = 0;
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/

		/* *****PORT_7*****--------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if (spiState == 0 && port == 7) {
			spiState = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spiTx[0] = uartRxSaved[27];
			spiTx[1] = uartRxSaved[28];
			spiTx[2] = uartRxSaved[29];
			spiTx[3] = uartRxSaved[30];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spiTx[5] = spiTx[0] + spiTx[1] + spiTx[2] + spiTx[3]
					+ spiTx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOB->BRR = (1 << 13);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) spiTx,
					(uint8_t*) spiRx, 6);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if (board_6_type != 0 && (last_port_state & (1 << 5)) == 0) {
				uartTx[0x03] &= ~(1 << 5); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 5); 	 // Poslednee sostoyanie ravno 1
				uartTxFlag = 3;		      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if (board_6_type == 0 && (last_port_state & (1 << 5)) != 0) {
				uartTx[0x03] |= (1 << 5); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~(1 << 5);    // Poslednee sostoyanie ravno 0
				uartTxFlag = 3;	      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -----------------------------------------------------------------------------------*/
			if (last_sig_p_6[0] != uartTx[25]) {
				last_sig_p_6[0] = uartTx[25];
				uartTxFlag = 3;
			}
			if (last_sig_p_6[1] != uartTx[26]) {
				last_sig_p_6[1] = uartTx[26];
				uartTxFlag = 3;
			}
			if (last_sig_p_6[2] != uartTx[27]) {
				last_sig_p_6[2] = uartTx[27];
				uartTxFlag = 3;
			}
			if (last_sig_p_6[3] != uartTx[28]) {
				last_sig_p_6[3] = uartTx[28];
				uartTxFlag = 3;
			}
		}

		if (spiState == 2 && port == 7) {
			spi_rx_ctrl_summ = spiRx[0] + spiRx[1] + spiRx[2] + spiRx[3]
					+ spiRx[4];

			if (spi_rx_ctrl_summ == spiRx[5]) {
				port = 8;

				board_7_type = spiRx[4];
				uartTx[29] = spiRx[0];
				uartTx[30] = spiRx[1];
				uartTx[31] = spiRx[2];
				uartTx[32] = spiRx[3];
			}

			spiState = 0;
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/

		/* *****PORT_8*****--------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if (spiState == 0 && port == 8) {
			spiState = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spiTx[0] = uartRxSaved[31];
			spiTx[1] = uartRxSaved[32];
			spiTx[2] = uartRxSaved[33];
			spiTx[3] = uartRxSaved[34];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spiTx[5] = spiTx[0] + spiTx[1] + spiTx[2] + spiTx[3]
					+ spiTx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOB->BRR = (1 << 12);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) spiTx,
					(uint8_t*) spiRx, 6);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if (board_7_type != 0 && (last_port_state & (1 << 6)) == 0) {
				uartTx[0x03] &= ~(1 << 6); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 6);  	 // Poslednee sostoyanie ravno 1
				uartTxFlag = 3;		      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if (board_7_type == 0 && (last_port_state & (1 << 6)) != 0) {
				uartTx[0x03] |= (1 << 6); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~(1 << 6);    // Poslednee sostoyanie ravno 0
				uartTxFlag = 3;	      // Razreshaem peredachy
			}
			/* ------------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -------------------------------------------------------------------------------------*/
			if (last_sig_p_7[0] != uartTx[29]) {
				last_sig_p_7[0] = uartTx[29];
				uartTxFlag = 3;
			}
			if (last_sig_p_7[1] != uartTx[30]) {
				last_sig_p_7[1] = uartTx[30];
				uartTxFlag = 3;
			}
			if (last_sig_p_7[2] != uartTx[31]) {
				last_sig_p_7[2] = uartTx[31];
				uartTxFlag = 3;
			}
			if (last_sig_p_7[3] != uartTx[32]) {
				last_sig_p_7[3] = uartTx[32];
				uartTxFlag = 3;
			}
		}

		if (spiState == 2 && port == 8) {
			spi_rx_ctrl_summ = spiRx[0] + spiRx[1] + spiRx[2] + spiRx[3]
					+ spiRx[4];

			if (spi_rx_ctrl_summ == spiRx[5]) {
				port = 9;

				board_8_type = spiRx[4];
				uartTx[33] = spiRx[0];
				uartTx[34] = spiRx[1];
				uartTx[35] = spiRx[2];
				uartTx[36] = spiRx[3];
			}

			spiState = 0;
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/

		/* *****PORT_9*****--------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if (spiState == 0 && port == 9) {
			spiState = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spiTx[0] = uartRxSaved[35];
			spiTx[1] = uartRxSaved[36];
			spiTx[2] = uartRxSaved[37];
			spiTx[3] = uartRxSaved[38];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spiTx[5] = spiTx[0] + spiTx[1] + spiTx[2] + spiTx[3]
					+ spiTx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOB->BRR = (1 << 2);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) spiTx,
					(uint8_t*) spiRx, 6);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if (board_8_type != 0 && (last_port_state & (1 << 7)) == 0) {
				uartTx[0x03] &= ~(1 << 7); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 7);  	 // Poslednee sostoyanie ravno 1
				uartTxFlag = 3;		      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if (board_8_type == 0 && (last_port_state & (1 << 7)) != 0) {
				uartTx[0x03] |= (1 << 7); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~(1 << 7);    // Poslednee sostoyanie ravno 0
				uartTxFlag = 3;	      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -----------------------------------------------------------------------------------*/
			if (last_sig_p_8[0] != uartTx[33]) {
				last_sig_p_8[0] = uartTx[33];
				uartTxFlag = 3;
			}
			if (last_sig_p_8[1] != uartTx[34]) {
				last_sig_p_8[1] = uartTx[34];
				uartTxFlag = 3;
			}
			if (last_sig_p_8[2] != uartTx[35]) {
				last_sig_p_8[2] = uartTx[35];
				uartTxFlag = 3;
			}
			if (last_sig_p_8[3] != uartTx[36]) {
				last_sig_p_8[3] = uartTx[36];
				uartTxFlag = 3;
			}
		}

		if (spiState == 2 && port == 9) {
			spi_rx_ctrl_summ = spiRx[0] + spiRx[1] + spiRx[2] + spiRx[3]
					+ spiRx[4];

			if (spi_rx_ctrl_summ == spiRx[5]) {
				port = 10;

				board_9_type = spiRx[4];
				uartTx[37] = spiRx[0];
				uartTx[38] = spiRx[1];
				uartTx[39] = spiRx[2];
				uartTx[40] = spiRx[3];
			}

			spiState = 0;
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/

		/* *****PORT_10*****-------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if (spiState == 0 && port == 10) {
			spiState = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spiTx[0] = uartRxSaved[39];
			spiTx[1] = uartRxSaved[40];
			spiTx[2] = uartRxSaved[41];
			spiTx[3] = uartRxSaved[42];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spiTx[5] = spiTx[0] + spiTx[1] + spiTx[2] + spiTx[3]
					+ spiTx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOB->BRR = (1 << 1);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) spiTx,
					(uint8_t*) spiRx, 6);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if (board_9_type != 0 && (last_port_state & (1 << 8)) == 0) {
				uartTx[0x04] &= ~(1 << 0); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 8);  	 // Poslednee sostoyanie ravno 1
				uartTxFlag = 3;		      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if (board_9_type == 0 && (last_port_state & (1 << 8)) != 0) {
				uartTx[0x04] |= (1 << 0); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~(1 << 8);    // Poslednee sostoyanie ravno 0
				uartTxFlag = 3;	      // Razreshaem peredachy
			}
			/* ------------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -------------------------------------------------------------------------------------*/
			if (last_sig_p_9[0] != uartTx[37]) {
				last_sig_p_9[0] = uartTx[37];
				uartTxFlag = 3;
			}
			if (last_sig_p_9[1] != uartTx[38]) {
				last_sig_p_9[1] = uartTx[38];
				uartTxFlag = 3;
			}
			if (last_sig_p_9[2] != uartTx[39]) {
				last_sig_p_9[2] = uartTx[39];
				uartTxFlag = 3;
			}
			if (last_sig_p_9[3] != uartTx[40]) {
				last_sig_p_9[3] = uartTx[40];
				uartTxFlag = 3;
			}
		}

		if (spiState == 2 && port == 10) {
			spi_rx_ctrl_summ = spiRx[0] + spiRx[1] + spiRx[2] + spiRx[3]
					+ spiRx[4];

			if (spi_rx_ctrl_summ == spiRx[5]) {
				port = 11;

				board_10_type = spiRx[4];
				uartTx[41] = spiRx[0];
				uartTx[42] = spiRx[1];
				uartTx[43] = spiRx[2];
				uartTx[44] = spiRx[3];
			}

			spiState = 0;
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/

		/* *****PORT_11*****-------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if (spiState == 0 && port == 11) {
			spiState = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spiTx[0] = uartRxSaved[43];
			spiTx[1] = uartRxSaved[44];
			spiTx[2] = uartRxSaved[45];
			spiTx[3] = uartRxSaved[46];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spiTx[5] = spiTx[0] + spiTx[1] + spiTx[2] + spiTx[3]
					+ spiTx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOB->BRR = (1 << 0);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) spiTx,
					(uint8_t*) spiRx, 6);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if (board_10_type != 0 && (last_port_state & (1 << 9)) == 0) {
				uartTx[0x04] &= ~(1 << 1); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 9);	 // Poslednee sostoyanie ravno 1
				uartTxFlag = 3;		      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if (board_10_type == 0 && (last_port_state & (1 << 9)) != 0) {
				uartTx[0x04] |= (1 << 1); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~(1 << 9);    // Poslednee sostoyanie ravno 0
				uartTxFlag = 3;	      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -----------------------------------------------------------------------------------*/
			if (last_sig_p_10[0] != uartTx[41]) {
				last_sig_p_10[0] = uartTx[41];
				uartTxFlag = 3;
			}
			if (last_sig_p_10[1] != uartTx[42]) {
				last_sig_p_10[1] = uartTx[42];
				uartTxFlag = 3;
			}
			if (last_sig_p_10[2] != uartTx[43]) {
				last_sig_p_10[2] = uartTx[43];
				uartTxFlag = 3;
			}
			if (last_sig_p_10[3] != uartTx[44]) {
				last_sig_p_10[3] = uartTx[44];
				uartTxFlag = 3;
			}
		}

		if (spiState == 2 && port == 11) {
			spi_rx_ctrl_summ = spiRx[0] + spiRx[1] + spiRx[2] + spiRx[3]
					+ spiRx[4];

			if (spi_rx_ctrl_summ == spiRx[5]) {
				port = 12;

				board_11_type = spiRx[4];
				uartTx[45] = spiRx[0];
				uartTx[46] = spiRx[1];
				uartTx[47] = spiRx[2];
				uartTx[48] = spiRx[3];
			}

			spiState = 0;
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/

		/* *****PORT_12*****-------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if (spiState == 0 && port == 12) {
			spiState = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spiTx[0] = uartRxSaved[47];
			spiTx[1] = uartRxSaved[48];
			spiTx[2] = uartRxSaved[49];
			spiTx[3] = uartRxSaved[50];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spiTx[5] = spiTx[0] + spiTx[1] + spiTx[2] + spiTx[3]
					+ spiTx[4];


			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOC->BRR = (1 << 5);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) spiTx,
					(uint8_t*) spiRx, 6);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if (board_11_type != 0
					&& (last_port_state & (1 << 10)) == 0) {
				uartTx[0x04] &= ~(1 << 2); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 10); 	 // Poslednee sostoyanie ravno 1
				uartTxFlag = 3;		      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if (board_11_type == 0
					&& (last_port_state & (1 << 10)) != 0) {
				uartTx[0x04] |= (1 << 2); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~(1 << 10);   // Poslednee sostoyanie ravno 0
				uartTxFlag = 3;	      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -----------------------------------------------------------------------------------*/
			if (last_sig_p_11[0] != uartTx[45]) {
				last_sig_p_11[0] = uartTx[45];
				uartTxFlag = 3;
			}
			if (last_sig_p_11[1] != uartTx[46]) {
				last_sig_p_11[1] = uartTx[46];
				uartTxFlag = 3;
			}
			if (last_sig_p_11[2] != uartTx[47]) {
				last_sig_p_11[2] = uartTx[47];
				uartTxFlag = 3;
			}
			if (last_sig_p_11[3] != uartTx[48]) {
				last_sig_p_11[3] = uartTx[48];
				uartTxFlag = 3;
			}
		}

		if (spiState == 2 && port == 12) {
			spi_rx_ctrl_summ = spiRx[0] + spiRx[1] + spiRx[2] + spiRx[3]
					+ spiRx[4];

			if (spi_rx_ctrl_summ == spiRx[5]) {
				port = 1;

				board_12_type = spiRx[4];
				uartTx[49] = spiRx[0];
				uartTx[50] = spiRx[1];
				uartTx[51] = spiRx[2];
				uartTx[52] = spiRx[3];
			}

			spiState = 0;
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/























		if (uartTxMs > 40) {
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
