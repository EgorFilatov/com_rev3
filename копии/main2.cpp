/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Massivy priemo-peredachi po SPI, gde posledniy bit- kontrolnaya symma --------------------------------------------------------*/
uint8_t spi_rx [0x06] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Massiv prinimaemyi po SPI

uint8_t spi_tx [0x06] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Massiv otpravlyaemyi po SPI
/* ------------------------------------------------------------------------------------------------------------------------------*/

/* Obrabotka obmena SPI ---------------------------------------------------------------------------------------------------------*/

uint8_t spi_tx_rx_on = 0; // Flag sostoyaniya SPI
uint8_t spi_port  = 1; // Nomer porta, oprashivaemogo SPI
/* ------------------------------------------------------------------------------------------------------------------------------*/

/* Shifrovanie obmena po SPI ----------------------------------------------------------------------------------------------------*/
uint8_t spi_rx_ctrl_summ = 0; // Kontrolnaya summa prinyatogo po SPI massiva
uint8_t spi_rx_check = 0;
uint8_t spi_state = 0;
/* ------------------------------------------------------------------------------------------------------------------------------*/

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

/* UART ------------------------------------------------------------------------------------------*/
uint8_t uart_rx0 [53] = {0x00}; // Massiv, prinimaemy po UART
uint8_t uart_rx1 [53] = {0x00}; // Massiv, prinimaemy po UART
uint8_t uart_rx_slice [53] = {0x00}; // Kopiya massiva, prinimaemogo po UART na tekyschii moment
uint8_t uart_rx_state0 = 0;
uint8_t uart_rx_state0 = 1;
/* ------------------------------------------------------------------------------------------------------------------------------*/

/* Itogovyi massiv, otpravlyaemy v UART -----------------------------------------------------------------------------------------*/
uint8_t uart_tx [55] = {0x55,0xAA,  // Kontrolnye bity (0x55,0xAA)
												0x32,       // Razmer massiva
												0xFF,0xFF,  // Bity sostoyaniya portov(0-est plata, 1-net platy)

												0x00,0x00,  // Sostoyanie platy port 1
												0x00,0x00,
												0x00,0x00,  // Sostoyanie platy port 2
												0x00,0x00,
												0x00,0x00,  // Sostoyanie platy port 3
												0x00,0x00,
												0x00,0x00,  // Sostoyanie platy port 4
												0x00,0x00,
												0x00,0x00,  // Sostoyanie platy port 5
												0x00,0x00,
												0x00,0x00,  // Sostoyanie platy port 6
												0x00,0x00,
												0x00,0x00,  // Sostoyanie platy port 7
												0x00,0x00,
												0x00,0x00,  // Sostoyanie platy port 8
												0x00,0x00,
												0x00,0x00,  // Sostoyanie platy port 9
												0x00,0x00,
												0x00,0x00,  // Sostoyanie platy port 10
												0x00,0x00,
												0x00,0x00,  // Sostoyanie platy port 11
												0x00,0x00,
												0x00,0x00,  // Sostoyanie platy port 12
												0x00,0x00,

												0x00,0x00}; // Kontrolnaya summa



uint8_t uart_tx_slice [55] = {0x00};
/* ------------------------------------------------------------------------------------------------------------------------------*/

/* Sostoyanie portov (1-est plata/0-net platy) ----------------------------------------------------------------------------------*/
uint16_t last_port_state = 0;
/* ------------------------------------------------------------------------------------------------------------------------------*/

/* Razreshenie na peredachy po UART (1-razresheno/0-ne razresheno) --------------------------------------------------------------*/
uint8_t uart_tx_on = 1;
uint8_t uart_tx_state = 0;
uint8_t tim_6_flag = 0;

/* ------------------------------------------------------------------------------------------------------------------------------*/

/* Poslednee sostoyanie prinyatogo signala --------------------------------------------------------------------------------------*/
uint8_t last_sig_p_1  [0x04] = {0x00, 0x00, 0x00, 0x00}; // Port 1
uint8_t last_sig_p_2  [0x04] = {0x00, 0x00, 0x00, 0x00}; // Port 2
uint8_t last_sig_p_3  [0x04] = {0x00, 0x00, 0x00, 0x00}; // Port 3
uint8_t last_sig_p_4  [0x04] = {0x00, 0x00, 0x00, 0x00}; // Port 4
uint8_t last_sig_p_5  [0x04] = {0x00, 0x00, 0x00, 0x00}; // Port 5
uint8_t last_sig_p_6  [0x04] = {0x00, 0x00, 0x00, 0x00}; // Port 6
uint8_t last_sig_p_7  [0x04] = {0x00, 0x00, 0x00, 0x00}; // Port 7
uint8_t last_sig_p_8  [0x04] = {0x00, 0x00, 0x00, 0x00}; // Port 8
uint8_t last_sig_p_9  [0x04] = {0x00, 0x00, 0x00, 0x00}; // Port 9
uint8_t last_sig_p_10 [0x04] = {0x00, 0x00, 0x00, 0x00}; // Port 10
uint8_t last_sig_p_11 [0x04] = {0x00, 0x00, 0x00, 0x00}; // Port 11
uint8_t last_sig_p_12 [0x04] = {0x00, 0x00, 0x00, 0x00}; // Port 12
/* ------------------------------------------------------------------------------------------------------------------------------*/



uint8_t TEMP [0x05] = {0x55,0xAA,0x02,0x00,0x00}; // Massiv dlya pokazanii temperatyry

/* Obrabotka oshibki priema UART ------------------------------------------------------------------------------------------------*/
uint8_t uart_rx_ready  = 0; // Massiv po UART prinyat bez oshibok
/* ------------------------------------------------------------------------------------------------------------------------------*/

/* Obrabotka kontrolnyh summ obmena po UART -------------------------------------------------------------------------------------*/
uint16_t uart_tx_ctrl_summ = 0x00; // Kontrolnaya summa otpravlyaemogo massiva
uint16_t uart_rx_ctrl_summ = 0x00; // Kontrolnaya summa prinyatogo massiva
uint8_t uart_rx_ctrl_summArr[0x02] = {0x00, 0x00}; // Kontrolnaya summa prinyatogo massiva, razlozennaya v massiv
uint8_t flag = 0;

uint8_t uartRxMs = 0;
uint8_t uartRxFlagOn = 0;
#define UART_RX_PERIOD_MS 2
/* ------------------------------------------------------------------------------------------------------------------------------*/

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

	HAL_TIM_Base_Start(&htim6);		//Vklychenie taimera 6
	HAL_TIM_Base_Start_IT(&htim6); //Vklychenie taimera 6

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
		if (uart_rx_state0 == 0) {
			uart_rx_state0 = 1;
			HAL_UART_Receive_IT(&huart2, (uint8_t*) uart_rx0, 0x35); // Priem po UART
		}


		/* Preryvaem peredachy, pri oshibkach priema: snachala smotrim pervy bit, esli on nepravilny, to preryvaem, esli pravilny, to smotrim vtoroi bit, esli on nepravilny, to vse preryvaem,
		esli pravilny, to smotrim tretii bit, esli on nepravilny, to vse preryvaem, esli pravilny, to idem dalshe */
		if (uart_rx_state0 == 1 && huart2.RxXferCount < 0x0035 && huart2.RxXferCount > 0) {
			if (uartRxFlagOn == 0) {
				uartRxMs = 0;
				uartRxFlagOn = 1;
			} else if (uartRxFlagOn == 1) {
				if (uartRxMs > UART_RX_PERIOD_MS) {
					HAL_UART_AbortReceive_IT(&huart2);
					uart_rx_state0 = 1;
					HAL_UART_Receive_IT(&huart2, (uint8_t*)uart_rx0, 0x35);
					uartRxFlagOn = 0;
				}
			}
		}


		/* Esli zaconchilsya priem po UART, to schitaem i proveryaem kontrolnyy symmy, esli ona nepravilnaya, to vse obnylyaem, a esli pravilnaya, to daem razreschenie na zapis v TY*/
		if(uart_rx_state0 == 2)
		{
			uartRxFlagOn = 0;
			if (uart_rx0[0] == 0x55 && uart_rx0[1] == 0xAA && uart_rx0[2] == 0x30) {

			uart_rx_ctrl_summ = 0;
			for (uint8_t i = 0; i <= 50; i++) {
				uart_rx_ctrl_summ += uart_rx0[i];
			}

			uart_rx_ctrl_summArr[0x00] = (uint8_t) uart_rx_ctrl_summ;
			uart_rx_ctrl_summArr[0x01] = (uint8_t) (uart_rx_ctrl_summ >> 8);

			if (uart_rx_ctrl_summArr[0x00] == uart_rx0[0x33] && uart_rx_ctrl_summArr[0x01] == uart_rx0[0x34]) {
				if (uart_rx_ready == 0) {
					for (uint8_t i = 0; i <= 52; i++) {
						uart_rx_slice[i] = uart_rx0[i];
					}

					uart_rx_ready = 1;
				}
			}
			}
			uart_rx_state0 = 0;

		}




		/* *****PORT_1*****--------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if(spi_state == 0 && spi_port == 1 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_state = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spi_tx[0] = uart_rx_slice[3];
			spi_tx[1] = uart_rx_slice[4];
			spi_tx[2] = uart_rx_slice[5];
			spi_tx[3] = uart_rx_slice[6];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spi_tx[5] = spi_tx[0] + spi_tx[1] + spi_tx[2] + spi_tx[3] + spi_tx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOC->BRR = (1<<9);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)spi_tx, (uint8_t*)spi_rx, 0x06);

			/* Platy vstavili v port 12----------------------------------------------------------------------------------------------------*/
			if(board_12_type != 0x00 && (last_port_state & (1<<11)) == 0x00)
			{
				uart_tx   [0x04] &= ~ (1 << 3); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 11); 	      // Poslednee sostoyanie ravno 1
				uart_tx_on = 1;		      // Razreshaem peredachy
			}

			/* Platy dostali iz porta 12---------------------------------------------------------------------------------------------------*/
			if(board_12_type == 0x00 && (last_port_state & (1<<11)) != 0x00)
			{
				uart_tx   [0x04] |= (1 << 3); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~ (1 << 11); // Poslednee sostoyanie ravno 0
				uart_tx_on = 1;	      // Razreshaem peredachy
			}

			/* Otpravka massiva pri izmenenii v signale------------------------------------------------------------------------------------*/
			if (last_sig_p_12[0] != uart_tx[49])
			{
				last_sig_p_12[0] = uart_tx[49];
				uart_tx_on = 1;
			}
			if (last_sig_p_12[1] != uart_tx[50])
			{
				last_sig_p_12[1] = uart_tx[50];
				uart_tx_on = 1;
			}
			if (last_sig_p_12[2] != uart_tx[51])
			{
				last_sig_p_12[2] = uart_tx[51];
				uart_tx_on = 1;
			}
			if (last_sig_p_12[3] != uart_tx[52])
			{
				last_sig_p_12[3] = uart_tx[52];
				uart_tx_on = 1;
			}


		}


		if(spi_state == 2 && spi_port == 1 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_rx_ctrl_summ = spi_rx[0] + spi_rx[1] + spi_rx[2] + spi_rx[3] + spi_rx[4];

			if(spi_rx_ctrl_summ == spi_rx[5])
			{
				spi_state = 0;
				spi_port = 2;

				board_1_type = spi_rx[4];
				uart_tx [5]  = spi_rx[0];
				uart_tx [6]  = spi_rx[1];
				uart_tx [7]  = spi_rx[2];
				uart_tx [8]  = spi_rx[3];
			}
			else
			{
				spi_state = 0;
			}
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/
		/* ------------------------------------------------------------------------------------------------------------------------------*/


		/* *****PORT_2*****--------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if(spi_state == 0 && spi_port == 2 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_state = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spi_tx[0] = uart_rx_slice[7];
			spi_tx[1] = uart_rx_slice[8];
			spi_tx[2] = uart_rx_slice[9];
			spi_tx[3] = uart_rx_slice[10];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spi_tx[5] = spi_tx[0] + spi_tx[1] + spi_tx[2] + spi_tx[3] + spi_tx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOC->BRR = (1<<8);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)spi_tx, (uint8_t*)spi_rx, 0x06);

			/* Platy vstavili v port 1-----------------------------------------------------------------------------------------------------*/
			if(board_1_type != 0x00 && (last_port_state & (1<<0)) == 0x00)
			{
				uart_tx [0x03] &= ~ (1 << 0); // Zapis 0 v bity sostoyaniya porta
				last_port_state |=   (1 << 0);  	// Sostoyanie porta ravno 1
				uart_tx_on = 1;		      // Razreshaem peredachy po UART
			}

			/* Platy dostali iz porta 1----------------------------------------------------------------------------------------------------*/
			if(board_1_type == 0x00 && (last_port_state & (1<<0)) != 0x00)
			{
				uart_tx [0x03] |= (1 << 0); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~ (1 << 0);   // Sostoyanie ravno 0
				uart_tx_on = 1;
			}

			/* Otpravka massiva pri izmenenii v signale -----------------------------------------------------------------------------------*/
			if (last_sig_p_1[0] != uart_tx[5])
			{
				last_sig_p_1[0] = uart_tx[5];
				uart_tx_on = 1;
			}
			if (last_sig_p_1[1] != uart_tx[6])
			{
				last_sig_p_1[1] = uart_tx[6];
				uart_tx_on = 1;
			}
			if (last_sig_p_1[2] != uart_tx[7])
			{
				last_sig_p_1[2] = uart_tx[7];
				uart_tx_on = 1;
			}
			if (last_sig_p_1[3] != uart_tx[8])
			{
				last_sig_p_1[3] = uart_tx[8];
				uart_tx_on = 1;
			}


		}


		if(spi_state == 2 && spi_port == 2 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_rx_ctrl_summ = spi_rx[0] + spi_rx[1] + spi_rx[2] + spi_rx[3] + spi_rx[4];

			if(spi_rx_ctrl_summ == spi_rx[5])
			{
				spi_state = 0;
				spi_port = 3;

				board_2_type = spi_rx[4];
				uart_tx [9]  = spi_rx[0];
				uart_tx [10]  = spi_rx[1];
				uart_tx [11]  = spi_rx[2];
				uart_tx [12]  = spi_rx[3];
			}
			else
			{
				spi_state = 0;
			}
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/
		/* ------------------------------------------------------------------------------------------------------------------------------*/


		/* *****PORT_3*****--------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if(spi_state == 0 && spi_port == 3 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_state = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spi_tx[0] = uart_rx_slice[11];
			spi_tx[1] = uart_rx_slice[12];
			spi_tx[2] = uart_rx_slice[13];
			spi_tx[3] = uart_rx_slice[14];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spi_tx[5] = spi_tx[0] + spi_tx[1] + spi_tx[2] + spi_tx[3] + spi_tx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOC->BRR = (1<<7);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)spi_tx, (uint8_t*)spi_rx, 0x06);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if(board_2_type != 0x00 && (last_port_state & (1<<1)) == 0x00)
			{
				uart_tx [0x03] &= ~ (1 << 1); // Zapis 0 v bity sostoyaniya porta
				last_port_state |=   (1 << 1);  	 		  // Poslednee sostoyanie ravno 1
				uart_tx_on = 1;		 			// Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/


			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if(board_2_type == 0x00 && (last_port_state & (1<<1)) != 0x00)
			{
				uart_tx [0x03] |= (1 << 1); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~ (1 << 1);        // Poslednee sostoyanie ravno 0
				uart_tx_on = 1;	      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -----------------------------------------------------------------------------------*/
			if (last_sig_p_2[0] != uart_tx[9])
			{
				last_sig_p_2[0] = uart_tx[9];
				uart_tx_on = 1;
			}
			if (last_sig_p_2[1] != uart_tx[10])
			{
				last_sig_p_2[1] = uart_tx[10];
				uart_tx_on = 1;
			}
			if (last_sig_p_2[2] != uart_tx[11])
			{
				last_sig_p_2[2] = uart_tx[11];
				uart_tx_on = 1;
			}
			if (last_sig_p_2[3] != uart_tx[12])
			{
				last_sig_p_2[3] = uart_tx[12];
				uart_tx_on = 1;
			}


		}

		if(spi_state == 2 && spi_port == 3 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_rx_ctrl_summ = spi_rx[0] + spi_rx[1] + spi_rx[2] + spi_rx[3] + spi_rx[4];

			if(spi_rx_ctrl_summ == spi_rx[5])
			{
				spi_state = 0;
				spi_port = 4;

				board_3_type = spi_rx[4];
				uart_tx [13]  = spi_rx[0];
				uart_tx [14]  = spi_rx[1];
				uart_tx [15]  = spi_rx[2];
				uart_tx [16]  = spi_rx[3];
			}
			else
			{
				spi_state = 0;
			}
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/
		/* ------------------------------------------------------------------------------------------------------------------------------*/


		/* *****PORT_4*****--------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if(spi_state == 0 && spi_port == 4 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_state = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spi_tx[0] = uart_rx_slice[15];
			spi_tx[1] = uart_rx_slice[16];
			spi_tx[2] = uart_rx_slice[17];
			spi_tx[3] = uart_rx_slice[18];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spi_tx[5] = spi_tx[0] + spi_tx[1] + spi_tx[2] + spi_tx[3] + spi_tx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOC->BRR = (1<<6);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)spi_tx, (uint8_t*)spi_rx, 0x06);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if(board_3_type != 0x00 && (last_port_state & (1<<2)) == 0x00)
			{
				uart_tx   [0x03] &= ~ (1 << 2); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 2);  	      // Poslednee sostoyanie ravno 1
				uart_tx_on = 1;		      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if(board_3_type == 0x00 && (last_port_state & (1<<2)) != 0x00)
			{
				uart_tx   [0x03] |= (1 << 2); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~ (1 << 2);       // Poslednee sostoyanie ravno 0
				uart_tx_on = 1;	      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -----------------------------------------------------------------------------------*/
			if (last_sig_p_3[0] != uart_tx[13])
			{
				last_sig_p_3[0] = uart_tx[13];
				uart_tx_on = 1;
			}
			if (last_sig_p_3[1] != uart_tx[14])
			{
				last_sig_p_3[1] = uart_tx[14];
				uart_tx_on = 1;
			}
			if (last_sig_p_3[2] != uart_tx[15])
			{
				last_sig_p_3[2] = uart_tx[15];
				uart_tx_on = 1;
			}
			if (last_sig_p_3[3] != uart_tx[16])
			{
				last_sig_p_3[3] = uart_tx[16];
				uart_tx_on = 1;
			}


		}

		if(spi_state == 2 && spi_port == 4 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_rx_ctrl_summ = spi_rx[0] + spi_rx[1] + spi_rx[2] + spi_rx[3] + spi_rx[4];

			if(spi_rx_ctrl_summ == spi_rx[5])
			{
				spi_state = 0;
				spi_port = 5;

				board_4_type = spi_rx[4];
				uart_tx [17]  = spi_rx[0];
				uart_tx [18]  = spi_rx[1];
				uart_tx [19]  = spi_rx[2];
				uart_tx [20]  = spi_rx[3];
			}
			else
			{
				spi_state = 0;
			}
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/
		/* ------------------------------------------------------------------------------------------------------------------------------*/


		/* *****PORT_5*****--------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if(spi_state == 0 && spi_port == 5 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_state = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spi_tx[0] = uart_rx_slice[19];
			spi_tx[1] = uart_rx_slice[20];
			spi_tx[2] = uart_rx_slice[21];
			spi_tx[3] = uart_rx_slice[22];

				/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
				spi_tx[5] = spi_tx[0] + spi_tx[1] + spi_tx[2] + spi_tx[3] + spi_tx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOB->BRR = (1<<15);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)spi_tx, (uint8_t*)spi_rx, 0x06);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if(board_4_type != 0x00 && (last_port_state & (1<<3)) == 0x00)
			{
				uart_tx   [0x03] &= ~ (1 << 3); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 3); 	      // Poslednee sostoyanie ravno 1
				uart_tx_on = 1;		      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/


			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if(board_4_type == 0x00 && (last_port_state & (1<<3)) != 0x00)
			{
				uart_tx   [0x03] |= (1 << 3); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~ (1 << 3);       // Poslednee sostoyanie ravno 0
				uart_tx_on = 1;	      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -----------------------------------------------------------------------------------*/
			if (last_sig_p_4[0] != uart_tx[17])
			{
				last_sig_p_4[0] = uart_tx[17];
				uart_tx_on = 1;
			}
			if (last_sig_p_4[1] != uart_tx[18])
			{
				last_sig_p_4[1] = uart_tx[18];
				uart_tx_on = 1;
			}
			if (last_sig_p_4[2] != uart_tx[19])
			{
				last_sig_p_4[2] = uart_tx[19];
				uart_tx_on = 1;
			}
			if (last_sig_p_4[3] != uart_tx[20])
			{
				last_sig_p_4[3] = uart_tx[20];
				uart_tx_on = 1;
			}


		}

		if(spi_state == 2 && spi_port == 5 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_rx_ctrl_summ = spi_rx[0] + spi_rx[1] + spi_rx[2] + spi_rx[3] + spi_rx[4];

			if(spi_rx_ctrl_summ == spi_rx[5])
			{
				spi_state = 0;
				spi_port = 6;

				board_5_type = spi_rx[4];
				uart_tx [21]  = spi_rx[0];
				uart_tx [22]  = spi_rx[1];
				uart_tx [23]  = spi_rx[2];
				uart_tx [24]  = spi_rx[3];
			}
			else
			{
				spi_state = 0;
			}
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/
		/* ------------------------------------------------------------------------------------------------------------------------------*/


		/* *****PORT_6*****--------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if(spi_state == 0 && spi_port == 6 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_state = 1;


			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spi_tx[0] = uart_rx_slice[23];
			spi_tx[1] = uart_rx_slice[24];
			spi_tx[2] = uart_rx_slice[25];
			spi_tx[3] = uart_rx_slice[26];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spi_tx[5] = spi_tx[0] + spi_tx[1] + spi_tx[2] + spi_tx[3] + spi_tx[4];


			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOB->BRR = (1<<14);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)spi_tx, (uint8_t*)spi_rx, 0x06);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if(board_5_type != 0x00 && (last_port_state & (1<<4)) == 0x00)
			{
				uart_tx   [0x03] &= ~ (1 << 4); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 4);  	      // Poslednee sostoyanie ravno 1
				uart_tx_on = 1;		      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/


			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if(board_5_type == 0x00 && (last_port_state & (1<<4)) != 0x00)
			{
				uart_tx   [0x03] |= (1 << 4); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~ (1 << 4);       // Poslednee sostoyanie ravno 0
				uart_tx_on = 1;	      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -----------------------------------------------------------------------------------*/
			if (last_sig_p_5[0] != uart_tx[21])
			{
				last_sig_p_5[0] = uart_tx[21];
				uart_tx_on = 1;
			}
			if (last_sig_p_5[1] != uart_tx[22])
			{
				last_sig_p_5[1] = uart_tx[22];
				uart_tx_on = 1;
			}
			if (last_sig_p_5[2] != uart_tx[23])
			{
				last_sig_p_5[2] = uart_tx[23];
				uart_tx_on = 1;
			}
			if (last_sig_p_5[3] != uart_tx[24])
			{
				last_sig_p_5[3] = uart_tx[24];
				uart_tx_on = 1;
			}


		}

		if(spi_state == 2 && spi_port == 6 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_rx_ctrl_summ = spi_rx[0] + spi_rx[1] + spi_rx[2] + spi_rx[3] + spi_rx[4];

			if(spi_rx_ctrl_summ == spi_rx[5])
			{
				spi_state = 0;
				spi_port = 7;

				board_6_type = spi_rx[4];
				uart_tx [25]  = spi_rx[0];
				uart_tx [26]  = spi_rx[1];
				uart_tx [27]  = spi_rx[2];
				uart_tx [28]  = spi_rx[3];
			}
			else
			{
				spi_state = 0;
			}
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/
		/* ------------------------------------------------------------------------------------------------------------------------------*/


		/* *****PORT_7*****--------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if(spi_state == 0 && spi_port == 7 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_state = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spi_tx[0] = uart_rx_slice[27];
			spi_tx[1] = uart_rx_slice[28];
			spi_tx[2] = uart_rx_slice[29];
			spi_tx[3] = uart_rx_slice[30];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spi_tx[5] = spi_tx[0] + spi_tx[1] + spi_tx[2] + spi_tx[3] + spi_tx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOB->BRR = (1<<13);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)spi_tx, (uint8_t*)spi_rx, 0x06);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if(board_6_type != 0x00 && (last_port_state & (1<<5)) == 0x00)
			{
				uart_tx   [0x03] &= ~ (1 << 5); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 5); 	      // Poslednee sostoyanie ravno 1
				uart_tx_on = 1;		      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if(board_6_type == 0x00 && (last_port_state & (1<<5)) != 0x00)
			{
				uart_tx   [0x03] |= (1 << 5); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~ (1 << 5);       // Poslednee sostoyanie ravno 0
				uart_tx_on = 1;	      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -----------------------------------------------------------------------------------*/
			if (last_sig_p_6[0] != uart_tx[25])
			{
				last_sig_p_6[0] = uart_tx[25];
				uart_tx_on = 1;
			}
			if (last_sig_p_6[1] != uart_tx[26])
			{
				last_sig_p_6[1] = uart_tx[26];
				uart_tx_on = 1;
			}
			if (last_sig_p_6[2] != uart_tx[27])
			{
				last_sig_p_6[2] = uart_tx[27];
				uart_tx_on = 1;
			}
			if (last_sig_p_6[3] != uart_tx[28])
			{
				last_sig_p_6[3] = uart_tx[28];
				uart_tx_on = 1;
			}


		}

		if(spi_state == 2 && spi_port == 7 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_rx_ctrl_summ = spi_rx[0] + spi_rx[1] + spi_rx[2] + spi_rx[3] + spi_rx[4];

			if(spi_rx_ctrl_summ == spi_rx[5])
			{
				spi_state = 0;
				spi_port = 8;

				board_7_type = spi_rx[4];
				uart_tx [29]  = spi_rx[0];
				uart_tx [30]  = spi_rx[1];
				uart_tx [31]  = spi_rx[2];
				uart_tx [32]  = spi_rx[3];
			}
			else
			{
				spi_state = 0;
			}
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/
		/* ------------------------------------------------------------------------------------------------------------------------------*/


		/* *****PORT_8*****--------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if(spi_state == 0 && spi_port == 8 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_state = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spi_tx[0] = uart_rx_slice[31];
			spi_tx[1] = uart_rx_slice[32];
			spi_tx[2] = uart_rx_slice[33];
			spi_tx[3] = uart_rx_slice[34];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spi_tx[5] = spi_tx[0] + spi_tx[1] + spi_tx[2] + spi_tx[3] + spi_tx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOB->BRR = (1<<12);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)spi_tx, (uint8_t*)spi_rx, 0x06);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if(board_7_type != 0x00 && (last_port_state & (1<<6)) == 0x00)
			{
				uart_tx   [0x03] &= ~ (1 << 6); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 6);  	      // Poslednee sostoyanie ravno 1
				uart_tx_on = 1;		      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/


			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if(board_7_type == 0x00 && (last_port_state & (1<<6)) != 0x00)
			{
				uart_tx   [0x03] |= (1 << 6); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~ (1 << 6);       // Poslednee sostoyanie ravno 0
				uart_tx_on = 1;	      // Razreshaem peredachy
			}
			/* ------------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -------------------------------------------------------------------------------------*/
			if (last_sig_p_7[0] != uart_tx[29])
			{
				last_sig_p_7[0] = uart_tx[29];
				uart_tx_on = 1;
			}
			if (last_sig_p_7[1] != uart_tx[30])
			{
				last_sig_p_7[1] = uart_tx[30];
				uart_tx_on = 1;
			}
			if (last_sig_p_7[2] != uart_tx[31])
			{
				last_sig_p_7[2] = uart_tx[31];
				uart_tx_on = 1;
			}
			if (last_sig_p_7[3] != uart_tx[32])
			{
				last_sig_p_7[3] = uart_tx[32];
				uart_tx_on = 1;
			}


		}

		if(spi_state == 2 && spi_port == 8 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_rx_ctrl_summ = spi_rx[0] + spi_rx[1] + spi_rx[2] + spi_rx[3] + spi_rx[4];

			if(spi_rx_ctrl_summ == spi_rx[5])
			{
				spi_state = 0;
				spi_port = 9;

				board_8_type = spi_rx[4];
				uart_tx [33]  = spi_rx[0];
				uart_tx [34]  = spi_rx[1];
				uart_tx [35]  = spi_rx[2];
				uart_tx [36]  = spi_rx[3];
			}
			else
			{
				spi_state = 0;
			}
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/
		/* ------------------------------------------------------------------------------------------------------------------------------*/


		/* *****PORT_9*****--------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if(spi_state == 0 && spi_port == 9 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_state = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spi_tx[0] = uart_rx_slice[35];
			spi_tx[1] = uart_rx_slice[36];
			spi_tx[2] = uart_rx_slice[37];
			spi_tx[3] = uart_rx_slice[38];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spi_tx[5] = spi_tx[0] + spi_tx[1] + spi_tx[2] + spi_tx[3] + spi_tx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOB->BRR = (1<<2);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)spi_tx, (uint8_t*)spi_rx, 0x06);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if(board_8_type != 0x00 && (last_port_state & (1<<7)) == 0x00)
			{
				uart_tx   [0x03] &= ~ (1 << 7); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 7);  	      // Poslednee sostoyanie ravno 1
				uart_tx_on = 1;		      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/


			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if(board_8_type == 0x00 && (last_port_state & (1<<7)) != 0x00)
			{
				uart_tx   [0x03] |= (1 << 7); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~ (1 << 7);       // Poslednee sostoyanie ravno 0
				uart_tx_on = 1;	      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -----------------------------------------------------------------------------------*/
			if (last_sig_p_8[0] != uart_tx[33])
			{
				last_sig_p_8[0] = uart_tx[33];
				uart_tx_on = 1;
			}
			if (last_sig_p_8[1] != uart_tx[34])
			{
				last_sig_p_8[1] = uart_tx[34];
				uart_tx_on = 1;
			}
			if (last_sig_p_8[2] != uart_tx[35])
			{
				last_sig_p_8[2] = uart_tx[35];
				uart_tx_on = 1;
			}
			if (last_sig_p_8[3] != uart_tx[36])
			{
				last_sig_p_8[3] = uart_tx[36];
				uart_tx_on = 1;
			}


		}

		if(spi_state == 2 && spi_port == 9 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_rx_ctrl_summ = spi_rx[0] + spi_rx[1] + spi_rx[2] + spi_rx[3] + spi_rx[4];

			if(spi_rx_ctrl_summ == spi_rx[5])
			{
				spi_state = 0;
				spi_port = 10;

				board_9_type = spi_rx[4];
				uart_tx [37]  = spi_rx[0];
				uart_tx [38]  = spi_rx[1];
				uart_tx [39]  = spi_rx[2];
				uart_tx [40]  = spi_rx[3];
			}
			else
			{
				spi_state = 0;
			}
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/
		/* ------------------------------------------------------------------------------------------------------------------------------*/


		/* *****PORT_10*****-------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if(spi_state == 0 && spi_port == 10 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_state = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spi_tx[0] = uart_rx_slice[39];
			spi_tx[1] = uart_rx_slice[40];
			spi_tx[2] = uart_rx_slice[41];
			spi_tx[3] = uart_rx_slice[42];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spi_tx[5] = spi_tx[0] + spi_tx[1] + spi_tx[2] + spi_tx[3] + spi_tx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOB->BRR = (1<<1);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)spi_tx, (uint8_t*)spi_rx, 0x06);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if(board_9_type != 0x00 && (last_port_state & (1<<8)) == 0x00)
			{
				uart_tx   [0x04] &= ~ (1 << 0); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 8);  	      // Poslednee sostoyanie ravno 1
				uart_tx_on = 1;		      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/


			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if(board_9_type == 0x00 && (last_port_state & (1<<8)) != 0x00)
			{
				uart_tx   [0x04] |= (1 << 0); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~ (1 << 8);       // Poslednee sostoyanie ravno 0
				uart_tx_on = 1;	      // Razreshaem peredachy
			}
			/* ------------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -------------------------------------------------------------------------------------*/
			if (last_sig_p_9[0] != uart_tx[37])
			{
				last_sig_p_9[0] = uart_tx[37];
				uart_tx_on = 1;
			}
			if (last_sig_p_9[1] != uart_tx[38])
			{
				last_sig_p_9[1] = uart_tx[38];
				uart_tx_on = 1;
			}
			if (last_sig_p_9[2] != uart_tx[39])
			{
				last_sig_p_9[2] = uart_tx[39];
				uart_tx_on = 1;
			}
			if (last_sig_p_9[3] != uart_tx[40])
			{
				last_sig_p_9[3] = uart_tx[40];
				uart_tx_on = 1;
			}


		}

		if(spi_state == 2 && spi_port == 10 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_rx_ctrl_summ = spi_rx[0] + spi_rx[1] + spi_rx[2] + spi_rx[3] + spi_rx[4];

			if(spi_rx_ctrl_summ == spi_rx[5])
			{
				spi_state = 0;
				spi_port = 11;

				board_10_type = spi_rx[4];
				uart_tx [41]  = spi_rx[0];
				uart_tx [42]  = spi_rx[1];
				uart_tx [43]  = spi_rx[2];
				uart_tx [44]  = spi_rx[3];
			}
			else
			{
				spi_state = 0;
			}
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/
		/* ------------------------------------------------------------------------------------------------------------------------------*/


		/* *****PORT_11*****-------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if(spi_state == 0 && spi_port == 11 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_state = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spi_tx[0] = uart_rx_slice[43];
			spi_tx[1] = uart_rx_slice[44];
			spi_tx[2] = uart_rx_slice[45];
			spi_tx[3] = uart_rx_slice[46];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spi_tx[5] = spi_tx[0] + spi_tx[1] + spi_tx[2] + spi_tx[3] + spi_tx[4];

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOB->BRR = (1<<0);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)spi_tx, (uint8_t*)spi_rx, 0x06);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if(board_10_type != 0x00 && (last_port_state & (1<<9)) == 0x00)
			{
				uart_tx   [0x04] &= ~ (1 << 1); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 9);	      // Poslednee sostoyanie ravno 1
				uart_tx_on = 1;		      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/


			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if(board_10_type == 0x00 && (last_port_state & (1<<9)) != 0x00)
			{
				uart_tx   [0x04] |= (1 << 1); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~ (1 << 9);       // Poslednee sostoyanie ravno 0
				uart_tx_on = 1;	      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -----------------------------------------------------------------------------------*/
			if (last_sig_p_10[0] != uart_tx[41])
			{
				last_sig_p_10[0] = uart_tx[41];
				uart_tx_on = 1;
			}
			if (last_sig_p_10[1] != uart_tx[42])
			{
				last_sig_p_10[1] = uart_tx[42];
				uart_tx_on = 1;
			}
			if (last_sig_p_10[2] != uart_tx[43])
			{
				last_sig_p_10[2] = uart_tx[43];
				uart_tx_on = 1;
			}
			if (last_sig_p_10[3] != uart_tx[44])
			{
				last_sig_p_10[3] = uart_tx[44];
				uart_tx_on = 1;
			}


		}

		if(spi_state == 2 && spi_port == 11 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_rx_ctrl_summ = spi_rx[0] + spi_rx[1] + spi_rx[2] + spi_rx[3] + spi_rx[4];

			if(spi_rx_ctrl_summ == spi_rx[5])
			{
				spi_state = 0;
				spi_port = 12;

				board_11_type = spi_rx[4];
				uart_tx [45]  = spi_rx[0];
				uart_tx [46]  = spi_rx[1];
				uart_tx [47]  = spi_rx[2];
				uart_tx [48]  = spi_rx[3];
			}
			else
			{
				spi_state = 0;
			}
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/
		/* ------------------------------------------------------------------------------------------------------------------------------*/


		/* *****PORT_12*****-------------------------------------------------------------------------------------------------------------*/
		/* Esli sostoyanie SPI ravno 0, to proveryaem, chto proishodit opros nyznogo porta, i vse nogi spi podnyaty----------------------*/
		if(spi_state == 0 && spi_port == 12 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_state = 1;

			/* Prisvaivaem massivy otpravlyaemomy po SPI znacheniya signalov, prinyatyh po UART--------------------------------------------*/
			spi_tx[0] = uart_rx_slice[47];
			spi_tx[1] = uart_rx_slice[48];
			spi_tx[2] = uart_rx_slice[49];
			spi_tx[3] = uart_rx_slice[50];

			/* Schitayem kontrolnyu summy, i zapisyvaem v poslednii element massiva, otpravlyaemogo po SPI---------------------------------*/
			spi_tx[5] = spi_tx[0] + spi_tx[1] + spi_tx[2] + spi_tx[3] + spi_tx[4];
			uart_rx_ready = 0;

			/* Opyskaem nogy CS, i proizvodim priemo-peredachu po SPI----------------------------------------------------------------------*/
			GPIOC->BRR = (1<<5);
			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)spi_tx, (uint8_t*)spi_rx, 0x06);

			/* Platy vstavili v port ------------------------------------------------------------------------------------------------------*/
			if(board_11_type != 0x00 && (last_port_state & (1<<10)) == 0x00)
			{
				uart_tx   [0x04] &= ~ (1 << 2); // Zapis 0 v bity sostoyaniya porta
				last_port_state |= (1 << 10); 	      // Poslednee sostoyanie ravno 1
				uart_tx_on = 1;		      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/


			/* Platy dostali iz porta -----------------------------------------------------------------------------------------------------*/
			if(board_11_type == 0x00 && (last_port_state & (1<<10)) != 0x00)
			{
				uart_tx   [0x04] |= (1 << 2); // Zapis 1 v bity sostoyaniya porta
				last_port_state &= ~ (1 << 10);       // Poslednee sostoyanie ravno 0
				uart_tx_on = 1;	      // Razreshaem peredachy
			}
			/* ----------------------------------------------------------------------------------------------------------------------------*/

			/* Otpravka massiva pri izmenenii v signale -----------------------------------------------------------------------------------*/
			if (last_sig_p_11[0] != uart_tx[45])
			{
				last_sig_p_11[0] = uart_tx[45];
				uart_tx_on = 1;
			}
			if (last_sig_p_11[1] != uart_tx[46])
			{
				last_sig_p_11[1] = uart_tx[46];
				uart_tx_on = 1;
			}
			if (last_sig_p_11[2] != uart_tx[47])
			{
				last_sig_p_11[2] = uart_tx[47];
				uart_tx_on = 1;
			}
			if (last_sig_p_11[3] != uart_tx[48])
			{
				last_sig_p_11[3] = uart_tx[48];
				uart_tx_on = 1;
			}


		}

		if(spi_state == 2 && spi_port == 12 && (GPIOC->IDR & (1<<9)) != 0x00 && (GPIOC->IDR & (1<<8)) != 0x00 && (GPIOC->IDR & (1<<7)) != 0x00 && (GPIOC->IDR & (1<<6)) != 0x00 && (GPIOB->IDR & (1<<15)) != 0x00 && (GPIOB->IDR & (1<<14)) != 0x00 && (GPIOB->IDR & (1<<13)) != 0x00 && (GPIOB->IDR & (1<<12)) != 0x00 && (GPIOB->IDR & (1<<2)) != 0x00 && (GPIOB->IDR & (1<<1)) != 0x00 && (GPIOB->IDR & (1<<0)) != 0x00 && (GPIOC->IDR & (1<<5)) != 0x00)
		{
			spi_rx_ctrl_summ = spi_rx[0] + spi_rx[1] + spi_rx[2] + spi_rx[3] + spi_rx[4];

			if(spi_rx_ctrl_summ == spi_rx[5])
			{
				spi_state = 0;
				spi_port = 1;

				board_12_type = spi_rx[4];
				uart_tx [49]  = spi_rx[0];
				uart_tx [50]  = spi_rx[1];
				uart_tx [51]  = spi_rx[2];
				uart_tx [52]  = spi_rx[3];
			}
			else
			{
				spi_state = 0;
			}
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/
		/* ------------------------------------------------------------------------------------------------------------------------------*/





























		if(tim_6_flag == 1 && uart_tx_on == 0)
		{
			tim_6_flag = 0;
			uart_tx_on = 2;
		}

	  /* Peredacha massiva "uart_tx" po UART ------------------------------------------------------------------------------------------*/
		if(uart_tx_on != 0 && uart_tx_state == 0) //&& HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
		{
			if(uart_tx_on == 1 || uart_tx_on == 2)
			{
				flag = flag + 1;
				uart_tx_slice[0]  = 0x55 ;
				uart_tx_slice[1]  = 0xAA ;
				uart_tx_slice[2]  = uart_tx[2] ;
				uart_tx_slice[3]  = uart_tx[3] ;
				uart_tx_slice[4]  = uart_tx[4] ;
				uart_tx_slice[5]  = uart_tx[5] ;
				uart_tx_slice[6]  = uart_tx[6] ;
				uart_tx_slice[7]  = uart_tx[7] ;
				uart_tx_slice[8]  = uart_tx[8] ;
				uart_tx_slice[9]  = uart_tx[9] ;
				uart_tx_slice[10] = uart_tx[10];
				uart_tx_slice[11] = uart_tx[11];
				uart_tx_slice[12] = uart_tx[12];
				uart_tx_slice[13] = uart_tx[13];
				uart_tx_slice[14] = uart_tx[14];
				uart_tx_slice[15] = uart_tx[15];
				uart_tx_slice[16] = uart_tx[16];
				uart_tx_slice[17] = uart_tx[17];
				uart_tx_slice[18] = uart_tx[18];
				uart_tx_slice[19] = uart_tx[19];
				uart_tx_slice[20] = uart_tx[20];
				uart_tx_slice[21] = uart_tx[21];
				uart_tx_slice[22] = uart_tx[22];
				uart_tx_slice[23] = uart_tx[23];
				uart_tx_slice[24] = uart_tx[24];
				uart_tx_slice[25] = uart_tx[25];
				uart_tx_slice[26] = uart_tx[26];
				uart_tx_slice[27] = uart_tx[27];
				uart_tx_slice[28] = uart_tx[28];
				uart_tx_slice[29] = uart_tx[29];
				uart_tx_slice[30] = uart_tx[30];
				uart_tx_slice[31] = uart_tx[31];
				uart_tx_slice[32] = uart_tx[32];
				uart_tx_slice[33] = uart_tx[33];
				uart_tx_slice[34] = uart_tx[34];
				uart_tx_slice[35] = uart_tx[35];
				uart_tx_slice[36] = uart_tx[36];
				uart_tx_slice[37] = uart_tx[37];
				uart_tx_slice[38] = uart_tx[38];
				uart_tx_slice[39] = uart_tx[39];
				uart_tx_slice[40] = uart_tx[40];
				uart_tx_slice[41] = uart_tx[41];
				uart_tx_slice[42] = uart_tx[42];
				uart_tx_slice[43] = uart_tx[43];
				uart_tx_slice[44] = uart_tx[44];
				uart_tx_slice[45] = uart_tx[45];
				uart_tx_slice[46] = uart_tx[46];
				uart_tx_slice[47] = uart_tx[47];
				uart_tx_slice[48] = uart_tx[48];
				uart_tx_slice[49] = uart_tx[49];
				uart_tx_slice[50] = uart_tx[50];
				uart_tx_slice[51] = uart_tx[51];
				uart_tx_slice[52] = uart_tx[52];
				uart_tx_slice[53] = uart_tx[53];
				uart_tx_slice[54] = uart_tx[54];

				uart_tx_ctrl_summ = 					uart_tx_slice[0]
								+	uart_tx_slice[1] 	+ uart_tx_slice[2]
								+ uart_tx_slice[3] 	+ uart_tx_slice[4]
								+	uart_tx_slice[5] 	+ uart_tx_slice[6]
								+ uart_tx_slice[7] 	+ uart_tx_slice[8]
								+ uart_tx_slice[9] 	+ uart_tx_slice[10]
								+ uart_tx_slice[11] + uart_tx_slice[12]
								+ uart_tx_slice[13] + uart_tx_slice[14]
								+ uart_tx_slice[15] + uart_tx_slice[16]
								+ uart_tx_slice[17] + uart_tx_slice[18]
								+ uart_tx_slice[19] + uart_tx_slice[20]
								+ uart_tx_slice[21] + uart_tx_slice[22]
								+ uart_tx_slice[23] + uart_tx_slice[24]
								+ uart_tx_slice[25] + uart_tx_slice[26]
								+ uart_tx_slice[27] + uart_tx_slice[28]
								+ uart_tx_slice[29] + uart_tx_slice[30]
								+ uart_tx_slice[31] + uart_tx_slice[32]
								+ uart_tx_slice[33] + uart_tx_slice[34]
								+ uart_tx_slice[35] + uart_tx_slice[36]
								+ uart_tx_slice[37] + uart_tx_slice[38]
								+ uart_tx_slice[39] + uart_tx_slice[40]
								+ uart_tx_slice[41] + uart_tx_slice[42]
								+ uart_tx_slice[43] + uart_tx_slice[44]
								+ uart_tx_slice[45] + uart_tx_slice[46]
								+ uart_tx_slice[47] + uart_tx_slice[48]
								+ uart_tx_slice[49] + uart_tx_slice[50]
								+ uart_tx_slice[51] + uart_tx_slice[52];

				uart_tx_slice[53] = (uint8_t)uart_tx_ctrl_summ;
				uart_tx_slice[54] = (uint8_t)(uart_tx_ctrl_summ >> 8);
			}

			uart_tx_on = uart_tx_on + 1;

			if(uart_tx_on == 3)
			{
				uart_tx_on = 0;
			}
			uart_tx_state = 1;
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)uart_tx_slice, 55);
		}
		/* ------------------------------------------------------------------------------------------------------------------------------*/



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
  htim6.Init.Prescaler = 47999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4;
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
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
	uart_rx_state0 = 2;
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart)
{
	uart_tx_state = 0;
}

void HAL_SPI_TxRxCpltCallback (SPI_HandleTypeDef * hspi)
{
	GPIOC->BSRR = (1<<9);
	GPIOC->BSRR = (1<<8);
	GPIOC->BSRR = (1<<7);
	GPIOC->BSRR = (1<<6);
	GPIOB->BSRR = (1<<15);
	GPIOB->BSRR = (1<<14);
	GPIOB->BSRR = (1<<13);
	GPIOB->BSRR = (1<<12);
	GPIOB->BSRR = (1<<2);
	GPIOB->BSRR = (1<<1);
	GPIOB->BSRR = (1<<0);
	GPIOC->BSRR = (1<<5);

	spi_state = 2;
	spi_rx_check = spi_rx_check + 1;
}
/* Otpravka po taimery kazdye 10s -----------------------------------------------------------------------------------------------*/
uint16_t txMs = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) //check if the interrupt comes from TIM6
	{
		++uartRxMs;
		++txMs;
		if (txMs > 300) {
			tim_6_flag = 1;
			txMs = 0;
		}

	}
}
/* ------------------------------------------------------------------------------------------------------------------------------*/

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
