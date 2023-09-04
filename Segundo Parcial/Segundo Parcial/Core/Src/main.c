/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/****************************************************************************************
 * Nombre: Angel Gabrir Segura V. 2021-0311
 *
 * Microcontroladores
 *
 * Segundo Parcial
 *
 *
 *
 ***************************************************************************************/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "matriztarea1.h"
#include<math.h>
#include<stdio.h>
#include<string.h>
#include "stdbool.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t VALOR_ADC;
char val[10];
int matris_on[8][8];
int wave = 0;
uint32_t Medida = 0;
//int VALOR_ADC;
//float cont3=0;
//int cont4=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t counter = 0;
//uint32_t counterr = 0;
int16_t count = 0;
//int16_t countt = 0;
int16_t Position_timee = 0;
int16_t Position_voltt = 0;
int16_t Position_volt = 1;
int8_t Stat;
GPIO_PinState estpul;
bool ctrl = false;
uint8_t RX1_Char = 0x00;
uint8_t MSG1[] = "OK\r\n";
uint8_t MSG3[] = "KLK\r\n";
int16_t previous_position = 0;
uint8_t MSG2[20];
//int posicion_anterior = 0;
//int velocidad = 0;
///////////////////////////nuevo//////////////////////
//#define RxBuf_SIZE 2048
//#define MainBuf_SIZE 2048

//uint8_t RxBuf[RxBuf_SIZE];
//uint8_t MainBuf[MainBuf_SIZE];

//uint16_t oldPos = 0;
//uint16_t newPos = 0;
//int isOK = 0;
//
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
//	if (huart->Instance == USART2) {
//		oldPos = newPos;  // Update the last position before copying new data
//
//		/* If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
//		 * This is to maintain the circular buffer
//		 * The old data in the main buffer will be overlapped
//		 */
//		if (oldPos + Size > MainBuf_SIZE) // If the current position + new data size is greater than the main buffer
//		{
//			uint16_t datatocopy = MainBuf_SIZE - oldPos; // find out how much space is left in the main buffer
//			memcpy((uint8_t*) MainBuf + oldPos, RxBuf, datatocopy); // copy data in that remaining space
//
//			oldPos = 0;  // point to the start of the buffer
//			memcpy((uint8_t*) MainBuf, (uint8_t*) RxBuf + datatocopy,
//					(Size - datatocopy));  // copy the remaining data
//			newPos = (Size - datatocopy);  // update the position
//		}
//
//		/* if the current position + new data size is less than the main buffer
//		 * we will simply copy the data into the buffer and update the position
//		 */
//		else {
//			memcpy((uint8_t*) MainBuf + oldPos, RxBuf, Size);
//			newPos = Size + oldPos;
//		}
//
//		/* start the DMA again */
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*) RxBuf, RxBuf_SIZE);
//		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
//
//	}
//	/******************* PROCESS (Little) THE DATA HERE*****************/
//	/* Let's say we want to check for the keyword "OK" within our incoming DATA */
//	for (int i = 0; i < Size; i++) {
//		if ((RxBuf[i] == 'O') && (RxBuf[i + 1] == 'K')) {
//			isOK = 1;
//		}
//	}
//}

void Uart() {
	//--------[ Read The Button State & Send It Via UART ]---------
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
		//HAL_UART_Transmit(&huart2, MSG2, sizeof(MSG2), 100);
		//HAL_UART_Transmit(&huart2, MSG1, sizeof(MSG1), 100);
	}

	else {
		HAL_UART_Transmit(&huart2, MSG1, sizeof(MSG1), 100);
	}
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)) {

			//HAL_UART_Transmit(&huart2, MSG2, sizeof(MSG2), 100);
			//HAL_UART_Transmit(&huart2, MSG1, sizeof(MSG1), 100);
		}
	else {
		HAL_UART_Transmit(&huart2, MSG3, sizeof(MSG3), 100);
		}
	//--------[ Read The Received Character & Toggle LEDs Accordingly ]--------
	if (RX1_Char == '1') {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
		HAL_UART_Receive_IT(&huart2, &RX1_Char, 1);
		RX1_Char = 0x00;

	}
	if (RX1_Char == '2') {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
		HAL_UART_Receive_IT(&huart2, &RX1_Char, 1);
		RX1_Char = 0x00;
	}
	HAL_Delay(1);
}

/*void button() {
 estpul = HAL_GPIO_ReadPin(Pulsador_GPIO_Port, Pulsador_Pin);

 if (estpul && !ctrl) {
 ctrl = true;
 } else if (!estpul && ctrl) {
 HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
 ctrl = false;
 }
 if (HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin) == GPIO_PIN_SET) {
 Stat = 1; // Establecer Stat en 1 cuando el pin del LED está activo
 } else {
 Stat = 0; // Establecer Stat en 0 cuando el pin del LED está inactivo
 }
 }
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

	counter = __HAL_TIM_GET_COUNTER(htim);

	count = (int16_t) counter;
	Position_voltt = count / 4;
	if (Position_voltt < 0) {
		Position_voltt = 0;
		__HAL_TIM_SET_COUNTER(htim, 0);

	}

	 // Verificar si Position_voltt está aumentando o disminuyendo
	 if (Position_voltt > previous_position) {
	 strcpy(MSG2, "aumentar\r\n");
	 HAL_UART_Transmit(&huart2, MSG2, sizeof(MSG2), 100);
	 } else if (Position_voltt < previous_position) {
	 strcpy(MSG2, "disminuir\r\n");
	 HAL_UART_Transmit(&huart2, MSG2, sizeof(MSG2), 100);
	 }
	 previous_position = Position_voltt;
}

//---------[ UART Data Reception Completion CallBackFunc. ]---------
void HAL_USART_RxCpltCallback(UART_HandleTypeDef *huart) {

	HAL_UART_Receive_IT(&huart2, &RX1_Char, 1);

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_DAC_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
	HAL_UART_Receive_IT(&huart2, &RX1_Char, 1);
	HAL_ADC_Start_DMA(&hadc1, &VALOR_ADC, 1);
	HAL_TIM_Base_Start(&htim15);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, &VALOR_ADC, 1, DAC_ALIGN_12B_R);
	HAL_UART_Transmit(&huart2, MSG1, sizeof(MSG1), 100);
	max_init(0x02);
	Uart();


	// Cambiar la frecuencia del temporizador TIM15 a 1000 Hz
	///////////////////////nuevo///////////////////////
	//HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);
	//__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	//button();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		Uart();
		//	button();
		sprintf(val, "%u\n", VALOR_ADC);
		uint8_t len = strlen(val);
		HAL_UART_Transmit(&huart2, (uint8_t*) val, len, 100);

		if (Stat == 0) {

			if (VALOR_ADC > 1) {
				for (int i = 0; i < 8; i++) {
					matris_on[7][i] = 1;
				}
			}
			max_clear();
			if (VALOR_ADC >= (250 * Position_volt * 0.1)
					&& VALOR_ADC <= (720 * Position_volt * 0.1)) {
				for (int i = wave; i < 8; i++) {
					matris_on[i][0] = 1;
				}
			} else {
				matris_on[wave][0] = 0;
			}

			// Columna 2
			if (VALOR_ADC >= (1260 * Position_volt * 0.1)
					&& VALOR_ADC <= (1780 * Position_volt * 0.1)) {
				for (int i = wave; i < 8; i++) {
					matris_on[i][2] = 1;
				}
			} else {
				matris_on[wave][2] = 0;
			}

			// Columna 3
			if (VALOR_ADC >= (1780 * Position_volt * 0.1)
					&& VALOR_ADC <= (2280 * Position_volt * 0.1)) {
				for (int i = wave; i < 8; i++) {
					matris_on[i][3] = 1;
				}
			} else {
				matris_on[wave][3] = 0;
			}

			// Columna 4
			if (VALOR_ADC >= (2280 * Position_volt * 0.1)
					&& VALOR_ADC <= (2780 * Position_volt * 0.1)) {
				for (int i = wave; i < 8; i++) {
					matris_on[i][4] = 1;
				}
			} else {
				matris_on[wave][4] = 0;
			}

			// Columna 5
			if (VALOR_ADC >= (2780 * Position_volt * 0.1)
					&& VALOR_ADC <= (3280 * Position_volt * 0.1)) {
				for (int i = wave; i < 8; i++) {
					matris_on[i][5] = 1;
				}
			} else {
				matris_on[wave][5] = 0;
			}

			// Columna 6
			if (VALOR_ADC >= (3280 * Position_volt * 0.1)
					&& VALOR_ADC <= (3780 * Position_volt * 0.1)) {
				for (int i = wave; i < 8; i++) {
					matris_on[i][6] = 1;
				}
			} else {
				matris_on[wave][6] = 0;
			}

			// Columna 7
			if (VALOR_ADC >= (3780 * Position_volt * 0.1)
					&& VALOR_ADC <= (4000 * Position_volt * 0.1)) {
				for (int i = wave; i < 8; i++) {
					matris_on[i][7] = 1;
				}
			} else {
				matris_on[wave][7] = 0;
			}

			// Actualizar los LEDs en la matriz
			for (int z = 0; z < 8; z++) {
				for (int i = 0; i < 8; i++) {
					setled(i, z, matris_on[i][z]);
				}
			}

			wave++;
			HAL_Delay(30);
			if (wave > 7) {
				wave = 0;
			}
		} else if (Stat == 1) {

			if (VALOR_ADC > 1) {
				for (int i = 0; i < 8; i++) {
					matris_on[7][i] = 1;
				}
			}
			max_clear();
			if (VALOR_ADC >= (250 * 4 * 0.1) && VALOR_ADC <= (720 * 4 * 0.1)) {
				for (int i = 0; i < 8; i++) {
					matris_on[wave][i] = 1;
				}
			} else {
				for (int i = 0; i < 8; i++) {
					matris_on[wave][i] = 0;
				}
			}

			for (int z = 0; z < 8; z++) {
				for (int i = 0; i < 8; i++) {
					if (matris_on[i][z] == 1) {
						setled(i, z, 1);
					} else {
						setled(i, z, 0);
					}
				}
			}
			wave++;
			HAL_Delay(1);
			if (wave > 7) {
				wave = 0;
			}
		}
	}
}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM15|RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T15_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 64-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 100-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, R2_Pin|LED_Pin|GPIO_PIN_14|GPIO_PIN_15
                          |R4_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Pulsador_Pin C3_Pin */
  GPIO_InitStruct.Pin = Pulsador_Pin|C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : R2_Pin LED_Pin PB14 PB15
                           R4_Pin PB6 */
  GPIO_InitStruct.Pin = R2_Pin|LED_Pin|GPIO_PIN_14|GPIO_PIN_15
                          |R4_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : R1_Pin */
  GPIO_InitStruct.Pin = R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(R1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : C4_Pin C2_Pin */
  GPIO_InitStruct.Pin = C4_Pin|C2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : C1_Pin */
  GPIO_InitStruct.Pin = C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(C1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
	while (1) {
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
