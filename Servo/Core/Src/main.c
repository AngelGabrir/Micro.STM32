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
/*****************************************
 *  Nombre: Angel Gabrir Segura V.        *
 *                                        *
 *  Matricula: 2021-0311                  *
 *                                        *
 *  Primer parcial                       *
 *                                        */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include "max_matrix_stm32.h"
#include "keypad_4x4.h"
#include "stdio.h"
#include "stdlib.h"
#include "StepMotor.h"
#include "Servo.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#define SERVO_TIMER &htim2
//#define SERVO_CHANNEL TIM_CHANNEL_1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t periodCount = 0; //////////////
//uint32_t servoDelay = 0; ///////////////////////
uint32_t stepperDelay = 0; /////////////////////

char key;
char keypad;
#define TRIG_PIN GPIO_PIN_0
#define TRIG_PORT GPIOC
#define ECHO_PIN GPIO_PIN_1
#define ECHO_PORT GPIOC
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance = 0;  // cm
char strCopy[15];
uint8_t change = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);

	while (__HAL_TIM_GET_COUNTER(&htim1) < us)
		;
}

#define stepsperrev 4096

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {

		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);

	}

	if (htim->Instance == TIM1) {

			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);

		}
	if (htim->Instance == TIM15) {

			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);

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
	//Servo servo;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	//HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim15);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // pull the TRIG pin low
	lcd_init();
	max_init(0x02);
	//Servo_Init(&servo, &htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	write_char(2, 1);
	lcd_put_cur(0, 0);
	lcd_send_string("---BIENVENIDO---");
	HAL_Delay(2000);
	max_clear();
	lcd_clear();
	write_char(63, 1);
	lcd_put_cur(0, 0);
	lcd_send_string("  SEL. OPCION  ");
	lcd_put_cur(1, 0);
	lcd_send_string("   SIGUIENTE   ");
	HAL_Delay(2000);
	lcd_clear();
	max_clear();
	lcd_put_cur(0, 0);
	lcd_send_string("  (A)SENSOR   ");
	lcd_put_cur(1, 0);
	lcd_send_string("(B)SERVO (C)M.PP");
	scroll_string((uint8_t*) "A B C  ", 100, left);

	while (1) {

		char keypad_key = Keypad_Get_Char();
		if (keypad_key == 'A') {
			lcd_clear();
			max_clear();
			write_char(65, 1);
			HAL_Delay(200);
			while (keypad_key == 'A') {
				HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET); // pull the TRIG pin HIGH
				__HAL_TIM_SET_COUNTER(&htim1, 0);
				while (__HAL_TIM_GET_COUNTER (&htim1) < 10)
					; // wait for 10 us
				HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // pull the TRIG pin low

				pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
				//wait for the echo pin to go high
				while (!(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN))
						&& pMillis + 10 > HAL_GetTick())
					;
				Value1 = __HAL_TIM_GET_COUNTER(&htim1);

				pMillis = HAL_GetTick();// used this to avoid infinite while loop (for timeout)
				// wait for the echo pin to go low
				while ((HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN))
						&& pMillis + 50 > HAL_GetTick())
					;

				Value2 = __HAL_TIM_GET_COUNTER(&htim1);

				lcd_put_cur(0, 0);
				lcd_send_string("Dist= ");
				Distance = (Value2 - Value1) * 0.034 / 2;

				lcd_send_string("Dist= ");
				Distance = (Value2 - Value1) * 0.034 / 2;
				// Limpia el espacio para la distancia
				lcd_put_cur(0, 6);// Establece el cursor en la columna 7, fila 0
				lcd_send_data(' ');					// Borra el espacio anterior

				// Muestra los dígitos de la distancia en el lugar deseado
				lcd_put_cur(0, 6);// Establece el cursor en la columna 7, fila 0
				lcd_send_data((Distance / 100) + 48);				// 100th pos
				lcd_send_data(((Distance / 10) % 10) + 48);			// 10th pos
				lcd_send_data((Distance % 10) + 48);				// 1st pos
				lcd_send_string(" cm");
				//HAL_Delay(800);

				key = Distance;
				char keyString[5];// Variable para almacenar el valor de key como cadena de caracteres
				static int previousKey = 0;	// Variable para almacenar el valor anterior de key
				//uint8_t change = 0;

				if (key < previousKey) {
					change = 0;
					//stepper_step_angle(90, change, 13);
					write_char(26, 1);
					HAL_Delay(500);
					max_clear();
					//lcd_put_cur(1, 0);
					//lcd_send_string("Disminuye");
					//HAL_Delay(1000);
				} else if (key > previousKey) {
					change = 1;
					//stepper_step_angle(90, change, 13);
					write_char(27, 1);
					HAL_Delay(500);
					max_clear();
					//lcd_put_cur(1, 0);
					//lcd_send_string("Aumenta");
					//HAL_Delay(1000);
				}
				static int currentAngle = 0; // Variable para almacenar la posición actual del stepper

					if (Distance != currentAngle) {
						int stepsToMove = abs(Distance - currentAngle);
						stepperDelay = stepsToMove;
						stepper_step_angle(stepsToMove, change, 13);
						currentAngle = Distance;

					}
					int mov = Distance * 3;
					htim2.Instance->CCR1 = 500 + (mov * 11.11111);



				snprintf(keyString, sizeof(keyString), "%d", key); // Convierte el valor de key a una cadena de caracteres
				lcd_clear(); // Borra la pantalla
				lcd_send_string(keyString); // Muestra el valor de key en la pantalla
				previousKey = key; // Actualiza el valor anterior de key

				if (Keypad_Get_Char() == '#') {
					lcd_clear();
					max_clear();
					break; // Exit the while loop and go back to the beginning
				}
			}
			lcd_put_cur(0, 0);
			lcd_send_string("  (A)SENSOR   ");
			lcd_put_cur(1, 0);
			lcd_send_string("(B)SERVO (C)M.PP");
		} else if (keypad_key == 'B') {
			lcd_clear();
			max_clear();
			write_char(66, 1);
			HAL_Delay(200);
			while (keypad_key == 'B') {
				lcd_put_cur(0, 0);
				lcd_send_string("---INT. ANGULO--");
				lcd_put_cur(1, 0);
				lcd_send_string("GRADOS:");
				write_char(132, 1);

				char digits[4]; // Variable para almacenar los dígitos ingresados (3 dígitos + el carácter nulo)
				int digitCount = 0; // Contador de dígitos

				while (digitCount < 3) {
					keypad = Keypad_Get_Char();

					if (keypad >= '0' && keypad <= '9') {
						digits[digitCount] = keypad;
						digitCount++;
						lcd_send_data(keypad);
					}

					// Si se presiona el botón 'D', se borra la pantalla LCD y se establece el ángulo del servo en 0 grados
					if (keypad == 'D') {
						lcd_clear();
						htim2.Instance->CCR1 = 500;  // 0.5ms = 0 grados
						break;
					}

					// Si se presiona el botón '#', se sale de la selección y vuelve al inicio
					if (keypad == '#') {
						lcd_clear();
						keypad_key = ' '; // Reiniciar la opción seleccionada para volver al inicio
						max_clear();
						break;

					}

				}

				if (digitCount == 3) {
					digits[digitCount] = '\0'; // Agregar el carácter nulo al final de los dígitos para formar una cadena válida
					int angle = atoi(digits); // Convertir los dígitos a un entero

					// Validar el rango del ángulo (por ejemplo, asegurarse de que esté entre 0 y 180 grados)
					if (angle >= 0 && angle <= 180) {
						int servoAngle = 500 + angle * 11.1111; // Calcular el valor correspondiente del ángulo para el servo
						htim2.Instance->CCR1 = servoAngle;
						HAL_Delay(3000);
						lcd_clear();
						lcd_put_cur(0, 0);
						lcd_send_string("---INT. ANGULO--");
						lcd_put_cur(1, 0);
						lcd_send_string("GRADOS:");
						write_char(132, 1);
						max_clear();

					} else {
						max_clear();
						lcd_clear();
						write_char(130, 1);
						lcd_put_cur(0, 0);
						lcd_send_string("Angulo fuera de ");
						lcd_put_cur(1, 0);
						lcd_send_string("rango ");
						HAL_Delay(2000);
						max_clear();
						lcd_put_cur(0, 0);
						lcd_send_string("---INT. ANGULO--");
						lcd_put_cur(1, 0);
						lcd_send_string("GRADOS:");
						write_char(132, 1);
						// Angulo fuera de rango, mostrar mensaje de error o tomar alguna acción adecuada

					}
				}
			}
			lcd_put_cur(0, 0);
			lcd_send_string("  (A)SENSOR   ");
			lcd_put_cur(1, 0);
			lcd_send_string("(B)SERVO (C)M.PP");

		} else if (keypad_key == 'C') {
			lcd_clear();
			max_clear();
			write_char(80, 1);
			HAL_Delay(200);
			while (keypad_key == 'C') {
				lcd_put_cur(0, 0);
				lcd_send_string("---INT. PASOS--");
				lcd_put_cur(1, 0);
				lcd_send_string("PASOS:");
				write_char(80, 1);

				char digits[4]; // Variable para almacenar los dígitos ingresados (3 dígitos + el carácter nulo)
				int digitCount = 0; // Contador de dígitos
				int exitFlag = 0; // Bandera para indicar si se debe salir completamente

				while (digitCount < 3) {
					keypad = Keypad_Get_Char();

					if (keypad >= '0' && keypad <= '9') {
						digits[digitCount] = keypad;
						digitCount++;
						lcd_send_data(keypad);

					}

					// Si se presiona el botón '#', se sale de la selección y vuelve al inicio
					if (keypad == '#') {
						lcd_clear();
						keypad_key = ' '; // Reiniciar la opción seleccionada para volver al inicio
						max_clear();
						exitFlag = 1; // Activar la bandera para salir completamente
						break;

					}

					// Verificar si se presionó la tecla D para eliminar el último dígito ingresado
					if (keypad == 'D') {
						if (digitCount > 0) {
							digitCount--; // Decrementar el índice de num1 para eliminar el último dígito
							lcd_send_cmd(0x10); // Mover el cursor a la posición anterior en la pantalla LCD
							lcd_send_data(' '); // Borrar el dígito anterior en la pantalla LCD
							lcd_send_cmd(0x10); // Mover el cursor nuevamente a la posición anterior

						}
					}
				}
				if (exitFlag) {
					break; // Salir completamente del bucle while (keypad_key == 'C')
				}

				lcd_put_cur(0, 0);
				lcd_send_string("  (A)SENSOR   ");
				lcd_put_cur(1, 0);
				lcd_send_string("(B)SERVO (C)M.PP");
				if (digitCount == 3) {
					digits[digitCount] = '\0'; // Agregar el carácter nulo al final de los dígitos para formar una cadena válida
					lcd_clear();
					lcd_put_cur(0, 0);
					lcd_send_string("Direccion:");
					lcd_put_cur(1, 0);
					lcd_send_string("0 - CW, 1 - CCW");
				}

				int direction = -1; // Valor inicial inválido para la dirección
				while (direction != 0 && direction != 1) {
					keypad = Keypad_Get_Char();

					if (keypad == '0' || keypad == '1') {
						direction = keypad - '0'; // Convertir el carácter a un entero (0 o 1)
						lcd_clear();
						lcd_put_cur(0, 0);
						lcd_send_string("MOTOR EN MOV.");

					}

					// Si se presiona el botón '#', se sale de la selección y vuelve al inicio
					if (keypad == '#') {
						lcd_clear();
						keypad_key = ' '; // Reiniciar la opción seleccionada para volver al inicio
						max_clear();
						exitFlag = 1; // Activar la bandera para salir completamente
						break;
					}
				}
				if (exitFlag) {
					break; // Salir completamente del bucle while (keypad_key == 'C')
				}
				int stepsToMove = atoi(digits); // Convertir los dígitos a un entero
				if (direction == 0) {
					max_clear();
					write_char(26, 1);
					// Dirección: CW (sentido de las agujas del reloj)
					// Mover el motor paso a paso en sentido CW aquí
					stepper_step_angle(stepsToMove, 1, 13);
					max_clear();
				} else if (direction == 1) {
					max_clear();
					write_char(27, 1);
					// Dirección: CCW (sentido contrario a las agujas del reloj)
					// Mover el motor paso a paso en sentido CCW aquí
					stepper_step_angle(stepsToMove, 0, 13);

					max_clear();
				}

			}
			lcd_put_cur(0, 0);
			lcd_send_string("  (A)SENSOR   ");
			lcd_put_cur(1, 0);
			lcd_send_string("(B)SERVO (C)M.PP");

		} else if (keypad_key != 0) {
			for (int i = 0; i < 1; i++) {
			}
			lcd_clear();
			lcd_enviar("Opcion invalida  ", 0, 0);
			lcd_send_string(&key);
			HAL_Delay(1000);
			lcd_clear();
			lcd_put_cur(0, 0);
			lcd_send_string("  (A)SENSOR   ");
			lcd_put_cur(1, 0);
			lcd_send_string("(B)SERVO (C)M.PP");
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM15;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x2000090E;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim15.Init.Prescaler = 72-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 7000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  huart2.Init.BaudRate = 38400;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN2_Pin|GPIO_PIN_4|IN4_Pin|R1_Pin
                          |GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN3_Pin|R2_Pin|LD2_Pin|GPIO_PIN_14
                          |GPIO_PIN_15|R3_Pin|R4_Pin|GPIO_PIN_6
                          |IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN2_Pin PA4 IN4_Pin R1_Pin
                           PA11 */
  GPIO_InitStruct.Pin = IN2_Pin|GPIO_PIN_4|IN4_Pin|R1_Pin
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN3_Pin R2_Pin LD2_Pin PB14
                           PB15 R3_Pin R4_Pin PB6
                           IN1_Pin */
  GPIO_InitStruct.Pin = IN3_Pin|R2_Pin|LD2_Pin|GPIO_PIN_14
                          |GPIO_PIN_15|R3_Pin|R4_Pin|GPIO_PIN_6
                          |IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : C4_Pin */
  GPIO_InitStruct.Pin = C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(C4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : C3_Pin C2_Pin C1_Pin */
  GPIO_InitStruct.Pin = C3_Pin|C2_Pin|C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {
 if (htim->Instance == TIM1)
 {
 keypad_keyY = Keypad_Get_Char(); // Obtén el valor de la tecla del teclado

 if (keypad_keyY == 'B')
 {
 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
 }

 // Agrega aquí tu código adicional para ejecutar stepper_step_angle() u otras operaciones relacionadas
 }
 }*/
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
