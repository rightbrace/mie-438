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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
	uint32_t  red;
	uint32_t  green;
	uint32_t  blue;
	uint8_t*  data;
} Reading;


Reading KnownValues[] = {
		{2949, 3693, 2573, "Red"},
//		{3073, 2848, 2533, "Green"},
		{2905, 2649, 2616, "Blue"},
		{3425, 3763, 2728, "Yellow"},
};

char paused = 1;

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) > (b) ? (a) : (b))

uint32_t CalculateDistance(Reading a, Reading b) {
	// You would think this would be an issue with unsigned values, but it just works (tm)
	return (a.red - b.red) * (a.red - b.red) + (a.green - b.green) * (a.green - b.green) + (a.blue - b.blue) * (a.blue - b.blue);
}

void SetColor(bool red, bool green, bool blue) {

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, !blue);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, !red);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, !green);

}

uint32_t ReadAnalog() {
	uint32_t result = 0;
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) {
		result = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);

	return result;
}

void CaptureReading(Reading* result) {

	const int CHARGE_DELAY = 150;
	const int DISCHARGE_DELAY = 150;

	SetColor(1, 0, 0);
	HAL_Delay(CHARGE_DELAY);
	result->red = ReadAnalog();
	SetColor(0, 0, 0);
	HAL_Delay(DISCHARGE_DELAY);

	SetColor(0, 1, 0);
	HAL_Delay(CHARGE_DELAY);
	result->green = ReadAnalog();
	SetColor(0, 0, 0);
	HAL_Delay(DISCHARGE_DELAY);

	SetColor(0, 0, 1);
	HAL_Delay(CHARGE_DELAY);
	result->blue = ReadAnalog();
	SetColor(0, 0, 0);

}

void SetServoAngle(unsigned long channel, uint32_t pulseMicros) {
	// Tbe timer is configured to output a 50Hz (20,000 us) square wave,
	// where the channel's compare value is the high time in us.
	//
	// Servos have nominal pulse-width to angle mapping, but they're not perfect.
	// Better to check specific pulse lengths required to get the settings we need.
	// Generally, 900-2100us is supposed to represent the range of motion
	//
	// Channel is one of three values, each corresponding to a pin:
	// TIM_CHANNEL_1 = PA_6 = A5
	// TIM_CHANNEL_2 = PB_5 = D11 AND D5???
	// TIM_CHANNEL_3 = PB_0 = D3
	__HAL_TIM_SET_COMPARE(&htim3, channel, pulseMicros);
}

uint32_t ChutePulseMicros[] = {
		530,
		640,
		805,
		1035,
		1240,
		1440,
		1650,
		1850,
		2050,
};


uint32_t DistributorSenseMicros = 500;
uint32_t DistributorDropMicros = 2600;

void SetDistributorPosition(uint32_t position) {
	SetServoAngle(TIM_CHANNEL_1, position);
}

void SendChuteTo(int destinationChute) {
	SetServoAngle(TIM_CHANNEL_2, ChutePulseMicros[destinationChute]);
}

uint32_t ResetBarClosedMicros = 900;
uint32_t ResetBarOpenMicros = 1500;

void SetResetBar(int open) {
	SetServoAngle(TIM_CHANNEL_3, open ? ResetBarOpenMicros : ResetBarClosedMicros);
}


unsigned char PrintBuffer[256];

unsigned char rxBuffer[256] = {0};
// Index of next byte to be read
unsigned char uartReadIdx = 0;
// Index of next byte to be written
unsigned char uartWriteIdx = 0;
// Whether the write pointer has passed the read pointer
unsigned char uartOverflow = 0;

unsigned char uartAvailable() {
	return uartWriteIdx != uartReadIdx;
}

unsigned char uartRead() {
	return rxBuffer[uartReadIdx++];
}

void uartAcknowledgeOverflow() {
	uartOverflow = 0;
}

unsigned char ImageData[16] = {0};
unsigned char ColumnHeights[8] = {0};

unsigned char GetPixelColor(unsigned char x, unsigned char y) {
	unsigned char idx = y * 2 + x / 4;
	unsigned char color = 0b11 & (ImageData[idx] >> (6-(x%4)*2));
	while (color > 3);
	return color;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uartWriteIdx ++; // Will wrap automatically
	if (uartWriteIdx == uartReadIdx) uartOverflow = 1;
	HAL_UART_Receive_IT(&huart2, &rxBuffer[uartWriteIdx], 1);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // Setup UART interrupt
  HAL_UART_Receive_IT(&huart2, &rxBuffer[uartWriteIdx], 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);


  // Setup dropper to closed
  SetResetBar(0);

  // Set distributor to grab a ball
  SetDistributorPosition(DistributorDropMicros);
  HAL_Delay(1000);
  SetDistributorPosition(DistributorSenseMicros);
  HAL_Delay(1000);

  while (1)
  {

//	  while(1) {
//		  for (int i = 0; i < 9; i++) {
//
//			  SendChuteTo(i);
//			  sprintf(PrintBuffer, "Chute: %d, value: %d\n", i, ChutePulseMicros[i]);
//			  HAL_UART_Transmit(&huart2, PrintBuffer, strlen(PrintBuffer), 1000);
//		  }
//	  }

	  if (uartOverflow) {
		  sprintf(PrintBuffer, "Inbound buffer overflow\n");
		  HAL_UART_Transmit(&huart2, PrintBuffer, strlen(PrintBuffer), 1000);
		  while (1);
	  }

	  // Check for commands from PC
	  if (uartAvailable()) {

		  char v = uartRead();

		  if (v == 'i') { // Image data


			  sprintf(PrintBuffer, "Receiving new image data\n");
			  HAL_UART_Transmit(&huart2, PrintBuffer, strlen(PrintBuffer), 1000);

			  for (int i = 0; i < 16; i++) {
				  ImageData[i] = uartRead();
			  }

			  // Drop reset bar
			  SetResetBar(1);
			  HAL_Delay(1000);
			  SetResetBar(0);
			  HAL_Delay(1000);

			  // Clear distributor
			  SetDistributorPosition(DistributorDropMicros);
			  HAL_Delay(1000);
			  SetDistributorPosition(DistributorSenseMicros);
			  HAL_Delay(1000);

			  // Clear stacks
			  for (int i = 0; i < 8; i++) ColumnHeights[i] = 0;

			  paused = 1;

		  } if (v == 'p') { // pause
			  paused = !paused;
			  sprintf(PrintBuffer, paused ? "Stop\n" : "Start \n");
			  HAL_UART_Transmit(&huart2, PrintBuffer, strlen(PrintBuffer), 1000);
		  }

		  // Await terminator
		  if (uartRead() != ';') {
			  sprintf(PrintBuffer, "Message termination Error\n");
			  HAL_UART_Transmit(&huart2, PrintBuffer, strlen(PrintBuffer), 1000);
			  while (1);
		  }

	  }

	  if (paused) {
		  continue;
	  }

	  // Read current candy color
	  Reading reading;
	  CaptureReading(&reading);
	  uint32_t closestDist = 0xffffffff;
	  int closestReadingIdx = 0;
	  for (int i = 0; i < sizeof(KnownValues) / sizeof(*KnownValues); i++) {
		  uint32_t dist = CalculateDistance(KnownValues[i], reading);
		  if (dist < closestDist) {
			  closestDist = dist;
			  closestReadingIdx = i;
		  }
	  }

	  sprintf(PrintBuffer, "%d, %d, %d, \"%s\" (guess)\n", reading.red, reading.green, reading.blue, KnownValues[closestReadingIdx].data);
	  HAL_UART_Transmit(&huart2, PrintBuffer, strlen(PrintBuffer), 1000);

	  // Select appropriate destination and send chute there, marking slot in array
	  int chute = 8; // Drop in last chute if no matches
	  for (int x = 0; x < 8; x++) {
		  int y = 7 - ColumnHeights[x];
		  if (y < 0) continue;
		  if (GetPixelColor(x, y) == closestReadingIdx) {
			  chute = x;
			  ColumnHeights[chute]++;
			  break;
		  }
	  }

	  sprintf(PrintBuffer, "Sending %s ball to chute %d\n", KnownValues[closestReadingIdx].data, chute);
	  HAL_UART_Transmit(&huart2, PrintBuffer, strlen(PrintBuffer), 1000);

	  SendChuteTo(chute);

	  // Render out current model of the output.
	  // Lower case is to-be-placed, upper is already-placed balls
	  char *buf = PrintBuffer;
	  for (int y = 0; y < 8; y++) {
		  for (int x = 0; x < 8; x++) {
			  int color = GetPixelColor(x, y);
			  char symbol = KnownValues[color].data[0];
			  if (y < (8-ColumnHeights[x])) {
				  symbol += 'a' - 'A';
			  }
			  *buf = symbol;
			  buf++;
		  }
		  *buf = '\n';
		  buf++;
	  }
	  *buf = 0;
	  HAL_UART_Transmit(&huart2, PrintBuffer, strlen(PrintBuffer), 1000);


	  // Wait small amount
	  HAL_Delay(350);

	  // Rotate distributor into dropping position
	  SetDistributorPosition(DistributorDropMicros);

	  // Wait small amount
	  HAL_Delay(750);

	  // Rotate distributor into sensing position
	  SetDistributorPosition(DistributorSenseMicros);

	  // Wait small amount
	  HAL_Delay(750);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  htim1.Init.Prescaler = 8;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA3 PA4 PA5
                           PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
