/* USER CODE BEGIN Header */
/*
 * main.c
 * Copyright (C) Jo�o Melga <joaolucasfm@gmail.com>
 *
 * Version 1.0
 *
 * This software controls an STM32f103c8t6 microcontroller manage
 * aerators engines according to luminosity and print messages via
 * UART.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the Licence, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. Se the
 * GNU General Public Licence for more details.
 *
 * Created on: feb-03-2020
 *
 * This software is designed to run under STM32 System Workbench IDE + CubeMX.
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "StateMachine.h"
#include "eeprom.h"

#define DEAFULT_LUMINOSITY_TRIGGER 3000
#define MAX_MSG_GAP 750000 // 15 minutes
#define MIN_MSG_GAP 7500 // 7,5 seconds

#define ERROR_STATE 0
#define MOTOR_ON_STATE 1
#define MOTOR_OFF_STATE 2
#define MOTOR_SWITCHING_ON_STATE 3
#define MOTOR_SWITCHING_OFF_STATE 4
#define MOTOR_SWITCHING_VERIFYING_STATE 5
#define BOOTING_STATE 6

#define NO_MSG 0
#define MOTOR_ON_AUTO_MSG 1
#define MOTOR_OFF_AUTO_MSG 2
#define MOTOR_ON_MANUALY_MSG 3
#define MOTOR_OFF_MANUALY_MSG 4
#define MOTOR_ERROR_MSG 5
#define AUTOMATIC_MODE_ON_MSG 6
#define AUTOMATIC_MODE_OFF_MSG 7
#define BOOTING_MSG 8

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

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void UART_TransmitMessage(char *);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// StateMachine stateMachine;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t luminosity = 0, luminosityTrigger = DEAFULT_LUMINOSITY_TRIGGER, state = BOOTING_STATE;
	uint8_t motorVoltageDetected, automaticModeEnabled = 1, messageType = BOOTING_MSG;
	unsigned int lastMsgTimestamp = - MIN_MSG_GAP, timePassedSinceLastMsg = 0;
	GPIO_PinState lowLuminosityIndicator;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_FLASH_Unlock();
  EE_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  EE_ReadVariable(VirtAddVarTab[0], &luminosityTrigger);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1);
		luminosity = (luminosity * 99 + HAL_ADC_GetValue(&hadc1)) / 100;
		HAL_ADC_Stop(&hadc1);
		HAL_Delay(1);

		motorVoltageDetected = HAL_GPIO_ReadPin(GPIOB, MOTOR_STATUS_Pin);
		lowLuminosityIndicator = luminosity < luminosityTrigger ? GPIO_PIN_SET : GPIO_PIN_RESET;
		timePassedSinceLastMsg = HAL_GetTick() - lastMsgTimestamp;

		HAL_GPIO_WritePin(GPIOA, LED_D1_Pin, HAL_GPIO_ReadPin(GPIOB, MOTOR_STATUS_Pin));
		HAL_GPIO_WritePin(GPIOA, LED_D2_Pin, automaticModeEnabled);
		HAL_GPIO_WritePin(GPIOA, LED_D3_Pin, lowLuminosityIndicator);

		if (HAL_GPIO_ReadPin(GPIOB, BUTTON_D3_Pin)) {
			HAL_ADCEx_Calibration_Start(&hadc1);
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 1);
			luminosityTrigger = HAL_ADC_GetValue(&hadc1);

			EE_WriteVariable(VirtAddVarTab[0], luminosityTrigger);

			HAL_Delay(300);
			while (HAL_GPIO_ReadPin(GPIOB, BUTTON_D3_Pin));
		}
		else if (HAL_GPIO_ReadPin(GPIOB, BUTTON_D2_Pin)) {
			automaticModeEnabled = !automaticModeEnabled;

			HAL_Delay(300);
			while (HAL_GPIO_ReadPin(GPIOB, BUTTON_D2_Pin));

			messageType = automaticModeEnabled ? AUTOMATIC_MODE_ON_MSG : AUTOMATIC_MODE_OFF_MSG;
		}

		if ((messageType != NO_MSG && timePassedSinceLastMsg > MIN_MSG_GAP) || timePassedSinceLastMsg > MAX_MSG_GAP) {
			char* message;
			asprintf(&message, " Motor1,%i,%i,%i,%i,%i,%i ", messageType, motorVoltageDetected, luminosity, luminosityTrigger, automaticModeEnabled, state);

			UART_TransmitMessage(message);
			HAL_Delay(1000);

			messageType = NO_MSG;
			lastMsgTimestamp = HAL_GetTick();
		}

		switch (state) {
		case BOOTING_STATE: {
			state = motorVoltageDetected ? MOTOR_ON_STATE : MOTOR_OFF_STATE;
			break;
		}
		case MOTOR_ON_STATE:
			if (!motorVoltageDetected) {
				state = MOTOR_OFF_STATE;
				messageType = MOTOR_OFF_MANUALY_MSG;
			}
			else if (automaticModeEnabled && luminosity > luminosityTrigger)
				state = (MOTOR_SWITCHING_OFF_STATE);

			break;

		case MOTOR_OFF_STATE:
			if (motorVoltageDetected) {
				state = (MOTOR_ON_STATE);
				messageType = MOTOR_ON_MANUALY_MSG;
			}
			else if (automaticModeEnabled && luminosity <= luminosityTrigger)
				state = (MOTOR_SWITCHING_ON_STATE);

			break;

		case MOTOR_SWITCHING_ON_STATE:
			HAL_GPIO_WritePin(GPIOA, MOTOR_START_Pin, GPIO_PIN_SET);

			for (int i = 0; i < 30 && !motorVoltageDetected; i++) {
				HAL_GPIO_WritePin(GPIOA, LED_D1_Pin, GPIO_PIN_SET);
				HAL_Delay(50);
				HAL_GPIO_WritePin(GPIOA, LED_D1_Pin, GPIO_PIN_RESET);
				HAL_Delay(50);

				motorVoltageDetected = HAL_GPIO_ReadPin(GPIOB, MOTOR_STATUS_Pin);
			}

			HAL_GPIO_WritePin(GPIOA, MOTOR_START_Pin, GPIO_PIN_RESET);
			state = (MOTOR_SWITCHING_VERIFYING_STATE);
			break;

		case MOTOR_SWITCHING_OFF_STATE:
			HAL_GPIO_WritePin(GPIOA, MOTOR_STOP_Pin, GPIO_PIN_SET);

			for (int i = 0; i < 20 && motorVoltageDetected; i++) {
				HAL_GPIO_WritePin(GPIOA, LED_D1_Pin, GPIO_PIN_SET);
				HAL_Delay(50);
				HAL_GPIO_WritePin(GPIOA, LED_D1_Pin, GPIO_PIN_RESET);
				HAL_Delay(50);

				motorVoltageDetected = HAL_GPIO_ReadPin(GPIOB, MOTOR_STATUS_Pin);
			}

			HAL_GPIO_WritePin(GPIOA, MOTOR_STOP_Pin, GPIO_PIN_RESET);
			state = (MOTOR_SWITCHING_VERIFYING_STATE);
			break;

		case MOTOR_SWITCHING_VERIFYING_STATE:
			if (!motorVoltageDetected && luminosity > luminosityTrigger) {
				state = (MOTOR_OFF_STATE);
				messageType = MOTOR_OFF_AUTO_MSG;
			}
			else if (motorVoltageDetected && luminosity <= luminosityTrigger) {
				state = (MOTOR_ON_STATE);
				messageType = MOTOR_ON_AUTO_MSG;
			}
			else {
				state = (ERROR_STATE);
				messageType = MOTOR_ERROR_MSG;
			}

			break;

		case ERROR_STATE:
			HAL_GPIO_WritePin(GPIOA, LED_D1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, LED_D2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, LED_D3_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOA, LED_D1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, LED_D2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, LED_D3_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);

			if (!automaticModeEnabled || (!motorVoltageDetected && luminosity > luminosityTrigger)) {
				state = (MOTOR_OFF_STATE);
				messageType = automaticModeEnabled ? MOTOR_OFF_AUTO_MSG : MOTOR_OFF_MANUALY_MSG;
			}
			else if (!automaticModeEnabled || (motorVoltageDetected && luminosity <= luminosityTrigger)) {
				state = (MOTOR_ON_STATE);
				messageType = automaticModeEnabled ? MOTOR_ON_AUTO_MSG : MOTOR_ON_MANUALY_MSG;
			}

			break;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  htim1.Init.Prescaler = 799;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_START_Pin|MOTOR_STOP_Pin|LED_D1_Pin|LED_D2_Pin
                          |LED_D3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_D1_GPIO_Port, BUZZER_D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOTOR_START_Pin MOTOR_STOP_Pin LED_D1_Pin LED_D2_Pin
                           LED_D3_Pin */
  GPIO_InitStruct.Pin = MOTOR_START_Pin|MOTOR_STOP_Pin|LED_D1_Pin|LED_D2_Pin
                          |LED_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_D1_Pin */
  GPIO_InitStruct.Pin = BUTTON_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BUTTON_D1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_D3_Pin MOTOR_STATUS_Pin BUTTON_D2_Pin */
  GPIO_InitStruct.Pin = BUTTON_D3_Pin|MOTOR_STATUS_Pin|BUTTON_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_D1_Pin */
  GPIO_InitStruct.Pin = BUZZER_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_D1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void UART_TransmitMessage(char *message) {
	HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 1000);
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/