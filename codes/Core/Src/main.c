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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "eeprom.h"
#include "ssd1306.h"
#include "fonts.h"

#define ADC_RESOLUTION 3.3/4095 // 3.3 VCC divided by 2^12 = 4096, because ADC has 12bits
#define DEAFULT_LUMINOSITY_TRIGGER 3000
#define LDR_RESISTENCE_TRIGGER 5000 // Resistence in Ohms
#define MAX_MSG_GAP 750000 // 15 minutes
#define MIN_MSG_GAP 7500 // 7,5 seconds
#define ANIMATION_GAP 200
#define MOTOR_TOGGLE_MAX_TIME 3000 // 3 seconds
#define MOTOR_TOGGLE_DELAY 5000 // 5 seconds

#define ICONS_X 40
#define ICONS_Y 34
#define TEXT_Y 18

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

#define AERATOR_ICON 0
#define LIGHT_ICON 1
#define STARTER_ICON 2
#define RAY_ICON 3
#define PROPELLER_1_ICON 4
#define PROPELLER_2_ICON 5

#define CIRCLE_BTN 0
#define CROSS_BTN 1

#define DASHBOARD_VIEW 0
#define SETTINGS_VIEW 1

#define DISPLAY_ARROW_POSITIONS 4
#define NUMBER_OF_SETTINGS 5

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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* Definitions for AeratorsTask */
osThreadId_t AeratorsTaskHandle;
const osThreadAttr_t AeratorsTask_attributes = { .name = "AeratorsTask",
    .priority = (osPriority_t) osPriorityNormal, .stack_size = 128 * 4 };
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = { .name = "DisplayTask",
    .priority = (osPriority_t) osPriorityLow, .stack_size = 128 * 4 };
/* Definitions for WiFiTask */
osThreadId_t WiFiTaskHandle;
const osThreadAttr_t WiFiTask_attributes = { .name = "WiFiTask", .priority =
    (osPriority_t) osPriorityLow, .stack_size = 128 * 4 };
/* Definitions for ButtonsTask */
osThreadId_t ButtonsTaskHandle;
const osThreadAttr_t ButtonsTask_attributes = { .name = "ButtonsTask",
    .priority = (osPriority_t) osPriorityLow, .stack_size = 128 * 4 };

/* USER CODE BEGIN PV */

/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NB_OF_VAR] = { 0x5555 };
float relativeLuminosity = 0;
uint32_t ldrResistence = 0;
uint16_t luminosityTrigger = LDR_RESISTENCE_TRIGGER;
uint8_t motorVoltageDetected, automaticModeEnabled = 1, messageType =
BOOTING_MSG;
GPIO_PinState lowLuminosityDetected;

uint8_t state = 0;
uint32_t btnPressedFlag = 0;
uint8_t btnPressedName = CIRCLE_BTN;
uint8_t currentView = DASHBOARD_VIEW;
uint8_t arrowDisplayPosition = 0;
uint8_t arrowSettingsPosition = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == CIRCLE_BTN_Pin)
    btnPressedName = CIRCLE_BTN;
  else if (GPIO_Pin == CROSS_BTN_Pin)
    btnPressedName = CROSS_BTN;
  else
    return;

  btnPressedFlag = 1;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
void StartAeratorsTask(void *argument);
void StartDisplayTask(void *argument);
void StartWiFiTask(void *argument);
void StartButtonsTask(void *argument);

/* USER CODE BEGIN PFP */
static uint32_t LDR_GetResistence();
static void ADC_ConfigChannel(uint32_t);
static void UART_TransmitMessage(char *);
static void SSD1306_PutIcon(uint8_t, uint8_t, uint8_t, uint8_t);
static void SSD1306_PutHeaderButtons(char*, char*);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// StateMachine stateMachine;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
  HAL_FLASH_Unlock();
  EE_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  EE_ReadVariable(VirtAddVarTab[0], &luminosityTrigger);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of AeratorsTask */
  AeratorsTaskHandle = osThreadNew(StartAeratorsTask, NULL,
      &AeratorsTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL,
      &DisplayTask_attributes);

  /* creation of WiFiTask */
  WiFiTaskHandle = osThreadNew(StartWiFiTask, NULL, &WiFiTask_attributes);

  /* creation of ButtonsTask */
  ButtonsTaskHandle = osThreadNew(StartButtonsTask, NULL,
      &ButtonsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    /* if ((messageType != NO_MSG && timePassedSinceLastMsg > MIN_MSG_GAP) || timePassedSinceLastMsg > MAX_MSG_GAP) {
     char* message;
     sprintf(&message, " Motor1,%i,%i,%i,%i,%i,%i ", messageType, motorVoltageDetected, luminosity, luminosityTrigger, automaticModeEnabled, state);

     UART_TransmitMessage(message);
     HAL_Delay(1000);

     messageType = NO_MSG;
     lastMsgTimestamp = HAL_GetTick();
     } */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
  RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = { 0 };

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
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }
  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
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
static void MX_TIM1_Init(void) {

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
  TIM_MasterConfigTypeDef sMasterConfig = { 0 };

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 799;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
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
static void MX_USART1_UART_Init(void) {

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
  if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE()
  ;
  __HAL_RCC_GPIOD_CLK_ENABLE()
  ;
  __HAL_RCC_GPIOA_CLK_ENABLE()
  ;
  __HAL_RCC_GPIOB_CLK_ENABLE()
  ;

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA,
  MOTOR_START_Pin | MOTOR_STOP_Pin | LED_D1_Pin | LED_D2_Pin | LED_D3_Pin,
      GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_D1_GPIO_Port, BUZZER_D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOTOR_START_Pin MOTOR_STOP_Pin LED_D1_Pin LED_D2_Pin
   LED_D3_Pin */
  GPIO_InitStruct.Pin = MOTOR_START_Pin | MOTOR_STOP_Pin | LED_D1_Pin
      | LED_D2_Pin | LED_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_D1_Pin */
  GPIO_InitStruct.Pin = BUTTON_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BUTTON_D1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CROSS_BTN_Pin CIRCLE_BTN_Pin */
  GPIO_InitStruct.Pin = CROSS_BTN_Pin | CIRCLE_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_D1_Pin */
  GPIO_InitStruct.Pin = BUZZER_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_D1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_STATUS_Pin */
  GPIO_InitStruct.Pin = MOTOR_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MOTOR_STATUS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
static void MOTOR_WriteState() {
  switch (state) {
  case MOTOR_SWITCHING_ON_STATE:
    HAL_GPIO_WritePin(GPIOA, MOTOR_START_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, MOTOR_STOP_Pin, GPIO_PIN_SET);
    break;

  case MOTOR_SWITCHING_OFF_STATE:
    HAL_GPIO_WritePin(GPIOA, MOTOR_START_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, MOTOR_STOP_Pin, GPIO_PIN_RESET);
    break;

  default:
    HAL_GPIO_WritePin(GPIOA, MOTOR_START_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, MOTOR_STOP_Pin, GPIO_PIN_SET);
    break;
  }
}

static uint32_t LDR_GetResistence() {
  uint32_t adcValue = 0, resistence;
  float serieResistorTension;

  if (HAL_ADC_GetState(&hadc1) != HAL_ADC_STATE_REG_BUSY) {
    ADC_ConfigChannel(ADC_CHANNEL_0);
    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
      adcValue = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);
  }

  serieResistorTension = adcValue * ADC_RESOLUTION;
  resistence = ((3.3 * 10000) / serieResistorTension) - 10000;
  return resistence >= 0 ? resistence : 0;
}

static void ADC_ConfigChannel(uint32_t channel) {
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    Error_Handler();
}

static void UART_TransmitMessage(char *message) {
  HAL_UART_Transmit(&huart1, (uint8_t*) message, strlen(message), 1000);
}

static void SSD1306_PutIcon(uint8_t x, uint8_t y, uint8_t icon, uint8_t color) {
  switch (icon) {
  case AERATOR_ICON:
    SSD1306_DrawFilledRectangle(x + 7, y + 0, 8, 10, color);
    SSD1306_DrawRectangle(x + 7, y + 6, 8, 12, color);
    SSD1306_DrawFilledRectangle(x + 0, y + 16, 22, 4, color);
    SSD1306_DrawLine(x + 11, y + 12, x + 11, y + 28, color);
    break;

  case LIGHT_ICON:
    SSD1306_DrawLine(x + 2, y + 2, x + 22, y + 22, color);
    SSD1306_DrawLine(x + 22, y + 2, x + 2, y + 22, color);
    SSD1306_DrawLine(x + 12, y + 0, x + 12, y + 24, color);
    SSD1306_DrawLine(x + 0, y + 12, x + 24, y + 12, color);
    SSD1306_DrawFilledCircle(x + 12, y + 12, 8, !color);
    SSD1306_DrawFilledCircle(x + 12, y + 12, 6, color);
    break;

  case STARTER_ICON:
    SSD1306_DrawRectangle(x, y, 16, 24, color);
    SSD1306_DrawCircle(x + 5, y + 7, 2, color);
    SSD1306_DrawFilledCircle(x + 5, y + 14, 2, color);
    break;

  case PROPELLER_1_ICON:
    SSD1306_DrawTriangle(x - 2, y, x - 6, y - 2, x - 6, y + 2, color);
    SSD1306_DrawTriangle(x + 2, y, x + 6, y - 2, x + 6, y + 2, color);
    break;

  case PROPELLER_2_ICON:
    SSD1306_DrawTriangle(x - 2, y, x - 4, y - 2, x - 4, y + 2, color);
    SSD1306_DrawTriangle(x + 2, y, x + 4, y - 2, x + 4, y + 2, color);
    break;
  }
}

static void SSD1306_PutHeaderButtons(char *text1, char *text2) {
#define BUTTONS_X 0
#define BUTTONS_Y 0
#define BUTTON_X_OFFSET_1 (BUTTONS_X + 64)

  SSD1306_DrawFilledRectangle(BUTTONS_X + 0, BUTTONS_Y + 0,
  BUTTON_X_OFFSET_1 - 3, 12, 1);
  SSD1306_DrawFilledCircle(BUTTONS_X + 5, BUTTONS_Y + 6, 4, 0);
  SSD1306_GotoXY(BUTTONS_X + 12, BUTTONS_Y + 3);
  SSD1306_Puts(text1, &Font_7x10, 0);

  SSD1306_DrawFilledRectangle(BUTTON_X_OFFSET_1 + 0, BUTTONS_Y + 0,
      128 - BUTTON_X_OFFSET_1, 12, 1);
  SSD1306_DrawLine(BUTTON_X_OFFSET_1 + 2 + 1, BUTTONS_Y + 3 + 0,
  BUTTON_X_OFFSET_1 + 2 + 7, BUTTONS_Y + 3 + 6, 0);
  SSD1306_DrawLine(BUTTON_X_OFFSET_1 + 2 + 1, BUTTONS_Y + 3 + 1,
  BUTTON_X_OFFSET_1 + 2 + 6, BUTTONS_Y + 3 + 6, 0);
  SSD1306_DrawLine(BUTTON_X_OFFSET_1 + 2 + 0, BUTTONS_Y + 3 + 1,
  BUTTON_X_OFFSET_1 + 2 + 6, BUTTONS_Y + 3 + 7, 0);
  SSD1306_DrawLine(BUTTON_X_OFFSET_1 + 2 + 0, BUTTONS_Y + 3 + 6,
  BUTTON_X_OFFSET_1 + 2 + 6, BUTTONS_Y + 3 + 0, 0);
  SSD1306_DrawLine(BUTTON_X_OFFSET_1 + 2 + 1, BUTTONS_Y + 3 + 6,
  BUTTON_X_OFFSET_1 + 2 + 6, BUTTONS_Y + 3 + 1, 0);
  SSD1306_DrawLine(BUTTON_X_OFFSET_1 + 2 + 1, BUTTONS_Y + 3 + 7,
  BUTTON_X_OFFSET_1 + 2 + 7, BUTTONS_Y + 3 + 1, 0);
  SSD1306_GotoXY(BUTTON_X_OFFSET_1 + 12, BUTTONS_Y + 3);
  SSD1306_Puts(text2, &Font_7x10, 0);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartAeratorsTask */
/**
 * @brief  Function implementing the AeratorsTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartAeratorsTask */
void StartAeratorsTask(void *argument) {
  /* USER CODE BEGIN 5 */
  GPIO_PinState lowLuminosityDetected;
  uint32_t timeSinceLastMotorToggle = HAL_GetTick();
  uint32_t lastMotorToggleTimestamp = 0;

  /* Infinite loop */
  for (;;) {
    ldrResistence = LDR_GetResistence();
    relativeLuminosity = (relativeLuminosity * 19
        + (float) luminosityTrigger / ldrResistence) / 20;
    motorVoltageDetected = HAL_GPIO_ReadPin(GPIOB, MOTOR_STATUS_Pin);

    lowLuminosityDetected =
        motorVoltageDetected ?
            (relativeLuminosity > 1.1 ? GPIO_PIN_RESET : GPIO_PIN_SET) :
            (relativeLuminosity < 0.9 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    timeSinceLastMotorToggle = HAL_GetTick() - lastMotorToggleTimestamp;

    switch (state) {
    case BOOTING_STATE:
      MOTOR_WriteState();
      state = motorVoltageDetected ? MOTOR_ON_STATE : MOTOR_OFF_STATE;
      break;

    case MOTOR_ON_STATE:
      MOTOR_WriteState();

      if (!motorVoltageDetected)
        state = MOTOR_OFF_STATE;
      else if (automaticModeEnabled && !lowLuminosityDetected) {
        lastMotorToggleTimestamp = HAL_GetTick();
        state = MOTOR_SWITCHING_OFF_STATE;
      }

      break;

    case MOTOR_OFF_STATE:
      MOTOR_WriteState();

      if (motorVoltageDetected)
        state = MOTOR_ON_STATE;
      else if (automaticModeEnabled && lowLuminosityDetected) {
        lastMotorToggleTimestamp = HAL_GetTick();
        state = MOTOR_SWITCHING_ON_STATE;
      }

      break;

    case MOTOR_SWITCHING_ON_STATE:
      if (motorVoltageDetected)
        state = MOTOR_ON_STATE;

      else if (timeSinceLastMotorToggle
          > MOTOR_TOGGLE_MAX_TIME + MOTOR_TOGGLE_DELAY)
        state = ERROR_STATE;

      else if (timeSinceLastMotorToggle > MOTOR_TOGGLE_DELAY)
        MOTOR_WriteState();

      break;

    case MOTOR_SWITCHING_OFF_STATE:
      if (!motorVoltageDetected)
        state = MOTOR_OFF_STATE;

      else if (timeSinceLastMotorToggle
          > MOTOR_TOGGLE_MAX_TIME + MOTOR_TOGGLE_DELAY)
        state = ERROR_STATE;

      else if (timeSinceLastMotorToggle > MOTOR_TOGGLE_DELAY)
        MOTOR_WriteState();

      break;

    case ERROR_STATE:
      MOTOR_WriteState();

      if (!automaticModeEnabled)
        state = BOOTING_STATE;
      else if (!motorVoltageDetected && !lowLuminosityDetected)
        state = MOTOR_OFF_STATE;
      else if (motorVoltageDetected && lowLuminosityDetected)
        state = MOTOR_ON_STATE;
      break;
    }

    osDelay(20);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
 * @brief Function implementing the DisplayTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument) {
  /* USER CODE BEGIN StartDisplayTask */
  uint8_t propellerFrame = 0;
  uint8_t starterFrame = 0;
  char displayText[50];

  SSD1306_Init();
  SSD1306_ToggleOpposed();

  SSD1306_DrawFilledRectangle(0, 0, 128, 64, 1);
  SSD1306_GotoXY(17, 20);
  SSD1306_Puts("Aerators", &Font_11x18, 0);
  SSD1306_GotoXY(7, 39);
  SSD1306_Puts("Controller", &Font_11x18, 1);

  SSD1306_UpdateScreen();
  osDelay(2000);
  SSD1306_DrawFilledRectangle(0, 0, 128, 64, 0);

  // Print icons and buttons
  SSD1306_PutIcon(12, ICONS_Y, STARTER_ICON, 1);
  SSD1306_PutIcon(53, ICONS_Y, AERATOR_ICON, 1);
  SSD1306_PutIcon(96, ICONS_Y, LIGHT_ICON, 1);

  // Print
  SSD1306_PutHeaderButtons("Modo", "Calibr.");

  SSD1306_UpdateScreen();

  /* Infinite loop */
  for (;;) {
    // switch (currentView) {
    // case DASHBOARD_VIEW: {
    SSD1306_DrawFilledRectangle(0, TEXT_Y, 128, ICONS_Y - TEXT_Y - 1, 0);

    SSD1306_GotoXY(9, TEXT_Y);
    SSD1306_Puts(automaticModeEnabled ? "Auto" : "Man.", &Font_7x10, 1);

    SSD1306_GotoXY(53, TEXT_Y);
    SSD1306_Puts(motorVoltageDetected ? "Lig." : "Des.", &Font_7x10, 1);

    sprintf(displayText, "%i%%", (int) (relativeLuminosity * 100));
    SSD1306_GotoXY(96, TEXT_Y);
    SSD1306_Puts(displayText, &Font_7x10, 1);

    if (motorVoltageDetected)
      propellerFrame = (propellerFrame + 1) % 4;

    if (state == MOTOR_SWITCHING_ON_STATE || state == MOTOR_SWITCHING_OFF_STATE
        || state == ERROR_STATE) {
      starterFrame = !starterFrame;
      SSD1306_PutIcon(12, ICONS_Y, STARTER_ICON, starterFrame);
    } else
      SSD1306_PutIcon(12, ICONS_Y, STARTER_ICON, 1);

    switch (propellerFrame) {
    case 0:
      SSD1306_PutIcon(64, 60, PROPELLER_2_ICON, 0);
      SSD1306_PutIcon(64, 60, PROPELLER_1_ICON, 1);
      break;

    case 1:
      SSD1306_PutIcon(64, 60, PROPELLER_1_ICON, 0);
      SSD1306_PutIcon(64, 60, PROPELLER_2_ICON, 1);
      break;

    case 2:
      SSD1306_PutIcon(64, 60, PROPELLER_2_ICON, 0);
      break;

    case 3:
      SSD1306_PutIcon(64, 60, PROPELLER_2_ICON, 1);
      break;

    }

    SSD1306_UpdateScreen();
    osDelay(150);
    /* case SETTINGS_VIEW:
     {

     }
     }
     } */
    /* USER CODE END StartDisplayTask */
  }
}

/* USER CODE BEGIN Header_StartWiFiTask */
/**
 * @brief Function implementing the WiFiTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartWiFiTask */
void StartWiFiTask(void *argument) {
  /* USER CODE BEGIN StartWiFiTask */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END StartWiFiTask */
}

/* USER CODE BEGIN Header_StartButtonsTask */
/**
 * @brief Function implementing the ButtonsTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartButtonsTask */
void StartButtonsTask(void *argument) {
  /* USER CODE BEGIN StartButtonsTask */
  /* Infinite loop */
  for (;;) {
    // osThreadFlagsWait(btnPressedFlag, osFlagsWaitAny, osWaitForever);

    if (btnPressedFlag) {
      // switch (currentView) {
      // case DASHBOARD_VIEW: {
      if (btnPressedName == CIRCLE_BTN)
        automaticModeEnabled = !automaticModeEnabled;
      else {
        luminosityTrigger = ldrResistence;
        EE_WriteVariable(VirtAddVarTab[0], luminosityTrigger);
        osDelay(200);

        /* currentView = SETTINGS_VIEW;
         }
         break;
         }
         }
         case SETTINGS_VIEW: {
         if (btnPressedName == CIRCLE_BTN) {
         switch (arrowSettingsPosition) {
         case LUMINOSITY_TRIGGER_POSITION: {
         luminosityTrigger = ldrResistence;
         EE_WriteVariable(VirtAddVarTab[0], luminosityTrigger);
         osDelay(200);
         }
         }
         } else { // CROSS_BTN
         if (arrowDisplayPosition + 1 < DISPLAY_ARROW_POSITIONS)
         arrowDisplayPosition = (arrowDisplayPosition + 1)
         % DISPLAY_ARROW_POSITIONS;

         if (arrowSettingsPosition + 1 < NUMBER_OF_SETTINGS)
         arrowSettingsPosition = (arrowSettingsPosition + 1)
         % NUMBER_OF_SETTINGS;
         else {
         arrowDisplayPosition = 0;
         arrowSettingsPosition = 0;
         currentView = DASHBOARD_VIEW;
         }
         }

         break;
         }
         }
         }

         // btnPressedFlag = 0;

         if (HAL_GPIO_ReadPin(GPIOB, BUTTON_D3_Pin)) {
         luminosityTrigger = ldrResistence;
         EE_WriteVariable(VirtAddVarTab[0], luminosityTrigger);

         osDelay(200);
         while (HAL_GPIO_ReadPin(GPIOB, BUTTON_D3_Pin))
         osDelay(10);

         } else if (HAL_GPIO_ReadPin(GPIOB, BUTTON_D2_Pin)) {
         automaticModeEnabled = !automaticModeEnabled;

         osDelay(200);
         while (HAL_GPIO_ReadPin(GPIOB, BUTTON_D2_Pin))
         osDelay(10);
         } */
      }

      osDelay(100);
      btnPressedFlag = 0;
    } else
      osDelay(100);
  }
  /* USER CODE END StartButtonsTask */

}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM4 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
