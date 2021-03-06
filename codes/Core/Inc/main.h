/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PHOTORESISTOR_A1_Pin GPIO_PIN_0
#define PHOTORESISTOR_A1_GPIO_Port GPIOA
#define MOTOR_START_Pin GPIO_PIN_1
#define MOTOR_START_GPIO_Port GPIOA
#define MOTOR_STOP_Pin GPIO_PIN_2
#define MOTOR_STOP_GPIO_Port GPIOA
#define LED_D1_Pin GPIO_PIN_3
#define LED_D1_GPIO_Port GPIOA
#define LED_D2_Pin GPIO_PIN_4
#define LED_D2_GPIO_Port GPIOA
#define LED_D3_Pin GPIO_PIN_5
#define LED_D3_GPIO_Port GPIOA
#define BUTTON_D1_Pin GPIO_PIN_6
#define BUTTON_D1_GPIO_Port GPIOA
#define CROSS_BTN_Pin GPIO_PIN_0
#define CROSS_BTN_GPIO_Port GPIOB
#define CROSS_BTN_EXTI_IRQn EXTI0_IRQn
#define BUZZER_D1_Pin GPIO_PIN_1
#define BUZZER_D1_GPIO_Port GPIOB
#define MOTOR_STATUS_Pin GPIO_PIN_10
#define MOTOR_STATUS_GPIO_Port GPIOB
#define CIRCLE_BTN_Pin GPIO_PIN_11
#define CIRCLE_BTN_GPIO_Port GPIOB
#define CIRCLE_BTN_EXTI_IRQn EXTI15_10_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
