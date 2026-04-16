/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define ACT_LIFTER_RPWM_Pin GPIO_PIN_0
#define ACT_LIFTER_RPWM_GPIO_Port GPIOC
#define ACT_LIFTER_LPWM_Pin GPIO_PIN_1
#define ACT_LIFTER_LPWM_GPIO_Port GPIOC
#define ACT_PUSHER_RPWM_Pin GPIO_PIN_2
#define ACT_PUSHER_RPWM_GPIO_Port GPIOC
#define ACT_PUSHER_LPWM_Pin GPIO_PIN_3
#define ACT_PUSHER_LPWM_GPIO_Port GPIOC
#define ACT_LIFTER_ENCA_Pin GPIO_PIN_0
#define ACT_LIFTER_ENCA_GPIO_Port GPIOA
#define ACT_LIFTER_ENCB_Pin GPIO_PIN_1
#define ACT_LIFTER_ENCB_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TX_FROM_ESP_Pin GPIO_PIN_6
#define TX_FROM_ESP_GPIO_Port GPIOC
#define RX_TO_ESP_Pin GPIO_PIN_7
#define RX_TO_ESP_GPIO_Port GPIOC
#define L_MOTOR_RPWM_Pin GPIO_PIN_8
#define L_MOTOR_RPWM_GPIO_Port GPIOA
#define L_MOTOR_LPWM_Pin GPIO_PIN_9
#define L_MOTOR_LPWM_GPIO_Port GPIOA
#define R_MOTOR_RPWM_Pin GPIO_PIN_10
#define R_MOTOR_RPWM_GPIO_Port GPIOA
#define R_MOTOR_LPWM_Pin GPIO_PIN_11
#define R_MOTOR_LPWM_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define MOTOR_ENC_L_A_Pin GPIO_PIN_4
#define MOTOR_ENC_L_A_GPIO_Port GPIOB
#define MOTOR_ENC_L_B_Pin GPIO_PIN_5
#define MOTOR_ENC_L_B_GPIO_Port GPIOB
#define MOTOR_ENC_R_A_Pin GPIO_PIN_6
#define MOTOR_ENC_R_A_GPIO_Port GPIOB
#define MOTOR_ENC_R_B_Pin GPIO_PIN_7
#define MOTOR_ENC_R_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
