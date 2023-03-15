/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HALL_TIM_PSC 167
#define HALL_TIM_Counter 4999
#define ADC_TIM_PSC 167
#define ADC_TIM_Counter 499
#define PWM_TIM_PSC 1
#define PWM_TIM_Counter 5659
#define MOTOR_SD_Pin GPIO_PIN_6
#define MOTOR_SD_GPIO_Port GPIOE
#define KEY3_Pin GPIO_PIN_13
#define KEY3_GPIO_Port GPIOC
#define CURR_U_ADC_Pin GPIO_PIN_6
#define CURR_U_ADC_GPIO_Port GPIOF
#define CURR_V_ADC_Pin GPIO_PIN_7
#define CURR_V_ADC_GPIO_Port GPIOF
#define CURR_W_ADC_Pin GPIO_PIN_8
#define CURR_W_ADC_GPIO_Port GPIOF
#define VBUS_Pin GPIO_PIN_9
#define VBUS_GPIO_Port GPIOF
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOA
#define HALL_INPUT1_Pin GPIO_PIN_10
#define HALL_INPUT1_GPIO_Port GPIOH
#define HALL_INPUT2_Pin GPIO_PIN_11
#define HALL_INPUT2_GPIO_Port GPIOH
#define HALL_INPUT3_Pin GPIO_PIN_12
#define HALL_INPUT3_GPIO_Port GPIOH
#define KEY2_Pin GPIO_PIN_2
#define KEY2_GPIO_Port GPIOG
#define KEY4_Pin GPIO_PIN_3
#define KEY4_GPIO_Port GPIOG
#define KEY5_Pin GPIO_PIN_4
#define KEY5_GPIO_Port GPIOG
#define MOTOR_OCNPWM1_Pin GPIO_PIN_13
#define MOTOR_OCNPWM1_GPIO_Port GPIOH
#define MOTOR_OCNPWM2_Pin GPIO_PIN_14
#define MOTOR_OCNPWM2_GPIO_Port GPIOH
#define MOTOR_OCNPWM3_Pin GPIO_PIN_15
#define MOTOR_OCNPWM3_GPIO_Port GPIOH
#define MOTOR_OCPWM1_Pin GPIO_PIN_5
#define MOTOR_OCPWM1_GPIO_Port GPIOI
#define MOTOR_OCPWM2_Pin GPIO_PIN_6
#define MOTOR_OCPWM2_GPIO_Port GPIOI
#define MOTOR_OCPWM3_Pin GPIO_PIN_7
#define MOTOR_OCPWM3_GPIO_Port GPIOI
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
