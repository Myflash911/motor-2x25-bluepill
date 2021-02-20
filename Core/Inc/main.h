/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define LED_ON() 				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)
#define LED_OFF()				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)
#define BLINK()					{LED_ON(); HAL_Delay(100); LED_OFF(); HAL_Delay(300);}
#define LONG_BLINK()			{LED_ON(); HAL_Delay(500); LED_OFF(); HAL_Delay(300);}

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define ENC2_A_Pin GPIO_PIN_0
#define ENC2_A_GPIO_Port GPIOA
#define ENC2_B_Pin GPIO_PIN_1
#define ENC2_B_GPIO_Port GPIOA
#define LMT1_BWD_Pin GPIO_PIN_12
#define LMT1_BWD_GPIO_Port GPIOB
#define LMT1_BWD_EXTI_IRQn EXTI15_10_IRQn
#define LMT1_FWD_Pin GPIO_PIN_13
#define LMT1_FWD_GPIO_Port GPIOB
#define LMT1_FWD_EXTI_IRQn EXTI15_10_IRQn
#define LMT2_BWD_Pin GPIO_PIN_14
#define LMT2_BWD_GPIO_Port GPIOB
#define LMT2_BWD_EXTI_IRQn EXTI15_10_IRQn
#define LMT2_FWD_Pin GPIO_PIN_15
#define LMT2_FWD_GPIO_Port GPIOB
#define LMT2_FWD_EXTI_IRQn EXTI15_10_IRQn
#define ENC1_A_Pin GPIO_PIN_8
#define ENC1_A_GPIO_Port GPIOA
#define ENC1_B_Pin GPIO_PIN_9
#define ENC1_B_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define VERSION "v0.8 2021"

#define ENC_LIMITSWITCH_POS	1000;
#define ENC1_Position		TIM1->CNT
#define ENC2_Position		TIM2->CNT

#define isLimitSwitchReleased(Pin)	 (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB, (Pin)))
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
