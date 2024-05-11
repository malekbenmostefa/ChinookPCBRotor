/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#include "pitch.h"
#include "rpm.h"
#include "torque.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint32_t time_interval;
extern uint32_t rpm_value;
extern uint8_t rpm_pulse_count;
extern uint32_t adc_value;
extern uint8_t new_adc_value; // 1 s'il y a une nouvelle valeur lue, sinon 0
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
#define RPM_EXTI_Pin GPIO_PIN_0
#define RPM_EXTI_GPIO_Port GPIOC
#define RPM_EXTI_EXTI_IRQn EXTI0_IRQn
#define Torque_ADC_Pin GPIO_PIN_0
#define Torque_ADC_GPIO_Port GPIOA
#define Torque_GPIO_IN_Pin GPIO_PIN_1
#define Torque_GPIO_IN_GPIO_Port GPIOA
#define Etat_Torque_Pin GPIO_PIN_4
#define Etat_Torque_GPIO_Port GPIOA
#define Etat_RPM_Pin GPIO_PIN_5
#define Etat_RPM_GPIO_Port GPIOA
#define Etat_MCU_Pin GPIO_PIN_6
#define Etat_MCU_GPIO_Port GPIOA
#define Pitch_UART_TX_Pin GPIO_PIN_12
#define Pitch_UART_TX_GPIO_Port GPIOC
#define Pitch_UART_RX_Pin GPIO_PIN_2
#define Pitch_UART_RX_GPIO_Port GPIOD
#define Pitch_GPIO_OUT_Pin GPIO_PIN_3
#define Pitch_GPIO_OUT_GPIO_Port GPIOB
#define Pitch_busy_line_Pin GPIO_PIN_4
#define Pitch_busy_line_GPIO_Port GPIOB
#define OldPitch_GPIO_OUT_Pin GPIO_PIN_5
#define OldPitch_GPIO_OUT_GPIO_Port GPIOB
#define OldPitch_GPIO_IN_Pin GPIO_PIN_6
#define OldPitch_GPIO_IN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
