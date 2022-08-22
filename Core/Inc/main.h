/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
typedef struct SensorData_{
	float wheel_rpm;
	float rotor_rpm;
	float torque;
	float loadcell;
	float wind_direction;
	float wind_speed;
	uint32_t pitch_encoder;
	uint32_t mast_encoder;
	uint8_t limit1, limit2;

	// Motor modes
	uint32_t feedback_pitch_mode;
	uint32_t feedback_mast_mode;

	uint32_t feedback_pitch_rops;

	// Motor faults
	uint32_t pitch_fault_stall;
	uint32_t mast_fault_stall;

	// Processed sensor data
	float pitch_angle;
	float mast_angle;
	float vehicle_speed;
}SensorData;

extern SensorData sensor_data;

extern uint32_t wheel_rpm_counter;
extern uint32_t rotor_rpm_counter;
extern uint32_t rpm_counter_time;

// Weather station
extern uint8_t rx_buff[64];
extern uint8_t index_buff;
extern uint8_t ws_receive_flag;

extern uint8_t timer_50ms_flag;
extern uint8_t timer_100ms_flag;
extern uint8_t timer_500ms_flag;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Pitch_Clock_Pin GPIO_PIN_2
#define Pitch_Clock_GPIO_Port GPIOE
#define LIMIT1_Pin GPIO_PIN_3
#define LIMIT1_GPIO_Port GPIOE
#define LIMIT1_EXTI_IRQn EXTI3_IRQn
#define LIMIT2_Pin GPIO_PIN_4
#define LIMIT2_GPIO_Port GPIOE
#define LIMIT2_EXTI_IRQn EXTI4_IRQn
#define Pitch_Data_Pin GPIO_PIN_5
#define Pitch_Data_GPIO_Port GPIOE
#define Mast_Clock_Pin GPIO_PIN_5
#define Mast_Clock_GPIO_Port GPIOA
#define Mast_Data_Pin GPIO_PIN_6
#define Mast_Data_GPIO_Port GPIOA
#define USB_RESET_Pin GPIO_PIN_4
#define USB_RESET_GPIO_Port GPIOC
#define USB_TX_EN_Pin GPIO_PIN_5
#define USB_TX_EN_GPIO_Port GPIOC
#define TorqueADC_Pin GPIO_PIN_0
#define TorqueADC_GPIO_Port GPIOB
#define LoadcellADC_Pin GPIO_PIN_1
#define LoadcellADC_GPIO_Port GPIOB
#define LORA_EN_Pin GPIO_PIN_10
#define LORA_EN_GPIO_Port GPIOE
#define LORA_RESET_Pin GPIO_PIN_11
#define LORA_RESET_GPIO_Port GPIOE
#define LORA_INT_Pin GPIO_PIN_12
#define LORA_INT_GPIO_Port GPIOE
#define LORA_INT_EXTI_IRQn EXTI15_10_IRQn
#define LORA_CS_Pin GPIO_PIN_14
#define LORA_CS_GPIO_Port GPIOB
#define LED_WARNING_Pin GPIO_PIN_8
#define LED_WARNING_GPIO_Port GPIOD
#define LED_ERROR_Pin GPIO_PIN_9
#define LED_ERROR_GPIO_Port GPIOD
#define LED_CANA_Pin GPIO_PIN_10
#define LED_CANA_GPIO_Port GPIOD
#define LED_CANB_Pin GPIO_PIN_11
#define LED_CANB_GPIO_Port GPIOD
#define PB2_Pin GPIO_PIN_14
#define PB2_GPIO_Port GPIOD
#define PB2_EXTI_IRQn EXTI15_10_IRQn
#define PB1_Pin GPIO_PIN_15
#define PB1_GPIO_Port GPIOD
#define PB1_EXTI_IRQn EXTI15_10_IRQn
#define SD_DETECT_Pin GPIO_PIN_12
#define SD_DETECT_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_4
#define LED4_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOD
#define Rotor_RPM_Pin GPIO_PIN_0
#define Rotor_RPM_GPIO_Port GPIOE
#define Rotor_RPM_EXTI_IRQn EXTI0_IRQn
#define Wheel_RPM_Pin GPIO_PIN_1
#define Wheel_RPM_GPIO_Port GPIOE
#define Wheel_RPM_EXTI_IRQn EXTI1_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
