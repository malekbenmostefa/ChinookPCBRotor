/**
 ******************************************************************************
 * File Name          : rpm.h
 * Description        :
 * Created            : Mar 8, 2024
 * Author             : Malek Benmostefa et Cedric Verdi
 ******************************************************************************
 */

/* Prevent recursive inclusion -----------------------------------------------*/
#ifndef __RPM_H__
#define __RPM_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/* Defines -------------------------------------------------------------------*/

/* Type definitions ----------------------------------------------------------*/
extern uint32_t time_interval; // Variable déclarée dans le main.h
extern uint32_t rpm_value; // Variable déclarée dans le main.h
extern uint8_t rpm_pulse_count;
/* Function prototypes ------------------------------------------------------ */
void EXTI_Init(void);
void* EXTI_CallBack(void);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);

#endif
