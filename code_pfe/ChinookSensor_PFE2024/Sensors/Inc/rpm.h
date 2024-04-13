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

/* Function prototypes ------------------------------------------------------ */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);

#endif
