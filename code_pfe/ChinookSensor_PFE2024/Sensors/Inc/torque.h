/**
 ******************************************************************************
 * File Name          : torque.h
 * Description        :
 * Created            : Mar 8, 2024
 * Author             : Malek Benmostefa et Cedric Verdi
 ******************************************************************************
 */

/* Prevent recursive inclusion -----------------------------------------------*/
#ifndef __TORQUE_H__
#define __TORQUE_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/* Defines -------------------------------------------------------------------*/

/* Type definitions ----------------------------------------------------------*/

/* Function prototypes ------------------------------------------------------ */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

#endif
