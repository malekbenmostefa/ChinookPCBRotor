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
extern uint32_t adc_value; // Variable déclarée dans le main.h
extern uint8_t new_adc_value = 0; // 1 s'il y a une nouvelle valeur lue, sinon 0
/* Function prototypes ------------------------------------------------------ */

#endif
