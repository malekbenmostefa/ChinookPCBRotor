/**
 ******************************************************************************
 * File Name          : torque.c
 * Description        :
 * Created            : Mar 8, 2024
 * Author             : Malek Benmostefa et Cedric Verdi
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "torque.h"
/* Defines -------------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Public functions  ---------------------------------------------------------*/

/**
 * @brief  Fonction qui sera appelée lorsqu'une conversion sera complétée
 * @param  None
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	adc_value = HAL_ADC_GetValue(hadc);
	new_adc_value = 1;
}

/**
 * @brief  Fonction qui fait autre chose
 * @param  None
 * @retval None
 */
void fonction2(void)
{
    
}

/* Private functions ---------------------------------------------------------*/
