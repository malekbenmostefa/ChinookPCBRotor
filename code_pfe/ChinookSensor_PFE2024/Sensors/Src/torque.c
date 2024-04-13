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
uint32_t adc_value = 0; // Variable déclarée dans le main.h
uint8_t new_adc_value = 0; // 1 s'il y a une nouvelle valeur lue, sinon 0
/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Public functions  ---------------------------------------------------------*/

/**
 * @brief  Fonction qui sera appelée lorsqu'une conversion sera complétée sur un ADC
 * @param  ADC_HandleTypeDef *hadc
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	// Instruction pour tester la fréquence de conversion (mesure à l'oscilloscope)
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);

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
