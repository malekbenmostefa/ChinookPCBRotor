/**
 ******************************************************************************
 * File Name          : rpm.c
 * Description        :
 * Created            : Mar 8, 2024
 * Author             : Malek Benmostefa et Cedric Verdi
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "rpm.h"
/* Defines -------------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
uint32_t time_interval = 0; // Variable déclarée dans le main.h
uint32_t rpm_value = 0; // Variable déclarée dans le main.h
uint8_t rpm_pulse_count = 0;
/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Public functions  ---------------------------------------------------------*/

/**
 * @brief  Fonction qui est appelée en callback lorsqu'un pulse est détecté du capteur rpm
 * @param  None
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	time_interval = TIM2->CNT;
	if(rpm_pulse_count == 0)
	{
		rpm_pulse_count = 1;
	}
	else if(rpm_pulse_count == 1)
	{
		TIM2->CNT = 0;
		rpm_pulse_count = 0;
		rpm_value = time_interval*60/1000000; // Conversion rotation par us en rpm
		// Si overflow interrupt de tim2, alors rpm=0 (on reset rpm_pulse_count=0)
	}
	else
	{
		// Erreur (valeur de rpm_pulse_count erroné), on reset les valeurs
		rpm_pulse_count = 0;
		time_interval = 0;
		rpm_value = 0;
	}

}

/**
 * @brief  Fonction qui est appelée dans l'interrupt timer2 causé par un overflow
 * 			Lorsque cette fonction est appelée, cela veut dire qu'on ne recoit pas
 * 			de pulse du capteur rpm et que le rpm est donc à 0 (l'hélice ne tourne pas)
 * @param  TIM_HandleTypeDef *htim : typedef relié au timer2
 * @retval None
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	rpm_pulse_count = 0;
	time_interval = 0;
	rpm_value = 0;
	TIM2->CNT = 0;
}

/* Private functions ---------------------------------------------------------*/
