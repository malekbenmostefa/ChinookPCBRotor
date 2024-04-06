/**
 ******************************************************************************
 * File Name          : pitch.c
 * Description        :
 * Created            : Mar 8, 2024
 * Author             : Malek Benmostefa et Cedric Verdi
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "pitch.h"
/* Defines -------------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Public functions  ---------------------------------------------------------*/

uint8_t SB_cmd(uint8_t addr,uint8_t cmd){
	uint8_t request = 0;

	request = addr;
	request = (request << 4) + cmd;

	return request;
}

uint8_t pitch_send_SB_cmd(UART_HandleTypeDef *huart, uint8_t *request){

	HAL_UART_Transmit_IT(huart, (uint8_t*)request, sizeof(*request));

	uint8_t cmd = (*request >> 4);

	switch(cmd){
		case REQUEST_POSITION :
			HAL_UART_Receive_IT(huart,rx_buffer,POSITION_LENGTH);
			break;
		case REQUEST_POSITION_STATUS :
			HAL_UART_Receive_IT(huart,rx_buffer,POSITION_LENGTH+STATUT_LENGTH);
			break;
		case REQUEST_POSITION_TIME_STATUS :
			HAL_UART_Receive_IT(huart,rx_buffer,POSITION_LENGTH+TIME_LENGTH+STATUT_LENGTH);
			break;
	}
	return 1;

}

uint8_t encodeur_Init(UART_HandleTypeDef *uart, uint8_t addr){

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	__NOP();
}
/* Private functions ---------------------------------------------------------*/
