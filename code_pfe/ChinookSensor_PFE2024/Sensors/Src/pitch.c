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

uint8_t encodeur_Init(UART_HandleTypeDef *uart, uint8_t addr){

}

uint8_t SB_cmd(uint8_t addr,uint8_t cmd){
	uint8_t cmd_to_send = 0;

	cmd_to_send = addr;
	cmd_to_send = (cmd_to_send << 4) + cmd;

	return cmd_to_send;
}

uint8_t transmit_SB_cmd(UART_HandleTypeDef *huart, uint8_t *cmd){

	HAL_UART_Transmit_IT(huart, cmd, sizeof(*cmd));

	return 1;

}
/* Private functions ---------------------------------------------------------*/
