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

uint8_t turn_mode = 0;

uint8_t checksum[1];

uint8_t position_rx_buff[2];
uint8_t position_time_rx_buff[4];
uint8_t position_time_status_rx_buff[5];

uint8_t serial_number_rx_buff[5];
uint8_t address_rx_buff[2];
uint8_t factory_info_rx_buff[15];
uint8_t resolution_rx_buff[3];
uint8_t mode_rx_buff[3];

uint8_t busy_line_state = 0;

uint8_t data_tx[20];
/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Public functions  ---------------------------------------------------------*/

uint8_t calculate_checksum(uint8_t *data_tx, uint8_t taille_buffer_tx, uint8_t *data_rx, uint8_t taille_buffer_rx){
	uint8_t result_checksum = 0;

	for(uint16_t i = 0; i < taille_buffer_tx; i++){
		result_checksum ^= data_tx[i];
	}
	for(uint16_t i = 0; i < taille_buffer_rx - 1; i++){
		result_checksum ^= data_rx[i];
	}

	return result_checksum;
}

uint8_t SB_cmd(uint8_t addr,uint8_t cmd){
	uint8_t request = 0;

	request = addr;
	request = (request << 4) + cmd;

	return request;
}

uint8_t pitch_send_SB_cmd(UART_HandleTypeDef *huart, uint8_t request){

	HAL_UART_Transmit_IT(huart, (uint8_t*)&request, sizeof(request));

	uint8_t cmd = request & 0x0F; // Bit mask sur 4 bit LSB

	switch(cmd){
		case REQUEST_POSITION :
			HAL_UART_Receive(huart,position_rx_buff,2,RECEIVE_TIMEOUT);
			break;
		case REQUEST_POSITION_STATUS :
			HAL_UART_Receive(huart,position_time_rx_buff,POSITION_LENGTH+STATUT_LENGTH,RECEIVE_TIMEOUT);
			break;
		case REQUEST_POSITION_TIME_STATUS :
			HAL_UART_Receive(huart,position_time_status_rx_buff,POSITION_LENGTH+TIME_LENGTH+STATUT_LENGTH,RECEIVE_TIMEOUT);
			break;
	}
	return 1;

}

uint8_t pitch_set_origin(UART_HandleTypeDef *huart, uint8_t addr){
	uint8_t test_checksum = 0;
	uint8_t resultat_checksum = 0;
	data_tx[0] = SB_cmd(addr, REQUEST_MB_CMD);
	data_tx[1] = MODE_SET_ORIGIN;

	HAL_UART_Transmit_IT(huart, (uint8_t*)data_tx, 2);

	HAL_UART_Receive(huart, (uint8_t*)checksum, sizeof(checksum),RECEIVE_TIMEOUT);
	resultat_checksum = calculate_checksum(data_tx, 6, checksum, sizeof(checksum));
	if(resultat_checksum == checksum[sizeof(checksum)-1]){
		test_checksum = 1;
	}
	else{
		test_checksum = 0;
	}

	return test_checksum;
}

uint8_t pitch_set_absolute_position(UART_HandleTypeDef *huart, uint8_t addr, uint8_t turn_mode,  uint8_t *position){
	uint8_t test_checksum = 0;
	uint8_t resultat_checksum = 0;
	if(turn_mode == SINGLE_TURN_MODE){
		data_tx[0] = SB_cmd(addr, REQUEST_MB_CMD);
		data_tx[1] = MODE_SET_ABS_POSITION;
		data_tx[2] = position[0];
		data_tx[3] = position[1];

		HAL_UART_Transmit_IT(huart, (uint8_t*)data_tx, 4);
	}
	else{
		data_tx[0] = SB_cmd(addr, REQUEST_MB_CMD);
		data_tx[1] = MODE_SET_ABS_POSITION;
		data_tx[2] = position[0];
		data_tx[3] = position[1];
		data_tx[4] = position[2];
		data_tx[5] = position[3];

		HAL_UART_Transmit_IT(huart, (uint8_t*)data_tx, 6);
	}

	HAL_UART_Receive(huart, (uint8_t*)checksum, sizeof(checksum),RECEIVE_TIMEOUT);
	resultat_checksum = calculate_checksum(data_tx, 6, checksum, sizeof(checksum));
	if(resultat_checksum == checksum[sizeof(checksum)-1]){
		test_checksum = 1;
	}
	else{
		test_checksum = 0;
	}

	return test_checksum;
}

uint8_t pitch_read_serial_number(UART_HandleTypeDef *huart, uint8_t addr){
	uint8_t test_checksum = 0;
	uint8_t resultat_checksum = 0;
	data_tx[0] = SB_cmd(addr, REQUEST_MB_CMD);
	data_tx[1] = MODE_READ_SN;

	HAL_UART_Transmit_IT(huart, (uint8_t*)data_tx, 2);

	HAL_UART_Receive(huart,(uint8_t*)serial_number_rx_buff, sizeof(serial_number_rx_buff),RECEIVE_TIMEOUT);
	resultat_checksum = calculate_checksum(data_tx, 6, serial_number_rx_buff, sizeof(serial_number_rx_buff));
	if(resultat_checksum == serial_number_rx_buff[sizeof(serial_number_rx_buff)-1]){
		test_checksum = 1;
	}
	else{
		test_checksum = 0;
	}

	return test_checksum;
}

uint8_t pitch_check_serial_number(UART_HandleTypeDef *huart, uint8_t addr, uint8_t *serial_number, uint8_t *mask){
	data_tx[0] = SB_cmd(addr, REQUEST_MB_CMD);
	data_tx[1] = MODE_CHECK_SN;
	data_tx[2] = serial_number[0];
	data_tx[3] = serial_number[1];
	data_tx[4] = serial_number[2];
	data_tx[5] = serial_number[3];
	data_tx[6] = mask[0];
	data_tx[7] = mask[1];
	data_tx[8] = mask[2];
	data_tx[9] = mask[3];

	HAL_UART_Transmit_IT(huart, (uint8_t*)data_tx, 10);

	if(HAL_GPIO_ReadPin(pitch_busy_line_GPIO_Port, pitch_busy_line_Pin)){
		busy_line_state = 1;
	}
	else{
		busy_line_state = 0;
	}

	return busy_line_state;
}

uint8_t pitch_fail_serial_number(UART_HandleTypeDef *huart, uint8_t addr, uint8_t *serial_number, uint8_t *mask){
	data_tx[0] = SB_cmd(addr, REQUEST_MB_CMD);
	data_tx[1] = MODE_FAIL_SN;
	data_tx[2] = serial_number[0];
	data_tx[3] = serial_number[1];
	data_tx[4] = serial_number[2];
	data_tx[5] = serial_number[3];
	data_tx[6] = mask[0];
	data_tx[7] = mask[1];
	data_tx[8] = mask[2];
	data_tx[9] = mask[3];

	HAL_UART_Transmit_IT(huart, (uint8_t*)data_tx, 10);

	if(HAL_GPIO_ReadPin(pitch_busy_line_GPIO_Port, pitch_busy_line_Pin)){
		busy_line_state = 0;
	}
	else{
		busy_line_state = 1;
	}

	return busy_line_state;
}

uint8_t pitch_get_address(UART_HandleTypeDef *huart, uint8_t addr, uint8_t *serial_number){
	uint8_t test_checksum = 0;
	uint8_t resultat_checksum = 0;
	data_tx[0] = SB_cmd(addr, REQUEST_MB_CMD);
	data_tx[1] = MODE_GET_ADDRESS;
	data_tx[2] = serial_number[0];
	data_tx[3] = serial_number[1];
	data_tx[4] = serial_number[2];
	data_tx[5] = serial_number[3];

	HAL_UART_Transmit_IT(huart, (uint8_t*)data_tx, 6);

	HAL_UART_Receive(huart,(uint8_t*)address_rx_buff, sizeof(address_rx_buff),RECEIVE_TIMEOUT);
	resultat_checksum = calculate_checksum(data_tx, 6, address_rx_buff, sizeof(address_rx_buff));
	if(resultat_checksum == address_rx_buff[sizeof(address_rx_buff)-1]){
		test_checksum = 1;
	}
	else{
		test_checksum = 0;
	}

	return test_checksum;
}

uint8_t pitch_assign_address(UART_HandleTypeDef *huart, uint8_t addr, uint8_t *serial_number, uint8_t new_addr){
	uint8_t test_checksum = 0;
	uint8_t resultat_checksum = 0;
	data_tx[0] = SB_cmd(addr, REQUEST_MB_CMD);
	data_tx[1] = MODE_SET_ADDRESS;
	data_tx[2] = serial_number[0];
	data_tx[3] = serial_number[1];
	data_tx[4] = serial_number[2];
	data_tx[5] = serial_number[3];
	data_tx[6] = new_addr;

	HAL_UART_Transmit_IT(huart, (uint8_t*)data_tx, 7);

	HAL_UART_Receive(huart, (uint8_t*)checksum, sizeof(checksum),RECEIVE_TIMEOUT);
	resultat_checksum = calculate_checksum(data_tx, 7, checksum, sizeof(checksum));
	if(resultat_checksum == checksum[sizeof(checksum)-1]){
		test_checksum = 1;
	}
	else{
		test_checksum = 0;
	}

	return test_checksum;

	return 1;
}

uint8_t pitch_read_factory_info(UART_HandleTypeDef *huart, uint8_t addr){
	uint8_t test_checksum = 0;
	uint8_t resultat_checksum = 0;
	data_tx[0] = SB_cmd(addr, REQUEST_MB_CMD);
	data_tx[1] = MODE_READ_FI;

	HAL_UART_Transmit_IT(huart, (uint8_t*)data_tx, 2);

	HAL_UART_Receive(huart, (uint8_t*)factory_info_rx_buff, sizeof(factory_info_rx_buff),RECEIVE_TIMEOUT);
	resultat_checksum = calculate_checksum(data_tx, 2, factory_info_rx_buff, sizeof(factory_info_rx_buff));
	if(resultat_checksum == factory_info_rx_buff[sizeof(factory_info_rx_buff)-1]){
		test_checksum = 1;
	}
	else{
		test_checksum = 0;
	}

	return test_checksum;
}

uint8_t pitch_read_resolution(UART_HandleTypeDef *huart, uint8_t addr){
	uint8_t test_checksum = 0;
	uint8_t resultat_checksum = 0;
	data_tx[0] = SB_cmd(addr, REQUEST_MB_CMD);
	data_tx[1] = MODE_READ_RESOLUTION;

	HAL_UART_Transmit_IT(huart, (uint8_t*)data_tx, 2);

	HAL_UART_Receive(huart, (uint8_t*)resolution_rx_buff, sizeof(resolution_rx_buff),RECEIVE_TIMEOUT);
	resultat_checksum = calculate_checksum(data_tx, 2, resolution_rx_buff, sizeof(resolution_rx_buff));
	if(resultat_checksum == resolution_rx_buff[sizeof(resolution_rx_buff)-1]){
		test_checksum = 1;
	}
	else{
		test_checksum = 0;
	}

	return test_checksum;
}

uint8_t pitch_change_resolution(UART_HandleTypeDef *huart, uint8_t addr, uint8_t *resolution){
	uint8_t test_checksum = 0;
	uint8_t resultat_checksum = 0;
	data_tx[0] = SB_cmd(addr, REQUEST_MB_CMD);
	data_tx[1] = MODE_SET_RESOLUTION;
	data_tx[2] = resolution[0];
	data_tx[3] = resolution[1];

	HAL_UART_Transmit_IT(huart, (uint8_t*)data_tx, 4);

	HAL_UART_Receive(huart, (uint8_t*)checksum, sizeof(checksum),RECEIVE_TIMEOUT);
	resultat_checksum = calculate_checksum(data_tx, 4, checksum, sizeof(checksum));
	if(resultat_checksum == checksum[sizeof(checksum)-1]){
		test_checksum = 1;
	}
	else{
		test_checksum = 0;
	}

	return test_checksum;
}

uint8_t pitch_read_mode(UART_HandleTypeDef *huart, uint8_t addr){
	uint8_t test_checksum = 0;
	uint8_t resultat_checksum = 0;
	data_tx[0] = SB_cmd(addr, REQUEST_MB_CMD);
	data_tx[1] = MODE_READ_MODE;

	HAL_UART_Transmit_IT(huart, (uint8_t*)data_tx, 2);

	HAL_UART_Receive(huart, (uint8_t*)checksum, sizeof(checksum),RECEIVE_TIMEOUT);
	resultat_checksum = calculate_checksum(data_tx, 2, checksum, sizeof(checksum));
	if(resultat_checksum == checksum[sizeof(checksum)-1]){
		test_checksum = 1;
	}
	else{
		test_checksum = 0;
	}

	return test_checksum;
}

uint8_t pitch_change_mode(UART_HandleTypeDef *huart, uint8_t addr, uint8_t mode){
	uint8_t test_checksum = 0;
	uint8_t resultat_checksum = 0;
	data_tx[0] = SB_cmd(addr, REQUEST_MB_CMD);
	data_tx[1] = (uint8_t)MODE_SET_MODE;
	data_tx[2] = mode;

	HAL_UART_Transmit_IT(huart, (uint8_t*)data_tx, 3);

	HAL_UART_Receive(huart, (uint8_t*)checksum, sizeof(checksum),RECEIVE_TIMEOUT);
	resultat_checksum = calculate_checksum(data_tx, 3, checksum, sizeof(checksum));
	if(resultat_checksum == checksum[sizeof(checksum)-1]){
		test_checksum = 1;
	}
	else{
		test_checksum = 0;
	}

	return test_checksum;
}

uint8_t pitch_change_power_up_mode(UART_HandleTypeDef *huart, uint8_t addr, uint8_t mode){
	uint8_t test_checksum = 0;
	uint8_t resultat_checksum = 0;
	data_tx[0] = SB_cmd(addr, REQUEST_MB_CMD);
	data_tx[1] = MODE_CHANGE_POWER_UP;
	data_tx[2] = mode;

	HAL_UART_Transmit_IT(huart, (uint8_t*)data_tx, 3);

	HAL_UART_Receive(huart, (uint8_t*)checksum, sizeof(checksum),RECEIVE_TIMEOUT);
	resultat_checksum = calculate_checksum(data_tx, 3, checksum, sizeof(checksum));
	if(resultat_checksum == checksum[sizeof(checksum)-1]){
		test_checksum = 1;
	}
	else{
		test_checksum = 0;
	}

	return test_checksum;
}

uint8_t pitch_reset(UART_HandleTypeDef *huart, uint8_t addr){
	uint8_t test_checksum = 0;
	uint8_t resultat_checksum = 0;
	data_tx[0] = SB_cmd(addr, REQUEST_MB_CMD);
	data_tx[1] = MODE_RESET;

	HAL_UART_Transmit_IT(huart, (uint8_t*)data_tx, 2);

	HAL_UART_Receive(huart, (uint8_t*)checksum, sizeof(checksum),RECEIVE_TIMEOUT);
	resultat_checksum = calculate_checksum(data_tx, 2, checksum, sizeof(checksum));
	if(resultat_checksum == checksum[sizeof(checksum)-1]){
		test_checksum = 1;
	}
	else{
		test_checksum = 0;
	}

	return test_checksum;
}

uint8_t pitch_loopback_mode(UART_HandleTypeDef *huart, uint8_t addr){
	data_tx[0] = SB_cmd(addr, REQUEST_MB_CMD);
	data_tx[1] = MODE_LOOPBACK;

	HAL_UART_Transmit_IT(huart, (uint8_t*)data_tx, 2);
	return 1;
}

uint8_t pitch_off_line(UART_HandleTypeDef *huart, uint8_t addr){
	uint8_t test_checksum = 0;
	uint8_t resultat_checksum = 0;
	data_tx[0] = SB_cmd(addr, REQUEST_MB_CMD);
	data_tx[1] = MODE_OFF_LINE;

	HAL_UART_Transmit_IT(huart, (uint8_t*)data_tx, 2);

	HAL_UART_Receive(huart, (uint8_t*)checksum, sizeof(checksum),RECEIVE_TIMEOUT);
	resultat_checksum = calculate_checksum(data_tx, 2, checksum, sizeof(checksum));
	if(resultat_checksum == checksum[sizeof(checksum)-1]){
		test_checksum = 1;
	}
	else{
		test_checksum = 0;
	}

	return test_checksum;
}

uint8_t pitch_change_baud_rate(UART_HandleTypeDef *huart, uint8_t addr, uint8_t baud_rate){
	uint8_t test_checksum = 0;
	uint8_t resultat_checksum = 0;
	data_tx[0] = SB_cmd(addr, REQUEST_MB_CMD);
	data_tx[1] = MODE_BAUD_RATE;
	data_tx[2] = baud_rate;

	HAL_UART_Transmit_IT(huart, (uint8_t*)data_tx, 3);

	HAL_UART_Receive(huart, (uint8_t*)checksum, sizeof(checksum),RECEIVE_TIMEOUT);
	resultat_checksum = calculate_checksum(data_tx, 2, checksum, sizeof(checksum));
	if(resultat_checksum == checksum[sizeof(checksum)-1]){
		test_checksum = 1;
	}
	else{
		test_checksum = 0;
	}

	return test_checksum;
}

void pitch_Init(UART_HandleTypeDef *huart, uint8_t addr){
	uint8_t mode = 0;

	mode &= ~(1 << MODE_REV);
	mode &= ~(1 << MODE_STB);
	mode &= ~(1 << MODE_MULTI);
	mode |=  (1 << MODE_SIZE);
	mode &= ~(1 << MODE_INCR);

	pitch_change_mode(huart, addr, mode);

}

/* Private functions ---------------------------------------------------------*/
