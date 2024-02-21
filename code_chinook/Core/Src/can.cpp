

#include "can.h"

#include "stm32f4xx_hal.h"


// CAN variables
uint8_t txData[8];
uint8_t rxData[8];
CAN_TxHeaderTypeDef pTxHeader;
CAN_RxHeaderTypeDef pRxHeader;
uint32_t txMailbox;


void ProcessCanMessage();


void ProcessCanMessage()
{
	/*
	typedef union BytesToType_
	{
		struct
		{
			uint8_t bytes[4];
		};
		int32_t int_val;
		uint32_t uint_val;
		float float_val;
	} BytesToType;
	static BytesToType bytesToType;
	*/


	// Indicate CAN working with CAN led
	HAL_GPIO_TogglePin(LED_CANA_GPIO_Port, LED_CANA_Pin);

	// Technically CAN data can be 8 bytes but we only send 4-bytes data to the motor driver
	// uint32_t upper_can_data = rxData[4] | (rxData[5] << 8) | (rxData[6] << 16) | (rxData[7] << 24);
	uint32_t can_data = rxData[0] | (rxData[1] << 8) | (rxData[2] << 16) | (rxData[3] << 24);

	if (pRxHeader.StdId == DRIVEMOTOR_PITCH_MODE_FEEDBACK)
	{
		sensor_data.feedback_pitch_mode = (can_data & 0xFF);
	}
	else if (pRxHeader.StdId == DRIVEMOTOR_MAST_MODE_FEEDBACK)
	{
		sensor_data.feedback_mast_mode = (can_data & 0xFF);
	}
	else if (pRxHeader.StdId == DRIVEMOTOR_PITCH_DONE)
	{
		pitch_done = 1;
	}
	else if (pRxHeader.StdId == DRIVEMOTOR_PITCH_FAULT_STALL)
	{
		sensor_data.pitch_fault_stall = (can_data & 0xFF);
	}
	else if (pRxHeader.StdId == DRIVEMOTOR_MAST_FAULT_STALL)
	{
		sensor_data.mast_fault_stall = (can_data & 0xFF);
	}
	else if (pRxHeader.StdId == BACKPLANE_TOTAL_VOLTAGE)
	{

	}
	else if (pRxHeader.StdId == BACKPLANE_TOTAL_CURRENT)
	{

	}
	else if (pRxHeader.StdId == VOLANT_MANUAL_ROPS_CMD)
	{
		uint8_t rops_data = (can_data & 0xFF);
		if (rops_data == ROPS_ENABLE)
			b_rops = 1;
		else if (rops_data == ROPS_DISABLE)
			b_rops = 0;
		else
		{
			// Unknown value received for ROPS command
			// Assume it was meant to activate ROPS
			b_rops = 1;
		}
	}
	else if (pRxHeader.StdId == DRIVEMOTOR_ROPS_FEEDBACK)
	{
		uint8_t rops_data = (can_data & 0xFF);
		sensor_data.feedback_pitch_rops = rops_data;
		/*
		if (rops_data == ROPS_ENABLE)
			sensor_data.feedback_pitch_rops = 1;
		else if (rops_data == ROPS_DISABLE)
			sensor_data.feedback_pitch_rops = 0;
		else
			sensor_data.feedback_pitch_rops = rops_data;
		*/
	}
	else if (pRxHeader.StdId == BACKPLANE_DUMMY_TRAFFIC_MARIO)
	{
		// Dummy traffic
	}
	else
	{
		// Unknown CAN ID
	}
}

void CAN_ReceiveFifoCallback(CAN_HandleTypeDef* hcan, uint32_t fifo)
{
	uint32_t num_messages = HAL_CAN_GetRxFifoFillLevel(hcan, fifo);
	for (int i = 0; i < num_messages; ++i)
	{
		if (HAL_CAN_GetRxMessage(hcan, fifo, &pRxHeader, rxData) != HAL_OK)
		{
			Error_Handler();
		}

		ProcessCanMessage();
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	HAL_GPIO_TogglePin(LED_CANB_GPIO_Port, LED_CANB_Pin);
	CAN_ReceiveFifoCallback(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	HAL_GPIO_TogglePin(LED_CANB_GPIO_Port, LED_CANB_Pin);
	CAN_ReceiveFifoCallback(hcan, CAN_RX_FIFO1);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan)
{
	// HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
	HAL_GPIO_TogglePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
}

HAL_StatusTypeDef TransmitCAN(uint32_t id, uint8_t* buf, uint8_t size, uint8_t with_priority)
{
	// CAN_TxHeaderTypeDef msg;
	pTxHeader.StdId = id;
	pTxHeader.IDE = CAN_ID_STD;
	pTxHeader.RTR = CAN_RTR_DATA;
	pTxHeader.DLC = size; // Number of bytes to send
	pTxHeader.TransmitGlobalTime = DISABLE;

	uint8_t found_mailbox = 0;
	for (int i = 0; i < 25; ++i)
	{
		// Check that mailbox is available for tx
		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
		{
			found_mailbox = 1;
			break;
		}
		// Otherwise wait until free mailbox
		// for (int j = 0; j < 500; ++j) {}
		delay_us(50);
	}
	if (!found_mailbox)
	{
		// HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
	}

	if (with_priority)
	{
		// If message is important, make sure no other messages are queud to ensure it will be sent after any other
		// values that could override it.
		for (int i = 0; i < 10; ++i)
		{
			// Check that all 3 mailboxes are empty
			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 3)
				break;
			// Otherwise wait until 3 free mailbox
			// for (int j = 0; j < 500; ++j) {}
			delay_us(50);
		}
	}

	uint32_t mb;
	HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, buf, &mb);
	if (ret != HAL_OK)
	{
		HAL_GPIO_WritePin(LED_WARNING_GPIO_Port, LED_WARNING_Pin, GPIO_PIN_SET);
		return ret;
	}

	// Successful transmit
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

	// Update the error led if had a successful can write
	HAL_GPIO_WritePin(LED_WARNING_GPIO_Port, LED_WARNING_Pin, GPIO_PIN_RESET);
	// ToggleLed(LED_CAN);
	return ret;
}
